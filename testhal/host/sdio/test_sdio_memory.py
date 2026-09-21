"""Compile the real SDIO block-buffer path with a DMA-inaccessible host region.

No fixed-address mapping: CCM is a normal host array classified by the same
production address predicate. The emulated DMA refuses that region.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
CC = os.environ.get("CC", "cc")


def function(source, name):
    match = re.search(r"(?:static )?bool " + name + r"\(", source)
    if not match:
        return ""
    start = match.start()
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define TRUE 1
#define FALSE 0
#define HAL_SUCCESS false
#define HAL_FAILED true
#define STM32F4XX
#define MMCSD_BLOCK_SIZE 512U
#define SDC_OVERFLOW_ERROR 128U
#define osalDbgAssert(c, m) assert(c)
#define osalDbgCheck(c) assert(c)
static _Alignas(16) uint8_t ccm[65536];
static _Alignas(16) uint8_t ram[2048];
static uint8_t disk[2048];
#define CCMDATARAM_BASE ((uintptr_t)ccm)
typedef struct { uint32_t errors; _Alignas(16) uint8_t buf[512]; } SDCDriver;
static unsigned calls, rejected, failAt, copied;
static bool backend(SDCDriver *driver, uint32_t block, uint8_t *buf,
                    uint32_t count, bool write) {
    calls++;
    if ((uintptr_t)buf >= (uintptr_t)ccm &&
        (uintptr_t)buf < (uintptr_t)ccm + sizeof(ccm)) {
        rejected++;
        return HAL_FAILED;
    }
    if (calls == failAt) return HAL_FAILED;
    assert(block >= 10 && block + count <= 14);
    if (buf == driver->buf) copied++;
    if (write) memcpy(disk + (block - 10) * 512, buf, count * 512);
    else memcpy(buf, disk + (block - 10) * 512, count * 512);
    return HAL_SUCCESS;
}
static bool sdc_lld_read_aligned(SDCDriver *d, uint32_t b, uint8_t *p, uint32_t n) {
    return backend(d, b, p, n, false);
}
static bool sdc_lld_write_aligned(SDCDriver *d, uint32_t b, const uint8_t *p, uint32_t n) {
    return backend(d, b, (uint8_t *)p, n, true);
}
@FUNCTIONS@
int main(int argc, char **argv) {
    assert(argc == 5);
    unsigned region = (unsigned)atoi(argv[1]);
    bool write = atoi(argv[2]) != 0;
    failAt = (unsigned)atoi(argv[3]);
    unsigned blocks = (unsigned)atoi(argv[4]);
    uint8_t *p = (region < 2 ? ram : ccm) + (region & 1);
    for (unsigned i = 0; i < sizeof(disk); i++) disk[i] = (uint8_t)(i * 17 + i / 512);
    memset(p, 0xa5, blocks * 512);
    if (write) memcpy(p, disk, blocks * 512);
    SDCDriver driver = {0};
    bool error = write ? sdc_lld_write(&driver, 10, p, blocks)
                       : sdc_lld_read(&driver, 10, p, blocks);
    if (!error) assert(memcmp(p, disk, blocks * 512) == 0);
    if (error && !write && !rejected) {
        unsigned completed = (calls - 1) * 512;
        assert(memcmp(p, disk, completed) == 0);
        for (unsigned i = completed; i < blocks * 512; i++) assert(p[i] == 0xa5);
    }
    printf("%u %u %u %u\n", (unsigned)error, rejected, calls, copied);
    return 0;
}
'''


class SdioMemoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="chibios-sdio-memory-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.path = Path(cls.tmp.name)
        source = (ROOT / "os/hal/ports/STM32/LLD/SDIOv1/hal_sdc_lld.c").read_text()
        functions = "\n".join(function(source, name) for name in (
            "sdc_lld_buffer_valid", "sdc_lld_needs_bounce", "sdc_lld_read", "sdc_lld_write"))
        # The old driver truncates pointers for alignment only; preserve its
        # behavior without pointer-size warnings in the 64-bit host harness.
        functions = functions.replace("(unsigned)buf", "(uintptr_t)buf")
        (cls.path / "test.c").write_text(HARNESS.replace("@FUNCTIONS@", functions))
        cls.executables = []
        for support in (0, 1):
            exe = cls.path / (f"test{support}" + (".exe" if os.name == "nt" else ""))
            define = f"STM32_SDC_SDIO_UNALIGNED_SUPPORT={support}"
            if Path(CC).name.lower() in ("cl", "cl.exe"):
                command = [CC, "/nologo", "/std:c11", "/W4", "/WX", "/TC", "/D" + define,
                           str(cls.path / "test.c"), "/Fe:" + str(exe), "/Fo:" + str(cls.path / f"test{support}.obj")]
            else:
                command = [CC, "-std=c11", "-Wall", "-Wextra", "-Werror", "-D" + define,
                           str(cls.path / "test.c"), "-o", str(exe)]
            subprocess.run(command, check=True)
            cls.executables.append(exe)

    def run_path(self, region, write, fail=0, blocks=3, support=1):
        result = subprocess.run([str(self.executables[support]), str(region), str(write),
                                 str(fail), str(blocks)], check=True, capture_output=True, text=True)
        return tuple(map(int, result.stdout.split()))

    def test_aligned_ccm_reaches_inaccessible_dma(self):
        for support in (0, 1):
            for write in (0, 1):
                self.assertEqual(self.run_path(2, write, support=support), (1, 1, 1, 0))

    def test_sram_keeps_direct_multi_sector_path(self):
        for write in (0, 1):
            self.assertEqual(self.run_path(0, write), (0, 0, 1, 0))

    def test_unaligned_existing_bounce_preserves_data(self):
        for region in (1, 3):
            for write in (0, 1):
                self.assertEqual(self.run_path(region, write), (0, 0, 3, 3))

    def test_failed_bounce_stops_without_copying_invalid_read(self):
        for fail in (1, 2, 3):
            for write in (0, 1):
                self.assertEqual(self.run_path(3, write, fail), (1, 0, fail, fail - 1))



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=CC)
    args, remaining = parser.parse_known_args()
    CC = args.cc
    unittest.main(argv=[__file__] + remaining)
