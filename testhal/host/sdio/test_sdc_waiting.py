"""Real SDC busy-state and SDIO command functions with emulated time/status."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
CC = os.environ.get("CC", "cc")


def extract(source, name):
    match = re.search(r"(?:static )?(?:bool|void) " + name + r"\(", source)
    if not match:
        return ""
    opening = source.index("{", match.start())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


HARNESS = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#define TRUE 1
#define HAL_FAILED true
#define HAL_SUCCESS false
#define SDC_NICE_WAITING TRUE
#define SDC_WAIT_FOR_TRANSFER_TIMEOUT_MS 1000
#define STM32_SDC_COMMAND_TIMEOUT_MS 100
#define SDC_DATA_TIMEOUT 4U
#define SDC_COMMAND_TIMEOUT 8U
#define MMCSD_CMD_SEND_STATUS 13U
#define MMCSD_STS_TRAN 4U
#define MMCSD_STS_DATA 5U
#define MMCSD_STS_RCV 6U
#define MMCSD_STS_PRG 7U
#define MMCSD_R1_STS(x) ((x) & 15U)
#define MMCSD_R1_ERROR(x) ((x) & 0x80000000U)
#define SDIO_STA_CMDSENT 1U
#define SDIO_ICR_CMDSENTC 1U
#define SDIO_STA_CMDREND 2U
#define SDIO_STA_CTIMEOUT 4U
#define SDIO_STA_CCRCFAIL 8U
#define SDIO_STA_ERROR_MASK 12U
#define SDIO_CMD_CPSMEN 1U
#define SDIO_CMD_WAITRESP_0 2U
#define SDIO_CMD_WAITRESP_1 4U
#define OSAL_MS2I(x) (x)
typedef uint32_t systime_t;
static systime_t now = UINT32_MAX-10;
static inline systime_t osalOsGetSystemTimeX(void) { return now++; }
static inline bool osalTimeIsInRangeX(systime_t t, systime_t a, systime_t b) { return t-a < b-a; }
static inline void osalThreadSleepMilliseconds(unsigned ms) { now += ms; }
typedef struct { uint32_t STA, ICR, CMD, ARG, RESP1, RESP2, RESP3, RESP4; } Sdio;
typedef struct { Sdio *sdio; uint32_t errors, rca; } SDCDriver;
static void sdc_lld_collect_errors(SDCDriver *d, uint32_t flags) { d->errors |= flags; }
@FUNCTIONS@
int main(int argc, char **argv) {
    if (argc != 3) return 2;
    int kind = atoi(argv[1]), healthy = atoi(argv[2]);
    Sdio io = {0};
    SDCDriver d = {&io, 0, 1};
    uint32_t resp[4] = {0};
    if (healthy || kind == 4) io.STA = SDIO_STA_CMDSENT | SDIO_STA_CMDREND;
    io.RESP1 = healthy ? MMCSD_STS_TRAN : MMCSD_STS_PRG;
    bool error = false;
    switch (kind) {
    case 0: sdc_lld_send_cmd_none(&d, 0, 0); error = d.errors != 0; break;
    case 1: error = sdc_lld_send_cmd_short(&d, 0, 0, resp); break;
    case 2: error = sdc_lld_send_cmd_short_crc(&d, 0, 0, resp); break;
    case 3: error = sdc_lld_send_cmd_long_crc(&d, 0, 0, resp); break;
    case 4: error = _sdc_wait_for_transfer_state_internal(&d, true); break;
    }
    printf("%u %u\n", (unsigned)error, d.errors);
    return 0;
}
'''


class SdcWaitingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="chibios-sdc-wait-")
        cls.addClassCleanup(cls.tmp.cleanup)
        path = Path(cls.tmp.name)
        low = (ROOT / "os/hal/ports/STM32/LLD/SDIOv1/hal_sdc_lld.c").read_text()
        high = (ROOT / "os/hal/src/hal_sdc.c").read_text()
        functions = "\n".join(extract(low, name) for name in (
            "sdc_lld_wait_command", "sdc_lld_send_cmd_none", "sdc_lld_send_cmd_short",
            "sdc_lld_send_cmd_short_crc", "sdc_lld_send_cmd_long_crc"))
        functions += "\n" + extract(high, "_sdc_wait_for_transfer_state_internal")
        source = path / "test.c"
        source.write_text(HARNESS.replace("@FUNCTIONS@", functions))
        cls.exe = path / ("test.exe" if os.name == "nt" else "test")
        if Path(CC).name.lower() in ("cl", "cl.exe"):
            command = [CC, "/nologo", "/std:c11", "/W4", "/WX", "/TC", str(source),
                       "/Fe:" + str(cls.exe), "/Fo:" + str(path / "test.obj")]
        else:
            command = [CC, "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(cls.exe)]
        subprocess.run(command, check=True)

    def run_path(self, kind, healthy):
        return subprocess.run([str(self.exe), str(kind), str(healthy)], check=True,
                              capture_output=True, text=True, timeout=1)

    def test_missing_command_completion_returns_timeout(self):
        for kind in range(4):
            with self.subTest(kind=kind):
                self.assertEqual(self.run_path(kind, 0).stdout.strip(), "1 8")

    def test_forever_busy_card_returns_timeout_across_clock_wrap(self):
        self.assertEqual(self.run_path(4, 0).stdout.strip(), "1 4")

    def test_healthy_commands_and_card(self):
        for kind in range(5):
            self.assertEqual(self.run_path(kind, 1).stdout.strip(), "0 0")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=CC)
    args, remaining = parser.parse_known_args()
    CC = args.cc
    unittest.main(argv=[__file__] + remaining)
