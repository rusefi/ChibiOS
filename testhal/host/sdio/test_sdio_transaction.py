"""Exercise the production SDIOv1 completion path with emulated registers.

No STM32 is needed: compile the actual driver function and DMA wait macro,
then leave DMA enabled after a simulated card error. A subprocess deadline
detects the otherwise infinite wait without hanging the test runner.
Run with Python 3 and a native C compiler, e.g. --cc gcc or --cc clang.
"""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
LLD = ROOT / "os/hal/ports/STM32/LLD"
CC = os.environ.get("CC", "cc")


def function(source, name):
    start = source.index("static bool " + name + "(")
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def macro(source, name):
    lines = source[source.index("#define " + name + "("):].splitlines()
    end = 0
    while lines[end].endswith("\\"):
        end += 1
    return "\n".join(lines[:end + 1])


HARNESS = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define HAL_SUCCESS false
#define HAL_FAILED true
#define SDIO_STA_DATAEND (1U << 8)
#define SDIO_STA_DCRCFAIL (1U << 1)
#define SDIO_STA_DTIMEOUT (1U << 3)
#define SDIO_STA_TXUNDERR (1U << 4)
#define SDIO_STA_RXOVERR (1U << 5)
#define SDIO_STA_STBITERR (1U << 9)
#define SDIO_ICR_ALL_FLAGS 0xFFFFFFFFU
#define STM32_DMA_CR_EN 1U
#define STM32_DMA_CR_TCIE 2U
#define STM32_DMA_CR_HTIE 4U
#define STM32_DMA_CR_TEIE 8U
#define STM32_DMA_CR_DMEIE 16U
#define MMCSD_CMD_STOP_TRANSMISSION 12U
#define SDIO_DCTRL_DTDIR 2U
#define SDC_DATA_TIMEOUT 4U
#define STM32_SDC_READ_TIMEOUT_MS 25U
#define STM32_SDC_WRITE_TIMEOUT_MS 250U
#define STM32_SDC_DMA_TIMEOUT_MS 10U
#define STM32_SDC_IRQ_MARGIN_MS 10U
#define MSG_OK 0
#define MSG_TIMEOUT -1
#define OSAL_MS2I(x) (x)
#define STM32_SDIO_NUMBER 49
typedef uint32_t systime_t;
typedef int msg_t;
static systime_t now = UINT32_MAX - 5;
static inline systime_t osalOsGetSystemTimeX(void) { return now++; }
static inline bool osalTimeIsInRangeX(systime_t t, systime_t a, systime_t b) { return t-a < b-a; }
static inline void osalThreadSleepMilliseconds(unsigned ms) { now += ms; }
static inline void osalSysHalt(const char *message) { (void)message; abort(); }
static inline void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
typedef struct { volatile uint32_t CR; } DmaRegisters;
typedef struct { DmaRegisters *stream; } Dma;
typedef struct { volatile uint32_t MASK, DCTRL, STA, ICR; } Sdio;
typedef struct { Sdio *sdio; Dma *dma; void *thread; uint32_t errors; } SDCDriver;
static int locked, suspended, dmaCleared, stopped;
static int unsafeStop;
static Sdio *activeSdio;
static bool stopFailed;
static void osalSysLock(void) { locked = 1; puts("locked"); fflush(stdout); }
static void osalSysUnlock(void) { locked = 0; puts("unlocked"); fflush(stdout); }
static int irqMode;
static inline void osalThreadSuspendS(void **thread) {
    (void)thread; suspended++;
    if (irqMode == 2) for (;;) {}
}
static inline msg_t osalThreadSuspendTimeoutS(void **thread, unsigned timeout) {
    (void)thread; suspended++; now += timeout;
    return irqMode == 2 ? MSG_TIMEOUT : MSG_OK;
}
static inline void dmaStreamClearInterrupt(Dma *dma) {
    (void)dma;
    dmaCleared++;
}
static bool sdc_lld_send_cmd_short_crc(SDCDriver *sdcp, uint32_t cmd,
                                      uint32_t arg, uint32_t *resp) {
    (void)sdcp; (void)cmd; (void)arg; (void)resp;
    stopped++;
    return stopFailed;
}
static void sdc_lld_collect_errors(SDCDriver *d, uint32_t flags) { d->errors |= flags; }
#define dmaStreamDisable(d) do { unsafeStop = activeSdio->DCTRL != 0; (d)->stream->CR = 0; } while (0)
@DMA@
@FUNCTION@
@CLEANUP@
int main(int argc, char **argv) {
    if (argc != 6) return 2;
    Sdio sdio = { (uint32_t)strtoul(argv[3], NULL, 0), 99,
                  (uint32_t)strtoul(argv[1], NULL, 0), 0 };
    DmaRegisters regs = { (uint32_t)strtoul(argv[2], NULL, 0) };
    Dma dma = { &regs };
    SDCDriver driver = { &sdio, &dma, NULL, 0 };
    irqMode = atoi(argv[3]);
    uint32_t response = 0;
    stopFailed = atoi(argv[5]);
    if (atoi(argv[5]) == 2) {
        activeSdio = &sdio;
        sdc_lld_error_cleanup(&driver, atoi(argv[4]), &response);
        printf("cleanup %d %d %u %u\n", unsafeStop, locked, sdio.DCTRL, regs.CR);
        return 0;
    }
    bool failed = sdc_lld_wait_transaction_end(&driver, atoi(argv[4]), &response);
    printf("result %d %d %d %d %d %u %u %u\n", failed, locked,
           suspended, dmaCleared, stopped, sdio.MASK, sdio.DCTRL, sdio.ICR);
    return 0;
}
'''


class SdioTransactionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="chibios-sdio-")
        cls.addClassCleanup(cls.directory.cleanup)
        path = Path(cls.directory.name)
        source = HARNESS.replace("@DMA@", macro(
            (LLD / "DMAv2/stm32_dma.h").read_text(), "dmaWaitCompletion"))
        source = source.replace("@FUNCTION@", function(
            (LLD / "SDIOv1/hal_sdc_lld.c").read_text(), "sdc_lld_wait_transaction_end"))
        source = source.replace("@CLEANUP@", function(
            (LLD / "SDIOv1/hal_sdc_lld.c").read_text().replace("static void sdc_lld_error_cleanup", "static bool sdc_lld_error_cleanup"),
            "sdc_lld_error_cleanup").replace("static bool sdc_lld_error_cleanup", "static void sdc_lld_error_cleanup"))
        (path / "test.c").write_text(source)
        cls.executable = path / ("test.exe" if os.name == "nt" else "test")
        if Path(CC).name.lower() in ("cl", "cl.exe"):
            command = [CC, "/nologo", "/std:c11", "/W4", "/WX", "/TC",
                       str(path / "test.c"), "/Fe:" + str(cls.executable),
                       "/Fo:" + str(path / "test.obj")]
        else:
            command = [CC, "-std=c11", "-Wall", "-Wextra", "-Werror",
                       str(path / "test.c"), "-o", str(cls.executable)]
        subprocess.run(command, check=True)

    def run_driver(self, flags, dma=0, pending_irq=0, blocks=1, stop_failed=0):
        return subprocess.run([str(self.executable), str(flags), str(dma),
                               str(pending_irq), str(blocks), str(stop_failed)],
                              capture_output=True, text=True, timeout=1, check=True)

    def test_card_errors_wait_forever_for_unfinished_dma(self):
        for flag in (1 << 1, 1 << 3, 1 << 4, 1 << 5, 1 << 9):
            for pending_irq in (0, 1):
                with self.subTest(flag=flag, pending_irq=pending_irq):
                    with self.assertRaises(subprocess.TimeoutExpired):
                        self.run_driver(flag, dma=1, pending_irq=pending_irq)

    def test_dataend_with_error_and_dma_stuck_waits_forever(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            self.run_driver((1 << 8) | (1 << 1), dma=1, blocks=4)

    def test_missing_dataend_with_dma_stuck_waits_forever(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            self.run_driver(0, dma=1)

    def test_success_single_block(self):
        self.assertIn("result 0 0 0 1 0 0 0 4294967295", self.run_driver(1 << 8).stdout)

    def test_success_multi_block_after_irq_wait(self):
        self.assertIn("result 0 0 1 1 1 0 0 4294967295", self.run_driver(1 << 8, pending_irq=1, blocks=4).stdout)

    def test_stop_command_failure_is_reported(self):
        self.assertIn("result 1 0 0 1 1 0 0 4294967295", self.run_driver(1 << 8, blocks=4, stop_failed=1).stdout)

    def test_missing_irq_waits_forever(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            self.run_driver(0, dma=1, pending_irq=2)

    def test_dataend_with_dma_stuck_waits_forever(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            self.run_driver(1 << 8, dma=1)

    def test_cleanup_stops_dma_before_peripheral(self):
        self.assertIn("cleanup 1 0 0 0", self.run_driver(1 << 3, dma=1, stop_failed=2).stdout)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    args, remaining = parser.parse_known_args()
    CC = args.cc
    unittest.main(argv=[__file__] + remaining)
