# SDIOv1 host regression tests

These tests compile functions from the actual HAL sources with small host-side
register and DMA models. They need Python 3 and a C11 compiler; no target board
or third-party Python packages are required.

```sh
CC=gcc python -m unittest discover -s testhal/host/sdio -p 'test_*.py' -v
CC=clang python -m unittest discover -s testhal/host/sdio -p 'test_*.py' -v
```

On Windows, set `CC` to `gcc` in a MinGW environment or `cl` in a Visual Studio
developer shell. Each script also accepts `--cc <compiler>` when run directly.
The workflow covers Linux GCC/Clang, macOS Clang and Windows GCC/MSVC.

## Buffer accessibility

An aligned buffer in STM32F4 CCM is not DMA-accessible. The driver copies one
sector at a time through its existing `SDCDriver.buf`; that driver object must
reside in DMA-accessible SRAM. This also works with generic unaligned-buffer
support disabled. Aligned SRAM retains the direct multi-sector path.

The tests use a normal host array to represent the 64 KiB CCM region rather
than mapping a fixed address. They check aligned and unaligned read/write
paths, data contents, partial failures and invalid buffer ranges. No complete
application page is moved to SRAM and no additional sector buffer is allocated.

The tests reproduce software behavior, not electrical SDIO timing or real
interrupt scheduling. Firmware builds and hardware testing remain necessary.

## Completion and recovery

SDIO transfer errors are checked before waiting for DMA, with the kernel
unlocked. This incorporates the completion-ordering repair from Giovanni Di
Sirio's upstream [6c8f039](https://github.com/ChibiOS/ChibiOS/commit/6c8f03904b149b9a136152250420a9032e711794),
with additional error-flag checks and software deadlines. It is not a backport
of the entire upstream commit or its SDMMCv1 changes.

The IRQ wait uses the configured read/write timeout plus a 10 ms margin. DMA
completion/abort has a 10 ms deadline and command completion has a 100 ms
deadline. These are per-operation bounds, not a maximum duration for an entire
filesystem write. Cleanup stops peripheral requests before disabling DMA and
clears pending SDIO interrupts without resetting the shared DMA controller.
If DMA physically refuses to disable, the driver halts instead of returning a
buffer that DMA still owns.

`SDC_WAIT_FOR_TRANSFER_TIMEOUT_MS` optionally bounds a card that keeps reporting
DATA/RCV/PRG. Its default is zero, retaining existing behavior for long card
erases. Applications can opt in after choosing a deadline appropriate to their
cards and operations; the tests exercise a 1000 ms deadline.

Regression cases include card errors with unfinished DMA, missing IRQs,
unfinished DMA after DATAEND, missing command responses, a permanently busy
card, clock wraparound, healthy transfers and cleanup ordering. The model does
not fully emulate hardware interrupt races or a physically stuck DMA enable bit.
