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
