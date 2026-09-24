### Artery ###

This RusEFI fork of ChibiOS have some progress of porting ChibiOS to Artery
AT32 MCU. At least some testing wa done on AT32F435 base development board.

Current AT32 port mostly reuse STM32 hal drivers. Yes, AT32 is very similar
to SMT32 including register organization. But almost all registers and bits
inside registers have its own naming. While function of most of them are
same as on SMT32.

To keep STM32 drivers happy we decided to keep STM32 prefixes and register/
fields naming. Don't be confised.

Current AT32 status:
- os/hal/ports/AT32/AT32F4xx/ is added with basic support
- clock setup, IRQ mapping and startup is taken from similar STM32 chip,
  adjusted and works. Few fixed (up to 288MHz CPU) recomended by DS PLL
  settings defined and tested.
- UART is working, STM32 USARTv1 driver is reused.
- USB OTG is working (testhal/AT32/AT32F4xx/USB_CDC_IAD).
  STM32 OTGv1 is reused.
- ADC + DMA is working. ADCv2 and DMAv1 from STM32 are reused.
  DMAMUX sources are defined for AT32. DMA driver a bit adjusted for AT32
- EFL support is implemented for AT32 chips. See EFL-MFS demo in
  testhal/STM32/multi/EFL-MFS
- more is comming


## Kernel 7.0 / HAL 9.1 migration (2026-09-24)

The AT32 port and its STM32 driver extensions must move together when changing
ChibiOS baselines. This port was restored on top of 14314f2aa68 from the following
commits in stable_21.11.x.rusefi_clean_history:

| Commit | Work |
| --- | --- |
| 794d9ace87 | AT32 device headers, startup, platform, board and examples |
| 172f3c87e1 | AT32 build workflow |
| 3fca2716fd | Separate AT32 DMA mux register blocks and MUXSEL initialization |
| 3a7dae2255 | AT32 PWM timer 32-bit mode |
| 9220946430 | AT32 MFS test configuration |
| a1fa0234ec | SPI v1 selection and system timer inclusion |
| 04a54cb661 | ADC DMA mux integration |
| ef6a69fd35 | SPI DMA mux integration |
| adbb64e0c9 | SPI DMA stream validation with a mux |
| 95cc431f14 | Board-selected MFS configuration and flash driver (adapted) |

Additional migration changes provide the static HAL clock-query fallback,
terminate the EFL configuration field, and provide timer debug-stop macros.
The examples use the current kernel/HAL configuration versions, ARMv7-M make
fragment, test library path, and compiler make fragment. The MFS example selects
EFLD2; existing STM32 examples retain their previous configuration. CI builds
all four AT32 examples and uses GITHUB_PATH to select its compiler.

Validation with arm-none-eabi-gcc 14.2.1:

- M74.9 production firmware compiled and linked with a fresh object/dependency
  directory, producing addressed HEX and S-record images.
- AT32F435 demo, USB CDC, ADC, and MFS test firmware compiled and linked.
- STM32F407 demo, STM32F4 ADC, and STM32L476 MFS regression builds passed.
- No target hardware tests were performed during this migration. The status
  notes above describe earlier port work, not new hardware validation.

To build M74.9 against this checkout without changing its submodule, run from
the board repository (replace the path with your ChibiOS checkout):

```sh
bash compile_firmware.sh -j12 CHIBIOS=/absolute/path/to/ChibiOS \
  BUILDDIR=/tmp/m749-at32-build DEPDIR=/tmp/m749-at32-dep
```

Use new build and dependency directories when comparing ChibiOS revisions.
The firmware repository must reference the resulting ChibiOS revision before
its default submodule build or CI can use this port.
