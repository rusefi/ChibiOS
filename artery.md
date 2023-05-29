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