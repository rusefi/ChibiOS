/*
    ChibiOS - Copyright (C) 2006..2021 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    RP2040/hal_lld.h
 * @brief   RP2040 HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

/*
 * Registry definitions.
 */
#include "rp_registry.h"

/* From Pico-SDK */
#include "hardware/clocks.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(RP2040) || defined(__DOXYGEN__)
#define PLATFORM_NAME           "RP2040"

#else
#error "RP2040 device not specified"
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define RP_ROSCCLK              6500000     /**< 6.5MHz internal clock.     */
/** @} */


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the clocks initialization in the HAL.
 */
#if !defined(RP_NO_INIT) || defined(__DOXYGEN__)
#define RP_NO_INIT                          FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(RP2040_MCUCONF)
#error "Using a wrong mcuconf.h file, RP2040_MCUCONF not defined"
#endif

/*
 * Board files sanity checks.
 */
#if !defined(RP_XOSCCLK)
#error "RP_XOSCCLK not defined in board.h"
#endif

/**
 * @name    Various clock points.
 * @{
 */
#define RP_GPOUT0_CLK           hal_lld_get_clock(clk_gpout0)
#define RP_GPOUT1_CLK           hal_lld_get_clock(clk_gpout1)
#define RP_GPOUT2_CLK           hal_lld_get_clock(clk_gpout2)
#define RP_GPOUT3_CLK           hal_lld_get_clock(clk_gpout3)
#define RP_REF_CLK              hal_lld_get_clock(clk_ref)
#define RP_CORE_CLK             hal_lld_get_clock(clk_sys)
#define RP_PERI_CLK             hal_lld_get_clock(clk_peri)
#define RP_USB_CLK              hal_lld_get_clock(clk_usb)
#define RP_ADC_CLK              hal_lld_get_clock(clk_adc)
#define RP_RTC_CLK              hal_lld_get_clock(clk_rtc)
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef enum clock_index clock_index_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "rp_isr.h"
#include "rp_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

__STATIC_INLINE uint32_t hal_lld_get_clock(clock_index_t clk_index) {

  return clock_get_hz(clk_index);
}

__STATIC_INLINE void hal_lld_peripheral_reset(uint32_t mask) {

  RESETS->RESET |=  mask;
}

__STATIC_INLINE void hal_lld_peripheral_unreset(uint32_t mask) {

  RESETS->RESET &= ~mask;
  while ((RESETS->RESET_DONE & mask) == 0U) {
    /* Waiting for reset.*/
  }
}

#endif /* HAL_LLD_H */

/** @} */
