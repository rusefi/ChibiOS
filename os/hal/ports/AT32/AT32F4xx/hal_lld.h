/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    STM32F4xx/hal_lld.h
 * @brief   STM32F4xx/STM32F2xx HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "at32_registry.h"

#include "hal_lld_type1.h"

/* Various helpers.*/
#include "nvic.h"
#include "cache.h"
#include "mpu_v7m.h"
#include "at32_isr.h"
#include "stm32_dma.h"
#include "stm32_exti.h"
#include "at32_rcc.h"
#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Requires use of SPIv2 driver model.
 */
#define HAL_LLD_SELECT_SPI_V2           TRUE

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/**
 * @brief   MCO1 divider clock.
 */
#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) || defined(__DOXYGEN__)
#define STM32_MCO1DIVCLK             STM32_HSICLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_LSE
#define STM32_MCO1DIVCLK             STM32_LSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_HSE
#define STM32_MCO1DIVCLK             STM32_HSECLK

#elif STM32_MCO1SEL == STM32_MCO1SEL_PLL
#define STM32_MCO1DIVCLK             STM32_PLLCLKOUT

#else
#error "invalid STM32_MCO1SEL value specified"
#endif

/**
 * @brief   MCO1 output pin clock.
 */
#if (STM32_MCO1PRE == STM32_MCO1PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO1CLK                STM32_MCO1DIVCLK

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV2
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 2)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV3
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 3)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV4
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 4)

#elif STM32_MCO1PRE == STM32_MCO1PRE_DIV5
#define STM32_MCO1CLK                (STM32_MCO1DIVCLK / 5)

#else
#error "invalid STM32_MCO1PRE value specified"
#endif

/**
 * @brief   MCO2 divider clock.
 */
#if (STM32_MCO2SEL == STM32_MCO2SEL_HSE) || defined(__DOXYGEN__)
#define STM32_MCO2DIVCLK             STM32_HSECLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLL
#define STM32_MCO2DIVCLK             STM32_PLLCLKOUT

#elif STM32_MCO2SEL == STM32_MCO2SEL_SYSCLK
#define STM32_MCO2DIVCLK             STM32_SYSCLK

#elif STM32_MCO2SEL == STM32_MCO2SEL_PLLI2S
#define STM32_MCO2DIVCLK             STM32_PLLI2S

#else
#error "invalid STM32_MCO2SEL value specified"
#endif

/**
 * @brief   MCO2 output pin clock.
 */
#if (STM32_MCO2PRE == STM32_MCO2PRE_DIV1) || defined(__DOXYGEN__)
#define STM32_MCO2CLK                STM32_MCO2DIVCLK

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV2
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 2)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV3
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 3)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV4
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 4)

#elif STM32_MCO2PRE == STM32_MCO2PRE_DIV5
#define STM32_MCO2CLK                (STM32_MCO2DIVCLK / 5)

#else
#error "invalid STM32_MCO2PRE value specified"
#endif

/**
 * @brief   RTC HSE divider setting.
 */
#if ((STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 31)) ||            \
    defined(__DOXYGEN__)
#define STM32_RTCPRE                (STM32_RTCPRE_VALUE << 16)
#else
#error "invalid STM32_RTCPRE value specified"
#endif

/**
 * @brief   HSE divider toward RTC clock.
 */
#if ((STM32_RTCPRE_VALUE >= 2) && (STM32_RTCPRE_VALUE <= 31))  ||           \
    defined(__DOXYGEN__)
#define STM32_HSEDIVCLK             (STM32_HSECLK / STM32_RTCPRE_VALUE)
#else
#error "invalid STM32_RTCPRE value specified"
#endif

/**
 * @brief   RTC clock.
 */
#if (STM32_RTCSEL == STM32_RTCSEL_NOCLOCK) || defined(__DOXYGEN__)
#define STM32_RTCCLK                0

#elif STM32_RTCSEL == STM32_RTCSEL_LSE
#define STM32_RTCCLK                STM32_LSECLK

#elif STM32_RTCSEL == STM32_RTCSEL_LSI
#define STM32_RTCCLK                STM32_LSICLK

#elif STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#define STM32_RTCCLK                STM32_HSEDIVCLK

#else
#error "invalid STM32_RTCSEL value specified"
#endif

/**
 * @brief   48MHz frequency.
 */
#if STM32_CLOCK48_REQUIRED || defined(__DOXYGEN__)
#if (STM32_CK48MSEL == STM32_CK48MSEL_PLL) || defined(__DOXYGEN__)
#if (STM32_PLLCLKOUT == 48000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_1
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 72000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_1_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 96000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_2
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 120000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_2_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 144000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_3
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 168000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_3_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 192000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_4
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 216000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_4_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 240000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 264000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_5_5
    #define STM32_PLL48CLK      48000000
#elif (STM32_PLLCLKOUT == 288000000)
    #define STM32_USBDIV_VALUE  RCC_MISC2_USB_DIV_6
    #define STM32_PLL48CLK      48000000
#else
    #error "no divider to get 48000000 USB clock from PLLCLKOUT"
#endif
#elif STM32_CK48MSEL == STM32_CK48MSEL_HICK
#error "HICK clock as USB 48MHz source is not implemented"
//#define STM32_PLL48CLK              STM32_PLL48CLK_ALTSRC
#else
#error "invalid source selected for PLL48CLK clock"
#endif
#else /* !STM32_CLOCK48_REQUIRED */
#define STM32_PLL48CLK              0
#endif /* STM32_CLOCK48_REQUIRED */

#if !STM32_HAS_RCC_DCKCFGR || (STM32_TIMPRE == STM32_TIMPRE_PCLK) ||        \
    defined(__DOXYGEN__)
/**
 * @brief   Clock of timers connected to APB1
 *          (Timers 2, 3, 4, 5, 6, 7, 12, 13, 14).
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               (STM32_PCLK1 * 1)
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 2)
#endif

/**
 * @brief   Clock of timers connected to APB2 (Timers 1, 8, 9, 10, 11).
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               (STM32_PCLK2 * 1)
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 2)
#endif

#else /* STM32_HAS_RCC_DCKCFGR && (STM32_TIMPRE == STM32_TIMPRE_HCLK) */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) ||                                    \
    (STM32_PPRE1 == STM32_PPRE1_DIV2) ||                                    \
    ((STM32_PPRE1 == STM32_PPRE1_DIV4) &&                                   \
     (STM32_TIMPRE_PRESCALE4 == TRUE)) || defined(__DOXYGEN__)
#define STM32_TIMCLK1               STM32_HCLK
#else
#define STM32_TIMCLK1               (STM32_PCLK1 * 4)
#endif

#if (STM32_PPRE2 == STM32_PPRE2_DIV1) ||                                    \
    (STM32_PPRE2 == STM32_PPRE2_DIV2) ||                                    \
    ((STM32_PPRE2 == STM32_PPRE2_DIV4) &&                                   \
     (STM32_TIMPRE_PRESCALE4 == TRUE)) || defined(__DOXYGEN__)
#define STM32_TIMCLK2               STM32_HCLK
#else
#define STM32_TIMCLK2               (STM32_PCLK2 * 4)
#endif
#endif /* STM32_HAS_RCC_DCKCFGR && (STM32_TIMPRE == STM32_TIMPRE_HCLK) */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* HAL_LLD_H */

/** @} */
