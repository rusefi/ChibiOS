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
 * @file    STM32F4xx/hal_lld_type1.h
 * @brief   STM32F4xx/STM32F2xx HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - STM32_LSECLK.
 *          - STM32_LSE_BYPASS (optionally).
 *          - STM32_HSECLK.
 *          - STM32_HSE_BYPASS (optionally).
 *          - STM32_VDD (as hundredths of Volt).
 *          .
 *          One of the following macros must also be defined:
 *          - STM32F2XX for High-performance STM32F2 devices.
 *          - STM32F405xx, STM32F415xx, STM32F407xx, STM32F417xx,
 *            STM32F446xx for High-performance STM32F4 devices of
 *            Foundation line.
 *          - STM32F401xx, STM32F410xx, STM32F411xx, STM32F412xx
 *            for High-performance STM32F4 devices of Access line.
 *          - STM32F427xx, STM32F437xx, STM32F429xx, STM32F439xx, STM32F469xx,
 *            STM32F479xx for High-performance STM32F4 devices of Advanced line.
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_TYPE1_H
#define HAL_LLD_TYPE1_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS              TRUE

/**
 * @name    Platform identification macros
 * @{
 */
#if defined(AT32F435xx) || defined(AT32F437xx)
#define PLATFORM_NAME           "AT32F435/437 High Performance with DSP, FPU and MPU"

#else
#error "AT32FF4xx device not specified"
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
#if defined(AT32F435xx) || defined(AT32F437xx) || defined(__DOXYGEN__)
/**
 * @brief   Absolute maximum system clock.
 */
#define STM32_SYSCLK_MAX        288000000

/**
 * @brief   Maximum HSE clock frequency.
 */
#define STM32_HSECLK_MAX        25000000

/**
 * @brief   Maximum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MAX    25000000

/**
 * @brief   Minimum HSE clock frequency.
 */
#define STM32_HSECLK_MIN        4000000

/**
 * @brief   Minimum HSE clock frequency using an external source.
 */
#define STM32_HSECLK_BYP_MIN    1000000

/**
 * @brief   Maximum LSE clock frequency.
 */
#define STM32_LSECLK_MAX        32768

/**
 * @brief   Maximum LSE clock frequency using an external source.
 */
#define STM32_LSECLK_BYP_MAX    1000000

/**
 * @brief   Minimum LSE clock frequency.
 */
#define STM32_LSECLK_MIN        32768

/**
 * @brief   Maximum PLLs input clock frequency.
 */
#define STM32_PLLIN_MAX         16000000

/**
 * @brief   Minimum PLLs input clock frequency.
 */
#define STM32_PLLIN_MIN         2000000

/**
 * @brief   Maximum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MAX        1200000000

/**
 * @brief   Minimum PLLs VCO clock frequency.
 */
#define STM32_PLLVCO_MIN        500000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define STM32_PLLOUT_MAX        288000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define STM32_PLLOUT_MIN        16000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define STM32_PCLK1_MAX         144000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define STM32_PCLK2_MAX         144000000
#endif
/** @} */

/**
 * @name    Internal clock sources
 * @{
 */
#define STM32_HSICLK            16000000    /**< High speed internal clock. */
#define STM32_LSICLK            32000       /**< Low speed internal clock.  */
/** @} */

/**
 * @name    PWR_CR register bits definitions
 * @{
 */
#define STM32_VOS_SCALE3        0x00004000
#define STM32_VOS_SCALE2        0x00008000
#define STM32_VOS_SCALE1        0x0000C000
#define STM32_PLS_MASK          (7 << 5)    /**< PLS bits mask.             */
#define STM32_PLS_LEV0          (0 << 5)    /**< PVD level 0.               */
#define STM32_PLS_LEV1          (1 << 5)    /**< PVD level 1.               */
#define STM32_PLS_LEV2          (2 << 5)    /**< PVD level 2.               */
#define STM32_PLS_LEV3          (3 << 5)    /**< PVD level 3.               */
#define STM32_PLS_LEV4          (4 << 5)    /**< PVD level 4.               */
#define STM32_PLS_LEV5          (5 << 5)    /**< PVD level 5.               */
#define STM32_PLS_LEV6          (6 << 5)    /**< PVD level 6.               */
#define STM32_PLS_LEV7          (7 << 5)    /**< PVD level 7.               */
/** @} */

/**
 * @name    RCC_PLLCFGR register bits definitions
 * @{
 */
#define STM32_PLLP_MASK         (7 << 16)   /**< PLLP mask.                 */
#define STM32_PLLP_DIV4         (2 << 16)   /**< PLL clock divided by 2.    */
#define STM32_PLLP_DIV8         (3 << 16)   /**< PLL clock divided by 4.    */
#define STM32_PLLP_DIV16        (4 << 16)   /**< PLL clock divided by 6.    */
#define STM32_PLLP_DIV32        (5 << 16)   /**< PLL clock divided by 8.    */

#define STM32_PLLSRC_HSI        (0 << 22)   /**< PLL clock source is HSI.   */
#define STM32_PLLSRC_HSE        (1 << 22)   /**< PLL clock source is HSE.   */
/** @} */

/**
 * @name    RCC_CFGR register bits definitions
 * @{
 */
#define STM32_SW_MASK           (3 << 0)    /**< SW mask.                   */
#define STM32_SW_HSI            (0 << 0)    /**< SYSCLK source is HSI.      */
#define STM32_SW_HSE            (1 << 0)    /**< SYSCLK source is HSE.      */
#define STM32_SW_PLL            (2 << 0)    /**< SYSCLK source is PLL.      */

#define STM32_HPRE_MASK         (15 << 4)   /**< HPRE mask.                 */
#define STM32_HPRE_DIV1         (0 << 4)    /**< SYSCLK divided by 1.       */
#define STM32_HPRE_DIV2         (8 << 4)    /**< SYSCLK divided by 2.       */
#define STM32_HPRE_DIV4         (9 << 4)    /**< SYSCLK divided by 4.       */
#define STM32_HPRE_DIV8         (10 << 4)   /**< SYSCLK divided by 8.       */
#define STM32_HPRE_DIV16        (11 << 4)   /**< SYSCLK divided by 16.      */
#define STM32_HPRE_DIV64        (12 << 4)   /**< SYSCLK divided by 64.      */
#define STM32_HPRE_DIV128       (13 << 4)   /**< SYSCLK divided by 128.     */
#define STM32_HPRE_DIV256       (14 << 4)   /**< SYSCLK divided by 256.     */
#define STM32_HPRE_DIV512       (15 << 4)   /**< SYSCLK divided by 512.     */

#define STM32_PPRE1_MASK        (7 << 10)   /**< PPRE1 mask.                */
#define STM32_PPRE1_DIV1        (0 << 10)   /**< HCLK divided by 1.         */
#define STM32_PPRE1_DIV2        (4 << 10)   /**< HCLK divided by 2.         */
#define STM32_PPRE1_DIV4        (5 << 10)   /**< HCLK divided by 4.         */
#define STM32_PPRE1_DIV8        (6 << 10)   /**< HCLK divided by 8.         */
#define STM32_PPRE1_DIV16       (7 << 10)   /**< HCLK divided by 16.        */

#define STM32_PPRE2_MASK        (7 << 13)   /**< PPRE2 mask.                */
#define STM32_PPRE2_DIV1        (0 << 13)   /**< HCLK divided by 1.         */
#define STM32_PPRE2_DIV2        (4 << 13)   /**< HCLK divided by 2.         */
#define STM32_PPRE2_DIV4        (5 << 13)   /**< HCLK divided by 4.         */
#define STM32_PPRE2_DIV8        (6 << 13)   /**< HCLK divided by 8.         */
#define STM32_PPRE2_DIV16       (7 << 13)   /**< HCLK divided by 16.        */

#define STM32_RTCPRE_MASK       (31 << 16)  /**< RTCPRE mask.               */

#define STM32_MCO1SEL_MASK      (3 << 21)   /**< MCO1 mask.                 */
#define STM32_MCO1SEL_HSI       (0 << 21)   /**< HSI clock on MCO1 pin.     */
#define STM32_MCO1SEL_LSE       (1 << 21)   /**< LSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_HSE       (2 << 21)   /**< HSE clock on MCO1 pin.     */
#define STM32_MCO1SEL_PLL       (3 << 21)   /**< PLL clock on MCO1 pin.     */

#define STM32_MCO1PRE_MASK      (7 << 24)   /**< MCO1PRE mask.              */
#define STM32_MCO1PRE_DIV1      (0 << 24)   /**< MCO1 divided by 1.         */
#define STM32_MCO1PRE_DIV2      (4 << 24)   /**< MCO1 divided by 2.         */
#define STM32_MCO1PRE_DIV3      (5 << 24)   /**< MCO1 divided by 3.         */
#define STM32_MCO1PRE_DIV4      (6 << 24)   /**< MCO1 divided by 4.         */
#define STM32_MCO1PRE_DIV5      (7 << 24)   /**< MCO1 divided by 5.         */

#define STM32_MCO2PRE_MASK      (7 << 27)   /**< MCO2PRE mask.              */
#define STM32_MCO2PRE_DIV1      (0 << 27)   /**< MCO2 divided by 1.         */
#define STM32_MCO2PRE_DIV2      (4 << 27)   /**< MCO2 divided by 2.         */
#define STM32_MCO2PRE_DIV3      (5 << 27)   /**< MCO2 divided by 3.         */
#define STM32_MCO2PRE_DIV4      (6 << 27)   /**< MCO2 divided by 4.         */
#define STM32_MCO2PRE_DIV5      (7 << 27)   /**< MCO2 divided by 5.         */

#define STM32_MCO2SEL_MASK      (3 << 30)   /**< MCO2 mask.                 */
#define STM32_MCO2SEL_SYSCLK    (0 << 30)   /**< SYSCLK clock on MCO2 pin.  */
#define STM32_MCO2SEL_PLLI2S    (1 << 30)   /**< PLLI2S clock on MCO2 pin.  */
#define STM32_MCO2SEL_HSE       (2 << 30)   /**< HSE clock on MCO2 pin.     */
#define STM32_MCO2SEL_PLL       (3 << 30)   /**< PLL clock on MCO2 pin.     */

/**
 * @name    RCC_PLLI2SCFGR register bits definitions
 * @{
 */
#define STM32_PLLI2SM_MASK      (31 << 0)   /**< PLLI2SM mask.              */
#define STM32_PLLI2SN_MASK      (511 << 6)  /**< PLLI2SN mask.              */
#define STM32_PLLI2SP_MASK      (3 << 16)   /**< PLLI2SP mask.              */
#define STM32_PLLI2SP_DIV2      (0 << 16)   /**< PLLI2S clock divided by 2. */
#define STM32_PLLI2SP_DIV4      (1 << 16)   /**< PLLI2S clock divided by 4. */
#define STM32_PLLI2SP_DIV6      (2 << 16)   /**< PLLI2S clock divided by 6. */
#define STM32_PLLI2SP_DIV8      (3 << 16)   /**< PLLI2S clock divided by 8. */
#define STM32_PLLI2SSRC_MASK    (1 << 22)   /**< PLLI2SSRC mask.            */
#define STM32_PLLI2SSRC_PLLSRC  (0 << 22)   /**< PLLI2SSRC is selected PLL
                                                 source.                    */
#define STM32_PLLI2SSRC_CKIN    (1 << 22)   /**< PLLI2SSRC is I2S_CKIN.     */
#define STM32_PLLI2SQ_MASK      (15 << 24)  /**< PLLI2SQ mask.              */
#define STM32_PLLI2SR_MASK      (7 << 28)   /**< PLLI2SR mask.              */
/** @} */

/**
 * @name    RCC_PLLSAICFGR register bits definitions
 * @{
 */
#define STM32_PLLSAIM_MASK      (31 << 0)   /**< PLLSAIM mask.              */
#define STM32_PLLSAIN_MASK      (511 << 6)  /**< PLLSAIN mask.              */
#define STM32_PLLSAIP_MASK      (3 << 16)   /**< PLLSAIP mask.              */
#define STM32_PLLSAIP_DIV2      (0 << 16)   /**< PLLSAI clock divided by 2. */
#define STM32_PLLSAIP_DIV4      (1 << 16)   /**< PLLSAI clock divided by 4. */
#define STM32_PLLSAIP_DIV6      (2 << 16)   /**< PLLSAI clock divided by 6. */
#define STM32_PLLSAIP_DIV8      (3 << 16)   /**< PLLSAI clock divided by 8. */
#define STM32_PLLSAIQ_MASK      (15 << 24)  /**< PLLSAIQ mask.              */
#define STM32_PLLSAIR_MASK      (7 << 28)   /**< PLLSAIR mask.              */
/** @} */

/**
 * @name    RCC_BDCR register bits definitions
 * @{
 */
#define STM32_RTCSEL_MASK       (3 << 8)    /**< RTC source mask.           */
#define STM32_RTCSEL_NOCLOCK    (0 << 8)    /**< No RTC source.             */
#define STM32_RTCSEL_LSE        (1 << 8)    /**< RTC source is LSE.         */
#define STM32_RTCSEL_LSI        (2 << 8)    /**< RTC source is LSI.         */
#define STM32_RTCSEL_HSEDIV     (3 << 8)    /**< RTC source is HSE divided. */
/** @} */

/**
 * @name    RCC_DCKCFGR register bits definitions
 * @{
 */
#define STM32_PLLI2SDIVQ_MASK   (31 << 0)   /**< PLLI2SDIVQ mask.           */
#define STM32_PLLSAIDIVQ_MASK   (31 << 8)   /**< PLLSAIDIVQ mask.           */

#define STM32_PLLSAIDIVR_MASK   (3 << 16)   /**< PLLSAIDIVR mask.           */
#define STM32_PLLSAIDIVR_DIV2   (0 << 16)   /**< LCD_CLK is R divided by 2. */
#define STM32_PLLSAIDIVR_DIV4   (1 << 16)   /**< LCD_CLK is R divided by 4. */
#define STM32_PLLSAIDIVR_DIV8   (2 << 16)   /**< LCD_CLK is R divided by 8. */
#define STM32_PLLSAIDIVR_DIV16  (3 << 16)   /**< LCD_CLK is R divided by 16.*/
#define STM32_PLLSAIDIVR_OFF    0xFFFFFFFFU /**< LCD CLK is not required.   */

#define STM32_TIMPRE_MASK       (1 << 24)   /**< TIMPRE mask.               */
#define STM32_TIMPRE_PCLK       (0 << 24)   /**< TIM clocks from PCLKx.     */
#define STM32_TIMPRE_HCLK       (1 << 24)   /**< TIM clocks from HCLK.      */

#define STM32_I2S1SEL_MASK      (3 << 25)   /**< I2S1SEL mask.              */
#define STM32_I2S1SEL_PLLR      (0 << 25)   /**< I2S1 source is PLLR.       */
#define STM32_I2S1SEL_AFIN      (1 << 25)   /**< I2S1 source is AF Input.   */
#define STM32_I2S1SEL_MCO1      (2 << 25)   /**< I2S1 source is MCO1.       */
#define STM32_I2S1SEL_OFF       0xFFFFFFFFU /**< I2S1 clock is not required.*/

#define STM32_I2S2SEL_MASK      (3 << 27)   /**< I2S2SEL mask.              */
#define STM32_I2S2SEL_PLLR      (0 << 27)   /**< I2S2 source is PLLR.       */
#define STM32_I2S2SEL_AFIN      (1 << 27)   /**< I2S2 source is AF Input.   */
#define STM32_I2S2SEL_MCO1      (2 << 27)   /**< I2S2 source is MCO1.       */
#define STM32_I2S2SEL_OFF       0xFFFFFFFFU /**< I2S2 clock is not required.*/

#define STM32_DSISEL_MASK       (1 << 28)   /**< DSISEL mask.               */
#define STM32_DSISEL_PHY        (0 << 28)   /**< DSI source is DSI-PSY.     */
#define STM32_DSISEL_PLLR       (1 << 28)   /**< DSI source is PLLR.        */
/** @} */

/**
 * @name    RCC_DCKCFGR2 register bits definitions
 * @{
 */
#define STM32_CECSEL_MASK       (1 << 26)   /**< CECSEL mask.               */
#define STM32_CECSEL_LSE        (0 << 26)   /**< CEC source is LSE.         */
#define STM32_CECSEL_HSIDIV488  (1 << 26)   /**< CEC source is HSI/488.     */

#define STM32_SDMMCSEL_MASK     (1 << 28)   /**< SDMMCSEL mask.             */
#define STM32_SDMMCSEL_PLL48CLK (0 << 28)   /**< SDMMC source is PLL48CLK.  */
#define STM32_SDMMCSEL_SYSCLK   (1 << 28)   /**< SDMMC source is SYSCLK.    */

#define STM32_SPDIFSEL_MASK     (1 << 29)   /**< SPDIFSEL mask.             */
#define STM32_SPDIFSEL_PLLI2S   (0 << 29)   /**< SPDIF source is PLLI2S.    */
#define STM32_SPDIFSEL_PLL      (1 << 29)   /**< SPDIF source is PLL.       */

#define STM32_LPTIM1SEL_MASK    (3 << 30)   /**< LPTIM1 mask.               */
#define STM32_LPTIM1SEL_APB     (0 << 30)   /**< LPTIM1 source is APB.      */
#define STM32_LPTIM1SEL_HSI     (1 << 30)   /**< LPTIM1 source is HSI.      */
#define STM32_LPTIM1SEL_LSI     (2 << 30)   /**< LPTIM1 source is LSI.      */
#define STM32_LPTIM1SEL_LSE     (3 << 30)   /**< LPTIM1 source is LSE.      */
/** @} */

/**
 * @name    RCC_MISC1 register bits definitions
 * @{
 */
#define STM32_CK48MSEL_MASK     (1 << 13)   /**< CK48MSEL mask.             */
#define STM32_CK48MSEL_PLL      (0 << 13)   /**< PLL48CLK source is PLL.    */
#define STM32_CK48MSEL_HICK     (1 << 13)   /**< PLL48CLK source is HICK.   */
#define STM32_CK48MSEL_PLLALT   (1 << 13)   /**< Alias.                     */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PWR/RCC initialization in the HAL.
 */
#if !defined(STM32_NO_INIT) || defined(__DOXYGEN__)
#define STM32_NO_INIT               FALSE
#endif

/**
 * @brief   Enables or disables the programmable voltage detector.
 */
#if !defined(STM32_PVD_ENABLE) || defined(__DOXYGEN__)
#define STM32_PVD_ENABLE            FALSE
#endif

/**
 * @brief   Sets voltage level for programmable voltage detector.
 */
#if !defined(STM32_PLS) || defined(__DOXYGEN__)
#define STM32_PLS                   STM32_PLS_LEV0
#endif

/**
 * @brief   Enables the backup RAM regulator.
 */
#if !defined(STM32_BKPRAM_ENABLE) || defined(__DOXYGEN__)
#define STM32_BKPRAM_ENABLE         FALSE
#endif

/**
 * @brief   Enables or disables the HSI clock source.
 */
#if !defined(STM32_HSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSI_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the LSI clock source.
 */
#if !defined(STM32_LSI_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSI_ENABLED           FALSE
#endif

/**
 * @brief   Enables or disables the HSE clock source.
 */
#if !defined(STM32_HSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_HSE_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the LSE clock source.
 */
#if !defined(STM32_LSE_ENABLED) || defined(__DOXYGEN__)
#define STM32_LSE_ENABLED           FALSE
#endif

/**
 * @brief   USB/SDIO clock setting.
 */
#if !defined(STM32_CLOCK48_REQUIRED) || defined(__DOXYGEN__)
#define STM32_CLOCK48_REQUIRED      TRUE
#endif

/**
 * @brief   Main clock source selection.
 * @note    If the selected clock source is not the PLL then the PLL is not
 *          initialized and started.
 * @note    The default value is calculated for a 168MHz system clock from
 *          an external 8MHz HSE clock.
 */
#if !defined(STM32_SW) || defined(__DOXYGEN__)
#define STM32_SW                    STM32_SW_PLL
#endif

/**
 * @brief   Clock source for the PLLs.
 * @note    This setting has only effect if the PLL is selected as the
 *          system clock source.
 * @note    The default value is calculated for a 120MHz system clock from
 *          an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLSRC) || defined(__DOXYGEN__)
#define STM32_PLLSRC                STM32_PLLSRC_HSE
#endif

/**
 * @brief   PLLM divider value.
 * @note    The allowed values are 1..15.
 * @note    The default value is calculated for a 96MHz system clock from
 *          an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLM_VALUE            2
#endif

/**
 * @brief   PLLN multiplier value.
 * @note    The allowed values are 31..500.
 * @note    The default value is calculated for a 120MHz system clock from
 *          an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLN_VALUE            192
#endif

/**
 * @brief   PLLP divider value.
 * @note    The allowed values are 4, 8, 16, 32.
 * @note    The default value is calculated for a 120MHz system clock from
 *          an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLP_VALUE            8
#endif

/**
 * @brief   I2S clock source (post-PLL).
 * @note    Not all devices have this setting, it is alternative to
 *          @p STM32_PLLI2SSRC.
  */
#if !defined(STM32_I2SSRC) || defined(__DOXYGEN__)
#define STM32_I2SSRC                STM32_I2SSRC_CKIN
#endif

/**
 * @brief   I2S clock source (pre-PLL).
 * @note    Not all devices have this setting, it is alternative to
 *          @p STM32_I2SSRC.
 */
#if !defined(STM32_PLLI2SSRC) || defined(__DOXYGEN__)
#define STM32_PLLI2SSRC             STM32_PLLI2SSRC_CKIN
#endif

/**
 * @brief   I2S external clock value, zero if not present.
 * @note    Not all devices have this setting.
 */
#if !defined(STM32_I2SCKIN_VALUE) || defined(__DOXYGEN__)
#define STM32_I2SCKIN_VALUE         0
#endif

/**
 * @brief   PLLI2SN multiplier value.
 * @note    The allowed values are 192..432, except for
 *          STM32F446 where values are 50...432.
 * @note    The default value is calculated for a 96MHz I2S clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLI2SN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SN_VALUE         192
#endif

/**
 * @brief   PLLI2SM divider value.
 * @note    The allowed values are 2..63.
 * @note    The default value is calculated for a 96MHz I2S clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLI2SM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SM_VALUE         4
#endif

/**
 * @brief   PLLI2SR divider value.
 * @note    The allowed values are 2..7.
 * @note    The default value is calculated for a 96MHz I2S clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLI2SR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SR_VALUE         4
#endif

/**
 * @brief   PLLI2SP divider value.
 * @note    The allowed values are 2, 4, 6 and 8.
 */
#if !defined(STM32_PLLI2SP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SP_VALUE         4
#endif

/**
 * @brief   PLLI2SQ divider value.
 * @note    The allowed values are 2..15.
 */
#if !defined(STM32_PLLI2SQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SQ_VALUE         4
#endif

/**
 * @brief   PLLI2SDIVQ divider value (SAI clock divider).
 * @note    The allowed values are 1..32.
 */
#if !defined(STM32_PLLI2SDIVQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLI2SDIVQ_VALUE      1
#endif

/**
 * @brief   PLLSAIM value.
 * @note    The allowed values are 2..63.
 * @note    The default value is calculated for a 96MHz SAI clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLSAIM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIM_VALUE         4
#endif

/**
 * @brief   PLLSAIN value.
 * @note    The allowed values are 50..432.
 * @note    The default value is calculated for a 96MHz SAI clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLSAIN_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIN_VALUE         192
#endif

/**
 * @brief   PLLSAIM value.
 * @note    The allowed values are 2..63.
 * @note    The default value is calculated for a 96MHz SAI clock
 *          output from an external 8MHz HSE clock.
 */
#if !defined(STM32_PLLSAIM_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIM_VALUE         4
#endif

/**
 * @brief   PLLSAIR value.
 * @note    The allowed values are 2..7.
 */
#if !defined(STM32_PLLSAIR_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIR_VALUE         4
#endif

/**
 * @brief   PLLSAIP divider value.
 * @note    The allowed values are 2, 4, 6 and 8.
 */
#if !defined(STM32_PLLSAIP_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIP_VALUE         8
#endif

/**
 * @brief   PLLSAIQ value.
 * @note    The allowed values are 2..15.
 */
#if !defined(STM32_PLLSAIQ_VALUE) || defined(__DOXYGEN__)
#define STM32_PLLSAIQ_VALUE         4
#endif

/**
 * @brief   PLLSAIDIVR divider value (SAI clock divider).
 */
#if !defined(STM32_PLLSAIDIVR) || defined(__DOXYGEN__)
#define STM32_PLLSAIDIVR            STM32_PLLSAIDIVR_OFF
#endif

/**
 * @brief   TIM prescaler clock source.
 */
#if !defined(STM32_TIMPRE) || defined(__DOXYGEN__)
#define STM32_TIMPRE                STM32_TIMPRE_PCLK
#endif

/**
 * @brief   PLL48CLK clock source.
 */
#if !defined(STM32_CK48MSEL) || defined(__DOXYGEN__)
#define STM32_CK48MSEL              STM32_CK48MSEL_PLL
#endif

/**
 * @brief   AHB prescaler value.
 */
#if !defined(STM32_HPRE) || defined(__DOXYGEN__)
#define STM32_HPRE                  STM32_HPRE_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(STM32_PPRE1) || defined(__DOXYGEN__)
#define STM32_PPRE1                 STM32_PPRE1_DIV2
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(STM32_PPRE2) || defined(__DOXYGEN__)
#define STM32_PPRE2                 STM32_PPRE2_DIV2
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(STM32_RTCSEL) || defined(__DOXYGEN__)
#define STM32_RTCSEL                STM32_RTCSEL_LSE
#endif

/**
 * @brief   RTC HSE prescaler value.
 */
#if !defined(STM32_RTCPRE_VALUE) || defined(__DOXYGEN__)
#define STM32_RTCPRE_VALUE          8
#endif

/**
 * @brief   MCO1 clock source value.
 * @note    The default value outputs HSI clock on MCO1 pin.
 */
#if !defined(STM32_MCO1SEL) || defined(__DOXYGEN__)
#define STM32_MCO1SEL               STM32_MCO1SEL_HSI
#endif

/**
 * @brief   MCO1 prescaler value.
 * @note    The default value outputs HSI clock on MCO1 pin.
 */
#if !defined(STM32_MCO1PRE) || defined(__DOXYGEN__)
#define STM32_MCO1PRE               STM32_MCO1PRE_DIV1
#endif

/**
 * @brief   MCO2 clock source value.
 * @note    The default value outputs SYSCLK / 5 on MCO2 pin.
 */
#if !defined(STM32_MCO2SEL) || defined(__DOXYGEN__)
#define STM32_MCO2SEL               STM32_MCO2SEL_SYSCLK
#endif

/**
 * @brief   MCO2 prescaler value.
 * @note    The default value outputs SYSCLK / 5 on MCO2 pin.
 */
#if !defined(STM32_MCO2PRE) || defined(__DOXYGEN__)
#define STM32_MCO2PRE               STM32_MCO2PRE_DIV5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(STM32F4XX) || defined(__DOXYGEN__)
/*
 * Configuration-related checks.
 */
#if !defined(STM32F4xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F4xx_MCUCONF not defined"
#endif

#if defined(STM32F401xx) && !defined(STM32F401_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F401_MCUCONF not defined"
#endif

#if defined(STM32F410xx) && !defined(STM32F410_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F410_MCUCONF not defined"
#endif

#if defined(STM32F411xx) && !defined(STM32F411_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F411_MCUCONF not defined"
#endif

#if defined(STM32F412xx) && !defined(STM32F412_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F412_MCUCONF not defined"
#endif

#if defined(STM32F405xx) && !defined(STM32F405_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F405_MCUCONF not defined"
#endif

#if defined(STM32F415xx) && !defined(STM32F415_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F415_MCUCONF not defined"
#endif

#if defined(STM32F407xx) && !defined(STM32F407_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F407_MCUCONF not defined"
#endif

#if defined(STM32F417xx) && !defined(STM32F417_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F417_MCUCONF not defined"
#endif

#if defined(STM32F427xx) && !defined(STM32F427_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F427_MCUCONF not defined"
#endif

#if defined(STM32F429xx) && !defined(STM32F429_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F429_MCUCONF not defined"
#endif

#if defined(STM32F437xx) && !defined(STM32F437_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F437_MCUCONF not defined"
#endif

#if defined(STM32F439xx) && !defined(STM32F439_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F439_MCUCONF not defined"
#endif

#if defined(STM32F446xx) && !defined(STM32F446_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F446_MCUCONF not defined"
#endif

#if defined(STM32F469xx) && !defined(STM32F469_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F469_MCUCONF not defined"
#endif

#if defined(STM32F479xx) && !defined(STM32F479_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F479_MCUCONF not defined"
#endif

#else /* !defined(STM32F4XX) */
/*
 * Configuration-related checks.
 */
#if !defined(STM32F2xx_MCUCONF)
#error "Using a wrong mcuconf.h file, STM32F2xx_MCUCONF not defined"
#endif
#endif /* !defined(STM32F4XX) */

/**
 * @name    Maximum frequency thresholds, wait states and
 *          parallelism for flash access.
 * @{
 */
#if defined(STM32F429_439xx) || defined(STM32F427_437xx) ||                 \
    defined(STM32F40_41xxx)  || defined(STM32F446xx)     ||                 \
    defined(STM32F469_479xx) || defined(__DOXYGEN__)
#if ((STM32_VDD >= 270) && (STM32_VDD <= 360)) || defined(__DOXYGEN__)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         60000000
#define STM32_2WS_THRESHOLD         90000000
#define STM32_3WS_THRESHOLD         120000000
#define STM32_4WS_THRESHOLD         150000000
#define STM32_5WS_THRESHOLD         180000000
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         96000000
#define STM32_4WS_THRESHOLD         120000000
#define STM32_5WS_THRESHOLD         144000000
#define STM32_6WS_THRESHOLD         168000000
#define STM32_7WS_THRESHOLD         180000000
#define STM32_8WS_THRESHOLD         0
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         22000000
#define STM32_1WS_THRESHOLD         44000000
#define STM32_2WS_THRESHOLD         66000000
#define STM32_3WS_THRESHOLD         88000000
#define STM32_4WS_THRESHOLD         110000000
#define STM32_5WS_THRESHOLD         132000000
#define STM32_6WS_THRESHOLD         154000000
#define STM32_7WS_THRESHOLD         176000000
#define STM32_8WS_THRESHOLD         180000000
#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         20000000
#define STM32_1WS_THRESHOLD         40000000
#define STM32_2WS_THRESHOLD         60000000
#define STM32_3WS_THRESHOLD         80000000
#define STM32_4WS_THRESHOLD         100000000
#define STM32_5WS_THRESHOLD         120000000
#define STM32_6WS_THRESHOLD         140000000
#define STM32_7WS_THRESHOLD         168000000
#define STM32_8WS_THRESHOLD         0
#else
#error "invalid VDD voltage specified"
#endif

#elif defined(STM32F412xx)
#if (STM32_VDD >= 270) && (STM32_VDD <= 360)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         64000000
#define STM32_2WS_THRESHOLD         90000000
#define STM32_3WS_THRESHOLD         100000000
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           2
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         96000000
#define STM32_4WS_THRESHOLD         100000000
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         18000000
#define STM32_1WS_THRESHOLD         36000000
#define STM32_2WS_THRESHOLD         54000000
#define STM32_3WS_THRESHOLD         72000000
#define STM32_4WS_THRESHOLD         90000000
#define STM32_5WS_THRESHOLD         100000000
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 170) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         16000000
#define STM32_1WS_THRESHOLD         32000000
#define STM32_2WS_THRESHOLD         48000000
#define STM32_3WS_THRESHOLD         64000000
#define STM32_4WS_THRESHOLD         80000000
#define STM32_5WS_THRESHOLD         96000000
#define STM32_6WS_THRESHOLD         100000000
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           0
#else
#error "invalid VDD voltage specified"
#endif

#elif defined(STM32F410xx) || defined(STM32F411xx)
#if (STM32_VDD >= 270) && (STM32_VDD <= 360)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         64000000
#define STM32_2WS_THRESHOLD         90000000
#define STM32_3WS_THRESHOLD         100000000
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           2
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         96000000
#define STM32_4WS_THRESHOLD         100000000
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         18000000
#define STM32_1WS_THRESHOLD         36000000
#define STM32_2WS_THRESHOLD         54000000
#define STM32_3WS_THRESHOLD         72000000
#define STM32_4WS_THRESHOLD         90000000
#define STM32_5WS_THRESHOLD         100000000
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 171) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         16000000
#define STM32_1WS_THRESHOLD         32000000
#define STM32_2WS_THRESHOLD         48000000
#define STM32_3WS_THRESHOLD         64000000
#define STM32_4WS_THRESHOLD         80000000
#define STM32_5WS_THRESHOLD         96000000
#define STM32_6WS_THRESHOLD         100000000
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           0

#else
#error "invalid VDD voltage specified"
#endif

#elif defined(STM32F401xx)
#if (STM32_VDD >= 270) && (STM32_VDD <= 360)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         60000000
#define STM32_2WS_THRESHOLD         84000000
#define STM32_3WS_THRESHOLD         0
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           2

#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         84000000
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1

#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         18000000
#define STM32_1WS_THRESHOLD         36000000
#define STM32_2WS_THRESHOLD         54000000
#define STM32_3WS_THRESHOLD         72000000
#define STM32_4WS_THRESHOLD         84000000
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1

#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         16000000
#define STM32_1WS_THRESHOLD         32000000
#define STM32_2WS_THRESHOLD         48000000
#define STM32_3WS_THRESHOLD         64000000
#define STM32_4WS_THRESHOLD         80000000
#define STM32_5WS_THRESHOLD         84000000
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_8WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           0

#else
#error "invalid VDD voltage specified"
#endif

#else /* STM32F2XX */
#if (STM32_VDD >= 270) && (STM32_VDD <= 360)
#define STM32_0WS_THRESHOLD         30000000
#define STM32_1WS_THRESHOLD         60000000
#define STM32_2WS_THRESHOLD         90000000
#define STM32_3WS_THRESHOLD         120000000
#define STM32_4WS_THRESHOLD         0
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           2
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define STM32_0WS_THRESHOLD         24000000
#define STM32_1WS_THRESHOLD         48000000
#define STM32_2WS_THRESHOLD         72000000
#define STM32_3WS_THRESHOLD         96000000
#define STM32_4WS_THRESHOLD         120000000
#define STM32_5WS_THRESHOLD         0
#define STM32_6WS_THRESHOLD         0
#define STM32_7WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define STM32_0WS_THRESHOLD         18000000
#define STM32_1WS_THRESHOLD         36000000
#define STM32_2WS_THRESHOLD         54000000
#define STM32_3WS_THRESHOLD         72000000
#define STM32_4WS_THRESHOLD         90000000
#define STM32_5WS_THRESHOLD         108000000
#define STM32_6WS_THRESHOLD         120000000
#define STM32_7WS_THRESHOLD         0
#define STM32_FLASH_PSIZE           1
#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define STM32_0WS_THRESHOLD         16000000
#define STM32_1WS_THRESHOLD         32000000
#define STM32_2WS_THRESHOLD         48000000
#define STM32_3WS_THRESHOLD         64000000
#define STM32_4WS_THRESHOLD         80000000
#define STM32_5WS_THRESHOLD         96000000
#define STM32_6WS_THRESHOLD         112000000
#define STM32_7WS_THRESHOLD         120000000
#define STM32_FLASH_PSIZE           0

#else
#error "invalid VDD voltage specified"
#endif
#endif /* STM32F2XX */
/** @} */

/*
 * HSI related checks.
 */
#if STM32_HSI_ENABLED
#else /* !STM32_HSI_ENABLED */

#if STM32_SW == STM32_SW_HSI
#error "HSI not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCO1SEL == STM32_MCO1SEL_HSI) ||                                 \
    ((STM32_MCO1SEL == STM32_MCO1SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSI))
#error "HSI not enabled, required by STM32_MCO1SEL"
#endif

#if (STM32_MCO2SEL == STM32_MCO2SEL_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_MCO2SEL"
#endif

#if (STM32_I2SSRC == STM32_I2SSRC_PLLI2S) &&                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_I2SSRC"
#endif

#if (STM32_PLLI2SSRC == STM32_PLLI2SSRC_PLLSRC) &&                          \
    (STM32_PLLSRC == STM32_PLLSRC_HSI)
#error "HSI not enabled, required by STM32_I2SSRC"
#endif
#endif /* !STM32_HSI_ENABLED */

/*
 * HSE related checks.
 */
#if STM32_HSE_ENABLED

#if STM32_HSECLK == 0
#error "HSE frequency not defined"
#else /* STM32_HSECLK != 0 */
#if defined(STM32_HSE_BYPASS)
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_BYP_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_BYP_MAX)"
#endif
#else /* !defined(STM32_HSE_BYPASS) */
#if (STM32_HSECLK < STM32_HSECLK_MIN) || (STM32_HSECLK > STM32_HSECLK_MAX)
#error "STM32_HSECLK outside acceptable range (STM32_HSECLK_MIN...STM32_HSECLK_MAX)"
#endif
#endif /* !defined(STM32_HSE_BYPASS) */
#endif /* STM32_HSECLK != 0 */
#else /* !STM32_HSE_ENABLED */

#if STM32_SW == STM32_SW_HSE
#error "HSE not enabled, required by STM32_SW"
#endif

#if (STM32_SW == STM32_SW_PLL) && (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_SW and STM32_PLLSRC"
#endif

#if (STM32_MCO1SEL == STM32_MCO1SEL_HSE) ||                                 \
    ((STM32_MCO1SEL == STM32_MCO1SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_MCO1SEL"
#endif

#if (STM32_MCO2SEL == STM32_MCO2SEL_HSE) ||                                 \
    ((STM32_MCO2SEL == STM32_MCO2SEL_PLL) &&                                \
     (STM32_PLLSRC == STM32_PLLSRC_HSE))
#error "HSE not enabled, required by STM32_MCO2SEL"
#endif

#if (STM32_I2SSRC == STM32_I2SSRC_PLLI2S) &&                                \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_I2SSRC"
#endif

#if (STM32_PLLI2SSRC == STM32_PLLI2SSRC_PLLSRC) &&                          \
    (STM32_PLLSRC == STM32_PLLSRC_HSE)
#error "HSE not enabled, required by STM32_PLLI2SSRC"
#endif

#if STM32_RTCSEL == STM32_RTCSEL_HSEDIV
#error "HSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_HSE_ENABLED */

/*
 * LSI related checks.
 */
#if STM32_LSI_ENABLED
#else /* !STM32_LSI_ENABLED */

#if HAL_USE_RTC && (STM32_RTCSEL == STM32_RTCSEL_LSI)
#error "LSI not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_LSI_ENABLED */

/*
 * LSE related checks.
 */
#if STM32_LSE_ENABLED

#if (STM32_LSECLK == 0)
#error "LSE frequency not defined"
#endif

#if (STM32_LSECLK < STM32_LSECLK_MIN) || (STM32_LSECLK > STM32_LSECLK_MAX)
#error "STM32_LSECLK outside acceptable range (STM32_LSECLK_MIN...STM32_LSECLK_MAX)"
#endif

#else /* !STM32_LSE_ENABLED */

#if STM32_RTCSEL == STM32_RTCSEL_LSE
#error "LSE not enabled, required by STM32_RTCSEL"
#endif

#endif /* !STM32_LSE_ENABLED */

/**
 * @brief   Clock frequency feeding PLLs.
 */
#if (STM32_PLLSRC == STM32_PLLSRC_HSE) || defined(__DOXYGEN__)
#define STM32_PLLSRCCLK             STM32_HSECLK
#elif STM32_PLLSRC == STM32_PLLSRC_HSI
#define STM32_PLLSRCCLK             STM32_HSICLK
#else
#error "invalid STM32_PLLSRC value specified"
#endif

/**
 * @brief   STM32_PLLM field.
 */
#if ((STM32_PLLM_VALUE >= 1) && (STM32_PLLM_VALUE <= 15)) ||                \
    defined(__DOXYGEN__)
#define STM32_PLLM                  (STM32_PLLM_VALUE << 0)
#else
#error "invalid STM32_PLLM_VALUE value specified"
#endif

/**
 * @brief   PLL input clock frequency.
 */
#define STM32_PLLCLKIN              (STM32_PLLSRCCLK / STM32_PLLM_VALUE)

/*
 * PLLs input frequency range check.
 */
#if (STM32_PLLCLKIN < STM32_PLLIN_MIN) || (STM32_PLLCLKIN > STM32_PLLIN_MAX)
#error "STM32_PLLCLKIN outside acceptable range (STM32_PLLIN_MIN...STM32_PLLIN_MAX)"
#endif

/*
 * PLL enable check.
 */
#if (STM32_CLOCK48_REQUIRED &&                                              \
     STM32_HAS_RCC_CK48MSEL &&                                              \
     (STM32_CK48MSEL == STM32_CK48MSEL_PLL)) ||                             \
    (STM32_SW == STM32_SW_PLL) ||                                           \
    (STM32_MCO1SEL == STM32_MCO1SEL_PLL) ||                                 \
    (STM32_MCO2SEL == STM32_MCO2SEL_PLL) ||                                 \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define STM32_ACTIVATE_PLL          TRUE
#else
#define STM32_ACTIVATE_PLL          FALSE
#endif

/**
 * @brief   STM32_PLLN field.
 */
#if ((STM32_PLLN_VALUE >= 31) && (STM32_PLLN_VALUE <= 500)) ||              \
    defined(__DOXYGEN__)
#define STM32_PLLN                  (STM32_PLLN_VALUE << 6)
#else
#error "invalid STM32_PLLN_VALUE value specified"
#endif

/**
 * @brief   STM32_PLLP field.
 */
#if (STM32_PLLP_VALUE == 4) || defined(__DOXYGEN__)
#define STM32_PLLP                  STM32_PLLP_DIV4
#elif STM32_PLLP_VALUE == 8
#define STM32_PLLP                  STM32_PLLP_DIV8
#elif STM32_PLLP_VALUE == 16
#define STM32_PLLP                  STM32_PLLP_DIV16
#elif STM32_PLLP_VALUE == 32
#define STM32_PLLP                  STM32_PLLP_DIV32
#else
#error "invalid STM32_PLLP_VALUE value specified"
#endif

/**
 * @brief   PLL VCO frequency.
 */
#define STM32_PLLVCO                (STM32_PLLCLKIN * STM32_PLLN_VALUE)

/*
 * PLL VCO frequency range check.
 */
#if (STM32_PLLVCO < STM32_PLLVCO_MIN) || (STM32_PLLVCO > STM32_PLLVCO_MAX)
#error "STM32_PLLVCO outside acceptable range (STM32_PLLVCO_MIN...STM32_PLLVCO_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#define STM32_PLLCLKOUT             (STM32_PLLVCO / STM32_PLLP_VALUE)

/*
 * PLL output frequency range check.
 */
#if (STM32_PLLCLKOUT < STM32_PLLOUT_MIN) || (STM32_PLLCLKOUT > STM32_PLLOUT_MAX)
#error "STM32_PLLCLKOUT outside acceptable range (STM32_PLLOUT_MIN...STM32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if STM32_NO_INIT || defined(__DOXYGEN__)
#define STM32_SYSCLK                STM32_HSICLK
#elif (STM32_SW == STM32_SW_HSI)
#define STM32_SYSCLK                STM32_HSICLK
#elif (STM32_SW == STM32_SW_HSE)
#define STM32_SYSCLK                STM32_HSECLK
#elif (STM32_SW == STM32_SW_PLL)
#define STM32_SYSCLK                STM32_PLLCLKOUT
#else
#error "invalid STM32_SW value specified"
#endif

/* Check on the system clock.*/
#if STM32_SYSCLK > STM32_SYSCLK_MAX
#error "STM32_SYSCLK above maximum rated frequency (STM32_SYSCLK_MAX)"
#endif

/* Calculating VOS settings, it is different for each sub-platform.*/
#if defined(STM32F429_439xx) || defined(STM32F427_437xx) ||                 \
    defined(STM32F446xx)     || defined(STM32F469_479xx) ||                 \
    defined(__DOXYGEN__)
#if STM32_SYSCLK <= 120000000
#define STM32_VOS                   STM32_VOS_SCALE3
#define STM32_OVERDRIVE_REQUIRED    FALSE
#elif STM32_SYSCLK <= 144000000
#define STM32_VOS                   STM32_VOS_SCALE2
#define STM32_OVERDRIVE_REQUIRED    FALSE
#elif STM32_SYSCLK <= 168000000
#define STM32_VOS                   STM32_VOS_SCALE1
#define STM32_OVERDRIVE_REQUIRED    FALSE
#else
#define STM32_VOS                   STM32_VOS_SCALE1
#define STM32_OVERDRIVE_REQUIRED    TRUE
#endif

#elif defined(STM32F40_41xxx)
#if STM32_SYSCLK <= 144000000
#define STM32_VOS                   STM32_VOS_SCALE2
#else
#define STM32_VOS                   STM32_VOS_SCALE1
#endif
#define STM32_OVERDRIVE_REQUIRED    FALSE

#elif defined(STM32F401xx)
#if STM32_SYSCLK <= 60000000
#define STM32_VOS                   STM32_VOS_SCALE3
#else
#define STM32_VOS                   STM32_VOS_SCALE2
#endif
#define STM32_OVERDRIVE_REQUIRED    FALSE

#elif defined(STM32F410xx) || defined(STM32F411xx) ||                       \
      defined(STM32F412xx)
#if STM32_SYSCLK <= 64000000
#define STM32_VOS                   STM32_VOS_SCALE3
#elif STM32_SYSCLK <= 84000000
#define STM32_VOS                   STM32_VOS_SCALE2
#else
#define STM32_VOS                   STM32_VOS_SCALE1
#endif
#define STM32_OVERDRIVE_REQUIRED    FALSE

#else /* STM32F2XX */
#define STM32_OVERDRIVE_REQUIRED    FALSE
#endif

/**
 * @brief   AHB frequency.
 */
#if (STM32_HPRE == STM32_HPRE_DIV1) || defined(__DOXYGEN__)
#define STM32_HCLK                  (STM32_SYSCLK / 1)
#elif STM32_HPRE == STM32_HPRE_DIV2
#define STM32_HCLK                  (STM32_SYSCLK / 2)
#elif STM32_HPRE == STM32_HPRE_DIV4
#define STM32_HCLK                  (STM32_SYSCLK / 4)
#elif STM32_HPRE == STM32_HPRE_DIV8
#define STM32_HCLK                  (STM32_SYSCLK / 8)
#elif STM32_HPRE == STM32_HPRE_DIV16
#define STM32_HCLK                  (STM32_SYSCLK / 16)
#elif STM32_HPRE == STM32_HPRE_DIV64
#define STM32_HCLK                  (STM32_SYSCLK / 64)
#elif STM32_HPRE == STM32_HPRE_DIV128
#define STM32_HCLK                  (STM32_SYSCLK / 128)
#elif STM32_HPRE == STM32_HPRE_DIV256
#define STM32_HCLK                  (STM32_SYSCLK / 256)
#elif STM32_HPRE == STM32_HPRE_DIV512
#define STM32_HCLK                  (STM32_SYSCLK / 512)
#else
#error "invalid STM32_HPRE value specified"
#endif

/*
 * AHB frequency check.
 */
#if STM32_HCLK > STM32_SYSCLK_MAX
#error "STM32_HCLK exceeding maximum frequency (STM32_SYSCLK_MAX)"
#endif

/**
 * @brief   APB1 frequency.
 */
#if (STM32_PPRE1 == STM32_PPRE1_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK1                 (STM32_HCLK / 1)
#elif STM32_PPRE1 == STM32_PPRE1_DIV2
#define STM32_PCLK1                 (STM32_HCLK / 2)
#elif STM32_PPRE1 == STM32_PPRE1_DIV4
#define STM32_PCLK1                 (STM32_HCLK / 4)
#elif STM32_PPRE1 == STM32_PPRE1_DIV8
#define STM32_PCLK1                 (STM32_HCLK / 8)
#elif STM32_PPRE1 == STM32_PPRE1_DIV16
#define STM32_PCLK1                 (STM32_HCLK / 16)
#else
#error "invalid STM32_PPRE1 value specified"
#endif

/*
 * APB1 frequency check.
 */
#if STM32_PCLK1 > STM32_PCLK1_MAX
#error "STM32_PCLK1 exceeding maximum frequency (STM32_PCLK1_MAX)"
#endif

/**
 * @brief   APB2 frequency.
 */
#if (STM32_PPRE2 == STM32_PPRE2_DIV1) || defined(__DOXYGEN__)
#define STM32_PCLK2                 (STM32_HCLK / 1)
#elif STM32_PPRE2 == STM32_PPRE2_DIV2
#define STM32_PCLK2                 (STM32_HCLK / 2)
#elif STM32_PPRE2 == STM32_PPRE2_DIV4
#define STM32_PCLK2                 (STM32_HCLK / 4)
#elif STM32_PPRE2 == STM32_PPRE2_DIV8
#define STM32_PCLK2                 (STM32_HCLK / 8)
#elif STM32_PPRE2 == STM32_PPRE2_DIV16
#define STM32_PCLK2                 (STM32_HCLK / 16)
#else
#error "invalid STM32_PPRE2 value specified"
#endif

/*
 * APB2 frequency check.
 */
#if STM32_PCLK2 > STM32_PCLK2_MAX
#error "STM32_PCLK2 exceeding maximum frequency (STM32_PCLK2_MAX)"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void stm32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_TYPE1_H */

/** @} */
