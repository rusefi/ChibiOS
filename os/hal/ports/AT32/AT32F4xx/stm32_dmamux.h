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
 * @file    AT32F4xx/stm32_dmamux.h
 * @brief   AT32F4xx DMAMUX handler header.
 *
 * @addtogroup STM32L4xxp_DMAMUX
 * @{
 */

#ifndef STM32_DMAMUX_H
#define STM32_DMAMUX_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    DMAMUX1 request sources
 * @{
 */
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_ADC1          5
#define STM32_DMAMUX1_DAC1_CH1      6
/* reserved 7 */
#define STM32_DMAMUX1_TIM6_UP       8
#define STM32_DMAMUX1_TIM7_UP       9
#define STM32_DMAMUX1_SPI1_RX       10
#define STM32_DMAMUX1_SPI1_TX       11
#define STM32_DMAMUX1_SPI2_RX       12
#define STM32_DMAMUX1_SPI2_TX       13
#define STM32_DMAMUX1_SPI3_RX       14
#define STM32_DMAMUX1_SPI3_TX       15
#define STM32_DMAMUX1_I2C1_RX       16
#define STM32_DMAMUX1_I2C1_TX       17
#define STM32_DMAMUX1_I2C2_RX       18
#define STM32_DMAMUX1_I2C2_TX       19
#define STM32_DMAMUX1_I2C3_RX       20
#define STM32_DMAMUX1_I2C3_TX       21
/* reserved 22 */
/* reserved 23 */
#define STM32_DMAMUX1_USART1_RX     24
#define STM32_DMAMUX1_USART1_TX     25
#define STM32_DMAMUX1_USART2_RX     26
#define STM32_DMAMUX1_USART2_TX     27
#define STM32_DMAMUX1_USART3_RX     28
#define STM32_DMAMUX1_USART3_TX     29
#define STM32_DMAMUX1_UART4_RX      30
#define STM32_DMAMUX1_UART4_TX      31
#define STM32_DMAMUX1_UART5_RX      32
#define STM32_DMAMUX1_UART5_TX      33
/* reserved 34 */
/* reserved 35 */
#define STM32_DMAMUX1_ADC2          36
#define STM32_DMAMUX1_ADC3          37
/* reserved 38 */
#define STM32_DMAMUX1_SDIO1         39
#define STM32_DMAMUX1_QSPI1         40
#define STM32_DMAMUX1_DAC2          41
#define STM32_DMAMUX1_TIM1_CH1      42
#define STM32_DMAMUX1_TIM1_CH2      43
#define STM32_DMAMUX1_TIM1_CH3      44
#define STM32_DMAMUX1_TIM1_CH4      45
#define STM32_DMAMUX1_TIM1_UP       46
#define STM32_DMAMUX1_TIM1_TRIG     47
#define STM32_DMAMUX1_TIM1_COM      48
#define STM32_DMAMUX1_TIM8_CH1      49
#define STM32_DMAMUX1_TIM8_CH2      50
#define STM32_DMAMUX1_TIM8_CH3      51
#define STM32_DMAMUX1_TIM8_CH4      52
#define STM32_DMAMUX1_TIM8_UP       53
#define STM32_DMAMUX1_TIM8_TRIG     54
#define STM32_DMAMUX1_TIM8_COM      55
#define STM32_DMAMUX1_TIM2_CH1      56
#define STM32_DMAMUX1_TIM2_CH2      57
#define STM32_DMAMUX1_TIM2_CH3      58
#define STM32_DMAMUX1_TIM2_CH4      59
#define STM32_DMAMUX1_TIM2_UP       60
#define STM32_DMAMUX1_TIM3_CH1      61
#define STM32_DMAMUX1_TIM3_CH2      62
#define STM32_DMAMUX1_TIM3_CH3      63
#define STM32_DMAMUX1_TIM3_CH4      64
#define STM32_DMAMUX1_TIM3_UP       65
#define STM32_DMAMUX1_TIM3_TRIG     66
#define STM32_DMAMUX1_TIM4_CH1      67
#define STM32_DMAMUX1_TIM4_CH2      68
#define STM32_DMAMUX1_TIM4_CH3      69
#define STM32_DMAMUX1_TIM4_CH4      70
#define STM32_DMAMUX1_TIM4_UP       71
#define STM32_DMAMUX1_TIM5_CH1      72
#define STM32_DMAMUX1_TIM5_CH2      73
#define STM32_DMAMUX1_TIM5_CH3      74
#define STM32_DMAMUX1_TIM5_CH4      75
#define STM32_DMAMUX1_TIM5_UP       76
/* reserved 77 */
/* reserved 78 */
/* reserved 79 */
/* reserved 80 */
/* reserved 81 */
/* reserved 82 */
/* reserved 83 */
/* reserved 84 */
/* reserved 85 */
#define STM32_DMAMUX1_TIM20_CH1     86
#define STM32_DMAMUX1_TIM20_CH2     87
#define STM32_DMAMUX1_TIM20_CH3     88
#define STM32_DMAMUX1_TIM20_CH4     89
#define STM32_DMAMUX1_TIM20_UP      90
/* reserved 91 */
/* reserved 92 */
#define STM32_DMAMUX1_TIM02_TRIG    93
#define STM32_DMAMUX1_TIM02_COM     94
/* reserved 95 */
/* reserved 96 */
/* reserved 97 */
/* reserved 98 */
/* reserved 99 */
/* reserved 100 */
/* reserved 102 */
#define STM32_DMAMUX1_SDIO2         103
#define STM32_DMAMUX1_QSPI2         104
#define STM32_DMAMUX1_DVP           105
#define STM32_DMAMUX1_SPI4_RX       106
#define STM32_DMAMUX1_SPI4_TX       107
/* reserved 108 */
/* reserved 109 */
#define STM32_DMAMUX1_SAI2_RX       110
#define STM32_DMAMUX1_SAI2_TX       111
#define STM32_DMAMUX1_SAI3_RX       110
#define STM32_DMAMUX1_SAI3_TX       111
#define STM32_DMAMUX1_USART6_RX     112
#define STM32_DMAMUX1_USART6_TX     113
#define STM32_DMAMUX1_UART7_RX      114
#define STM32_DMAMUX1_UART7_TX      115
#define STM32_DMAMUX1_UART8_RX      116
#define STM32_DMAMUX1_UART8_TX      117
/* reserved 118 */
/* reserved 119 */
/* reserved 120 */
/* reserved 121 */
/* reserved 122 */
/* reserved 123 */
/* reserved 124 */
/* reserved 125 */
#define STM32_DMAMUX1_TIM2_TRIG     126
/* reserved 127 */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

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

#ifdef __cplusplus
}
#endif

#endif /* STM32_DMAMUX_H */

/** @} */
