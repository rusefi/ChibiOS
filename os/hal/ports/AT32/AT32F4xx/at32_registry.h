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
 * @file    STM32F4xx/stm32_registry.h
 * @brief   STM32F4xx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef AT32_REGISTRY_H
#define AT32_REGISTRY_H

#if defined(AT32F437xx) || defined(AT32F435xx)
#define STM32F427_437xx
#define STM32F4XX

#else
#error "Artery AT32F4xx device not specified"
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    STM32F4xx/STM32F2xx capabilities
 * @{
 */

/*===========================================================================*/
/* Common.                                                                   */
/*===========================================================================*/

/* RNG attributes.*/
#define STM32_HAS_RNG1                      TRUE

/* RTC attributes.*/
#define STM32_HAS_RTC                       TRUE
#if !defined(STM32F2XX)
#define STM32_RTC_HAS_SUBSECONDS            TRUE
#else
#define STM32_RTC_HAS_SUBSECONDS            FALSE
#endif
#define STM32_RTC_HAS_PERIODIC_WAKEUPS      TRUE
#define STM32_RTC_NUM_ALARMS                2
#define STM32_RTC_STORAGE_SIZE              80
#define STM32_RTC_TAMP_STAMP_HANDLER        Vector48
#define STM32_RTC_WKUP_HANDLER              Vector4C
#define STM32_RTC_ALARM_HANDLER             VectorE4
#define STM32_RTC_TAMP_STAMP_NUMBER         2
#define STM32_RTC_WKUP_NUMBER               3
#define STM32_RTC_ALARM_NUMBER              41
#define STM32_RTC_ALARM_EXTI                17
#define STM32_RTC_TAMP_STAMP_EXTI           21
#define STM32_RTC_WKUP_EXTI                 22
#define STM32_RTC_IRQ_ENABLE() do {                                         \
  nvicEnableVector(STM32_RTC_TAMP_STAMP_NUMBER, STM32_IRQ_EXTI21_PRIORITY); \
  nvicEnableVector(STM32_RTC_WKUP_NUMBER, STM32_IRQ_EXTI22_PRIORITY);       \
  nvicEnableVector(STM32_RTC_ALARM_NUMBER, STM32_IRQ_EXTI17_PRIORITY);      \
} while (false)

/*===========================================================================*/
/* AT32F435xx, AT32F437xx                                                    */
/*===========================================================================*/

#if defined(STM32F429_439xx) || defined(STM32F427_437xx)

/* Clock tree attributes.*/
/* TODO: Artery! */
#define STM32_HAS_RCC_PLLSAI                FALSE
/* TODO: Artery! */
#define STM32_HAS_RCC_PLLI2S                FALSE
/* TODO: Artery! */
#define STM32_HAS_RCC_DCKCFGR               FALSE
#define STM32_HAS_RCC_DCKCFGR2              FALSE
#define STM32_HAS_RCC_I2SSRC                TRUE
#define STM32_HAS_RCC_I2SPLLSRC             FALSE
#define STM32_HAS_RCC_CK48MSEL              FALSE
#define STM32_RCC_CK48MSEL_USES_I2S         FALSE
#define STM32_TIMPRE_PRESCALE4              TRUE

/* ADC attributes.*/
#define STM32_ADC_HANDLER                   Vector88
#define STM32_ADC_NUMBER                    18

#define STM32_HAS_ADC1                      TRUE

#define STM32_HAS_ADC2                      TRUE

#define STM32_HAS_ADC3                      TRUE

#define STM32_HAS_ADC4                      FALSE

#define STM32_HAS_SDADC1                    FALSE
#define STM32_HAS_SDADC2                    FALSE
#define STM32_HAS_SDADC3                    FALSE

/* CAN attributes.*/
#define STM32_HAS_CAN1                      TRUE
#define STM32_HAS_CAN2                      TRUE
#define STM32_HAS_CAN3                      FALSE
#define STM32_CAN_MAX_FILTERS               28

/* DAC attributes.*/
#define STM32_HAS_DAC1_CH1                  TRUE

#define STM32_HAS_DAC1_CH2                  TRUE

#define STM32_HAS_DAC2_CH1                  FALSE
#define STM32_HAS_DAC2_CH2                  FALSE

/* DMA attributes.*/
#define STM32_ADVANCED_DMA                  TRUE
#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_HAS_DMAMUXSEL             TRUE
//#define STM32_DMA_SUPPORTS_DMAMUX           TRUE
#define STM32_DMA_SUPPORTS_CSELR            FALSE
#define STM32_DMA1_NUM_CHANNELS             7
#define STM32_DMA2_NUM_CHANNELS             7

#define STM32_HAS_DMA1                      TRUE
#define STM32_DMA1_CH1_HANDLER              Vector120
#define STM32_DMA1_CH2_HANDLER              Vector124
#define STM32_DMA1_CH3_HANDLER              Vector128
#define STM32_DMA1_CH4_HANDLER              Vector12C
#define STM32_DMA1_CH5_HANDLER              Vector130
#define STM32_DMA1_CH6_HANDLER              Vector150
#define STM32_DMA1_CH7_HANDLER              Vector154
#define STM32_DMA1_CH1_NUMBER               56
#define STM32_DMA1_CH2_NUMBER               57
#define STM32_DMA1_CH3_NUMBER               58
#define STM32_DMA1_CH4_NUMBER               59
#define STM32_DMA1_CH5_NUMBER               60
#define STM32_DMA1_CH6_NUMBER               68
#define STM32_DMA1_CH7_NUMBER               69

#define STM32_HAS_DMA2                      TRUE
#define STM32_DMA2_CH1_HANDLER              Vector1F0
#define STM32_DMA2_CH2_HANDLER              Vector1F4
#define STM32_DMA2_CH3_HANDLER              Vector1F8
#define STM32_DMA2_CH4_HANDLER              Vector1FC
#define STM32_DMA2_CH5_HANDLER              Vector200
#define STM32_DMA2_CH6_HANDLER              Vector204
#define STM32_DMA2_CH7_HANDLER              Vector208
#define STM32_DMA2_CH1_NUMBER               108
#define STM32_DMA2_CH2_NUMBER               109
#define STM32_DMA2_CH3_NUMBER               110
#define STM32_DMA2_CH4_NUMBER               111
#define STM32_DMA2_CH5_NUMBER               112
#define STM32_DMA2_CH6_NUMBER               113
#define STM32_DMA2_CH7_NUMBER               114

/* ETH attributes.*/
#define STM32_HAS_ETH                       TRUE
#define STM32_ETH_HANDLER                   Vector134
#define STM32_ETH_NUMBER                    61

/* EXTI attributes.*/
#define STM32_EXTI_NUM_LINES                23
#define STM32_EXTI_IMR1_MASK                0x00000000U

/* GPIO attributes.*/
#define STM32_HAS_GPIOA                     TRUE
#define STM32_HAS_GPIOB                     TRUE
#define STM32_HAS_GPIOC                     TRUE
#define STM32_HAS_GPIOD                     TRUE
#define STM32_HAS_GPIOE                     TRUE
#define STM32_HAS_GPIOH                     TRUE
#define STM32_HAS_GPIOF                     TRUE
#define STM32_HAS_GPIOG                     TRUE
/* TODO: Artery AT32F435/7 */
#define STM32_HAS_GPIOI                     FALSE
#define STM32_HAS_GPIOJ                     FALSE
#define STM32_HAS_GPIOK                     FALSE
#define STM32_GPIO_EN_MASK                  (RCC_AHB1ENR_GPIOAEN |          \
                                             RCC_AHB1ENR_GPIOBEN |          \
                                             RCC_AHB1ENR_GPIOCEN |          \
                                             RCC_AHB1ENR_GPIODEN |          \
                                             RCC_AHB1ENR_GPIOEEN |          \
                                             RCC_AHB1ENR_GPIOFEN |          \
                                             RCC_AHB1ENR_GPIOGEN |          \
                                             RCC_AHB1ENR_GPIOHEN)

/* I2C attributes.*/
#define STM32_HAS_I2C1                      TRUE

#define STM32_HAS_I2C2                      TRUE

#define STM32_HAS_I2C3                      TRUE

#define STM32_HAS_I2C4                      FALSE

/* QUADSPI attributes.*/
#define STM32_HAS_QUADSPI1                  FALSE

/* SDIO attributes.*/
#define STM32_HAS_SDIO                      TRUE

/* SPI attributes.*/
#define STM32_HAS_SPI1                      TRUE
#define STM32_SPI1_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI2                      TRUE
#define STM32_SPI2_SUPPORTS_I2S             TRUE
#define STM32_SPI2_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI3                      TRUE
#define STM32_SPI3_SUPPORTS_I2S             TRUE
#define STM32_SPI3_I2S_FULLDUPLEX           TRUE

#define STM32_HAS_SPI4                      TRUE
#define STM32_SPI4_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI5                      TRUE
#define STM32_SPI5_SUPPORTS_I2S             FALSE

#define STM32_HAS_SPI6                      TRUE
#define STM32_SPI6_SUPPORTS_I2S             FALSE

/* TIM attributes.*/
#define STM32_TIM_MAX_CHANNELS              4

#define STM32_HAS_TIM1                      TRUE
#define STM32_TIM1_IS_32BITS                FALSE
#define STM32_TIM1_CHANNELS                 4

#define STM32_HAS_TIM2                      TRUE
#define STM32_TIM2_IS_32BITS                TRUE
#define STM32_TIM2_CHANNELS                 4

#define STM32_HAS_TIM3                      TRUE
#define STM32_TIM3_IS_32BITS                FALSE
#define STM32_TIM3_CHANNELS                 4

#define STM32_HAS_TIM4                      TRUE
#define STM32_TIM4_IS_32BITS                FALSE
#define STM32_TIM4_CHANNELS                 4

#define STM32_HAS_TIM5                      TRUE
#define STM32_TIM5_IS_32BITS                TRUE
#define STM32_TIM5_CHANNELS                 4

#define STM32_HAS_TIM6                      TRUE
#define STM32_TIM6_IS_32BITS                FALSE
#define STM32_TIM6_CHANNELS                 0

#define STM32_HAS_TIM7                      TRUE
#define STM32_TIM7_IS_32BITS                FALSE
#define STM32_TIM7_CHANNELS                 0

#define STM32_HAS_TIM8                      TRUE
#define STM32_TIM8_IS_32BITS                FALSE
#define STM32_TIM8_CHANNELS                 4

#define STM32_HAS_TIM9                      TRUE
#define STM32_TIM9_IS_32BITS                FALSE
#define STM32_TIM9_CHANNELS                 2

#define STM32_HAS_TIM10                     TRUE
#define STM32_TIM10_IS_32BITS               FALSE
#define STM32_TIM10_CHANNELS                1

#define STM32_HAS_TIM11                     TRUE
#define STM32_TIM11_IS_32BITS               FALSE
#define STM32_TIM11_CHANNELS                1

#define STM32_HAS_TIM12                     TRUE
#define STM32_TIM12_IS_32BITS               FALSE
#define STM32_TIM12_CHANNELS                2

#define STM32_HAS_TIM13                     TRUE
#define STM32_TIM13_IS_32BITS               FALSE
#define STM32_TIM13_CHANNELS                1

#define STM32_HAS_TIM14                     TRUE
#define STM32_TIM14_IS_32BITS               FALSE
#define STM32_TIM14_CHANNELS                1

#define STM32_HAS_TIM15                     FALSE
#define STM32_HAS_TIM16                     FALSE
#define STM32_HAS_TIM17                     FALSE
#define STM32_HAS_TIM18                     FALSE
#define STM32_HAS_TIM19                     FALSE
#define STM32_HAS_TIM20                     FALSE
#define STM32_HAS_TIM21                     FALSE
#define STM32_HAS_TIM22                     FALSE

/* USART attributes.*/
#define STM32_HAS_USART1                    TRUE
#define STM32_HAS_USART2                    TRUE
#define STM32_HAS_USART3                    TRUE
#define STM32_HAS_UART4                     TRUE
#define STM32_HAS_UART5                     TRUE
#define STM32_HAS_USART6                    TRUE
#define STM32_HAS_UART7                     TRUE
#define STM32_HAS_UART8                     TRUE
#define STM32_HAS_UART9                     FALSE
#define STM32_HAS_UART10                    FALSE
#define STM32_HAS_LPUART1                   FALSE

/* USB attributes.*/
#define STM32_OTG_STEPPING                  2
#define STM32_HAS_OTG1                      TRUE
#define STM32_OTG1_ENDPOINTS                7
#define STM32_HAS_OTG2                      TRUE
#define STM32_OTG2_ENDPOINTS                7

#define STM32_HAS_USB                       FALSE

/* IWDG attributes.*/
#define STM32_HAS_IWDG                      TRUE
#define STM32_IWDG_IS_WINDOWED              FALSE

/* LTDC attributes.*/
#define STM32_HAS_LTDC                      TRUE

/* DMA2D attributes.*/
#define STM32_HAS_DMA2D                     TRUE

/* FSMC attributes.*/
#define STM32_HAS_FSMC                      TRUE
#define STM32_FSMC_IS_FMC                   TRUE
#define STM32_FSMC_HANDLER                  Vector100
#define STM32_FSMC_NUMBER                   48

/* CRC attributes.*/
#define STM32_HAS_CRC                       TRUE
#define STM32_CRC_PROGRAMMABLE              FALSE

#endif /* defined(STM32F429_439xx) || defined(STM32F427_437xx) */

#endif /* AT32_REGISTRY_H */

/** @} */
