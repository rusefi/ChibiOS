/*
    ChibiOS - Copyright (C) 2023 Andrey Gusakov

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    lis2dw12.h
 * @brief   LIS2DW12 MEMS interface module header.
 *
 * @addtogroup LIS2DW12
 * @ingroup EX_ST
 * @{
 */

#ifndef _LIS2DW12_H_
#define _LIS2DW12_H_

#include "ex_accelerometer.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Version identification
 * @{
 */
/**
 * @brief   LIS2DW12 driver version string.
 */
#define EX_LIS2DW12_VERSION                 "1.1.1"

/**
 * @brief   LIS2DW12 driver version major number.
 */
#define EX_LIS2DW12_MAJOR                   1

/**
 * @brief   LIS2DW12 driver version minor number.
 */
#define EX_LIS2DW12_MINOR                   1

/**
 * @brief   LIS2DW12 driver version patch number.
 */
#define EX_LIS2DW12_PATCH                   1
/** @} */

/**
 * @brief   LIS2DW12 accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LIS2DW12_ACC_NUMBER_OF_AXES         3U

#define LIS2DW12_ACC_2G                     2.0f
#define LIS2DW12_ACC_4G                     4.0f
#define LIS2DW12_ACC_8G                     8.0f
#define LIS2DW12_ACC_16G                    16.0f

#define LIS2DW12_ACC_SENS_2G                (0.244f * 0.00980665f)
#define LIS2DW12_ACC_SENS_4G                (0.488f * 0.00980665f)
#define LIS2DW12_ACC_SENS_8G                (0.976f * 0.00980665f)
#define LIS2DW12_ACC_SENS_16G               (1.952f * 0.00980665f)

#define LIS2DW12_ACC_BIAS                   0.0f
/** @} */

/**
 * @name    LIS2DW12 communication interfaces related bit masks
 * @{
 */
#define LIS2DW12_DI_MASK                    0xFF
#define LIS2DW12_DI(n)                      (1 << n)
#define LIS2DW12_AD_MASK                    0x7F
#define LIS2DW12_AD(n)                      (1 << n)
#define LIS2DW12_RW                         (1 << 7)
/** @} */

/**
 * @name    LIS2DW12 register addresses
 * @{
 */
#define  LIS2DW12_AD_WHO_AM_I               0x0F
#define  LIS2DW12_AD_CTRL_REG1              0x20
#define  LIS2DW12_AD_CTRL_REG2              0x21
#define  LIS2DW12_AD_CTRL_REG3              0x22
#define  LIS2DW12_AD_CTRL_REG4_INT1         0x23
#define  LIS2DW12_AD_CTRL_REG5_INT2         0x24
#define  LIS2DW12_AD_CTRL_REG6              0x25
#define  LIS2DW12_AD_OUT_T                  0x26
#define  LIS2DW12_AD_STATUS_REG             0x27
#define  LIS2DW12_AD_OUT_X_L                0x28
#define  LIS2DW12_AD_OUT_X_H                0x29
#define  LIS2DW12_AD_OUT_Y_L                0x2A
#define  LIS2DW12_AD_OUT_Y_H                0x2B
#define  LIS2DW12_AD_OUT_Z_L                0x2C
#define  LIS2DW12_AD_OUT_Z_H                0x2D
#define  LIS2DW12_AD_FIFO_CTRL              0x2E
#define  LIS2DW12_AD_FIFO_SAMPLES           0x2F
#define  LIS2DW12_AD_TAP_THS_X              0x30
#define  LIS2DW12_AD_TAP_THS_Y              0x31
#define  LIS2DW12_AD_TAP_THS_Z              0x32
#define  LIS2DW12_AD_INT_DUR                0x33
#define  LIS2DW12_AD_WAKE_UP_THS            0x34
#define  LIS2DW12_AD_WAKE_UP_DUR            0x35
#define  LIS2DW12_AD_FREE_FALL              0x36
#define  LIS2DW12_AD_STATUS_DUP             0x37
#define  LIS2DW12_AD_WAKE_UP_SRC            0x38
#define  LIS2DW12_AD_TAP_SRC                0x39
#define  LIS2DW12_AD_SIXD_SRC               0x3A
#define  LIS2DW12_AD_ALL_INT_SRC            0x3B
#define  LIS2DW12_AD_X_OFS_USR              0x3C
#define  LIS2DW12_AD_Y_OFS_USR              0x3D
#define  LIS2DW12_AD_Z_OFS_USR              0x3E
#define  LIS2DW12_AD_CTRL_REG7              0x3F

/** @} */

/**
 * @name    LIS2DW12_CTRL_REG1 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG1_MASK             0xFF
/** @} */

/**
 * @name    LIS2DW12_CTRL_REG2 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG2_REG2_MASK        0xDF
#define LIS2DW12_CTRL_REG2_SIM              (1 << 0)
#define LIS2DW12_CTRL_REG2_I2C_DISABLE      (1 << 1)
#define LIS2DW12_CTRL_REG2_IF_ADD_INC       (1 << 2)
#define LIS2DW12_CTRL_REG2_BDU              (1 << 3)
#define LIS2DW12_CTRL_REG2_CS_PU_DIS        (1 << 4)
#define LIS2DW12_CTRL_REG2_SOFT_RESET       (1 << 6)
#define LIS2DW12_CTRL_REG2_BOOT             (1 << 7)
/** @} */

/**
 * @name    LIS2DW12_CTRL_REG3 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG3_MASK             0xFF

/**
 * @name    LIS2DW12_CTRL_REG4 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG4_MASK             0xFF

/**
 * @name    LIS2DW12_CTRL_REG5 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG5_MASK             0xFF

/** @} */

/**
 * @name    LIS2DW12_CTRL_REG6 register bits definitions
 * @{
 */
#define LIS2DW12_CTRL_REG6_MASK             0xFC
#define LIS2DW12_CTRL_REG6_BW_MASK          0xC0
#define LIS2DW12_CTRL_REG6_FS_MASK          0x30
#define LIS2DW12_CTRL_REG6_FDS_HPF          (1 << 3)
#define LIS2DW12_CTRL_REG6_FDS_LPF          (0 << 3)
#define LIS2DW12_CTRL_REG6_LOW_NOISE        (1 << 2)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LIS2DW12 SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LIS2DW12_USE_SPI) || defined(__DOXYGEN__)
#define LIS2DW12_USE_SPI                    TRUE
#endif

/**
 * @brief   LIS2DW12 shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS2DW12_SHARED_SPI) || defined(__DOXYGEN__)
#define LIS2DW12_SHARED_SPI                 FALSE
#endif

/**
 * @brief   LIS2DW12 I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LIS2DW12_USE_I2C) || defined(__DOXYGEN__)
#define LIS2DW12_USE_I2C                    FALSE
#endif

/**
 * @brief   LIS2DW12 shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS2DW12_SHARED_I2C) || defined(__DOXYGEN__)
#define LIS2DW12_SHARED_I2C                 FALSE
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !(LIS2DW12_USE_SPI ^ LIS2DW12_USE_I2C)
#error "LIS2DW12_USE_SPI and LIS2DW12_USE_I2C cannot be both true or both false"
#endif

#if LIS2DW12_USE_SPI && !HAL_USE_SPI
#error "LIS2DW12_USE_SPI requires HAL_USE_SPI"
#endif

#if LIS2DW12_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LIS2DW12_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LIS2DW12_USE_I2C && !HAL_USE_I2C
#error "LIS2DW12_USE_I2C requires HAL_USE_I2C"
#endif

#if LIS2DW12_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LIS2DW12_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/*
 * CHTODO: Add support for LIS2DW12 over I2C.
 */
#if LIS2DW12_USE_I2C
#error "LIS2DW12 over I2C still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @name    LIS2DW12 data structures and types
 * @{
 */
/**
 * @brief   Structure representing a LIS2DW12 driver.
 */
typedef struct LIS2DW12Driver LIS2DW12Driver;

/**
 * @brief   LIS2DW12 output data rate.
 */
typedef enum {
  LIS2DW12_ACC_ODR_PD = 0x00,       /**< Power down mode                    */
  LIS2DW12_ACC_ODR_12P5HZLP = 0x10, /**< ODR 12.5 Hz/1.6 Hz low power.      */
  LIS2DW12_ACC_ODR_12P5HZ = 0x20,   /**< ODR 12.5 Hz.                       */
  LIS2DW12_ACC_ODR_25HZ = 0x30,     /**< ODR 25 Hz.                         */
  LIS2DW12_ACC_ODR_50HZ = 0x40,     /**< ODR 50 Hz.                         */
  LIS2DW12_ACC_ODR_100HZ = 0x50,    /**< ODR 100 Hz.                        */
  LIS2DW12_ACC_ODR_200HZ = 0x60,    /**< ODR 200 Hz.                        */
  LIS2DW12_ACC_ODR_400HZ = 0x70,    /**< ODR 400 Hz/200 Hz low power.       */
  LIS2DW12_ACC_ODR_800HZ = 0x80,    /**< ODR 800 Hz/200 Hz low power.       */
  LIS2DW12_ACC_ODR_1600HZ = 0x90    /**< ODR 1600 Hz/200 Hz low power.      */
}lis2dw12_acc_odr_t;

/**
 * @brief   LIS2DW12 output resolution.
 */
typedef enum {
  LIS2DW12_ACC_OR_LP = 0x00,        /**< Low-Power Mode, 12/14-bit          */
  LIS2DW12_ACC_OR_HP = 0x04,        /**< High-Performance mode, 14-bit      */
  LIS2DW12_ACC_OR_SINGLE = 0x08     /**< Single data convertion, 12/14bit   */
}lis2dw12_acc_or_t;

/**
 * @brief   LIS2DW12 low power mode.
 */
typedef enum {
  LIS2DW12_ACC_LP_MODE1 = 0x00,     /**< Low-Power Mode 1, 12-bit           */
  LIS2DW12_ACC_LP_MODE2 = 0x01,     /**< Low-Power Mode 2, 14-bit           */
  LIS2DW12_ACC_LP_MODE3 = 0x02,     /**< Low-Power Mode 3, 14-bit           */
  LIS2DW12_ACC_LP_MODE4 = 0x03,     /**< Low-Power Mode 4, 14-bit           */
}lis2dw12_acc_lp_t;

/**
 * @brief   LIS2DW12 Bandwidth selection.
 */
typedef enum {
  LIS2DW12_ACC_BW_ODR2 = 0x00,      /**< ODR/2.                             */
  LIS2DW12_ACC_BW_ODR4 = 0x40,      /**< ODR/2.                             */
  LIS2DW12_ACC_BW_ODR10 = 0x80,     /**< ODR/10.                            */
  LIS2DW12_ACC_BW_ODR20 = 0xC0,      /**< ODR/20.                            */
}lis2dw12_acc_bw_t;

/**
 * @brief   LIS2DW12 full scale.
 */
typedef enum {
  LIS2DW12_ACC_FS_2G = 0x00,        /**< Full scale �2g.                    */
  LIS2DW12_ACC_FS_4G = 0x10,        /**< Full scale �4g.                    */
  LIS2DW12_ACC_FS_8G = 0x20,        /**< Full scale �8g.                    */
  LIS2DW12_ACC_FS_16G = 0x30        /**< Full scale �16g.                   */
}lis2dw12_acc_fs_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LIS2DW12_UNINIT = 0,              /**< Not initialized.                   */
  LIS2DW12_STOP = 1,                /**< Stopped.                           */
  LIS2DW12_READY = 2,               /**< Ready.                             */
} lis2dw12_state_t;

/**
 * @brief   LIS2DW12 configuration structure.
 */
typedef struct {

#if (LIS2DW12_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LIS2DW12.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LIS2DW12.
   */
  const SPIConfig           *spicfg;
#endif /* LIS2DW12_USE_SPI */
#if (LIS2DW12_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LIS2DW12.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LIS2DW12.
   */
  const I2CConfig           *i2ccfg;
#endif /* LIS2DW12_USE_I2C */
  /**
   * @brief LIS2DW12 accelerometer subsystem initial sensitivity.
   */
  float                     *accsensitivity;
  /**
   * @brief LIS2DW12 accelerometer subsystem initial bias.
   */
  float                     *accbias;
  /**
   * @brief LIS2DW12 output data rate selection.
   */
  lis2dw12_acc_odr_t        accoutputdatarate;
  /**
   * @brief LIS2DW12 output data resolution selection.
   */
  lis2dw12_acc_or_t         accoutputresolution;
  /**
   * @brief LIS2DW12 low power mode selection
   */
  lis2dw12_acc_lp_t         acclowpowermode;
  /**
   * @brief LIS2DW12 output bandwidth selection.
   */
  lis2dw12_acc_bw_t         accbadwidthselect;
  /**
   * @brief LIS2DW12 accelerometer subsystem initial full scale.
   */
  lis2dw12_acc_fs_t         accfullscale;
} LIS2DW12Config;

/**
 * @brief   @p LIS2DW12 specific methods.
 */
#define _lis2dw12_methods_alone                                             \
  /* Change full scale value of LIS2DW12 .*/                                \
  msg_t (*set_full_scale)(LIS2DW12Driver *devp, lis2dw12_acc_fs_t fs);


/**
 * @brief   @p LIS2DW12 specific methods with inherited ones.
 */
#define _lis2dw12_methods                                                   \
  _base_object_methods                                                      \
  _lis2dw12_methods_alone

/**
 * @extends BaseObjectVMT
 *
 * @brief   @p LIS2DW12 accelerometer virtual methods table.
 */
struct LIS2DW12VMT {
  _lis2dw12_methods
};

/**
 * @brief   @p LIS2DW12Driver specific data.
 */
#define _lis2dw12_data                                                      \
  /* Driver state.*/                                                        \
  lis2dw12_state_t          state;                                          \
  /* Current configuration data.*/                                          \
  const LIS2DW12Config      *config;                                        \
  /* Accelerometer subsystem axes number.*/                                 \
  size_t                    accaxes;                                        \
  /* Current sensitivity.*/                                                 \
  float                     accsensitivity[LIS2DW12_ACC_NUMBER_OF_AXES];    \
  /* Bias data.*/                                                           \
  int32_t                   accbias[LIS2DW12_ACC_NUMBER_OF_AXES];           \
  /* Current full scale value.*/                                            \
  float                     accfullscale;

/**
 * @brief   LIS2DW12 3-axis accelerometer class.
 */
struct LIS2DW12Driver {
  /** @brief Virtual Methods Table.*/
  const struct LIS2DW12VMT     *vmt;
  /** @brief Base accelerometer interface.*/
  BaseAccelerometer           acc_if;
  _lis2dw12_data
};
/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Return the number of axes of the BaseAccelerometer.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 *
 * @return              the number of axes.
 *
 * @api
 */
#define lis2dw12AccelerometerGetAxesNumber(devp)                            \
        accelerometerGetAxesNumber(&((devp)->acc_if))

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lis2dw12AccelerometerReadRaw(devp, axes)                            \
        accelerometerReadRaw(&((devp)->acc_if), axes)

/**
 * @brief   Retrieves cooked data from the BaseAccelerometer.
 * @note    This data is manipulated according to the formula
 *          cooked = (raw * sensitivity) - bias.
 * @note    Final data is expressed as milli-G.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 * @param[out] axes     a buffer which would be filled with cooked data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
#define lis2dw12AccelerometerReadCooked(devp, axes)                         \
        accelerometerReadCooked(&((devp)->acc_if), axes)

/**
 * @brief   Set bias values for the BaseAccelerometer.
 * @note    Bias must be expressed as milli-G.
 * @note    The bias buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 * @param[in] bp        a buffer which contains biases.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis2dw12AccelerometerSetBias(devp, bp)                              \
        accelerometerSetBias(&((devp)->acc_if), bp)

/**
 * @brief   Reset bias values for the BaseAccelerometer.
 * @note    Default biases value are obtained from device datasheet when
 *          available otherwise they are considered zero.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis2dw12AccelerometerResetBias(devp)                                \
        accelerometerResetBias(&((devp)->acc_if))

/**
 * @brief   Set sensitivity values for the BaseAccelerometer.
 * @note    Sensitivity must be expressed as milli-G/LSB.
 * @note    The sensitivity buffer must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 * @param[in] sp        a buffer which contains sensitivities.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 *
 * @api
 */
#define lis2dw12AccelerometerSetSensitivity(devp, sp)                       \
        accelerometerSetSensitivity(&((devp)->acc_if), sp)

/**
 * @brief   Reset sensitivity values for the BaseAccelerometer.
 * @note    Default sensitivities value are obtained from device datasheet.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lis2dw12AccelerometerResetSensitivity(devp)                         \
        accelerometerResetSensitivity(&((devp)->acc_if))

/**
 * @brief   Changes the LIS2DW12Driver accelerometer fullscale value.
 * @note    This function also rescale sensitivities and biases based on
 *          previous and next fullscale value.
 * @note    A recalibration is highly suggested after calling this function.
 *
 * @param[in] devp      pointer to @p LIS2DW12Driver.
 * @param[in] fs        new fullscale value.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    otherwise.
 *
 * @api
 */
#define lis2dw12AccelerometerSetFullScale(devp, fs)                         \
        (devp)->vmt->acc_set_full_scale(devp, fs)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lis2dw12ObjectInit(LIS2DW12Driver *devp);
  msg_t lis2dw12Start(LIS2DW12Driver *devp, const LIS2DW12Config *config);
  void lis2dw12Stop(LIS2DW12Driver *devp);
#ifdef __cplusplus
}
#endif

#endif /* _LIS2DW12_H_ */

/** @} */

