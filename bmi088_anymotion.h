/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi088_anymotion.h
* @date       2023-03-27
* @version    v1.7.1
*
*/

/*! \file bmi088_anymotion.h
 * \brief Sensor Driver for BMI088_ANYMOTION family of sensors */

/*!
 * @defgroup bmi088_anymotion BMI088_ANYMOTION
 */

#ifndef BMI088_ANYMOTION_H_
#define BMI088_ANYMOTION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/*                          Header files                             */
/*********************************************************************/
#include "bmi08.h"

/*********************************************************************/
/*                     Macro Definitions                             */
/*********************************************************************/

/**\name    Accel unique chip identifier */
#define BMI088_ANYMOTION_ACCEL_CHIP_ID_PRIM                   UINT8_C(0x1A)
#define BMI088_ANYMOTION_ACCEL_CHIP_ID_SEC                    UINT8_C(0x1E)

/**\name    Orientation result register*/
#define BMI088_ANYMOTION_ACCEL_ANY_MOT_INT                    UINT8_C(0x02)
#define BMI088_ANYMOTION_ACCEL_ERR_INT_ENABLE                 UINT8_C(0x80)

/**\name  BMI09 Accel Range */
#define BMI088_ANYMOTION_ACCEL_RANGE_3G                       UINT8_C(0x00)
#define BMI088_ANYMOTION_ACCEL_RANGE_6G                       UINT8_C(0x01)
#define BMI088_ANYMOTION_ACCEL_RANGE_12G                      UINT8_C(0x02)
#define BMI088_ANYMOTION_ACCEL_RANGE_24G                      UINT8_C(0x03)

#define BMI088_ANYMOTION_ACCEL_ANY_MOT_INT_DISABLE            UINT8_C(0x00)
#define BMI088_ANYMOTION_ACCEL_ANY_MOT_INT_ENABLE             UINT8_C(0x02)
#define BMI088_ANYMOTION_ACCEL_ERR_INT_DISABLE                UINT8_C(0x00)

/**\name    Axis Remap Feature size */
#define BMI088_ANYMOTION_FEATURE_SIZE                         UINT8_C(0x1E)

/**\name    Accel remap start address */
#define BMI088_ANYMOTION_ADDR_AXES_REMAP_START                UINT8_C(0x1C)

/**\name    Mask definition for config version */
#define BMI088_ANYMOTION_CONFIG_MAJOR_MASK                    UINT16_C(0x3C0)
#define BMI088_ANYMOTION_CONFIG_MINOR_MASK                    UINT8_C(0x3F)

/**\name    Bit position for major version from config */
#define BMI088_ANYMOTION_CONFIG_MAJOR_POS                     UINT8_C(0x06)

/**\name    Config ID start address */
#define BMI088_ANYMOTION_ADDR_CONFIG_ID_START                 UINT8_C(0x1A)

#define BMI088_ANYMOTION_E_REMAP_ERROR                        INT8_C(-11)

#define BMI088_ANYMOTION_STATUS_SET                           UINT8_C(1)
#define BMI088_ANYMOTION_STATUS_CLEAR                         UINT8_C(0)

/* Power mode switching delay 30ms - refer data sheet table 3 */
#define BMI088_ANYMOTION_GYRO_POWER_MODE_SWITCHING_DELAY_MS   UINT8_C(30)

/* Accel device init delay - refer data sheet section 3 */
#define BMI088_ANYMOTION_ACCEL_DEVICE_INIT_DELAY_MS           UINT8_C(50)

/* Accel power mode switching delay - data sheet section 4.1.1 */
#define BMI088_ANYMOTION_ACCEL_POWER_MODE_SWITCHING_DELAY_MS  UINT8_C(5)

/* Delay required to get temperature data 1.28s - refer data sheet section 5.3.7 */
#define BMI088_ANYMOTION_TEMPERATURE_DATA_READ_DELAY_MS       UINT8_C(1300)

/* Self-test: Resulting minimum difference signal in mg for BMI088_ANYMOTION */
#define BMI088_ANYMOTION_ST_ACC_X_AXIS_SIGNAL_DIFF            UINT16_C(1000)
#define BMI088_ANYMOTION_ST_ACC_Y_AXIS_SIGNAL_DIFF            UINT16_C(1000)
#define BMI088_ANYMOTION_ST_ACC_Z_AXIS_SIGNAL_DIFF            UINT16_C(500)

/**\name     Feature start Addresses  */
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_ADR                  UINT8_C(0x00)

/**\name    Accel Any-motion Macros  */
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_LEN                  UINT8_C(0x02)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_THRESHOLD_MASK       UINT16_C(0x07FF)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_THRESHOLD_SHIFT      UINT8_C(0x00)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_SEL_MASK             UINT16_C(0x0800)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_SEL_SHIFT            UINT8_C(0x0B)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_DURATION_MASK        UINT16_C(0x1FFF)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_DURATION_SHIFT       UINT8_C(0x00)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_X_EN_MASK            UINT16_C(0x2000)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_X_EN_SHIFT           UINT8_C(0x0D)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_Y_EN_MASK            UINT16_C(0x4000)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_Y_EN_SHIFT           UINT8_C(0x0E)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_Z_EN_MASK            UINT16_C(0x8000)
#define BMI088_ANYMOTION_ACCEL_ANYMOTION_Z_EN_SHIFT           UINT8_C(0x0F)

/*********************************************************************/
/*! @name       Macro Definitions for Axes re-mapping                */
/*********************************************************************/

/**\name Enable/Disable Selections */
#define BMI088_ANYMOTION_X_AXIS                               UINT8_C(0)
#define BMI088_ANYMOTION_Y_AXIS                               UINT8_C(1)
#define BMI088_ANYMOTION_Z_AXIS                               UINT8_C(2)

/**\name Define values of axis and its sign for re-map settings */
#define BMI088_ANYMOTION_MAP_X_AXIS                           UINT8_C(0x00)
#define BMI088_ANYMOTION_MAP_Y_AXIS                           UINT8_C(0x01)
#define BMI088_ANYMOTION_MAP_Z_AXIS                           UINT8_C(0x02)
#define BMI088_ANYMOTION_MAP_POSITIVE                         UINT8_C(0x00)
#define BMI088_ANYMOTION_MAP_NEGATIVE                         UINT8_C(0x01)

/*! @name Macros for the user-defined values of axes and their polarities */
#define BMI088_ANYMOTION_X                                    UINT8_C(0x01)
#define BMI088_ANYMOTION_NEG_X                                UINT8_C(0x09)
#define BMI088_ANYMOTION_Y                                    UINT8_C(0x02)
#define BMI088_ANYMOTION_NEG_Y                                UINT8_C(0x0A)
#define BMI088_ANYMOTION_Z                                    UINT8_C(0x04)
#define BMI088_ANYMOTION_NEG_Z                                UINT8_C(0x0C)
#define BMI088_ANYMOTION_AXIS_MASK                            UINT8_C(0x07)
#define BMI088_ANYMOTION_AXIS_SIGN                            UINT8_C(0x08)

/**\name Mask definitions for axes re-mapping */
#define BMI088_ANYMOTION_X_AXIS_MASK                          UINT8_C(0x03)
#define BMI088_ANYMOTION_X_AXIS_SIGN_MASK                     UINT8_C(0x04)
#define BMI088_ANYMOTION_Y_AXIS_MASK                          UINT8_C(0x18)
#define BMI088_ANYMOTION_Y_AXIS_SIGN_MASK                     UINT8_C(0x20)
#define BMI088_ANYMOTION_Z_AXIS_MASK                          UINT8_C(0xC0)
#define BMI088_ANYMOTION_Z_AXIS_SIGN_MASK                     UINT8_C(0x01)

/**\name Bit position for axes re-mapping */
#define BMI088_ANYMOTION_X_AXIS_SIGN_POS                      UINT8_C(0x02)
#define BMI088_ANYMOTION_Y_AXIS_POS                           UINT8_C(0x03)
#define BMI088_ANYMOTION_Y_AXIS_SIGN_POS                      UINT8_C(0x05)
#define BMI088_ANYMOTION_Z_AXIS_POS                           UINT8_C(0x06)

/*********************************************************************/
/*                          Enumerators                              */
/*********************************************************************/

/*!
 *  @brief Enum to select accelerometer interrupts
 */
enum bmi088_anymotion_accel_int_types {
    BMI088_ANYMOTION_ACCEL_DATA_RDY_INT,
    /* FIFO Watermark interrupt */
    BMI088_ANYMOTION_ACCEL_INT_FIFO_WM,
    /* FIFO Full interrupt */
    BMI088_ANYMOTION_ACCEL_INT_FIFO_FULL,
    /* Accel anymotion interrupt*/
    BMI088_ANYMOTION_ANYMOTION_INT,
    /* Accel Error interrupt */
    BMI088_ANYMOTION_ERROR_INT
};

/*********************************************************************/
/*                          Structures                               */
/*********************************************************************/

/*!
 *  @brief Anymotion config structure
 */
struct bmi088_anymotion_anymotion_cfg
{
    /* 11 bit threshold of anymotion detection (threshold = X mg * 2,048 (5.11 format)) */
    uint16_t threshold;

    /*! Enable any-motion feature */
    uint16_t enable;

    /* 13 bit set the duration for any- and nomotion (time = duration * 20ms (@50Hz)) */
    uint16_t duration;

    /* Enable anymotion detection for x axis */
    uint16_t x_en;

    /* Enable anymotion detection for y axis */
    uint16_t y_en;

    /* Enable anymotion detection for z axis */
    uint16_t z_en;
};

/*! @name Structure to store the re-mapped axis */
struct bmi088_anymotion_remap
{
    /*! Re-mapped x-axis */
    uint8_t x;

    /*! Re-mapped y-axis */
    uint8_t y;

    /*! Re-mapped z-axis */
    uint8_t z;
};

/*********************************************************************/
/* Function prototype declarations */

/*********************** BMI088_ANYMOTION Accelerometer function prototypes ************************/

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiInit Accel Initialization
 * @brief Initialize the accel sensor and device structure
 */

/*!
 * \ingroup bmi088_anymotionApiInit
 * \page bmi088_anymotion_api_bmi088_anymotion_init bmi088_anymotion_init
 * \code
 * int8_t bmi088_anymotion_init(struct bmi08_dev *dev);
 * \endcode
 * @details This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 *
 *  @param[in,out] dev  : Structure instance of bmi08_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_init(struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiConfig Configurations
 * @brief Setting up configurations
 */

/*!
 * \ingroup bmi088_anymotionApiConfig
 * \page bmi088_anymotion_api_bmi088_anymotion_set_meas_conf bmi088_anymotion_set_meas_conf
 * \code
 * int8_t bmi088_anymotion_set_meas_conf(struct bmi08_dev *dev);
 * \endcode
 * @details This API sets the Output data rate, range and bandwidth
 *  of accel sensor.
 *  @param[in] dev  : Structure instance of bmi08_dev.
 *
 *  @note : The user must select one among the following macros to
 *  select range value for BMI09 accel
 *
 *@verbatim
 *      config                                   |   value
 *      -----------------------------------------|---------------------------
 *      BMI088_ANYMOTION_ACCEL_RANGE_3G          |   0x00
 *      BMI088_ANYMOTION_ACCEL_RANGE_6G          |   0x01
 *      BMI088_ANYMOTION_ACCEL_RANGE_12G         |   0x02
 *      BMI088_ANYMOTION_ACCEL_RANGE_24G         |   0x03
 *@endverbatim
 *
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_set_meas_conf(struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiData Accel Data
 * @brief Read accel data from the sensor
 */

/*!
 * \ingroup bmi088_anymotionApiData
 * \page bmi088_anymotion_api_bmi088_anymotion_get_data bmi088_anymotion_get_data
 * \code
 * int8_t bmi088_anymotion_get_data(struct bmi08_sensor_data *accel, struct bmi08_dev *dev);
 * \endcode
 * @details This API reads the accel data from the sensor,
 *  store it in the bmi08_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[in]  dev    : Structure instance of bmi08_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_get_data(struct bmi08_sensor_data *accel, struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiIntConfig Accel Interrupt configurations
 * @brief Set accel sensor interrupt configurations
 */

/*!
 * \ingroup bmi088_anymotionApiIntConfig
 * \page bmi088_anymotion_api_bmi088_anymotion_set_int_config bmi088_anymotion_set_int_config
 * \code
 * int8_t bmi088_anymotion_set_int_config(const struct bmi08_accel_int_channel_cfg *int_config, enum bmi088_anymotion_accel_int_types int_type, struct bmi08_dev *dev);
 * \endcode
 * @details This API configures the necessary accel interrupt
 *  based on the user settings in the bmi088_anymotion_accel_int_channel_cfg
 *  structure instance.
 *
 *  @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 *  @param[in] int_type    : Enumerator instance of bmi088_anymotion_accel_int_types.
 *  @param[in] dev         : Structure instance of bmi08_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_set_int_config(const struct bmi08_accel_int_channel_cfg *int_config,
                                       enum bmi088_anymotion_accel_int_types int_type,
                                       struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiSelftest Accel Self test
 * @brief Perform self test of accel sensor
 */

/*!
 * \ingroup bmi088_anymotionApiSelftest
 * \page bmi088_anymotion_api_bmi088_anymotion_perform_selftest bmi088_anymotion_perform_selftest
 * \code
 * int8_t bmi088_anymotion_perform_selftest(struct bmi08_dev *dev);
 * \endcode
 * @details This API checks whether the self test functionality of the sensor
 *  is working or not
 *
 *  @param[in] dev    : Structure instance of bmi08_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_perform_selftest(struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiAnymotion Accel Anymotion
 * @brief Configure anymotion of sensor
 */

/*!
 * \ingroup bmi088_anymotionApiAnymotion
 * \page bmi088_anymotion_api_bmi088_anymotion_configure_anymotion bmi088_anymotion_configure_anymotion
 * \code
 * int8_t bmi088_anymotion_configure_anymotion(struct bmi088_anymotion_anymotion_cfg anymotion_cfg, struct bmi08_dev *dev);
 * \endcode
 * @details This API is used to enable/disable and configure the anymotion
 *  feature.
 *
 *  @param[in] anymotion_cfg : configure anymotion feature
 *  @param[in] dev : Structure instance of bmi08_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_configure_anymotion(struct bmi088_anymotion_anymotion_cfg anymotion_cfg, struct bmi08_dev *dev);

/*!
 * \ingroup bmi088_anymotionApiInt
 * \page bmi088_anymotion_api_bmi088_anymotion_get_feat_int_status bmi088_anymotion_get_feat_int_status
 * \code
 * int8_t bmi088_anymotion_get_feat_int_status(uint8_t *int_status, struct bmi08_dev *dev);
 * \endcode
 * @details This API is to get accel feature interrupt status
 *
 * @param[out] int_status      : Variable to store interrupt status
 * @param[in]  dev             : Structure instance of bmi08_dev
 *
 *@verbatim
 *-----------------------------------------
 *   int_status    |     Interrupt
 *-----------------------------------------
 *      0x02       |    Any-Motion
 *------------------------------------------
 *@endverbatim
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bmi088_anymotion_get_feat_int_status(uint8_t *int_status, struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiRemap Axis Remap
 * @brief Functions of axis remapping of bmi09 sensor
 */

/*!
 * \ingroup bmi088_anymotionApiRemap
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 * \page bmi088_anymotion_api_bmi088_anymotion_set_remap_axes bmi088_anymotion_set_remap_axes
 * \code
 * int8_t bmi088_anymotion_set_remap_axes(const struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev);
 * \endcode
 * @details This API sets the re-mapped x, y and z axes to the sensor and
 * updates them in the device structure.
 *
 * @param[in] remapped_axis    : Pointer to store axes re-mapping data.
 * @param[in] dev              : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0  -> Fail
 *
 */
int8_t bmi088_anymotion_set_remap_axes(const struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev);

/*!
 * \ingroup bmi088_anymotionApiRemap
 * \page bmi088_anymotion_api_bmi088_anymotion_get_remap_axes bmi088_anymotion_get_remap_axes
 * \code
 * int8_t bmi088_anymotion_get_remap_axes(struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev);
 * \endcode
 * @details This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 *
 * @param[out] remapped_axis   : Structure instance of bmi088_anymotion_remap
 * @param[in] dev              : Structure instance of bmi08_dev
 *
 * @return Result of API execution status.
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
int8_t bmi088_anymotion_get_remap_axes(struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev);

/**
 * \ingroup bmi088_anymotion
 * \defgroup bmi088_anymotionApiVersion Major and Minor Revision
 * @brief Reads major and minor revision of sensor
 */

/*!
 * \ingroup bmi088_anymotionApiVersion
 * \page bmi088_anymotion_api_bmi088_anymotion_get_version_config bmi088_anymotion_get_version_config
 * \code
 *int8_t bmi088_anymotion_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bmi08_dev *dev);
 * \endcode
 * @details This API is used to get the config file major and minor information.
 *
 * @param[in] dev              : Structure instance of bmi08_dev.
 * @param[out] config_major    : Pointer to data buffer to store the config major.
 * @param[out] config_minor    : Pointer to data buffer to store the config minor.
 *
 *  @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
int8_t bmi088_anymotion_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bmi08_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_ANYMOTION_H_ */

/** @}*/
