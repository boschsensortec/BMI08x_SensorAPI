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
* @file       bmi088_anymotiona.c
* @date       2023-03-27
* @version    v1.7.1
*
*/

/*! \file bmi088_anymotiona.c
 * \brief Sensor Driver for BMI088_ANYMOTION family of sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include <stdio.h>
#include "bmi088_anymotion.h"

/****************************************************************************/

/** \name       Macros
 ****************************************************************************/

/**\name    Value of LSB_PER_G = (power(2, BMI088_ANYMOTION_16_BIT_RESOLUTION) / (2 * range)) */
#define LSB_PER_G  UINT32_C(1365) /* for the 16-bit resolution and 24g range */

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/*!
 * @brief Accel self-test diff xyz data structure
 */
struct bmi088_anymotion_selftest_delta_limit
{
    /*! Accel X  data */
    int16_t x;

    /*! Accel Y  data */
    int16_t y;

    /*! Accel Z  data */
    int16_t z;
};

/**\name Feature configuration file */
const uint8_t bmi088_anymotion_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x44, 0x00, 0x80, 0x2e, 0xe4, 0x01, 0xb0, 0xf0,
    0x10, 0x30, 0x21, 0x2e, 0x16, 0xf0, 0x80, 0x2e, 0x9f, 0x00, 0x19, 0x50, 0x41, 0x30, 0x01, 0x42, 0x3c, 0x82, 0x01,
    0x2e, 0x42, 0x00, 0x42, 0x40, 0x42, 0x42, 0x02, 0x30, 0x17, 0x56, 0x25, 0x2e, 0x42, 0x00, 0x03, 0x0a, 0x49, 0x82,
    0xc0, 0x2e, 0x40, 0x42, 0x00, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd4,
    0x5b, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e,
    0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x70, 0x50, 0xd0, 0x7f, 0xf5, 0x7f, 0xe4, 0x7f, 0x34, 0x30, 0x01, 0x2e, 0x01, 0xf0, 0x0e, 0xbc,
    0x8e, 0xba, 0x92, 0x7f, 0xc1, 0x7f, 0xbb, 0x7f, 0xa3, 0x7f, 0xa5, 0x04, 0x05, 0x52, 0x07, 0x50, 0x98, 0x2e, 0x94,
    0x00, 0x11, 0x30, 0x23, 0x2e, 0x29, 0x00, 0x01, 0x31, 0x23, 0x2e, 0xb8, 0xf0, 0xe4, 0x6f, 0xd0, 0x6f, 0xf5, 0x6f,
    0xc1, 0x6f, 0x92, 0x6f, 0xa3, 0x6f, 0xbb, 0x6f, 0x90, 0x5f, 0xc8, 0x2e, 0x98, 0x2e, 0xa8, 0x00, 0x20, 0x26, 0x98,
    0x2e, 0x90, 0x00, 0x01, 0x2e, 0x40, 0xf0, 0x21, 0x2e, 0x41, 0x00, 0x10, 0x30, 0x21, 0x2e, 0x59, 0xf0, 0x98, 0x2e,
    0xa3, 0x00, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x01, 0x2e, 0x29, 0x00, 0x00, 0xb2, 0x0b, 0x2f, 0x00, 0x30, 0x21,
    0x2e, 0x29, 0x00, 0x05, 0x50, 0x98, 0x2e, 0x9e, 0x01, 0x20, 0x30, 0x21, 0x2e, 0x5f, 0xf0, 0x05, 0x50, 0x98, 0x2e,
    0x11, 0x01, 0x98, 0x2e, 0xa3, 0x00, 0x01, 0x2e, 0x42, 0x00, 0x01, 0x30, 0x21, 0x2e, 0x5e, 0xf0, 0x23, 0x2e, 0x42,
    0x00, 0xe3, 0x2d, 0x00, 0x31, 0xc0, 0x2e, 0x21, 0x2e, 0xba, 0xf0, 0x43, 0x86, 0x25, 0x40, 0x04, 0x40, 0xd8, 0xbe,
    0x2c, 0x0b, 0x22, 0x11, 0x54, 0x42, 0x03, 0x80, 0x4b, 0x0e, 0xf6, 0x2f, 0xb8, 0x2e, 0x1a, 0x24, 0x30, 0x00, 0x80,
    0x2e, 0x65, 0x00, 0x01, 0x2e, 0x55, 0xf0, 0xc0, 0x2e, 0x21, 0x2e, 0x55, 0xf0, 0x15, 0x50, 0x41, 0x30, 0x02, 0x40,
    0x51, 0x0a, 0x01, 0x42, 0x18, 0x82, 0x09, 0x50, 0x60, 0x42, 0x70, 0x3c, 0x0b, 0x54, 0x42, 0x42, 0x69, 0x82, 0x82,
    0x32, 0x43, 0x40, 0x18, 0x08, 0x02, 0x0a, 0x40, 0x42, 0x42, 0x80, 0x02, 0x3f, 0x01, 0x40, 0x10, 0x50, 0x4a, 0x08,
    0xfb, 0x7f, 0x11, 0x42, 0x0b, 0x31, 0x0b, 0x42, 0x3e, 0x80, 0xf1, 0x30, 0x01, 0x42, 0x00, 0x2e, 0x01, 0x2e, 0x40,
    0xf0, 0x1e, 0xb2, 0x01, 0x2f, 0x1a, 0x90, 0x20, 0x2f, 0x03, 0x30, 0x0f, 0x50, 0x0d, 0x54, 0xd4, 0x33, 0x06, 0x30,
    0x13, 0x52, 0xf5, 0x32, 0x1d, 0x1a, 0xe3, 0x22, 0x18, 0x1a, 0x11, 0x58, 0xe3, 0x22, 0x04, 0x30, 0xd5, 0x40, 0xb5,
    0x0d, 0xe1, 0xbe, 0x6f, 0xbb, 0x80, 0x91, 0xa9, 0x0d, 0x01, 0x89, 0xb5, 0x23, 0x10, 0xa1, 0xf7, 0x2f, 0xda, 0x0e,
    0xd4, 0x33, 0xeb, 0x2f, 0x01, 0x2e, 0x2f, 0x00, 0x70, 0x1a, 0x00, 0x30, 0x21, 0x30, 0x02, 0x2c, 0x08, 0x22, 0x30,
    0x30, 0x00, 0xb2, 0x06, 0x2f, 0x21, 0x2e, 0x59, 0xf0, 0x98, 0x2e, 0xa3, 0x00, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e,
    0xfb, 0x6f, 0xf0, 0x5f, 0xb8, 0x2e, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xaa,
    0x00, 0x05, 0xe0, 0x2a, 0x00, 0x89, 0xf0, 0xaf, 0x00, 0xff, 0x00, 0xff, 0xb7, 0x00, 0x02, 0x00, 0xb0, 0x05, 0x80,
    0xb1, 0xf0, 0x80, 0x00, 0x59, 0xf0, 0x40, 0x1c, 0x88, 0x00, 0x3e, 0x00, 0x88, 0x00, 0x09, 0x2e, 0x01, 0x01, 0x0b,
    0x2e, 0x00, 0x01, 0x42, 0xbd, 0xaf, 0xb9, 0xc1, 0xbc, 0x54, 0xbf, 0x1f, 0xb9, 0xef, 0xbb, 0xcf, 0xb8, 0x9a, 0x0b,
    0x10, 0x50, 0xc0, 0xb3, 0xb1, 0x0b, 0x77, 0x2f, 0x80, 0xb3, 0x75, 0x2f, 0x0f, 0x2e, 0x43, 0x00, 0x01, 0x8c, 0xc0,
    0x91, 0x13, 0x2f, 0xc1, 0x83, 0x23, 0x2e, 0x43, 0x00, 0x00, 0x40, 0x21, 0x2e, 0x3d, 0x00, 0x1f, 0x50, 0x91, 0x41,
    0x11, 0x42, 0x01, 0x30, 0x82, 0x41, 0x02, 0x42, 0xf0, 0x5f, 0x23, 0x2e, 0x2d, 0x00, 0x23, 0x2e, 0x2e, 0x00, 0x23,
    0x2e, 0x40, 0x00, 0xb8, 0x2e, 0xd5, 0xbe, 0xc3, 0xbf, 0x55, 0xba, 0xc0, 0xb2, 0xf3, 0xba, 0x07, 0x30, 0x03, 0x30,
    0x09, 0x2f, 0xf0, 0x7f, 0x00, 0x2e, 0x00, 0x40, 0x07, 0x2e, 0x3d, 0x00, 0x03, 0x04, 0x00, 0xa8, 0xf8, 0x04, 0xc3,
    0x22, 0xf0, 0x6f, 0x80, 0xb2, 0x07, 0x2f, 0x82, 0x41, 0x0f, 0x2e, 0x3e, 0x00, 0x97, 0x04, 0x07, 0x30, 0x80, 0xa8,
    0xfa, 0x05, 0xd7, 0x23, 0x40, 0xb2, 0x01, 0x30, 0x02, 0x30, 0x0a, 0x2f, 0x02, 0x84, 0xf7, 0x7f, 0x00, 0x2e, 0x82,
    0x40, 0x0f, 0x2e, 0x3f, 0x00, 0x97, 0x04, 0x80, 0xa8, 0xca, 0x05, 0x97, 0x22, 0xf7, 0x6f, 0x5c, 0x0f, 0x0f, 0x2f,
    0x7c, 0x0f, 0x0d, 0x2f, 0x54, 0x0f, 0x0b, 0x2f, 0x05, 0x2e, 0x2e, 0x00, 0x81, 0x84, 0x23, 0x2e, 0x2d, 0x00, 0x55,
    0x0e, 0x25, 0x2e, 0x2e, 0x00, 0x0e, 0x2f, 0x23, 0x2e, 0x40, 0x00, 0x0c, 0x2d, 0x07, 0x2e, 0x2d, 0x00, 0x12, 0x30,
    0xda, 0x28, 0x23, 0x2e, 0x2e, 0x00, 0x5d, 0x0e, 0x27, 0x2e, 0x2d, 0x00, 0x01, 0x2f, 0x25, 0x2e, 0x40, 0x00, 0x03,
    0x2e, 0x40, 0x00, 0x40, 0xb2, 0x12, 0x2f, 0x00, 0x40, 0x21, 0x2e, 0x3d, 0x00, 0x1f, 0x50, 0x91, 0x41, 0x11, 0x42,
    0x21, 0x30, 0x82, 0x41, 0x02, 0x42, 0x00, 0x2e, 0x01, 0x2e, 0x42, 0x00, 0x01, 0x0a, 0x21, 0x2e, 0x42, 0x00, 0x03,
    0x2d, 0x00, 0x30, 0x21, 0x2e, 0x43, 0x00, 0xf0, 0x5f, 0xb8, 0x2e, 0x60, 0x50, 0x03, 0x2e, 0x0e, 0x01, 0xe0, 0x7f,
    0xf1, 0x7f, 0xdb, 0x7f, 0x30, 0x30, 0x21, 0x54, 0x0a, 0x1a, 0x28, 0x2f, 0x1a, 0x25, 0x7a, 0x82, 0x00, 0x30, 0x43,
    0x30, 0x32, 0x30, 0x05, 0x30, 0x04, 0x30, 0xf6, 0x6f, 0xf2, 0x09, 0xfc, 0x13, 0xc2, 0xab, 0xb3, 0x09, 0xef, 0x23,
    0x80, 0xb3, 0xe6, 0x6f, 0xb7, 0x01, 0x00, 0x2e, 0x8b, 0x41, 0x4b, 0x42, 0x03, 0x2f, 0x46, 0x40, 0x86, 0x17, 0x81,
    0x8d, 0x46, 0x42, 0x41, 0x8b, 0x23, 0xbd, 0xb3, 0xbd, 0x03, 0x89, 0x41, 0x82, 0x07, 0x0c, 0x43, 0xa3, 0xe6, 0x2f,
    0xe1, 0x6f, 0xa2, 0x6f, 0x52, 0x42, 0x00, 0x2e, 0xb2, 0x6f, 0x52, 0x42, 0x00, 0x2e, 0xc2, 0x6f, 0x42, 0x42, 0x03,
    0xb2, 0x06, 0x2f, 0x01, 0x2e, 0x59, 0xf0, 0x01, 0x32, 0x01, 0x0a, 0x21, 0x2e, 0x59, 0xf0, 0x06, 0x2d, 0x01, 0x2e,
    0x59, 0xf0, 0xf1, 0x3d, 0x01, 0x08, 0x21, 0x2e, 0x59, 0xf0, 0xdb, 0x6f, 0xa0, 0x5f, 0xb8, 0x2e, 0x00, 0x2e, 0x10,
    0x24, 0xfa, 0x01, 0x11, 0x24, 0x00, 0x0c, 0x12, 0x24, 0x80, 0x2e, 0x13, 0x24, 0x18, 0x00, 0x12, 0x42, 0x13, 0x42,
    0x41, 0x1a, 0xfb, 0x2f, 0x10, 0x24, 0x50, 0x39, 0x11, 0x24, 0x21, 0x2e, 0x21, 0x2e, 0x10, 0x00, 0x23, 0x2e, 0x11,
    0x00, 0x80, 0x2e, 0x10, 0x00
};

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t null_ptr_check(const struct bmi08_dev *dev);

/*!
 * @brief This API configures the pins which fire the interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] int_type    : Enumerator instance of bmi088_anymotion_accel_int_types.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_int_pin_config(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] int_type    : Enumerator instance of bmi088_anymotion_accel_int_types.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_anymotion_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API sets error interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] int_type    : Enumerator instance of bmi088_anymotion_accel_int_types.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_accel_err_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This internal API gets the re-mapped x, y and z axes from the sensor.
 *
 * @param[out] remap    : Structure that stores local copy of re-mapped axes.
 * @param[in] dev       : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_remap_axes(struct bmi08_axes_remap *remap, struct bmi08_dev *dev);

/*!
 * @brief This internal API sets the re-mapped x, y and z axes in the sensor.
 *
 * @param[in] remap     : Structure that stores local copy of re-mapped axes.
 * @param[in] dev       : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_remap_axes(const struct bmi08_axes_remap *remap, struct bmi08_dev *dev);

/*!
 * @brief This internal API gets the re-mapped accelerometer.
 *
 * @param[out] data         : Structure instance of bmi08_sensor_data.
 * @param[in]  dev          : Structure instance of bmi08_dev.
 *
 * @return None
 *
 * @retval None
 */
static void get_remapped_data(struct bmi08_sensor_data *data, const struct bmi08_dev *dev);

/*!
 * @brief This internal API is to store re-mapped axis and sign values
 * in device structure
 *
 * @param[in] remap_axis      : Value of re-mapped axis
 * @param[out]  axis          : Re-mapped axis value stored in device structure
 * @param[out]  sign          : Re-mapped axis sign stored in device structure
 *
 * @return None
 *
 * @retval None
 */
static void assign_remap_axis(uint8_t remap_axis, uint8_t *axis, uint8_t *sign);

/*!
 * @brief This internal API is to receive re-mapped axis and sign values
 * in device structure
 *
 * @param[in] remap_axis      : Re-mapped axis value
 * @param[in]  remap_sign     : Re-mapped axis sign value
 * @param[out]  axis          : Re-mapped axis stored in local structure
 *
 * @return None
 *
 * @retval None
 */
static void receive_remap_axis(uint8_t remap_axis, uint8_t remap_sign, uint8_t *axis);

/*!
 * @brief This API performs the pre-requisites needed to perform the self-test
 *
 * @param[in] dev : structure instance of bmi08_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t enable_self_test(struct bmi08_dev *dev);

/*!
 * @brief This API reads the accel data with the positive excitation
 *
 * @param[out] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev   : structure instance of bmi08_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t positive_excited_accel(struct bmi08_sensor_data *accel_pos, struct bmi08_dev *dev);

/*!
 * @brief This API reads the accel data with the negative excitation
 *
 * @param[out] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev   : structure instance of bmi08_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t negative_excited_accel(struct bmi08_sensor_data *accel_neg, struct bmi08_dev *dev);

/*!
 * @brief This API validates the self-test results
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t validate_accel_self_test(const struct bmi08_sensor_data *accel_pos,
                                       const struct bmi08_sensor_data *accel_neg);

/*!
 * @brief This API converts lsb value of axes to mg for self-test
 *
 * @param[in] accel_data_diff     : Pointer variable used to pass accel difference
 * values in g
 *
 * @param[out] accel_data_diff_mg : Pointer variable used to store accel
 * difference values in mg
 *
 * @return None
 */
static void convert_lsb_g(const struct bmi088_anymotion_selftest_delta_limit *accel_data_diff,
                          struct bmi088_anymotion_selftest_delta_limit *accel_data_diff_mg);

/*!
 * @brief This API sets the FIFO watermark interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_fifo_wm_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_accel_data_ready_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev);

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_fifo_full_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
int8_t bmi088_anymotion_init(struct bmi08_dev *dev)
{
    int8_t rslt;

    rslt = bmi08a_init(dev);

    if (rslt == BMI08_OK)
    {
        /* Structure to define the default values for axes re-mapping */
        struct bmi08_axes_remap axes_remap = {
            .x_axis = BMI088_ANYMOTION_MAP_X_AXIS, .x_axis_sign = BMI088_ANYMOTION_MAP_POSITIVE,
            .y_axis = BMI088_ANYMOTION_MAP_Y_AXIS, .y_axis_sign = BMI088_ANYMOTION_MAP_POSITIVE,
            .z_axis = BMI088_ANYMOTION_MAP_Z_AXIS, .z_axis_sign = BMI088_ANYMOTION_MAP_POSITIVE
        };

        /* Check for chip id validity */
        if ((dev->accel_chip_id == BMI088_ANYMOTION_ACCEL_CHIP_ID_PRIM) ||
            (dev->accel_chip_id == BMI088_ANYMOTION_ACCEL_CHIP_ID_SEC))
        {
            /* Set the default values for axis
             *  re-mapping in the device structure
             */
            dev->remap = axes_remap;

            /* Assign stream file */
            dev->config_file_ptr = bmi088_anymotion_config_file;
        }
        else
        {
            rslt = BMI08_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of accel sensor.
 */
int8_t bmi088_anymotion_set_meas_conf(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = { 0 };
    uint8_t range;
    uint8_t is_range_invalid = FALSE;

    /* Check validity of ODR and BW */
    rslt = bmi08a_set_meas_conf(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        range = dev->accel_cfg.range;

        /* Check for valid Range */
        if (range > BMI088_ANYMOTION_ACCEL_RANGE_24G)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If Range is valid, write it to accel config registers */
        if (!is_range_invalid)
        {
            /* Read accel config. register */
            rslt = bmi08a_get_regs(BMI08_REG_ACCEL_RANGE, &data, 1, dev);
            if (rslt == BMI08_OK)
            {
                /* Update data with current range values */
                data = BMI08_SET_BITS_POS_0(data, BMI08_ACCEL_RANGE, range);

                /* Write accel range to register */
                rslt = bmi08a_set_regs(BMI08_REG_ACCEL_RANGE, &data, 1, dev);
            }
        }
        else
        {
            /* Invalid configuration present in ODR, BW, Range */
            rslt = BMI08_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the bmi08_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi088_anymotion_get_data(struct bmi08_sensor_data *accel, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Proceed if null check is fine */
    if (accel != NULL)
    {
        /* Read accel sensor data */
        rslt = bmi08a_get_regs(BMI08_REG_ACCEL_X_LSB, data, 6, dev);

        if (rslt == BMI08_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            accel->x = ((int16_t) msblsb); /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            accel->y = ((int16_t) msblsb); /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            accel->z = ((int16_t) msblsb); /* Data in Z axis */

            /* Get the re-mapped accelerometer data */
            get_remapped_data(accel, dev);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the bmi088_anymotion_int_cfg
 * structure instance.
 */
int8_t bmi088_anymotion_set_int_config(const struct bmi08_accel_int_channel_cfg *int_config,
                                       enum bmi088_anymotion_accel_int_types int_type,
                                       struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Proceed if null check is fine */
    if (int_config != NULL)
    {
        switch (int_type)
        {
            case BMI088_ANYMOTION_ACCEL_DATA_RDY_INT:
                rslt = set_accel_data_ready_int(int_config, dev);
                break;

            case BMI088_ANYMOTION_ACCEL_INT_FIFO_WM:
                rslt = set_fifo_wm_int(int_config, dev);
                break;

            case BMI088_ANYMOTION_ACCEL_INT_FIFO_FULL:
                rslt = set_fifo_full_int(int_config, dev);
                break;

            case BMI088_ANYMOTION_ANYMOTION_INT:

                /* Anymotion interrupt */
                rslt = set_accel_anymotion_int(int_config, dev);
                break;

            case BMI088_ANYMOTION_ERROR_INT:

                /* Error interrupt */
                rslt = set_accel_err_int(int_config, dev);
                break;

            default:
                rslt = BMI08_E_INVALID_CONFIG;
                break;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 */
int8_t bmi088_anymotion_configure_anymotion(struct bmi088_anymotion_anymotion_cfg anymotion_cfg, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI088_ANYMOTION_ACCEL_ANYMOTION_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Enable data synchronization */
        data[0] = (anymotion_cfg.threshold & BMI088_ANYMOTION_ACCEL_ANYMOTION_THRESHOLD_MASK);
        data[0] |=
            ((anymotion_cfg.enable << BMI088_ANYMOTION_ACCEL_ANYMOTION_SEL_SHIFT) &
             BMI088_ANYMOTION_ACCEL_ANYMOTION_SEL_MASK);
        data[1] = (anymotion_cfg.duration & BMI088_ANYMOTION_ACCEL_ANYMOTION_DURATION_MASK);
        data[1] |=
            ((anymotion_cfg.x_en << BMI088_ANYMOTION_ACCEL_ANYMOTION_X_EN_SHIFT) &
             BMI088_ANYMOTION_ACCEL_ANYMOTION_X_EN_MASK);
        data[1] |=
            ((anymotion_cfg.y_en << BMI088_ANYMOTION_ACCEL_ANYMOTION_Y_EN_SHIFT) &
             BMI088_ANYMOTION_ACCEL_ANYMOTION_Y_EN_MASK);
        data[1] |=
            ((anymotion_cfg.z_en << BMI088_ANYMOTION_ACCEL_ANYMOTION_Z_EN_SHIFT) &
             BMI088_ANYMOTION_ACCEL_ANYMOTION_Z_EN_MASK);
        rslt = bmi08a_write_feature_config(BMI088_ANYMOTION_ACCEL_ANYMOTION_ADR,
                                           &data[0],
                                           BMI088_ANYMOTION_ACCEL_ANYMOTION_LEN,
                                           dev);
    }

    return rslt;
}

/*!
 * @brief This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 */
int8_t bmi088_anymotion_get_remap_axes(struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI08_OK) && (remapped_axis != NULL))
    {
        /* Get the re-mapped axes from the sensor */
        rslt = get_remap_axes(&dev->remap, dev);
        if (rslt == BMI08_OK)
        {
            /* Store the receive re-mapped axis and sign from device structure */
            receive_remap_axis(dev->remap.x_axis, dev->remap.x_axis_sign, &remapped_axis->x);
            receive_remap_axis(dev->remap.y_axis, dev->remap.y_axis_sign, &remapped_axis->y);
            receive_remap_axis(dev->remap.z_axis, dev->remap.z_axis_sign, &remapped_axis->z);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the re-mapped x, y and z axes to the sensor and
 * updates the them in the device structure.
 */
int8_t bmi088_anymotion_set_remap_axes(const struct bmi088_anymotion_remap *remapped_axis, struct bmi08_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store all the re-mapped axes */
    uint8_t remap_axes = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);
    if ((rslt == BMI08_OK) && (remapped_axis != NULL))
    {
        /* Check whether all the axes are re-mapped */
        remap_axes = remapped_axis->x | remapped_axis->y | remapped_axis->z;

        /* If all the axes are re-mapped */
        if ((remap_axes & BMI088_ANYMOTION_AXIS_MASK) == BMI088_ANYMOTION_AXIS_MASK)
        {
            /* Store the value of re-mapped in device structure */
            assign_remap_axis(remapped_axis->x, &dev->remap.x_axis, &dev->remap.x_axis_sign);
            assign_remap_axis(remapped_axis->y, &dev->remap.y_axis, &dev->remap.y_axis_sign);
            assign_remap_axis(remapped_axis->z, &dev->remap.z_axis, &dev->remap.z_axis_sign);

            /* Set the re-mapped axes in the sensor */
            rslt = set_remap_axes(&dev->remap, dev);
        }
        else
        {
            rslt = BMI088_ANYMOTION_E_REMAP_ERROR;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to get the config file major and minor information.
 */
int8_t bmi088_anymotion_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bmi08_dev *dev)
{
    /* Initialize configuration file */
    uint8_t feature_config[BMI088_ANYMOTION_FEATURE_SIZE] = { 0 };

    /* Update index to config file version */
    uint8_t index = BMI088_ANYMOTION_ADDR_CONFIG_ID_START;

    /* Variable to define LSB */
    uint8_t lsb = 0;

    /* Variable to define MSB */
    uint8_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    /* Result of api are returned to this variable */
    int8_t rslt;

    if ((config_major != NULL) && (config_minor != NULL))
    {
        /* Get config file identification from the sensor */
        rslt = bmi08a_get_regs(BMI08_REG_ACCEL_FEATURE_CFG,
                               (uint8_t *)feature_config,
                               BMI088_ANYMOTION_FEATURE_SIZE,
                               dev);

        if (rslt == BMI08_OK)
        {
            /* Get word to calculate config file identification */
            lsb = feature_config[index++];
            msb = feature_config[index++];

            lsb_msb = (uint16_t)(msb << 8 | lsb);

            /* Get major and minor version */
            *config_major = BMI08_GET_BITS(lsb_msb, BMI088_ANYMOTION_CONFIG_MAJOR);
            *config_minor = BMI08_GET_BITS_POS_0(lsb, BMI088_ANYMOTION_CONFIG_MINOR);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self-test functionality of the sensor
 *  is working or not.
 */
int8_t bmi088_anymotion_perform_selftest(struct bmi08_dev *dev)
{
    int8_t rslt;
    int8_t self_test_rslt = 0;
    struct bmi08_sensor_data accel_pos, accel_neg;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Pre-requisites for self-test */
        rslt = enable_self_test(dev);

        if (rslt == BMI08_OK)
        {
            rslt = positive_excited_accel(&accel_pos, dev);

            if (rslt == BMI08_OK)
            {
                rslt = negative_excited_accel(&accel_neg, dev);

                if (rslt == BMI08_OK)
                {
                    /* Validate the self-test result */
                    rslt = validate_accel_self_test(&accel_pos, &accel_neg);

                    /* Store the status of self-test result */
                    self_test_rslt = rslt;

                    /* Perform soft-reset */
                    rslt = bmi08a_soft_reset(dev);

                    /* Check to ensure bus operations are success */
                    if (rslt == BMI08_OK)
                    {
                        /* Restore self_test_rslt as return value */
                        rslt = self_test_rslt;
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API gets accel feature interrupt status
 */
int8_t bmi088_anymotion_get_feat_int_status(uint8_t *int_status, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t status = 0;

    if (int_status != NULL)
    {
        rslt = bmi08a_get_regs(BMI08_REG_ACCEL_INT_STAT_0, &status, 1, dev);
        if (rslt == BMI08_OK)
        {
            (*int_status) = status;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi08_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL) ||
        (dev->intf_ptr_accel == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI08_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI08_OK;
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0, data, is_channel_invalid = FALSE;

    switch (int_config->int_channel)
    {
        case BMI08_INT_CHANNEL_1:

            /* Update reg_addr based on channel inputs */
            reg_addr = BMI08_REG_ACCEL_INT1_IO_CONF;
            break;
        case BMI08_INT_CHANNEL_2:

            /* Update reg_addr based on channel inputs */
            reg_addr = BMI08_REG_ACCEL_INT2_IO_CONF;
            break;
        default:
            is_channel_invalid = TRUE;
            break;
    }

    if (!is_channel_invalid)
    {
        /* Read interrupt pin configuration register */
        rslt = bmi08a_get_regs(reg_addr, &data, 1, dev);

        if (rslt == BMI08_OK)
        {
            /* Update data with user configured bmi088_anymotion_int_cfg structure */
            data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_LVL, int_config->int_pin_cfg.lvl);
            data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_OD, int_config->int_pin_cfg.output_mode);

            data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_IO, int_config->int_pin_cfg.enable_int_pin);
            data = BMI08_SET_BIT_VAL_0(data, BMI08_ACCEL_INT_IN);

            /* Write to interrupt pin configuration register */
            rslt = bmi08a_set_regs(reg_addr, &data, 1, dev);
        }
    }
    else
    {
        rslt = BMI08_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 */
static int8_t set_accel_anymotion_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BMI08_OK)
    {
        data = BMI088_ANYMOTION_ACCEL_ANY_MOT_INT_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_1:
                reg_addr = BMI08_REG_ACCEL_INT1_MAP;
                break;

            case BMI08_INT_CHANNEL_2:
                reg_addr = BMI08_REG_ACCEL_INT2_MAP;
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            rslt = bmi08a_get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI08_ENABLE)
            {
                /* Interrupt B mapped to INT1/INT2 */
                data |= BMI088_ANYMOTION_ACCEL_ANY_MOT_INT_ENABLE;
            }
            else
            {
                data &= ~BMI088_ANYMOTION_ACCEL_ANY_MOT_INT_ENABLE;
            }

            if (rslt == BMI08_OK)
            {
                /* Write to interrupt map register */
                rslt = bmi08a_set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI08_OK)
            {
                /* Set input interrupt configuration */
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets error interrupt for accel sensor
 */
static int8_t set_accel_err_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BMI08_OK)
    {
        data = BMI088_ANYMOTION_ACCEL_ERR_INT_DISABLE;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_1:
                reg_addr = BMI08_REG_ACCEL_INT1_MAP;
                break;

            case BMI08_INT_CHANNEL_2:
                reg_addr = BMI08_REG_ACCEL_INT2_MAP;
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            rslt = bmi08a_get_regs(reg_addr, &data, 1, dev);

            if (int_config->int_pin_cfg.enable_int_pin == BMI08_ENABLE)
            {
                /* Interrupt B mapped to INT1/INT2 */
                data |= BMI088_ANYMOTION_ACCEL_ERR_INT_ENABLE;
            }
            else
            {
                data &= ~BMI088_ANYMOTION_ACCEL_ERR_INT_ENABLE;
            }

            if (rslt == BMI08_OK)
            {
                /* Write to interrupt map register */
                rslt = bmi08a_set_regs(reg_addr, &data, 1, dev);
            }

            if (rslt == BMI08_OK)
            {
                /* Set input interrupt configuration */
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API performs x, y and z-axis re-mapping in the sensor.
 */
static int8_t set_remap_axes(const struct bmi08_axes_remap *remap_data, struct bmi08_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMI088_ANYMOTION_FEATURE_SIZE] = { 0 };

    /* Initialize index to set re-mapped data */
    uint8_t index = BMI088_ANYMOTION_ADDR_AXES_REMAP_START;

    /* Variable to define x-axis to be re-mapped */
    uint8_t x_axis;

    /* Variable to define y-axis to be re-mapped */
    uint8_t y_axis;

    /* Variable to define z-axis to be re-mapped */
    uint8_t z_axis;

    /* Variable to define x-axis sign to be re-mapped */
    uint8_t x_axis_sign;

    /* Variable to define y-axis sign to be re-mapped */
    uint8_t y_axis_sign;

    /* Variable to define z-axis sign to be re-mapped */
    uint8_t z_axis_sign;

    if (remap_data != NULL)
    {
        /* Read the configuration file */
        rslt = bmi08a_get_regs(BMI08_REG_ACCEL_FEATURE_CFG, feature_config, BMI088_ANYMOTION_FEATURE_SIZE, dev);

        if (rslt == BMI08_OK)
        {
            /* Get x-axis to be re-mapped */
            x_axis = remap_data->x_axis & BMI088_ANYMOTION_X_AXIS_MASK;

            /* Get x-axis sign to be re-mapped */
            x_axis_sign = (remap_data->x_axis_sign << BMI088_ANYMOTION_X_AXIS_SIGN_POS) &
                          BMI088_ANYMOTION_X_AXIS_SIGN_MASK;

            /* Get y-axis to be re-mapped */
            y_axis = (remap_data->y_axis << BMI088_ANYMOTION_Y_AXIS_POS) & BMI088_ANYMOTION_Y_AXIS_MASK;

            /* Get y-axis sign to be re-mapped */
            y_axis_sign = (remap_data->y_axis_sign << BMI088_ANYMOTION_Y_AXIS_SIGN_POS) &
                          BMI088_ANYMOTION_Y_AXIS_SIGN_MASK;

            /* Get z-axis to be re-mapped */
            z_axis = (remap_data->z_axis << BMI088_ANYMOTION_Z_AXIS_POS) & BMI088_ANYMOTION_Z_AXIS_MASK;

            /* Get z-axis sign to be re-mapped */
            z_axis_sign = remap_data->z_axis_sign & BMI088_ANYMOTION_Z_AXIS_SIGN_MASK;

            /* Set the first byte for axis re-mapping */
            feature_config[index] = x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis;

            /* Set the second byte for axis re-mapping */
            feature_config[index + 1] = z_axis_sign;

            /* Set the re-mapped axes */
            rslt = bmi08a_set_regs(BMI08_REG_ACCEL_FEATURE_CFG, feature_config, BMI088_ANYMOTION_FEATURE_SIZE, dev);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the x, y and z axis re-mapped data from the sensor.
 */
static int8_t get_remap_axes(struct bmi08_axes_remap *remap_data, struct bmi08_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Initialize configuration file */
    uint8_t feature_config[BMI088_ANYMOTION_FEATURE_SIZE] = { 0 };

    /* Initialize index to get re-mapped data */
    uint8_t index = BMI088_ANYMOTION_ADDR_AXES_REMAP_START;

    if (remap_data != NULL)
    {
        /* Read the configuration file */
        rslt = bmi08a_get_regs(BMI08_REG_ACCEL_FEATURE_CFG, feature_config, BMI088_ANYMOTION_FEATURE_SIZE, dev);

        if (rslt == BMI08_OK)
        {
            /* Get re-mapped x-axis */
            remap_data->x_axis = BMI08_GET_BITS_POS_0(feature_config[index], BMI088_ANYMOTION_X_AXIS);

            /* Get re-mapped x-axis sign */
            remap_data->x_axis_sign = BMI08_GET_BITS(feature_config[index], BMI088_ANYMOTION_X_AXIS_SIGN);

            /* Get re-mapped y-axis */
            remap_data->y_axis = BMI08_GET_BITS(feature_config[index], BMI088_ANYMOTION_Y_AXIS);

            /* Get re-mapped y-axis sign */
            remap_data->y_axis_sign = BMI08_GET_BITS(feature_config[index], BMI088_ANYMOTION_Y_AXIS_SIGN);

            /* Get re-mapped z-axis */
            remap_data->z_axis = BMI08_GET_BITS(feature_config[index], BMI088_ANYMOTION_Z_AXIS);

            /* Get re-mapped z-axis sign */
            remap_data->z_axis_sign = BMI08_GET_BITS_POS_0(feature_config[index + 1], BMI088_ANYMOTION_Z_AXIS_SIGN);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets the re-mapped accelerometer.
 */
static void get_remapped_data(struct bmi08_sensor_data *data, const struct bmi08_dev *dev)
{
    /* Array to defined the re-mapped sensor data */
    int16_t remap_data[3] = { 0 };
    int16_t pos_multiplier = INT16_C(1);
    int16_t neg_multiplier = INT16_C(-1);

    /* Fill the array with the un-mapped sensor data */
    remap_data[0] = data->x;
    remap_data[1] = data->y;
    remap_data[2] = data->z;

    /* Get the re-mapped x axis data */
    if (dev->remap.x_axis_sign == BMI088_ANYMOTION_MAP_POSITIVE)
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * pos_multiplier);
    }
    else
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * neg_multiplier);
    }

    /* Get the re-mapped y axis data */
    if (dev->remap.y_axis_sign == BMI088_ANYMOTION_MAP_POSITIVE)
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * pos_multiplier);
    }
    else
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * neg_multiplier);
    }

    /* Get the re-mapped z axis data */
    if (dev->remap.z_axis_sign == BMI088_ANYMOTION_MAP_POSITIVE)
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * pos_multiplier);
    }
    else
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * neg_multiplier);
    }
}

/*!
 * @brief This internal API is to store remapped axis and sign values
 * in device structure
 */
static void assign_remap_axis(uint8_t remap_axis, uint8_t *axis, uint8_t *sign)
{
    /* Variable to store the re-mapped axis value */
    uint8_t axis_val = remap_axis & BMI088_ANYMOTION_AXIS_MASK;

    switch (axis_val)
    {
        case BMI088_ANYMOTION_X:

            /* If mapped to x-axis */
            (*axis) = BMI088_ANYMOTION_MAP_X_AXIS;
            break;
        case BMI088_ANYMOTION_Y:

            /* If mapped to y-axis */
            (*axis) = BMI088_ANYMOTION_MAP_Y_AXIS;
            break;
        case BMI088_ANYMOTION_Z:

            /* If mapped to z-axis */
            (*axis) = BMI088_ANYMOTION_MAP_Z_AXIS;
            break;
        default:
            break;
    }

    /* Store the re-mapped axis sign in the device structure */
    if (remap_axis & BMI088_ANYMOTION_AXIS_SIGN)
    {
        /* If axis mapped to negative sign */
        (*sign) = BMI088_ANYMOTION_MAP_NEGATIVE;
    }
    else
    {
        /* If axis mapped to positive sign */
        (*sign) = BMI088_ANYMOTION_MAP_POSITIVE;
    }
}

/*!
 * @brief This internal API is to receive remapped axis and sign values
 * in device structure and to local structure
 */
static void receive_remap_axis(uint8_t remap_axis, uint8_t remap_sign, uint8_t *axis)
{
    /* Get the re-mapped axis value from device structure */
    switch (remap_axis)
    {
        case BMI088_ANYMOTION_MAP_X_AXIS:

            /* If mapped to x-axis */
            (*axis) = BMI088_ANYMOTION_X;
            break;
        case BMI088_ANYMOTION_MAP_Y_AXIS:

            /* If mapped to y-axis */
            (*axis) = BMI088_ANYMOTION_Y;
            break;
        case BMI088_ANYMOTION_MAP_Z_AXIS:

            /* If mapped to z-axis */
            (*axis) = BMI088_ANYMOTION_Z;
            break;
        default:
            break;
    }

    /* Get the re-mapped axis sign from device structure */
    if (remap_sign)
    {
        /* If axis is mapped to negative sign */
        (*axis) |= BMI088_ANYMOTION_AXIS_SIGN;
    }
}

/*!
 * @brief This API performs the pre-requisites needed to perform the self-test
 */
static int8_t enable_self_test(struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Configuring sensors to perform accel self-test */
    dev->accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
    dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;

    /* Check the chip id of the accel variant and assign the range */

    dev->accel_cfg.range = BMI088_ANYMOTION_ACCEL_RANGE_24G;

    dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    /* Enable Accel sensor */
    rslt = bmi08a_set_power_mode(dev);
    if (rslt == BMI08_OK)
    {
        /* Configure sensors with above configured settings */
        rslt = bmi088_anymotion_set_meas_conf(dev);

        if (rslt == BMI08_OK)
        {
            /* Self-test delay */
            dev->delay_us(BMI08_SELF_TEST_DELAY_MS * 1000, dev->intf_ptr_accel);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the positive excitation
 */
static int8_t positive_excited_accel(struct bmi08_sensor_data *accel_pos, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMI08_ACCEL_POSITIVE_SELF_TEST;

    /* Enable positive excitation for all 3 axes */
    rslt = bmi08a_set_regs(BMI08_REG_ACCEL_SELF_TEST, &reg_data, 1, dev);
    if (rslt == BMI08_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_us(BMI08_SELF_TEST_DATA_READ_MS * 1000, dev->intf_ptr_accel);
        rslt = bmi088_anymotion_get_data(accel_pos, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the negative excitation
 */
static int8_t negative_excited_accel(struct bmi08_sensor_data *accel_neg, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMI08_ACCEL_NEGATIVE_SELF_TEST;

    /* Enable negative excitation for all 3 axes */
    rslt = bmi08a_set_regs(BMI08_REG_ACCEL_SELF_TEST, &reg_data, 1, dev);
    if (rslt == BMI08_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_us(BMI08_SELF_TEST_DATA_READ_MS * 1000, dev->intf_ptr_accel);
        rslt = bmi088_anymotion_get_data(accel_neg, dev);

        if (rslt == BMI08_OK)
        {
            /* Disable self-test */
            reg_data = BMI08_ACCEL_SWITCH_OFF_SELF_TEST;
            rslt = bmi08a_set_regs(BMI08_REG_ACCEL_SELF_TEST, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API validates the self-test results
 */
static int8_t validate_accel_self_test(const struct bmi08_sensor_data *accel_pos,
                                       const struct bmi08_sensor_data *accel_neg)
{
    int8_t rslt;

    /*! Structure for difference of accel values in g */
    struct bmi088_anymotion_selftest_delta_limit accel_data_diff = { 0 };

    /*! Structure for difference of accel values in mg */
    struct bmi088_anymotion_selftest_delta_limit accel_data_diff_mg = { 0 };

    accel_data_diff.x = (BMI08_ABS(accel_pos->x - accel_neg->x));
    accel_data_diff.y = (BMI08_ABS(accel_pos->y - accel_neg->y));
    accel_data_diff.z = (BMI08_ABS(accel_pos->z - accel_neg->z));

    /*! Converting LSB of the differences of accel values to mg */
    convert_lsb_g(&accel_data_diff, &accel_data_diff_mg);

    /* Validating accel data by comparing with minimum value of the axes in mg */
    /* x axis limit 1000mg, y axis limit 1000mg and z axis limit 500mg */
    if (accel_data_diff_mg.x >= BMI088_ANYMOTION_ST_ACC_X_AXIS_SIGNAL_DIFF &&
        accel_data_diff_mg.y >= BMI088_ANYMOTION_ST_ACC_Y_AXIS_SIGNAL_DIFF &&
        accel_data_diff_mg.z >= BMI088_ANYMOTION_ST_ACC_Z_AXIS_SIGNAL_DIFF)
    {
        /* Updating Okay status */
        rslt = BMI08_OK;
    }
    else
    {
        /* Updating Error status */
        rslt = BMI08_E_SELF_TEST_FAIL;
    }

    return rslt;
}

/*!
 *  @brief This API converts lsb value of axes to mg for self-test.
 */
static void convert_lsb_g(const struct bmi088_anymotion_selftest_delta_limit *accel_data_diff,
                          struct bmi088_anymotion_selftest_delta_limit *accel_data_diff_mg)
{
    /* Accel x value in mg */
    accel_data_diff_mg->x = (int16_t) ((accel_data_diff->x / (int32_t)LSB_PER_G) * 1000);

    /* Accel y value in mg */
    accel_data_diff_mg->y = (int16_t) ((accel_data_diff->y / (int32_t)LSB_PER_G) * 1000);

    /* Accel z value in mg */
    accel_data_diff_mg->z = (int16_t) ((accel_data_diff->z / (int32_t)LSB_PER_G) * 1000);
}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static int8_t set_accel_data_ready_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT1_DRDY, conf);
                break;

            case BMI08_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT2_DRDY, conf);
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI08_OK)
            {
                /* Write to interrupt map register */
                rslt = bmi08a_set_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water mark interrupt for accel sensor.
 */
static int8_t set_fifo_wm_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT1_FWM, conf);
                break;

            case BMI08_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT2_FWM, conf);
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI08_OK)
            {
                /* Write to interrupt map register */
                rslt = bmi08a_set_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev)
{
    int8_t rslt = BMI08_OK;
    uint16_t index;
    uint8_t temp_buff[BMI08_MAX_LEN];

    if (dev->intf == BMI08_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = reg_addr | BMI08_SPI_RD_MASK;
    }

    /* Read the data from the register */
    dev->intf_rslt = dev->read(reg_addr, temp_buff, (len + dev->dummy_byte), dev->intf_ptr_accel);

    if (dev->intf_rslt == BMI08_INTF_RET_SUCCESS)
    {
        for (index = 0; index < len; index++)
        {
            /* Updating the data buffer */
            reg_data[index] = temp_buff[index + dev->dummy_byte];
        }
    }
    else
    {
        /* Failure case */
        rslt = BMI08_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor.
 */
static int8_t set_fifo_full_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_1:

                /* Updating the data */
                data = BMI08_SET_BITS_POS_0(data, BMI08_ACCEL_INT1_FFULL, conf);
                break;

            case BMI08_INT_CHANNEL_2:

                /* Updating the data */
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT2_FFULL, conf);
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == BMI08_OK)
            {
                /* Write to interrupt map register */
                rslt = bmi08a_set_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*! @endcond */

/** @}*/
