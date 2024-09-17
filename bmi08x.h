/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bmi08x.h
* @date       2024-07-29
* @version    v1.9.0
*
*/

/*! \file bmi08x.h
 * \brief Sensor Driver for BMI08x family of sensors */

/**
 * \ingroup bmi08x
 * \defgroup bmi08ag BMI08A / BMI08G
 */

#ifndef _BMI08X_H
#define _BMI08X_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/*                          Header Files                             */
/*********************************************************************/
#include "bmi08.h"

/*********************************************************************/
/*                     Macro Definitions                             */
/*********************************************************************/

/**\name    BMI085 Accel Range */
#define BMI085_ACCEL_RANGE_2G   UINT8_C(0x00)
#define BMI085_ACCEL_RANGE_4G   UINT8_C(0x01)
#define BMI085_ACCEL_RANGE_8G   UINT8_C(0x02)
#define BMI085_ACCEL_RANGE_16G  UINT8_C(0x03)

/**\name  BMI088 Accel Range */
#define BMI088_ACCEL_RANGE_3G   UINT8_C(0x00)
#define BMI088_ACCEL_RANGE_6G   UINT8_C(0x01)
#define BMI088_ACCEL_RANGE_12G  UINT8_C(0x02)
#define BMI088_ACCEL_RANGE_24G  UINT8_C(0x03)

/**\name    BMI085 Accel unique chip identifier */
#define BMI085_ACCEL_CHIP_ID    UINT8_C(0x1F)

/**\name    BMI088 Accel unique chip identifier */
#define BMI088_ACCEL_CHIP_ID    UINT8_C(0x1E)

/*********************** BMI08x Accelerometer function prototypes ************************/

/**
 * \ingroup bmi08xag
 * \defgroup bmi08xaApiInit Gyro Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bmi08xaApiInit
 * \page bmi08xa_api_bmi08xa_init bmi08xa_init
 * \code
 * int8_t bmi08xa_init(struct bmi08_dev *dev);
 * \endcode
 * @details This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 *
 *  @param[in,out] dev  : Structure instance of bmi08_dev.
 *  @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
int8_t bmi08xa_init(struct bmi08_dev *dev);

/**
 * \ingroup bmi08xag
 * \defgroup bmi08xaApiConf Read accel configurations
 * @brief Read / Write configurations of accel sensor
 */

/*!
 * \ingroup bmi08xaApiConf
 * \page bmi08xa_api_bmi08xa_set_meas_conf bmi08xa_set_meas_conf
 * \code
 * int8_t bmi08xa_set_meas_conf(const struct bmi08_dev *dev);
 * \endcode
 * @details This API sets the Output data rate, range and bandwidth
 *  of accel sensor.
 *  @param[in] dev  : Structure instance of bmi08_dev.
 *
 *  @note : The user must select one among the following macros to
 *  select range value for BMI085 accel
 *
 *@verbatim
 *      config                         |   value
 *      -------------------------------|---------------------------
 *      BMI085_ACCEL_RANGE_2G          |   0x00
 *      BMI085_ACCEL_RANGE_4G          |   0x01
 *      BMI085_ACCEL_RANGE_8G          |   0x02
 *      BMI085_ACCEL_RANGE_16G         |   0x03
 *@endverbatim
 *
 *  @note : The user must select one among the following macros to
 *  select range value for BMI088 accel
 *
 *@verbatim
 *      config                         |   value
 *      -------------------------------|---------------------------
 *      BMI088_ACCEL_RANGE_3G          |   0x00
 *      BMI088_ACCEL_RANGE_6G          |   0x01
 *      BMI088_ACCEL_RANGE_12G         |   0x02
 *      BMI088_ACCEL_RANGE_24G         |   0x03
 *@endverbatim
 *
 *  @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
int8_t bmi08xa_set_meas_conf(struct bmi08_dev *dev);

/**
 * \ingroup bmi08xag
 * \defgroup bmi08xaApiSelftest Accel Self test
 * @brief Perform self test of accel sensor
 */

/*!
 * \ingroup bmi08xaApiSelftest
 * \page bmi08xa_api_bmi08xa_perform_selftest bmi08xa_perform_selftest
 * \code
 * int8_t bmi08xa_perform_selftest(struct bmi08_dev *dev);
 * \endcode
 * @details This API checks whether the self test functionality of the accel sensor
 *  is working or not
 *
 *  @param[in] dev    : Structure instance of bmi08_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *  @retval > 0 -> Warning
 *
 */
int8_t bmi08xa_perform_selftest(struct bmi08_dev *dev);

/**
 * \ingroup bmi08xag
 * \defgroup bmi08xaApiSync Data Synchronization
 * @brief Enable / Disable data synchronization
 */

/*!
 * \ingroup bmi08xaApiSync
 * \page bmi08xa_api_bmi08xa_configure_data_synchronization bmi08xa_configure_data_synchronization
 * \code
 * int8_t bmi08xa_configure_data_synchronization(struct bmi08_data_sync_cfg sync_cfg, struct bmi08_dev *dev);
 * \endcode
 * @details This API is used to enable/disable the data synchronization
 *  feature.
 *
 *  @param[in] sync_cfg : Configure sync feature
 *  @param[in] dev : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
int8_t bmi08xa_configure_data_synchronization(struct bmi08_data_sync_cfg sync_cfg, struct bmi08_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* _BMI08X_H */
