/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef COMMON_H
#define COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "coines.h"
#include "bmi08_defs.h"

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMI08_INTF_RET_SUCCESS -> Success
 *  @retval != BMI08_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bma      : Structure instance of bmi08_dev
 *  @param[in] intf     : Interface selection parameter
 *                          For I2C : BMI08_I2C_INTF
 *                          For SPI : BMI08_SPI_INTF
 *  @param[in] variant  : Sensor variant parameter
 *                          For BMI085 : BMI085_VARIANT
 *                          For BMI088 : BMI088_VARIANT
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi08_interface_init(struct bmi08_dev *bma, uint8_t intf, uint8_t variant);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi08_error_codes_print_result(const char api_name[], int8_t rslt);

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bmi08_coines_deinit(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* COMMON_H */
