/**
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file       bmi08x.h
 * @date       25 Sep 2019
 * @version    1.4.0
 *
 */

/*! \file bmi08x.h
 * \brief Sensor Driver for BMI08x family of sensors */
#ifndef BMI08X_H_
#define BMI08X_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* header files */
#include "bmi08x_defs.h"

/*********************************************************************/
/* (extern) variable declarations */
/*********************************************************************/
/* function prototype declarations */
/*********************** BMI08x Accelerometer function prototypes ************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 *
 *  @param[in,out] dev  : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success /-ve value -> Error
 */
int8_t bmi08a_init(struct bmi08x_dev *dev);

/*!
 *  @brief This API is used to write the binary configuration in the sensor
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_write_config_file(const struct bmi08x_dev *dev);

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 *
 *  @param[in] reg_addr : Address offset of the feature setting inside the feature conf region.
 *  @param[in] reg_data : Feature settings.
 *  @param[in] len : Number of 16 bit words to be written.
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_write_feature_config(uint8_t reg_addr, const uint16_t *reg_data, uint8_t len,
                                   const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the error status from the accel sensor.
 *
 *  Below table mention the types of error which can occur in the sensor
 *@verbatim
 *************************************************************************
 *        Error           |       Description
 *************************|***********************************************
 *                        |       Fatal Error, chip is not in operational
 *        fatal           |       state (Boot-, power-system).
 *                        |       This flag will be reset only by
 *                        |       power-on-reset or soft reset.
 *************************|***********************************************
 *                        |       Value        Name       Description
 *        error_code      |       000        no_error     no error
 *                        |       001        accel_err      error in
 *                        |                               ACCEL_CONF
 *************************************************************************
 *@endverbatim
 *  @param[out] err_reg : Pointer to structure variable which stores the
 *  error status read from the sensor.
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_get_error_status(struct bmi08x_err_reg *err_reg, const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the status of the accel sensor.
 *
 *  Below table lists the sensor status flags
 *@verbatim
 *************************************************************************
 *        Status                    |       Description
 ***********************************|*************************************
 *        drdy_accel                | Data ready for Accel.
 *************************************************************************
 *@endverbatim
 *
 *  @param[out] status : Variable used to store the sensor status flags
 *  which is read from the sensor.
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t bmi08a_get_status(uint8_t *status, const struct bmi08x_dev *dev);

/*!
 *  @brief This API resets the accel sensor.
 *
 *  @param[in] dev  : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_soft_reset(const struct bmi08x_dev *dev);

/*!
 * @brief This API reads the accel config values ie odr, band width and range from the sensor,
 * store it in the bmi08x_dev structure instance
 * passed by the user.
 *  @param[in,out]  dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t bmi08a_get_meas_conf(struct bmi08x_dev *dev);

/*!
 *  @brief This API sets the Output data rate, range and bandwidth
 *  of accel sensor.
 *  @param[in] dev  : Structure instance of bmi08x_dev.
 *
 *  @note : The user must select one among the following macros to
 *  select range value for BMI085 accel
 *      config                         |   value
 *      -------------------------------|---------------------------
 *      BMI085_ACCEL_RANGE_2G          |   0x00
 *      BMI085_ACCEL_RANGE_4G          |   0x01
 *      BMI085_ACCEL_RANGE_8G          |   0x02
 *      BMI085_ACCEL_RANGE_16G         |   0x03
 *
 *  @note : The user must select one among the following macros to
 *  select range value for BMI088 accel
 *      config                         |   value
 *      -------------------------------|---------------------------
 *      BMI088_ACCEL_RANGE_3G          |   0x00
 *      BMI088_ACCEL_RANGE_6G          |   0x01
 *      BMI088_ACCEL_RANGE_12G         |   0x02
 *      BMI088_ACCEL_RANGE_24G         |   0x03
 *
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_set_meas_conf(const struct bmi08x_dev *dev);

/*!
 * @brief This API reads the accel power mode from the sensor,
 * store it in the bmi08x_dev structure instance
 * passed by the user.
 *  @param[in,out]  dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t bmi08a_get_power_mode(struct bmi08x_dev *dev);

/*!
 *  @brief This API sets the power mode of the accel sensor.
 *
 *  @param[in] dev  : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_set_power_mode(const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the accel data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[in]  dev    : Structure instance of bmi08x_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success /-ve value -> Error
 */
int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel, const struct bmi08x_dev *dev);

/*!
 *  @brief This API configures the necessary accel interrupt
 *  based on the user settings in the bmi08x_accel_int_channel_cfg
 *  structure instance.
 *
 *  @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 *  @param[in] dev         : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_set_int_config(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the temperature of the sensor in degree Celcius.
 *
 *  @param[in]  dev             : Structure instance of bmi08x_dev.
 *  @param[out] sensor_temp     : Pointer to store sensor temperature in degree Celcius
 *
 *  @note Temperature data output must be divided by a factor of 1000
 *
 *  Consider sensor_temp = 19520 , Then the actual temperature is 19.520 degree Celsius
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_get_sensor_temperature(const struct bmi08x_dev *dev, int32_t *sensor_temp);

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 *
 *  @param[in]  dev             : Structure instance of bmi08x_dev.
 *  @param[out] sensor_time     : Pointer to store sensor time
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08a_get_sensor_time(const struct bmi08x_dev *dev, uint32_t *sensor_time);

/*!
 *  @brief This API checks whether the self test functionality of the sensor
 *  is working or not
 *
 *  @param[in] dev    : Structure instance of bmi08x_dev
 *
 *  @return results of self test
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  BMI08X_OK                      |  Self test success
 *  BMI08X_W_SELF_TEST_FAIL        |  self test fail
 */
int8_t bmi08a_perform_selftest(struct bmi08x_dev *dev);

/*********************** BMI088 Gyroscope function prototypes ****************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 *
 *  @param[in,out] dev : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_init(struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 *  @brief This API resets the gyro sensor.
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t bmi08g_soft_reset(const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the gyro odr and range from the sensor,
 *  store it in the bmi08x_dev structure instance
 *  passed by the user.
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 *  @note : band width also updated, which is same as odr
 *          Refer user guide for detailed info.
 */
int8_t bmi08g_get_meas_conf(struct bmi08x_dev *dev);

/*!
 *  @brief This API sets the output data rate, range and bandwidth
 *  of gyro sensor.
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 *  @note : No need to give the band width parameter,
 *          odr will update the band width.
 *          Refer user guide for detailed info.
 */
int8_t bmi08g_set_meas_conf(const struct bmi08x_dev *dev);

/*!
 *  @brief This API gets the power mode of the gyro sensor and store it
 *  inside the instance of bmi08x_dev passed by the user.
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_get_power_mode(struct bmi08x_dev *dev);

/*!
 *  @brief This API sets the power mode of the gyro sensor.
 *
 *  @param[in] dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_set_power_mode(const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the gyro data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] gyro   : Structure pointer to store gyro data
 *  @param[in] dev     : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro, const struct bmi08x_dev *dev);

/*!
 *  @brief This API configures the necessary gyro interrupt
 *  based on the user settings in the bmi08x_gyro_int_channel_cfg
 *  structure instance.
 *
 *  @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 *  @param[in] dev         : Structure instance of bmi08x_dev.
 *  @note : Refer user guide for detailed info.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t bmi08g_set_int_config(const struct bmi08x_gyro_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not
 *
 *  @param[in]  dev : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  BMI08X_OK                      |  Self test success
 *  BMI08X_W_SELF_TEST_FAIL        |  self test fail
 */
int8_t bmi08g_perform_selftest(const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 *
 * @param[in] config        : Structure instance of FIFO configurations.
 * @param[in] dev           : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_set_fifo_config(const struct accel_fifo_config *config, const struct bmi08x_dev *dev);

/*!
 * @brief This API gets the FIFO configuration from the sensor.
 *
 * @param[out] config   : Structure instance to get FIFO configuration value.
 * @param[in]  dev      : Structure instance of bmi08x_dev.
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_get_fifo_config(struct accel_fifo_config *config, const struct bmi08x_dev *dev);

/*!
 * @brief This API reads FIFO data.
 *
 * @param[in, out] fifo     : Structure instance of bmi08x_fifo_frame.
 * @param[in]      dev      : Structure instance of bmi08x_dev.
 *
 * @note APS has to be disabled before calling this function.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_read_fifo_data(struct bmi08x_fifo_frame *fifo, const struct bmi08x_dev *dev);

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 *
 * @param[out] fifo_length  : Pointer variable to store the value of FIFO byte
 *                            counter.
 * @param[in]  dev          : Structure instance of bmi08x_dev.
 *
 * @note The byte counter is updated each time a complete frame is read or
 * written.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_get_fifo_length(uint16_t *fifo_length, const struct bmi08x_dev *dev);

/*!
 * @brief This API gets the FIFO water mark level which is set in the sensor.
 *
 * @param[out] wm        : Pointer variable to store FIFO water-mark level.
 * @param[in]  dev            : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_get_fifo_wm(uint16_t *wm, const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the FIFO water mark level which is set in the sensor.
 *
 * @param[out] wm        : Pointer variable to store FIFO water-mark level.
 * @param[in]  dev            : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */
int8_t bmi08a_set_fifo_wm(uint16_t wm, const struct bmi08x_dev *dev);

/*!
 * This API parses and extracts the accelerometer frames from FIFO data read by
 * the "bmi08x_read_fifo_data" API and stores it in the "accel_data" structure
 * instance.
 *
 * @param[out]    accel_data   : Structure instance of bmi08x_sensor_data
 *                               where the parsed data bytes are stored.
 * @param[in,out] accel_length : Number of accelerometer frames.
 * @param[in,out] fifo         : Structure instance of bmi08x_fifo_frame.
 * @param[in]     dev          : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval BMI08X_W_PARTIAL_READ - Warning : There are more frames to be read
 */
int8_t bmi08a_extract_accel(struct bmi08x_sensor_data *accel_data,
                            uint16_t *accel_length,
                            struct bmi08x_fifo_frame *fifo,
                            const struct bmi08x_dev *dev);

/*!
 * @brief This API gets the down sampling rate, configured for FIFO
 * accelerometer.
 *
 * @param[out] fifo_downs : Pointer variable to store the down sampling rate
 * @param[in]  dev            : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 */

int8_t bmi08a_get_fifo_down_sample(uint8_t *fifo_downs, const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the down sampling rate for FIFO accelerometer FIFO data.
 *
 * @param[in] fifo_downs : Variable to set the down sampling rate.
 * @param[in] dev            : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI08X_OK - Success.
 * @retval BMI08X_E_NULL_PTR - Error: Null pointer error
 * @retval BMI08X_E_COM_FAIL - Error: Communication fail
 * @retval BMI08X_E_OUT_OF_RANGE - Error: Out of range
 */
int8_t bmi08a_set_fifo_down_sample(uint8_t fifo_downs, const struct bmi08x_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI08X_H_ */

/** @}*/
