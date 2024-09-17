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
 * @file       bmi08a.c
 * @date       2024-07-29
 * @version    v1.9.0
 *
 */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"

/*********************** Function prototypes ************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t dev_null_ptr_check(const struct bmi08_dev *dev);

/*!
 * @brief This API is used to validate the variable for null conditions.
 *
 * @param[in] data_parm : void pointer to hold address of variables.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t generic_null_ptr_check(void *data_parm);

/*!
 *  @brief This API writes or reads the data to/from the given register address.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read or write.
 *  @param[out] reg_data : Pointer to data buffer.
 *  @param[in] len       : No. of bytes of data to be read or write.
 *  @param[in] dev       : Structure instance of bmi08_dev.
 *  @param[in] select    : Flag to select functionality between getting and setting
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev, uint8_t select);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_int_pin_config(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

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
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi08_accel_int_channel_cfg *int_config,
                                            struct bmi08_dev *dev);

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_accel_sync_input(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API writes the config stream data in memory using burst mode
 *
 * @param[in] stream_data : Pointer to store data of 32 bytes
 * @param[in] index       : Represents value in multiple of 32 bytes
 * @param[in] dev         : Structure instance of bmi08_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bmi08_dev *dev);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data.
 *
 * @param[out] acc              : Structure instance of bmi08_sensor_data
 *                                where the parsed data bytes are stored.
 * @param[in]  data_start_index : Index value of the accelerometer data bytes
 *                                which is to be parsed from the FIFO data.
 * @param[in]  fifo             : Structure instance of bmi08_fifo_frame.
 *
 * @return None
 * @retval None
 *
 */
static void unpack_accel_data(struct bmi08_sensor_data *acc,
                              uint16_t data_start_index,
                              const struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in both header and header-less mode. It updates the current data
 * byte to be parsed.
 *
 * @param[in,out] acc       : Structure instance of bmi08_sensor_data where
 *                            where the parsed data bytes are stored.
 * @param[in,out] idx       : Index value of number of bytes parsed.
 * @param[in,out] acc_idx   : Index value of accelerometer data (x,y,z axes)
 *                            frame to be parsed.
 * @param[in]     frame     : Either data is enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]     fifo      : Structure instance of bmi08_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 * @retval > 0 -> Warning
 *
 */
static int8_t unpack_accel_frame(struct bmi08_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which contains skipped
 *                             frame count.
 * @param[in] fifo           : Structure instance of bmi08_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 * @retval > 0 -> Warning
 *
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current frame length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 *
 * @param[in,out] data_index           : Index of the FIFO data which is to be
 *                                       moved ahead of the current frame length
 * @param[in]     current_frame_length : Number of bytes in the current frame.
 * @param[in]     fifo                 : Structure instance of bmi08_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 * @retval > 0 -> Warning
 *
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which has the sensor time.
 * @param[in]     fifo       : Structure instance of bmi08_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 * @retval > 0 -> Warning
 *
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to reset the FIFO related configurations
 * in the FIFO frame structure for the next FIFO read.
 *
 * @param[in, out] fifo     : Structure instance of bmi08_fifo_frame.
 * @param[in]      dev      : Structure instance of bmi08_dev.
 *
 * @return None
 * @retval None
 *
 */
static void reset_fifo_frame_structure(struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data in header mode.
 *
 * @param[out] acc          : Structure instance of bmi08x_sens_data where
 *                            the parsed accelerometer data bytes are stored.
 * @param[in] accel_length  : Number of accelerometer frames (x,y,z data).
 * @param[in] fifo          : Structure instance of bmi08_fifo_frame.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 * @retval > 0 -> Warning
 *
 */
static int8_t extract_acc_header_mode(struct bmi08_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct bmi08_fifo_frame *fifo);

/*!
 * @brief This API sets the FIFO full or FIFO water mark interrupt for accelerometer sensor
 *
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08_dev.
 * @param[in] select_fifo_trigger         : Flag to select FIFO full or FIFO water-mark interrupt
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_fifo_full_wm_int(const struct bmi08_accel_int_channel_cfg *int_config,
                                   struct bmi08_dev *dev,
                                   uint8_t select_fifo_trigger);

/*!
 * @brief This API configures and maps the interrupt pin
 * @param[in] rslt        : Return value to be validated
 * @param[in] data        : Data to be configured
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] dev        : Structure instance of bmi08_dev.
 *
 * @return None
 * @retval None
 */
static void config_map_int_pin(int8_t *rslt,
                               uint8_t *data,
                               const struct bmi08_accel_int_channel_cfg *int_config,
                               struct bmi08_dev *dev);

/*!
 * @brief This API configures the interrupt pins based the selected channel
 * @param[in] int_config  : Structure instance of bmi08_accel_int_channel_cfg.
 * @param[in] reg_addr    : To store the register address
 * @param[in] ch1         : Interrupt Channel 1
 * @param[in] ch2         : Interrupt Channel 2
 * @param[out] rslt       : Stores the error values
 * @param[out] val        : Error values
 *
 * @return None
 * @retval None
 */
static void int_pin_channel_config(const struct bmi08_accel_int_channel_cfg *int_config,
                                   uint8_t *reg_addr,
                                   uint8_t ch1,
                                   uint8_t ch2,
                                   int8_t *rslt,
                                   int8_t val);

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
int8_t bmi08a_init(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        dev->accel_chip_id = 0;

        if (dev->intf == BMI08_SPI_INTF)
        {
            /* Set dummy byte in case of SPI interface */
            dev->dummy_byte = BMI08_ENABLE;

            /* Dummy read of Chip-ID in SPI mode */
            rslt = set_get_regs(BMI08_REG_ACCEL_CHIP_ID, &chip_id, BMI08_REG_ACCEL_CHIP_ID_LENGTH, dev, GET_FUNC);
        }
        else
        {
            /* Make dummy byte 0 in case of I2C interface */
            dev->dummy_byte = BMI08_DISABLE;
        }

        if (rslt == BMI08_OK)
        {
            rslt = set_get_regs(BMI08_REG_ACCEL_CHIP_ID, &chip_id, BMI08_REG_ACCEL_CHIP_ID_LENGTH, dev, GET_FUNC);

            if (rslt == BMI08_OK)
            {
                /* Store the chip ID in dev structure */
                dev->accel_chip_id = chip_id;
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API uploads the bmi08 config file onto the device.
 */
int8_t bmi08a_load_config_file(struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Config loading disable */
    uint8_t config_load = BMI08_DISABLE;

    /* APS disable */
    uint8_t aps_disable = BMI08_DISABLE;

    uint16_t index = 0;
    uint8_t reg_data = 0;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev); /* Type-casted to void to address compilation warning */
    rslt |= generic_null_ptr_check((void *) dev->config_file_ptr);

    /* Check if config file pointer is not null */
    if (rslt == BMI08_OK)
    {
        /* Check whether the read/write length is valid */
        if (dev->read_write_len > 0)
        {
            /* Disable advanced power save mode */
            rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_PWR_CONF,
                                       &aps_disable,
                                       BMI08_REG_ACCEL_PWR_CONF_LENGTH,
                                       dev,
                                       SET_FUNC);

            if (rslt == BMI08_OK)
            {
                /* Wait until APS disable is set. Refer the data-sheet for more information */
                dev->delay_us(450, dev->intf_ptr_accel);

                /* Disable config loading */
                rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_INIT_CTRL,
                                           &config_load,
                                           BMI08_REG_ACCEL_PWR_CONF_LENGTH,
                                           dev,
                                           SET_FUNC);
            }

            if (rslt == BMI08_OK)
            {
                for (index = 0; index < BMI08_CONFIG_STREAM_SIZE;
                     index += dev->read_write_len)
                {
                    /* Write the config stream */
                    rslt = stream_transfer_write((dev->config_file_ptr + index), index, dev);
                }

                if (rslt == BMI08_OK)
                {
                    /* Enable config loading and FIFO mode */
                    config_load = BMI08_ENABLE;

                    rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_INIT_CTRL,
                                               &config_load,
                                               BMI08_REG_ACCEL_INIT_CTRL_LENGHT,
                                               dev,
                                               SET_FUNC);

                    if (rslt == BMI08_OK)
                    {
                        /* Wait till ASIC is initialized. Refer the data-sheet for more information */
                        dev->delay_us(BMI08_MS_TO_US(BMI08_ASIC_INIT_TIME_MS), dev->intf_ptr_accel);

                        /* Check for config initialization status (1 = OK) */
                        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_INTERNAL_STAT,
                                                   &reg_data,
                                                   BMI08_REG_ACCEL_INTERNAL_STAT_LENGTH,
                                                   dev,
                                                   GET_FUNC);
                    }
                }

                /* Check for initialization status */
                if (rslt == BMI08_OK && reg_data != BMI08_INIT_OK)
                {
                    rslt = BMI08_E_CONFIG_STREAM_ERROR;
                }
            }
        }
        else
        {
            rslt = BMI08_E_RD_WR_LENGTH_INVALID;
        }
    }

    return rslt;
}

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 */
int8_t bmi08a_write_feature_config(uint8_t reg_addr, const uint16_t *reg_data, uint8_t len, struct bmi08_dev *dev)
{

    int8_t rslt;
    int8_t index = 0;
    uint16_t read_length = (reg_addr * 2) + (len * 2);
    uint8_t feature_data[read_length];

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev); /* Type-casted to void to address compilation warning */
    rslt |= generic_null_ptr_check(((void *)reg_data));

    if (rslt == BMI08_OK)
    {
        /* Read feature space up to the given feature position */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_FEATURE_CFG, &feature_data[0], read_length, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Apply the given feature config. */
            for (index = 0; index < len; ++index)
            {
                /* The feature config space is 16bit aligned. */
                feature_data[(reg_addr * 2) + (index * 2)] = reg_data[index] & 0xFF;
                feature_data[(reg_addr * 2) + (index * 2) + 1] = reg_data[index] >> 8;
            }

            /* Write back updated feature space */
            rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_FEATURE_CFG, &feature_data[0], read_length, dev, SET_FUNC);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 */
int8_t bmi08a_get_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev, uint8_t select)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *) reg_data);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        if (len > 0)
        {
            if (select == GET_FUNC)
            {
                /* Reading from the register */
                rslt = set_get_regs(reg_addr, reg_data, len, dev, GET_FUNC);
            }
            else
            {
                /* Writing to the register */
                rslt = set_get_regs(reg_addr, reg_data, len, dev, SET_FUNC);

                /* Delay for suspended mode of the sensor is 450 us */
                if (dev->accel_cfg.power == BMI08_ACCEL_PM_SUSPEND)
                {
                    dev->delay_us(450, dev->intf_ptr_accel);
                }
                /* Delay for Normal mode of the sensor is 2 us */
                else if (dev->accel_cfg.power == BMI08_ACCEL_PM_ACTIVE)
                {
                    dev->delay_us(2, dev->intf_ptr_accel);
                }
                else
                {
                    /* Invalid power input */
                    rslt = BMI08_E_INVALID_INPUT;
                }
            }
        }
        else
        {
            rslt = BMI08_E_RD_WR_LENGTH_INVALID;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the error status from the accel sensor.
 */
int8_t bmi08a_get_error_status(struct bmi08_err_reg *err_reg, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *) err_reg);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Read the error codes */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_ERR, &data, BMI08_REG_ACCEL_ERR_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Fatal error */
            err_reg->fatal_err = BMI08_GET_BITS_POS_0(data, BMI08_FATAL_ERR);

            /* User error */
            err_reg->err_code = BMI08_GET_BITS(data, BMI08_ERR_CODE);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of the accel sensor.
 */
int8_t bmi08a_get_status(uint8_t *status, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *)status);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Read the status */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_STATUS, &data, BMI08_REG_ACCEL_STATUS_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Updating the status */
            *status = BMI08_GET_BITS(data, BMI08_ACCEL_STATUS);
        }
    }

    return rslt;
}

/*!
 *  @brief This API resets the accel sensor.
 */
int8_t bmi08a_soft_reset(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        data = BMI08_SOFT_RESET_CMD;

        /* Reset accel device */
        rslt = set_get_regs(BMI08_REG_ACCEL_SOFTRESET, &data, BMI08_REG_ACCEL_SOFTRESET_LENGTH, dev, SET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Delay 1 ms after reset value is written to its register */
            dev->delay_us(BMI08_MS_TO_US(BMI08_ACCEL_SOFTRESET_DELAY_MS), dev->intf_ptr_accel);

            /* After soft reset SPI mode in the initialization phase, need to  perform a dummy SPI read
             * operation, The soft-reset performs a fundamental reset to the device,
             * which is largely equivalent to a power cycle. */
            if (dev->intf == BMI08_SPI_INTF)
            {
                /* Dummy SPI read operation of Chip-ID */
                rslt =
                    bmi08a_get_set_regs(BMI08_REG_ACCEL_CHIP_ID, &data, BMI08_REG_ACCEL_CHIP_ID_LENGTH, dev, GET_FUNC);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel config value i.e. odr, band width and range from the sensor,
 * store it in the bmi08_dev structure instance passed by the user.
 *
 */
int8_t bmi08a_get_meas_conf(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_CONF, data, BMI08_REG_ACCEL_CONF_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            dev->accel_cfg.odr = data[0] & BMI08_ACCEL_ODR_MASK;
            dev->accel_cfg.bw = (data[0] & BMI08_ACCEL_BW_MASK) >> 4;
            dev->accel_cfg.range = data[1] & BMI08_ACCEL_RANGE_MASK;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate and bandwidth
 * of accel sensor.
 */
int8_t bmi08a_set_meas_conf(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t bw, odr;
    uint8_t data = { 0 };
    uint8_t is_odr_invalid = FALSE, is_bw_invalid = FALSE;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        odr = dev->accel_cfg.odr;
        bw = dev->accel_cfg.bw;

        /* Check for valid ODR */
        if ((odr < BMI08_ACCEL_ODR_12_5_HZ) || (odr > BMI08_ACCEL_ODR_1600_HZ))
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        /* Check for valid bandwidth */
        if (bw > BMI08_ACCEL_BW_NORMAL)
        {
            /* Updating the status */
            is_bw_invalid = TRUE;
        }

        /* Invalid configuration present in ODR and BW */
        if ((!is_odr_invalid) && (!is_bw_invalid))
        {
            /* Read accel config. register */
            rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_CONF, &data, (BMI08_REG_ACCEL_CONF_LENGTH - 1), dev, GET_FUNC);
            if (rslt == BMI08_OK)
            {
                /* Update data with new odr and bw values */
                data = BMI08_SET_BITS_POS_0(data, BMI08_ACCEL_ODR, odr);
                data = BMI08_SET_BITS(data, BMI08_ACCEL_BW, bw);

                /* Write accel range to register */
                rslt =
                    bmi08a_get_set_regs(BMI08_REG_ACCEL_CONF, &data, (BMI08_REG_ACCEL_CONF_LENGTH - 1), dev, SET_FUNC);

                if (rslt == BMI08_OK)
                {
                    /* Delay required to set accel configurations */
                    dev->delay_us(BMI08_SET_ACCEL_CONF_DELAY * 1000, dev->intf_ptr_accel);
                }
            }
            else
            {
                /* Invalid configuration present in ODR and BW */
                rslt = BMI08_E_INVALID_CONFIG;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel power mode from the sensor, store it in the bmi08_dev structure
 * instance passed by the user.
 */
int8_t bmi08a_get_power_mode(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_PWR_CONF, &data, BMI08_REG_ACCEL_PWR_CONF_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Updating the current power mode */
            dev->accel_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the accel sensor.
 */
int8_t bmi08a_set_power_mode(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode;
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        power_mode = dev->accel_cfg.power;

        /* Configure data array to write to accel power configuration register */
        if (power_mode == BMI08_ACCEL_PM_ACTIVE)
        {
            data[0] = BMI08_ACCEL_PM_ACTIVE;
            data[1] = BMI08_ACCEL_POWER_ENABLE;
        }
        else if (power_mode == BMI08_ACCEL_PM_SUSPEND)
        {
            data[0] = BMI08_ACCEL_PM_SUSPEND;
            data[1] = BMI08_ACCEL_POWER_DISABLE;
        }
        else
        {
            /* Invalid power input */
            rslt = BMI08_E_INVALID_INPUT;
        }

        if (rslt == BMI08_OK)
        {
            /*enable accel sensor*/
            rslt = set_get_regs(BMI08_REG_ACCEL_PWR_CONF, &data[0], BMI08_REG_ACCEL_PWR_CONF_LENGTH, dev, SET_FUNC);

            if (rslt == BMI08_OK)
            {
                /*delay between power ctrl and power config*/
                dev->delay_us(BMI08_MS_TO_US(BMI08_POWER_CONFIG_DELAY), dev->intf_ptr_accel);

                /* write to accel power configuration register */
                rslt = set_get_regs(BMI08_REG_ACCEL_PWR_CTRL, &data[1], BMI08_REG_ACCEL_PWR_CTRL_LENGTH, dev, SET_FUNC);

                if (rslt == BMI08_OK)
                {
                    /*delay required to switch power modes*/
                    dev->delay_us(BMI08_MS_TO_US(BMI08_POWER_CONFIG_DELAY), dev->intf_ptr_accel);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the bmi08_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi08a_get_data(struct bmi08_sensor_data *accel, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *) accel);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Read accel sensor data */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_X_LSB, data, BMI08_REG_ACCEL_X_LSB_LENGHT, dev, GET_FUNC);

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
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the bmi08x_int_cfg
 * structure instance.
 */
int8_t bmi08a_set_int_config(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)int_config));

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        switch (int_config->int_type)
        {
            case BMI08_ACCEL_INT_DATA_RDY:

                /* Data ready interrupt */
                rslt = set_accel_data_ready_int(int_config, dev);
                break;
            case BMI08_ACCEL_INT_SYNC_DATA_RDY:

                /* synchronized data ready interrupt */
                rslt = set_accel_sync_data_ready_int(int_config, dev);
                break;
            case BMI08_ACCEL_SYNC_INPUT:

                /* input for synchronization on accel */
                rslt = set_accel_sync_input(int_config, dev);
                break;
            case BMI08_ACCEL_INT_FIFO_WM:

                /* FIFO watermark interrupt */
                rslt = set_fifo_full_wm_int(int_config, dev, FIFO_WM_INT);
                break;
            case BMI08_ACCEL_INT_FIFO_FULL:

                /* FIFO full interrupt */
                rslt = set_fifo_full_wm_int(int_config, dev, FIFO_FULL_INT);
                break;
            default:
                rslt = BMI08_E_INVALID_CONFIG;
                break;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the temperature of the sensor in degree Celcius.
 */
int8_t bmi08a_get_sensor_temperature(struct bmi08_dev *dev, int32_t *sensor_temp)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint16_t msb, lsb;
    uint16_t msblsb;
    int16_t temp;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)sensor_temp));

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Read sensor temperature */
        rslt = bmi08a_get_set_regs(BMI08_REG_TEMP_MSB, data, BMI08_REG_TEMP_MSB_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            msb = (data[0] << 3); /* MSB data */
            lsb = (data[1] >> 5); /* LSB data */
            msblsb = (uint16_t) (msb + lsb);

            if (msblsb > 1023)
            {
                /* Updating the msblsb */
                temp = (int16_t) (msblsb - 2048);
            }
            else
            {
                temp = (int16_t) msblsb;
            }

            /* sensor temperature */
            *sensor_temp = (temp * 125) + 23000;
        }
    }

    return rslt;

}

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 */
int8_t bmi08a_get_sensor_time(struct bmi08_dev *dev, uint32_t *sensor_time)
{
    int8_t rslt;
    uint8_t data[3] = { 0 };
    uint32_t byte2, byte1, byte0;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)sensor_time));

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Read 3-byte sensor time */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_SENSORTIME_0, data, BMI08_SENSOR_TIME_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            byte0 = data[0]; /* Lower byte */
            byte1 = (data[1] << 8); /* Middle byte */
            byte2 = (data[2] << 16); /* Higher byte */

            /* Sensor time */
            *sensor_time = (byte2 | byte1 | byte0);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets or sets the FIFO configuration of the sensor.
 */
int8_t bmi08a_get_set_fifo_config(struct bmi08_accel_fifo_config *config, struct bmi08_dev *dev, uint8_t select)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)config));

    if (rslt == BMI08_OK)
    {
        /* Get the FIFO configuration value */
        rslt = bmi08a_get_set_regs(BMI08_FIFO_CONFIG_0_ADDR, data, BMI08_FIFO_CONFIG_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            if (select == GET_FUNC)
            {
                /* Get mode selection */
                config->mode = BMI08_GET_BITS_POS_0(data[0], BMI08_ACC_FIFO_MODE_CONFIG);

                /* Get the accel enable */
                config->accel_en = BMI08_GET_BITS(data[1], BMI08_ACCEL_EN);

                /* Get the interrupt_1 enable/disable */
                config->int1_en = BMI08_GET_BITS(data[1], BMI08_ACCEL_INT1_EN);

                /* Get the interrupt_2 enable/disable */
                config->int2_en = BMI08_GET_BITS(data[1], BMI08_ACCEL_INT2_EN);
            }
            else
            {
                /* To set the stream mode or FIFO mode */
                data[0] = BMI08_SET_BITS_POS_0(data[0], BMI08_ACC_FIFO_MODE_CONFIG, config->mode);

                /* To enable the Accel in FIFO configuration */
                data[1] = BMI08_SET_BITS(data[1], BMI08_ACCEL_EN, config->accel_en);

                /* To enable the interrupt_1 in FIFO configuration */
                data[1] = BMI08_SET_BITS(data[1], BMI08_ACCEL_INT1_EN, config->int1_en);

                /* To enable the interrupt_2 in FIFO configuration */
                data[1] = BMI08_SET_BITS(data[1], BMI08_ACCEL_INT2_EN, config->int2_en);

                rslt = bmi08a_get_set_regs(BMI08_FIFO_CONFIG_0_ADDR, data, BMI08_FIFO_CONFIG_LENGTH, dev, SET_FUNC);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t bmi08a_read_fifo_data(struct bmi08_fifo_frame *fifo, struct bmi08_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store available fifo length */
    uint16_t fifo_length;

    /* Array to store FIFO configuration data */
    uint8_t config_data = 0;

    /* Variable to define FIFO address */
    uint8_t addr = BMI08_FIFO_DATA_ADDR;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)fifo));

    if (rslt == BMI08_OK)
    {
        /* Clear the FIFO data structure */
        reset_fifo_frame_structure(fifo);

        if (dev->intf == BMI08_SPI_INTF)
        {
            /* SPI mask added */
            addr = addr | BMI08_SPI_RD_MASK;
        }

        /* Read available fifo length */
        rslt = bmi08a_get_fifo_length(&fifo_length, dev);

        if (rslt == BMI08_OK)
        {
            fifo->length = fifo_length + dev->dummy_byte;

            /* Read FIFO data */
            dev->intf_rslt = dev->read(addr, fifo->data, (uint32_t)fifo->length, dev->intf_ptr_accel);

            /* If interface read fails, update rslt variable with communication failure */
            if (dev->intf_rslt != BMI08_INTF_RET_SUCCESS)
            {
                rslt = BMI08_E_COM_FAIL;
            }
        }

        if (rslt == BMI08_OK)
        {
            /* Get the set FIFO frame configurations */
            rslt = bmi08a_get_set_regs(BMI08_FIFO_CONFIG_1_ADDR,
                                       &config_data,
                                       BMI08_FIFO_INPUT_CFG_LENGTH,
                                       dev,
                                       GET_FUNC);
            if (rslt == BMI08_OK)
            {
                /* Get sensor enable status, of which the data
                 * is to be read
                 */
                fifo->acc_data_enable = (uint16_t)((uint16_t)config_data & BMI08_ACCEL_EN_MASK);
            }
        }
        else
        {
            rslt = BMI08_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t bmi08a_get_fifo_length(uint16_t *fifo_length, struct bmi08_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO data length */
    uint8_t data[BMI08_FIFO_DATA_LENGTH] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)fifo_length));

    if (rslt == BMI08_OK)
    {
        /* read fifo length */
        rslt = bmi08a_get_set_regs(BMI08_FIFO_LENGTH_0_ADDR, data, BMI08_FIFO_DATA_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            /* Get the MSB byte of FIFO length */
            data[1] = BMI08_GET_BITS_POS_0(data[1], BMI08_FIFO_BYTE_COUNTER_MSB);

            /* Get total FIFO length */
            (*fifo_length) = (uint16_t)((uint16_t)(data[1] << 8) | data[0]);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t bmi08a_get_set_fifo_wm(uint16_t *wm, struct bmi08_dev *dev, uint8_t select)
{
    int8_t rslt;

    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)wm));

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_get_set_regs(BMI08_FIFO_WTM_0_ADDR, data, BMI08_FIFO_WTM_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            if (select == GET_FUNC)
            {
                *wm = (data[1] << 8) | (data[0]);
            }
            else
            {
                /* Get LSB value of FIFO water-mark */
                data[0] = BMI08_GET_LSB(*wm);

                /* Get MSB value of FIFO water-mark */
                data[1] = BMI08_GET_MSB(*wm);

                /* Set the FIFO water-mark level */
                rslt = bmi08a_get_set_regs(BMI08_FIFO_WTM_0_ADDR, data, BMI08_FIFO_WTM_LENGTH, dev, SET_FUNC);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the accelerometer frames from FIFO data
 * read by the "bmi08x_read_fifo_data" API and stores it in the "accel_data"
 * structure instance.
 */
int8_t bmi08a_extract_accel(struct bmi08_sensor_data *accel_data,
                            uint16_t *accel_length,
                            struct bmi08_fifo_frame *fifo,
                            const struct bmi08_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *) accel_data));
    rslt |= generic_null_ptr_check(((void *) accel_length));
    rslt |= generic_null_ptr_check(((void *) fifo));

    if ((rslt == BMI08_OK))
    {
        /* Check if this is the first iteration of data unpacking
         * if yes, then consider dummy byte on SPI
         */
        if (fifo->acc_byte_start_idx == 0)
        {
            /* Dummy byte included */
            fifo->acc_byte_start_idx = dev->dummy_byte;
        }

        /* Parsing the FIFO data in header mode */
        rslt = extract_acc_header_mode(accel_data, accel_length, fifo);
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the down sampling rates which is configured for
 * accelerometer FIFO data.
 */
int8_t bmi08a_get_set_fifo_down_sample(uint8_t *fifo_downs, struct bmi08_dev *dev, uint8_t select)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *)fifo_downs));

    if (rslt == BMI08_OK)
    {
        /* Read the accelerometer FIFO down data sampling rate */
        rslt = bmi08a_get_set_regs(BMI08_FIFO_DOWNS_ADDR, &data, BMI08_FIFO_DOWNS_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            if (select == GET_FUNC)
            {
                (*fifo_downs) = BMI08_GET_BITS(data, BMI08_ACC_FIFO_DOWNS);
            }
            else
            {
                data = BMI08_SET_BITS(data, BMI08_ACC_FIFO_DOWNS, *fifo_downs);
                rslt = bmi08a_get_set_regs(BMI08_FIFO_DOWNS_ADDR, &data, BMI08_FIFO_DOWNS_LENGTH, dev, SET_FUNC);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the i2c_wdt_sel and i2c_wdt_en information
 */
int8_t bmi08a_get_set_i2c_wdt(uint8_t *i2c_wdt_sel, uint8_t *i2c_wdt_en, struct bmi08_dev *dev, uint8_t select)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store wdt information */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *)i2c_wdt_sel);
    rslt |= generic_null_ptr_check(((void *)i2c_wdt_en));

    if ((rslt == BMI08_OK))
    {
        /* Read the watchdog information */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_WDT, &data, BMI08_REG_ACCEL_WDT_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            if (select == GET_FUNC)
            {
                (*i2c_wdt_sel) = BMI08_GET_BITS(data, BMI08_I2C_WDT_SEL);
                (*i2c_wdt_en) = BMI08_GET_BITS(data, BMI08_I2C_WDT_EN);
            }
            else
            {
                data = BMI08_SET_BITS(data, BMI08_I2C_WDT_SEL, *i2c_wdt_sel);
                data = BMI08_SET_BITS(data, BMI08_I2C_WDT_EN, *i2c_wdt_en);

                rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_WDT, &data, BMI08_REG_ACCEL_WDT_LENGTH, dev, SET_FUNC);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the data synchronization
 *  feature.
 */
int8_t bmi08a_configure_data_synchronization(struct bmi08_data_sync_cfg sync_cfg, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI08_ACCEL_DATA_SYNC_LEN];

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Change sensor meas config */
        switch (sync_cfg.mode)
        {
            case BMI08_ACCEL_DATA_SYNC_MODE_2000HZ:
                dev->accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
                dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
                dev->gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
                break;
            case BMI08_ACCEL_DATA_SYNC_MODE_1000HZ:
                dev->accel_cfg.odr = BMI08_ACCEL_ODR_800_HZ;
                dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
                dev->gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
                break;
            case BMI08_ACCEL_DATA_SYNC_MODE_400HZ:
                dev->accel_cfg.odr = BMI08_ACCEL_ODR_400_HZ;
                dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;
                dev->gyro_cfg.bw = BMI08_GYRO_BW_47_ODR_400_HZ;
                break;
            default:
                break;
        }

        rslt = bmi08a_set_meas_conf(dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi08g_set_meas_conf(dev);
            if (rslt == BMI08_OK)
            {
                /* Enable data synchronization */
                data[0] = (sync_cfg.mode & BMI08_ACCEL_DATA_SYNC_MODE_MASK);
                rslt = bmi08a_write_feature_config(BMI08_ACCEL_DATA_SYNC_ADR, &data[0], BMI08_ACCEL_DATA_SYNC_LEN, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi08_sensor_data structure instance
 *  passed by the user.
 */
int8_t bmi08a_get_synchronized_data(struct bmi08_sensor_data *accel,
                                    struct bmi08_sensor_data *gyro,
                                    struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr, data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Proceed if pointers are not NULL */
    if ((accel != NULL) && (gyro != NULL))
    {
        /* Read accel x,y sensor data */
        reg_addr = BMI08_REG_ACCEL_GP_0;
        rslt = bmi08a_get_set_regs(reg_addr, &data[0], BMI08_REG_READ_ACCEL_XY_LENGTH, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Read accel sensor data */
            reg_addr = BMI08_REG_ACCEL_GP_4;
            rslt = bmi08a_get_set_regs(reg_addr, &data[4], BMI08_REG_READ_ACCEL_LENGTH, dev, GET_FUNC);

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

                /* Read gyro sensor data */
                rslt = bmi08g_get_data(gyro, dev);
            }
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi08x_int_cfg
 *  structure instance.
 */
int8_t bmi08a_set_data_sync_int_config(const struct bmi08_int_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check(((void *) int_config));

    if (rslt == BMI08_OK)
    {
        /* Configure accel sync data ready interrupt configuration */
        rslt = bmi08a_set_int_config(&int_config->accel_int_config_1, dev);
        if (rslt == BMI08_OK)
        {
            rslt = bmi08a_set_int_config(&int_config->accel_int_config_2, dev);
            if (rslt == BMI08_OK)
            {
                /* Configure gyro data ready interrupt configuration */
                rslt = bmi08g_set_int_config(&int_config->gyro_int_config_1, dev);
                if (rslt == BMI08_OK)
                {
                    rslt = bmi08g_set_int_config(&int_config->gyro_int_config_2, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API gets accel feature interrupt status
 */
int8_t bmi08a_get_data_int_status(uint8_t *int_status, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t status = 0;

    /* Check for null pointer in the device structure */
    rslt = dev_null_ptr_check(dev); /* Type-casted to void to address compilation warning */
    rslt |= generic_null_ptr_check(((void *) int_status));

    if (rslt == BMI08_OK)
    {
        rslt =
            bmi08a_get_set_regs(BMI08_REG_ACCEL_INT_STAT_1, &status, BMI08_REG_ACCEL_INT_STATUS_LENGTH, dev, GET_FUNC);
        if (rslt == BMI08_OK)
        {
            (*int_status) = status;
        }
    }

    return rslt;
}

/***************************************************************************/

/**\name        Static Function definitions
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t dev_null_ptr_check(const struct bmi08_dev *dev)
{
    int8_t rslt = BMI08_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to validate the variable for null conditions.
 */
static int8_t generic_null_ptr_check(void *data_parm)
{
    int8_t rslt = BMI08_OK;

    if (data_parm == NULL)
    {
        /* Device structure pointer is not valid */
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes or reads the data to/from the given register address.
 */
static int8_t set_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev, uint8_t select)
{
    int8_t rslt = BMI08_OK;
    uint16_t index;
    uint8_t temp_buff[BMI08_MAX_LEN];

    if (select == GET_FUNC)
    {
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
    }
    else
    {
        /* Configuring reg_addr for SPI Interface */
        if (dev->intf == BMI08_SPI_INTF)
        {
            /* Configuring reg_addr for SPI Interface */
            reg_addr = (reg_addr & BMI08_SPI_WR_MASK);
        }

        /* write to an accel register */
        dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr_accel);

        if (dev->intf_rslt != BMI08_INTF_RET_SUCCESS)
        {
            /* Updating the error status */
            rslt = BMI08_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt, is_channel_invalid = FALSE;
    uint8_t reg_addr = 0, data;

    int_pin_channel_config(int_config,
                           &reg_addr,
                           BMI08_REG_ACCEL_INT1_IO_CONF,
                           BMI08_REG_ACCEL_INT2_IO_CONF,
                           &is_channel_invalid,
                           1); /*configures the interrupt pins based the selected channel */

    if (!is_channel_invalid)
    {
        /* Read interrupt pin configuration register */
        rslt = set_get_regs(reg_addr, &data, 1, dev, GET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Update data with user configured bmi08x_int_cfg structure */
            data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_LVL, int_config->int_pin_cfg.lvl);
            data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_OD, int_config->int_pin_cfg.output_mode);

            if (int_config->int_type == BMI08_ACCEL_SYNC_INPUT)
            {
                data = BMI08_SET_BITS_POS_0(data, BMI08_ACCEL_INT_EDGE, BMI08_ENABLE);
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_IN, int_config->int_pin_cfg.enable_int_pin);
                data = BMI08_SET_BIT_VAL_0(data, BMI08_ACCEL_INT_IO);
            }
            else
            {
                data = BMI08_SET_BITS(data, BMI08_ACCEL_INT_IO, int_config->int_pin_cfg.enable_int_pin);
                data = BMI08_SET_BIT_VAL_0(data, BMI08_ACCEL_INT_IN);
            }

            /* Write to interrupt pin configuration register */
            rslt = bmi08a_get_set_regs(reg_addr, &data, BMI08_REG_INT_CFG_LENGTH, dev, SET_FUNC);
        }
    }
    else
    {
        rslt = BMI08_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static int8_t set_accel_data_ready_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = set_get_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, BMI08_REG_ACCEL_INT_MAP_DATA_LENGTH, dev, GET_FUNC);

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

        config_map_int_pin(&rslt, &data, int_config, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *) int_config);

    if (rslt == BMI08_OK)
    {
        data = BMI08_ACCEL_DATA_SYNC_INT_DISABLE;

        int_pin_channel_config(int_config,
                               &reg_addr,
                               BMI08_REG_ACCEL_INT1_MAP,
                               BMI08_REG_ACCEL_INT2_MAP,
                               &rslt,
                               BMI08_E_INVALID_INPUT); /* configures the interrupt pins based the selected channel */

        if (rslt == BMI08_OK)
        {
            if (int_config->int_pin_cfg.enable_int_pin == BMI08_ENABLE)
            {
                /*interrupt A mapped to INT1/INT2 */
                data = BMI08_ACCEL_DATA_SYNC_INT_ENABLE;
            }

            /* Write to interrupt map register */
            rslt = bmi08a_get_set_regs(reg_addr, &data, BMI08_REG_ACCEL_INT_MAP_DATA_LENGTH, dev, SET_FUNC);

            if (rslt == BMI08_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 */
static int8_t set_accel_sync_input(const struct bmi08_accel_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = dev_null_ptr_check(dev);
    rslt |= generic_null_ptr_check((void *) int_config);

    if (rslt == BMI08_OK)
    {
        /*set input interrupt configuration*/
        rslt = set_int_pin_config(int_config, dev);
    }

    return rslt;
}

/*!
 *  @brief This API writes the config stream data in memory using burst mode.
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
    uint8_t asic_lsb = ((index / 2) & 0x0F);

    /* Write to feature config register */
    rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_RESERVED_5B, &asic_lsb, BMI08_REG_FEATURE_CFG_LENGTH, dev, SET_FUNC);
    if (rslt == BMI08_OK)
    {
        /* Write to feature config register */
        rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_RESERVED_5C, &asic_msb, BMI08_REG_FEATURE_CFG_LENGTH, dev, SET_FUNC);

        if (rslt == BMI08_OK)
        {
            /* Write to feature config registers */
            rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_FEATURE_CFG,
                                       (uint8_t *)stream_data,
                                       dev->read_write_len,
                                       dev,
                                       SET_FUNC);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct bmi08_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI08_OK;

    /* Validate data index */
    if (((*data_index) + BMI08_FIFO_SKIP_FRM_LENGTH) > fifo->length)
    {
        /* Update the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI08_W_FIFO_EMPTY;
    }
    else
    {
        /* Update skipped frame count in the FIFO structure */
        fifo->skipped_frame_count = fifo->data[(*data_index)];

        /* Move the data index by 1 byte */
        (*data_index) = (*data_index) + 1;

        /* More frames could be read */
        rslt = BMI08_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to reset the FIFO related configurations in
 * the FIFO frame structure for the next FIFO read.
 */
static void reset_fifo_frame_structure(struct bmi08_fifo_frame *fifo)
{
    /* Reset FIFO data structure */
    fifo->acc_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
}

/*!
 * @brief This internal API is used to parse accelerometer data from the
 * FIFO data.
 */
static void unpack_accel_data(struct bmi08_sensor_data *acc,
                              uint16_t data_start_index,
                              const struct bmi08_fifo_frame *fifo)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    /* Accelerometer raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->z = (int16_t)((data_msb << 8) | data_lsb);

}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in header mode. It updates the current data
 * byte to be parsed.
 */
static int8_t unpack_accel_frame(struct bmi08_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct bmi08_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI08_OK;

    switch (frame)
    {
        /* If frame contains only accelerometer data */
        case BMI08_FIFO_HEADER_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + BMI08_FIFO_ACCEL_LENGTH) > fifo->length)
            {
                /* Update the data index as complete*/
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = BMI08_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], *idx, fifo);

            /* Update data index */
            (*idx) = (*idx) + BMI08_FIFO_ACCEL_LENGTH;

            /* Update accelerometer frame index */
            (*acc_idx)++;

            break;
        default:

            /* Move the data index to the last byte in case of invalid values */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = BMI08_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current_frame_length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi08_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI08_OK;

    /* Validate data index */
    if (((*data_index) + current_frame_length) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI08_W_FIFO_EMPTY;
    }
    else
    {
        /* Move the data index to next frame */
        (*data_index) = (*data_index) + current_frame_length;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct bmi08_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = BMI08_OK;

    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Validate data index */
    if (((*data_index) + BMI08_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = BMI08_W_FIFO_EMPTY;
    }
    else
    {
        /* Get sensor time from the FIFO data */
        sensor_time_byte3 = fifo->data[(*data_index) + BMI08_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + BMI08_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Update sensor time in the FIFO structure */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

        /* Move the data index by 3 bytes */
        (*data_index) = (*data_index) + BMI08_SENSOR_TIME_LENGTH;

    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO in header mode.
 */
static int8_t extract_acc_header_mode(struct bmi08_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct bmi08_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = BMI08_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to index the data bytes */
    uint16_t data_index;

    /* Variable to index accelerometer frames */
    uint16_t accel_index = 0;

    /* Variable to indicate accelerometer frames read */
    uint16_t frame_to_read = *accel_length;

    for (data_index = fifo->acc_byte_start_idx; data_index < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_index];

        /* Index shifted to next byte where data starts */
        data_index++;
        switch (frame_header)
        {
            /* If header defines accelerometer frame */
            case BMI08_FIFO_HEADER_ACC_FRM:
            case BMI08_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_accel_frame(acc, &data_index, &accel_index, frame_header, fifo);
                break;

            /* If header defines sensor time frame */
            case BMI08_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_index, fifo);
                break;

            /* If header defines skip frame */
            case BMI08_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_index, fifo);
                break;

            /* If header defines Input configuration frame */
            case BMI08_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_index, BMI08_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines sample drop frame */
            case BMI08_FIFO_SAMPLE_DROP_FRM:
                rslt = move_next_frame(&data_index, BMI08_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case BMI08_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data index to the last byte to mark completion */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI08_W_FIFO_EMPTY;
                break;
            default:

                /* Move the data index to the last byte in case of invalid values */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = BMI08_W_FIFO_EMPTY;
                break;
        }

        /* Break if Number of frames to be read is complete or FIFO is mpty */
        if ((frame_to_read == accel_index) || (rslt == BMI08_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the accelerometer frame index */
    (*accel_length) = accel_index;

    /* Update the accelerometer byte index */
    fifo->acc_byte_start_idx = data_index;

    return rslt;
}

/*!
 * @brief This API sets the FIFO full or FIFO water mark interrupt for accelerometer sensor.
 */
static int8_t set_fifo_full_wm_int(const struct bmi08_accel_int_channel_cfg *int_config,
                                   struct bmi08_dev *dev,
                                   uint8_t select_fifo_trigger)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = set_get_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA, &data, BMI08_REG_ACCEL_INT_MAP_DATA_LENGTH, dev, GET_FUNC);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;
        if (select_fifo_trigger == FIFO_FULL_INT)
        {
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
        }
        else
        {
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

        }

        config_map_int_pin(&rslt, &data, int_config, dev);
    }

    return rslt;
}

/*!
 * @brief This API configures and maps the interrupt pin
 */
static void config_map_int_pin(int8_t *rslt,
                               uint8_t *data,
                               const struct bmi08_accel_int_channel_cfg *int_config,
                               struct bmi08_dev *dev)
{
    if (*rslt == BMI08_OK)
    {
        /* Configure interrupt pins */
        *rslt = set_int_pin_config(int_config, dev);

        if (*rslt == BMI08_OK)
        {
            /* Write to interrupt map register */
            *rslt = bmi08a_get_set_regs(BMI08_REG_ACCEL_INT1_INT2_MAP_DATA,
                                        data,
                                        BMI08_REG_ACCEL_INT_MAP_DATA_LENGTH,
                                        dev,
                                        SET_FUNC);
        }
    }
}

/*!
 * @brief This API configures the interrupt pins based the selected channel
 */
static void int_pin_channel_config(const struct bmi08_accel_int_channel_cfg *int_config,
                                   uint8_t *reg_addr,
                                   uint8_t ch1,
                                   uint8_t ch2,
                                   int8_t *rslt,
                                   int8_t val)
{
    if (int_config->int_channel == BMI08_INT_CHANNEL_1)
    {
        *reg_addr = ch1;
    }
    else if (int_config->int_channel == BMI08_INT_CHANNEL_2)
    {
        *reg_addr = ch2;
    }
    else
    {
        *rslt = val;
    }
}

/*! @endcond */
