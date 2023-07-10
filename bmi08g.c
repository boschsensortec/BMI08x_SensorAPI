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
* @file       bmi08g.c
* @date       2023-03-27
* @version    v1.7.1
*
*/

/*! \file bmi08g.c
 * \brief Sensor Driver for BMI08 family of sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "bmi08.h"

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t null_ptr_check(const struct bmi08_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out]reg_data  : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *data, uint32_t len, struct bmi08_dev *dev);

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
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 *
 * @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_gyro_data_ready_int(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API sets the FIFO full, FIFO watermark interrupts for gyro sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_fifo_int(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_int_pin_config(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev);

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 *
 *  @param[in] selftest : Variable used to enable or disable
 *  the Gyro self test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | BMI08_DISABLE
 *  0x01    | BMI08_ENABLE
 *
 *  @param[in] dev : Structure instance of bmi08x_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_gyro_selftest(uint8_t selftest, struct bmi08_dev *dev);

/*!
 * @brief This internal API is used to get fifo data byte count
 *
 *
 * @param[in] fifo            : Structure instance of bmi08x_gyr_fifo_config.
 * @param[out] frame_size     : Size of the frame with respect to axis selected
 * @param[out] fifo_data_byte : Stores the number of bytes to be read
 *
 */
static void get_fifo_data_length(const struct bmi08_gyr_fifo_config *fifo, int8_t frame_size, uint16_t *fifo_data_byte);

/*!
 * @brief This internal API computes the number of bytes of gyroscope FIFO data
 * which is to be parsed.
 *
 * @param[out] len         : Number of bytes to be parsed.
 * @param[in]  gyr_count   : Number of gyroscope frames to be read.
 * @param[in]  fifo_conf   : Structure instance of bmi08x_gyr_fifo_config.
 * @param[in]  fifo        : Structure instance of bmi08x_fifo_frame.
 *
 */
static void parse_fifo_gyro_len(uint16_t *len,
                                const uint16_t *gyr_count,
                                const struct bmi08_gyr_fifo_config *fifo_conf,
                                const struct bmi08_fifo_frame *fifo);

/*!
 * @brief This internal API computes the number of bytes of gyroscope FIFO data
 * which is to be parsed in header-less mode.
 *
 * @param[out] gyro           : Structure instance of bmi08x_sensor_data.
 * @param[in,out] data_index  : Index value of number of bytes
 * @param[in]  fifo_conf      : Structure instance of bmi08x_gyr_fifo_config.
 * @param[in]  fifo           : Structure instance of bmi08x_fifo_frame.
 */
static void unpack_gyro_data(struct bmi08_sensor_data *gyro,
                             uint16_t *data_index,
                             const struct bmi08_gyr_fifo_config *fifo_conf,
                             const struct bmi08_fifo_frame *fifo);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 */
int8_t bmi08g_init(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        dev->gyro_chip_id = 0;

        /* Read gyro chip id */
        rslt = get_regs(BMI08_REG_GYRO_CHIP_ID, &chip_id, 1, dev);

        if (rslt == BMI08_OK)
        {
            if (chip_id == BMI08_GYRO_CHIP_ID)
            {
                /* Store the chip ID in dev structure */
                dev->gyro_chip_id = chip_id;
            }
            else
            {
                rslt = BMI08_E_DEV_NOT_FOUND;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of gyro sensor.
 */
int8_t bmi08g_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Reading from the register */
            rslt = get_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = BMI08_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of gyro sensor.
 */
int8_t bmi08g_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Writing to the register */
            rslt = set_regs(reg_addr, reg_data, len, dev);

            /* Delay for suspended mode of the sensor is 450 us */
            if (dev->gyro_cfg.power == BMI08_GYRO_PM_SUSPEND || dev->gyro_cfg.power == BMI08_GYRO_PM_DEEP_SUSPEND)
            {
                dev->delay_us(450, dev->intf_ptr_gyro);
            }
            /* Delay for Normal mode of the sensor is 2 us */
            else if (dev->gyro_cfg.power == BMI08_GYRO_PM_NORMAL)
            {
                dev->delay_us(2, dev->intf_ptr_gyro);
            }
            else
            {
                /* Invalid power input */
                rslt = BMI08_E_INVALID_INPUT;
            }
        }
        else
        {
            rslt = BMI08_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API resets the gyro sensor.
 */
int8_t bmi08g_soft_reset(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Reset gyro device */
        data = BMI08_SOFT_RESET_CMD;
        rslt = bmi08g_set_regs(BMI08_REG_GYRO_SOFTRESET, &data, 1, dev);

        if (rslt == BMI08_OK)
        {
            /* delay 30 ms after writing reset value to its register */
            dev->delay_us(BMI08_MS_TO_US(BMI08_GYRO_SOFTRESET_DELAY), dev->intf_ptr_gyro);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro odr and range from the sensor, store it in the bmi08x_dev
 * structure instance passed by the user.
 */
int8_t bmi08g_get_meas_conf(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_RANGE, data, 2, dev);

        if (rslt == BMI08_OK)
        {
            dev->gyro_cfg.range = data[0];
            dev->gyro_cfg.odr = (data[1] & BMI08_GYRO_BW_MASK);
            dev->gyro_cfg.bw = dev->gyro_cfg.odr;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of gyro sensor.
 */
int8_t bmi08g_set_meas_conf(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint8_t odr, range;
    uint8_t is_range_invalid = FALSE, is_odr_invalid = FALSE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        odr = dev->gyro_cfg.odr;
        range = dev->gyro_cfg.range;

        if (odr > BMI08_GYRO_BW_32_ODR_100_HZ)
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        if (range > BMI08_GYRO_RANGE_125_DPS)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If ODR and Range is valid, write it to gyro config. registers */
        if ((!is_odr_invalid) && (!is_range_invalid))
        {
            /* Read range value from the range register */
            rslt = bmi08g_get_regs(BMI08_REG_GYRO_BANDWIDTH, &data, 1, dev);

            if (rslt == BMI08_OK)
            {
                data = BMI08_SET_BITS_POS_0(data, BMI08_GYRO_BW, odr);

                /* Write odr value to odr register */
                rslt = bmi08g_set_regs(BMI08_REG_GYRO_BANDWIDTH, &data, 1, dev);

                if (rslt == BMI08_OK)
                {
                    /* Read range value from the range register */
                    rslt = bmi08g_get_regs(BMI08_REG_GYRO_RANGE, &data, 1, dev);
                }

                if (rslt == BMI08_OK)
                {
                    data = BMI08_SET_BITS_POS_0(data, BMI08_GYRO_RANGE, range);

                    /* Write range value to range register */
                    rslt = bmi08g_set_regs(BMI08_REG_GYRO_RANGE, &data, 1, dev);
                }

                if (rslt == BMI08_OK)
                {
                    /* Delay required to set configurations */
                    dev->delay_us(BMI08_GYRO_SET_CONFIG_DELAY * 1000, dev->intf_ptr_gyro);
                }
            }
        }
        else
        {
            /* Invalid configuration present in ODR, Range */
            rslt = BMI08_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro power mode from the sensor,
 * store it in the bmi08x_dev structure instance
 * passed by the user.
 *
 */
int8_t bmi08g_get_power_mode(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_LPM1, &data, 1, dev);

        if (rslt == BMI08_OK)
        {
            /* Updating the power mode in the dev structure */
            dev->gyro_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the gyro sensor.
 */
int8_t bmi08g_set_power_mode(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode, data;
    uint8_t is_power_switching_mode_valid = TRUE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /*read the previous power state*/
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_LPM1, &data, 1, dev);

        if (rslt == BMI08_OK)
        {
            power_mode = dev->gyro_cfg.power;

            /* Switching between normal mode and the suspend modes is allowed, it is not possible to switch
             * between suspend and deep suspend and vice versa. Check for invalid power switching,
             * (i.e) deep suspend to suspend */
            if ((power_mode == BMI08_GYRO_PM_SUSPEND) && (data == BMI08_GYRO_PM_DEEP_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check for invalid power switching (i.e) from suspend to deep suspend */
            if ((power_mode == BMI08_GYRO_PM_DEEP_SUSPEND) && (data == BMI08_GYRO_PM_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check if power switching mode is valid*/
            if (is_power_switching_mode_valid)
            {
                /* Write power to power register */
                rslt = bmi08g_set_regs(BMI08_REG_GYRO_LPM1, &power_mode, 1, dev);

                if (rslt == BMI08_OK)
                {
                    /* Time required to switch the power mode */
                    dev->delay_us(BMI08_MS_TO_US(BMI08_GYRO_POWER_MODE_CONFIG_DELAY), dev->intf_ptr_gyro);
                }
            }
            else
            {
                /* Updating the error */
                rslt = BMI08_E_INVALID_INPUT;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro data from the sensor,
 * store it in the bmi08x_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi08g_get_data(struct bmi08_sensor_data *gyro, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08_OK) && (gyro != NULL))
    {
        /* read gyro sensor data */
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_X_LSB, data, 6, dev);

        if (rslt == BMI08_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            gyro->x = (int16_t)msblsb; /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            gyro->y = (int16_t)msblsb; /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            gyro->z = (int16_t)msblsb; /* Data in Z axis */
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary gyro interrupt
 * based on the user settings in the bmi08x_int_cfg
 * structure instance.
 */
int8_t bmi08g_set_int_config(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08_OK) && (int_config != NULL))
    {

        switch (int_config->int_type)
        {
            case BMI08_GYRO_INT_DATA_RDY:

                /* Data ready interrupt */
                rslt = set_gyro_data_ready_int(int_config, dev);
                break;
            case BMI08_GYRO_INT_FIFO_WM:
            case BMI08_GYRO_INT_FIFO_FULL:

                /* FIFO interrupt */
                rslt = set_fifo_int(int_config, dev);
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
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not.
 */
int8_t bmi08g_perform_selftest(struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, loop_break = 1;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08_OK)
    {
        /* Enable the gyro self-test */
        rslt = set_gyro_selftest(BMI08_ENABLE, dev);

        if (rslt == BMI08_OK)
        {
            /* Loop till self-test ready bit is set */
            while (loop_break)
            {
                /* Read self-test register to check if self-test ready bit is set */
                rslt = bmi08g_get_regs(BMI08_REG_GYRO_SELF_TEST, &data, 1, dev);

                if (rslt == BMI08_OK)
                {
                    data = BMI08_GET_BITS(data, BMI08_GYRO_SELF_TEST_RDY);

                    if (data)
                    {
                        /* If self-test ready bit is set, exit the loop */
                        loop_break = 0;
                    }
                }
                else
                {
                    /* Exit the loop in case of communication failure */
                    loop_break = 0;
                }
            }

            if (rslt == BMI08_OK)
            {
                /* Read self-test register to check for self-test Ok bit */
                rslt = bmi08g_get_regs(BMI08_REG_GYRO_SELF_TEST, &data, 1, dev);

                if (rslt == BMI08_OK)
                {
                    data = BMI08_GET_BITS(data, BMI08_GYRO_SELF_TEST_RESULT);

                    rslt = bmi08g_soft_reset(dev);

                    if (rslt == BMI08_OK)
                    {
                        /* Updating the self test result */
                        rslt = (int8_t) data;
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API gets gyro data ready interrupt status
 */
int8_t bmi08g_get_data_int_status(uint8_t *int_status, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t status = 0;

    if (int_status != NULL)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT_STAT_1, &status, 1, dev);
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

/*!
 *  @brief This API is used to get fifo overrun.
 */
int8_t bmi08g_get_fifo_overrun(uint8_t *fifo_overrun, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = 0;

    if (fifo_overrun != NULL)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_FIFO_STATUS, &reg_data, 1, dev);

        if (rslt == BMI08_OK)
        {
            *fifo_overrun = BMI08_GET_BITS(reg_data, BMI08_GYRO_FIFO_OVERRUN);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to get fifo configuration of the sensor.
 */
int8_t bmi08g_get_fifo_config(struct bmi08_gyr_fifo_config *fifo_conf, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t fifo_config[2] = { 0 };
    uint8_t reg_data = 0;

    if (fifo_conf != NULL)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_FIFO_CONFIG0, fifo_config, 2, dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi08g_get_regs(BMI08_REG_GYRO_FIFO_STATUS, &reg_data, 1, dev);

            if (rslt == BMI08_OK)
            {
                fifo_conf->tag = BMI08_GET_BITS(fifo_config[0], BMI08_GYRO_FIFO_TAG);

                fifo_conf->wm_level = BMI08_GET_BITS_POS_0(fifo_config[0], BMI08_GYRO_FIFO_WM_LEVEL);

                fifo_conf->mode = BMI08_GET_BITS(fifo_config[1], BMI08_GYRO_FIFO_MODE);

                fifo_conf->data_select = BMI08_GET_BITS_POS_0(fifo_config[1], BMI08_GYRO_FIFO_DATA_SELECT);

                fifo_conf->frame_count = BMI08_GET_BITS_POS_0(reg_data, BMI08_GYRO_FIFO_FRAME_COUNT);
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
 *  @brief This API is used to get fifo configuration of the sensor.
 */
int8_t bmi08g_set_fifo_config(const struct bmi08_gyr_fifo_config *fifo_conf, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t fifo_config[2] = { 0 };

    if (fifo_conf != NULL)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_FIFO_CONFIG0, fifo_config, 2, dev);

        if (rslt == BMI08_OK)
        {
            fifo_config[0] = BMI08_SET_BITS(fifo_config[0], BMI08_GYRO_FIFO_TAG, fifo_conf->tag);

            fifo_config[0] = BMI08_SET_BITS_POS_0(fifo_config[0], BMI08_GYRO_FIFO_WM_LEVEL, fifo_conf->wm_level);

            fifo_config[1] = BMI08_SET_BITS_POS_0(fifo_config[1], BMI08_GYRO_FIFO_DATA_SELECT, fifo_conf->data_select);

            fifo_config[1] = BMI08_SET_BITS(fifo_config[1], BMI08_GYRO_FIFO_MODE, fifo_conf->mode);

            rslt = bmi08g_set_regs(BMI08_REG_GYRO_FIFO_CONFIG0, fifo_config, 2, dev);
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t bmi08g_get_fifo_length(const struct bmi08_gyr_fifo_config *fifo_config, struct bmi08_fifo_frame *fifo)
{
    int8_t rslt = BMI08_OK;
    uint16_t fifo_data_byte_count = 0;

    if ((fifo != NULL) && (fifo_config != NULL))
    {
        if (fifo_config->data_select == BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED)
        {
            get_fifo_data_length(fifo_config, BMI08_GYRO_FIFO_XYZ_AXIS_FRAME_SIZE, &fifo_data_byte_count);
        }
        else
        {
            get_fifo_data_length(fifo_config, BMI08_GYRO_FIFO_SINGLE_AXIS_FRAME_SIZE, &fifo_data_byte_count);
        }

        if (fifo->length > fifo_data_byte_count)
        {
            fifo->length = fifo_data_byte_count;
        }
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to read the fifo data from the sensor.
 */
int8_t bmi08g_read_fifo_data(const struct bmi08_fifo_frame *fifo, struct bmi08_dev *dev)
{
    int8_t rslt = BMI08_OK;

    if (fifo != NULL)
    {
        rslt = bmi08g_get_regs(BMI08_REG_GYRO_FIFO_DATA, fifo->data, fifo->length, dev);
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to extract gyroscope data from fifo.
 */
void bmi08g_extract_gyro(struct bmi08_sensor_data *gyro_data,
                         const uint16_t *gyro_length,
                         const struct bmi08_gyr_fifo_config *fifo_conf,
                         const struct bmi08_fifo_frame *fifo)
{
    uint16_t data_index = 0;
    uint16_t gyro_index = 0;
    uint16_t data_read_length = 0;

    /* Get the number of gyro bytes to be read */
    parse_fifo_gyro_len(&data_read_length, gyro_length, fifo_conf, fifo);

    for (; data_index < data_read_length;)
    {
        unpack_gyro_data(&gyro_data[gyro_index], &data_index, fifo_conf, fifo);
        gyro_index++;
    }
}

int8_t bmi08g_enable_watermark(uint8_t enable, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    if (enable)
    {
        reg_data = BMI08_GYRO_FIFO_WM_ENABLE_VAL;
        rslt = bmi08g_set_regs(BMI08_REG_GYRO_FIFO_WM_ENABLE, &reg_data, 1, dev);
    }
    else
    {
        reg_data = BMI08_GYRO_FIFO_WM_DISABLE_VAL;
        rslt = bmi08g_set_regs(BMI08_REG_GYRO_FIFO_WM_ENABLE, &reg_data, 1, dev);
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

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
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
 * @brief This API reads the data from the given register address of gyro sensor.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev)
{
    int8_t rslt = BMI08_OK;

    if (dev->intf == BMI08_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr | BMI08_SPI_RD_MASK);
    }

    /* Read gyro register */
    dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr_gyro);

    if (dev->intf_rslt != BMI08_INTF_RET_SUCCESS)
    {
        /* Updating the error */
        rslt = BMI08_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address of gyro sensor.
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08_dev *dev)
{
    int8_t rslt = BMI08_OK;
    uint8_t count = 0;

    if (dev->intf == BMI08_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & BMI08_SPI_WR_MASK);
    }

    /* SPI write requires to set The MSB of reg_addr as 0
     * but in default the MSB is always 0
     */
    if (len == 1)
    {
        dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr_gyro);

        if (dev->intf_rslt != BMI08_INTF_RET_SUCCESS)
        {
            /* Failure case */
            rslt = BMI08_E_COM_FAIL;
        }
    }

    /* Burst write is not allowed thus we split burst case write
     * into single byte writes Thus user can write multiple bytes
     * with ease
     */
    if (len > 1)
    {
        for (count = 0; count < len; count++)
        {
            dev->intf_rslt = dev->write(reg_addr, &reg_data[count], 1, dev->intf_ptr_gyro);

            reg_addr++;

            if (dev->intf_rslt != BMI08_INTF_RET_SUCCESS)
            {
                /* Failure case */
                rslt = BMI08_E_COM_FAIL;
                break;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static int8_t set_gyro_data_ready_int(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    /* read interrupt map register */
    rslt = get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_3:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08_SET_BITS_POS_0(data[0], BMI08_GYRO_INT3_MAP, conf);
                break;

            case BMI08_INT_CHANNEL_4:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08_SET_BITS(data[0], BMI08_GYRO_INT4_MAP, conf);
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            /*condition to check disabling the interrupt in single channel when both
             * interrupts channels are enabled*/
            if (data[0] & BMI08_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4)
            {
                /* Updating the data */
                /* Data to enable new data ready interrupt */
                data[1] = BMI08_GYRO_DRDY_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = BMI08_GYRO_DRDY_INT_DISABLE_VAL;
            }

            /* write data to interrupt map register */
            rslt = bmi08g_set_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

            if (rslt == BMI08_OK)
            {
                /* Configure interrupt pin */
                rslt = set_int_pin_config(int_config, dev);

                if (rslt == BMI08_OK)
                {
                    /* Write data to interrupt control register */
                    rslt = bmi08g_set_regs(BMI08_REG_GYRO_INT_CTRL, &data[1], 1, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static int8_t set_fifo_int(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    /* Read interrupt map register */
    rslt = get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

    if (rslt == BMI08_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08_INT_CHANNEL_3:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08_SET_BITS(data[0], BMI08_GYRO_FIFO_INT3, conf);
                break;

            case BMI08_INT_CHANNEL_4:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08_SET_BITS(data[0], BMI08_GYRO_FIFO_INT4, conf);
                break;

            default:
                rslt = BMI08_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08_OK)
        {
            /* Condition to check disabling the interrupt in single channel when both
             * interrupts channels are enabled*/
            if (data[0] & BMI08_GYRO_MAP_FIFO_BOTH_INT3_INT4)
            {
                /* Updating the data */
                /* Data to enable new data ready interrupt */
                data[1] = BMI08_GYRO_FIFO_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = BMI08_GYRO_FIFO_INT_DISABLE_VAL;
            }

            /* write data to interrupt map register */
            rslt = bmi08g_set_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

            if (rslt == BMI08_OK)
            {
                /* Configure interrupt pin */
                rslt = set_int_pin_config(int_config, dev);

                if (rslt == BMI08_OK)
                {
                    /* write data to interrupt control register */
                    rslt = bmi08g_set_regs(BMI08_REG_GYRO_INT_CTRL, &data[1], 1, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi08_gyro_int_channel_cfg *int_config, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Read interrupt configuration register */
    rslt = get_regs(BMI08_REG_GYRO_INT3_INT4_IO_CONF, &data, 1, dev);

    if (rslt == BMI08_OK)
    {
        switch (int_config->int_channel)
        {
            /* Interrupt pin or channel 3 */
            case BMI08_INT_CHANNEL_3:

                /* Update data with user configured bmi08x_int_cfg structure */
                data = BMI08_SET_BITS_POS_0(data, BMI08_GYRO_INT3_LVL, int_config->int_pin_cfg.lvl);
                data = BMI08_SET_BITS(data, BMI08_GYRO_INT3_OD, int_config->int_pin_cfg.output_mode);
                break;

            case BMI08_INT_CHANNEL_4:

                /* Update data with user configured bmi08x_int_cfg structure */
                data = BMI08_SET_BITS(data, BMI08_GYRO_INT4_LVL, int_config->int_pin_cfg.lvl);
                data = BMI08_SET_BITS(data, BMI08_GYRO_INT4_OD, int_config->int_pin_cfg.output_mode);
                break;

            default:
                break;
        }

        /* write to interrupt configuration register */
        rslt = bmi08g_set_regs(BMI08_REG_GYRO_INT3_INT4_IO_CONF, &data, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 */
static int8_t set_gyro_selftest(uint8_t selftest, struct bmi08_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for valid selftest input */
    if ((selftest == BMI08_ENABLE) || (selftest == BMI08_DISABLE))
    {
        /* Read self test register */
        rslt = get_regs(BMI08_REG_GYRO_SELF_TEST, &data, 1, dev);

        if (rslt == BMI08_OK)
        {
            /* Enable self-test */
            data = BMI08_SET_BITS_POS_0(data, BMI08_GYRO_SELF_TEST_EN, selftest);

            /* write self test input value to self-test register */
            rslt = bmi08g_set_regs(BMI08_REG_GYRO_SELF_TEST, &data, 1, dev);
        }
    }
    else
    {
        rslt = BMI08_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to get fifo data length.
 */
static void get_fifo_data_length(const struct bmi08_gyr_fifo_config *fifo, int8_t frame_size, uint16_t *fifo_data_byte)
{
    if (fifo->tag)
    {
        *fifo_data_byte = (uint16_t)(fifo->frame_count * (frame_size + 2));
    }
    else
    {
        *fifo_data_byte = (uint16_t)(fifo->frame_count * frame_size);
    }
}

/*!
 *  @brief This internal API is used to get length of gyroscope data in fifo.
 */
static void parse_fifo_gyro_len(uint16_t *len,
                                const uint16_t *gyr_count,
                                const struct bmi08_gyr_fifo_config *fifo_conf,
                                const struct bmi08_fifo_frame *fifo)
{
    if (fifo_conf->tag == 0)
    {
        *len = fifo->length;
    }
    else if ((fifo_conf->tag == 1))
    {
        if (fifo_conf->data_select == BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED)
        {
            *len = (uint16_t)((*gyr_count) * BMI08_GYRO_FIFO_XYZ_AXIS_FRAME_SIZE);
        }
        else
        {
            *len = (uint16_t)((*gyr_count) * BMI08_GYRO_FIFO_SINGLE_AXIS_FRAME_SIZE);
        }
    }
}

/*!
 *  @brief This internal API is used to unpack the gyroscope data.
 */
static void unpack_gyro_data(struct bmi08_sensor_data *gyro,
                             uint16_t *data_index,
                             const struct bmi08_gyr_fifo_config *fifo_conf,
                             const struct bmi08_fifo_frame *fifo)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    uint16_t idx;

    idx = *data_index;

    /* Gyroscope x data */
    data_lsb = fifo->data[idx++];
    data_msb = fifo->data[idx++];
    gyro->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyroscope y data */
    data_lsb = fifo->data[idx++];
    data_msb = fifo->data[idx++];
    gyro->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Gyroscope z data */
    data_lsb = fifo->data[idx++];
    data_msb = fifo->data[idx++];
    gyro->z = (int16_t)((data_msb << 8) | data_lsb);

    if (fifo_conf->tag == 1)
    {
        idx += 2;
    }

    *data_index = idx;
}

/*! @endcond */
