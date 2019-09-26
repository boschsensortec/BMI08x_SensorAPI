/**\mainpage
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
 * @file        bmi088.c
 * @date        25 Sep 2019
 * @version     1.4.0
 *
 */

/*! \file bmi088.c
 * \brief Sensor Driver for BMI085 family of sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"
#include "bmi088.h"

#if BMI08X_FEATURE_BMI088 == 1

/****************************************************************************/

/** \name       Macros
 ****************************************************************************/

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
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/
extern const uint8_t bmi08x_config_file[];

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for bmi088 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 */
int8_t bmi088_init(struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Initialize bmi088 accel sensor */
    rslt = bmi08a_init(dev);

    if (rslt == BMI08X_OK)
    {
        /* Initialize bmi088 gyro sensor */
        rslt = bmi08g_init(dev);
    }

    return rslt;
}

/*!
 *  @brief This API uploads the bmi088 config file onto the device.
 */
int8_t bmi088_apply_config_file(struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Assign stream file */
        dev->config_file_ptr = bmi08x_config_file;

        /* Upload binary */
        rslt = bmi08a_write_config_file(dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the data synchronization
 *  feature.
 */
int8_t bmi088_configure_data_synchronization(struct bmi08x_data_sync_cfg sync_cfg, struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI08X_ACCEL_DATA_SYNC_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Change sensor meas config */
        switch (sync_cfg.mode)
        {
            case BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ:
                dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
                dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
                dev->gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
                break;
            case BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ:
                dev->accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
                dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
                dev->gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
                break;
            case BMI08X_ACCEL_DATA_SYNC_MODE_400HZ:
                dev->accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
                dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;
                dev->gyro_cfg.bw = BMI08X_GYRO_BW_47_ODR_400_HZ;
                break;
            default:
                break;
        }
        rslt = bmi08a_set_meas_conf(dev);
        if (rslt != BMI08X_OK)
        {
            return rslt;
        }

        rslt = bmi08g_set_meas_conf(dev);
        if (rslt != BMI08X_OK)
        {
            return rslt;
        }

        /* Enable data synchronization */
        data[0] = (sync_cfg.mode & BMI08X_ACCEL_DATA_SYNC_MODE_MASK);
        rslt = bmi08a_write_feature_config(BMI08X_ACCEL_DATA_SYNC_ADR, &data[0], BMI08X_ACCEL_DATA_SYNC_LEN, dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 */
int8_t bmi088_configure_anymotion(struct bmi08x_anymotion_cfg anymotion_cfg, const struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint16_t data[BMI08X_ACCEL_ANYMOTION_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Enable data synchronization */
        data[0] = (anymotion_cfg.threshold & BMI08X_ACCEL_ANYMOTION_THRESHOLD_MASK);
        data[0] |=
            ((anymotion_cfg.nomotion_sel << BMI08X_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT) &
             BMI08X_ACCEL_ANYMOTION_NOMOTION_SEL_MASK);
        data[1] = (anymotion_cfg.duration & BMI08X_ACCEL_ANYMOTION_DURATION_MASK);
        data[1] |= ((anymotion_cfg.x_en << BMI08X_ACCEL_ANYMOTION_X_EN_SHIFT) & BMI08X_ACCEL_ANYMOTION_X_EN_MASK);
        data[1] |= ((anymotion_cfg.y_en << BMI08X_ACCEL_ANYMOTION_Y_EN_SHIFT) & BMI08X_ACCEL_ANYMOTION_Y_EN_MASK);
        data[1] |= ((anymotion_cfg.z_en << BMI08X_ACCEL_ANYMOTION_Z_EN_SHIFT) & BMI08X_ACCEL_ANYMOTION_Z_EN_MASK);
        rslt = bmi08a_write_feature_config(BMI08X_ACCEL_ANYMOTION_ADR, &data[0], BMI08X_ACCEL_ANYMOTION_LEN, dev);
    }

    return rslt;
}

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 */
int8_t bmi088_get_synchronized_data(struct bmi08x_sensor_data *accel,
                                    struct bmi08x_sensor_data *gyro,
                                    const struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr, data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08X_OK) && (accel != NULL) && (gyro != NULL))
    {
        /* Read accel x,y sensor data */
        reg_addr = BMI08X_ACCEL_GP_0_REG;
        rslt = bmi08a_get_regs(reg_addr, &data[0], 4, dev);

        if (rslt == BMI08X_OK)
        {
            /* Read accel sensor data */
            reg_addr = BMI08X_ACCEL_GP_4_REG;
            rslt = bmi08a_get_regs(reg_addr, &data[4], 2, dev);

            if (rslt == BMI08X_OK)
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
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi08x_int_cfg
 *  structure instance.
 */
int8_t bmi088_set_data_sync_int_config(const struct bmi08x_int_cfg *int_config, const struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Configure accel sync data ready interrupt configuration */
    rslt = bmi08a_set_int_config(&int_config->accel_int_config_1, dev);
    if (rslt != BMI08X_OK)
    {
        return rslt;
    }

    rslt = bmi08a_set_int_config(&int_config->accel_int_config_2, dev);
    if (rslt != BMI08X_OK)
    {
        return rslt;
    }

    /* Configure gyro data ready interrupt configuration */
    rslt = bmi08g_set_int_config(&int_config->gyro_int_config_1, dev);
    if (rslt != BMI08X_OK)
    {
        return rslt;
    }

    rslt = bmi08g_set_int_config(&int_config->gyro_int_config_2, dev);

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI08X_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI08X_OK;
    }

    return rslt;
}
#endif

/** @}*/
