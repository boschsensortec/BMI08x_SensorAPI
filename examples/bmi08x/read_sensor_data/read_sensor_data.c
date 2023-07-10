/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bmi08x.h"
#include "common.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro;

/*! bmi08 accel int config */
struct bmi08_accel_int_channel_cfg accel_int_config;

/*! bmi08 gyro int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second square.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor with default
 */
static int8_t init_bmi08(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08(void)
{
    int8_t rslt;

    rslt = bmi08xa_init(&bmi08dev);
    bmi08_error_codes_print_result("bmi08xa_init", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_init", rslt);
    }

    if (rslt == BMI08_OK)
    {
        printf("Uploading config file !\n");
        rslt = bmi08a_load_config_file(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_load_config_file", rslt);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08xa_set_meas_conf", rslt);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_250_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_meas_conf", rslt);
    }

    return rslt;

}

/*!
 *  @brief This API is used to enable bmi08 interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_get_regs", rslt);
    }

    return rslt;
}

/*!
 *  @brief This API is used to disable bmi08 interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t disable_bmi08_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 *
 */
int main(void)
{
    int8_t rslt;

    uint8_t times_to_read = 0;
    float x = 0.0, y = 0.0, z = 0.0;
    uint8_t status = 0;

    /* Interface given as parameter
     *           For I2C : BMI08_I2C_INTF
     *           For SPI : BMI08_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08_interface_init(&bmi08dev, BMI08_I2C_INTF, BMI085_VARIANT);
    bmi08_error_codes_print_result("bmi08_interface_init", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = init_bmi08();
        bmi08_error_codes_print_result("init_bmi08", rslt);

        /* Enable data ready interrupts */
        rslt = enable_bmi08_interrupt();
        bmi08_error_codes_print_result("enable_bmi08_interrupt", rslt);

        if (rslt == BMI08_OK)
        {
            if (bmi08dev.accel_cfg.power == BMI08_ACCEL_PM_ACTIVE)
            {
                printf("\nACCEL DATA\n");
                printf("Accel data in LSB units and Gravity data in m/s^2\n");
                printf("Accel data range : 16G for BMI085 and 24G for BMI088\n\n");

                printf("Sample_Count, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

                while (times_to_read < 10)
                {
                    rslt = bmi08a_get_data_int_status(&status, &bmi08dev);
                    bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

                    if (status & BMI08_ACCEL_DATA_READY_INT)
                    {
                        rslt = bmi08a_get_data(&bmi08_accel, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08a_get_data", rslt);

                        if (bmi08dev.variant == BMI085_VARIANT)
                        {
                            /* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
                            x = lsb_to_mps2(bmi08_accel.x, 16, 16);
                            y = lsb_to_mps2(bmi08_accel.y, 16, 16);
                            z = lsb_to_mps2(bmi08_accel.z, 16, 16);
                        }
                        else if (bmi08dev.variant == BMI088_VARIANT)
                        {
                            /* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
                            x = lsb_to_mps2(bmi08_accel.x, 24, 16);
                            y = lsb_to_mps2(bmi08_accel.y, 24, 16);
                            z = lsb_to_mps2(bmi08_accel.z, 24, 16);
                        }

                        printf("%d, %5d, %5d, %5d, %4.2f, %4.2f, %4.2f\n",
                               times_to_read,
                               bmi08_accel.x,
                               bmi08_accel.y,
                               bmi08_accel.z,
                               x,
                               y,
                               z);

                        times_to_read = times_to_read + 1;
                    }
                }
            }

            if (bmi08dev.gyro_cfg.power == BMI08_GYRO_PM_NORMAL)
            {
                times_to_read = 0;

                printf("\n\nGYRO DATA\n");
                printf("Gyro data in LSB units and degrees per second\n");
                printf("Gyro data range : 250 dps for BMI085 and BMI088\n\n");

                printf("Sample_Count, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_DPS_X, Gyr_DPS_Y, Gyr_DPS_Z\n");

                while (times_to_read < 10)
                {
                    rslt = bmi08g_get_data_int_status(&status, &bmi08dev);
                    bmi08_error_codes_print_result("bmi08g_get_data_int_status", rslt);

                    if (status & BMI08_GYRO_DATA_READY_INT)
                    {
                        rslt = bmi08g_get_data(&bmi08_gyro, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08g_get_data", rslt);

                        /* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
                        x = lsb_to_dps(bmi08_gyro.x, (float)250, 16);
                        y = lsb_to_dps(bmi08_gyro.y, (float)250, 16);
                        z = lsb_to_dps(bmi08_gyro.z, (float)250, 16);

                        printf("%d, %5d, %5d, %5d, %4.2f, %4.2f, %4.2f\n",
                               times_to_read,
                               bmi08_gyro.x,
                               bmi08_gyro.y,
                               bmi08_gyro.z,
                               x,
                               y,
                               z);

                        times_to_read = times_to_read + 1;
                    }
                }
            }
        }

        /* Disable data ready interrupts */
        rslt = disable_bmi08_interrupt();
        bmi08_error_codes_print_result("disable_bmi08_interrupt", rslt);
    }

    bmi08_coines_deinit();

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
