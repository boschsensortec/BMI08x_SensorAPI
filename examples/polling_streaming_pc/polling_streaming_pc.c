/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

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

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! accel streaming response  buffer */
uint8_t bmi08x_accel_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];

/*! gyro streaming response buffer */
uint8_t bmi08x_gyro_stream_buffer[COINES_STREAM_RSP_BUF_SIZE];

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
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
 * @brief    This internal API is used to initialize the bmi08x sensor with default
 */
static int8_t init_bmi08x(void);

/*!
 * @brief   This internal API is used to send stream settings
 */
static void send_stream_settings(void);

/*! This internal API is used to read sensor data */
void read_sensor_data(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to send stream settings
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void send_stream_settings(void)
{
    struct coines_streaming_config stream_config;

    struct coines_streaming_blocks stream_block;

    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        stream_config.intf = COINES_SENSOR_INTF_I2C;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        stream_config.intf = COINES_SENSOR_INTF_SPI;
    }

    stream_config.i2c_bus = COINES_I2C_BUS_0; /* if intf is I2C */
    stream_config.spi_bus = COINES_SPI_BUS_0; /* if intf is SPI */

    /* For I2C */
    stream_config.dev_addr = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
    stream_config.cs_pin = COINES_SHUTTLE_PIN_8;
    stream_config.sampling_time = 625; /* 1.6 khz */

    /* 1 micro second / 2 milli second */
    stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC; /* micro second */
    stream_block.no_of_blocks = 1;
    stream_block.reg_start_addr[0] = BMI08X_REG_ACCEL_X_LSB; /* Accel data start address */

    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        stream_block.no_of_data_bytes[0] = 6;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        /* Actual data length is 6 bytes 1 byte needed to
         * initiate the SPI communication */
        stream_block.no_of_data_bytes[0] = 7;
    }

    coines_config_streaming(1, &stream_config, &stream_block);

    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        stream_config.intf = COINES_SENSOR_INTF_I2C;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        stream_config.intf = COINES_SENSOR_INTF_SPI;
    }

    stream_config.i2c_bus = COINES_I2C_BUS_0; /* if intf is I2C */
    stream_config.spi_bus = COINES_SPI_BUS_0; /* if intf is SPI */

    /* For I2C */
    stream_config.dev_addr = BMI08X_GYRO_I2C_ADDR_PRIMARY;
    stream_config.cs_pin = COINES_SHUTTLE_PIN_14;
    stream_config.sampling_time = 500; /* 2 Khz */

    /* 1 micro second / 2 milli second */
    stream_config.sampling_units = COINES_SAMPLING_TIME_IN_MICRO_SEC; /* micro second */
    stream_block.no_of_blocks = 1;
    stream_block.reg_start_addr[0] = BMI08X_REG_GYRO_X_LSB; /* gyro data start address */
    stream_block.no_of_data_bytes[0] = 6;

    coines_config_streaming(2, &stream_config, &stream_block);
}

/*!
 *  @brief This internal API is used to read sensor data
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
void read_sensor_data(void)
{
    int16_t rslt;
    uint8_t lsb, msb;
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t valid_sample_count = 0;
    int idx = 0;
    int buffer_index = 0;
    float x = 0.0, y = 0.0, z = 0.0;

    if (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE)
    {
        memset(&bmi08x_accel_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(1, 1, &bmi08x_accel_stream_buffer[0], &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            printf("\nACCEL DATA \n");
            printf("Accel data in LSB units and Gravity data in m/s^2\n");
            printf("Accel data range : 16G for BMI085 and 24G for BMI088\n\n");

            if (valid_sample_count > 100)
            {
                valid_sample_count = 100;
            }

            for (idx = 0; idx < valid_sample_count; idx++)
            {
                if (bmi08xdev.intf == BMI08X_SPI_INTF)
                {
                    buffer_index++; /*dummy byte; ignore for SPI */
                }

                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                ax = (msb << 8) | lsb;

                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                ay = (msb << 8) | lsb;

                lsb = bmi08x_accel_stream_buffer[buffer_index++];
                msb = bmi08x_accel_stream_buffer[buffer_index++];
                az = (msb << 8) | lsb;

                printf("ACCEL[%d]  Acc_Raw_X : %-5d  Acc_Raw_Y : %-5d   Acc_Raw_Z : %-5d   ", idx, ax, ay, az);

                if (bmi08xdev.variant == BMI085_VARIANT)
                {
                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
                    x = lsb_to_mps2(ax, 16, 16);
                    y = lsb_to_mps2(ay, 16, 16);
                    z = lsb_to_mps2(az, 16, 16);
                }
                else if (bmi08xdev.variant == BMI088_VARIANT)
                {
                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
                    x = lsb_to_mps2(ax, 24, 16);
                    y = lsb_to_mps2(ay, 24, 16);
                    z = lsb_to_mps2(az, 24, 16);
                }

                /* Print the data in m/s2. */
                printf("\t  Acc_ms2_X = %4.2f   Acc_ms2_Y = %4.2f   Acc_ms2_Z = %4.2f\n", x, y, z);
            }
        }
    }

    if (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL)
    {
        memset(&bmi08x_gyro_stream_buffer[0], 0, COINES_STREAM_RSP_BUF_SIZE);
        rslt = coines_read_stream_sensor_data(2, 1, &bmi08x_gyro_stream_buffer[0], &valid_sample_count);
        if (rslt == COINES_SUCCESS)
        {
            buffer_index = 0;
            printf("\n\nGYRO DATA \n");
            printf("Gyro data in LSB units and degrees per second\n");
            printf("Gyro data range : 250 dps for BMI085 and BMI088\n\n");

            if (valid_sample_count > 100)
            {
                valid_sample_count = 100;
            }

            for (idx = 0; idx < valid_sample_count; idx++)
            {

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gx = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gy = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gz = (msb << 8) | lsb;

                printf("GYRO[%d]  Gyr_Raw_X : %-5d   Gyr_Raw_Y : %-5d    Gyr_Raw_Z : %-5d   ", idx, gx, gy, gz);

                /* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
                x = lsb_to_dps(gx, 250, 16);
                y = lsb_to_dps(gy, 250, 16);
                z = lsb_to_dps(gz, 250, 16);

                /* Print the data in dps. */
                printf("\t  Gyr_DPS_X = %4.2f   Gyr_DPS_Y = %4.2f   Gyr_DPS_Z = %4.2f\n", x, y, z);
            }
        }
    }
}

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x(void)
{
    int8_t rslt;

    rslt = bmi08a_init(&bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_init", rslt);

    if (rslt == BMI08X_OK)
    {
        rslt = bmi08g_init(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_init", rslt);
    }

    if (rslt == BMI08X_OK)
    {
        printf("Uploading config file !\n");
        rslt = bmi08a_load_config_file(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);
    }

    if (rslt == BMI08X_OK)
    {
        bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;

        if (bmi08xdev.variant == BMI085_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08xdev.variant == BMI088_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;

        rslt = bmi08a_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08a_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

        bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
        bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
        bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
        bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
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

    /* Interface given as parameter
     *           For I2C : BMI08X_I2C_INTF
     *           For SPI : BMI08X_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_I2C_INTF, BMI085_VARIANT);
    bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);

    if (rslt == BMI08X_OK)
    {
        rslt = init_bmi08x();
        bmi08x_error_codes_print_result("init_bmi08x", rslt);

        if (rslt == BMI08X_OK)
        {
            /* Send streaming settings */
            send_stream_settings();

            /* Start polling streaming */
            coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);

            /* Read sensor data */
            read_sensor_data();

            /* Stop polling streaming */
            coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);

            /* Wait for 100 ms */
            coines_delay_msec(100);
        }
    }

    bmi08x_coines_deinit();

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    float half_scale = ((1 << bit_width) / 2.0f);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}
