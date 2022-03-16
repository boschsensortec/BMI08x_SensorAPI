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

/*! @brief Sample file how to stream bmi08x sensor data based on data ready interrupt using LIB COINES */

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

/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

/*!accel streaming configuration */
struct coines_streaming_config accel_stream_config;

/*! gyro stream configuration*/
struct coines_streaming_config gyro_stream_config;

/*! streaming accel sensor register block */
struct coines_streaming_blocks accel_stream_block;

/*! streaming gyro sensor register block */
struct coines_streaming_blocks gyro_stream_block;

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
 * @brief   This internal API is used to send stream settings
 */
static void send_stream_settings(void);

/*!
 * @brief    This internal API is used to read sensor data
 */
void read_sensor_data(void);

/*!
 * @brief   This internal API is used to initialize the bmi08x sensor
 */
static int8_t init_bmi08x(void);

/*!
 * @brief   This internal API is used to enable the bmi08x interrupt
 */
static int8_t enable_bmi08x_interrupt();

/*!
 * @brief    This internal API is used to disable the bmi08x interrupt
 */
static int8_t disable_bmi08x_interrupt();

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
static void send_stream_settings()
{
    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        accel_stream_config.intf = COINES_SENSOR_INTF_I2C;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        accel_stream_config.intf = COINES_SENSOR_INTF_SPI;
    }

    accel_stream_config.i2c_bus = COINES_I2C_BUS_0; /*if intf is I2C */
    accel_stream_config.spi_bus = COINES_SPI_BUS_0; /* if intf is SPI */

    /*for I2C */
    accel_stream_config.dev_addr = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
    accel_stream_config.cs_pin = COINES_SHUTTLE_PIN_8;
    accel_stream_config.int_pin = COINES_SHUTTLE_PIN_21;
    accel_stream_config.int_timestamp = 1;
    accel_stream_block.no_of_blocks = 1;
    accel_stream_block.reg_start_addr[0] = BMI08X_REG_ACCEL_X_LSB; /*accel data start address */

    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        accel_stream_block.no_of_data_bytes[0] = 6;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        accel_stream_block.no_of_data_bytes[0] = 7; /* actual data length is 6 bytes 1 byte needed to initiate the spi
                                                     * communication */
    }

    coines_config_streaming(1, &accel_stream_config, &accel_stream_block);

    if (bmi08xdev.intf == BMI08X_I2C_INTF)
    {
        gyro_stream_config.intf = COINES_SENSOR_INTF_I2C;
    }
    else if (bmi08xdev.intf == BMI08X_SPI_INTF)
    {
        gyro_stream_config.intf = COINES_SENSOR_INTF_SPI;
    }

    gyro_stream_config.i2c_bus = COINES_I2C_BUS_0; /*if intf is I2C */
    gyro_stream_config.spi_bus = COINES_SPI_BUS_0; /* if intf is SPI */

    /*for I2C */
    gyro_stream_config.dev_addr = BMI08X_GYRO_I2C_ADDR_PRIMARY;
    gyro_stream_config.cs_pin = COINES_SHUTTLE_PIN_14;
    gyro_stream_config.int_pin = COINES_SHUTTLE_PIN_22;
    gyro_stream_config.int_timestamp = 1;
    gyro_stream_block.no_of_blocks = 1;
    gyro_stream_block.reg_start_addr[0] = BMI08X_REG_GYRO_X_LSB; /*gyro data start address */
    gyro_stream_block.no_of_data_bytes[0] = 6;

    coines_config_streaming(2, &gyro_stream_config, &gyro_stream_block);
}

/*!
 *  @brief This internal API is used to read sensor data
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
void read_sensor_data()
{
    int16_t rslt;
    uint8_t lsb, msb;
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t valid_sample_count = 0;
    uint32_t packet_count = 0;
    uint64_t accel_time_stamp = 0;
    uint64_t gyro_time_stamp = 0;
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
                packet_count = 0;

                packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 24;
                packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 16;
                packet_count |= bmi08x_accel_stream_buffer[buffer_index++] << 8;
                packet_count |= bmi08x_accel_stream_buffer[buffer_index++];
                if (bmi08xdev.intf == BMI08X_SPI_INTF)
                {
                    buffer_index++; /*dummy byte; */
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

                if (accel_stream_config.int_timestamp)
                {
                    accel_time_stamp = 0;

                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 40;
                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 32;
                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 24;
                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 16;
                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++] << 8;
                    accel_time_stamp |= (uint64_t)bmi08x_accel_stream_buffer[buffer_index++];
                }

                /* Timestamp in microseconds can be obtained by following formula
                 *   Timestamp(us) = (48bit_timestamp / 30)
                 */
                printf("ACCEL[%d]  Acc_Raw_X : %5d  Acc_Raw_Y : %5d   Acc_Raw_Z : %5d   T(us) : %ld   ",
                       packet_count,
                       ax,
                       ay,
                       az,
                       (long)(accel_time_stamp / 30));

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

                packet_count = 0;

                packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 24;
                packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 16;
                packet_count |= bmi08x_gyro_stream_buffer[buffer_index++] << 8;
                packet_count |= bmi08x_gyro_stream_buffer[buffer_index++];

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gx = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gy = (msb << 8) | lsb;

                lsb = bmi08x_gyro_stream_buffer[buffer_index++];
                msb = bmi08x_gyro_stream_buffer[buffer_index++];
                gz = (msb << 8) | lsb;

                if (gyro_stream_config.int_timestamp)
                {
                    gyro_time_stamp = 0;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 40;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 32;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 24;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 16;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++] << 8;
                    gyro_time_stamp |= (uint64_t)bmi08x_gyro_stream_buffer[buffer_index++];
                }

                /* Timestamp in microseconds can be obtained by following formula
                 *   Timestamp(us) = (48bit_timestamp / 30)
                 */
                printf("GYRO[%d]  Gyr_Raw_X : %5d   Gyr_Raw_Y : %5d   Gyr_Raw_Z : %5d   T(us) : %ld   ",
                       packet_count,
                       gx,
                       gy,
                       gz,
                       (long)(gyro_time_stamp / 30));

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
        printf("BMI08x initialization success !\n");
        printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);

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

        if ((rslt == BMI08X_OK) &&
            (bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_SUSPEND &&
             (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_SUSPEND ||
              bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_DEEP_SUSPEND)))
        {
            printf("Accel and gyro sensors are in suspend mode\n Use them in active/normal mode !!");
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08x_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t disable_bmi08x_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
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
        /* Initialize the sensor */
        rslt = init_bmi08x();
        bmi08x_error_codes_print_result("init_bmi08x", rslt);

        /* Send streaming settings */
        send_stream_settings();

        /* Enables 48-bit system timer */
        coines_trigger_timer(COINES_TIMER_START, COINES_TIMESTAMP_ENABLE);

        /* Wait for 10 ms */
        coines_delay_msec(10);

        /* Start interrupt streaming */
        coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_START);

        /* Enable data ready interrupts */
        rslt = enable_bmi08x_interrupt();
        bmi08x_error_codes_print_result("enable_bmi08x_interrupt", rslt);

        if (rslt == BMI08X_OK)
        {
            /* Read sensor data */
            read_sensor_data();

            /* Stop interrupt streaming */
            coines_start_stop_streaming(COINES_STREAMING_MODE_INTERRUPT, COINES_STREAMING_STOP);

            /* Stop timer */
            coines_trigger_timer(COINES_TIMER_STOP, COINES_TIMESTAMP_DISABLE);

            /* Wait for 100 ms */
            coines_delay_msec(100);

            /* Disable data ready interrupts */
            rslt = disable_bmi08x_interrupt();
            bmi08x_error_codes_print_result("disable_bmi08x_interrupt", rslt);
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
