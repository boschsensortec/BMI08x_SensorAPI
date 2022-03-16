/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi08x.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!                  Macros                                                   */

/* Buffer size allocated to store raw FIFO data */
#define BMI08X_FIFO_RAW_DATA_BUFFER_SIZE        UINT16_C(600)

/* Length of data to be read from FIFO */
#define BMI08X_FIFO_RAW_DATA_USER_LENGTH        UINT16_C(600)

/* Number of Gyro frames to be extracted from FIFO */
#define BMI08X_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro[100] = { { 0 } };

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

/******************************************************************************/
/*!                   Static Functions                                        */

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
        bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_100_HZ;

        if (bmi08xdev.variant == BMI085_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_2G;
        }
        else if (bmi08xdev.variant == BMI088_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08a_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

        bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_32_ODR_100_HZ;
        bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_125_DPS;
        bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_32_ODR_100_HZ;
        bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
        coines_delay_msec(1);
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

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /* Enable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg*)&gyro_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

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

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08X_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /* Disable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg*)&gyro_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

    return rslt;
}

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Gyroscope fifo configurations */
    struct bmi08x_gyr_fifo_config gyr_conf = { 0 };

    /* Fifo frame structure */
    struct bmi08x_fifo_frame fifo = { 0 };

    /* Number of gyroscope frames */
    uint16_t gyro_length = BMI08X_FIFO_EXTRACTED_DATA_FRAME_COUNT;

    /* Variable to index bytes */
    uint16_t idx = 0;

    /* Variable that holds loop count of fifo example */
    uint8_t try = 1;

    /* Variable that holds interrupt status */
    uint8_t status = 0;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI08X_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

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
            /* Enable data ready interrupts */
            rslt = enable_bmi08x_interrupt();
            bmi08x_error_codes_print_result("enable_bmi08x_interrupt", rslt);

            printf("Gyro FIFO full interrupt data\n");
            if (rslt == BMI08X_OK)
            {
                gyr_conf.mode = BMI08X_GYRO_FIFO_MODE;
                gyr_conf.tag = BMI08X_GYRO_FIFO_TAG_DISABLED;

                rslt = bmi08g_set_fifo_config(&gyr_conf, &bmi08xdev);
                bmi08x_error_codes_print_result("bmi08g_set_fifo_config", rslt);

                /* Update FIFO structure */
                fifo.data = fifo_data;
                fifo.length = BMI08X_FIFO_RAW_DATA_USER_LENGTH;

                while (try <= 3)
                {
                    rslt = bmi08g_get_data_int_status(&status, &bmi08xdev);
                    bmi08x_error_codes_print_result("bmi08g_set_fifo_config", rslt);

                    if (status & BMI08X_GYRO_FIFO_FULL_INT)
                    {
                        printf("\nIteration : %d\n", try);

                        gyro_length = BMI08X_FIFO_EXTRACTED_DATA_FRAME_COUNT;

                        rslt = bmi08g_get_fifo_config(&gyr_conf, &bmi08xdev);
                        bmi08x_error_codes_print_result("bmi08g_get_fifo_config", rslt);

                        rslt = bmi08g_get_fifo_length(&gyr_conf, &fifo);
                        bmi08x_error_codes_print_result("bmi08g_get_fifo_length", rslt);

                        /* Read FIFO data */
                        rslt = bmi08g_read_fifo_data(&fifo, &bmi08xdev);
                        bmi08x_error_codes_print_result("bmi08g_read_fifo_data", rslt);

                        printf("FIFO buffer size : %d\n", BMI08X_FIFO_RAW_DATA_BUFFER_SIZE);
                        printf("FIFO length available : %d\n\n", fifo.length);

                        printf("Requested data frames before parsing: %d\n", gyro_length);

                        /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                        bmi08g_extract_gyro(bmi08x_gyro, &gyro_length, &gyr_conf, &fifo);
                        bmi08x_error_codes_print_result("bmi08g_extract_gyro", rslt);

                        printf("Parsed gyroscope frames: %d\n", gyr_conf.frame_count);

                        /* Print the parsed gyroscope data from the FIFO buffer */
                        for (idx = 0; idx < gyr_conf.frame_count; idx++)
                        {
                            printf("GYRO[%d] X : %d\t Y : %d\t Z : %d\n",
                                   idx,
                                   bmi08x_gyro[idx].x,
                                   bmi08x_gyro[idx].y,
                                   bmi08x_gyro[idx].z);
                        }

                        try++;
                    }
                }
            }
        }

        /* Disable data ready interrupts */
        rslt = disable_bmi08x_interrupt();
        bmi08x_error_codes_print_result("disable_bmi08x_interrupt", rslt);
    }

    bmi08x_coines_deinit();

    return rslt;
}
