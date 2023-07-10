/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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
#define BMI08_FIFO_RAW_DATA_BUFFER_SIZE        UINT16_C(600)

/* Length of data to be read from FIFO */
#define BMI08_FIFO_RAW_DATA_USER_LENGTH        UINT16_C(600)

/* Number of Gyro frames to be extracted from FIFO */
#define BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro[100] = { { 0 } };

/*! bmi08 gyro int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config;

/******************************************************************************/
/*!                   Static Functions                                        */

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
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_2G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08xa_set_meas_conf", rslt);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_125_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_meas_conf", rslt);
        coines_delay_msec(1);
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

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg*)&gyro_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);

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

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg*)&gyro_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);

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
    struct bmi08_gyr_fifo_config gyr_conf = { 0 };

    /* Fifo frame structure */
    struct bmi08_fifo_frame fifo = { 0 };

    /* Number of gyroscope frames */
    uint16_t gyro_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;

    /* Variable to index bytes */
    uint16_t idx = 0;

    /* Variable that holds loop count of fifo example */
    uint8_t try = 1;

    /* Variable that holds interrupt status */
    uint8_t status = 0;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI08_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

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

        if (rslt == BMI08_OK)
        {
            /* Enable data ready interrupts */
            rslt = enable_bmi08_interrupt();
            bmi08_error_codes_print_result("enable_bmi08_interrupt", rslt);

            printf("Gyro FIFO full interrupt data\n");
            if (rslt == BMI08_OK)
            {
                gyr_conf.mode = BMI08_GYRO_FIFO_MODE;
                gyr_conf.tag = BMI08_GYRO_FIFO_TAG_DISABLED;

                rslt = bmi08g_set_fifo_config(&gyr_conf, &bmi08dev);
                bmi08_error_codes_print_result("bmi08g_set_fifo_config", rslt);

                /* Update FIFO structure */
                fifo.data = fifo_data;
                fifo.length = BMI08_FIFO_RAW_DATA_USER_LENGTH;

                while (try <= 3)
                {
                    rslt = bmi08g_get_data_int_status(&status, &bmi08dev);
                    bmi08_error_codes_print_result("bmi08g_set_fifo_config", rslt);

                    if (status & BMI08_GYRO_FIFO_FULL_INT)
                    {
                        printf("\nIteration : %d\n", try);

                        gyro_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;

                        rslt = bmi08g_get_fifo_config(&gyr_conf, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08g_get_fifo_config", rslt);

                        rslt = bmi08g_get_fifo_length(&gyr_conf, &fifo);
                        bmi08_error_codes_print_result("bmi08g_get_fifo_length", rslt);

                        /* Read FIFO data */
                        rslt = bmi08g_read_fifo_data(&fifo, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08g_read_fifo_data", rslt);

                        printf("FIFO buffer size : %d\n", BMI08_FIFO_RAW_DATA_BUFFER_SIZE);
                        printf("FIFO length available : %d\n\n", fifo.length);

                        printf("Requested data frames before parsing: %d\n", gyro_length);

                        /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                        bmi08g_extract_gyro(bmi08_gyro, &gyro_length, &gyr_conf, &fifo);
                        bmi08_error_codes_print_result("bmi08g_extract_gyro", rslt);

                        printf("Parsed gyroscope frames: %d\n", gyr_conf.frame_count);

                        printf("\nFrame_Count, X, Y, Z\n");

                        /* Print the parsed gyroscope data from the FIFO buffer */
                        for (idx = 0; idx < gyr_conf.frame_count; idx++)
                        {
                            printf("%d, %d, %d, %d\n", idx, bmi08_gyro[idx].x, bmi08_gyro[idx].y, bmi08_gyro[idx].z);
                        }

                        try++;
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
