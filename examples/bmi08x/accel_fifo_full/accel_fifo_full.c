/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdint.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"
#include "bmi08x.h"
#include "common.h"

/*********************************************************************/
/*                              Macros                               */
/*********************************************************************/

/* Buffer size allocated to store raw FIFO data for accel */
#define BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE             UINT16_C(1024)

/* Length of data to be read from FIFO for accel */
#define BMI08_ACC_FIFO_RAW_DATA_USER_LENGTH             UINT16_C(1024)

/* Number of Accel frames to be extracted from FIFO */
#define BMI08_ACC_FIFO_FULL_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

/*********************************************************************/
/*                       Global variables                            */
/*********************************************************************/

/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel[100] = { { 0 } };

/*! bmi08 accel int config */
struct bmi08_accel_int_channel_cfg accel_int_config;

/*********************************************************************/
/*                      Static function declarations                 */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor with default
 */
static int8_t init_bmi08(void);

/*********************************************************************/
/*                            Functions                              */
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

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;

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

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_FIFO_FULL;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

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
    accel_int_config.int_type = BMI08_ACCEL_INT_FIFO_FULL;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

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

    uint8_t status = 0;

    /* Initialize FIFO frame structure */
    struct bmi08_fifo_frame fifo_frame = { 0 };

    /* To configure the FIFO accel configurations */
    struct bmi08_accel_fifo_config config;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Number of accelerometer frames */
    uint16_t accel_length = BMI08_ACC_FIFO_FULL_EXTRACTED_DATA_FRAME_COUNT;

    /* Variable to index bytes */
    uint16_t idx = 0;

    uint8_t try = 1;

    /* Variable to store sensor time value */
    uint32_t sensor_time;

    /* Variable to store available fifo length */
    uint16_t fifo_length;

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

        printf("Accel FIFO full interrupt data\n");

        if (rslt == BMI08_OK)
        {
            config.accel_en = BMI08_ENABLE;

            /* Set FIFO configuration by enabling accelerometer */
            rslt = bmi08a_set_fifo_config(&config, &bmi08dev);
            bmi08_error_codes_print_result("bmi08a_set_fifo_config", rslt);

            while (try <= 3)
            {
                rslt = bmi08a_get_data_int_status(&status, &bmi08dev);
                bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

                if (status & BMI08_ACCEL_FIFO_FULL_INT)
                {
                    printf("\nIteration : %d\n", try);

                    /* Update FIFO structure */
                    fifo_frame.data = fifo_data;
                    fifo_frame.length = BMI08_ACC_FIFO_RAW_DATA_USER_LENGTH;

                    accel_length = BMI08_ACC_FIFO_FULL_EXTRACTED_DATA_FRAME_COUNT;

                    rslt = bmi08a_get_fifo_length(&fifo_length, &bmi08dev);
                    bmi08_error_codes_print_result("bmi08a_get_fifo_length", rslt);

                    printf("FIFO buffer size : %d\n", fifo_frame.length);
                    printf("FIFO length available : %d\n\n", fifo_length);

                    printf("Requested data frames before parsing: %d\n", accel_length);

                    if (rslt == BMI08_OK)
                    {
                        /* Read FIFO data */
                        rslt = bmi08a_read_fifo_data(&fifo_frame, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08a_read_fifo_data", rslt);

                        /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                        rslt = bmi08a_extract_accel(bmi08_accel, &accel_length, &fifo_frame, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08a_extract_accel", rslt);

                        printf("Parsed accelerometer frames: %d\n", accel_length);

                        printf("\nFrame_Count, X, Y, Z\n");

                        /* Print the parsed accelerometer data from the FIFO buffer */
                        for (idx = 0; idx < accel_length; idx++)
                        {
                            printf("%d, %d, %d, %d\n", idx, bmi08_accel[idx].x, bmi08_accel[idx].y, bmi08_accel[idx].z);
                        }

                        rslt = bmi08a_get_sensor_time(&bmi08dev, &sensor_time);
                        bmi08_error_codes_print_result("bmi08a_get_sensor_time", rslt);

                        printf("Sensor time : %.4lf   s\n", (sensor_time * BMI08_SENSORTIME_RESOLUTION));
                    }

                    try++;
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
