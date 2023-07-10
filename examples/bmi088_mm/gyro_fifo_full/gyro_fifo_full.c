/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>

#include "bmi088_mm.h"
#include "common.h"

/******************************************************************************/
/*!                  Macros                                                   */

/* Buffer size allocated to store raw FIFO data */
#define BMI08_FIFO_RAW_DATA_BUFFER_SIZE        UINT16_C(800)

/* Length of data to be read from FIFO */
#define BMI08_FIFO_RAW_DATA_USER_LENGTH        UINT16_C(800)

/* Number of Gyro frames to be extracted from FIFO */
#define BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

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
static void init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel & gyro) */
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
    {
        printf("BMI08 initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);
        bmi08_check_rslt("bmi08a_soft_reset", rslt);
    }
    else
    {
        printf("BMI08 initialization failure!\n");
        exit(COINES_E_FAILURE);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
        bmi08_check_rslt("bmi08g_set_power_mode", rslt);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
        bmi08dev->gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
        rslt = bmi08g_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi08g_set_meas_conf", rslt);
        coines_delay_msec(1);
    }
}

/*!
 *  @brief This API is used to enable bmi08 interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* BMI08 gyro int config */
    struct bmi08_gyro_int_channel_cfg gyro_int_config;

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg*)&gyro_int_config, bmi08dev);
    bmi08_check_rslt("bmi08g_set_int_config", rslt);

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
static int8_t disable_bmi08_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* BMI08 gyro int config */
    struct bmi08_gyro_int_channel_cfg gyro_int_config;

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
    gyro_int_config.int_type = BMI08_GYRO_INT_FIFO_FULL;
    gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable gyro fifo interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg*)&gyro_int_config, bmi08dev);
    bmi08_check_rslt("bmi08g_set_int_config", rslt);

    return rslt;
}

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Structure instance of bmi08_dev */
    struct bmi08_dev bmi08;

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

    /* Variable to hold the bmi08 gyro data */
    struct bmi08_sensor_data bmi08_gyro[BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT] = { { 0 } };

    /* Interface given as parameter
     * For I2C : BMI08_I2C_INTF
     * For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_SPI_INTF);
    bmi08_check_rslt("bmi08_interface_init", rslt);

    init_bmi08(&bmi08);

    if (rslt == BMI08_OK)
    {
        /* Enable FIFO full interrupt */
        rslt = enable_bmi08_interrupt(&bmi08);
        bmi08_check_rslt("enable_bmi08_interrupt", rslt);

        printf("Gyro FIFO full interrupt data\n");
        if (rslt == BMI08_OK)
        {
            gyr_conf.mode = BMI08_GYRO_FIFO_MODE;
            gyr_conf.tag = BMI08_GYRO_FIFO_TAG_DISABLED;

            rslt = bmi08g_set_fifo_config(&gyr_conf, &bmi08);
            bmi08_check_rslt("bmi08g_set_fifo_config", rslt);

            /* Update FIFO structure */
            fifo.data = fifo_data;
            fifo.length = BMI08_FIFO_RAW_DATA_USER_LENGTH;

            while (try <= 10)
            {
                rslt = bmi08g_get_data_int_status(&status, &bmi08);
                bmi08_check_rslt("bmi08g_set_fifo_config", rslt);

                if (status & BMI08_GYRO_FIFO_FULL_INT)
                {
                    printf("\nIteration : %d\n", try);

                    gyro_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;

                    rslt = bmi08g_get_fifo_config(&gyr_conf, &bmi08);
                    bmi08_check_rslt("bmi08g_get_fifo_config", rslt);

                    rslt = bmi08g_get_fifo_length(&gyr_conf, &fifo);
                    bmi08_check_rslt("bmi08g_get_fifo_length", rslt);

                    /* Read FIFO data */
                    rslt = bmi08g_read_fifo_data(&fifo, &bmi08);
                    bmi08_check_rslt("bmi08g_read_fifo_data", rslt);

                    printf("Requested data frames before parsing: %d\n", gyro_length);
                    printf("FIFO length available : %d\n", fifo.length);

                    /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                    bmi08g_extract_gyro(bmi08_gyro, &gyro_length, &gyr_conf, &fifo);

                    printf("Parsed gyroscope frames: %d\n", gyr_conf.frame_count);

                    /* Print the parsed gyroscope data from the FIFO buffer */
                    for (idx = 0; idx < gyr_conf.frame_count; idx++)
                    {
                        printf("GYRO[%d] X : %d\t Y : %d\t Z : %d\n",
                               idx,
                               bmi08_gyro[idx].x,
                               bmi08_gyro[idx].y,
                               bmi08_gyro[idx].z);
                    }

                    try++;
                }
            }
        }
    }

    /* Disable FIFO full interrupts */
    rslt = disable_bmi08_interrupt(&bmi08);

    bmi08_coines_deinit();

    return rslt;
}
