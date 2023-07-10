/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "bmi088_anymotion.h"
#include "common.h"

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*!
 * @brief This internal API is used to configure accel data ready interrupts
 */
static void configure_accel_data_ready_interrupts(struct bmi08_dev *bmi08dev);

/*********************************************************************/
/*                          Functions                                */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel) */
    if (bmi088_anymotion_init(bmi08dev) == BMI08_OK)
    {
        printf("BMI08 initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);
    }
    else
    {
        printf("BMI08 initialization failure!\n");
        exit(COINES_E_FAILURE);
    }

    if (rslt == BMI08_OK)
    {
        /* Set accel power mode */
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        printf("Uploading config file !\n");
        rslt = bmi08a_load_config_file(bmi08dev);
    }

    /* API uploads the bmi08 config file onto the device */
    if (rslt == BMI08_OK)
    {
        printf("Upload done !\n");

        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_200_HZ;
        bmi08dev->accel_cfg.range = BMI088_ANYMOTION_ACCEL_RANGE_3G;
        rslt = bmi088_anymotion_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi088_anymotion_set_meas_conf", rslt);
    }
}

static void configure_accel_data_ready_interrupts(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_accel_int_channel_cfg accel_int_config;

    /* Configure the Interrupt configurations for accel */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi088_anymotion_set_int_config(&accel_int_config, BMI088_ANYMOTION_ACCEL_DATA_RDY_INT, bmi08dev);

    if (rslt != BMI08_OK)
    {
        printf("Failure in interrupt configurations \n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 */
int main(void)
{
    int8_t rslt;
    struct bmi08_dev bmi08;
    uint8_t status = 0;
    struct bmi088_anymotion_remap remap_data = { 0 };
    struct bmi08_sensor_data accel;

    char data_array[13][18] =
    { { 0 }, { "BMI08_X" }, { "BMI08_Y" }, { 0 }, { "BMI08_Z" }, { 0 }, { 0 }, { 0 }, { 0 }, { "BMI08_NEG_X" },
      { "BMI08_NEG_Y" }, { 0 }, { "BMI08_NEG_Z" } };

    /* Interface reference is given as a parameter
     *         For I2C : BMI08_I2C_INTF
     *         For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_SPI_INTF);
    bmi08_check_rslt("bmi08_interface_init", rslt);

    /* Initialize the sensors */
    init_bmi08(&bmi08);

    configure_accel_data_ready_interrupts(&bmi08);

    printf("\nAXIS_REMAP_FUNC_TEST 1\n");
    printf("Get sensor data of re-mapped axes\n");

    rslt = bmi088_anymotion_get_remap_axes(&remap_data, &bmi08);
    bmi08_check_rslt("bmi088_anymotion_get_remap_axes", rslt);

    printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
    printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
    printf("Re-mapped z value = %s\n", data_array[remap_data.z]);

    printf("Expected Re-mapped x value = BMI08_X\n");
    printf("Expected Re-mapped y value = BMI08_Y\n");
    printf("Expected Re-mapped z value = BMI08_Z\n");

    if ((remap_data.x == BMI088_ANYMOTION_X) && (remap_data.y == BMI088_ANYMOTION_Y) &&
        (remap_data.z == BMI088_ANYMOTION_Z))
    {
        printf(">> PASS\n");
    }
    else
    {
        printf(">> FAIL\n");
    }

    printf("Print mapped data\n");

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);

        if (status & BMI08_ACCEL_DATA_READY_INT)
        {
            rslt = bmi088_anymotion_get_data(&accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

            break;
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 2\n");
    printf("Get sensor data of re-mapped axes\n");

    remap_data.x = BMI088_ANYMOTION_NEG_Y;
    remap_data.y = BMI088_ANYMOTION_Z;
    remap_data.z = BMI088_ANYMOTION_NEG_X;

    rslt = bmi088_anymotion_set_remap_axes(&remap_data, &bmi08);
    bmi08_check_rslt("bmi088_anymotion_set_remap_axes", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi088_anymotion_get_remap_axes(&remap_data, &bmi08);
        bmi08_check_rslt("bmi088_anymotion_get_remap_axes", rslt);

        if (rslt == BMI08_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMI08_NEG_Y\n");
        printf("Expected Re-mapped y value = BMI08_Z\n");
        printf("Expected Re-mapped z value = BMI08_NEG_X\n");

        if ((remap_data.x == BMI088_ANYMOTION_NEG_Y) && (remap_data.y == BMI088_ANYMOTION_Z) &&
            (remap_data.z == BMI088_ANYMOTION_NEG_X))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);

        if (status & BMI08_ACCEL_DATA_READY_INT)
        {
            rslt = bmi088_anymotion_get_data(&accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

            break;
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 3\n");
    printf("Get sensor data of re-mapped axes - 2nd combination\n");

    remap_data.x = BMI088_ANYMOTION_NEG_Z;
    remap_data.y = BMI088_ANYMOTION_NEG_X;
    remap_data.z = BMI088_ANYMOTION_Y;

    rslt = bmi088_anymotion_set_remap_axes(&remap_data, &bmi08);
    bmi08_check_rslt("bmi088_anymotion_set_remap_axes", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi088_anymotion_get_remap_axes(&remap_data, &bmi08);
        bmi08_check_rslt("bmi088_anymotion_get_remap_axes", rslt);

        if (rslt == BMI08_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMI08_NEG_Z\n");
        printf("Expected Re-mapped y value = BMI08_NEG_X\n");
        printf("Expected Re-mapped z value = BMI08_Y\n");

        if ((remap_data.x == BMI088_ANYMOTION_NEG_Z) && (remap_data.y == BMI088_ANYMOTION_NEG_X) &&
            (remap_data.z == BMI088_ANYMOTION_Y))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);

        if (status & BMI08_ACCEL_DATA_READY_INT)
        {
            rslt = bmi088_anymotion_get_data(&accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

            break;
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 4\n");
    printf("Get sensor data of re-mapped axes - 3rd combination\n");

    remap_data.x = BMI088_ANYMOTION_Y;
    remap_data.y = BMI088_ANYMOTION_Z;
    remap_data.z = BMI088_ANYMOTION_X;

    rslt = bmi088_anymotion_set_remap_axes(&remap_data, &bmi08);
    bmi08_check_rslt("bmi088_anymotion_set_remap_axes", rslt);
    if (rslt == BMI08_OK)
    {
        rslt = bmi088_anymotion_get_remap_axes(&remap_data, &bmi08);
        bmi08_check_rslt("bmi088_anymotion_get_remap_axes", rslt);

        if (rslt == BMI08_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMI08_Y\n");
        printf("Expected Re-mapped y value = BMI08_Z\n");
        printf("Expected Re-mapped z value = BMI08_X\n");

        if ((remap_data.x == BMI088_ANYMOTION_Y) && (remap_data.y == BMI088_ANYMOTION_Z) &&
            (remap_data.z == BMI088_ANYMOTION_X))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);

        if (status & BMI08_ACCEL_DATA_READY_INT)
        {
            rslt = bmi088_anymotion_get_data(&accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

            break;
        }
    }

    printf("\nAXIS_REMAP_FUNC_TEST 5\n");
    printf("Get sensor data of re-mapped axes - 4th combination\n");

    remap_data.x = BMI088_ANYMOTION_NEG_X;
    remap_data.y = BMI088_ANYMOTION_NEG_Y;
    remap_data.z = BMI088_ANYMOTION_NEG_Z;

    rslt = bmi088_anymotion_set_remap_axes(&remap_data, &bmi08);
    bmi08_check_rslt("bmi088_anymotion_set_remap_axes", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi088_anymotion_get_remap_axes(&remap_data, &bmi08);
        bmi08_check_rslt("bmi088_anymotion_get_remap_axes", rslt);

        if (rslt == BMI08_OK)
        {
            printf("Re-mapped x value = %s\n", data_array[remap_data.x]);
            printf("Re-mapped y value = %s\n", data_array[remap_data.y]);
            printf("Re-mapped z value = %s\n", data_array[remap_data.z]);
        }

        printf("Expected Re-mapped x value = BMI08_NEG_X\n");
        printf("Expected Re-mapped y value = BMI08_NEG_Y\n");
        printf("Expected Re-mapped z value = BMI08_NEG_Z\n");

        if ((remap_data.x == BMI088_ANYMOTION_NEG_X) && (remap_data.y == BMI088_ANYMOTION_NEG_Y) &&
            (remap_data.z == BMI088_ANYMOTION_NEG_Z))
        {
            printf(">> PASS\n");
        }
        else
        {
            printf(">> FAIL\n");
        }
    }

    printf("Print mapped data\n");

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);

        if (status & BMI08_ACCEL_DATA_READY_INT)
        {
            rslt = bmi088_anymotion_get_data(&accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("Accel :: X = %d Y = %d Z = %d\n", accel.x, accel.y, accel.z);

            break;
        }
    }

    bmi08_coines_deinit();

    return rslt;
}
