/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "bmi088_mm.h"
#include "common.h"

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

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

    /* Initialize bmi08 sensors (accel & gyro) */
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
    {
        printf("BMI08 initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

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
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
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

        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_24G;
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_50_HZ;
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
        rslt = bmi088_mma_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi088_mma_set_meas_conf", rslt);
    }
}

static void configure_bmi08_high_g_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi088_mm_high_g_cfg high_g_cfg;
    struct bmi08_accel_int_channel_cfg high_g_int_cfg;

    rslt = bmi088_mma_get_high_g_config(&high_g_cfg, bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Configure high-g settings */
        high_g_cfg.threshold = 4000; /* 4000*24g/32768 = 2.93g */
        high_g_cfg.hysteresis = 2000; /* 2000*24g/32768 = 1.46g */
        high_g_cfg.duration = 30; /* 150 ms --> 150/5       */
        high_g_cfg.enable = 1;
        high_g_cfg.select_x = 1;
        high_g_cfg.select_y = 1;
        high_g_cfg.select_z = 1;
        rslt = bmi088_mma_set_high_g_config(&high_g_cfg, bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        /* Map high-g interrupt to INT1 */
        high_g_int_cfg.int_channel = BMI08_INT_CHANNEL_1;
        high_g_int_cfg.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        high_g_int_cfg.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        high_g_int_cfg.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
        rslt = bmi088_mma_set_int_config(&high_g_int_cfg, BMI088_MM_HIGH_G_INT, bmi08dev);
        bmi08_check_rslt("bmi088_mma_set_int_config", rslt);
    }
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 */
int main(void)
{
    struct bmi08_dev bmi08;
    struct bmi088_mm_high_g_out high_g_out = { 0 };
    int8_t rslt;
    uint8_t status = 0;
    uint8_t interrupt_count = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMI08_I2C_INTF
     *         For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_I2C_INTF);
    bmi08_check_rslt("bmi08_interface_init", rslt);

    /* Initialize the sensors */
    init_bmi08(&bmi08);

    configure_bmi08_high_g_interrupt(&bmi08);

    printf("Perform motion to detect High-g\n");
    printf("Direction Value : 1 for negative axis, 0 for positive axis\n\n");

    for (;;)
    {
        rslt = bmi088_mma_get_feat_int_status(&status, &bmi08);
        bmi08_check_rslt("bmi088_mma_get_feat_int_status", rslt);

        if (status & BMI088_MM_ACCEL_HIGH_G_INT)
        {
            rslt = bmi088_mma_get_high_g_output(&high_g_out, &bmi08);
            bmi08_check_rslt("bmi088_mma_get_high_g_output", rslt);

            printf("High-g detection %d :: \t", interrupt_count);

            if (high_g_out.x == 1)
            {
                printf("Axis X in Direction %d\n", high_g_out.direction);
            }
            else if (high_g_out.y == 1)
            {
                printf("Axis Y in Direction %d\n", high_g_out.direction);
            }
            else if (high_g_out.z == 1)
            {
                printf("Axis Z in Direction %d\n", high_g_out.direction);
            }
            else
            {
                printf("Invalid Output\n");
            }

            interrupt_count++;
            if (interrupt_count == 10)
            {
                printf("High-g testing done. Exiting! \n");
                break;
            }
        }
    }

    bmi08_coines_deinit();

    return rslt;
}
