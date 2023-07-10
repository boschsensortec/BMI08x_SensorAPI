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

    rslt = bmi088_anymotion_init(bmi08dev);

    /* Initialize bmi08 sensors (accel) */
    if (rslt == BMI08_OK)
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
        bmi08dev->accel_cfg.range = BMI088_ANYMOTION_ACCEL_RANGE_6G;
        rslt = bmi088_anymotion_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi088_anymotion_set_meas_conf", rslt);
    }
}

static void configure_bmi088_any_motion_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi088_anymotion_anymotion_cfg any_motion_cfg = { 0 };
    struct bmi08_accel_int_channel_cfg no_motion_int_cfg;

    /* Configure any-motion settings */
    any_motion_cfg.threshold = 0xAA; /* (0.124g * 2^15)/24g = 0xAA */
    any_motion_cfg.duration = 5; /* 100ms/20 = 5 */
    any_motion_cfg.enable = 1;
    any_motion_cfg.x_en = 1;
    any_motion_cfg.y_en = 1;
    any_motion_cfg.z_en = 1;
    rslt = bmi088_anymotion_configure_anymotion(any_motion_cfg, bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Map any-motion interrupt to INT1 */
        no_motion_int_cfg.int_channel = BMI08_INT_CHANNEL_1;
        no_motion_int_cfg.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        no_motion_int_cfg.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        no_motion_int_cfg.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
        rslt = bmi088_anymotion_set_int_config(&no_motion_int_cfg, BMI088_ANYMOTION_ANYMOTION_INT, bmi08dev);
        bmi08_check_rslt("bmi088_anymotion_set_int_config", rslt);
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

    configure_bmi088_any_motion_interrupt(&bmi08);

    printf("Move board to detect any-motion\n");

    for (;;)
    {
        rslt = bmi088_anymotion_get_feat_int_status(&status, &bmi08);
        if (status & BMI088_ANYMOTION_ACCEL_ANY_MOT_INT)
        {
            printf("Any-motion detected %d\n", interrupt_count);
            interrupt_count++;
            if (interrupt_count == 10)
            {
                printf("Any-motion testing done. Exiting! \n");
                break;
            }
        }
    }

    bmi08_coines_deinit();

    return rslt;
}
