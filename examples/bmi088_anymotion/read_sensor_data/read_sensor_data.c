/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "coines.h"
#include "bmi088_anymotion.h"
#include "common.h"

/*********************************************************************/
/*                              Macros                               */
/*********************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*********************************************************************/
/*                        Global variables                           */
/*********************************************************************/

struct bmi08_sensor_data bmi08_accel;

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

/*!
 * @brief This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*!
 * @brief This internal API is used to configure accel data ready interrupts
 */
static void configure_accel_data_ready_interrupts(struct bmi08_dev *bmi08dev);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in Meter Per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

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
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
        bmi08dev->accel_cfg.range = BMI088_ANYMOTION_ACCEL_RANGE_3G;

        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(bmi08dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi088_anymotion_set_meas_conf(bmi08dev);
            bmi08_check_rslt("bmi088_anymotion_set_meas_conf", rslt);
        }
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
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 */
int main(void)
{
    struct bmi08_dev bmi08;
    uint8_t status = 0;
    int8_t rslt;
    float x, y, z;
    uint8_t times_to_read = 0;

    /* Interface reference is given as a parameter
     *         For I2C : BMI08_I2C_INTF
     *         For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_SPI_INTF);
    bmi08_check_rslt("bmi08_interface_init", rslt);

    /* Initialize the sensors */
    init_bmi08(&bmi08);

    configure_accel_data_ready_interrupts(&bmi08);

    for (;;)
    {
        /* Read accel data ready interrupt status */
        rslt = bmi08a_get_data_int_status(&status, &bmi08);
        bmi08_check_rslt("bmi08a_get_data_int_status", rslt);

        if ((status & BMI08_ACCEL_DATA_READY_INT) && (times_to_read < 50))
        {
            rslt = bmi088_anymotion_get_data(&bmi08_accel, &bmi08);
            bmi08_check_rslt("bmi088_anymotion_get_data", rslt);

            printf("\nAccel Data set : %d\n", times_to_read);

            printf("\nAcc_X = %d\t", bmi08_accel.x);
            printf("Acc_Y = %d\t", bmi08_accel.y);
            printf("Acc_Z = %d", bmi08_accel.z);

            /* Converting lsb to meter per second squared for 16 bit accelerometer at 3G range. */
            x = lsb_to_mps2(bmi08_accel.x, (float)3, 16);
            y = lsb_to_mps2(bmi08_accel.y, (float)3, 16);
            z = lsb_to_mps2(bmi08_accel.z, (float)3, 16);

            /* Print the data in m/s2. */
            printf("\nAcc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);

            times_to_read++;
        }

        if (times_to_read == 50)
        {
            break;
        }
    }

    bmi08_coines_deinit();

    return rslt;
}
