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
#include <string.h>
#include <math.h>

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
unsigned char data_sync_int = false;

/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! bmi08 int config */
struct bmi08_int_cfg int_config;

/*Data Sync configuration object*/
struct bmi08_data_sync_cfg sync_cfg;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro;

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
 * @brief    This internal API is used to initialize the bmi08 sensor
 */
static int8_t init_bmi08(void);

/*!
 * @brief    BMI08x data sync. interrupt callback
 */
void bmi08_data_sync_int();

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor with default.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08(void)
{
    int8_t rslt;

    /* Initialize bmi08a */
    rslt = bmi08xa_init(&bmi08dev);
    bmi08_error_codes_print_result("bmi08xa_init", rslt);

    /* Initialize bmi08g */
    rslt = bmi08g_init(&bmi08dev);
    bmi08_error_codes_print_result("bmi08g_init", rslt);

    if (rslt == BMI08_OK)
    {
        printf("BMI08x initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08dev.gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_soft_reset", rslt);

        printf("Uploading BMI08X data synchronization feature config !\n");
        rslt = bmi08a_load_config_file(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_load_config_file", rslt);

        /* Set accel power mode */
        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

        if (rslt == BMI08_OK)
        {
            bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
            rslt = bmi08g_set_power_mode(&bmi08dev);
            bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);
        }

        if ((bmi08dev.accel_cfg.power == BMI08_ACCEL_PM_ACTIVE) && (bmi08dev.gyro_cfg.power == BMI08_GYRO_PM_NORMAL))
        {
            /* API uploads the bmi08 config file onto the device */
            if (rslt == BMI08_OK)
            {
                /* Assign accel range setting */
                if (bmi08dev.variant == BMI085_VARIANT)
                {
                    bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
                }
                else if (bmi08dev.variant == BMI088_VARIANT)
                {
                    bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
                }

                /* Assign gyro range setting */
                bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;

                /* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
                sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;

                rslt = bmi08xa_configure_data_synchronization(sync_cfg, &bmi08dev);
                bmi08_error_codes_print_result("bmi08xa_configure_data_synchronization", rslt);
            }

            if (rslt == BMI08_OK)
            {
                printf("BMI08x data synchronization feature configured !\n\n");
            }
            else
            {
                printf("BMI08x data synchronization feature configuration failure!\n\n");
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
static int8_t enable_bmi08_data_synchronization_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    /* Configure host data ready interrupt */
    #if defined(MCU_APP20)
    int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;
    #elif defined(MCU_APP30)
    int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_2;
    #endif
    int_config.accel_int_config_1.int_type = BMI08_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Configure Accel syncronization input interrupt pin */
    #if defined(MCU_APP20)
    int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_2;
    #elif defined(MCU_APP30)
    int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_1;
    #endif
    int_config.accel_int_config_2.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set gyro interrupt pin configuration */
    #if defined(MCU_APP20)
    int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_3;
    #elif defined(MCU_APP30)
    int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_4;
    #endif
    int_config.gyro_int_config_1.int_type = BMI08_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

    #if defined(MCU_APP20)
    int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_4;
    #elif defined(MCU_APP30)
    int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_3;
    #endif
    int_config.gyro_int_config_2.int_type = BMI08_GYRO_INT_DATA_RDY;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

    /* Enable synchronization interrupt pin */
    rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);

    return rslt;
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return int8_t
 *
 */
static int8_t disable_bmi08_data_synchronization_interrupt()
{
    int8_t rslt;

    /*turn off the sync feature*/
    sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_OFF;

    rslt = bmi08xa_configure_data_synchronization(sync_cfg, &bmi08dev);
    bmi08_error_codes_print_result("bmi08xa_configure_data_synchronization", rslt);

    /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
    /* configure synchronization interrupt pins */
    if (rslt == BMI08_OK)
    {
        /* Set accel interrupt pin configuration */
        /* Configure host data ready interrupt */
    #if defined(MCU_APP20)
        int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_2;
    #endif
        int_config.accel_int_config_1.int_type = BMI08_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Configure Accel synchronization input interrupt pin */
    #if defined(MCU_APP20)
        int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_2;
    #elif defined(MCU_APP30)
        int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_1;
    #endif
        int_config.accel_int_config_2.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Set gyro interrupt pin configuration */
    #if defined(MCU_APP20)
        int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_3;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_4;
    #endif
        int_config.gyro_int_config_1.int_type = BMI08_GYRO_INT_DATA_RDY;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    #if defined(MCU_APP20)
        int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_4;
    #elif defined(MCU_APP30)
        int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_3;
    #endif
        int_config.gyro_int_config_2.int_type = BMI08_GYRO_INT_DATA_RDY;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

        /* Disable synchronization interrupt pin */
        rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 */
int main(void)
{
    int8_t rslt;
    float acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;
    float gyr_x = 0.0, gyr_y = 0.0, gyr_z = 0.0;

    /* Interface given as parameter
     *           For I2C : BMI08_I2C_INTF
     *           For SPI : BMI08_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08_interface_init(&bmi08dev, BMI08_I2C_INTF, BMI085_VARIANT);
    bmi08_error_codes_print_result("bmi08_interface_init", rslt);

    /* Initialize the sensors */
    rslt = init_bmi08();
    bmi08_error_codes_print_result("init_bmi08", rslt);

#if defined(MCU_APP20)
    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi08_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#elif defined(MCU_APP30)
    coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6, bmi08_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#endif

    /* Enable data ready interrupts */
    rslt = enable_bmi08_data_synchronization_interrupt();
    bmi08_error_codes_print_result("enable_bmi08_data_synchronization_interrupt", rslt);

    uint32_t start_time = coines_get_millis();

    printf("Accel data range : 16G for BMI085 and 24G for BMI088\n");
    printf("Gyro data range : 250 dps for BMI085 and BMI088\n\n");

    printf(
        "\nAcc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_DPS_X, Gyr_DPS_Y, Gyr_DPS_Z, Timestamp\n");

    /* Run data synchronization for 100ms before disabling interrupts */
    while (coines_get_millis() - start_time < 100)
    {
        if (data_sync_int == true)
        {
            data_sync_int = false;

            rslt = bmi08a_get_synchronized_data(&bmi08_accel, &bmi08_gyro, &bmi08dev);
            bmi08_error_codes_print_result("bmi08a_get_synchronized_data", rslt);

            if (bmi08dev.variant == BMI085_VARIANT)
            {
                /* Converting lsb to meter per second squared for 16 bit accelerometer at 16G range. */
                acc_x = lsb_to_mps2(bmi08_accel.x, 16, 16);
                acc_y = lsb_to_mps2(bmi08_accel.y, 16, 16);
                acc_z = lsb_to_mps2(bmi08_accel.z, 16, 16);
            }
            else if (bmi08dev.variant == BMI088_VARIANT)
            {
                /* Converting lsb to meter per second squared for 16 bit accelerometer at 24G range. */
                acc_x = lsb_to_mps2(bmi08_accel.x, 24, 16);
                acc_y = lsb_to_mps2(bmi08_accel.y, 24, 16);
                acc_z = lsb_to_mps2(bmi08_accel.z, 24, 16);
            }

            /* Converting lsb to degree per second for 16 bit gyro at 250 dps range. */
            gyr_x = lsb_to_dps(bmi08_gyro.x, (float)250, 16);
            gyr_y = lsb_to_dps(bmi08_gyro.y, (float)250, 16);
            gyr_z = lsb_to_dps(bmi08_gyro.z, (float)250, 16);

            printf("%d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %d, %4.2f, %4.2f, %4.2f, %lu\n",
                   bmi08_accel.x,
                   bmi08_accel.y,
                   bmi08_accel.z,
                   acc_x,
                   acc_y,
                   acc_z,
                   bmi08_gyro.x,
                   bmi08_gyro.y,
                   bmi08_gyro.z,
                   gyr_x,
                   gyr_y,
                   gyr_z,
                   coines_get_millis() - start_time);
        }
    }

    /* Disable data ready interrupts */
    rslt = disable_bmi08_data_synchronization_interrupt();
    bmi08_error_codes_print_result("disable_bmi08_data_synchronization_interrupt", rslt);

    bmi08_coines_deinit();

    return rslt;
}

/* BMI08x data sync. interrupt callback */
void bmi08_data_sync_int()
{
    data_sync_int = true;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
