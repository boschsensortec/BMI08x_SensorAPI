/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*********************************************************************/
/*                     System header files                           */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/*********************************************************************/
/*                       Own header files                            */
/*********************************************************************/
#include "bmi088_mm.h"
#include "common.h"

/*********************************************************************/
/*                         Global variables                          */
/*********************************************************************/
unsigned char data_sync_int = false;
unsigned int count = 0;

/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/

/*!
 * @brief    This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*!
 * @brief    bmi08 data sync. interrupt callback
 */
void bmi08_data_sync_int();

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
    struct bmi08_data_sync_cfg sync_cfg;

    /* Initialize bmi08 sensors (accel & gyro)*/
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

    if ((bmi08dev->accel_cfg.power != BMI08_ACCEL_PM_ACTIVE) || (bmi08dev->gyro_cfg.power != BMI08_GYRO_PM_NORMAL))
    {
        printf("Accel/gyro sensor in suspend mode\nUse in active/normal mode !!");
        exit(EXIT_FAILURE);
    }

    printf("Uploading BMI08 data synchronization feature config !\n");

    /*API uploads the bmi08 config file onto the device*/
    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_load_config_file(bmi08dev);

        /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
        if (rslt == BMI08_OK)
        {
            bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_24G;

            /* Assign gyro range setting*/
            bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;

            /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
            sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;
            rslt = bmi088_mma_configure_data_synchronization(sync_cfg, bmi08dev);
        }
    }

    if (rslt == BMI08_OK)
    {
        printf("BMI08 data synchronization feature configured !\n");
    }
    else
    {
        printf("BMI08 data synchronization feature configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void enable_bmi08_data_synchronization_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_int_cfg int_config;

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

    rslt = bmi08a_set_data_sync_int_config(&int_config, bmi08dev);

    if (rslt != BMI08_OK)
    {
        printf("BMI08 data synchronization enable interrupt configuration failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void disable_bmi08_data_synchronization_interrupt(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_int_cfg int_config;
    struct bmi08_data_sync_cfg sync_cfg;

    sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_OFF; /*turn off the sync feature*/

    rslt = bmi088_mma_configure_data_synchronization(sync_cfg, bmi08dev);

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

        /* Set gyro interrupt pin configuration*/
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

        rslt = bmi08a_set_data_sync_int_config(&int_config, bmi08dev);
    }

    if (rslt != BMI08_OK)
    {
        printf("BMI08 data synchronization disable interrupt configuration failure!\n");
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
    uint32_t start_time;

    struct bmi08_sensor_data bmi08_accel, bmi08_gyro;

    /* Interface reference is given as a parameter
     *         For I2C : BMI08_I2C_INTF
     *         For SPI : BMI08_SPI_INTF
     */
    rslt = bmi08_interface_init(&bmi08, BMI08_I2C_INTF);

    /* Initialize the sensors */
    init_bmi08(&bmi08);

#if defined(MCU_APP20)
    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi08_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#elif defined(MCU_APP30)
    coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6, bmi08_data_sync_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
#endif

    /* Enable data ready interrupts*/
    enable_bmi08_data_synchronization_interrupt(&bmi08);

    start_time = coines_get_millis();

    /* Run data synchronization for 1s before disabling interrupts */
    while (coines_get_millis() - start_time < 1000)
    {
        if (data_sync_int == true)
        {
            data_sync_int = false;

            rslt = bmi08a_get_synchronized_data(&bmi08_accel, &bmi08_gyro, &bmi08);
            bmi08_check_rslt("bmi08a_get_synchronized_data", rslt);
            count++;

            /*
             * Wait time to collect the accel samples for the datasync feature
             */
            if (count >= 2)
            {
                printf("ax:%d \t ay:%d \t az:%d \t gx:%d \t gy:%d \t gz:%d \t t(ms):%lu\n",
                       bmi08_accel.x,
                       bmi08_accel.y,
                       bmi08_accel.z,
                       bmi08_gyro.x,
                       bmi08_gyro.y,
                       bmi08_gyro.z,
                       coines_get_millis());
            }
        }
    }

    /* Reset count value */
    count = 0;

    /* Disable data ready interrupts */
    disable_bmi08_data_synchronization_interrupt(&bmi08);

    bmi08_coines_deinit();

    return rslt;
}

/* bmi08 data sync interrupt callback */
void bmi08_data_sync_int()
{
    data_sync_int = true;
}
