/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "common.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/*! BMI085 shuttle id */
#define BMI085_SHUTTLE_ID      UINT16_C(0x46)

/*! BMI088 shuttle id */
#define BMI088_SHUTTLE_ID      UINT16_C(0x66)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_i2c(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_i2c(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * SPI read function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * SPI write function map to COINES platform
 */
BMI08X_INTF_RET_TYPE bmi08x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_spi(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to COINES platform
 */
void bmi08x_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08X_OK;
    struct coines_board_info board_info;

    if (bmi08x != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);
        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        result = coines_get_board_info(&board_info);

#if defined(PC)
        setbuf(stdout, NULL);
#endif

        if (result == COINES_SUCCESS)
        {
            if ((board_info.shuttle_id == BMI085_SHUTTLE_ID) && (variant == BMI088_VARIANT))
            {
                printf(
                    "! Warning - BMI085 sensor shuttle and BMI088 variant used \n ,"
                    "This application will not support this combination \n");
            }

            if ((board_info.shuttle_id == BMI088_SHUTTLE_ID) && (variant == BMI085_VARIANT))
            {
                printf(
                    "! Warning - BMI088 sensor shuttle and BMI085 variant used \n ,"
                    "This application will not support this combination \n");
            }

            if ((board_info.shuttle_id != BMI085_SHUTTLE_ID) && (board_info.shuttle_id != BMI088_SHUTTLE_ID))
            {
                printf(
                    "! Warning invalid sensor shuttle (neither BMI085 nor BMI088 used) \n ,"
                    "This application will not support this sensor \n");
            }
        }

        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_usec(10000);

        /* Bus configuration : I2C */
        if (intf == BMI08X_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
            gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
            bmi08x->intf = BMI08X_I2C_INTF;
            bmi08x->read = bmi08x_i2c_read;
            bmi08x->write = bmi08x_i2c_write;

            /* SDO pin is made low for selecting I2C address 0x76*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
            coines_delay_msec(10);

            /* PS pin is made high for selecting I2C protocol (gyroscope)*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
        }
        /* Bus configuration : SPI */
        else if (intf == BMI08X_SPI_INTF)
        {
            printf("SPI Interface \n");

            /* To initialize the user SPI function */
            bmi08x->intf = BMI08X_SPI_INTF;
            bmi08x->read = bmi08x_spi_read;
            bmi08x->write = bmi08x_spi_write;

            /* SPI chip select pin for Accel (CSB1_A) */
            acc_dev_add = COINES_SHUTTLE_PIN_8;

            /* SPI chip select pin for Gyro (CSB2_G) */
            gyro_dev_add = COINES_SHUTTLE_PIN_14;

            /* CSB1 pin is made high for selecting SPI protocol (accelerometer)*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_8, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

            /* CS pin is made high for selecting SPI protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_14, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

            /* PS pin is made low for selecting SPI protocol*/
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
            coines_delay_msec(10);
            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_5_MHZ, COINES_SPI_MODE3);
        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08x->variant = variant;

        /* Assign accel device address to accel interface pointer */
        bmi08x->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08x->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08x->delay_us = bmi08x_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;

        coines_delay_usec(10000);

        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        coines_delay_usec(10000);
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08X_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMI08X_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMI08X_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMI08X_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMI08X_E_OUT_OF_RANGE)
        {
            printf("Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt == BMI08X_E_INVALID_INPUT)
        {
            printf("Error [%d] : Invalid input\r\n", rslt);
        }
        else if (rslt == BMI08X_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Config stream error\r\n", rslt);
        }
        else if (rslt == BMI08X_E_RD_WR_LENGTH_INVALID)
        {
            printf("Error [%d] : Invalid Read write length\r\n", rslt);
        }
        else if (rslt == BMI08X_E_INVALID_CONFIG)
        {
            printf("Error [%d] : Invalid config\r\n", rslt);
        }
        else if (rslt == BMI08X_E_FEATURE_NOT_SUPPORTED)
        {
            printf("Error [%d] : Feature not supported\r\n", rslt);
        }
        else if (rslt == BMI08X_W_FIFO_EMPTY)
        {
            printf("Warning [%d] : FIFO empty\r\n", rslt);
        }
        else
        {
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Deinitializes coines platform
 *
 *  @return void.
 */
void bmi08x_coines_deinit(void)
{
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(100);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(100);

    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
