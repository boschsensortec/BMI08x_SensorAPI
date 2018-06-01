/**\mainpage
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file        bmi08a.c
 * @date        27 May 2018
 * @version     1.1.0
 *
 */
/*! \file bmi08a.c
 \brief Sensor Driver for BMI08x family of sensors */
/****************************************************************************/
/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"
/****************************************************************************/
/** \name       Macros
 ****************************************************************************/
#if BMI08X_FEATURE_BMI085 == 1
/**\name    Value of LSB_PER_G = (power(2, BMI08X_16_BIT_RESOLUTION) / (2 * range)) */
#define LSB_PER_G       UINT32_C(2048)  /* for the 16-bit resolution and 16g range */
#elif BMI08X_FEATURE_BMI088 == 1
/**\name    Value of LSB_PER_G = (power(2, BMI08X_16_BIT_RESOLUTION) / (2 * range)) */
#define LSB_PER_G       UINT32_C(1365)  /* for the 16-bit resolution and 24g range */
#endif
/****************************************************************************/
/**\name        Local structures
 ****************************************************************************/
/*!
 * @brief Accel self test diff xyz data structure
 */
struct selftest_delta_limit {
	/*! Accel X  data */
	uint16_t x;
	/*! Accel Y  data */
	uint16_t y;
	/*! Accel Z  data */
	uint16_t z;
};

/****************************************************************************/
/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi08x_int_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_int_pin_config(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_data_ready_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_input(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of bmi08x_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_anymotion_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev);

/*!
 * @brief This API writes the config stream data in memory using burst mode
 *
 * @param[in] stream_data : Pointer to store data of 32 bytes
 * @param[in] index       : Represents value in multiple of 32 bytes
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, const struct bmi08x_dev *dev);

/*!
 * @brief This API performs the pre-requisites needed to perform the self test
 *
 * @param[in] dev : structure instance of bmi08x_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_self_test(struct bmi08x_dev *dev);

/*!
 * @brief This API reads the accel data with the positive excitation
 *
 * @param[out] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev	: structure instance of bmi08x_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t positive_excited_accel(struct bmi08x_sensor_data *accel_pos, const struct bmi08x_dev *dev);

/*!
 * @brief This API reads the accel data with the negative excitation
 *
 * @param[out] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev	: structure instance of bmi08x_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t negative_excited_accel(struct bmi08x_sensor_data *accel_neg, const struct bmi08x_dev *dev);

/*!
 * @brief This API validates the self test results
 *
 * @param[in] accel_pos	: Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] accel_neg	: Structure pointer to store accel data
 *                        for negative excitation
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Self test fail
 */
static int8_t validate_accel_self_test(const struct bmi08x_sensor_data *accel_pos,
		const struct bmi08x_sensor_data *accel_neg);

/*!
 * @brief This API converts lsb value of axes to mg for self-test
 *
 * @param[in] accel_data_diff     : Pointer variable used to pass accel difference
 * values in g
 *
 * @param[out] accel_data_diff_mg : Pointer variable used to store accel
 * difference values in mg
 *
 * @return None
 */
static void convert_lsb_g(const struct selftest_delta_limit *accel_data_diff,
		struct selftest_delta_limit *accel_data_diff_mg);

/****************************************************************************/
/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/
/**\name        Globals
 ****************************************************************************/

/****************************************************************************/
/**\name        Function definitions
 ****************************************************************************/
/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
int8_t bmi08a_init(struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		if (dev->intf == BMI08X_SPI_INTF) {
			/* Set dummy byte in case of SPI interface */
			dev->dummy_byte = BMI08X_ENABLE;
			/* Dummy read of Chip-ID in SPI mode */
			rslt = get_regs(BMI08X_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);
		} else {
			/* Make dummy byte 0 in case of I2C interface */
			dev->dummy_byte = BMI08X_DISABLE;
		}
		if (rslt == BMI08X_OK) {
			rslt = get_regs(BMI08X_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);

			if (rslt == BMI08X_OK) {
				/* Check for chip id validity */
				if (chip_id == BMI08X_ACCEL_CHIP_ID) {
					/* Store the chip ID in dev structure */
					dev->accel_chip_id = chip_id;
				} else {
					rslt = BMI08X_E_DEV_NOT_FOUND;
				}
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to write the binary configuration in the sensor.
 */
int8_t bmi08a_write_config_file(const struct bmi08x_dev *dev)
{
	int8_t rslt;
	/* Config loading disable*/
	uint8_t config_load = BMI08X_DISABLE;
	uint8_t current_acc_pwr_ctrl = 0;
	uint16_t index = 0;
	
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Check if config file pointer is not null */
	if ((rslt == BMI08X_OK) && (dev->config_file_ptr != NULL)) {
		
		/* Check whether the read/write length is valid */
		if (dev->read_write_len > 0) {
			/* deactivate accel, otherwise post processing can not be enabled safely */
			rslt = get_regs(BMI08X_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
			if (rslt != BMI08X_OK) {
				return rslt;
			}
			rslt = set_regs(BMI08X_ACCEL_PWR_CTRL_REG, &config_load, 1, dev);
			if (rslt == BMI08X_OK) {
				/*delay required to switch power modes*/
				dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
			} else {
				return rslt;
			}
			
			/* Disable config loading*/
			rslt = set_regs(BMI08X_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

			if (rslt == BMI08X_OK) {
				for (index = 0; index < BMI08X_CONFIG_STREAM_SIZE;
						index += dev->read_write_len) {
					/* Write the config stream */
					rslt = stream_transfer_write((dev->config_file_ptr + index),
							index, dev);
				}
				if (rslt == BMI08X_OK) {
					/* Enable config loading and FIFO mode */
					config_load = BMI08X_ENABLE;

					rslt = set_regs(BMI08X_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

					/* Wait till ASIC is initialized. Refer the data-sheet
					 * for more information */
					dev->delay_ms(BMI08X_ASIC_INIT_TIME_MS);
					
					/* Check for config initialization status (1 = OK)*/
					uint8_t reg_data = 0;
					rslt = get_regs(BMI08X_ACCEL_INTERNAL_STAT_REG, &reg_data, 1, dev);

					if(rslt == BMI08X_OK && reg_data != 1)
					{
						rslt = BMI08X_E_CONFIG_STREAM_ERROR;
					}
					else
					{
						/* reactivate accel */
						rslt = set_regs(BMI08X_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
						if (rslt == BMI08X_OK) {
							/*delay required to switch power modes*/
							dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
						}
					}
				}
			}
		} else {
			rslt = BMI08X_E_RD_WR_LENGTH_INVALID;
		}
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 */
int8_t bmi08a_write_feature_config(uint8_t reg_addr, uint16_t *reg_data, uint8_t len, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	uint16_t read_length = (reg_addr*2) + (len*2);
	uint8_t feature_data[read_length];

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		/* Read feature space up to the given feature position */		
		rslt = bmi08a_get_regs(BMI08X_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);
		
		if(rslt == BMI08X_OK)
		{	
			/* Apply the given feature config. */
			for(int i = 0; i < len; ++i)
			{
				/* Be careful: the feature config space is 16bit aligned! */
				feature_data[(reg_addr*2) + (i*2)] = reg_data[i]&0xFF;
				feature_data[(reg_addr*2) + (i*2) + 1] = reg_data[i]>>8;
			}
			
			/* Write back updated feature space */
			rslt = bmi08a_set_regs(BMI08X_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);
		}
	}

	return rslt;
}

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 */
int8_t bmi08a_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (reg_data != NULL)) {
		if (len > 0) {
			/* Reading from the register */
			rslt = get_regs(reg_addr, reg_data, len, dev);
		} else {
			rslt = BMI08X_E_RD_WR_LENGTH_INVALID;
		}
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 */
int8_t bmi08a_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (reg_data != NULL)) {
		if (len > 0) {
			/* Writing to the register */
			rslt = set_regs(reg_addr, reg_data, len, dev);
		} else {
			rslt = BMI08X_E_RD_WR_LENGTH_INVALID;
		}
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API reads the error status from the accel sensor.
 */
int8_t bmi08a_get_error_status(struct bmi08x_err_reg *err_reg, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		if (err_reg != NULL) {
			/* Read the error codes */
			rslt = get_regs(BMI08X_ACCEL_ERR_REG, &data, 1, dev);

			if (rslt == BMI08X_OK) {
				/* Fatal error */
				err_reg->fatal_err = BMI08X_GET_BITS_POS_0(data, BMI08X_FATAL_ERR);
				/* User error */
				err_reg->err_code = BMI08X_GET_BITS(data, BMI08X_ERR_CODE);
			}
		} else {
			rslt = BMI08X_E_NULL_PTR;
		}
	}

	return rslt;
}

/*!
 *  @brief This API reads the status of the accel sensor.
 */
int8_t bmi08a_get_status(uint8_t *status, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (status != NULL)) {
		/* Read the status */
		rslt = get_regs(BMI08X_ACCEL_STATUS_REG, &data, 1, dev);

		if (rslt == BMI08X_OK) {
			/* Updating the status */
			*status = BMI08X_GET_BITS(data, BMI08X_ACCEL_STATUS);
		}
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API resets the accel sensor.
 */
int8_t bmi08a_soft_reset(const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		data = BMI08X_SOFT_RESET_CMD;
		/* Reset accel device */
		rslt = set_regs(BMI08X_ACCEL_SOFTRESET_REG, &data, 1, dev);

		if (rslt == BMI08X_OK) {
			/* Delay 1 ms after reset value is written to its register */
			dev->delay_ms(BMI08X_ACCEL_SOFTRESET_DELAY_MS);
			/* After soft reset SPI mode in the initialization phase, need to  perform a dummy SPI read
			 * operation, The soft-reset performs a fundamental reset to the device, which is largely
			 * equivalent to a power cycle. */
			if (dev->intf == BMI08X_SPI_INTF) {
				/* Dummy SPI read operation of Chip-ID */
				rslt = get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the accel config value i.e. odr, band width and range from the sensor,
 * store it in the bmi08x_dev structure instance passed by the user.
 *
 */
int8_t bmi08a_get_meas_conf(struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		rslt = get_regs(BMI08X_ACCEL_CONF_REG, data, 2, dev);

		if (rslt == BMI08X_OK) {
			dev->accel_cfg.odr = data[0] & BMI08X_ACCEL_ODR_MASK;
			dev->accel_cfg.bw = (data[0] & BMI08X_ACCEL_BW_MASK) >> 4;
			dev->accel_cfg.range = data[1] & BMI08X_ACCEL_RANGE_MASK;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of accel sensor.
 */
int8_t bmi08a_set_meas_conf(const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };
	uint8_t bw, range, odr;
	uint8_t is_odr_invalid = FALSE, is_bw_invalid = FALSE, is_range_invalid = FALSE;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		odr = dev->accel_cfg.odr;
		bw = dev->accel_cfg.bw;
		range = dev->accel_cfg.range;

		/* Check for valid ODR */
		if ((odr < BMI08X_ACCEL_ODR_12_5_HZ) || (odr > BMI08X_ACCEL_ODR_1600_HZ)) {
			/* Updating the status */
			is_odr_invalid = TRUE;
		}
		/* Check for valid bandwidth */
		if (bw > BMI08X_ACCEL_BW_NORMAL) {
			/* Updating the status */
			is_bw_invalid = TRUE;
		}
#if BMI08X_FEATURE_BMI085 == 1
		/* Check for valid Range */
		if (range > BMI085_ACCEL_RANGE_16G) {
			/* Updating the status */
			is_range_invalid = TRUE;
		}
#elif BMI08X_FEATURE_BMI088 == 1
		/* Check for valid Range */
		if (range > BMI088_ACCEL_RANGE_24G) {
			/* Updating the status */
			is_range_invalid = TRUE;
		}
#endif
		/* If ODR, BW and Range are valid, write it to accel config. registers */
		if ((!is_odr_invalid) && (!is_bw_invalid) && (!is_range_invalid)) {
			/* Read accel config. register */
			rslt = get_regs(BMI08X_ACCEL_CONF_REG, data, 2, dev);
			if (rslt == BMI08X_OK) {
				/* Update data with new odr and bw values */
				data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_ACCEL_ODR, odr);
				data[0] = BMI08X_SET_BITS(data[0], BMI08X_ACCEL_BW, bw);
				/* Update data with current range values */
				data[1] = BMI08X_SET_BITS_POS_0(data[1], BMI08X_ACCEL_RANGE, range);
				/* write to range register */
				rslt = set_regs(BMI08X_ACCEL_CONF_REG, data, 2, dev);
			}
		} else {
			/* Invalid configuration present in ODR, BW, Range */
			rslt = BMI08X_E_INVALID_CONFIG;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the accel power mode from the sensor, store it in the bmi08x_dev structure
 * instance passed by the user.
 */
int8_t bmi08a_get_power_mode(struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		rslt = get_regs(BMI08X_ACCEL_PWR_CONF_REG, &data, 1, dev);

		if (rslt == BMI08X_OK) {
			/* Updating the current power mode */
			dev->accel_cfg.power = data;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the accel sensor.
 */
int8_t bmi08a_set_power_mode(const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t power_mode;
	uint8_t data[2];

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		power_mode = dev->accel_cfg.power;

		/* Configure data array to write to accel power configuration register */
		if (power_mode == BMI08X_ACCEL_PM_ACTIVE) {
			data[0] = BMI08X_ACCEL_PM_ACTIVE;
			data[1] = BMI08X_ACCEL_POWER_ENABLE;
		} else if (power_mode == BMI08X_ACCEL_PM_SUSPEND) {
			data[0] = BMI08X_ACCEL_PM_SUSPEND;
			data[1] = BMI08X_ACCEL_POWER_DISABLE;
		} else {
			/* Invalid power input */
			rslt = BMI08X_E_INVALID_INPUT;
		}

		if (rslt == BMI08X_OK) {
			/*enable accel sensor*/
			rslt = set_regs(BMI08X_ACCEL_PWR_CONF_REG, &data[0], 1, dev);

			if (rslt == BMI08X_OK) {
				/*delay between power ctrl and power config*/
				dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
				/* write to accel power configuration register */
				rslt = set_regs(BMI08X_ACCEL_PWR_CTRL_REG, &data[1], 1, dev);

				if (rslt == BMI08X_OK) {
					/*delay required to switch power modes*/
					dev->delay_ms(BMI08X_POWER_CONFIG_DELAY);
				}
			}

		}
	}

	return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the bmi08x_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi08a_get_data(struct bmi08x_sensor_data *accel, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (accel != NULL)) {
		/* Read accel sensor data */
		rslt = get_regs(BMI08X_ACCEL_X_LSB_REG, data, 6, dev);

		if (rslt == BMI08X_OK) {
			lsb = data[0];
			msb = data[1];
			msblsb = (msb << 8) | lsb;
			accel->x = ((int16_t) msblsb); /* Data in X axis */

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			accel->y = ((int16_t) msblsb); /* Data in Y axis */

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			accel->z = ((int16_t) msblsb); /* Data in Z axis */
		}

	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the bmi08x_int_cfg
 * structure instance.
 */
int8_t bmi08a_set_int_config(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (int_config != NULL)) {
		switch (int_config->int_type) {
		case BMI08X_ACCEL_DATA_RDY_INT:
			/* Data ready interrupt */
			rslt = set_accel_data_ready_int(int_config, dev);
			break;
		case BMI08X_ACCEL_SYNC_DATA_RDY_INT:
			/* synchronized data ready interrupt */
			rslt = set_accel_sync_data_ready_int(int_config, dev);
			break;
		case BMI08X_ACCEL_SYNC_INPUT:
			/* input for synchronization on accel */
			rslt = set_accel_sync_input(int_config, dev);
			break;
		case BMI08X_ACCEL_ANYMOTION_INT:
			/* Anymotion interrupt */
			rslt = set_accel_anymotion_int(int_config, dev);
			break;
		default:
			rslt = BMI08X_E_INVALID_CONFIG;
			break;
		}
	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API reads the temperature of the sensor in degree Celcius.
 */
int8_t bmi08a_get_sensor_temperature(const struct bmi08x_dev *dev, int32_t *sensor_temp)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };
	uint16_t msb, lsb;
	uint16_t msblsb;
	int16_t temp;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (sensor_temp != NULL)) {
		/* Read sensor temperature */
		rslt = get_regs(BMI08X_TEMP_MSB_REG, data, 2, dev);

		if (rslt == BMI08X_OK) {
			msb = (data[0] << 3); /* MSB data */
			lsb = (data[1] >> 5); /* LSB data */
			msblsb = (uint16_t) (msb + lsb);

			if (msblsb > 1023) {
				/* Updating the msblsb */
				temp = (int16_t) (msblsb - 2048);
			} else {
				temp = (int16_t) msblsb;
			}
			/* sensor temperature */
			*sensor_temp = (temp * 125) + 23000;
		}

	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;

}

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 */
int8_t bmi08a_get_sensor_time(const struct bmi08x_dev *dev, uint32_t *sensor_time)
{
	int8_t rslt;
	uint8_t data[3] = { 0 };
	uint32_t byte2, byte1, byte0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (sensor_time != NULL)) {
		/* Read 3-byte sensor time */
		rslt = get_regs(BMI08X_ACCEL_SENSORTIME_0_REG, data, 3, dev);

		if (rslt == BMI08X_OK) {
			byte0 = data[0]; /* Lower byte */
			byte1 = (data[1] << 8); /* Middle byte */
			byte2 = (data[2] << 16); /* Higher byte */

			/* Sensor time */
			*sensor_time = (byte2 | byte1 | byte0);
		}

	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API applies the passed IIR filter to the passed sensor data.
 */
struct bmi08x_sensor_data bmi08a_apply_iir_filter(struct bmi08x_sensor_data accel, struct bmi08x_iir_filter *iir)
{
	uint8_t indx;

	/* update internal states --> shift by 1*/
	for (indx = iir->filter_coef.filter_order; indx > 0; indx--)
	{
		iir->out[indx] = iir->out[indx - 1];
		iir->in[indx] = iir->in[indx - 1];
	}
	/* copy input value */
	iir->in[0].x = accel.x;
	iir->in[0].y = accel.y;
	iir->in[0].z = accel.z;

	/* calculate first sample */
	iir->out[0].x = iir->filter_coef.iir_b_coef[0]*iir->in[0].x;
	iir->out[0].y = iir->filter_coef.iir_b_coef[0]*iir->in[0].y;
	iir->out[0].z = iir->filter_coef.iir_b_coef[0]*iir->in[0].z;

	/*run iir algorithms for all samples*/
	for (indx = 1; indx <= iir->filter_coef.filter_order; indx++)
	{
		iir->out[0].x += iir->filter_coef.iir_b_coef[indx] * iir->in[indx].x - iir->filter_coef.iir_a_coef[indx] * iir->out[indx].x;
		iir->out[0].y += iir->filter_coef.iir_b_coef[indx] * iir->in[indx].y - iir->filter_coef.iir_a_coef[indx] * iir->out[indx].y;
		iir->out[0].z += iir->filter_coef.iir_b_coef[indx] * iir->in[indx].z - iir->filter_coef.iir_a_coef[indx] * iir->out[indx].z;
	}
	
	/*perform saturation*/
	iir->out[0].x = (iir->out[0].x > 32767.0) ? 32767.0 : ((iir->out[0].x < -32768.0) ? -32768.0 : iir->out[0].x);
	iir->out[0].y = (iir->out[0].y > 32767.0) ? 32767.0 : ((iir->out[0].y < -32768.0) ? -32768.0 : iir->out[0].y);
	iir->out[0].z = (iir->out[0].z > 32767.0) ? 32767.0 : ((iir->out[0].z < -32768.0) ? -32768.0 : iir->out[0].z);

	/*map to 16bit integer output*/
	struct bmi08x_sensor_data out;
	out.x = iir->out[0].x;
	out.y = iir->out[0].y;
	out.z = iir->out[0].z;

	return out;
}

int8_t bmi08a_init_iir_filter(struct bmi08x_iir_filter *iir)
{
	for(int indx = 0; indx <= iir->filter_coef.filter_order; ++indx)
	{
		iir->in[indx].x = 0;
		iir->in[indx].y = 0;
		iir->in[indx].z = 0;
		iir->out[indx].x = 0;
		iir->out[indx].y = 0;
		iir->out[indx].z = 0;
	}
	
	return BMI08X_OK;
}	

/*!
 *  @brief This API checks whether the self test functionality of the sensor
 *  is working or not.
 */
int8_t bmi08a_perform_selftest(struct bmi08x_dev *dev)
{
	int8_t rslt;
	int8_t self_test_rslt = 0;
	struct bmi08x_sensor_data accel_pos, accel_neg;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		/* pre-requisites for self test */
		rslt = enable_self_test(dev);

		if (rslt == BMI08X_OK) {
			rslt = positive_excited_accel(&accel_pos, dev);

			if (rslt == BMI08X_OK) {
				rslt = negative_excited_accel(&accel_neg, dev);

				if (rslt == BMI08X_OK) {
					/* Validate the self test result */
					rslt = validate_accel_self_test(&accel_pos, &accel_neg);
					/* Store the status of self test result */
					self_test_rslt = rslt;
					/* Perform soft reset */
					rslt = bmi08a_soft_reset(dev);
					/* Check to ensure bus operations are success */
					if (rslt == BMI08X_OK) {
						/* Restore self_test_rslt as return value */
						rslt = self_test_rslt;
					}
				}

			}

		}

	}

	return rslt;
}

/*****************************************************************************/
/* Static function definition */
/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = BMI08X_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BMI08X_OK;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint16_t index;
	uint16_t temp_len = len + dev->dummy_byte;
	uint8_t temp_buff[temp_len];

	if (dev->intf == BMI08X_SPI_INTF) {
		/* Configuring reg_addr for SPI Interface */
		reg_addr = reg_addr | BMI08X_SPI_RD_MASK;
	}
	/* Read the data from the register */
	rslt = dev->read(dev->accel_id, reg_addr, temp_buff, temp_len);

	if (rslt == BMI08X_OK) {
		for (index = 0; index < len; index++) {
			/* Updating the data buffer */
			reg_data[index] = temp_buff[index + dev->dummy_byte];
		}
	} else {
		/* Failure case */
		rslt = BMI08X_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API writes the data to the given register address.
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bmi08x_dev *dev)
{
	int8_t rslt;

	if (dev->intf == BMI08X_SPI_INTF) {
		/* Configuring reg_addr for SPI Interface */
		reg_addr = (reg_addr & BMI08X_SPI_WR_MASK);
	}
	/* write to an accel register */
	rslt = dev->write(dev->accel_id, reg_addr, reg_data, len);

	if (rslt != BMI08X_OK) {
		/* Updating the error status */
		rslt = BMI08X_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = 0, data, is_channel_invalid = FALSE;

	switch (int_config->int_channel) {
	case BMI08X_INT_CHANNEL_1:
		/* update reg_addr based on channel inputs */
		reg_addr = BMI08X_ACCEL_INT1_IO_CONF_REG;
		break;

	case BMI08X_INT_CHANNEL_2:
		/* update reg_addr based on channel inputs */
		reg_addr = BMI08X_ACCEL_INT2_IO_CONF_REG;
		break;

	default:
		is_channel_invalid = TRUE;
		break;
	}

	if (!is_channel_invalid) {
		/* Read interrupt pin configuration register */
		rslt = get_regs(reg_addr, &data, 1, dev);

		if (rslt == BMI08X_OK) {
			/* Update data with user configured bmi08x_int_cfg structure */
			data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_LVL, int_config->int_pin_cfg.lvl);
			data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_OD, int_config->int_pin_cfg.output_mode);

			if(int_config->int_type == BMI08X_ACCEL_SYNC_INPUT)
			{
				data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_EDGE, BMI08X_ENABLE);
				data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_IN, int_config->int_pin_cfg.enable_int_pin);
				data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_IO, BMI08X_DISABLE);
			}
			else
			{
				data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_IO, int_config->int_pin_cfg.enable_int_pin);
				data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT_IN, BMI08X_DISABLE);
			}

			/* Write to interrupt pin configuration register */
			rslt = set_regs(reg_addr, &data, 1, dev);
		}
	} else {
		rslt = BMI08X_E_INVALID_INPUT;
	}

	return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static int8_t set_accel_data_ready_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0, conf;

	/* Read interrupt map register */
	rslt = get_regs(BMI08X_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

	if (rslt == BMI08X_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;

		switch (int_config->int_channel) {
		case BMI08X_INT_CHANNEL_1:
			/* Updating the data */
			data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT1_DRDY, conf);
			break;

		case BMI08X_INT_CHANNEL_2:
			/* Updating the data */
			data = BMI08X_SET_BITS(data, BMI08X_ACCEL_INT2_DRDY, conf);
			break;

		default:
			rslt = BMI08X_E_INVALID_INPUT;
			break;
		}

		if (rslt == BMI08X_OK) {
			/* Configure interrupt pins */
			rslt = set_int_pin_config(int_config, dev);

			if (rslt == BMI08X_OK) {
				/* Write to interrupt map register */
				rslt = set_regs(BMI08X_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 */
static int8_t set_accel_sync_data_ready_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	if (rslt == BMI08X_OK) {

		data = BMI08X_ACCEL_INTA_DISABLE;

		switch (int_config->int_channel) {
		case BMI08X_INT_CHANNEL_1:
			reg_addr = BMI08X_ACCEL_INT1_MAP_REG;
			break;

		case BMI08X_INT_CHANNEL_2:
			reg_addr = BMI08X_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = BMI08X_E_INVALID_INPUT;
			break;
		}

		if (rslt == BMI08X_OK) {
			if (int_config->int_pin_cfg.enable_int_pin == BMI08X_ENABLE) {
				/*interrupt A mapped to INT1/INT2 */
				data = BMI08X_ACCEL_INTA_ENABLE;
			}
			/* Write to interrupt map register */
			rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == BMI08X_OK) {
				/*set input interrupt configuration*/
				rslt = set_int_pin_config(int_config, dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 */
static int8_t set_accel_sync_input(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	if (rslt == BMI08X_OK) {
		/*set input interrupt configuration*/
		rslt = set_int_pin_config(int_config, dev);
	}

	return rslt;
}

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 */
static int8_t set_accel_anymotion_int(const struct bmi08x_accel_int_channel_cfg *int_config, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	if (rslt == BMI08X_OK) {

		data = BMI08X_ACCEL_INTB_DISABLE;

		switch (int_config->int_channel) {
		case BMI08X_INT_CHANNEL_1:
			reg_addr = BMI08X_ACCEL_INT1_MAP_REG;
			break;

		case BMI08X_INT_CHANNEL_2:
			reg_addr = BMI08X_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = BMI08X_E_INVALID_INPUT;
			break;
		}

		if (rslt == BMI08X_OK) {
			if (int_config->int_pin_cfg.enable_int_pin == BMI08X_ENABLE) {
				/*interrupt B mapped to INT1/INT2 */
				data = BMI08X_ACCEL_INTB_ENABLE;
			}
			/* Write to interrupt map register */
			rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == BMI08X_OK) {
				/*set input interrupt configuration*/
				rslt = set_int_pin_config(int_config, dev);
			}
		}
	}

	return rslt;
}


/*!
 *  @brief This API writes the config stream data in memory using burst mode.
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
	uint8_t asic_lsb = ((index / 2) & 0x0F);

	/* Write to feature config register */
	rslt = set_regs(BMI08X_ACCEL_RESERVED_5B_REG, &asic_lsb, 1, dev);

	if (rslt == BMI08X_OK) {
		/* Write to feature config register */
		rslt = set_regs(BMI08X_ACCEL_RESERVED_5C_REG, &asic_msb, 1, dev);

		if (rslt == BMI08X_OK) {
			/* Write to feature config registers */
			rslt = set_regs(BMI08X_ACCEL_FEATURE_CFG_REG, (uint8_t *)stream_data, dev->read_write_len, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API performs the pre-requisites needed to perform the self test
 */
static int8_t enable_self_test(struct bmi08x_dev *dev)
{
	int8_t rslt;

	/* Configuring sensors to perform accel self test */
	dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
	dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;

	/*check the chip id of the accel variant and assign the range */
#if BMI08X_FEATURE_BMI085 == 1
	dev->accel_cfg.range = BMI085_ACCEL_RANGE_16G;
#elif BMI08X_FEATURE_BMI088 == 1
	dev->accel_cfg.range = BMI088_ACCEL_RANGE_24G;
#endif

	dev->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	/* Enable Accel sensor */
	rslt = bmi08a_set_power_mode(dev);

	if (rslt == BMI08X_OK) {
		/* Configure sensors with above configured settings */
		rslt = bmi08a_set_meas_conf(dev);

		if (rslt == BMI08X_OK) {
			/* Self test delay */
			dev->delay_ms(BMI08X_SELF_TEST_DELAY_MS);
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the accel data with the positive excitation
 */
static int8_t positive_excited_accel(struct bmi08x_sensor_data *accel_pos, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data = BMI08X_ACCEL_POSITIVE_SELF_TEST;

	/* Enable positive excitation for all 3 axes */
	rslt = set_regs(BMI08X_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);

	if (rslt == BMI08X_OK) {
		/* Read accel data after 50ms delay */
		dev->delay_ms(BMI08X_SELF_TEST_DATA_READ_MS);
		rslt = bmi08a_get_data(accel_pos, dev);
	}

	return rslt;
}

/*!
 * @brief This API reads the accel data with the negative excitation
 */
static int8_t negative_excited_accel(struct bmi08x_sensor_data *accel_neg, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data = BMI08X_ACCEL_NEGATIVE_SELF_TEST;

	/* Enable negative excitation for all 3 axes */
	rslt = set_regs(BMI08X_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);

	if (rslt == BMI08X_OK) {
		/* Read accel data after 50ms delay */
		dev->delay_ms(BMI08X_SELF_TEST_DATA_READ_MS);
		rslt = bmi08a_get_data(accel_neg, dev);

		if (rslt == BMI08X_OK) {
			/* Disable self test */
			reg_data = BMI08X_ACCEL_SWITCH_OFF_SELF_TEST;
			rslt = set_regs(BMI08X_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API validates the self test results
 */
static int8_t validate_accel_self_test(const struct bmi08x_sensor_data *accel_pos,
		const struct bmi08x_sensor_data *accel_neg)
{
	int8_t rslt;
	/*! Structure for difference of accel values in g */
	struct selftest_delta_limit accel_data_diff = { 0 };
	/*! Structure for difference of accel values in mg */
	struct selftest_delta_limit accel_data_diff_mg = { 0 };

	accel_data_diff.x = (uint16_t) (BMI08X_ABS(accel_pos->x) + BMI08X_ABS(accel_neg->x));
	accel_data_diff.y = (uint16_t) (BMI08X_ABS(accel_pos->y) + BMI08X_ABS(accel_neg->y));
	accel_data_diff.z = (uint16_t) (BMI08X_ABS(accel_pos->z) + BMI08X_ABS(accel_neg->z));

	/*! Converting LSB of the differences of
	 accel values to mg */
	convert_lsb_g(&accel_data_diff, &accel_data_diff_mg);

	/* Validating accel data by comparing with minimum value of the axes in mg */
	/* x axis limit 1000mg, y axis limit 1000mg and z axis limit 500mg */
	if (accel_data_diff_mg.x >= 1000 && accel_data_diff_mg.y >= 1000 && accel_data_diff_mg.z >= 500) {
		/* Updating Okay status */
		rslt = BMI08X_OK;
	} else {
		/* Updating Error status */
		rslt = BMI08X_W_SELF_TEST_FAIL;
	}

	return rslt;
}

/*!
 *  @brief This API converts lsb value of axes to mg for self-test.
 */
static void convert_lsb_g(const struct selftest_delta_limit *accel_data_diff,
		struct selftest_delta_limit *accel_data_diff_mg)
{
	/* accel x value in mg */
	accel_data_diff_mg->x = (uint16_t) ((accel_data_diff->x / (int32_t)LSB_PER_G) * 1000);
	/* accel y value in mg */
	accel_data_diff_mg->y = (uint16_t) ((accel_data_diff->y / (int32_t)LSB_PER_G) * 1000);
	/* accel z value in mg */
	accel_data_diff_mg->z = (uint16_t) ((accel_data_diff->z / (int32_t)LSB_PER_G) * 1000);
}

/** @}*/
