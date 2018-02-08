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
 * @file        bmi088.c
 * @date        02 Feb 2018
 * @version     1.0.0
 *
 */

/*! \file bmi088.c
 \brief Sensor Driver for BMI088 family of sensors */
/****************************************************************************/
/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"
#include "bmi088.h"
#if BMI08X_FEATURE_BMI088 == 1
/****************************************************************************/
/** \name       Macros
 ****************************************************************************/

/****************************************************************************/
/**\name        Local structures
 ****************************************************************************/

/****************************************************************************/
/*! Static Function Declarations
 ****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/
/**\name        Function definitions
 ****************************************************************************/
/*!
 *  @brief This API is the entry point for bmi088 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 */
int8_t bmi088_init(struct bmi08x_dev *dev)
{
	int8_t rslt;
	/*initialize bmi088 accel sensor*/
	rslt = bmi08a_init(dev);

	if (rslt == BMI08X_OK) {
		/*initialize bmi088 gyro sensor*/
		rslt = bmi08g_init(dev);
	}

	return rslt;
}
#endif
/** @}*/
