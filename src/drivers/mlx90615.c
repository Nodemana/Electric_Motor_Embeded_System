/**************************************************************************************************
 *  Filename:       opt3001.c
 *  Revised:        
 *  Revision:       
 *
 *  Description:    Driver for the Texas Instruments OP3001 Optical Sensor
 *
 *  Copyright (C) 2014 - 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "i2cOptDriver.h"
#include "i2cTmpDriver.h"
// #include "mlx90615.h"
#include "utils/uartstdio.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* Slave address */
#define MLX90615_I2C_ADDRESS            0x5B // Refer to SMBus slave address. it must be the LS 7 bytes (6-0)

/* Register addresses */
#define REG_EMISSIVITY                  0x02
#define RAW_IR_DATA						0x05
#define AMBIENT_TEMP					0x06
#define OBJECT_TEMP						0x07

#define ID_NUMBER_1          		    0x0E
#define ID_NUMBER_2                     0x0F

/* Commands */
#define READ_RAW_OBJECT_TEMP			0x27 // OP Code: 0010 aaaa, where aaaa is 4 LSBits of the memory map address to be read / written. We want T_A = 0x07
#define READ_RAW_AMBIENT_TEMP			0x26 // OP Code: 0010 aaaa, where aaaa is 4 LSBits of the memory map address to be read / written. We want T_O = 0x06
#define SLAVE_ADDRESS_WRITE				0xB6 // First 7 bits are SA, last bit is w/r, where w = 0, r = 1
#define SLAVE_ADDRESS_READ				0xB7 // First 7 bits are SA, last bit is w/r, where w = 0, r = 1	
/* Register values */
#define MANUFACTURER_ID                 0x5449  // TI
#define DEVICE_ID                       0x3001  // Opt 3001

#define CONFIG_RESET                    0xC810                   
#define CONFIG_TEST                     0xCC10

#define CONFIG_ENABLE                   0x10C4 // equivalent to 0xC410 as upper and lower bytes are received in reverse (100 ms, continuous)
#define CONFIG_DISABLE                  0x10C0 //  equivalent to 0xC010 as upper and lower bytes are received in reverse  (100 ms, shutdown)

/* Bit values */
#define DATA_RDY_BIT                    0x0080  // Data ready

/* Register length */
#define REGISTER_LENGTH                 2

/* Sensor data size */
#define DATA_LENGTH                     2

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
bool sensorMLX90615Init(void);
void sensorMLX90615Enable(bool enable);
bool sensorMLX90615Read(uint16_t *rawData);
void sensorMLX90615Convert(uint16_t rawData, float *convertedLux);
bool sensorMLX90615Test(void);

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Public functions
 * -------------------------------------------------------------------------------------------------
 */


/**************************************************************************************************
 * @fn          sensorOpt3001Init
 *
 * @brief       Initialize the temperature sensor driver
 *
 * @return      none
 **************************************************************************************************/
bool sensorMLX90615Init(void)
{
	sensorMLX90615Enable(true);
	return (true);
}


/**************************************************************************************************
 * @fn          sensorOpt3001Enable
 *
 * @brief       Turn the sensor on/off
 *
 * @return      none
 **************************************************************************************************/
void sensorMLX90615Enable(bool enable)
{
	// Stuff you need to do to enable the sensor
	// I don't think we need to do anything
}


/**************************************************************************************************
 * @fn          sensorOpt3001Read
 *
 * @brief       Read the result register
 *
 * @param       Buffer to store data in
 *
 * @return      TRUE if valid data
 **************************************************************************************************/

/*
	Approach to Read should be as follows:
	1. Write the slave address by sending 0xB6
	2. Write the command to slave
	3. Write the slave address read command
	4. Read the two bytes of data
	5. Read the Packet Error Code and do error checking
*/
bool sensorMLX90615Read(uint16_t *rawData)
{
	bool data_ready = 1;
	uint16_t val;

	if (data_ready)
	{
		if (TempReadI2C_2(MLX90615_I2C_ADDRESS, READ_RAW_OBJECT_TEMP, (uint8_t *)&val))
		{
			// Swap bytes
			*rawData = val;
		} 
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	return true;
}

/**************************************************************************************************
 * @fn          sensorOpt3001Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 **************************************************************************************************/
bool sensorMLX90615Test(void)
{
	uint16_t val;
	
	// Check manufacturer ID
	readI2C(MLX90615_I2C_ADDRESS, ID_NUMBER_1, (uint8_t *)&val);
	val = (val << 8) | (val>>8 &0xFF);

	if (val != MANUFACTURER_ID)
	{
		return false;
	}

	UARTprintf("Manufacturer ID Correct: %c%c\n", val & 0x00FF, (val >> 8) & 0x00FF);

	// Check device ID
	readI2C(MLX90615_I2C_ADDRESS, ID_NUMBER_2, (uint8_t *)&val);
	val = (val << 8) | (val>>8 &0xFF);

	if (val != DEVICE_ID)
	{
		return (false);
	}

	UARTprintf("Device ID Correct: %c%c\n", val & 0x00FF, (val >> 8) & 0x00FF);

	return (true);
}

/**************************************************************************************************
 * @fn          sensorOpt3001Convert
 *
 * @brief       Convert raw data to object and ambience temperature
 *
 * @param       rawData - raw data from sensor
 *
 * @param       convertedLux - converted value (lux)
 *
 * @return      none
 **************************************************************************************************/
void sensorMLX90615Convert(uint16_t rawData, float *convertedLux)
{
	// Convert data 
	*convertedLux = (rawData * 0.002) - 273.15;
}
