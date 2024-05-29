#ifndef _BMI160DRIVER_H_
#define _BMI160DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/float_utils.h"
#include <math.h>

// BMI160 header file
#include "drivers/bmi160.h"
#include "drivers/opt3001.h"

//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

/* BMI defines */
#define G_RANGE 4
//*****************************************************************************
//
// Global variable for storage of actual system clock frequency.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

/**** BMI160 Structs ****/
struct bmi160_accel_t AccelerationXYZ;  // Define the struct for reading the acceleration values
struct bmi160_t s_bmi160;               // Define the struct which holds the BMI160 config paramaters

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//*****************************************************************************
//
// Configure the BMI160 Write Function
//
//*****************************************************************************
s8 bmi160_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

//*****************************************************************************
//
// Configure the BMI160 I2C read function
//
//*****************************************************************************
s8 bmi160_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

//*****************************************************************************
//
// Configure the BMI160 I2C burst read function
//
//*****************************************************************************
s8 bmi160_i2c_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt);
//*****************************************************************************
//
// Configure the BMI160 delay function
//
//*****************************************************************************
void bmi160_delay_ms(u32 msek);


/*!
 *	@brief Used for I2C initialization
 *	@note
 *	The following function is used to map the
 *	I2C bus read, write, bmi160_delay_ms and
 *	device address with global structure bmi160_t
*/
s8 config_BMI160_struct(void);

/*!
 *	@brief Used to configure the BMI160 Sensor
 *
 *	@note The following function is used to configure the BMI160 Sensor
 *  @note It sets the config_BMI160_struct
 *  @note It asigns the function pointers to the bmi160_init function (to be used by software driver)
 *  @note It configures the acceleration registers used:
 *  - bmi160_set_accel_output_data_rate()
 *	- bmi160_set_accel_bw()
 *	- bmi160_set_accel_under_sampling_parameter()
 *	- bmi160_set_accel_range()
 *  @note It also sets the bmi160_set_command_register (not sure if this is required)
*/
BMI160_RETURN_FUNCTION_TYPE configureBMI160( void );

// Function to convert raw data to g
float convert_raw_to_g(int16_t raw_value);



#endif /* _BMI160DRIVER_H_ */