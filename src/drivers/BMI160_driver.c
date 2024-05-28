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
#include "float_utils.h"
#include <math.h>

// BMI160 header file
#include "drivers/bmi160.h"
#include"drivers/BMI160_driver.h"
#include "drivers/opt3001.h"

//*****************************************************************************
//
// Global variable for storage of actual system clock frequency.
//
//*****************************************************************************

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
s8 bmi160_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

    // UARTprintf("\nBus_write\n");
    // UARTprintf("dev_addr = %d, reg_addr = %d, cnt = %d\n", dev_addr, reg_addr, cnt);

    // Set the slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

    // Send the register address to write to
    I2CMasterDataPut(I2C0_BASE, reg_addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE));

    // Write the data
    for (uint8_t i = 0; i < cnt; i++) {
        I2CMasterDataPut(I2C0_BASE, reg_data[i]);
        if (i == (cnt - 1)) {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        } else {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        }
        while (I2CMasterBusy(I2C0_BASE));
    }

    return 0; // Success
}

//*****************************************************************************
//
// Configure the BMI160 I2C read function
//
//*****************************************************************************
s8 bmi160_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    // UARTprintf("\nBus_read\n");
    // UARTprintf("dev_addr = %d, reg_addr = %d, cnt = %d\n", dev_addr, reg_addr, cnt);

    // Load device slave address for writting 
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, reg_addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE)) { }

    // Load device slave address for reading
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    // UARTprintf("Receiving data: [");
    for (uint8_t i = 0; i < cnt; i++) {
        if (cnt == 1)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        } else if (i == 0)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        } else if (i == (cnt - 1)) 
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        } else 
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }
        while (I2CMasterBusy(I2C0_BASE));
        reg_data[i] = I2CMasterDataGet(I2C0_BASE);
        // UARTprintf("%d, ", reg_data[i]);
    }
    // UARTprintf("]\n");

    return 0; // Success
}

//*****************************************************************************
//
// Configure the BMI160 I2C burst read function
//
//*****************************************************************************
s8 bmi160_i2c_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt)
{
    // Set the slave address and register to read from
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);
    I2CMasterDataPut(I2C0_BASE, reg_addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));

    // Set the slave address for reading
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);
    for (uint32_t i = 0; i < cnt; i++) {

        if (i == 0)
        {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        } else if (i == (cnt - 1)) {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        } else {
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }
        while (I2CMasterBusy(I2C0_BASE));
        reg_data[i] = I2CMasterDataGet(I2C0_BASE);
        UARTprintf("Burst Receiving: %d", reg_data[i]);
    }

    return 0; // Success
}

//*****************************************************************************
//
// Configure the BMI160 delay function
//
//*****************************************************************************
void bmi160_delay_ms(u32 msek)
{
    SysCtlDelay((g_ui32SysClock / (3 * 1000)) * msek);
}


/*!
 *	@brief Used for I2C initialization
 *	@note
 *	The following function is used to map the
 *	I2C bus read, write, bmi160_delay_ms and
 *	device address with global structure bmi160_t
*/
s8 config_BMI160_struct(void)
{
    /*--------------------------------------------------------------------------*
    *  By using bmi160 the following structure parameter can be accessed
    *	Bus write function pointer: BMI160_WR_FUNC_PTR
    *	Bus read function pointer: BMI160_RD_FUNC_PTR
    *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
    *	I2C address: dev_addr
    *--------------------------------------------------------------------------*/
    s_bmi160.bus_write = bmi160_i2c_bus_write;
	s_bmi160.bus_read = bmi160_i2c_bus_read;

	//dl added	bmi160_i2c_burst_read
	s_bmi160.burst_read = bmi160_i2c_burst_read;
	s_bmi160.delay_msec = bmi160_delay_ms;
	s_bmi160.dev_addr = BMI160_I2C_ADDR2;

	return C_BMI160_ZERO_U8X;
}

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
BMI160_RETURN_FUNCTION_TYPE configureBMI160( void )
{
    u32 delay = 30;
    BMI160_RETURN_FUNCTION_TYPE com_rslt = C_BMI160_ZERO_U8X;

    /* Configure the I2C routine functinos */
    com_rslt = config_BMI160_struct();

    /*
    *   This function used to assign the value/reference of
    *	the following parameters
    *	I2C address
    *	Bus Write
    *	Bus read
    *	company_id
    */
	com_rslt += bmi160_init(&s_bmi160);

    UARTprintf("Comm result = %d\n", com_rslt);

    // Configure the sensor to read acceleration
    /*
     * The following functinos are recomended by the header file (and I added bmi160_set_command_register)
     * I added the delays as they use these in the bmi_support.c file
     */
    s_bmi160.delay_msec(delay);
    com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL); 
    s_bmi160.delay_msec(delay);
    com_rslt += bmi160_set_accel_output_data_rate(BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ);
    s_bmi160.delay_msec(delay);
    com_rslt += bmi160_set_accel_bw(0x02);
    s_bmi160.delay_msec(delay);
    com_rslt += bmi160_set_accel_under_sampling_parameter(BMI160_DISABLE);
    s_bmi160.delay_msec(delay);
    com_rslt += bmi160_set_accel_range(BMI160_ACCEL_RANGE_4G);

    /* 
     * Return the result 
     * 0  = success
     * -1 = fail
     */
    return com_rslt;
}

// Function to convert raw data to g
float convert_raw_to_g(int16_t raw_value) {
    return ((float)raw_value / 32768) * G_RANGE;
}