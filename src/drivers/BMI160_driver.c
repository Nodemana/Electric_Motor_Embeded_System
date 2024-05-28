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
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);
}

//*****************************************************************************
//
// Configure the I2C and its pins. 
//
//*****************************************************************************
void prvConfigureI2C_INT( void )
{
    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

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

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
// int main(void)
// {
//     /* Configure the system frequency */
//     g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                              SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
//                                              SYSCTL_CFG_VCO_480), 120000000);
//     /* Initialize the UART */
//     ConfigureUART();
//     /* Initialize the UART */
//     prvConfigureI2C_INT();

//     /* Configure the BMI160 Sensor */
//     s8 return_val = configureBMI160();
//     UARTprintf("return_val = %d\n", return_val);
//     //
//     // Clear the terminal and print the welcome message.
//     //
//     UARTprintf("BMI160 Example\n");

//     // Loop Forever
//     while(1)
//     {
//         SysCtlDelay(g_ui32SysClock);
//         s8 result = bmi160_read_accel_xyz(&AccelerationXYZ);
//         /* Convert the raw data into acceleration */
//         float accel_x_g = convert_raw_to_g(fabs(AccelerationXYZ.x));
//         float accel_y_g = convert_raw_to_g(fabs(AccelerationXYZ.y));
//         float accel_z_g = convert_raw_to_g(fabs(AccelerationXYZ.z));
//         float accel_avg = (accel_x_g + accel_y_g + accel_z_g) / 3;

//         UARTprintf("\n");
//         char accel_x_msg[23] = "Acceleartion X = : %f\n";
//         char accel_y_msg[23] = "Acceleartion Y = : %f\n";
//         char accel_z_msg[23] = "Acceleartion Z = : %f\n";
//         char accel_avg_msg[25] = "Acceleartion avg = : %f\n";
//         UartPrintFloat(accel_x_msg, sizeof(accel_x_msg), accel_x_g);
//         UartPrintFloat(accel_y_msg, sizeof(accel_y_msg), accel_y_g);
//         UartPrintFloat(accel_z_msg, sizeof(accel_z_msg), accel_z_g);
//         UartPrintFloat(accel_avg_msg, sizeof(accel_avg_msg), accel_avg);
//         UARTprintf("\n");
//         /* Print the data */
//         // if (result == 0){
//         //     UARTprintf("Successfully read acceleration: x = %d (%.3f g), y = %d (%.3f g), z = %d (%.3f g)\n",
//         //                AccelerationXYZ.x, accel_x_g,
//         //                AccelerationXYZ.y, accel_y_g,
//         //                AccelerationXYZ.z, accel_z_g);
//         // }
//         // else
//         // {
//         //     UARTprintf("Failed to read acceleration\n", AccelerationXYZ.x);
//         // }
        
//     }
// }


/**** Lux Data ****/
// float     convertedLux = 0;
// uint16_t  rawData = 0;
// bool low_limit_flag = false;
// bool high_limit_flag = false;
//   // Use the following main loop to test opt sensor
// int main( void )
// {
//     bool      worked, success;

//     /* Configure the system frequency */
//     g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                              SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
//                                              SYSCTL_CFG_VCO_480), 120000000);
//     /* Initialize the UART */
//     ConfigureUART();
//     UARTprintf("syst_clock = %d", g_ui32SysClock);
//     /* Initialize the UART */
//     prvConfigureI2C_INT();

//     // Test that sensor is set up correctly
//     UARTprintf("\nTesting OPT3001 Sensor:\n");
//     worked = sensorOpt3001Test();

//     while (!worked) {
//         SysCtlDelay(g_ui32SysClock);
//         UARTprintf("\nTest Failed, Trying again\n");
//         worked = sensorOpt3001Test();
//     }

//     UARTprintf("All Tests Passed!\n\n");

//     // Initialize opt3001 sensor
//     sensorOpt3001Init();
//     sensorOpt3001Enable(true);

//     // Loop Forever
//     while(1)
//     {
//         SysCtlDelay(g_ui32SysClock/100);

//         //Read and convert OPT values
//         success = sensorOpt3001Read(&rawData);

//         if (success) {
//             sensorOpt3001Convert(rawData, &convertedLux);

//             // Construct Text
//             // sprintf(tempStr, "Lux: %5.2f\n", convertedLux);
//             // usnprintf
//             int lux_int = (int)convertedLux;
//             if (low_limit_flag)
//             {
//                 UARTprintf("Low Light Event: %5d Lux\n", lux_int);
//                 low_limit_flag = false;
//             }
//             else if (high_limit_flag)
//             {
//                 UARTprintf("High Light Event: %5d Lux\n", lux_int);
//                 high_limit_flag = false;
//             }
//             else
//             {
//                 UARTprintf("Lux: %5d\n", lux_int);
//             }
//         }
//     }
// }

