/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"

// UART inlcudes
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

// Include I2C
#include "driverlib/i2c.h"

// Include Sensor

// Include Event
#include <event_groups.h>

///////////////////// Extern Functions ////////////////////
extern bool sensorMLX90615Init(void);
extern void sensorMLX90615Enable(bool enable);
extern bool sensorMLX90615Read(uint16_t *rawData);
extern void sensorMLX90615Convert(uint16_t rawData, float *convertedLux);
extern bool sensorMLX90615Test(void);
/*
 * The task to read the MLX90615 sensor.
 */
static void prvReadTempSensor(void *pvParameters);

/*
 * The task to configure the MLX90615 sensor.
 */
void prvConfigureMLX90615(void);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vTEMPTask(void);


void vTEMPTask(void)
{
    prvConfigureMLX90615();
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate(prvReadTempSensor,
                "TEMP",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 4,
                NULL);
}

static void prvReadTempSensor(void *pvParameters)
{
    uint16_t count = 0;
    // Define variables
    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    //
    // Configure the system frequency.
    //
    uint32_t g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
    for(;;)
    {

        count++;
        SysCtlDelay(g_ui32SysClock/10);
        // count++;
        // Read and convert light value from OPT3001 sensor

        //Read and convert OPT values
        success = sensorMLX90615Read(&rawData);

        if (success) {
            sensorMLX90615Convert(rawData, &convertedLux);

            // Construct Text
            // sprintf(tempStr, "Lux: %5.2f\n", convertedLux);
            int lux_int = (int)convertedLux;
            // UARTprintf("Raw Temp: %d\n", rawData);
            // UARTprintf("Converted Temp: %d\n", lux_int);
        }
        
    }
}

/*-----------------------------------------------------------*/
void prvConfigureMLX90615(void)
{
    // Define worked flag
    // bool worked;

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("MLX90615 Example\n");

    // //
    // // The I2C0 peripheral must be enabled before use.
    // //
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // //
    // // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // // This step is not necessary if your part does not support pin muxing.
    // //
    // GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    // GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    // //
    // // Select the I2C function for these pins.  This function will also
    // // configure the GPIO pins pins for I2C operation, setting them to
    // // open-drain operation with weak pull-ups.  Consult the data sheet
    // // to see which functions are allocated per pin.
    // //
    // GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    // GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    // I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    //
    // Enable interrupts to the processor.
    //
    // IntMasterEnable();

    // IntEnable(INT_I2C0);

    // // Test that sensor is set up correctly
    // UARTprintf("\nTesting OPT3001 Sensor:\n");
    // worked = sensorOpt3001Test();

    // while (!worked)
    // {
    //     SysCtlDelay(g_ui32SysClock);
    //     UARTprintf("\nTest Failed, Trying again\n");
    //     worked = sensorOpt3001Test();
    // }

    UARTprintf("All Tests Passed!\n\n");

    // Initialize opt3001 sensor
    // sensorOpt3001Init();
    // sensorOpt3001Enable(true);
}