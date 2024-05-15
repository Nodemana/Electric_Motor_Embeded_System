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
#include "drivers/opt3001.h"

/*
 * The task to read the OPT3001 sensor.
 */
static void prvReadLightSensor(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vLUXTask(void);

/*
 * Hardware configuration for the OPT3001 Sensor.
 */
void prvConfigureOPT3001(void);

void vLUXTask(void)
{
    // Configure the OPT3001 Sensor
    prvConfigureOPT3001();

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
    xTaskCreate(prvReadLightSensor,
                "LUX",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    // TimerEnable(TIMER0_BASE, TIMER_A);
}

static void prvReadLightSensor(void *pvParameters)
{
    uint16_t count = 0;
    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    //
    // Configure the system frequency.
    //
    uint32_t g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
    for (;;)
    {
        SysCtlDelay(g_ui32SysClock/100);
        // count++;
        // Read and convert light value from OPT3001 sensor

        //Read and convert OPT values
        success = sensorOpt3001Read(&rawData);

        if (success) {
            sensorOpt3001Convert(rawData, &convertedLux);

            // Construct Text
            // sprintf(tempStr, "Lux: %5.2f\n", convertedLux);
            int lux_int = (int)convertedLux;
            UARTprintf("Lux: %5d\n", lux_int);
        }
    }
}

/*-----------------------------------------------------------*/
void prvConfigureOPT3001(void)
{
    // Define worked flag
    bool worked;

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("OPT3001 Example\n");

    //
    // Enable interrupts to the processor.
    //
    // IntEnable(INT_I2C0);

    // Test that sensor is set up correctly
    UARTprintf("\nTesting OPT3001 Sensor:\n");
    worked = sensorOpt3001Test();

    while (!worked)
    {
        // SysCtlDelay(g_ui32SysClock);
        UARTprintf("\nTest Failed, Trying again\n");
        worked = sensorOpt3001Test();
    }

    UARTprintf("All Tests Passed!\n\n");

    // Initialize opt3001 sensor
    sensorOpt3001Init();
    sensorOpt3001Enable(true);
}