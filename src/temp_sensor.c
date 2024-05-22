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

// Include Sensor
#include "drivers/opt3001.h"

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
        SysCtlDelay(g_ui32SysClock / 2);
        // count++;
        // Read and convert light value from OPT3001 sensor

        //Read and convert OPT values
        // success = sensorMLX90615Read(&rawData);
        success = sensorOpt3001Read(&rawData);

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
    UARTprintf("Test!\n\n");
}