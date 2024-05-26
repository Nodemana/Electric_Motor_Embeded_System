/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * -------------------------------------------------------------------------------------------------
 */
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
#include "timers.h"

// UART inlcudes
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

// Include I2C
#include "driverlib/i2c.h"

// Include Sensor
#include "drivers/opt3001.h"

// Include Event
#include <event_groups.h>

// Inlcude que.h
#include "que.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */

// Define moving average window size
#define WINDOW_SIZE 6   // Window size of moving average

/* Que */


/* ------------------------------------------------------------------------------------------------
 *                                      Extern Global Variables
 * -------------------------------------------------------------------------------------------------
 */

/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

// Include system clock
extern uint32_t g_ui32SysClock;

// The binary semaphore used by the timer ISR & task
extern SemaphoreHandle_t xTimerSemaphore;


// Include queue
// extern QueueHandle_t xLuxSensorQueue;

/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */

// Flag to set filter data
volatile bool filter_data = true;
volatile bool data_change = false;

// Declare array to store sampled data for moving average
uint16_t sampledData[WINDOW_SIZE];
uint8_t currentIndex = 0;

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
 * -------------------------------------------------------------------------------------------------
 */

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

/*
 * Helper function for calculating the moving average
 */
uint16_t movingAverage(uint16_t newValue);

/* Timer Functions */
void vTimerCallback(TimerHandle_t xTimer); // Handles the timer interupt
void vLuxSoftwareTimer( void ); // Software timer

/*
 * Handles when Timer0A ends.
 */
//void xTimer0AHandler(void);

/* ------------------------------------------------------------------------------------------------
 *                                      Functions
 * -------------------------------------------------------------------------------------------------
 */

void vLUXTask(void)
{

    // Configure the OPT3001 Sensor
    prvConfigureOPT3001();

    /* Create the task as described in the comments at the top of this file.
     *
     */
    xTaskCreate(prvReadLightSensor,         // The function that implements the task.
                "LUX",                      // The text name for the Task 
                configMINIMAL_STACK_SIZE,   // The size of the stack to allocate to the task.
                NULL,                       // The parameter(s) passed to the task
                tskIDLE_PRIORITY + 2,       // The priority assigned to the task.
                NULL);                      // Task handler

    /* Set up the software timer */
    vLuxSoftwareTimer();
}



// Function to calculate moving average
uint16_t movingAverage(uint16_t newValue)
{
    static uint32_t sum = 0;

    // Update sum with new value
    sum += newValue - sampledData[currentIndex];

    // Update sampled data array
    sampledData[currentIndex] = newValue;

    // Increment index, wrap around if necessary
    currentIndex = (currentIndex + 1) % WINDOW_SIZE;

    // Return average
    return sum / WINDOW_SIZE;
}

/*-----------------------------------------------------------*/
void vLuxSoftwareTimer( void )
{
    // Create a timer
    TimerHandle_t xTimer = xTimerCreate(
        "Timer",                // Name of the timer
        pdMS_TO_TICKS(1000),    // Timer period in ticks (1 second here)
        pdTRUE,                 // Auto-reload
        (void *)0,              // Timer ID
        vTimerCallback          // Callback function
    );

    // Check if the timer was created successfully
    if (xTimer == NULL)
    {
        UARTprintf("Timer creation failed\n");
    }
    else
    {
        // Start the timer
        if (xTimerStart(xTimer, 0) != pdPASS)
        {
            UARTprintf("Timer start failed\n");
        }
    }
    UARTprintf("Timer created\n");

}

/* Timer Call Back function */
void vTimerCallback(TimerHandle_t xTimer)
{
    // Toggle an LED or perform any other task
    // UARTprintf("Timer Callback Executed\n");

    BaseType_t xLUXTaskWoken;

    /* Initialize the xLUXTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLUXTaskWoken = pdFALSE;

    /* Give the semaphore to unblock prvReadLightSensor.  */
    xSemaphoreGiveFromISR(xTimerSemaphore, &xLUXTaskWoken);
    /* This FreeRTOS API call will handle the context switch if it is
     * required or have no effect if that is not needed. */
    portYIELD_FROM_ISR(xLUXTaskWoken);
}

/*-----------------------------------------------------------*/

static void prvReadLightSensor(void *pvParameters)
{
    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;
    uint32_t lux_int = 0;
    uint32_t filteredValue = 0;
    SensorMsg LuxMsg; //, *pxPointerToLuxMessage;
    for (;;)
    {
        // // Wait for semaphore to be given by the timer ISR
        if (xSemaphoreTake(xTimerSemaphore, portMAX_DELAY) == pdPASS)
        {
            // Read and convert light value from OPT3001 sensor
            success = sensorOpt3001Read(&rawData);      
            // Check if it was a success
            if (success)
            {
                sensorOpt3001Convert(rawData, &convertedLux);
                // Construct Text
                lux_int = (int)convertedLux;
                // UARTprintf("Lux: %5d\n", lux_int);
                // Perform moving average filtering
                if (filter_data)
                {
                    filteredValue = movingAverage(lux_int);
                    //UARTprintf("Filtered data: %d\n", filteredValue);
                    LuxMsg.SensorReading = filteredValue;
                }
                else
                {
                    // LuxMsg.SensorReading = filteredValue;
                }
                // Print filtered value to console via UART
                // UARTprintf("Filtered Light Value: %5d\n", filteredValue);


                /* Pull the current time stamp. */
                LuxMsg.TimeStamp = xTaskGetTickCount();
                //UARTprintf("Sending data: %d\n", LuxMsg.SensorReading);
                // // /* Send the entire structure by value to the queue. */
                xQueueSend(/* The handle of the queue. */
                           xLuxSensorQueue,
                           /* The address of the LuxMessage variable.
                            * sizeof( struct AMessage ) bytes are copied from here into
                            * the queue. */
                           (void *)&LuxMsg,
                           /* Block time of 0 says don't block if the queue is already
                            * full.  Check the value returned by xQueueSend() to know
                            * if the message was sent to the queue successfully. */
                           (TickType_t)0);
                           vTaskDelay(pdMS_TO_TICKS(500));
                xEventGroupSetBits(xSensorEventGroup, LUX_DATA_READY);
            }
            else
            {
                UARTprintf("not success!\n\n");
            }
        }
    }
}

/*-----------------------------------------------------------*/
void prvConfigureOPT3001(void)
{
    // Define worked flag
    bool worked;

    // Test that sensor is set up correctly
    UARTprintf("\nTesting OPT3001 Sensor:\n");
    worked = sensorOpt3001Test();

    while (!worked)
    {
        SysCtlDelay(g_ui32SysClock);
        UARTprintf("\nTest Failed, Trying again\n");
        worked = sensorOpt3001Test();
    }

    UARTprintf("\n\nAll OPT3001 Tests Passed.\n\n");

    // Initialize opt3001 sensor
    sensorOpt3001Init();
    sensorOpt3001Enable(true);
    UARTprintf("Sensor Enabled");
}



/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}
