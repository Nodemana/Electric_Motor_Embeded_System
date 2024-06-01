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
#include "math.h"

// UART inlcudes
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

// Include I2C
#include "driverlib/i2c.h"

// Include Sensor
#include "drivers/BMI160_driver.h"

// Include Event
#include <event_groups.h>

// Inlcude que.h
#include "que.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */
// Define moving average window size
#define ACCEL_WINDOW_SIZE 6   // Window size of moving average

/* ------------------------------------------------------------------------------------------------
 *                                      Extern Global Variables
 * -------------------------------------------------------------------------------------------------
 */

// Include system clock
extern uint32_t g_ui32SysClock;

// The binary semaphore used by the timer ISR & task
extern SemaphoreHandle_t xAccelTimerSemaphore;

// ESTOP
extern SemaphoreHandle_t xESTOPSemaphore;

// Mutexes
extern SemaphoreHandle_t xSharedAccelThresholdFromGUI;

// Local threshold
uint32_t Threshold = 2;
/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */
// // Declare array to store sampled data for moving average
float sampledAccelData[ACCEL_WINDOW_SIZE];
uint8_t currentAccelIndex = 0;

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
 * -------------------------------------------------------------------------------------------------
 */

/*
 * The task to read the OPT3001 sensor.
 */
static void prvReadAccelSensor(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vACCELTask(void);

/* Timer Functions */
void vAccelTimerCallback(TimerHandle_t xTimer); // Handles the timer interupt
void vAccelSoftwareTimer(void);                 // Software timer

/* Moving average for acceleration */
float movingAccelAverage(float newValue);

/* ------------------------------------------------------------------------------------------------
 *                                      Functions
 * -------------------------------------------------------------------------------------------------
 */

void vACCELTask(void)
{
    /* Configure the BMI160 Sensor */
    s8 return_val = configureBMI160();
    UARTprintf("return_val = %d\n", return_val);
    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("BMI160 Example\n");

    /* Create the task as described in the comments at the top of this file.
     *
     */
    xTaskCreate(prvReadAccelSensor,       // The function that implements the task.
                "ACCEL",                  // The text name for the Task
                configMINIMAL_STACK_SIZE, // The size of the stack to allocate to the task.
                NULL,                     // The parameter(s) passed to the task
                tskIDLE_PRIORITY + 1,     // The priority assigned to the task.
                NULL);                    // Task handler

    /* Set up the software timer */
    vAccelSoftwareTimer();
}

/*-----------------------------------------------------------*/
void vAccelSoftwareTimer(void)
{
    // Create a timer
    TimerHandle_t xTimer = xTimerCreate(
        "AccelTimer",                // Name of the timer
        pdMS_TO_TICKS(10),    // Timer period in ticks (10ms second here)
        pdTRUE,                 // Auto-reload
        (void *)0,              // Timer ID
        vAccelTimerCallback          // Callback function
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
void vAccelTimerCallback(TimerHandle_t xTimer)
{
    // Toggle an LED or perform any other task
    // UARTprintf("Timer Callback Executed\n");

    BaseType_t xAccelTaskWoken;

    /* Initialize the xLUXTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xAccelTaskWoken = pdFALSE;

    /* Give the semaphore to unblock prvReadLightSensor.  */
    xSemaphoreGiveFromISR(xAccelTimerSemaphore, &xAccelTaskWoken);
    /* This FreeRTOS API call will handle the context switch if it is
     * required or have no effect if that is not needed. */
    portYIELD_FROM_ISR(xAccelTaskWoken);
}

// Function to calculate moving average
static float sum = 0;
float movingAccelAverage(float newValue)
{
    // Update sum with new value
    sum += newValue - sampledAccelData[currentAccelIndex];

    // Update sampled data array
    sampledAccelData[currentAccelIndex] = newValue;

    // Increment index, wrap around if necessary
    currentAccelIndex = (currentAccelIndex + 1) % ACCEL_WINDOW_SIZE;

    // Return average
    return sum / ACCEL_WINDOW_SIZE;
}


static void prvReadAccelSensor(void *pvParameters)
{
    // Init message que
    CalcMsg AccelMsg; //, *pxPointerToLuxMessage;

    // Init data
    float filteredAccel = 0;
    float accel_x_g = 0;
    float accel_y_g = 0;
    float accel_z_g = 0;
    float accel_avg = 0;
    // Loop Forever
    while (1)
    {
        // // Wait for semaphore to be given by the timer ISR
        if (xSemaphoreTake(xAccelTimerSemaphore, portMAX_DELAY) == pdPASS)
        {
            // SysCtlDelay(g_ui32SysClock);
            s8 result = bmi160_read_accel_xyz(&AccelerationXYZ);
            /* Convert the raw data into acceleration */
            if (result == 0)
            {
                // Get each acceleration component
                accel_x_g = convert_raw_to_g(fabs(AccelerationXYZ.x));
                accel_y_g = convert_raw_to_g(fabs(AccelerationXYZ.y));
                accel_z_g = convert_raw_to_g(fabs(AccelerationXYZ.z));
                // Get average acceleration component
                accel_avg = (accel_x_g + accel_y_g + accel_z_g) / 3;
                float accel_mag = sqrtf((accel_x_g*accel_x_g) + (accel_y_g*accel_y_g) + (accel_z_g*accel_z_g));
                // Filter result
                filteredAccel = movingAccelAverage(accel_avg);
                float filteredMag = movingAccelAverage(filteredMag);

                if(xSemaphoreTake(xSharedAccelerationThresholdFromGUI, 0) == pdPASS) {
                    Threshold = Shared_Power_Threshold;
                    // GIVE ESTOP
                    xSemaphoreGive(xSharedAccelerationThresholdFromGUI);
                }

                if(filteredMag > Threshold){
                    xSemaphoreGive(xESTOPSemaphore);
                }

                // Put data in que structure
                AccelMsg.CalculatedData = filteredAccel;
                AccelMsg.TimeStamp = xTaskGetTickCount();
                
                // Send data to the que
                xQueueSend(/* The handle of the queue. */
                           xAccelSensorQueue,
                           /* The address of the LuxMessage variable.
                            * sizeof( struct AMessage ) bytes are copied from here into
                            * the queue. */
                           (void *)&AccelMsg,
                           /* Block time of 0 says don't block if the queue is already
                            * full.  Check the value returned by xQueueSend() to know
                            * if the message was sent to the queue successfully. */
                           (TickType_t)0);
                           vTaskDelay(pdMS_TO_TICKS(500));
                xEventGroupSetBits(xSensorEventGroup, ACCEL_DATA_READY);

                // UARTprintf("\n");
                char accel_x_msg[23] = "Acceleartion X = : %f\n";
                char accel_y_msg[23] = "Acceleartion Y = : %f\n";
                char accel_z_msg[23] = "Acceleartion Z = : %f\n";
                char accel_avg_msg[25] = "Acceleartion avg = : %f\n";
                char accel_mag_msg[25] = "Acceleartion mag = : %f\n";
                // UartPrintFloat(accel_x_msg, sizeof(accel_x_msg), accel_x_g);
                // UartPrintFloat(accel_y_msg, sizeof(accel_y_msg), accel_y_g);
                // UartPrintFloat(accel_z_msg, sizeof(accel_z_msg), accel_z_g);
                // UartPrintFloat(accel_avg_msg, sizeof(accel_avg_msg), AccelMsg.ClaclulatedData);
                // UartPrintFloat(accel_mag_msg, sizeof(accel_mag_msg), filteredMag);
                // UARTprintf("\n");
            }
            else
            {
                UARTprintf("BMI read unseccusfull\n\n");
            }
        }
    }
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