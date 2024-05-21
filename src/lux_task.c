/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
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
 *    documentation and/or other materials provided with the
 *    distribution.
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
 *
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
/*-----------------------------------------------------------*/

// Define moving average window size
#define WINDOW_SIZE 6

/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

// Include system clock
extern uint32_t g_ui32SysClock;

// The binary semaphore used by the timer ISR & task
extern SemaphoreHandle_t xTimerSemaphore;

// Include queue
extern QueueHandle_t xStructQueue;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

// Flag to set filter data
volatile bool filter_data = true;
volatile bool data_change = false;

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
 * Handles when Timer0A ends.
 */
void xTimerHandler(void);
/*
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED(void);

/*
 * Hardware configuration for the OPT3001 Sensor.
 */
void prvConfigureOPT3001(void);

/*
 * Helper function for calculating the moving average
 */
uint16_t movingAverage(uint16_t newValue);

/*-----------------------------------------------------------*/

void vLUXTask(void)
{
    /* Light the initial LED. */
    prvConfigureLED();

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
                tskIDLE_PRIORITY + 1,
                NULL);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

// Declare array to store sampled data for moving average
uint16_t sampledData[WINDOW_SIZE];
uint8_t currentIndex = 0;

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

// Timer handler
void xTimerHandler(void)
{
    UARTprintf("int\n");
    /* Clear the hardware interrupt flag for Timer 0A. */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

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

#define TASK1_ID 0
/*
 * The queue used by both tasks.
 */
struct AMessage
{
    uint32_t ulMessageID;
    uint16_t lightValue;
    uint32_t ulTimeStamp;
} xMessage;

/*-----------------------------------------------------------*/

static void prvReadLightSensor(void *pvParameters)
{
    struct AMessage xMessage; //, *pxPointerToxMessage;
    xMessage.ulMessageID = TASK1_ID;
    for (;;)
    {
        // // Wait for semaphore to be given by the timer ISR
        if (xSemaphoreTake(xTimerSemaphore, portMAX_DELAY) == pdPASS)
        {
            // Read and convert light value from OPT3001 sensor
            bool success;
            uint16_t rawData = 0;
            float convertedLux = 0;

            success = sensorOpt3001Read(&rawData);

            // Check if it was a success
            if (success)
            {
                sensorOpt3001Convert(rawData, &convertedLux);

                // Construct Text
                uint16_t lux_int = (int)convertedLux;
                // UARTprintf("Lux: %5d\n", lux_int);
                // Perform moving average filtering
                if (filter_data)
                {
                    uint16_t filteredValue = movingAverage(lux_int);
                    xMessage.lightValue = filteredValue;
                }
                else
                {
                    xMessage.lightValue = lux_int;
                }

                // Print filtered value to console via UART
                // UARTprintf("Filtered Light Value: %5d\n", filteredValue);


                /* Pull the current time stamp. */
                xMessage.ulTimeStamp = xTaskGetTickCount();

                /* Send the entire structure by value to the queue. */
                xQueueSend(/* The handle of the queue. */
                           xStructQueue,
                           /* The address of the xMessage variable.
                            * sizeof( struct AMessage ) bytes are copied from here into
                            * the queue. */
                           (void *)&xMessage,
                           /* Block time of 0 says don't block if the queue is already
                            * full.  Check the value returned by xQueueSend() to know
                            * if the message was sent to the queue successfully. */
                           (TickType_t)0);
                           vTaskDelay(pdMS_TO_TICKS(500));
                           #define EVENT_READ (1 << 0)
                           extern EventGroupHandle_t xEventGroup;
                xEventGroupSetBits(xEventGroup, EVENT_READ);
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

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("OPT3001 Example\n");

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

    //
    // Enable interrupts to the processor.
    //
    // IntMasterEnable();

    // //
    // // Enable interrupts to the processor.
    // //
    // // I2CMasterIntEnable(I2C0_BASE);
    // /* Enable the GPIO Port P2 interrupt in the NVIC (nested interrupt vector controlle ). */
    // // MAP_I2CMasterIntEnable(I2C0_BASE);
    // // I2CMasterIntEnableEx(I2C0_BASE, (I2C_MASTER_INT_RX_FIFO_FULL | I2C_MASTER_INT_TX_FIFO_EMPTY| I2C_MASTER_INT_RX_FIFO_REQ |
    // //                                  I2C_MASTER_INT_TX_FIFO_REQ  | I2C_MASTER_INT_ARB_LOST     | I2C_MASTER_INT_STOP        |
    // //                                  I2C_MASTER_INT_START        | I2C_MASTER_INT_NACK         | I2C_MASTER_INT_TX_DMA_DONE |
    // //                                  I2C_MASTER_INT_RX_DMA_DONE  | I2C_MASTER_INT_TIMEOUT      | I2C_MASTER_INT_DATA));
    // I2CMasterIntEnableEx(I2C0_BASE, (I2C_MASTER_INT_DATA));
    // IntEnable(INT_I2C0);

    // Test that sensor is set up correctly
    UARTprintf("\nTesting OPT3001 Sensor:\n");
    worked = sensorOpt3001Test();

    while (!worked)
    {
        SysCtlDelay(g_ui32SysClock);
        UARTprintf("\nTest Failed, Trying again\n");
        worked = sensorOpt3001Test();
    }

    UARTprintf("All Tests Passed!\n\n");

    // Initialize opt3001 sensor
    sensorOpt3001Init();
    sensorOpt3001Enable(true);
}


void xButtonsHandler( void )
{
    // Interript status
    uint32_t ui32Status;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 200ms filter */
    if ((xTaskGetTickCount() - g_ui32TimeStamp ) > 200)
    {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            if (!filter_data)
            {
                data_change = true;
                filter_data = true;
                UARTprintf("filter data");
            }
        }
        else if ((ui32Status & USR_SW2) == USR_SW2)
        {
            if (filter_data)
            {
                data_change = true;
                filter_data = false;
                UARTprintf("data not filtered");
            }
        }

    }
    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

static void prvConfigureLED(void)
{
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
    LEDWrite(LED_D1, LED_D1);
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
