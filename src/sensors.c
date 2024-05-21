/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "timers.h"
#include "driverlib/i2c.h"


// Motor lib
#include <motorlib.h>

void vTimerCallback(TimerHandle_t xTimer);

void vSoftwareTimer( void );

void vSoftwareTimer( void )
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

}

/* Timer Call Back function */
void vTimerCallback(TimerHandle_t xTimer)
{
    // Toggle an LED or perform any other task
    UARTprintf("Timer Callback Executed\n");
}