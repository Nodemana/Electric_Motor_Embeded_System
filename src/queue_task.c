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

/*
 * The number of items the queue can hold.  This is 4 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty.
 */
#define mainQUEUE_LENGTH (4)

/*
 * The queue used by both tasks.
 */
typedef struct LuxMessage
{
    uint32_t ulMessageID;
    uint16_t lightValue;
    uint32_t ulTimeStamp;
} LuxMessage;

/*
 * Queue used to send and receive complete struct AMessage structures.
 */
QueueHandle_t xLuxSensorQueue = NULL;

/*
 * Called by main() to create the various queue tasks.
 */
void vQueueTask(void);
/*-----------------------------------------------------------*/

// // Declare event group handle
EventGroupHandle_t xEventGroup;

void vQueueTask(void)
{
    // Create event group
    xEventGroup = xEventGroupCreate();

    /* Create the queue used to send complete struct AMessage structures.  This can
    also be created after the schedule starts, but care must be task to ensure
    nothing uses the queue until after it has been created. */
    xLuxSensorQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(LuxMessage));

    if ((xLuxSensorQueue == NULL)) // || (xPointerQueue == NULL)
    {
    }
}
/*-----------------------------------------------------------*/