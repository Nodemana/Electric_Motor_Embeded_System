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

/*
 * The number of items the queue can hold.  This is 4 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty.
 */
#define mainQUEUE_LENGTH (4)

/*
 * The queue used by both tasks.
 */
struct AMessage
{
    uint32_t ulMessageID;
    uint16_t lightValue;
    uint32_t ulTimeStamp;
} xMessage;

/*
 * Queue used to send and receive complete struct AMessage structures.
 */
QueueHandle_t xStructQueue = NULL;

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
    xStructQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        mainQUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(xMessage));

    if ((xStructQueue == NULL)) // || (xPointerQueue == NULL)
    {
    }
}
/*-----------------------------------------------------------*/