#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H

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

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */
/* Message ID's of each task */
#define LUX_MESSAGE      0              // Message ID for the light sensor
#define ACCEL_MESSAGE    1              // Message ID for the temperatur sensor
#define POWER_MESSAGE    2              // Message ID for the power sensor
#define SPEED_MESSAGE    3              // Message ID for the speed sensor

/* Event Bits to set of each task when data has been read */
#define LUX_DATA_READY      (1 << 0)    // New data from light sensor is ready
#define ACCEL_DATA_READY    (1 << 1)    // New data from temperatur sensor is ready
#define POWER_DATA_READY    (1 << 2)    // New data from power sensor is ready
#define SPEED_DATA_READY    (1 << 3)    // New data from speed sensor is ready

/*
 * The number of items the queue can hold.  This is 4 as the receive task
 * will remove items as they are added, meaning the send task should always find
 * the queue empty.
 */
#define LUX_QUEUE_LENGTH   (4)          // Que length for the light sensor
#define ACCEL_QUEUE_LENGTH (4)          // Que length for the temperature sensor
#define POWER_QUEUE_LENGTH (4)          // Que length for the power sensor
#define SPEED_QUEUE_LENGTH (4)          // Que length for the speed sensor

/* ------------------------------------------------------------------------------------------------
 *                                           Variables
 * -------------------------------------------------------------------------------------------------
 */

/*
 * The is the message structure to send from sensor task to other tasks (e.g. display)
 */
typedef struct SensorMsg
{
    uint32_t SensorReading;
    uint32_t TimeStamp;
} SensorMsg;

typedef struct CalcMsg
{
    float ClaclulatedData;
    uint32_t TimeStamp;
} CalcMsg;


/*
 * The is the event group which tasks will read (i.e. GUI, E-STOP conditions)
 */
extern EventGroupHandle_t xSensorEventGroup;

/*
 * Queue used to send and receive complete struct AMessage structures. 
 * One has been created for each sensor
 */
extern QueueHandle_t xLuxSensorQueue;
extern QueueHandle_t xAccelSensorQueue;
extern QueueHandle_t xPowerSensorQueue;
extern QueueHandle_t xSpeedSensorQueue;

#endif // GLOBAL_STRUCT_H