#include "que.h"

/*-----------------------------------------------------------*/

/*
 * Called by main() to create the various queue tasks.
 */
void vQueueTask(void);
/*-----------------------------------------------------------*/

/*
 * The is the message structure to send from sensor task to other tasks (e.g. display)
 */
QueueHandle_t xLuxSensorQueue = NULL;
QueueHandle_t xAccelSensorQueue = NULL;
QueueHandle_t xPowerSensorQueue = NULL;
QueueHandle_t xSpeedSensorQueue = NULL;

/*
 * The is the event group which tasks will read (i.e. GUI, E-STOP conditions)
 */
EventGroupHandle_t xSensorEventGroup;

void vQueueTask(void)
{
    // Create event group
    xSensorEventGroup = xEventGroupCreate();
    /* Create the queue used to send complete struct AMessage structures.  This can
    also be created after the schedule starts, but care must be task to ensure
    nothing uses the queue until after it has been created. */
    xLuxSensorQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        LUX_QUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof(SensorMsg));

    xSpeedSensorQueue = xQueueCreate(
        /* The number of items the queue can hold. */
        SPEED_QUEUE_LENGTH,
        /* Size of each item is big enough to hold the
        whole structure. */
        sizeof( SensorMsg ) );
    
    xPowerSensorQueue = xQueueCreate(
                /* The number of items the queue can hold. */
                POWER_QUEUE_LENGTH,
                /* Size of each item is big enough to hold the
                whole structure. */
                sizeof( CalcMsg ) );

    xAccelSensorQueue = xQueueCreate(
                /* The number of items the queue can hold. */
                ACCEL_QUEUE_LENGTH,
                /* Size of each item is big enough to hold the
                whole structure. */
                sizeof( CalcMsg ) );

    if ((xLuxSensorQueue == NULL)) // || (xPointerQueue == NULL)
    {
    }
}
/*-----------------------------------------------------------*/