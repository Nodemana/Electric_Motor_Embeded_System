/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "semphr.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "que.h"

#include "motorlib.h"
#include "math.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */

#define FILTER_SIZE 10
#define TIMER_TICKS_PER_SEC 10
#define speedQUEUE_LENGTH (10)

enum states
{
    IDLE,
    STARTING,
    RUNNING,
    E_STOPPING,
};

/*-----------------------------------------------------------*/

/*
 * Queue used to send and receive complete struct Message structures.
 */


/* ------------------------------------------------------------------------------------------------
 *                                      Extern Global Variables
 * -------------------------------------------------------------------------------------------------
 */

// Mutexes
extern SemaphoreHandle_t xSpeedSemaphore;
extern SemaphoreHandle_t xSharedSpeedWithController;
extern SemaphoreHandle_t xSharedDutyWithMotor;
extern SemaphoreHandle_t xSharedSpeedESTOPThreshold;
extern SemaphoreHandle_t xSharedDutyWithController;



// Binary Semaphores
extern SemaphoreHandle_t xESTOPSemaphore;
extern SemaphoreHandle_t xControllerSemaphore;

extern uint32_t hall_state_counter;

extern int32_t revolutions_per_minute_shared;
extern int32_t acceleration_RPM_per_second_shared;
extern uint32_t next_duty_shared;

extern motor_control_state;

/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */

// ESTOP Speed Sense Task Handle
TaskHandle_t ESTOPSpeedSenseHandle;


/* ------------------------------------------------------------------------------------------------
 *                                      Extern Function Declarations
 * -------------------------------------------------------------------------------------------------
 */

extern int32_t GetAverage(int32_t *filter_pointer, uint32_t size);
extern int32_t FilterData(int32_t newData, int32_t *filter_pointer, uint32_t speed_filter_current_size, uint32_t max_filter_size);
extern void ShuffleData(int32_t *data, uint32_t size);

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
 * -------------------------------------------------------------------------------------------------
 */

static void prvESTOPSpeedSenseTask(void *pvParameters);
static void prvESTOPControllerTask(void *pvParameters);


/*
 * PID Controller
 */
int32_t PID(int32_t desired_speed, int32_t current_speed, int32_t *integral_error);
int32_t ESTOP_Controller(int32_t current_speed);
uint32_t RPM_to_Duty_Equation(int32_t RPM);




/*
 * Called by main() to initialise all motor associated tasks.
 */
void vCreateMotorTask(void);


/*
 * Hardware interrupt handlers
 */

/*-----------------------------------------------------------*/

void vCreateMotorTask(void)
{
    xTaskCreate(prvESTOPSpeedSenseTask,
                "ESTOP Speed Sensor",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                ESTOPSpeedSenseHandle);

    xTaskCreate(prvESTOPControllerTask,
                "ESTOP Controller",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvESTOPControllerTask(void *pvParameters)
{
    for(;;) {
        if(motor_control_state == E_STOPPING)
        {
            xSemaphoreGive(xESTOPSemaphore);
        } else {
            if(xSemaphoreTake(xESTOPSemaphore, portMAX_DELAY) == pdPASS){
                motor_control_state = E_STOPPING;
                vTaskPrioritySet( ESTOPSpeedSenseHandle, tskIDLE_PRIORITY + 3 );
            }
        }

    }
}