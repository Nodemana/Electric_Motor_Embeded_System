/*
 * hello_task
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

/******************************************************************************
 *
 * The Hello task creates a simple task to handle the UART output for the
 * 'Hello World!' message.  A loop is executed five times with a count down
 * before ending with the self-termination of the task that prints the UART
 * message by use of vTaskDelete.  The loop also includes a one second delay
 * that is achieved by using vTaskDelay.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 */

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

struct Message
{
    uint32_t payload;
    uint32_t timestamp;
} xMessage;

enum states
{
    IDLE,
    STARTING,
    RUNNING,
    E_STOPPING,
} motor_control_state;

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
extern SemaphoreHandle_t xSharedSetSpeedFromGUI;
extern SemaphoreHandle_t xSharedAccelerationThresholdFromGUI;



// Binary Semaphores
extern SemaphoreHandle_t xESTOPSemaphore;
extern SemaphoreHandle_t xControllerSemaphore;


extern uint32_t SpeedThreshold;
extern uint32_t Shared_Set_Speed;
extern double Shared_Acceleration_Threshold;

/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */

int32_t Hall_A;
int32_t Hall_B;
int32_t Hall_C;

volatile uint32_t hall_state_counter = 0;

int32_t revolutions_per_minute_shared;
int32_t acceleration_RPM_per_second_shared;
int32_t desired_speed_RPM_shared;
uint32_t next_duty_shared;
int32_t integral_error = 0;

enum states motor_control_state = STARTING;

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
 * -------------------------------------------------------------------------------------------------
 */

static void prvMotorTask(void *pvParameters);
static void prvSpeedSenseTask(void *pvParameters);
static void prvESTOPTask(void *pvParameters);
static void prvMotorControllerTask(void *pvParameters);


/*
 * PID Controller
 */
int32_t PID(int32_t desired_speed, int32_t current_speed, int32_t *integral_error);
int32_t ESTOP_Controller(int32_t current_speed);
uint32_t RPM_to_Duty_Equation(int32_t RPM);


int32_t GetAverage(int32_t *filter_pointer, uint32_t size);
int32_t FilterData(int32_t newData, int32_t *filter_pointer, uint32_t speed_filter_current_size, uint32_t max_filter_size);
void ShuffleData(int32_t *data, uint32_t size);

int32_t AccelerationCalculation(int32_t newData, int32_t *window_pointer, uint32_t window_current_size, uint32_t max_window_size);
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
    xTaskCreate(prvMotorTask,
                "Motor",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(prvSpeedSenseTask,
                "Speed",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(prvMotorControllerTask,
                "PID",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(prvESTOPTask,
                "ESTOP",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);
}
/*-----------------------------------------------------------*/

static void prvESTOPTask(void *pvParameters)
{
    for(;;) {
        if(xSemaphoreTake(xESTOPSemaphore, portMAX_DELAY) == pdPASS){
            motor_control_state = E_STOPPING;

        }
    }
}

static void prvMotorControllerTask(void *pvParameters)
{
    int32_t current_speed_RPM = NULL;
    int32_t desired_speed_RPM = 0;
    int32_t integral_error = 0;

    for(;;) {
        if(xSemaphoreTake(xControllerSemaphore, portMAX_DELAY) == pdPASS){
            if (xSemaphoreTake(xSharedSpeedWithController, portMAX_DELAY) == pdPASS) {
                    // Receive
                    current_speed_RPM = revolutions_per_minute_shared;
                    // UARTprintf("Speed: %d\n", current_speed_RPM);
                    // UARTprintf("State: %d\n", motor_control_state);
                    //acceleration_RPM_per_second = acceleration_RPM_per_second_shared;
                    xSemaphoreGive(xSharedSpeedWithController);
            }
            if (xSemaphoreTake(xSharedDutyWithMotor, portMAX_DELAY) == pdPASS) {
                if(motor_control_state == E_STOPPING){
                    next_duty_shared = RPM_to_Duty_Equation(ESTOP_Controller(current_speed_RPM));
                } else if(motor_control_state == RUNNING) {
                    // Recieve
                    desired_speed_RPM = desired_speed_RPM_shared;
                    // UARTprintf("Desired Speed: %d\n", desired_speed_RPM);
                    // UARTprintf("Speed: %d\n", current_speed_RPM);
                    // Send
                    next_duty_shared = RPM_to_Duty_Equation(PID(desired_speed_RPM, current_speed_RPM, &integral_error));
                    //UARTprintf("Next Duty: %d\n", next_duty_shared);
                } 
                // UARTprintf("Next Duty: %d\n", next_duty_shared);
                xSemaphoreGive(xSharedDutyWithMotor);
            }

            
            // UARTprintf("Desired RPM: %d\n", desired_speed_RPM);
            // UARTprintf("RPM: %d\n", current_speed_RPM);
            // UARTprintf("RPM/s: %d\n", acceleration_RPM_per_second);
            

        }
    }
}

static void prvMotorTask(void *pvParameters)
{
    uint16_t duty_value = 10;
    uint16_t period_value = 100;
    //uint16_t desired_duty = 100;

    int32_t desired_speed_RPM = 1500;

    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    initMotorLib(period_value);
    /* Set at >10% to get it to start */
    setDuty(duty_value);

    /* Kick start the motor */

    // 1. Read hall effect sensors
    //  Do an initial read of the hall effect sensor GPIO lines
    Hall_A = (GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 3) & 0x01;

    // UARTprintf("\nHall A: %d", Hall_A);

    Hall_B = (GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) >> 2) & 0x01;

    // UARTprintf("\nHall B: %d", Hall_B);

    Hall_C = (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2) & 0x01;

    // UARTprintf("\nHall C: %d\n", Hall_C);

    // give the read hall effect sensor lines to updateMotor() to move the motor
    updateMotor(Hall_A, Hall_B, Hall_C);

    enableMotor();
    for (;;)
    {
        if(xSemaphoreTake(xSharedSetSpeedFromGUI, 0) == pdPASS) {
            desired_speed_RPM = Shared_Set_Speed;
            //UARTprintf("desired_speed_RPM: %d\n", desired_speed_RPM);

            xSemaphoreGive(xSharedSetSpeedFromGUI);
        }
        switch (motor_control_state)
        {
        case IDLE:
            //UARTprintf("IDLE");
            //vTaskDelay(portMAX_DELAY);
            //motor_control_state = STARTING;
            break;
        case STARTING:
            // motor_error = desired_duty - duty_value;
            // duty_value = PID(motor_error, duty_value);
            // setDuty(duty_value);
            // if(motor_error == 0){
            motor_control_state = RUNNING;
            //};
            break;
        case RUNNING:
            //UARTprintf("RUNNING");
            if (xSemaphoreTake(xSharedDutyWithMotor, 0) == pdPASS) {
                // Recieve
                setDuty(next_duty_shared);
                // if (duty_value < desired_duty){
                //     duty_value += 1;
                // }
                // setDuty(duty_value);

                // Send
                desired_speed_RPM_shared = desired_speed_RPM;
                xSemaphoreGive(xSharedDutyWithMotor);
            }

            // UARTprintf("Desured Value: %d\n", desired_duty);
            // UARTprintf("Error Value: %d\n", motor_error);

            // UARTprintf("Duty Value: %d\n", duty_value);
            break;
        case E_STOPPING:
            desired_speed_RPM = 0;
            if (xSemaphoreTake(xSharedDutyWithMotor, 0) == pdPASS) {
                //UARTprintf("ESTOPPING");
                if(next_duty_shared < 15){
                    next_duty_shared = 0;
                    setDuty(next_duty_shared);
                    disableMotor();
                    motor_control_state = IDLE;
                } else {
                    // Recieve
                    setDuty(next_duty_shared);

                    // Send
                    desired_speed_RPM_shared = desired_speed_RPM;
                }
                xSemaphoreGive(xSharedDutyWithMotor);
            }
            break;
        default:
            return -1;
        }
    }
}

static void prvSpeedSenseTask(void *pvParameters)
{
    struct SensorMsg xMessage;

    int32_t last_revolutions_per_minute = 0;
    int32_t revolutions_per_minute_filter[FILTER_SIZE];
    int32_t speed_filter_current_size = 0;

    int32_t acceleration_RPM_per_second;
    int32_t acceleration_RPM_per_second_filter[FILTER_SIZE];
    int32_t acceleration_filter_current_size = 0;

    // int32_t revolutions_per_minute_one_second_window[TIMER_TICKS_PER_SEC];
    // int32_t window_current_size = 0;

    // TickType_t time_difference;
    TickType_t TickCount_Prev = 0;
    TickType_t TickCount_Curr;
    TickType_t TickCount;

    double TimeSinceLastTaskRun;
    double revolutions_per_second_double;
    double num_revs;

    for (;;)
    {
        if (xSemaphoreTake(xSpeedSemaphore, portMAX_DELAY) == pdPASS)
        {

            TickCount_Curr = xTaskGetTickCount();

            TickCount = TickCount_Curr - TickCount_Prev;

            //UARTprintf("Change in tick: %d\n", TickCount);

            TimeSinceLastTaskRun = (double)TickCount / configTICK_RATE_HZ;

            // TimeSinceLastTaskRun = (double)TickCount_Curr / configTICK_RATE_HZ;

            num_revs = ((double)hall_state_counter / 12.0);
            hall_state_counter = 0;

            //UARTprintf("Number of revs: %d\n", (int)num_revs);

            revolutions_per_second_double = num_revs / TimeSinceLastTaskRun;

            int32_t revolutions_per_second = (int)round(revolutions_per_second_double); // Timer runs at 1/10 of a second. 12 Hall states in one revolution.

            int32_t revolutions_per_minute = revolutions_per_second * 60;
            //UARTprintf("RPM before filtered: %d\n", revolutions_per_minute);

            int32_t filtered_revoltutions_per_minute = FilterData(revolutions_per_minute, revolutions_per_minute_filter, speed_filter_current_size, FILTER_SIZE);
            if (speed_filter_current_size != (FILTER_SIZE - 1))
            {
                speed_filter_current_size += 1;
            }

            // ESTOP CODE
            // if(xSemaphoreTake(xSharedSpeedESTOPThreshold, 0) == pdPASS)
            // {
            if(filtered_revoltutions_per_minute > SpeedThreshold){
                xSemaphoreGive(xESTOPSemaphore);
            }
            // }


            // ACCELERATION
            acceleration_RPM_per_second = revolutions_per_minute - last_revolutions_per_minute;
        //   acceleration_RPM_per_second = AccelerationCalculation(revolutions_per_minute, revolutions_per_minute_one_second_window, window_current_size, TIMER_TICKS_PER_SEC); // this is per 8th of a second.
        //     if (window_current_size != (TIMER_TICKS_PER_SEC - 1)){
        //         window_current_size += 1;
        //     }
            int32_t filtered_acceleration_RPM_per_second = FilterData(acceleration_RPM_per_second, acceleration_RPM_per_second_filter, acceleration_filter_current_size, FILTER_SIZE);
            if (acceleration_filter_current_size != (FILTER_SIZE - 1))
            {
                acceleration_filter_current_size += 1;
            }

            // Sharing Data With PID
            if (xSemaphoreTake(xSharedSpeedWithController, 0) == pdPASS) {
                //UARTprintf("Before Shared RPM %d\n", filtered_revoltutions_per_minute);
                revolutions_per_minute_shared = filtered_revoltutions_per_minute;
                acceleration_RPM_per_second_shared = filtered_acceleration_RPM_per_second;
                time_step_shared = TimeSinceLastTaskRun;
                xSemaphoreGive(xSharedSpeedWithController);
            }
            xSemaphoreGive(xControllerSemaphore);

            // ANOTHER METHOD FOR ACCELERATION
            // = AccelerationCalculation(revolutions_per_minute, revolutions_per_minute_one_second_window, window_current_size, TIMER_TICKS_PER_SEC); // this is per 8th of a second.
            // if (window_current_size != (TIMER_TICKS_PER_SEC - 1)){
            //     window_current_size += 1;
            // }

            // DEBUG PRINTS
            // UARTprintf("Hall States: %d\n", hall_state_counter);
            // UARTprintf("RPS: %d\n", revolutions_per_second);
            // UARTprintf("RPM: %d\n", revolutions_per_minute);
            UARTprintf("Filtered RPM %d\n", filtered_revoltutions_per_minute);
            UARTprintf("RPM/s: %d\n\n", acceleration_RPM_per_second);

            last_revolutions_per_minute = revolutions_per_minute;

            // TickCount_Prev = xTaskGetTickCount();
            TickCount_Prev = TickCount_Curr;

            // Message Construction
            xMessage.SensorReading = filtered_revoltutions_per_minute;
            xMessage.TimeStamp = TickCount_Prev;
            xMessage.TimeStamp = xTaskGetTickCount();

            /* Send the entire structure by value to the queue. */
            xQueueSend(xSpeedSensorQueue,
                       /* The address of the xMessage variable.
                        * sizeof( struct AMessage ) bytes are copied from here into
                        * the queue. */
                       (void *)&xMessage,
                       /* Block time of 0 says don't block if the queue is already
                        * full.  Check the value returned by xQueueSend() to know
                        * if the message was sent to the queue successfully. */
                       (TickType_t)0);
            xEventGroupSetBits(xSensorEventGroup, SPEED_DATA_READY);
        }
    }
}

void ShuffleData(int32_t *data, uint32_t size)
{
    // Shift all elements to the left by one position
    for (uint32_t i = 0; i < size - 1; ++i)
    {
        data[i] = data[i + 1];
    }
}

int32_t FilterData(int32_t newData, int32_t *filter_pointer, uint32_t speed_filter_current_size, uint32_t max_filter_size)
{
    if (speed_filter_current_size < (max_filter_size - 1))
    {
        // Buffer is not full, simply add the new data
        filter_pointer[speed_filter_current_size] = newData;
        //UARTprintf("Added Data: %d\n",  filter_pointer[speed_filter_current_size]);
    }
    else
    {
        // Buffer is full, shuffle data and insert newData
        ShuffleData(filter_pointer, max_filter_size);
        filter_pointer[max_filter_size - 1] = newData; //
        //UARTprintf("Added Data: %d\n", filter_pointer[max_filter_size - 1]);
    }
    return GetAverage(filter_pointer, speed_filter_current_size);
}

int32_t GetAverage(int32_t *filter_pointer, uint32_t size)
{
    int32_t sum = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        //UARTprintf("Summing Data: %d\n", filter_pointer[i]);
        if((filter_pointer[i] > 20000) || (filter_pointer[i] < -20000)){
            sum += filter_pointer[i+1%FILTER_SIZE]; // This is disgusting
        } else {
            sum += filter_pointer[i];
        }

    }
    //UARTprintf("END AVERAGE");
    return sum / size;
}

int32_t AccelerationCalculation(int32_t newData, int32_t *window_pointer, uint32_t window_current_size, uint32_t max_window_size)
{
    if (window_current_size < (max_window_size - 1))
    {
        // Buffer is not full, simply add the new data
        window_pointer[window_current_size] = newData;
        // UARTprintf("Added Data: %d\n",  filter_pointer[speed_filter_current_size]);
    }
    else
    {
        // Buffer is full, shuffle data and insert newData
        ShuffleData(window_pointer, max_window_size);
        window_pointer[max_window_size - 1] = newData; //
        // UARTprintf("Added Data: %d\n", filter_pointer[max_filter_size - 1]);
    }
    return window_pointer[window_current_size - 1] - window_pointer[0];
}


uint32_t RPM_to_Duty_Equation(int32_t RPM) {
    // UARTprintf("Conversion: %d\n", (uint32_t)round((0.0000006 * (RPM*RPM) + 0.0003*RPM + 13.686)));
    return (uint32_t)round((0.0000006 * (RPM*RPM) + 0.0003*RPM + 13.686));
}

/*
 * PID Controller
 */

int32_t PID(int32_t desired_speed, int32_t current_speed, int32_t *integral_error_ptr) {
    float Kp = 2;
    float Ki = 1;
    // UARTprintf("Desired RPM: %d\n", desired_speed);
    // UARTprintf("Current RPM: %d\n", current_speed);
    
    int32_t acceleration = (desired_speed - current_speed);
    *integral_error_ptr += acceleration;  // Accumulate the integral error
        if (*integral_error_ptr > 100) {
        *integral_error_ptr = 100;
    }
    if (*integral_error_ptr < -100) {
        *integral_error_ptr = -100;
    }
    // UARTprintf("Acceleration/Error: %d\n", acceleration);
    // UARTprintf("Integral Error: %d\n", *integral_error_ptr);
    int32_t total_error = acceleration;
    if (total_error > 500) {
        total_error = 500;
    }
    if (total_error < -500) {
        total_error = -500;
    }
    // UARTprintf("Minned Acceleration/Error: %d\n", total_error);
    // UARTprintf("Output: %d\n\n", (int32_t)round(current_speed + total_error * Kp + (*integral_error_ptr * Ki))); //  + (*integral_error_ptr * Ki)

    return (int32_t)round(current_speed + total_error * Kp  + (*integral_error_ptr * Ki)); // + (*integral_error_ptr * Ki)
}

int32_t ESTOP_Controller(int32_t current_speed) {
    int32_t acceleration = -2000;
    return (int32_t)round(current_speed + acceleration); 
}

/*-----------------------------------------------------------*/

/* Interrupt handlers */

void HallSensorHandler(void)
{
    // 1. Read hall effect sensors
    Hall_A = (GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 3) & 0x01;
    // Shift right by the pin number to get the logical value (0 or 1)

    Hall_B = (GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) >> 2) & 0x01;

    Hall_C = (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2) & 0x01;

    // 2. call update motor to change to next phase
    updateMotor(Hall_A, Hall_B, Hall_C);

    // 3. Clear interrupt
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTN_BASE, GPIO_PIN_2);

    // Increment State Counter:
    hall_state_counter++;
}

/*
 Timer 2A ISR
*/
void xTimer2AIntHandler_SpeedTimerISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Give the semaphore to unblock the task.
    xSemaphoreGiveFromISR(xSpeedSemaphore, &xHigherPriorityTaskWoken);
    // Perform a context switch if needed.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // Clear the timer interrupt.
    //  /* Clear the hardware interrupt flag for Timer 2A. */
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

/*
 Timer 3B ISR
*/
void xTimer3BIntHandler_ESTOPController(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSpeedSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


    TimerIntClear(TIMER3_BASE, TIMER_TIMB_TIMEOUT);
}

/*
 Timer 2B ISR
*/
// void xTimer2BIntHandler_PIDTimerISR(void)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     // Give the semaphore to unblock the task.
//     xSemaphoreGiveFromISR(xControllerSemaphore, &xHigherPriorityTaskWoken);
//     // Perform a context switch if needed.
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     // Clear the timer interrupt.
//     //  /* Clear the hardware interrupt flag for Timer 0A. */
//     TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);
// }