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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "driverlib/adc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


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

#include "que.h"
#include "float_utils.h"

#define ADC_SEQ_1 1
#define ADC_SEQ_2 2
#define ADC_STEP 0
#define TWELVE_BIT_MAX 4096
// Define moving average window size
#define WINDOW_SIZE 20   // Window size of moving average


/*-----------------------------------------------------------*/

// Semaphores
extern SemaphoreHandle_t xADCSemaphore;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvCurrentSensorTask( void *pvParameters); // Current Sensor Task

/*
 * Called by main() to create the Hello print task.
 */
void vCreateCurrentSensorTask(void);

/*
* Config Functions
*/
void ConfigADCInputs( void );

float MapVoltage(uint32_t);
float CalculateCurrent(float);
float CalculatePower(float);

/*
 * Helper function for calculating the moving average
 */
float rollingAverage(float newValue);
// Declare array to store sampled data for moving average
float sampleWindow[WINDOW_SIZE];
uint8_t idx = 0;

/*
 * Hardware interrupt handlers
 */
void ADC1_SEQ1_ISR(void);
void ADC1_SEQ2_ISR(void);
void xTimer3AIntHandler(void);

/*-----------------------------------------------------------*/

void vCreateCurrentSensorTask( void )
{
    // Config
    ConfigADCInputs();

    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */
    xTaskCreate( prvCurrentSensorTask,
                 "Current",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
}

/*-----------------------------------------------------------*/

// TODO:
//  - Get latest hall effect sensor data
//  - Estimate current using hall effect sensor data, current phase voltage and the state diagram from https://canvas.qut.edu.au/courses/17133/pages/egh456-assessment-2-design-of-embedded-system-for-electric-vehicle-group-project-details?module_item_id=1511474
//  - Create buffer with window size of at least 5
//  - Create filter 
//  - Run at 150Hz or higher (task is always running but the ADC interrupts are only done at 150Hz? Maybe dont even use a task? just fire an event when new measurements are read?)
//  - Provide functions and data to be accessible for the UI
static void prvCurrentSensorTask( void *pvParameters) {
    // Step 1: Initialise values.
    uint32_t ui32Value;
    uint32_t num_samples;
    uint16_t phase_A_raw_est;
    uint16_t phase_B_raw;
    uint16_t phase_C_raw;
    CalcMsg msg;

    for (;;) {
        // Step 2: Read ADCs.
        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_1, &ui32Value); // Read the value from the ADC.
            phase_B_raw = ui32Value;
        }

        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_2, &ui32Value); // Read the value from the ADC.
            phase_C_raw = ui32Value;
        }

        // Step 3: Phase A voltage. Assuming the processor processes ADC faster than the motor can do 1/12th of a revolution.
        // voltage in a 3 phase system always cnacles out, A + B + C = 0; so A = -(B + C).
        // Since our range goes between 0 andd 4096 and a regular sinewave goes between -1 and 1 we must account for this offset
        // using 3/2 * maximum - (B + C). This assures A + B + C = 0 if you were to scale it back between -1 and 1.
        phase_A_raw_est = (TWELVE_BIT_MAX * 3/2 )-(phase_B_raw + phase_C_raw);

        // Step 4: Convert voltage to current in Amps
        uint16_t raw_Vs[] = {phase_A_raw_est, phase_B_raw, phase_C_raw};
        char phase_letters[] = {'A', 'B', 'C'};
        float currents[3];
        for(uint32_t i = 0; i < 3; i++)
        {
            float voltage = MapVoltage(raw_Vs[i]);
            currents[i] = CalculateCurrent(voltage);

            // char voltage_msg[14] = "\t Voltage: %f";
            // char current_msg[14] = "\t Current: %f";
            // UARTprintf("\nPhase %c: ", phase_letters[i]);
            // UARTprintf("\t raw: %d", raw_Vs[i]);
            // UartPrintFloat(voltage_msg, sizeof(voltage_msg), voltage);
            // UARTprintf("\t Current: %d", (int32_t)(currents[i] * 100));
        }

        float total_cur =  fabsf(currents[1]) + fabsf(currents[2]) * 3/2;//+ fabsf(currents[0]);
        float avgCurrent = rollingAverage(total_cur);
        // char current_msg[14] = "\n Current: %f";
        // UartPrintFloat(current_msg, sizeof(current_msg), total_cur);
        float power = CalculatePower(avgCurrent);
        // char power_msg[18] = "\n Total power: %f\n";
        // UartPrintFloat(power_msg, sizeof(power_msg), power);


        // char power_msg[18] = "\n Total power: %f\n";
        // UartPrintFloat(power_msg, sizeof(power_msg), avgPower);

        msg.ClaclulatedData = power;
        msg.TimeStamp = xTaskGetTickCount();

        // Step 5: Add estimate to averaging list.
        xQueueSend(/* The handle of the queue. */
            xPowerSensorQueue,
            /* The address of the LuxMessage variable.
            * sizeof( struct AMessage ) bytes are copied from here into
            * the queue. */
            (void *)&msg,
            /* Block time of 0 says don't block if the queue is already
            * full.  Check the value returned by xQueueSend() to know
            * if the message was sent to the queue successfully. */
            (TickType_t)0);
        // vTaskDelay(pdMS_TO_TICKS( 10 ));
        xEventGroupSetBits(xSensorEventGroup, POWER_DATA_READY);
    }
}        

/*-----------------------------------------------------------*/

/// @brief Converts the raw ADC voltage measurement int SI Voltage.
/// @param raw_voltage The 
float MapVoltage(uint32_t raw_voltage)
{
    float VOLTAGE_MAX = 3.3;
    return ((float)raw_voltage / (float)TWELVE_BIT_MAX) * VOLTAGE_MAX;
}

/// @brief Converts the measured voltage across shunt resistor
/// into current through motor. Uses equation (3) from 
/// https://www.ti.com/lit/ds/slvsdj3d/slvsdj3d.pdf section 8.3.4.1
/// @param voltage is the voltage given by the voltage sensor. 
/// This will need to be converted from raw data into actual voltage
/// as the function expects voltage in SI Voltage.
/// @return Current for the given voltage's phase.
float CalculateCurrent(float voltage) 
{
    float VREF = 3.3;
    float G_SCA = 10.;
    float R_SENSE = 0.007;

    return ((VREF / 2.) - (voltage))/(G_SCA * R_SENSE);
}

float CalculatePower(float current)
{
    return current * 20.; // * 1.732;
}

/*-----------------------------------------------------------*/
static float sum = 0;
// Function to calculate moving average
float rollingAverage(float newValue)
{
    // Update sum with new value
    sum += newValue - sampleWindow[idx];

    // Update sampled data array
    sampleWindow[idx] = newValue;

    // Increment index, wrap around if necessary
    idx = (idx + 1) % WINDOW_SIZE;

    // Return average
    return sum / WINDOW_SIZE;
}

/*-----------------------------------------------------------*/

void ConfigADCInputs(void)
{
    UARTprintf("\nIniitialising ADC hardware");
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC1 ); // Enable ADC1 Perhipheral

    // ADC ISENC (Phase C voltage) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );

    // ADCClockConfigSet()

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_1, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC1_BASE, ADC_SEQ_1, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_END );
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_1 );

    ADCIntRegister(ADC1_BASE, ADC_SEQ_1, ADC1_SEQ1_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS1); // Enables Interrupts for specific Sequence
    // --------------------------------------------------------------------------------------------------------------
    
    // ADC ISENB (Phase B voltage) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTD_BASE, GPIO_PIN_7 );

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_2, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC1_BASE, ADC_SEQ_2, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH4 | ADC_CTL_END );
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_2 );

    ADCIntRegister(ADC1_BASE, ADC_SEQ_2, ADC1_SEQ2_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS2); // Enables Interrupts for specific Sequence
    UARTprintf("\nDone setting up ADC hardware.");
    TimerEnable(TIMER3_BASE, TIMER_A);
}

/*-----------------------------------------------------------*/

/* Interrupt handlers */

// ADC ISR for ISENC (Phase C voltage)
void ADC1_SEQ1_ISR(void) 
{
    BaseType_t xVoltageSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xVoltageSensorTaskWoken);
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);
    portYIELD_FROM_ISR( &xVoltageSensorTaskWoken );  
}

// ADC ISR for ISENB (Phase B voltage)
void ADC1_SEQ2_ISR(void)
{
    BaseType_t xVoltageSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xVoltageSensorTaskWoken);
    ADCIntClear(ADC1_BASE, ADC_SEQ_2);
    portYIELD_FROM_ISR( &xVoltageSensorTaskWoken );  
}

void xTimer3AIntHandler(void)
{
    ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);
    ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_2);

    // Clear the timer interrupt.
    //  /* Clear the hardware interrupt flag for Timer 0A. */
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}
