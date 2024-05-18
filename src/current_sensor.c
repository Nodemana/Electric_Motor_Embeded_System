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

#define ADC_SEQ_0 0
#define ADC_SEQ_1 1
#define ADC_SEQ_2 2
#define ADC_SEQ_3 3
#define ADC_STEP 0


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

void UartPrintFloat(char*, uint32_t, float);
float MapVoltage(uint32_t);
float CalculateCurrent(float);

/*
 * Hardware interrupt handlers
 */

void ADC0_SEQ1_ISR(void);
void ADC0_SEQ2_ISR(void);
void ADC0_SEQ3_ISR(void);

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

// TDOD:
//  - Get latest hall effect sensor data
//  - Estimate current using hall effect sensor data, current phase voltage and the state diagram from https://canvas.qut.edu.au/courses/17133/pages/egh456-assessment-2-design-of-embedded-system-for-electric-vehicle-group-project-details?module_item_id=1511474
//  - Create buffer with window size of at least 5
//  - Create filter 
//  - Run at 150Hz or higher (task is always running but the ADC interrupts are only done at 150Hz? Maybe dont even use a task? just fire an event when new measurements are read?)
//  - Provide functions and data to be accessible for the UI
static void prvCurrentSensorTask( void *pvParameters) {
    uint32_t ui32Value;
    uint32_t num_samples;

    for (;;) {
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_1);

        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC0_BASE, ADC_SEQ_1, &ui32Value); // Read the value from the ADC.
            // UARTprintf("\nPhase C ADC: %d", ui32Value);
            // UARTprintf("\nNum Samples: %d", num_samples);
            float voltage = MapVoltage(ui32Value);
            float current = CalculateCurrent(voltage);

            // uint32_t vmsd = (uint32_t)voltage;
            // uint32_t vd1 = (uint32_t)(voltage * 10) - (vmsd * 10);
            // uint32_t vd2 = (uint32_t)(voltage * 100) - (vmsd * 100) - (vd1 * 10);
            // UARTprintf("\t Voltage: %d.%d%d", vmsd, vd1, vd2);
            // UARTprintf("\ncurrent: %d", (int)(current * 100));
            // uint32_t cmsd = (uint32_t)current;
            // uint32_t cd1 = (uint32_t)(current * 10) - (cmsd * 10);
            // uint32_t cd2 = (uint32_t)(current * 100) - (cmsd * 100) - (cd1 * 10);
            // UARTprintf("\n Current: %d.%d%d", cmsd, cd1, cd2);
            char voltage_msg[14] = "\t Voltage: %f";
            char current_msg[14] = "\t Current: %f";
            UARTprintf("\nPhase C: ");
            UartPrintFloat(voltage_msg, sizeof(voltage_msg), voltage);
            UartPrintFloat(current_msg, sizeof(current_msg), current);
        }

        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_2);

        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC0_BASE, ADC_SEQ_2, &ui32Value); // Read the value from the ADC.
            float voltage = MapVoltage(ui32Value);
            float current = CalculateCurrent(voltage);
            char voltage_msg[14] = "\t Voltage: %f";
            char current_msg[14] = "\t Current: %f";
            UARTprintf("\nPhase B: ");
            UartPrintFloat(voltage_msg, sizeof(voltage_msg), voltage);
            UartPrintFloat(current_msg, sizeof(current_msg), current);
        }
        vTaskDelay(pdMS_TO_TICKS( 250 ));
    }

}        

/*-----------------------------------------------------------*/

/// @brief Prints the given string and float value to the uart console.
/// @param message is the string to be printed, this function expects 
/// it to contain "%f" but will still work without it. It can only have 
/// one "%f".
/// @param length is the length of the string.
/// @param value is the float value to be interpolated into the given string.
void UartPrintFloat(char* message, uint32_t length, float value)
{
    // In the string, find the %f and replace with %d.%d%d.
    uint32_t size = length + 5;
    char new_string[size];
    char prev_char = ' ';
    uint32_t offset = 0;
    for(int i = 0; i < length; i++)
    {
        if(message[i] == 'f' && prev_char == '%'){
            // %f
            new_string[i] = 'd';
            new_string[i + 1] = '.';
            new_string[i + 2] = '%';
            new_string[i + 3] = 'd';
            new_string[i + 4] = '%';
            new_string[i + 5] = 'd';
            offset = 5;
        }
        else{
            new_string[i + offset] = message[i];
            prev_char = message[i];
        }
    }

    uint32_t msd = (uint32_t)value;
    uint32_t d1 = (uint32_t)(value * 10) - (msd * 10);
    uint32_t d2 = (uint32_t)(value * 100) - (msd * 100) - (d1 * 10);
    UARTprintf(new_string, msd, d1, d2);
}

/// @brief Converts the raw ADC voltage measurement int SI Voltage.
/// @param raw_voltage The 
float MapVoltage(uint32_t raw_voltage)
{
    uint32_t TWELVE_BIT_MAX = 4096;
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

    return ((VREF / 2) - voltage)/(G_SCA * R_SENSE);
}

float CalculatePower(float current)
{
    return current * 20.;
}

/*-----------------------------------------------------------*/

void ConfigADCInputs(void)
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC0 ); // Enable ADC0 Perhipheral

    // ADC ISENC (Phase C voltage) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );

    ADCSequenceConfigure( ADC0_BASE, ADC_SEQ_1, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC0_BASE, ADC_SEQ_1, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_END );
    ADCSequenceEnable( ADC0_BASE, ADC_SEQ_1 );

    ADCIntRegister(ADC0_BASE, ADC_SEQ_1, ADC0_SEQ1_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC0_BASE, ADC_INT_SS1); // Enables Interrupts for specific Sequence
    // --------------------------------------------------------------------------------------------------------------
    
    // ADC ISENB (Phase B voltage) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTD_BASE, GPIO_PIN_7 );

    ADCSequenceConfigure( ADC0_BASE, ADC_SEQ_2, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC0_BASE, ADC_SEQ_2, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH4 | ADC_CTL_END );
    ADCSequenceEnable( ADC0_BASE, ADC_SEQ_2 );

    ADCIntRegister(ADC0_BASE, ADC_SEQ_2, ADC0_SEQ2_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC0_BASE, ADC_INT_SS2); // Enables Interrupts for specific Sequence
}

/*-----------------------------------------------------------*/

/* Interrupt handlers */

// ADC ISR for ISENC (Phase C voltage)
void ADC0_SEQ1_ISR(void) 
{
    BaseType_t xVoltageSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xVoltageSensorTaskWoken);
    ADCIntClear(ADC0_BASE, ADC_SEQ_1);
    portYIELD_FROM_ISR( &xVoltageSensorTaskWoken );  
}

// ADC ISR for ISENB (Phase B voltage)
void ADC0_SEQ2_ISR(void)
{
    BaseType_t xVoltageSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xVoltageSensorTaskWoken);
    ADCIntClear(ADC0_BASE, ADC_SEQ_2);
    portYIELD_FROM_ISR( &xVoltageSensorTaskWoken );  
}
