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

#include "motorlib.h"

#define ADC_SEQ 1
#define ADC_STEP 0

/*-----------------------------------------------------------*/

// Semaphores
extern SemaphoreHandle_t xADCSemaphore;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvMotorTask( void *pvParameters );
static void prvCurrentSensorTask( void *pvParameters); // Current Sensor Task

/*
 * Called by main() to create the Hello print task.
 */
void vCreateMotorTask( void );
void ConfigADCInputs( void );

/*
 * Hardware interrupt handlers
 */

void ADC0_ISR(void);

/*-----------------------------------------------------------*/

void vCreateMotorTask( void )
{
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
    xTaskCreate( prvMotorTask,
                 "Motor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

    xTaskCreate( prvCurrentSensorTask,
                 "Current",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
}
/*-----------------------------------------------------------*/

static void prvMotorTask( void *pvParameters )
{
    uint16_t duty_value = 5;
    uint16_t period_value = 50;
    int32_t Hall_A;
    int32_t Hall_B;
    int32_t Hall_C;


    bool success = false;

    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    success = initMotorLib(period_value);
    /* Set at >10% to get it to start */
    setDuty(duty_value);

    //UARTprintf("\n Success: %d", success);
    /* Kick start the motor */

    //1. Read hall effect sensors
    // Do an initial read of the hall effect sensor GPIO lines
    Hall_A = (GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 3) & 0x01;

    //UARTprintf("\nHall A: %d", Hall_A);

    Hall_B = (GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) >> 2) & 0x01;

    //UARTprintf("\nHall B: %d", Hall_B);

    Hall_C = (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2) & 0x01;

    //UARTprintf("\nHall C: %d", Hall_C);

    // give the read hall effect sensor lines to updateMotor() to move the motor
    updateMotor(Hall_A, Hall_B, Hall_C);

    // one single phase
    // Recommendation is to use an interrupt on the hall effect sensors GPIO lines 
    // So that the motor continues to be updated every time the GPIO lines change from high to low
    // or low to high
    // Include the updateMotor function call in the ISR to achieve this behaviour.

    enableMotor();
    /* Motor test - ramp up the duty cycle from 10% to 100%, than stop the motor */
    for (;;)
    {
        //ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);
        if(duty_value>=period_value){
            stopMotor(1);
        } else {
            setDuty(duty_value);
            vTaskDelay(pdMS_TO_TICKS( 250 ));
            duty_value++;
            
        }
       // while(!ADCIntStatus(ADC0_BASE, ADC_SEQ, false)) { } // While Transmitting wait..
        //ADC0_ISR();

    }
}

static void prvCurrentSensorTask( void *pvParameters) {
    uint32_t ui32Value;
    uint32_t num_samples;

    //UARTprintf("Before ADC Config\n\n");
    // Initialise ADC
    ConfigADCInputs();
    //UARTprintf("After ADC Config\n\n");

    for (;;) {
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);
        //UARTprintf("Before Sem!\n\n");
        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC0_BASE, ADC_SEQ, &ui32Value); // Read the value from the ADC.
            UARTprintf("\nADC: %d", ui32Value);
            UARTprintf("\nNum Samples: %d\n\n", num_samples);
        }
        vTaskDelay(pdMS_TO_TICKS( 250 ));
    }

}        

/*-----------------------------------------------------------*/


/* Interrupt handlers */

void HallSensorHandler(void)
{
    int32_t Hall_A;
    int32_t Hall_B;
    int32_t Hall_C;

    //1. Read hall effect sensors
    Hall_A = (GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 3) & 0x01;
    // Shift right by the pin number to get the logical value (0 or 1)

    Hall_B = (GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) >> 2) & 0x01;


    Hall_C = (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2) & 0x01;

    //2. call update motor to change to next phase
    updateMotor(Hall_A, Hall_B, Hall_C);
    
    //3. Clear interrupt
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTN_BASE, GPIO_PIN_2);

    // Could also add speed sensing code here too.

}


void ConfigADCInputs(void)
{
    
    // ADC ISENC (Phase C Current) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC0 );
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );

    ADCSequenceConfigure( ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC0_BASE, ADC_SEQ, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_END );
    ADCSequenceEnable( ADC0_BASE, ADC_SEQ );

    ADCIntRegister(ADC0_BASE, ADC_SEQ, ADC0_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC0_BASE, ADC_INT_SS1); // Enables Interrupts for specific Sequence

}

void ADC0_ISR(void) 
{
    BaseType_t xCurrentSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xCurrentSensorTaskWoken);
    ADCIntClear(ADC0_BASE, ADC_SEQ);
    portYIELD_FROM_ISR( &xCurrentSensorTaskWoken );  
}