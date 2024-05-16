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

/*
 * Hardware interrupt handlers
 */

void ADC1_SEQ1_ISR(void);
void ADC1_SEQ2_ISR(void);
void ADC1_SEQ3_ISR(void);

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


static void prvCurrentSensorTask( void *pvParameters) {
    uint32_t ui32Value;
    uint32_t num_samples;

    for (;;) {
        ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_1);

        //UARTprintf("Before Sem!\n\n");
        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_1, &ui32Value); // Read the value from the ADC.
            //UARTprintf("\nPhase C ADC: %d", ui32Value);
            //UARTprintf("\nNum Samples: %d\n\n", num_samples);
        }

        ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_2);

        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_2, &ui32Value); // Read the value from the ADC.
            //UARTprintf("\nPhase B ADC: %d", ui32Value);
            //UARTprintf("\nNum Samples: %d\n\n", num_samples);
        }

        ADCProcessorTrigger(ADC1_BASE, ADC_SEQ_3);

        if( xSemaphoreTake(xADCSemaphore, portMAX_DELAY) == pdPASS) {
            num_samples = ADCSequenceDataGet(ADC1_BASE, ADC_SEQ_3, &ui32Value); // Read the value from the ADC.
            //UARTprintf("\nPhase A ADC: %d", ui32Value);
            //UARTprintf("\nNum Samples: %d\n\n", num_samples);
        }

        vTaskDelay(pdMS_TO_TICKS( 250 ));
    }

}        

/*-----------------------------------------------------------*/

//uint16_t ScaleADC()

void ConfigADCInputs(void)
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_ADC1 ); // Enable ADC1 Perhipheral

    // ADC ISENC (Phase C Current) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTE_BASE, GPIO_PIN_3 );

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_1, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC1_BASE, ADC_SEQ_1, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_END );
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_1 );

    ADCIntRegister(ADC1_BASE, ADC_SEQ_1, ADC1_SEQ1_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS1); // Enables Interrupts for specific Sequence
    // --------------------------------------------------------------------------------------------------------------
    
    // ADC ISENB (Phase B Current) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTD_BASE, GPIO_PIN_7 );

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_2, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC1_BASE, ADC_SEQ_2, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH4 | ADC_CTL_END );
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_2 );

    ADCIntRegister(ADC1_BASE, ADC_SEQ_2, ADC1_SEQ2_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS2); // Enables Interrupts for specific Sequence
    // --------------------------------------------------------------------------------------------------------------
    
    // ADC ISENA (Phase A Current) Config
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
    //Makes GPIO an INPUT and sets them to be ANALOG
    GPIOPinTypeADC( GPIO_PORTA_BASE, GPIO_PIN_6 );

    ADCSequenceConfigure( ADC1_BASE, ADC_SEQ_3, ADC_TRIGGER_PROCESSOR, 0 );
    //uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config
    ADCSequenceStepConfigure( ADC1_BASE, ADC_SEQ_3, ADC_STEP, ADC_CTL_IE | ADC_CTL_CH0 | ADC_CTL_END );
    ADCSequenceEnable( ADC1_BASE, ADC_SEQ_3 );

    ADCIntRegister(ADC1_BASE, ADC_SEQ_3, ADC1_SEQ3_ISR); // Registers the ISR with the specific ADC and Sequence.
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS3); // Enables Interrupts for specific Sequence
    // --------------------------------------------------------------------------------------------------------------
}

/* Interrupt handlers */

// ADC ISR for ISENC (Phase C Current)
void ADC1_SEQ1_ISR(void) 
{
    BaseType_t xCurrentSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xCurrentSensorTaskWoken);
    ADCIntClear(ADC1_BASE, ADC_SEQ_1);
    portYIELD_FROM_ISR( &xCurrentSensorTaskWoken );  
}

// ADC ISR for ISENB (Phase B Current)
void ADC1_SEQ2_ISR(void)
{
    BaseType_t xCurrentSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xCurrentSensorTaskWoken);
    ADCIntClear(ADC1_BASE, ADC_SEQ_2);
    portYIELD_FROM_ISR( &xCurrentSensorTaskWoken );  
}

// ADC ISR for ISENA (Phase A Current)
void ADC1_SEQ3_ISR(void)
{
    BaseType_t xCurrentSensorTaskWoken;
    xSemaphoreGiveFromISR( xADCSemaphore, &xCurrentSensorTaskWoken);
    ADCIntClear(ADC1_BASE, ADC_SEQ_3);
    portYIELD_FROM_ISR( &xCurrentSensorTaskWoken );  
}