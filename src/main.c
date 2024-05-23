/******************************************************************************
 *
 * This motor test project provides an example of how to use the motor library
 * with a platformio / freeRTOS project. The main script initialises the hall
 * sensor interrupt, which run the update_motor function. The program also launches
 * a task that initialises the motors before ramping the speed from 10% to 100%.
 * Once the speed reaches 100%, the motor is stopped and the program ends.
 *
 *
 ******************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * -------------------------------------------------------------------------------------------------
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"

// Motor lib
#include <motorlib.h>

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                      Extern Global Variables
 * -------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */

/* The system clock frequency. */
uint32_t g_ui32SysClock;

// Semaphores
SemaphoreHandle_t xADCSemaphore = NULL;
SemaphoreHandle_t xSpeedSemaphore = NULL;

/* Global for binary semaphore shared between tasks. */
SemaphoreHandle_t xTimerSemaphore = NULL;

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
SemaphoreHandle_
 * -------------------------------------------------------------------------------------------------
 */

/* Set up the hardware ready to run this demo. */
static void prvSetupHardware(void);

void Config_Timers(void);

/* This function sets up UART0 to be used for a console to display information
 * as the example is running. */
static void prvConfigureUART(void);

/* Set up the I2C2 for temp and lux sensor. */
static void prvConfigureI2C2(void);

/* Timer configuration */
static void prvConfigureHWTimer(void);

/* API to trigger the 'Hello world' task. */
extern void vCreateCurrentSensorTask( void );

extern void vCreateMotorTask(void);

/* Software Timer */
// extern void vSoftwareTimer( void );

/* API to trigger the DISP task. */
extern void vDISPTask(void);

/* API to trigger the LUX task. */
extern void vLUXTask(void);

/* API to trigger the que task */
extern void vQueueTask(void);
extern void vCreateCurrentSensorTask( void );

static void prvConfigureHallInts(void);


/* ------------------------------------------------------------------------------------------------
 *                                      Functions
 * -------------------------------------------------------------------------------------------------
 */

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    // Semaphore Initialisation
    xADCSemaphore = xSemaphoreCreateBinary();
    xSpeedSemaphore = xSemaphoreCreateBinary();
    xTimerSemaphore = xSemaphoreCreateBinary();


    if ((xADCSemaphore != NULL) && (xSpeedSemaphore != NULL) && (xTimerSemaphore != NULL))
    {
        vDISPTask();
        vLUXTask();
        vQueueTask();
        vCreateMotorTask();
        vCreateCurrentSensorTask();

    /* Start the tasks and timer running. */
        vTaskStartScheduler();
    }
    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for (;;);
}
/*-----------------------------------------------------------*/
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
}

/*-----------------------------------------------------------*/

static void prvConfigureI2C2(void)
{
    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    I2CMasterEnable(I2C2_BASE);
}

/*-----------------------------------------------------------*/

static void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at 10 Hz. */
    TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock / 8);

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER2A);

     /* Enable Timer 0A. */
    TimerEnable(TIMER2_BASE, TIMER_A);

    // /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                             configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();

    Config_Timers();

    /* Configure the I2C2 for temp and lux sensor comms */
    prvConfigureI2C2();

    /* Configure the hardware timer to run in periodic mode. */
    prvConfigureHWTimer();
    
    // GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    // GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);
    // GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2);

    MAP_GPIODirModeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    MAP_GPIODirModeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    MAP_GPIODirModeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Set-up interrupts for hall sensors */
    prvConfigureHallInts();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/
static void prvConfigureHallInts(void)
{
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);

    /* Configure GPIO ports to trigger an interrupt on rising/falling or both edges. */
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_PIN_2);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    /* Enable the Ports interrupt in the NVIC. */
    IntEnable(INT_GPIOM);
    IntEnable(INT_GPIOH);
    IntEnable(INT_GPION);
    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

void Config_Timers(void) {
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_B, g_ui32SysClock/8); // 125 ms

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER1B);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerEnable(TIMER1_BASE, TIMER_B);
}


/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

void *malloc(size_t xSize)
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for (;;)
        ;
}
/*-----------------------------------------------------------*/
