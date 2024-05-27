/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * -------------------------------------------------------------------------------------------------
 */
/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

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

/* UI includes */
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

// Include Event
#include <event_groups.h>

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/radiobutton.h"
#include "grlib/slider.h"
#include "utils/ustdlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "images.h"

// Inlcude que.h
#include "que.h"
#include "float_utils.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Definitions
 * -------------------------------------------------------------------------------------------------
 */
#define NUM_SENSORS     4
#define Y_AXIS_ORIGIN   40
#define Y_AXIS_LENGTH   142
#define X_AXIS_ORIGIN   10
#define X_AXIS_LENGTH   223

#define DRAW_LUX        (0)
#define DRAW_TEMP       (1)
#define DRAW_POWER      (2)
#define DRAW_SPEED      (3)


/* ------------------------------------------------------------------------------------------------
 *                                      Extern Global Variables
 * -------------------------------------------------------------------------------------------------
 */
// Include system clock
extern uint32_t g_ui32SysClock;

/*
 * The is the event group which tasks will read (i.e. GUI, E-STOP conditions)
 */

/*
 * Queue used to send and receive complete struct AMessage structures. 
 * One has been created for each sensor
 */

/* ------------------------------------------------------------------------------------------------
 *                                     Local Global Variables
 * -------------------------------------------------------------------------------------------------
 */
/* Context */
tContext sContext;

typedef enum {
    LUX,
    TEMP,
    POWER,
    SPEED,
    NONE
} Sensors;
Sensors selected_sensor;

uint8_t sensors[NUM_SENSORS] = {DRAW_SPEED, DRAW_POWER, DRAW_TEMP, DRAW_LUX};

/* ------------------------------------------------------------------------------------------------
 *                                      Function Declarations
 * -------------------------------------------------------------------------------------------------
 */
void define_sensor_axis( void );

void clearAxis (int backround_colour );
/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vDISPTask(void);

/*
 * The tasks is to handle the display (using grlib).
 */
static void prvDisplayTask(void *pvParameters);

/*-----------------------------------------------------------*/

/* ------------------------------------------------------------------------------------------------
 *                                      Functions
 * -------------------------------------------------------------------------------------------------
 */
void vDISPTask(void)
{
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate(prvDisplayTask,
                "DISP",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#ifdef ewarm
#pragma data_alignment = 1024
tDMAControlTable psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(psDMAControlTable, 1024)
tDMAControlTable psDMAControlTable[64];
#else
tDMAControlTable psDMAControlTable[64] __attribute__((aligned(1024)));
#endif

//*****************************************************************************
//
// Forward declarations for the globals required to define the widgets at
// compile-time.
//
//*****************************************************************************
void OnPrevious(tWidget *psWidget);
void OnNext(tWidget *psWidget);
void OnIntroPaint(tWidget *psWidget, tContext *psContext);
void OnCanvasPaint(tWidget *psWidget, tContext *psContext);
void OnRadioChange(tWidget *psWidget, uint32_t bSelected);
void OnSliderChange(tWidget *psWidget, int32_t i32Value);
extern tCanvasWidget g_psPanels[];

//*****************************************************************************
//
// The first panel, which demonstrates the canvas widget.
//
//*****************************************************************************
Canvas(g_sCanvas6, g_psPanels, 0, 0,
       &g_sKentec320x240x16_SSD2119, 235, 49, 75, 30,
       CANVAS_STYLE_TEXT,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "[RPM]", 0, 0);

Canvas(g_sCanvas5, g_psPanels, &g_sCanvas6, 0,
       &g_sKentec320x240x16_SSD2119, 235, 83, 75, 30,
       CANVAS_STYLE_TEXT,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "[W]", 0, 0);

Canvas(g_sCanvas4, g_psPanels, &g_sCanvas5, 0,
       &g_sKentec320x240x16_SSD2119, 235, 118, 75, 30,
       CANVAS_STYLE_TEXT,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "[C]", 0, 0);

Canvas(g_sCanvas3, g_psPanels, &g_sCanvas4, 0,
       &g_sKentec320x240x16_SSD2119, 235, 155, 75, 30,
       CANVAS_STYLE_TEXT,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "[LUX]", 0, 0);

Canvas(g_sCanvas2, g_psPanels, &g_sCanvas3, 0,
       &g_sKentec320x240x16_SSD2119, 5, 27, 310, 20,
       CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE | CANVAS_STYLE_TEXT,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, "Date: 30/05/2024 Time: XX:XX ", 0, 0);

Canvas(g_sCanvas1, g_psPanels, &g_sCanvas2, 0,
       &g_sKentec320x240x16_SSD2119, 5, 50, 310, 135,
       CANVAS_STYLE_OUTLINE,
       ClrMidnightBlue, ClrGray, ClrSilver, &g_sFontCm22, 0, 0, 0);

tSliderWidget g_psSliders[] =
    {
        SliderStruct(g_psPanels, g_psSliders + 1, 0,
                     &g_sKentec320x240x16_SSD2119, 10, 87, 220, 25, 0, 100, 25,
                     (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                      SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                     ClrRed, ClrGray, ClrSilver, ClrWhite, ClrWhite,
                     &g_sFontCm20, "Power", 0, 0, OnSliderChange),
        SliderStruct(g_psPanels, g_psSliders + 2, 0,
                     &g_sKentec320x240x16_SSD2119, 10, 122, 220, 25, 0, 100, 25,
                     (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                      SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                     ClrRed, ClrGray, ClrSilver, ClrWhite, ClrWhite,
                     &g_sFontCm20, "Temp", 0, 0, OnSliderChange),
        SliderStruct(g_psPanels, g_psSliders + 3, 0,
                     &g_sKentec320x240x16_SSD2119, 10, 157, 220, 25, 0, 100, 25,
                     (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                      SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                     ClrLightBlue, ClrDarkBlue, ClrSilver, ClrWhite, ClrWhite,
                     &g_sFontCm20, "Light: Night/Day", 0, 0, OnSliderChange),
        SliderStruct(g_psPanels, &g_sCanvas1, 0,
                     &g_sKentec320x240x16_SSD2119, 10, 52, 220, 25, 0, 100, 50,
                     (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                      SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                     ClrGreen, ClrGray, ClrSilver, ClrWhite, ClrWhite,
                     &g_sFontCm18, "Speed Control", 0, 0,
                     OnSliderChange),
};

// Sent these high so they aren't used... for now
#define SLIDER_TEXT_VAL_INDEX 6
#define SLIDER_LOCKED_INDEX 6
#define SLIDER_CANVAS_VAL_INDEX 6

#define NUM_SLIDERS (sizeof(g_psSliders) / sizeof(g_psSliders[0]))

//*****************************************************************************
//
// The second panel, which contains a selection of radio buttons.
//
//*****************************************************************************
tContainerWidget g_psRadioContainers[];
tCanvasWidget g_psRadioButtonIndicators[] =
    {
        CanvasStruct(g_psRadioContainers, g_psRadioButtonIndicators + 1, 0,
                     &g_sKentec320x240x16_SSD2119, 95, 50, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),

        CanvasStruct(g_psRadioContainers, g_psRadioButtonIndicators + 2, 0,
                     &g_sKentec320x240x16_SSD2119, 95, 85, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),

        CanvasStruct(g_psRadioContainers, g_psRadioButtonIndicators + 3, 0,
                     &g_sKentec320x240x16_SSD2119, 95, 120, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),

        CanvasStruct(g_psRadioContainers + 1, 0, 0,
                     &g_sKentec320x240x16_SSD2119, 95, 155, 20, 20,
                     CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0),
};
tRadioButtonWidget g_psRadioButtons[] =
    {
        RadioButtonStruct(g_psRadioContainers, g_psRadioButtons + 1, 0,
                          &g_sKentec320x240x16_SSD2119, 240, 40, 80, 45,
                          RB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                          "Speed", 0, OnRadioChange),
        RadioButtonStruct(g_psRadioContainers, g_psRadioButtons + 2, 0,
                          &g_sKentec320x240x16_SSD2119, 240, 75, 80, 45,
                          RB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                          "Power", 0, OnRadioChange),
        RadioButtonStruct(g_psRadioContainers, g_psRadioButtons + 3, 0,
                          &g_sKentec320x240x16_SSD2119, 240, 110, 80, 45,
                          RB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                          "Temp", 0, OnRadioChange),
        RadioButtonStruct(g_psRadioContainers, g_psRadioButtonIndicators, 0,
                          &g_sKentec320x240x16_SSD2119, 240, 145, 80, 45,
                          RB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                          "Light", 0, OnRadioChange)};
#define NUM_RADIO1_BUTTONS (sizeof(g_psRadioButtons) / \
                            sizeof(g_psRadioButtons[0]))
tContainerWidget g_psRadioContainers[] =
    {
        
        ContainerStruct(g_psPanels + 1, g_psRadioContainers + 1, g_psRadioButtons,
                        &g_sKentec320x240x16_SSD2119, 5, 35, 233, 152,
                        CTR_STYLE_OUTLINE | CTR_STYLE_FILL, ClrWhite, ClrGray, ClrSilver,
                        &g_sFontCm20, 0),
        ContainerStruct(g_psPanels + 1, 0, 0,
                        &g_sKentec320x240x16_SSD2119, 239, 27, 75, 160,
                        CTR_STYLE_OUTLINE | CTR_STYLE_TEXT, 0, ClrGray, ClrSilver,
                        &g_sFontCm20, "Data")};

//*****************************************************************************
//
// An array of canvas widgets, one per panel.  Each canvas is filled with
// black, overwriting the contents of the previous panel.
//
//*****************************************************************************
tCanvasWidget g_psPanels[] =
    {
        CanvasStruct(0, 0, g_psSliders, &g_sKentec320x240x16_SSD2119, 0, 24, 320,
                     166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
        CanvasStruct(0, 0, g_psRadioContainers, &g_sKentec320x240x16_SSD2119, 0,
                     24, 320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0)};

//*****************************************************************************
//
// The number of panels.
//
//*****************************************************************************
#define NUM_PANELS (sizeof(g_psPanels) / sizeof(g_psPanels[0]))

//*****************************************************************************
//
// The names for each of the panels, which is displayed at the bottom of the
// screen.
//
//*****************************************************************************
char *g_pcPanei32Names[] =
    {
        "     Page One     ",
        "     Page Two     ",
        "     S/W Update    "};

//*****************************************************************************
//
// The buttons and text across the bottom of the screen.
//
//*****************************************************************************
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 190,
                  50, 50, PB_STYLE_FILL, ClrBlack, ClrBlack, 0, ClrSilver,
                  &g_sFontCm20, "-", g_pui8Blue50x50, g_pui8Blue50x50Press, 0, 0,
                  OnPrevious);

Canvas(g_sTitle, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 50, 190, 220, 50,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 270, 190,
                  50, 50, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrSilver, &g_sFontCm20, "+", g_pui8Blue50x50,
                  g_pui8Blue50x50Press, 0, 0, OnNext);

//*****************************************************************************
//
// The panel that is currently being displayed.
//
//*****************************************************************************
uint32_t g_ui32Panel;

//*****************************************************************************
//
// Handles presses of the previous panel button.
//
//*****************************************************************************
void OnPrevious(tWidget *psWidget)
{   
    selected_sensor = NONE;
    //
    // There is nothing to be done if the first panel is already being
    // displayed.
    //
    if (g_ui32Panel == 0)
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Decrement the panel index.
    //
    g_ui32Panel--;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if this is the first panel.
    //
    if (g_ui32Panel == 0)
    {
        //
        // Clear the previous button from the display since the first panel is
        // being displayed.
        //
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if the previous panel was the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 2))
    {
        //
        // Display the next button.
        //
        PushButtonImageOn(&g_sNext);
        PushButtonTextOn(&g_sNext);
        PushButtonFillOff(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

//*****************************************************************************
//
// Handles presses of the next panel button.
//
//*****************************************************************************
void OnNext(tWidget *psWidget)
{
    //
    // There is nothing to be done if the last panel is already being
    // displayed.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Increment the panel index.
    //
    g_ui32Panel++;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if the previous panel was the first panel.
    //
    if (g_ui32Panel == 1)
    {
        //
        // Display the previous button.
        //
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if this is the last panel.
    //
    if (g_ui32Panel == (NUM_PANELS - 1))
    {
        //
        // Clear the next button from the display since the last panel is being
        // displayed.
        //
        PushButtonImageOff(&g_sNext);
        PushButtonTextOff(&g_sNext);
        PushButtonFillOn(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }
}

//*****************************************************************************
//
// Handles paint requests for the introduction canvas widget.
//
//*****************************************************************************
void OnIntroPaint(tWidget *psWidget, tContext *psContext)
{
    //
    // Display the introduction text in the canvas.
    //
    GrContextFontSet(psContext, &g_sFontCm18);
    GrContextForegroundSet(psContext, ClrSilver);
    GrStringDraw(psContext, "This application demonstrates the", -1,
                 0, 32, 0);
    GrStringDraw(psContext, "TivaWare Graphics Library.", -1, 0, 50, 0);
    GrStringDraw(psContext, "Each panel shows a different feature of", -1, 0,
                 74, 0);
    GrStringDraw(psContext, "the graphics library. Widgets on the panels", -1, 0,
                 92, 0);
    GrStringDraw(psContext, "are fully operational; pressing them will", -1, 0,
                 110, 0);
    GrStringDraw(psContext, "result in visible feedback of some kind.", -1, 0,
                 128, 0);
    GrStringDraw(psContext, "Press the + and - buttons at the bottom", -1, 0,
                 146, 0);
    GrStringDraw(psContext, "of the screen to move between the panels.", -1, 0,
                 164, 0);
}

//*****************************************************************************
//
// Handles paint requests for the canvas demonstration widget.
//
//*****************************************************************************
void OnCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    uint32_t ui32Idx;

    //
    // Draw a set of radiating lines.
    //
    GrContextForegroundSet(psContext, ClrGoldenrod);
    for (ui32Idx = 50; ui32Idx <= 180; ui32Idx += 10)
    {
        GrLineDraw(psContext, 210, ui32Idx, 310, 230 - ui32Idx);
    }

    //
    // Indicate that the contents of this canvas were drawn by the application.
    //
    GrContextFontSet(psContext, &g_sFontCm12);
    GrStringDrawCentered(psContext, "App Drawn", -1, 260, 50, 1);
}

//*****************************************************************************
//
// Handles notifications from the slider controls.
//
//*****************************************************************************
void OnSliderChange(tWidget *psWidget, int32_t i32Value)
{
    static char pcCanvasText[5];
    static char pcSliderText[5];

    //
    // Is this the widget whose value we mirror in the canvas widget and the
    // locked slider?
    //
    if (psWidget == (tWidget *)&g_psSliders[SLIDER_CANVAS_VAL_INDEX])
    {
        //
        // Yes - update the canvas to show the slider value.
        //
        usprintf(pcCanvasText, "%3d%%", i32Value);
        CanvasTextSet(&g_sCanvas1, pcCanvasText);
        WidgetPaint((tWidget *)&g_sCanvas1);

        //
        // Also update the value of the locked slider to reflect this one.
        //
        SliderValueSet(&g_psSliders[SLIDER_LOCKED_INDEX], i32Value);
        WidgetPaint((tWidget *)&g_psSliders[SLIDER_LOCKED_INDEX]);
    }

    if (psWidget == (tWidget *)&g_psSliders[SLIDER_TEXT_VAL_INDEX])
    {
        //
        // Yes - update the canvas to show the slider value.
        //
        usprintf(pcSliderText, "%3d%%", i32Value);
        SliderTextSet(&g_psSliders[SLIDER_TEXT_VAL_INDEX], pcSliderText);
        WidgetPaint((tWidget *)&g_psSliders[SLIDER_TEXT_VAL_INDEX]);
    }
}

//*****************************************************************************
//
// Handles change notifications for the radio button widgets.
//
//*****************************************************************************

// This is to be replaces with the graph to display data
void OnRadioChange(tWidget *psWidget, uint32_t bSelected)
{
    uint32_t ui32Idx;
    //
    // Find the index of this radio button in the first group.
    //
    for (ui32Idx = 0; ui32Idx < NUM_RADIO1_BUTTONS; ui32Idx++)
    {
        if (psWidget == (tWidget *)(g_psRadioButtons + ui32Idx))
        {
            break;
        }
    }
    selected_sensor = sensors[ui32Idx];
}

//*****************************************************************************
//
// Plot the axis
//
//*****************************************************************************
/*
void xyPlaneDraw(axis x_axis, axis y_axis, int num_y_labels, bool grid_on)
{
    GrContextForegroundSet(&sContext, ClrDarkBlue); // Set colour to dark blue
    char cstr[10];
    // Draw y axix
    GrLineDrawV(&sContext, x_axis.origin, y_axis.origin, (y_axis.origin - y_axis.length));
    // Numbers on graph
    
    for (int i = 0; i < (y_axis.length / y_axis.interval); i++)
    {
        // Plot each interval of y on y-axis
        GrContextFontSet(&sContext, &g_sFontCm16);
        int y_range = y_axis.max - y_axis.min;
        usprintf(cstr, "%d", i * y_range / (num_y_labels-1) );
        GrStringDrawCentered(&sContext, cstr, -1,
                             x_axis.origin - 20, y_axis.origin - (i * y_axis.interval), 0);
        GrLineDrawH(&sContext, x_axis.origin-4, x_axis.origin + 4,  y_axis.origin - (i * y_axis.interval));
    }

    // Draw x axis
    GrLineDrawH(&sContext, x_axis.origin, (x_axis.origin + x_axis.length), y_axis.origin);
    
    if (grid_on)
    {
        // Plot lines for grid
    }
}
*/
void clearAxis (int backround_colour )
{
    tRectangle sRect;
    sRect.i16XMin = X_AXIS_ORIGIN;
    sRect.i16YMin = Y_AXIS_ORIGIN;
    sRect.i16XMax = X_AXIS_ORIGIN + X_AXIS_LENGTH;
    sRect.i16YMax = Y_AXIS_ORIGIN + Y_AXIS_LENGTH;
    GrContextForegroundSet(&sContext, backround_colour);
    GrRectFill(&sContext, &sRect);
}

void define_sensor_axis( void )
{
    
}
//*****************************************************************************
//
// A simple demonstration of the features of the TivaWare Graphics Library.
//
//*****************************************************************************
static void prvDisplayTask(void *pvParameters)
{
    tRectangle sRect;

    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                            120000000);

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "Electric Vehicle System", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 8, 0);

    //
    // Configure and enable uDMA
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    uDMAControlBaseSet(&psDMAControlTable[0]);
    uDMAEnable();

    //
    // Initialize the touch screen driver and have it route its messages to the
    // widget tree.
    //
    TouchScreenInit(g_ui32SysClock);
    TouchScreenCallbackSet(WidgetPointerMessage);

    //
    // Add the title block and the previous and next buttons to the widget
    // tree.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevious);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitle);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNext);

    //
    // Add the first panel to the widget tree.
    //
    g_ui32Panel = 0;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    CanvasTextSet(&g_sTitle, g_pcPanei32Names[0]);

    //
    // Issue the initial paint request to the widgets.
    //
    WidgetPaint(WIDGET_ROOT);

    //
    // Loop forever handling widget messages.
    //
    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
    EventBits_t DisplayBits;
    SensorMsg xReceivedMessage;
    CalcMsg xRecievedPower;
    selected_sensor = NONE;
    char cstr[10];
    while (1)
    {
        //
        // Process any messages in the widget message queue.
        //
        WidgetMessageQueueProcess();
        /* Wait a maximum of 100ms for either bit 0 or bit 4 to be set within the event group. Clear the bits before exiting. */
        EventBits_t DisplayBits = xEventGroupWaitBits(xSensorEventGroup,   /* The event group being tested. */
                                     LUX_DATA_READY | TEMP_DATA_READY | POWER_DATA_READY | SPEED_DATA_READY, /* The bits within the event group to wait for. */
                                     pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
                                     pdFALSE,       /* Don't wait for both bits, either bit will do. */
                                     xTicksToWait); /* Wait a maximum of 100ms for either bit to be set. */

        // Handle the current state
        switch (selected_sensor) 
        {
            case LUX:
                if ( ( ( DisplayBits & (LUX_DATA_READY) ) == (LUX_DATA_READY) ) )
                {
                    if (xQueueReceive(xLuxSensorQueue,
                              &(xReceivedMessage),
                              (TickType_t)10) == pdPASS)
                    {
                        //UARTprintf("Receiving data: %d\n", xReceivedMessage.SensorReading);
                    }
                    /* Call update_axis function to scale axis for lux sensor - to be added */

                    /* Plot the lux data by calling a plot_data function - to be added */
                    usprintf(cstr, "Lux: %d", xReceivedMessage.SensorReading);
                    clearAxis(ClrWhite);

                    //
                    // Put the application name in the middle of the banner.
                    //
                    GrContextForegroundSet(&sContext, ClrDarkBlue);
                    GrContextFontSet(&sContext, &g_sFontCm20);
                    GrStringDrawCentered(&sContext, cstr, -1,
                                        Y_AXIS_ORIGIN + Y_AXIS_LENGTH/2, X_AXIS_ORIGIN + X_AXIS_LENGTH/2, 0);
                }
                break;

            case TEMP:
                if ( ( ( DisplayBits & (TEMP_DATA_READY) ) == (TEMP_DATA_READY) ) )
                {
                    // Plot the data
                }
                clearAxis(ClrWhite);
                //
                // Put the application name in the middle of the banner.
                //
                GrContextForegroundSet(&sContext, ClrDarkBlue);
                GrContextFontSet(&sContext, &g_sFontCm20);
                GrStringDrawCentered(&sContext, "Temp: ", -1,
                                    Y_AXIS_ORIGIN + Y_AXIS_LENGTH/2, X_AXIS_ORIGIN + X_AXIS_LENGTH/2, 0);
                break;

            case POWER:
                if ( ( ( DisplayBits & (POWER_DATA_READY) ) == (POWER_DATA_READY) ) )
                {
                    // Plot the data
                }

                if (xQueueReceive(xPowerSensorQueue,
                        &(xRecievedPower),
                        (TickType_t)10) == pdPASS)
                {
                    //UARTprintf("Receiving data: %d\n", xReceivedMessage.SensorReading);
                }

                clearAxis(ClrWhite);
                GrContextForegroundSet(&sContext, ClrDarkBlue);
                GrContextFontSet(&sContext, &g_sFontCm20);

                float val = xRecievedPower.ClaclulatedData;
                char cstr[15];
                ftoa("Power: %f", cstr, 9, val);
                GrStringDrawCentered(&sContext, cstr, -1,
                                    Y_AXIS_ORIGIN + Y_AXIS_LENGTH/2, X_AXIS_ORIGIN + X_AXIS_LENGTH/2, 0);
                
            case SPEED:
                if ( ( ( DisplayBits & (SPEED_DATA_READY) ) == (SPEED_DATA_READY) ) )
                {
                    if (xQueueReceive(xSpeedSensorQueue,
                              &(xReceivedMessage),
                              (TickType_t)10) == pdPASS)
                    {
                        // UARTprintf("Receiving data: %d\n", xReceivedMessage.SensorReading);
                    }
                    /* Call update_axis function to scale axis for lux sensor - to be added */

                    /* Plot the lux data by calling a plot_data function - to be added */
                    usprintf(cstr, "Speed: %d", xReceivedMessage.SensorReading);
                    clearAxis(ClrWhite);

                    //
                    // Put the application name in the middle of the banner.
                    //
                    GrContextForegroundSet(&sContext, ClrDarkBlue);
                    GrContextFontSet(&sContext, &g_sFontCm20);
                    GrStringDrawCentered(&sContext, cstr, -1,
                                        Y_AXIS_ORIGIN + Y_AXIS_LENGTH/2, X_AXIS_ORIGIN + X_AXIS_LENGTH/2, 0);
                }
                break;
            
            case NONE:
                // Do nothing
                break;
            default:
                UARTprintf("ERROR\n");
                return -1;
                break;
        }
    }
}