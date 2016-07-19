/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  Standard C Included Files */
#include <string.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_smc.h"
#include "fsl_pmc.h"
#include "fsl_lptmr.h"
#include "fsl_gpio.h"

#include "myLed.h"
#include "myADC.h"
#include "myUI.h"
#include "menu.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_device_registers.h"
#include <stdbool.h>
#include "stdbool.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TASK_DELAY        200
/* Task priorities. */
#define TesterFnc_PRIORITY (configMAX_PRIORITIES - 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void vTesterFnc(void *pvParameters);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin, clock, debug console init */
    BOARD_InitPins();
    BOARD_BootClockRUN();
#if defined(DEGBUG_PRINT) && DEGBUG_PRINT
    BOARD_InitDebugConsole();
#endif
    //BOARD_LPTMRInit();
    //BOARD_RTCInit();

    //ESP_Init();
    //ADCInit();
    //UI_Init();
    //Menu_Init();

    //DBG("KL27 start");

    /* Set to allow entering vlps mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeVlp);
    //UpdateTimerTick(10);

    //CreateTimer(5, 1 , LedProcess, NULL); // Create led handle every 5ms
    //ADC_StartProcess();

    if (xTaskCreate(vTesterFnc, "vTesterFnc", configMINIMAL_STACK_SIZE + 60, NULL, TesterFnc_PRIORITY, NULL) != pdPASS)
    {
        //PRINTF("Failed to create vTesterFnc task");
    }

    vTaskStartScheduler();

    while (1)
    {

    }
}

static void vTesterFnc(void *pvParameters)
{
    //UNUSED_PARAMETER(pvParameter);
    for( ;; )
    {
        PROBE1_LED_OUT(1);
        vTaskDelay(TASK_DELAY);
        PROBE1_LED_OUT(0);
        vTaskDelay(TASK_DELAY);
    }
}
