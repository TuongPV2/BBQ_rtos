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

#include "cap1106.h"
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

/* Task priorities. */
//#define Tester_PRIORITY (configMAX_PRIORITIES - 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void vActivateLED(void *pvParameter);
void vDeactivateLED(void *pvParameter);
/*******************************************************************************
 * Variables
 ******************************************************************************/
TimerHandle_t xTimerHdl1,xTimerHdl2;
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

    /* Set to allow entering vlps mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeVlp);
	
	xTimerHdl1 = xTimerCreate("vTest Timer", (200/portTICK_RATE_MS), pdTRUE, (void *)1, vActivateLED);
	if( xTimerHdl1 != NULL)
	{
		
		xTimerStart( xTimerHdl1, 0 );
	}
	
	xTimerHdl2 = xTimerCreate("vTest Timer", (1005/portTICK_RATE_MS), pdTRUE, (void *)2, vDeactivateLED);
    if (xTimerHdl2 != NULL)
    {
        xTimerStart( xTimerHdl2, 0 );
    }
	
    vTaskStartScheduler();

    while (1)
    {

    }
}

void vActivateLED(void *pvParameter)
{
    PROBE2_LED_OUT(0);
}

void vDeactivateLED(void *pvParameter)
{
    PROBE2_LED_OUT(1);
}