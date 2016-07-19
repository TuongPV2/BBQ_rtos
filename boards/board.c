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

#include <stdint.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_lptmr.h"
#include "fsl_rtc.h"
#include "myUI.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern void KeyInterruptCallback(void);
static uint32_t lptmr_interval = 0;
rtc_datetime_t datetime = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/

// LPTMR irq handler function
void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    //TimerIrqHandler();
}

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq;
    /* SIM_SOPT2[27:26]:
     *  00: Clock Disabled
     *  01: IRC48M
     *  10: OSCERCLK
     *  11: MCGIRCCLK
     */
    CLOCK_SetLpuart1Clock(1);

    uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOARD_LPTMRInit(void)
{
    lptmr_config_t lptmrConfig;

    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    //lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_2;
    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);
    lptmr_interval = USEC_TO_COUNT(5000U, CLOCK_GetFreq(kCLOCK_LpoClk));
    /* Set timer period */
    LPTMR_SetTimerPeriod(LPTMR0, lptmr_interval);

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(LPTMR0_IRQn);

    /* Start counting */
    LPTMR_StartTimer(LPTMR0);
}

void BOARD_RTCInit(void)
{
    rtc_config_t rtc_cfg;

    RTC_GetDefaultConfig(&rtc_cfg);
    RTC_Init(RTC,&rtc_cfg);

    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;

    RTC_StopTimer(RTC);

    datetime.year   = 2016;
    datetime.month  = 1;
    datetime.day    = 1;
    datetime.hour   = 12;
    datetime.minute  = 0;
    datetime.second  = 0;

    RTC_SetDatetime(RTC, &datetime);

//    RTC_EnableInterrupts(RTC, kRTC_AlarmInterruptEnable);

//    datetime.second  = 10;
//    RTC_SetAlarm(RTC, &datetime);

//    EnableIRQ(RTC_IRQn);
    RTC_StartTimer(RTC);
}

void RTC_IRQHandler(void)
{
    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmInterruptEnable);
    }
}

//void StopTickCounter(void)
//{
//    LPTMR_StopTimer(LPTMR0);
//}

//void ResumeTickCounter(void)
//{
//    LPTMR_StartTimer(LPTMR0);
//}

//void EXT_HANDLER(void)
//{
//  // Todo: need check another interrupt source
//    KeyInterruptCallback();
//}
//
//void PORTA_IRQHandler(void)
//{
//    ESPRequestInterrupt();
//}
