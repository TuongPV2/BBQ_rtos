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
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initialize all pins used in this example
 *
 */
void BOARD_InitPins(void)
{
    ENABLE_GPIO_CLOCK();
    /* Initialize LPUART0 pins below */
    /* Affects PORTA_PCR1 register */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);
    /* Affects PORTA_PCR2 register */
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

    // Init debug port
#if defined(DEGBUG_PRINT) && DEGBUG_PRINT
    PORT_SetPinMux(PORTE, 30U, kPORT_MuxAlt5);
#endif
    // Probe led
    PORT_SetPinMux(PROBE1_LED_PIN_MUX);
    PORT_SetPinMux(PROBE2_LED_PIN_MUX);
    PORT_SetPinMux(PROBE3_LED_PIN_MUX);
    PORT_SetPinMux(PROBE4_LED_PIN_MUX);

    PROBE1_LED_PIN_INIT(0);
    PROBE2_LED_PIN_INIT(0);
    PROBE3_LED_PIN_INIT(0);
    PROBE4_LED_PIN_INIT(0);

    // Probe
    PORT_SetPinMux(PROBE1_PIN_MUX);
    PORT_SetPinMux(PROBE2_PIN_MUX);
    PORT_SetPinMux(PROBE3_PIN_MUX);
    PORT_SetPinMux(PROBE4_PIN_MUX);
    // 7 seg data
    PORT_SetPinMux(A_PIN_MUX);
    PORT_SetPinMux(B_PIN_MUX);
    PORT_SetPinMux(C_PIN_MUX);
    PORT_SetPinMux(D_PIN_MUX);
    PORT_SetPinMux(E_PIN_MUX);
    PORT_SetPinMux(F_PIN_MUX);
    PORT_SetPinMux(G_PIN_MUX);

    A_PIN_INIT(0);
    B_PIN_INIT(0);
    C_PIN_INIT(0);
    D_PIN_INIT(0);
    E_PIN_INIT(0);
    F_PIN_INIT(0);
    G_PIN_INIT(0);

    // 7 seg ctrl
    PORT_SetPinMux(CTRL1_PIN_MUX);
    PORT_SetPinMux(CTRL2_PIN_MUX);
    PORT_SetPinMux(CTRL3_PIN_MUX);
    CTRL1_PIN_INIT(0);
    CTRL2_PIN_INIT(0);
    CTRL3_PIN_INIT(0);

    // led c f target current
    PORT_SetPinMux(LED_DEG_C_PIN_MUX);
    PORT_SetPinMux(LED_DEG_F_PIN_MUX);
    PORT_SetPinMux(LED_TARGET_PIN_MUX);
    LED_DEG_F_PIN_INIT(0);
    LED_DEG_C_PIN_INIT(0);
    LED_TARGET_PIN_INIT(0);

#if defined(DEGBUG_PRINT) && !(DEGBUG_PRINT)
    PORT_SetPinMux(LEDCURRENT_PIN_MUX);
    LED_CURRENT_PIN_INIT(0);
#endif

    PORT_SetPinMux(SWITCH_PIN_MUX);
    SWITCH_PIN_INIT();

    PORT_SetPinMux(LED_BAT_LOW_PIN_MUX);
    PORT_SetPinMux(LED_BAT_FULL_PIN_MUX);
    LED_BAT_LOW_PIN_INIT(0);
    LED_BAT_FULL_PIN_INIT(0);

    // esp wk pin
    PORT_SetPinMux(ESP_FORCE_WK_PIN_MUX);
    ESP_FORCE_WK_PIN_INIT();
    // esp sleep status
    PORT_SetPinMux(ESP_SLEEP_STT_PIN_MUX);
    ESP_SLEEP_STT_INIT();

    // buzzer init
    PORT_SetPinMux(BUZZER_PIN_MUX);
    BUZZER_PIN_INIT(0);

    // test
    //     PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);
    // GPIO_PinInit(GPIOD, 7U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (1)});

//    // Bat
   // PORT_SetPinMux(BAT_PIN_MUX);

//    // switch
//    PORT_SetPinMux(SWITCH_PIN_MUX);
//    // I2C
//    //PORT_SetPinMux(I2C_SCL_PIN_MUX);
//    //PORT_SetPinMux(I2C_SDA_PIN_MUX);
//
    port_pin_config_t pinConfig = {0};
    // init esp depned
    PORT_SetPinMux(ESP_PIN_INDICATE_MUX);
    ESP_PIN_INDICATE_INIT();

    //PORT_SetPinMux(ESP_PIN_REQUEST_MUX);
    pinConfig.pullSelect = kPORT_PullUp;
    pinConfig.mux = kPORT_MuxAsGpio;

    PORT_SetPinConfig(PORTA, 13U, &pinConfig);
    ESP_PIN_REQUEST_INIT();
    ESP_PIN_REQUEST_INT_EN();

    pinConfig.pullSelect = kPORT_PullUp;
    pinConfig.mux = kPORT_MuxAlt2;

    PORT_SetPinConfig(PORTC, 10, &pinConfig);
    PORT_SetPinConfig(PORTC, 11, &pinConfig);
    EnableIRQ(PORTA_IRQn);
    // Set pin interrupt for cap1106 alert pin
    PORT_SetPinMux(I2C_INT_PIN_MUX);
    I2C_EN_INT_PIN();
    I2C_PIN_INIT();
}


