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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEGBUG_PRINT    1

#if defined(DEGBUG_PRINT) && DEGBUG_PRINT
#define DBG(fmt,agrs...) do{PRINTF("%-30s:%04d:%-20s(): DBG " fmt "\n", __FILE__, __LINE__, __FUNCTION__, ##agrs);}while(0)
#else
#define DBG(fmt,agrs...)
#endif
/* The board name */
#define BOARD_NAME "FRDM-KL27Z"

/* The LPUART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_LPUART
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) LPUART1
#define BOARD_DEBUG_UART_CLKSRC kCLOCK_McgIrc48MClk
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetPeriphClkFreq()
#define BOARD_UART_IRQ LPUART1_IRQn
#define BOARD_UART_IRQ_HANDLER LPUART1_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

#define BOARD_ACCEL_I2C_BASEADDR I2C1

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn /*!< Tickless timer IRQ number. */

#define BOARD_ADC_BASEADDR      ADC0
#define BOARD_ADC_CHANNEL_GROUP 0U
#define BOARD_I2C_BASEADDR      I2C1
#define BOARD_LPTMR_BASEADDR    LPTMR0

#define ESP_PIN_WAKEUP_MUX
#define ESP_PIN_WAKEUP_INIT

#define ESP_PIN_REQUEST_MUX             PORTA, 13U, kPORT_MuxAsGpio
#define ESP_PIN_REQUEST_INIT()          GPIO_PinInit(GPIOA, 13U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0})
#define ESP_PIN_REQUEST_INT_EN()        PORT_SetPinInterruptConfig(PORTA, 13U, kPORT_InterruptFallingEdge);
#define ESP_REQUEST_STATUS()            PORT_GetPinsInterruptFlags(PORTA) & (1U << 13)
#define ESP_REQUEST_STATUS_CLEAR()      PORT_ClearPinsInterruptFlags(PORTA, 1U << 13);

#define ESP_PIN_INDICATE(x)             GPIO_WritePinOutput(GPIOA, 12U, x)
#define ESP_PIN_INDICATE_MUX            PORTA, 12U, kPORT_MuxAsGpio
#define ESP_PIN_INDICATE_INIT()         GPIO_PinInit(GPIOA, 12U, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0})

#define ESP_FORCE_WK(x)                 GPIO_WritePinOutput(GPIOE, 1U, x)
#define ESP_FORCE_WK_PIN_MUX            PORTE, 1U, kPORT_MuxAsGpio
#define ESP_FORCE_WK_PIN_INIT()         GPIO_PinInit(GPIOE, 1U, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1})

#define ESP_SLEEP_STT_PIN_MUX           PORTA, 5U, kPORT_MuxAsGpio
#define ESP_SLEEP_STT_INIT()            GPIO_PinInit(GPIOA, 5U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0})
#define ESP_SLEEP_STT()                 GPIO_ReadPinInput(GPIOA, 5U)

#define LED_BAT_LOW_PIN_MUX             PORTB, 16U, kPORT_MuxAsGpio
#define LED_BAT_FULL_PIN_MUX            PORTB, 17U, kPORT_MuxAsGpio
#define LED_BAT_LOW_PIN_INIT(output)    GPIO_PinInit(GPIOB, 16U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_BAT_FULL_PIN_INIT(output)   GPIO_PinInit(GPIOB, 17U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_BAT(x)                      GPIOB->PDOR = (GPIOB->PDOR & (~(3 << 16))) | (uint32_t)(x << 16)
#define LED_LOW                         1U
#define LED_FULL                        2U

#define LED_DEG_C_PIN_MUX               PORTB, 18U, kPORT_MuxAsGpio
#define LED_DEG_F_PIN_MUX               PORTB, 19U, kPORT_MuxAsGpio
#define LED_DEG_C_PIN_INIT(output)      GPIO_PinInit(GPIOB, 18U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_DEG_F_PIN_INIT(output)      GPIO_PinInit(GPIOB, 19U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_DEG_FAHR(x)                 GPIOB->PDOR = (GPIOB->PDOR & (~(3 << 18))) | (uint32_t)(x << 18)
#define LED_DEG                         1U
#define LED_FAHR                        2U
#define LED_OFF                         0U

#define LED_TARGET_PIN_MUX              PORTE, 29U, kPORT_MuxAsGpio
#define LEDCURRENT_PIN_MUX              PORTE, 30U, kPORT_MuxAsGpio
#define LED_TARGET_PIN_INIT(output)     GPIO_PinInit(GPIOE, 29U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_CURRENT_PIN_INIT(output)    GPIO_PinInit(GPIOE, 30U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define LED_CURRENT_TARGET(x)           GPIOE->PDOR = (GPIOE->PDOR & (~(3 << 29))) | (uint32_t)(x << 29)
#define LED_CURRENT_TARGET_ON           3U
#define LED_CURRENT                     2U
#define LED_TARGET                      1U

#define BUZZER_PIN_MUX                  PORTD, 6U, kPORT_MuxAsGpio
#define BUZZER_PIN_INIT(output)         GPIO_PinInit(GPIOD, 6U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define BUZZER_ON()                     GPIOD->PSOR |= 1 << 6;
#define BUZZER_OFF()                    GPIOD->PCOR |= 1 << 6;

#define I2C_SCL_PIN_MUX                 PORTC, 10U, kPORT_MuxAlt2
#define I2C_SDA_PIN_MUX                 PORTC, 11U, kPORT_MuxAlt2

#define I2C_INT_PIN_MUX                 PORTE, 31U, kPORT_MuxAsGpio
#define I2C_PIN_INIT()                  GPIO_PinInit(GPIOE, 31U, &(gpio_pin_config_t){kGPIO_DigitalInput, 0})
#define I2C_EN_INT_PIN()                PORT_SetPinInterruptConfig(PORTE, 31U, kPORT_InterruptFallingEdge);\
                                        EnableIRQ(PORTB_PORTC_PORTD_PORTE_IRQn);
#define I2C_INT_STATUS()                PORT_GetPinsInterruptFlags(PORTE) & (1U << 31)
#define I2C_ALERT_CLEAR_FLAG()          PORT_ClearPinsInterruptFlags(PORTE, 1U << 31);

#define EXT_HANDLER                     PORTB_PORTC_PORTD_PORTE_IRQHandler


#define PROBE1_PIN_MUX                  PORTB, 0U, kPORT_PinDisabledOrAnalog
#define PROBE2_PIN_MUX                  PORTB, 1U, kPORT_PinDisabledOrAnalog
#define PROBE3_PIN_MUX                  PORTB, 2U, kPORT_PinDisabledOrAnalog
#define PROBE4_PIN_MUX                  PORTB, 3U, kPORT_PinDisabledOrAnalog

#define ADC_CHANNEL_PROBE1              8U
#define ADC_CHANNEL_PROBE2              9U
#define ADC_CHANNEL_PROBE3              12U
#define ADC_CHANNEL_PROBE4              13U

#define BAT_PIN_MUX                     PORTE, 23U, kPORT_PinDisabledOrAnalog

#define ADC_CHANNEL_BAT                 7U

#define SWITCH_PIN_MUX                  PORTD, 0U, kPORT_MuxAsGpio
#define SWITCH_PIN_INIT()               GPIO_PinInit(GPIOD, 0, &(gpio_pin_config_t){kGPIO_DigitalInput, 0})
#define SWITCH_STT()                    GPIO_ReadPinInput(GPIOD, 0U)

#define PROBE1_LED_PIN_MUX              PORTE, 16U, kPORT_MuxAsGpio
#define PROBE2_LED_PIN_MUX              PORTE, 20U, kPORT_MuxAsGpio
#define PROBE3_LED_PIN_MUX              PORTE, 21U, kPORT_MuxAsGpio
#define PROBE4_LED_PIN_MUX              PORTE, 22U, kPORT_MuxAsGpio

#define PROBE1_LED_OUT(x)               GPIO_WritePinOutput(GPIOE, 16U, x)
#define PROBE2_LED_OUT(x)               GPIO_WritePinOutput(GPIOE, 20U, x)
#define PROBE3_LED_OUT(x)               GPIO_WritePinOutput(GPIOE, 21U, x)
#define PROBE4_LED_OUT(x)               GPIO_WritePinOutput(GPIOE, 22U, x)

#define PROBE1_LED_PIN_INIT(output)     GPIO_PinInit(GPIOE, 16U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define PROBE2_LED_PIN_INIT(output)     GPIO_PinInit(GPIOE, 20U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define PROBE3_LED_PIN_INIT(output)     GPIO_PinInit(GPIOE, 21U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define PROBE4_LED_PIN_INIT(output)     GPIO_PinInit(GPIOE, 22U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})

#define A_PIN_MUX                       PORTC, 0U, kPORT_MuxAsGpio
#define B_PIN_MUX                       PORTC, 1U, kPORT_MuxAsGpio
#define C_PIN_MUX                       PORTC, 2U, kPORT_MuxAsGpio
#define D_PIN_MUX                       PORTC, 3U, kPORT_MuxAsGpio
#define E_PIN_MUX                       PORTC, 4U, kPORT_MuxAsGpio
#define F_PIN_MUX                       PORTC, 5U, kPORT_MuxAsGpio
#define G_PIN_MUX                       PORTC, 6U, kPORT_MuxAsGpio

#define A_PIN_INIT(output)              GPIO_PinInit(GPIOC, 0U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define B_PIN_INIT(output)              GPIO_PinInit(GPIOC, 1U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define C_PIN_INIT(output)              GPIO_PinInit(GPIOC, 2U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define D_PIN_INIT(output)              GPIO_PinInit(GPIOC, 3U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define E_PIN_INIT(output)              GPIO_PinInit(GPIOC, 4U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define F_PIN_INIT(output)              GPIO_PinInit(GPIOC, 5U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define G_PIN_INIT(output)              GPIO_PinInit(GPIOC, 6U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})

#define A_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 0U, x)
#define B_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 1U, x)
#define C_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 2U, x)
#define D_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 3U, x)
#define E_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 4U, x)
#define F_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 5U, x)
#define G_PIN_OUT(x)                    PIO_WritePinOutput(GPIOC, 6U, x)

#define LED_PUT_DATA(x)                 GPIOC->PDOR = (GPIOC->PDOR & (~0x7F)) | (uint32_t)(x & 0x7F)

#define CTRL1_PIN_MUX                   PORTC, 7U, kPORT_MuxAsGpio
#define CTRL2_PIN_MUX                   PORTC, 8U, kPORT_MuxAsGpio
#define CTRL3_PIN_MUX                   PORTC, 9U, kPORT_MuxAsGpio

#define CTRL1_PIN_INIT(output)          GPIO_PinInit(GPIOC, 7U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define CTRL2_PIN_INIT(output)          GPIO_PinInit(GPIOC, 8U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define CTRL3_PIN_INIT(output)          GPIO_PinInit(GPIOC, 9U, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})

#define CTRL1_PIN_OUT(x)                GPIO_WritePinOutput(GPIOC, 7U, x)
#define CTRL2_PIN_OUT(x)                GPIO_WritePinOutput(GPIOC, 8U, x)
#define CTRL3_PIN_OUT(x)                GPIO_WritePinOutput(GPIOC, 9U, x)

#define CTRL_PUT_DATA(x)                GPIOC->PDOR = (GPIOC->PDOR & (~(0x7 << 7))) | (uint32_t)(x << 7)

#define ENABLE_GPIO_CLOCK()             CLOCK_EnableClock(kCLOCK_PortB);\
                                        CLOCK_EnableClock(kCLOCK_PortC);\
                                        CLOCK_EnableClock(kCLOCK_PortD);\
                                        CLOCK_EnableClock(kCLOCK_PortE);\
                                        CLOCK_EnableClock(kCLOCK_PortA);

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_LPTMRInit(void);
void BOARD_InitDebugConsole(void);
void BOARD_RTCInit(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
