#ifndef UART_SMOKE_TEST_MAIN_H
#define UART_SMOKE_TEST_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define STATUS_OK_GPIO_Port GPIOB
#define STATUS_OK_Pin       GPIO_PIN_8
#define STATUS_ERR_GPIO_Port GPIOB
#define STATUS_ERR_Pin       GPIO_PIN_9

#define UART_SMOKE_TX_GPIO_Port GPIOB
#define UART_SMOKE_TX_Pin       GPIO_PIN_14
#define UART_SMOKE_RX_GPIO_Port GPIOB
#define UART_SMOKE_RX_Pin       GPIO_PIN_15

HAL_StatusTypeDef SystemClock_Config(void);
HAL_StatusTypeDef UART_Smoke_InitStatusGpio(void);
HAL_StatusTypeDef UART_Smoke_InitUsart1(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
