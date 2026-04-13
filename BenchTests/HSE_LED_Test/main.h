#ifndef HSE_LED_TEST_MAIN_H
#define HSE_LED_TEST_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define LED_OK_GPIO_Port GPIOB
#define LED_OK_Pin       GPIO_PIN_8
#define LED_ERR_GPIO_Port GPIOB
#define LED_ERR_Pin       GPIO_PIN_9

HAL_StatusTypeDef SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
