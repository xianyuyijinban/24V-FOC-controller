/* Minimal HAL configuration for standalone HSE LED bench test */
#ifndef STM32H7XX_HAL_CONF_H
#define STM32H7XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#if !defined(HSE_VALUE)
#define HSE_VALUE (25000000UL)
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT (100UL)
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT (5000UL)
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE (64000000UL)
#endif

#if !defined(CSI_VALUE)
#define CSI_VALUE (4000000UL)
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE (32000UL)
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE (12288000UL)
#endif

#define VDD_VALUE            (3300UL)
#define TICK_INT_PRIORITY    (7UL)
#define USE_RTOS             0U

#define USE_HAL_CORTEX_REGISTER_CALLBACKS 0U
#define USE_HAL_FLASH_REGISTER_CALLBACKS  0U
#define USE_HAL_GPIO_REGISTER_CALLBACKS   0U
#define USE_HAL_PWR_REGISTER_CALLBACKS    0U
#define USE_HAL_RCC_REGISTER_CALLBACKS    0U

#define assert_param(expr) ((void)0U)

#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_pwr_ex.h"

#ifdef __cplusplus
}
#endif

#endif
