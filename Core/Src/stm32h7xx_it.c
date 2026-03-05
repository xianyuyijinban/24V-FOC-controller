/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "head.h"
#include "uart_upload.h"
#include "adc_sampling.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* DRV8350S handle */
DRV8350S_Handle_t drv8350s;

/* FOC应用层句�?? */
FOC_AppHandle_t g_foc_app;

/* UART DMA buffers */
volatile uint16_t urT_data[8];
volatile uint8_t urR_data[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t adc_data[8] = {0};

#define UART_CMD_LINE_MAX       96U
#define UART_CMD_QUEUE_DEPTH    8U

static char s_uartCmdLine[UART_CMD_LINE_MAX];
static uint16_t s_uartCmdLen = 0U;
static char s_uartCmdQueue[UART_CMD_QUEUE_DEPTH][UART_CMD_LINE_MAX];
static volatile uint8_t s_uartCmdQueueWrite = 0U;
static volatile uint8_t s_uartCmdQueueRead = 0U;
static volatile uint8_t s_uartCmdDropUntilEol = 0U;
static volatile uint32_t s_uartRxRestartFailCount = 0U;

static size_t UART_BoundedStrLen(const char *str, size_t maxLen)
{
    size_t len = 0U;
    if (str == NULL) {
        return 0U;
    }

    while ((len < maxLen) && (str[len] != '\0')) {
        len++;
    }
    return len;
}

static void UART_CommandQueuePush(const char *line)
{
    uint8_t next;
    size_t copyLen;

    if (line == NULL) {
        return;
    }

    next = (uint8_t)((s_uartCmdQueueWrite + 1U) % UART_CMD_QUEUE_DEPTH);
    if (next == s_uartCmdQueueRead) {
        /* 队列满：丢弃最旧命令，优先保留最新输入 */
        s_uartCmdQueueRead = (uint8_t)((s_uartCmdQueueRead + 1U) % UART_CMD_QUEUE_DEPTH);
    }

    copyLen = UART_BoundedStrLen(line, UART_CMD_LINE_MAX - 1U);
    memcpy(s_uartCmdQueue[s_uartCmdQueueWrite], line, copyLen);
    s_uartCmdQueue[s_uartCmdQueueWrite][copyLen] = '\0';
    s_uartCmdQueueWrite = next;
}

static void UART_CommandExecute(const char *cmd)
{
    long int int_arg;
    float f1, f2;

    if (cmd == NULL) {
        return;
    }

    if (sscanf(cmd, "CMD:ENABLE,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            FOC_App_Enable(&g_foc_app);
        } else {
            FOC_App_Disable(&g_foc_app);
        }
        return;
    }

    if (sscanf(cmd, "CMD:MODE,%ld", &int_arg) == 1) {
        if (int_arg >= (long int)FOC_MODE_TORQUE && int_arg <= (long int)FOC_MODE_POSITION) {
            __disable_irq();
            FOC_App_SetControlMode(&g_foc_app, (FOC_ControlMode_t)int_arg);
            __enable_irq();
        }
        return;
    }

    if (sscanf(cmd, "CMD:IREF,%f,%f", &f1, &f2) == 2) {
        FOC_App_SetCurrentRef(&g_foc_app, f1, f2);
        return;
    }

    if (sscanf(cmd, "CMD:SREF,%f", &f1) == 1) {
        FOC_App_SetSpeedRef(&g_foc_app, f1);
        return;
    }

    if (sscanf(cmd, "CMD:PREF,%f", &f1) == 1) {
        FOC_App_SetPositionRef(&g_foc_app, f1);
        return;
    }

    if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            FOC_App_StartIdentify(&g_foc_app);
        } else {
            FOC_App_StopIdentify(&g_foc_app);
        }
        return;
    }

    if (strcmp(cmd, "CMD:CLEAR_FAULT") == 0) {
        uint16_t fs1 = 0U, fs2 = 0U;
        uint8_t drv_fault_active = 1U;
        uint8_t encoder_ok;
        uint8_t vbus_ok;

        (void)DRV8350S_ClearFaults(&drv8350s);

        if ((DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_FAULT_STATUS_1, &fs1) == 0) &&
            (DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_VGS_STATUS_2, &fs2) == 0)) {
            drv_fault_active = (((fs1 & 0x07FFU) != 0U) || ((fs2 & 0x00FFU) != 0U)) ? 1U : 0U;
            drv8350s.runtime.regFaultStatus1 = fs1;
            drv8350s.runtime.regVgsStatus2 = fs2;
            drv8350s.runtime.isFaultActive = drv_fault_active;
        }

        encoder_ok = TLE5012_IsDataValid();
        vbus_ok = (g_foc_app.Vbus >= FOC_UNDERVOLTAGE_THRESH) &&
                  (g_foc_app.Vbus <= FOC_OVERVOLTAGE_THRESH);

        __disable_irq();
        if ((!drv_fault_active) && encoder_ok && vbus_ok) {
            g_foc_app.fault_code = FOC_FAULT_NONE;
            if (g_foc_app.state == FOC_STATE_FAULT) {
                g_foc_app.state = FOC_STATE_READY;
            }
        } else {
            if (drv_fault_active) {
                g_foc_app.fault_code = FOC_FAULT_DRV8350S;
            } else if (!encoder_ok) {
                g_foc_app.fault_code = FOC_FAULT_ENCODER;
            } else if (g_foc_app.Vbus > FOC_OVERVOLTAGE_THRESH) {
                g_foc_app.fault_code = FOC_FAULT_OVERVOLTAGE;
            } else if (g_foc_app.Vbus < FOC_UNDERVOLTAGE_THRESH) {
                g_foc_app.fault_code = FOC_FAULT_UNDERVOLTAGE;
            }
        }
        __enable_irq();
        return;
    }

    if (sscanf(cmd, "CMD:PI_CURRENT,%f,%f", &f1, &f2) == 2) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            __disable_irq();
            FOC_PI_Init(&g_foc_app.foc.pi_d, f1, f2, g_foc_app.foc.pi_d.output_max, g_foc_app.foc.pi_d.output_min);
            FOC_PI_Init(&g_foc_app.foc.pi_q, f1, f2, g_foc_app.foc.pi_q.output_max, g_foc_app.foc.pi_q.output_min);
            __enable_irq();
        }
        return;
    }

    if (sscanf(cmd, "CMD:PI_SPEED,%f,%f", &f1, &f2) == 2) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            __disable_irq();
            FOC_PI_Init(&g_foc_app.pi_speed, f1, f2, g_foc_app.pi_speed.output_max, g_foc_app.pi_speed.output_min);
            __enable_irq();
        }
        return;
    }

    if (sscanf(cmd, "CMD:PI_POS,%f,%f", &f1, &f2) == 2) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            __disable_irq();
            FOC_PI_Init(&g_foc_app.pi_pos, f1, f2, g_foc_app.pi_pos.output_max, g_foc_app.pi_pos.output_min);
            __enable_irq();
        }
        return;
    }
}

void UART_Command_ProcessPending(void)
{
    char cmd[UART_CMD_LINE_MAX];
    size_t copyLen;

    while (s_uartCmdQueueRead != s_uartCmdQueueWrite) {
        __disable_irq();
        copyLen = UART_BoundedStrLen(s_uartCmdQueue[s_uartCmdQueueRead], UART_CMD_LINE_MAX - 1U);
        memcpy(cmd, s_uartCmdQueue[s_uartCmdQueueRead], copyLen);
        cmd[copyLen] = '\0';
        s_uartCmdQueueRead = (uint8_t)((s_uartCmdQueueRead + 1U) % UART_CMD_QUEUE_DEPTH);
        __enable_irq();

        UART_CommandExecute(cmd);
    }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN EV */


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */
    /* 处理ADC采样数据 */
    ADC_Sampling_Process();
  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  * @note  20kHz - 电流环控制频率
  * 
  * 控制架构（全部在TIM1中实现）：
 * - 电流环：20kHz（每周期执行）
 * - 速度环：2kHz（10分频）
 * - 位置环：200Hz（100分频）
 * - 参数识别：20kHz（在电流环周期内执行，保证注入波形精度）
 * - 编码器读取：5kHz（4分频）
 * - DRV8350S状态读取：20kHz（每周期）
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
    /* ===== 电流环 (20kHz) =====
     * 【重要】电流环每周期都执行，是FOC的核心
     * 包含：ADC采样、Clark/Park变换、PI控制、SVPWM生成
     */
    FOC_App_TIM1_IRQHandler(&g_foc_app);
    
    /* ===== TLE5012编码器读取 (5kHz = 20kHz/4) ===== */
    static uint8_t tle5012_div_counter = 0;
    if (++tle5012_div_counter >= 4) 
    {
        tle5012_div_counter = 0;
        TLE5012_StartRead();
    }
    
    /* ===== 速度环 (2kHz = 20kHz/10) =====
     * 计算转速 + 速度PI控制
     */
    static uint8_t speed_loop_div_counter = 0;
    if (++speed_loop_div_counter >= 10) 
    {
        speed_loop_div_counter = 0;
        FOC_App_SpeedLoop(&g_foc_app);
    }
    
    /* ===== 位置环 (200Hz = 20kHz/100) =====
     * 位置环输出速度给定，速度环使用
     */
    static uint8_t position_loop_div_counter = 0;
    if (++position_loop_div_counter >= 100) 
    {
        position_loop_div_counter = 0;
        FOC_App_PositionLoop(&g_foc_app);
    }
    
    /* ===== DRV8350S栅极驱动状态读取 (20kHz) ===== */
    DRV8350S_TIM1_UpdateCallback(&drv8350s);
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles TIM2 global interrupt.
  * @note  【已禁用】速度环/位置环/参数识别已移至TIM1中断分频
  *        如需使用TIM2，请取消下面的注释并重新配置
  */
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    /* TIM2在当前方案未启用，但仍需清除更新标志，避免误使能后中断风暴 */
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;
    }
    
    /* USER CODE END TIM2_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim2);  /* 已禁用 */
    /* USER CODE BEGIN TIM2_IRQn 1 */
    
    /* 调用FOC应用层TIM2中断处理 - 速度环和参数识别 */
    // FOC_App_TIM2_IRQHandler(&g_foc_app);  /* 已禁用 - 使用TIM1分频代替 */
    
    /* USER CODE END TIM2_IRQn 1 */
}

/* SPI DMA Complete Callback */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        DRV8350S_DMA_CompleteCallback(&drv8350s);
        
        uint8_t reg = drv8350s.readReq.registerAddr;
        (void)drv8350s.rxBuf[0];  /* First received word (previous frame response) - for debug */
        (void)drv8350s.rxBuf[1];  /* Second received word (actual data) - for debug */
        
        switch (reg) {
            case DRV8350S_REG_FAULT_STATUS_1:
                /* Handle fault if active */
                if (drv8350s.runtime.isFaultActive) {
                    /* Fault handling */
                }
                break;
            case DRV8350S_REG_VGS_STATUS_2:
                break;
            default:
                break;
        }
    }
    
    if (hspi == &hspi3) {
        TLE5012_ProcessData(tle5012_rx_buf);
    }
    
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        DRV8350S_DMA_ErrorCallback(&drv8350s);
        drv8350s.runtime.errorCount++;
    }
    
    if (hspi == &hspi3) {
        // TLE5012 CS接地，无�???操作
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        DrvUart_TxCpltCallback(huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;

    if (huart != &huart1) {
        return;
    }

    for (i = 0U; i < Size; i++) {
        uint8_t ch = urR_data[i];

        if (s_uartCmdDropUntilEol) {
            if (ch == '\n') {
                s_uartCmdDropUntilEol = 0U;
                s_uartCmdLen = 0U;
            }
            continue;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            if (s_uartCmdLen > 0U) {
                s_uartCmdLine[s_uartCmdLen] = '\0';
                UART_CommandQueuePush(s_uartCmdLine);
                s_uartCmdLen = 0U;
            }
            continue;
        }

        if (s_uartCmdLen < (UART_CMD_LINE_MAX - 1U)) {
            s_uartCmdLine[s_uartCmdLen++] = (char)ch;
        } else {
            /* 超长命令：丢弃本行，等待下一次换行重新同步 */
            s_uartCmdLen = 0U;
            s_uartCmdDropUntilEol = 1U;
        }
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)urR_data, sizeof(urR_data)) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    } else {
        /* 记录重启失败并做一次快速重试，避免接收链路静默失效 */
        s_uartRxRestartFailCount++;
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)urR_data, sizeof(urR_data)) == HAL_OK) {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        } else {
            s_uartRxRestartFailCount++;
        }
    }
    if (huart1.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
/* USER CODE END 1 */
