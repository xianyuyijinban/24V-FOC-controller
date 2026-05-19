#include "main.h"

static UART_HandleTypeDef huart1;
static volatile uint32_t s_loopbackPassCount = 0U;
static volatile uint32_t s_loopbackFailCount = 0U;
static volatile uint8_t s_loopbackLastTx = 0U;
static volatile uint8_t s_loopbackLastRx = 0U;
static volatile uint32_t s_loopbackLastIsrBeforeTx = 0U;
static volatile uint32_t s_loopbackLastIsrAfterTx = 0U;
static volatile uint32_t s_loopbackLastIsrAfterRx = 0U;
static volatile HAL_StatusTypeDef s_loopbackLastReceiveStatus = HAL_OK;

static void UART_Loopback_BusyDelay(volatile uint32_t cycles);
static void UART_Loopback_FailureLoop(void);
static HAL_StatusTypeDef UART_Loopback_RunOnce(void);

int main(void)
{
    HAL_Init();

    if (UART_Smoke_InitStatusGpio() != HAL_OK) {
        UART_Loopback_FailureLoop();
    }

    if (SystemClock_Config() != HAL_OK) {
        UART_Loopback_FailureLoop();
    }

    if (UART_Smoke_InitUsart1() != HAL_OK) {
        UART_Loopback_FailureLoop();
    }

    while (1) {
        if (UART_Loopback_RunOnce() != HAL_OK) {
            s_loopbackFailCount++;
            UART_Loopback_FailureLoop();
        }

        s_loopbackPassCount++;
        HAL_GPIO_TogglePin(STATUS_OK_GPIO_Port, STATUS_OK_Pin);
        HAL_GPIO_WritePin(STATUS_ERR_GPIO_Port, STATUS_ERR_Pin, GPIO_PIN_RESET);
        HAL_Delay(100U);
    }
}

HAL_StatusTypeDef SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 5;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return HAL_ERROR;
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1
                                | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef UART_Smoke_InitStatusGpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, STATUS_OK_Pin | STATUS_ERR_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = STATUS_OK_Pin | STATUS_ERR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    return HAL_OK;
}

HAL_StatusTypeDef UART_Smoke_InitUsart1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 230400;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
    huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef UART_Loopback_RunOnce(void)
{
    uint8_t tx = 0x55U;
    uint8_t rx = 0x00U;

    s_loopbackLastTx = tx;
    s_loopbackLastRx = 0U;
    s_loopbackLastIsrBeforeTx = huart1.Instance->ISR;

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    if (HAL_UART_Transmit(&huart1, &tx, 1U, 100U) != HAL_OK) {
        s_loopbackLastIsrAfterTx = huart1.Instance->ISR;
        return HAL_ERROR;
    }

    s_loopbackLastIsrAfterTx = huart1.Instance->ISR;
    s_loopbackLastReceiveStatus = HAL_UART_Receive(&huart1, &rx, 1U, 20U);
    s_loopbackLastRx = rx;
    s_loopbackLastIsrAfterRx = huart1.Instance->ISR;

    if (s_loopbackLastReceiveStatus != HAL_OK) {
        return HAL_ERROR;
    }

    return (rx == tx) ? HAL_OK : HAL_ERROR;
}

static void UART_Loopback_BusyDelay(volatile uint32_t cycles)
{
    while (cycles-- > 0U) {
        __NOP();
    }
}

static void UART_Loopback_FailureLoop(void)
{
    HAL_GPIO_WritePin(STATUS_OK_GPIO_Port, STATUS_OK_Pin, GPIO_PIN_RESET);

    while (1) {
        HAL_GPIO_WritePin(STATUS_ERR_GPIO_Port, STATUS_ERR_Pin, GPIO_PIN_SET);
        UART_Loopback_BusyDelay(400000U);
        HAL_GPIO_WritePin(STATUS_ERR_GPIO_Port, STATUS_ERR_Pin, GPIO_PIN_RESET);
        UART_Loopback_BusyDelay(400000U);
    }
}

void HAL_MspInit(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    UART_Loopback_FailureLoop();
}

void NMI_Handler(void)
{
    UART_Loopback_FailureLoop();
}

void HardFault_Handler(void)
{
    UART_Loopback_FailureLoop();
}

void MemManage_Handler(void)
{
    UART_Loopback_FailureLoop();
}

void BusFault_Handler(void)
{
    UART_Loopback_FailureLoop();
}

void UsageFault_Handler(void)
{
    UART_Loopback_FailureLoop();
}
