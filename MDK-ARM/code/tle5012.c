/**
 * @file    tle5012.c
 * @brief   TLE5012B磁编码器驱动实现
 *
 * 当前硬件为单 DATA 线 SSC：
 * - 命令阶段：MCU 驱动 DATA 发送读命令
 * - 数据阶段：MCU 释放 DATA，由传感器回传 Data + Safety
 */

#include "head.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

TLE5012_Data_t tle5012_sensor = {0};

/* 函数前置声明 */
static uint8_t TLE5012_CalculateCRC8(const uint16_t *words, uint8_t word_count);
static void TLE5012_AssertCS(void);
static void TLE5012_ReleaseCS(void);
static void TLE5012_HandleCommFault(TLE5012_Fault_t fault);
static void TLE5012_ConfigCommandPhasePins(void);
static void TLE5012_ConfigResponsePhasePins(void);
static void TLE5012_TwrDelay(void);
static HAL_StatusTypeDef TLE5012_StartRxPhase(void);

/* 私有变量 */
static uint8_t is_busy = 0U;
static uint16_t tle5012_tx_buf[1];
uint16_t tle5012_rx_buf[2];

/* CRC错误计数和超时保护 */
static uint8_t crc_error_count = 0U;
static uint32_t busy_start_time = 0U;
static TLE5012_FaultCallback_t fault_callback = NULL;

#define CRC_ERROR_THRESHOLD             3U
#define SPI_TIMEOUT_MS                  10U
#define TLE5012_TWR_DELAY_US           5U
#define TLE5012_GPIO_MODER_INPUT(pin)  (0x0U << ((pin) * 2U))
#define TLE5012_GPIO_MODER_AF(pin)     (0x2U << ((pin) * 2U))
#define TLE5012_GPIO_MODER_MASK(pin)   (0x3U << ((pin) * 2U))
#define TLE5012_DATA_MISO_PIN          11U
#define TLE5012_DATA_MOSI_PIN          12U
#define TLE5012_SAFETY_RESET_OK_MASK    0x8000U
#define TLE5012_SAFETY_SYSTEM_OK_MASK   0x4000U
#define TLE5012_SAFETY_INTERFACE_OK_MASK 0x2000U
#define TLE5012_SAFETY_ANGLE_OK_MASK    0x1000U

/* 命令字构建: RW=1, Lock=0, UPD=0, ADDR=0x02, ND=1 */
#define TLE5012_READ_CMD 0x8021U

static void TLE5012_AssertCS(void)
{
    HAL_GPIO_WritePin(TLE5012_CS_PORT, TLE5012_CS_PIN, GPIO_PIN_RESET);
}

static void TLE5012_ReleaseCS(void)
{
    HAL_GPIO_WritePin(TLE5012_CS_PORT, TLE5012_CS_PIN, GPIO_PIN_SET);
}

static void TLE5012_HandleCommFault(TLE5012_Fault_t fault)
{
    TLE5012_ReleaseCS();
    TLE5012_ConfigCommandPhasePins();

    if (!is_busy) {
        return;
    }

    is_busy = 0U;
    tle5012_sensor.status = 0U;
    tle5012_sensor.reset_fault = 0U;
    tle5012_sensor.data_valid = 0U;
    tle5012_sensor.crc_error = 1U;
    tle5012_sensor.update_flag = 1U;

    if (crc_error_count < 0xFFU) {
        crc_error_count++;
    }

    if ((fault_callback != NULL) && (crc_error_count >= CRC_ERROR_THRESHOLD)) {
        fault_callback(fault);
    }
}

static void TLE5012_ConfigCommandPhasePins(void)
{
    uint32_t moder = GPIOC->MODER;

    moder &= ~(TLE5012_GPIO_MODER_MASK(TLE5012_DATA_MISO_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_MOSI_PIN));
    moder |= TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_MISO_PIN) |
             TLE5012_GPIO_MODER_AF(TLE5012_DATA_MOSI_PIN);
    GPIOC->MODER = moder;
    __DSB();
}

static void TLE5012_ConfigResponsePhasePins(void)
{
    uint32_t moder = GPIOC->MODER;

    moder &= ~(TLE5012_GPIO_MODER_MASK(TLE5012_DATA_MISO_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_MOSI_PIN));
    moder |= TLE5012_GPIO_MODER_AF(TLE5012_DATA_MISO_PIN) |
             TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_MOSI_PIN);
    GPIOC->MODER = moder;
    __DSB();
}

static void TLE5012_TwrDelay(void)
{
    volatile uint32_t count;
    uint32_t iterations = ((SystemCoreClock / 1000000U) * TLE5012_TWR_DELAY_US) / 4U;

    if (iterations == 0U) {
        iterations = 1U;
    }

    for (count = 0U; count < iterations; ++count) {
        __NOP();
    }
}

static HAL_StatusTypeDef TLE5012_StartRxPhase(void)
{
    TLE5012_ConfigResponsePhasePins();
    TLE5012_TwrDelay();
    return HAL_SPI_Receive_DMA(&hspi3, (uint8_t *)tle5012_rx_buf, 2U);
}

void TLE5012_Init(void)
{
    memset(tle5012_tx_buf, 0, sizeof(tle5012_tx_buf));
    memset(tle5012_rx_buf, 0, sizeof(tle5012_rx_buf));
    memset(&tle5012_sensor, 0, sizeof(tle5012_sensor));
    crc_error_count = 0U;
    is_busy = 0U;

    TLE5012_ConfigCommandPhasePins();
    TLE5012_ReleaseCS();
}

void TLE5012_RegisterFaultCallback(TLE5012_FaultCallback_t callback)
{
    fault_callback = callback;
}

void TLE5012_StartRead(void)
{
    HAL_StatusTypeDef status;

    if (is_busy) {
        if ((HAL_GetTick() - busy_start_time) > SPI_TIMEOUT_MS) {
            (void)HAL_SPI_DMAStop(&hspi3);
            TLE5012_HandleCommFault(TLE5012_FAULT_TIMEOUT);
        }
        return;
    }

    is_busy = 1U;
    busy_start_time = HAL_GetTick();
    tle5012_tx_buf[0] = TLE5012_READ_CMD;
    memset(tle5012_rx_buf, 0, sizeof(tle5012_rx_buf));

    TLE5012_ConfigCommandPhasePins();
    TLE5012_AssertCS();
    status = HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)tle5012_tx_buf, 1U);
    if (status != HAL_OK) {
        TLE5012_HandleTransferError();
    }
}

void TLE5012_HandleTxComplete(void)
{
    if (!is_busy) {
        TLE5012_ReleaseCS();
        return;
    }

    if (TLE5012_StartRxPhase() != HAL_OK) {
        TLE5012_HandleTransferError();
    }
}

void TLE5012_ProcessData(uint16_t *rx_buf)
{
    uint16_t raw_data;
    uint16_t safety_word;
    uint16_t crc_words[2];
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t safety_ok;

    TLE5012_ReleaseCS();
    TLE5012_ConfigCommandPhasePins();
    is_busy = 0U;

    if (rx_buf == NULL) {
        tle5012_sensor.status = 0U;
        tle5012_sensor.reset_fault = 0U;
        tle5012_sensor.data_valid = 0U;
        tle5012_sensor.crc_error = 1U;
        tle5012_sensor.update_flag = 1U;
        return;
    }

    raw_data = rx_buf[0];
    safety_word = rx_buf[1];
    crc_words[0] = TLE5012_READ_CMD;
    crc_words[1] = raw_data;

    received_crc = (uint8_t)(safety_word & 0x00FFU);
    calculated_crc = TLE5012_CalculateCRC8(crc_words, 2U);
    tle5012_sensor.status = (uint8_t)(safety_word >> 8);
    tle5012_sensor.reset_fault = ((safety_word & TLE5012_SAFETY_RESET_OK_MASK) == 0U) ? 1U : 0U;
    safety_ok = (((safety_word & TLE5012_SAFETY_RESET_OK_MASK) != 0U) &&
                 ((safety_word & TLE5012_SAFETY_SYSTEM_OK_MASK) != 0U) &&
                 ((safety_word & TLE5012_SAFETY_INTERFACE_OK_MASK) != 0U) &&
                 ((safety_word & TLE5012_SAFETY_ANGLE_OK_MASK) != 0U)) ? 1U : 0U;

    tle5012_sensor.raw_angle = raw_data & 0x7FFFU;
    tle5012_sensor.angle = (float)tle5012_sensor.raw_angle * (360.0f / 32768.0f);

    if ((!safety_ok) || (received_crc != calculated_crc)) {
        tle5012_sensor.crc_error = 1U;
        tle5012_sensor.data_valid = 0U;
        if (crc_error_count < 0xFFU) {
            crc_error_count++;
        }
        if ((fault_callback != NULL) && (crc_error_count >= CRC_ERROR_THRESHOLD)) {
            fault_callback(safety_ok ? TLE5012_FAULT_CRC : TLE5012_FAULT_DATA);
        }
    } else {
        tle5012_sensor.crc_error = 0U;
        tle5012_sensor.data_valid = 1U;
        crc_error_count = 0U;
    }

    tle5012_sensor.update_flag = 1U;
}

void TLE5012_HandleTransferError(void)
{
    TLE5012_HandleCommFault(TLE5012_FAULT_DATA);
}

float TLE5012_GetAngle(void)
{
    return tle5012_sensor.angle;
}

uint8_t TLE5012_GetCRCErrorCount(void)
{
    return crc_error_count;
}

void TLE5012_ClearCRCErrorCount(void)
{
    crc_error_count = 0U;
}

uint8_t TLE5012_IsDataValid(void)
{
    return tle5012_sensor.data_valid;
}

static uint8_t TLE5012_CalculateCRC8(const uint16_t *words, uint8_t word_count)
{
    uint8_t crc = 0xFFU;
    uint8_t poly = 0x1DU;
    uint8_t word_index;
    uint8_t byte_index;

    if (words == NULL) {
        return 0U;
    }

    for (word_index = 0U; word_index < word_count; ++word_index) {
        for (byte_index = 0U; byte_index < 2U; ++byte_index) {
            uint8_t data = (byte_index == 0U)
                               ? (uint8_t)(words[word_index] >> 8)
                               : (uint8_t)(words[word_index] & 0x00FFU);
            uint8_t bit;

            crc ^= data;
            for (bit = 0U; bit < 8U; ++bit) {
                if ((crc & 0x80U) != 0U) {
                    crc = (uint8_t)((crc << 1) ^ poly);
                } else {
                    crc <<= 1;
                }
            }
        }
    }

    return (uint8_t)(~crc);
}
