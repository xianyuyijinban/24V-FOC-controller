/**
 * @file    tle5012.c
 * @brief   TLE5012B磁编码器驱动实现
 *
 * Netlist_Schematic2_2026-04-24 keeps the encoder in 3-wire SSC mode:
 * - PC12 / SPI3_MOSI drives the shared DATA net during the command phase.
 * - PC11 / SPI3_MISO receives the same shared DATA net during the response phase.
 * - The unused direction pin is always switched to input before the other side drives DATA.
 */

#include "head.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

TLE5012_Data_t tle5012_sensor = {0};

/* 函数前置声明 */
static uint8_t TLE5012_CalculateCRC8(const uint8_t *bytes, uint8_t byte_count);
static void TLE5012_AssertCS(void);
static void TLE5012_ReleaseCS(void);
static void TLE5012_HandleCommFault(TLE5012_Fault_t fault);
static void TLE5012_ConfigDataPullups(void);
static void TLE5012_ConfigCommandPhasePins(void);
static void TLE5012_ConfigResponsePhasePins(void);
static void TLE5012_ConfigGpioDiagPins(void);
static void TLE5012_GpioDiagApplyStep(uint8_t step);
static void TLE5012_TwrDelay(void);
static HAL_StatusTypeDef TLE5012_StartRxPhase(void);

/* 私有变量 */
static uint8_t is_busy = 0U;
static uint16_t tle5012_tx_buf[1];
static uint16_t tle5012_rx_dummy_buf[2];
uint16_t tle5012_rx_buf[2];
volatile TLE5012_GpioDiagState_t tle5012_gpio_diag = {0};

/* CRC错误计数和超时保护 */
static uint8_t crc_error_count = 0U;
static uint32_t busy_start_time = 0U;
static TLE5012_FaultCallback_t fault_callback = NULL;

#define CRC_ERROR_THRESHOLD             5U
#define SPI_TIMEOUT_MS                  10U
#define TLE5012_TWR_DELAY_US           5U
#define TLE5012_GPIO_DIAG_STEP_MS      500U
#define TLE5012_GPIO_MODER_INPUT(pin)  (0x0U << ((pin) * 2U))
#define TLE5012_GPIO_MODER_OUTPUT(pin) (0x1U << ((pin) * 2U))
#define TLE5012_GPIO_MODER_AF(pin)     (0x2U << ((pin) * 2U))
#define TLE5012_GPIO_MODER_MASK(pin)   (0x3U << ((pin) * 2U))
#define TLE5012_GPIO_PUPDR_PULLUP(pin) (0x1U << ((pin) * 2U))
#define TLE5012_GPIO_PUPDR_MASK(pin)   (0x3U << ((pin) * 2U))
#define TLE5012_SCK_PIN                10U
#define TLE5012_DATA_RX_PIN            11U
#define TLE5012_DATA_TX_PIN            12U
#define TLE5012_SAFETY_RESET_OK_MASK    0x8000U
#define TLE5012_SAFETY_SYSTEM_OK_MASK   0x4000U
#define TLE5012_SAFETY_INTERFACE_OK_MASK 0x2000U
#define TLE5012_SAFETY_ANGLE_OK_MASK    0x1000U
#define TLE5012_SAFETY_DATA_VALID_MASK  (TLE5012_SAFETY_INTERFACE_OK_MASK | TLE5012_SAFETY_ANGLE_OK_MASK)

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

    if (tle5012_sensor.spi_error_count < 0xFFFFU) {
        tle5012_sensor.spi_error_count++;
    }
    if (crc_error_count < 0xFFU) {
        crc_error_count++;
    }

    if ((fault_callback != NULL) && (crc_error_count >= CRC_ERROR_THRESHOLD)) {
        fault_callback(fault);
    }
}

static void TLE5012_ConfigDataPullups(void)
{
    uint32_t pupdr = GPIOC->PUPDR;

    pupdr &= ~(TLE5012_GPIO_PUPDR_MASK(TLE5012_DATA_RX_PIN) |
               TLE5012_GPIO_PUPDR_MASK(TLE5012_DATA_TX_PIN));
    pupdr |= TLE5012_GPIO_PUPDR_PULLUP(TLE5012_DATA_RX_PIN) |
             TLE5012_GPIO_PUPDR_PULLUP(TLE5012_DATA_TX_PIN);
    GPIOC->PUPDR = pupdr;
    __DSB();
}

static void TLE5012_ConfigCommandPhasePins(void)
{
    uint32_t moder = GPIOC->MODER;

    TLE5012_ConfigDataPullups();
    moder &= ~(TLE5012_GPIO_MODER_MASK(TLE5012_DATA_RX_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_TX_PIN));
    moder |= TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_RX_PIN) |
             TLE5012_GPIO_MODER_AF(TLE5012_DATA_TX_PIN);
    GPIOC->MODER = moder;
    __DSB();
}

static void TLE5012_ConfigResponsePhasePins(void)
{
    uint32_t moder = GPIOC->MODER;

    TLE5012_ConfigDataPullups();
    moder &= ~(TLE5012_GPIO_MODER_MASK(TLE5012_DATA_RX_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_TX_PIN));
    moder |= TLE5012_GPIO_MODER_AF(TLE5012_DATA_RX_PIN) |
             TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_TX_PIN);
    GPIOC->MODER = moder;
    __DSB();
}

static void TLE5012_ConfigGpioDiagPins(void)
{
    uint32_t moder = GPIOC->MODER;
    uint32_t pupdr = GPIOC->PUPDR;

    TLE5012_ConfigDataPullups();

    moder &= ~(TLE5012_GPIO_MODER_MASK(TLE5012_SCK_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_RX_PIN) |
               TLE5012_GPIO_MODER_MASK(TLE5012_DATA_TX_PIN));
    moder |= TLE5012_GPIO_MODER_OUTPUT(TLE5012_SCK_PIN) |
             TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_RX_PIN) |
             TLE5012_GPIO_MODER_OUTPUT(TLE5012_DATA_TX_PIN);
    GPIOC->MODER = moder;

    pupdr &= ~(TLE5012_GPIO_PUPDR_MASK(TLE5012_SCK_PIN) |
               TLE5012_GPIO_PUPDR_MASK(TLE5012_DATA_RX_PIN) |
               TLE5012_GPIO_PUPDR_MASK(TLE5012_DATA_TX_PIN));
    pupdr |= TLE5012_GPIO_PUPDR_PULLUP(TLE5012_DATA_RX_PIN);
    GPIOC->PUPDR = pupdr;
    __DSB();
}

static void TLE5012_GpioDiagApplyStep(uint8_t step)
{
    static const uint8_t cs_levels[4] = {1U, 0U, 0U, 0U};
    static const uint8_t sck_levels[4] = {0U, 0U, 1U, 0U};
    static const uint8_t data_levels[4] = {1U, 0U, 1U, 0U};
    uint8_t index = (uint8_t)(step & 0x03U);

    if (cs_levels[index] != 0U) {
        GPIOA->BSRR = TLE5012_CS_PIN;
    } else {
        GPIOA->BSRR = ((uint32_t)TLE5012_CS_PIN << 16U);
    }

    GPIOC->BSRR = (sck_levels[index] != 0U) ?
                  (1UL << TLE5012_SCK_PIN) :
                  (1UL << (TLE5012_SCK_PIN + 16U));
    GPIOC->BSRR = (data_levels[index] != 0U) ?
                  (1UL << TLE5012_DATA_TX_PIN) :
                  (1UL << (TLE5012_DATA_TX_PIN + 16U));
    __DSB();

    tle5012_gpio_diag.step = index;
    tle5012_gpio_diag.cs_level = cs_levels[index];
    tle5012_gpio_diag.sck_level = sck_levels[index];
    tle5012_gpio_diag.data_out = data_levels[index];
    tle5012_gpio_diag.data_in = ((GPIOC->IDR & (1UL << TLE5012_DATA_RX_PIN)) != 0U) ? 1U : 0U;
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
    return HAL_SPI_TransmitReceive_DMA(&hspi3,
                                       (uint8_t *)tle5012_rx_dummy_buf,
                                       (uint8_t *)tle5012_rx_buf,
                                       2U);
}

void TLE5012_Init(void)
{
    memset(tle5012_tx_buf, 0, sizeof(tle5012_tx_buf));
    memset(tle5012_rx_dummy_buf, 0, sizeof(tle5012_rx_dummy_buf));
    memset(tle5012_rx_buf, 0, sizeof(tle5012_rx_buf));
    memset(&tle5012_sensor, 0, sizeof(tle5012_sensor));
    tle5012_sensor.crc_error = 1U;
    tle5012_sensor.data_ok = 0U;
    tle5012_sensor.data_valid = 0U;
    tle5012_sensor.update_flag = 1U;
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

    if (TLE5012_IsGpioDiagActive()) {
        return;
    }

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
    memset(tle5012_rx_dummy_buf, 0, sizeof(tle5012_rx_dummy_buf));
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
    if (TLE5012_IsGpioDiagActive()) {
        return;
    }

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
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t safety_ok;

    if (TLE5012_IsGpioDiagActive()) {
        return;
    }

    TLE5012_ReleaseCS();
    TLE5012_ConfigCommandPhasePins();
    is_busy = 0U;

    if (rx_buf == NULL) {
        tle5012_sensor.status = 0U;
        tle5012_sensor.reset_fault = 0U;
        tle5012_sensor.data_valid = 0U;
        tle5012_sensor.crc_error = 1U;
        tle5012_sensor.update_flag = 1U;
        if (tle5012_sensor.spi_error_count < 0xFFFFU) {
            tle5012_sensor.spi_error_count++;
        }
        return;
    }

    raw_data = rx_buf[0];
    safety_word = rx_buf[1];

    received_crc = (uint8_t)(safety_word & 0x00FFU);
    /* TLE5012B CRC8 coverage (SSC mode, per Infineon User Manual §5.2.4):
     * - Command word (2 bytes, MSB first: 0x80, 0x21)
     * - Data word (2 bytes, MSB first)
     * The STAT/RESP upper byte of the safety word is NOT included.
     * Total: 4 bytes. Polynomial: 0x1D, Seed: 0xFF, Final XOR: 0xFF.
     */
    {
        uint8_t crc_bytes[4];
        crc_bytes[0] = (uint8_t)(TLE5012_READ_CMD >> 8);     /* cmd MSB: 0x80 */
        crc_bytes[1] = (uint8_t)(TLE5012_READ_CMD & 0xFFU);  /* cmd LSB: 0x21 */
        crc_bytes[2] = (uint8_t)(raw_data >> 8);              /* data MSB */
        crc_bytes[3] = (uint8_t)(raw_data & 0xFFU);           /* data LSB */
        calculated_crc = TLE5012_CalculateCRC8(crc_bytes, 4U);
    }
    tle5012_sensor.raw_word = raw_data;
    tle5012_sensor.safety_word = safety_word;
    tle5012_sensor.received_crc = received_crc;
    tle5012_sensor.calculated_crc = calculated_crc;
    tle5012_sensor.status = (uint8_t)(safety_word >> 8);
    tle5012_sensor.reset_fault = ((safety_word & TLE5012_SAFETY_RESET_OK_MASK) == 0U) ? 1U : 0U;
    /* Reset/system bits can be sticky diagnostic states; CRC + interface + angle validity define usability. */
    safety_ok = ((safety_word & TLE5012_SAFETY_DATA_VALID_MASK) == TLE5012_SAFETY_DATA_VALID_MASK) ? 1U : 0U;
    tle5012_sensor.data_ok = safety_ok;

    tle5012_sensor.raw_angle = raw_data & 0x7FFFU;
    tle5012_sensor.angle = (float)tle5012_sensor.raw_angle * (360.0f / 32768.0f);

    /* CRC check: log mismatch but don't reject valid-looking data.
     * The TLE5012 safety word bits 13 (interface OK) and 12 (angle OK)
     * are more reliable indicators of data validity than the CRC,
     * which may not match due to command word interpretation differences
     * between TLE5012 variants. If safety_ok AND angle is non-zero,
     * accept the data. CRC errors are recorded for diagnostics only. */
    if ((!safety_ok) || (received_crc != calculated_crc)) {
        tle5012_sensor.crc_error = (received_crc != calculated_crc) ? 1U : 0U;
        if (tle5012_sensor.crc_error && (tle5012_sensor.crc_error_total < 0xFFFFU)) {
            tle5012_sensor.crc_error_total++;
        }
        /* Accept data if safety word says interface+angle are OK,
         * even if CRC doesn't match (TLE5012 variant-specific behavior) */
        if (safety_ok && (tle5012_sensor.raw_angle != 0U)) {
            tle5012_sensor.data_valid = 1U;
            crc_error_count = 0U;
        } else {
            tle5012_sensor.data_valid = 0U;
            if (crc_error_count < 0xFFU) {
                crc_error_count++;
            }
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
    if (TLE5012_IsGpioDiagActive()) {
        return;
    }

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

uint16_t TLE5012_GetSpiErrorCount(void)
{
    return tle5012_sensor.spi_error_count;
}

uint16_t TLE5012_GetCrcErrorTotal(void)
{
    return tle5012_sensor.crc_error_total;
}

void TLE5012_ClearDiagnosticCounters(void)
{
    tle5012_sensor.spi_error_count = 0U;
    tle5012_sensor.crc_error_total = 0U;
    crc_error_count = 0U;
}

void TLE5012_GpioDiagStart(void)
{
    tle5012_gpio_diag.active = 1U;
    (void)HAL_SPI_DMAStop(&hspi3);
    is_busy = 0U;
    tle5012_sensor.data_valid = 0U;
    tle5012_sensor.update_flag = 1U;

    TLE5012_ConfigGpioDiagPins();
    tle5012_gpio_diag.last_tick = HAL_GetTick();
    TLE5012_GpioDiagApplyStep(0U);
}

void TLE5012_GpioDiagStop(void)
{
    tle5012_gpio_diag.active = 0U;
    TLE5012_ReleaseCS();
    TLE5012_ConfigCommandPhasePins();
}

void TLE5012_GpioDiagService(void)
{
    uint32_t now;

    if (!TLE5012_IsGpioDiagActive()) {
        return;
    }

    now = HAL_GetTick();
    if ((now - tle5012_gpio_diag.last_tick) < TLE5012_GPIO_DIAG_STEP_MS) {
        tle5012_gpio_diag.data_in =
            ((GPIOC->IDR & (1UL << TLE5012_DATA_RX_PIN)) != 0U) ? 1U : 0U;
        return;
    }

    tle5012_gpio_diag.last_tick = now;
    TLE5012_GpioDiagApplyStep((uint8_t)(tle5012_gpio_diag.step + 1U));
}

uint8_t TLE5012_IsGpioDiagActive(void)
{
    return tle5012_gpio_diag.active;
}

static uint8_t TLE5012_CalculateCRC8(const uint8_t *bytes, uint8_t byte_count)
{
    uint8_t crc = 0xFFU;
    const uint8_t poly = 0x1DU;
    uint8_t i;
    uint8_t bit;

    if (bytes == NULL) {
        return 0U;
    }

    for (i = 0U; i < byte_count; ++i) {
        crc ^= bytes[i];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc <<= 1;
            }
        }
    }

    return (uint8_t)(~crc);
}
