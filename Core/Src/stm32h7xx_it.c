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
#include <math.h>
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
extern TIM_HandleTypeDef htim1;

#define UART_CMD_LINE_MAX       96U
#define UART_CMD_QUEUE_DEPTH    8U
#define UART_ADC_PHASE_SCAN_MIN_TRIGGER 2U
#define UART_ADC_PHASE_SCAN_MAX_TRIGGER 46U
#define UART_ADC_PHASE_SCAN_TRIGGER_STEP 2U
#define UART_ADC_SECTOR_SCAN_SECTORS 6U
#define UART_ADC_SECTOR_SCAN_MIN_PHASES 3U

static char s_uartCmdLine[UART_CMD_LINE_MAX];
static uint16_t s_uartCmdLen = 0U;
static char s_uartCmdQueue[UART_CMD_QUEUE_DEPTH][UART_CMD_LINE_MAX];
static volatile uint8_t s_uartCmdQueueWrite = 0U;
static volatile uint8_t s_uartCmdQueueRead = 0U;
static volatile uint8_t s_uartCmdDropUntilEol = 0U;
static volatile uint32_t s_uartRxRestartFailCount = 0U;

typedef struct {
    uint32_t sum;
    uint64_t sumSq;
    uint16_t min;
    uint16_t max;
} UART_AdcNoiseAccum_t;

typedef struct {
    UART_AdcNoiseAccum_t accA;
    UART_AdcNoiseAccum_t accB;
    UART_AdcNoiseAccum_t accC;
    uint32_t sumCcrA;
    uint32_t sumCcrB;
    uint32_t sumCcrC;
    uint16_t count;
    uint8_t winA;
    uint8_t winB;
    uint8_t winC;
} UART_AdcSectorBucket_t;

static uint8_t s_adcNoiseActive = 0U;
static uint16_t s_adcNoiseTargetSamples = 0U;
static uint16_t s_adcNoiseCapturedSamples = 0U;
static uint32_t s_adcNoiseStartTick = 0U;
static uint32_t s_adcNoiseTimeoutMs = 0U;
static uint32_t s_adcNoiseLastFrameSequence = 0U;
static UART_AdcNoiseAccum_t s_adcNoiseAccum[ADC_CHANNEL_COUNT];

static void UART_ReevaluateVoltageFaultAfterThresholdUpdate(void)
{
    FOC_FaultCode_t voltageFault = FOC_FAULT_NONE;

    FOC_App_RefreshTelemetry(&g_foc_app);
    g_foc_app.warning_flags = FOC_App_GetVoltageWarningFlags(&g_foc_app);

    if ((g_foc_app.state != FOC_STATE_FAULT) ||
        ((g_foc_app.fault_code != FOC_FAULT_UNDERVOLTAGE) &&
         (g_foc_app.fault_code != FOC_FAULT_OVERVOLTAGE))) {
        return;
    }

    __disable_irq();
    if (FOC_App_GetVoltageTripFault(&g_foc_app, &voltageFault)) {
        g_foc_app.fault_code = voltageFault;
    } else if (FOC_App_IsVoltageFaultRecovered(&g_foc_app, g_foc_app.fault_code)) {
        g_foc_app.fault_code = FOC_FAULT_NONE;
        FOC_App_ResetMotionState(&g_foc_app);
        g_foc_app.state = g_foc_app.motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
    }
    __enable_irq();
}

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

static uint8_t UART_CommandParseFloatToken(const char **cursor, float *value)
{
    const char *p;
    float result = 0.0f;
    float frac_scale = 0.1f;
    int sign = 1;
    uint8_t has_digit = 0U;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return 0U;
    }

    p = *cursor;
    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        has_digit = 1U;
        result = (result * 10.0f) + (float)(*p - '0');
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            has_digit = 1U;
            result += (float)(*p - '0') * frac_scale;
            frac_scale *= 0.1f;
            p++;
        }
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value = (sign < 0) ? -result : result;
    *cursor = p;
    return 1U;
}

static uint8_t UART_CommandMatchPrefix(const char *cmd, const char *prefix, const char **args)
{
    size_t prefix_len;

    if ((cmd == NULL) || (prefix == NULL) || (args == NULL)) {
        return 0U;
    }

    prefix_len = UART_BoundedStrLen(prefix, UART_CMD_LINE_MAX);
    if (strncmp(cmd, prefix, prefix_len) != 0) {
        return 0U;
    }

    *args = &cmd[prefix_len];
    return 1U;
}

static uint8_t UART_CommandParseFloat1(const char *cmd, const char *prefix, float *v1)
{
    const char *p;

    if (!UART_CommandMatchPrefix(cmd, prefix, &p)) {
        return 0U;
    }
    if (!UART_CommandParseFloatToken(&p, v1)) {
        return 0U;
    }
    return (*p == '\0') ? 1U : 0U;
}

static uint8_t UART_CommandParseFloat2(const char *cmd, const char *prefix, float *v1, float *v2)
{
    const char *p;

    if (!UART_CommandMatchPrefix(cmd, prefix, &p)) {
        return 0U;
    }
    if (!UART_CommandParseFloatToken(&p, v1) || (*p != ',')) {
        return 0U;
    }
    p++;
    if (!UART_CommandParseFloatToken(&p, v2)) {
        return 0U;
    }
    return (*p == '\0') ? 1U : 0U;
}

static uint8_t UART_CommandIsPriority(const char *line)
{
    (void)line;
    return 0U;
}

static void UART_CommandQueueCopy(uint8_t index, const char *line)
{
    size_t copyLen;

    copyLen = UART_BoundedStrLen(line, UART_CMD_LINE_MAX - 1U);
    memcpy(s_uartCmdQueue[index], line, copyLen);
    s_uartCmdQueue[index][copyLen] = '\0';
}

static void UART_CommandQueuePushPriority(const char *line)
{
    uint8_t prev;

    if (line == NULL) {
        return;
    }

    prev = (uint8_t)((s_uartCmdQueueRead + UART_CMD_QUEUE_DEPTH - 1U) % UART_CMD_QUEUE_DEPTH);
    if (prev == s_uartCmdQueueWrite) {
        /* 队列满：为实时位置目标让路，丢弃尾部最新的普通命令 */
        s_uartCmdQueueWrite = (uint8_t)((s_uartCmdQueueWrite + UART_CMD_QUEUE_DEPTH - 1U) % UART_CMD_QUEUE_DEPTH);
    }

    UART_CommandQueueCopy(prev, line);
    s_uartCmdQueueRead = prev;
}

static void UART_CommandQueuePush(const char *line)
{
    uint8_t next;

    if (line == NULL) {
        return;
    }

    if (UART_CommandIsPriority(line)) {
        UART_CommandQueuePushPriority(line);
        return;
    }

    next = (uint8_t)((s_uartCmdQueueWrite + 1U) % UART_CMD_QUEUE_DEPTH);
    if (next == s_uartCmdQueueRead) {
        /* 队列满：丢弃最旧命令，优先保留最新输入 */
        s_uartCmdQueueRead = (uint8_t)((s_uartCmdQueueRead + 1U) % UART_CMD_QUEUE_DEPTH);
    }

    UART_CommandQueueCopy(s_uartCmdQueueWrite, line);
    s_uartCmdQueueWrite = next;
}

static void UART_CommandSendText(const char *text)
{
    size_t len;

    if (text == NULL) {
        return;
    }

    len = UART_BoundedStrLen(text, 512U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)text, (uint16_t)len, 200U);
}

static uint16_t UART_AdcNoiseIntegerSqrt(uint32_t value)
{
    uint32_t root = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0U) {
        if (value >= (root + bit)) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    return (uint16_t)root;
}

static void UART_AdcNoiseAccumInit(UART_AdcNoiseAccum_t *accum, uint16_t raw)
{
    accum->sum = 0U;
    accum->sumSq = 0ULL;
    accum->min = raw;
    accum->max = raw;
}

static void UART_AdcNoiseAccumAdd(UART_AdcNoiseAccum_t *accum, uint16_t raw)
{
    accum->sum += raw;
    accum->sumSq += ((uint64_t)raw * (uint64_t)raw);
    if (raw < accum->min) {
        accum->min = raw;
    }
    if (raw > accum->max) {
        accum->max = raw;
    }
}

static void UART_AdcNoiseAccumFinish(const UART_AdcNoiseAccum_t *accum,
                                     uint16_t count,
                                     ADC_Sampling_NoiseChannelStats_t *stats)
{
    uint32_t mean;
    uint64_t varianceNumerator;
    uint64_t varianceDenominator;
    uint32_t variance;

    mean = (accum->sum + ((uint32_t)count / 2U)) / (uint32_t)count;
    varianceNumerator = ((uint64_t)count * accum->sumSq) -
                        ((uint64_t)accum->sum * (uint64_t)accum->sum);
    varianceDenominator = (uint64_t)count * (uint64_t)count;
    variance = (uint32_t)((varianceNumerator + (varianceDenominator / 2ULL)) /
                          varianceDenominator);

    stats->min = accum->min;
    stats->max = accum->max;
    stats->mean = (uint16_t)mean;
    stats->peakToPeak = (uint16_t)(accum->max - accum->min);
    stats->stddev = UART_AdcNoiseIntegerSqrt(variance);
}

static uint8_t UART_CommandCanRunAdcNoiseTest(void)
{
    return ((g_foc_app.enable_pwm == 0U) &&
            (g_foc_app.state != FOC_STATE_RUNNING) &&
            (g_foc_app.state != FOC_STATE_PARAM_IDENTIFY)) ? 1U : 0U;
}

static uint16_t UART_CommandClampAdcNoiseSamples(uint16_t requestedSamples)
{
    uint16_t target = requestedSamples;

    if (target < ADC_NOISE_MIN_SAMPLES) {
        target = ADC_NOISE_MIN_SAMPLES;
    }
    if (target > ADC_NOISE_MAX_SAMPLES) {
        target = ADC_NOISE_MAX_SAMPLES;
    }

    return target;
}

static void UART_CommandAppendAdcNoiseChannel(char *buf,
                                              size_t len,
                                              const char *name,
                                              const ADC_Sampling_NoiseChannelStats_t *stats)
{
    size_t used;

    used = UART_BoundedStrLen(buf, len);
    if (used >= len) {
        return;
    }

    (void)snprintf(&buf[used], len - used,
                   ",%s:min=%u,max=%u,mean=%u,pp=%u,std=%u",
                   name,
                   (unsigned int)stats->min,
                   (unsigned int)stats->max,
                   (unsigned int)stats->mean,
                   (unsigned int)stats->peakToPeak,
                   (unsigned int)stats->stddev);
}

static void UART_CommandHandleAdcNoise(uint16_t requestedSamples)
{
    char response[512];
    ADC_Sampling_t *adc;

    if (!UART_CommandCanRunAdcNoiseTest()) {
        (void)snprintf(response, sizeof(response),
                       "ADC_NOISE,BUSY,state=%u,pwm=%u\r\n",
                       (unsigned int)g_foc_app.state,
                       (unsigned int)g_foc_app.enable_pwm);
        UART_CommandSendText(response);
        return;
    }

    if (s_adcNoiseActive != 0U) {
        UART_CommandSendText("ADC_NOISE,BUSY,active=1\r\n");
        return;
    }

    adc = ADC_Sampling_GetData();
    s_adcNoiseTargetSamples = UART_CommandClampAdcNoiseSamples(requestedSamples);
    s_adcNoiseCapturedSamples = 0U;
    s_adcNoiseStartTick = HAL_GetTick();
    s_adcNoiseTimeoutMs = 2000U + (uint32_t)s_adcNoiseTargetSamples;
    s_adcNoiseLastFrameSequence = adc->frameSequence;
    s_adcNoiseActive = 1U;

    (void)snprintf(response, sizeof(response),
                   "ADC_NOISE,START,n=%u\r\n",
                   (unsigned int)s_adcNoiseTargetSamples);
    UART_CommandSendText(response);
}

static void UART_CommandSendAdcNoiseResult(const ADC_Sampling_NoiseStats_t *stats)
{
    char response[512];
    size_t used;

    (void)snprintf(response, sizeof(response),
                   "ADC_NOISE,OK,n=%u",
                   (unsigned int)stats->samples);
    UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "A", &stats->currentA);
    UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "B", &stats->currentB);
    UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "C", &stats->currentC);
    UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "VBUS", &stats->vbus);
    used = UART_BoundedStrLen(response, sizeof(response));
    if (used < (sizeof(response) - 3U)) {
        response[used] = '\r';
        response[used + 1U] = '\n';
        response[used + 2U] = '\0';
    } else {
        response[sizeof(response) - 3U] = '\r';
        response[sizeof(response) - 2U] = '\n';
        response[sizeof(response) - 1U] = '\0';
    }
    UART_CommandSendText(response);
}

static void UART_CommandServiceAdcNoise(void)
{
    ADC_Sampling_t *adc;
    ADC_Sampling_NoiseStats_t stats;
    uint16_t raw[ADC_CHANNEL_COUNT];

    if (s_adcNoiseActive == 0U) {
        return;
    }

    if (!UART_CommandCanRunAdcNoiseTest()) {
        s_adcNoiseActive = 0U;
        UART_CommandSendText("ADC_NOISE,ERR,aborted\r\n");
        return;
    }

    if ((HAL_GetTick() - s_adcNoiseStartTick) > s_adcNoiseTimeoutMs) {
        s_adcNoiseActive = 0U;
        UART_CommandSendText("ADC_NOISE,ERR,timeout\r\n");
        return;
    }

    adc = ADC_Sampling_GetData();
    if (adc->frameSequence == s_adcNoiseLastFrameSequence) {
        return;
    }
    s_adcNoiseLastFrameSequence = adc->frameSequence;

    raw[ADC_CH_CURRENT_A] = adc->rawCurrentA;
    raw[ADC_CH_CURRENT_B] = adc->rawCurrentB;
    raw[ADC_CH_CURRENT_C] = adc->rawCurrentC;
    raw[ADC_CH_VBUS] = adc->rawVbus;

    if (s_adcNoiseCapturedSamples == 0U) {
        UART_AdcNoiseAccumInit(&s_adcNoiseAccum[ADC_CH_CURRENT_A], raw[ADC_CH_CURRENT_A]);
        UART_AdcNoiseAccumInit(&s_adcNoiseAccum[ADC_CH_CURRENT_B], raw[ADC_CH_CURRENT_B]);
        UART_AdcNoiseAccumInit(&s_adcNoiseAccum[ADC_CH_CURRENT_C], raw[ADC_CH_CURRENT_C]);
        UART_AdcNoiseAccumInit(&s_adcNoiseAccum[ADC_CH_VBUS], raw[ADC_CH_VBUS]);
    }

    UART_AdcNoiseAccumAdd(&s_adcNoiseAccum[ADC_CH_CURRENT_A], raw[ADC_CH_CURRENT_A]);
    UART_AdcNoiseAccumAdd(&s_adcNoiseAccum[ADC_CH_CURRENT_B], raw[ADC_CH_CURRENT_B]);
    UART_AdcNoiseAccumAdd(&s_adcNoiseAccum[ADC_CH_CURRENT_C], raw[ADC_CH_CURRENT_C]);
    UART_AdcNoiseAccumAdd(&s_adcNoiseAccum[ADC_CH_VBUS], raw[ADC_CH_VBUS]);
    s_adcNoiseCapturedSamples++;

    if (s_adcNoiseCapturedSamples < s_adcNoiseTargetSamples) {
        return;
    }

    memset((void*)&stats, 0, sizeof(stats));
    stats.samples = s_adcNoiseCapturedSamples;
    UART_AdcNoiseAccumFinish(&s_adcNoiseAccum[ADC_CH_CURRENT_A], stats.samples, &stats.currentA);
    UART_AdcNoiseAccumFinish(&s_adcNoiseAccum[ADC_CH_CURRENT_B], stats.samples, &stats.currentB);
    UART_AdcNoiseAccumFinish(&s_adcNoiseAccum[ADC_CH_CURRENT_C], stats.samples, &stats.currentC);
    UART_AdcNoiseAccumFinish(&s_adcNoiseAccum[ADC_CH_VBUS], stats.samples, &stats.vbus);

    s_adcNoiseActive = 0U;
    UART_CommandSendAdcNoiseResult(&stats);
}

static uint8_t UART_CommandCanRunAdcPhaseScan(void)
{
    return ((g_foc_app.enable_pwm != 0U) &&
            (g_foc_app.state == FOC_STATE_RUNNING)) ? 1U : 0U;
}

static uint8_t UART_CommandAdcSectorMinPhase(uint16_t ccrA, uint16_t ccrB, uint16_t ccrC)
{
    if ((ccrA <= ccrB) && (ccrA <= ccrC)) {
        return 0U;
    }
    if (ccrB <= ccrC) {
        return 1U;
    }
    return 2U;
}

static char UART_CommandAdcSectorPhaseName(uint8_t phase)
{
    static const char names[UART_ADC_SECTOR_SCAN_MIN_PHASES] = {'A', 'B', 'C'};

    if (phase >= UART_ADC_SECTOR_SCAN_MIN_PHASES) {
        return '?';
    }
    return names[phase];
}

static void UART_CommandAdcSectorBucketAdd(UART_AdcSectorBucket_t *bucket,
                                           const ADC_Sampling_t *adc)
{
    if (bucket->count == 0U) {
        UART_AdcNoiseAccumInit(&bucket->accA, adc->rawCurrentA);
        UART_AdcNoiseAccumInit(&bucket->accB, adc->rawCurrentB);
        UART_AdcNoiseAccumInit(&bucket->accC, adc->rawCurrentC);
        bucket->sumCcrA = 0U;
        bucket->sumCcrB = 0U;
        bucket->sumCcrC = 0U;
        bucket->winA = 0U;
        bucket->winB = 0U;
        bucket->winC = 0U;
    }

    UART_AdcNoiseAccumAdd(&bucket->accA, adc->rawCurrentA);
    UART_AdcNoiseAccumAdd(&bucket->accB, adc->rawCurrentB);
    UART_AdcNoiseAccumAdd(&bucket->accC, adc->rawCurrentC);
    bucket->sumCcrA += adc->pwmCompareA;
    bucket->sumCcrB += adc->pwmCompareB;
    bucket->sumCcrC += adc->pwmCompareC;
    bucket->winA |= adc->lowSideValidA;
    bucket->winB |= adc->lowSideValidB;
    bucket->winC |= adc->lowSideValidC;
    bucket->count++;
}

static void UART_CommandHandleAdcSectorScan(uint16_t requestedSamples)
{
    UART_AdcSectorBucket_t buckets[UART_ADC_SECTOR_SCAN_SECTORS][UART_ADC_SECTOR_SCAN_MIN_PHASES];
    char response[256];
    ADC_Sampling_t *adc;
    uint16_t samples;
    uint16_t captured = 0U;
    uint32_t lastFrame;
    uint32_t startTick;
    uint32_t timeoutMs;
    uint8_t sector;
    uint8_t minPhase;

    if (!UART_CommandCanRunAdcPhaseScan()) {
        (void)snprintf(response, sizeof(response),
                       "ADC_SECTOR_SCAN,BUSY,state=%u,pwm=%u\r\n",
                       (unsigned)g_foc_app.state,
                       (unsigned)g_foc_app.enable_pwm);
        UART_CommandSendText(response);
        return;
    }

    memset((void*)buckets, 0, sizeof(buckets));
    samples = UART_CommandClampAdcNoiseSamples(requestedSamples);
    adc = ADC_Sampling_GetData();
    lastFrame = adc->frameSequence;
    startTick = HAL_GetTick();
    timeoutMs = 500U + (uint32_t)samples;

    (void)snprintf(response, sizeof(response),
                   "ADC_SECTOR_SCAN,START,n=%u\r\n",
                   (unsigned)samples);
    UART_CommandSendText(response);

    while (captured < samples) {
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            break;
        }

        if (adc->frameSequence == lastFrame) {
            continue;
        }
        lastFrame = adc->frameSequence;

        sector = g_foc_app.foc.svpwm.sector;
        if ((sector < 1U) || (sector > UART_ADC_SECTOR_SCAN_SECTORS)) {
            continue;
        }

        minPhase = UART_CommandAdcSectorMinPhase(adc->pwmCompareA, adc->pwmCompareB, adc->pwmCompareC);
        UART_CommandAdcSectorBucketAdd(&buckets[sector - 1U][minPhase], adc);
        captured++;
    }

    for (sector = 1U; sector <= UART_ADC_SECTOR_SCAN_SECTORS; sector++) {
        for (minPhase = 0U; minPhase < UART_ADC_SECTOR_SCAN_MIN_PHASES; minPhase++) {
            UART_AdcSectorBucket_t *bucket = &buckets[sector - 1U][minPhase];
            ADC_Sampling_NoiseChannelStats_t statsA;
            ADC_Sampling_NoiseChannelStats_t statsB;
            ADC_Sampling_NoiseChannelStats_t statsC;

            if (bucket->count == 0U) {
                continue;
            }

            UART_AdcNoiseAccumFinish(&bucket->accA, bucket->count, &statsA);
            UART_AdcNoiseAccumFinish(&bucket->accB, bucket->count, &statsB);
            UART_AdcNoiseAccumFinish(&bucket->accC, bucket->count, &statsC);
            (void)snprintf(response, sizeof(response),
                           "ADC_SECTOR_SCAN,BUCKET,sector=%u,min=%c,count=%u,A:min=%u,max=%u,mean=%u,B:min=%u,max=%u,mean=%u,C:min=%u,max=%u,mean=%u,CCR:%u/%u/%u,win:%u/%u/%u\r\n",
                           (unsigned)sector,
                           UART_CommandAdcSectorPhaseName(minPhase),
                           (unsigned)bucket->count,
                           (unsigned)statsA.min,
                           (unsigned)statsA.max,
                           (unsigned)statsA.mean,
                           (unsigned)statsB.min,
                           (unsigned)statsB.max,
                           (unsigned)statsB.mean,
                           (unsigned)statsC.min,
                           (unsigned)statsC.max,
                           (unsigned)statsC.mean,
                           (unsigned)((bucket->sumCcrA + ((uint32_t)bucket->count / 2U)) / (uint32_t)bucket->count),
                           (unsigned)((bucket->sumCcrB + ((uint32_t)bucket->count / 2U)) / (uint32_t)bucket->count),
                           (unsigned)((bucket->sumCcrC + ((uint32_t)bucket->count / 2U)) / (uint32_t)bucket->count),
                           (unsigned)bucket->winA,
                           (unsigned)bucket->winB,
                           (unsigned)bucket->winC);
            UART_CommandSendText(response);
        }
    }

    (void)snprintf(response, sizeof(response),
                   "ADC_SECTOR_SCAN,DONE,captured=%u\r\n",
                   (unsigned)captured);
    UART_CommandSendText(response);
}

static uint16_t UART_CommandCapturePhaseScanPoint(uint16_t trigger,
                                                  uint16_t requestedSamples,
                                                  UART_AdcNoiseAccum_t *accA,
                                                  UART_AdcNoiseAccum_t *accB,
                                                  UART_AdcNoiseAccum_t *accC,
                                                  uint8_t *winA,
                                                  uint8_t *winB,
                                                  uint8_t *winC)
{
    ADC_Sampling_t *adc;
    uint16_t count = 0U;
    uint32_t lastFrame;
    uint32_t startTick;
    uint32_t timeoutMs;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger);
    adc = ADC_Sampling_GetData();
    lastFrame = adc->frameSequence;
    startTick = HAL_GetTick();
    timeoutMs = 250U + (uint32_t)requestedSamples;

    while (count < requestedSamples) {
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            break;
        }

        if (adc->frameSequence == lastFrame) {
            continue;
        }
        lastFrame = adc->frameSequence;

        if (count == 0U) {
            UART_AdcNoiseAccumInit(accA, adc->rawCurrentA);
            UART_AdcNoiseAccumInit(accB, adc->rawCurrentB);
            UART_AdcNoiseAccumInit(accC, adc->rawCurrentC);
        }

        UART_AdcNoiseAccumAdd(accA, adc->rawCurrentA);
        UART_AdcNoiseAccumAdd(accB, adc->rawCurrentB);
        UART_AdcNoiseAccumAdd(accC, adc->rawCurrentC);
        *winA = adc->lowSideValidA;
        *winB = adc->lowSideValidB;
        *winC = adc->lowSideValidC;
        count++;
    }

    return count;
}

static void UART_CommandHandleAdcPhaseScan(uint16_t requestedSamples)
{
    char response[256];
    uint16_t samples;
    uint16_t originalTrigger;
    uint16_t trigger;

    if (!UART_CommandCanRunAdcPhaseScan()) {
        (void)snprintf(response, sizeof(response),
                       "ADC_PHASE_SCAN,BUSY,state=%u,pwm=%u\r\n",
                       (unsigned)g_foc_app.state,
                       (unsigned)g_foc_app.enable_pwm);
        UART_CommandSendText(response);
        return;
    }

    samples = UART_CommandClampAdcNoiseSamples(requestedSamples);
    originalTrigger = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    (void)snprintf(response, sizeof(response),
                   "ADC_PHASE_SCAN,START,n=%u,start=%u,end=%u,step=%u\r\n",
                   (unsigned)samples,
                   (unsigned)UART_ADC_PHASE_SCAN_MIN_TRIGGER,
                   (unsigned)UART_ADC_PHASE_SCAN_MAX_TRIGGER,
                   (unsigned)UART_ADC_PHASE_SCAN_TRIGGER_STEP);
    UART_CommandSendText(response);

    for (trigger = UART_ADC_PHASE_SCAN_MIN_TRIGGER;
         trigger <= UART_ADC_PHASE_SCAN_MAX_TRIGGER;
         trigger = (uint16_t)(trigger + UART_ADC_PHASE_SCAN_TRIGGER_STEP)) {
        UART_AdcNoiseAccum_t accA;
        UART_AdcNoiseAccum_t accB;
        UART_AdcNoiseAccum_t accC;
        ADC_Sampling_NoiseChannelStats_t statsA;
        ADC_Sampling_NoiseChannelStats_t statsB;
        ADC_Sampling_NoiseChannelStats_t statsC;
        uint8_t winA = 0U;
        uint8_t winB = 0U;
        uint8_t winC = 0U;
        uint16_t captured;

        captured = UART_CommandCapturePhaseScanPoint(trigger, samples, &accA, &accB, &accC, &winA, &winB, &winC);
        if (captured == 0U) {
            (void)snprintf(response, sizeof(response),
                           "ADC_PHASE_SCAN,POINT,trig=%u,n=0,timeout=1\r\n",
                           (unsigned)trigger);
            UART_CommandSendText(response);
            continue;
        }

        UART_AdcNoiseAccumFinish(&accA, captured, &statsA);
        UART_AdcNoiseAccumFinish(&accB, captured, &statsB);
        UART_AdcNoiseAccumFinish(&accC, captured, &statsC);
        (void)snprintf(response, sizeof(response),
                       "ADC_PHASE_SCAN,POINT,trig=%u,n=%u,A:min=%u,max=%u,mean=%u,B:min=%u,max=%u,mean=%u,C:min=%u,max=%u,mean=%u,win=%u/%u/%u\r\n",
                       (unsigned)trigger,
                       (unsigned)captured,
                       (unsigned)statsA.min,
                       (unsigned)statsA.max,
                       (unsigned)statsA.mean,
                       (unsigned)statsB.min,
                       (unsigned)statsB.max,
                       (unsigned)statsB.mean,
                       (unsigned)statsC.min,
                       (unsigned)statsC.max,
                       (unsigned)statsC.mean,
                       (unsigned)winA,
                       (unsigned)winB,
                       (unsigned)winC);
        UART_CommandSendText(response);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, originalTrigger);
    (void)snprintf(response, sizeof(response),
                   "ADC_PHASE_SCAN,DONE,restore=%u\r\n",
                   (unsigned)originalTrigger);
    UART_CommandSendText(response);
}

static void UART_CommandExecute(const char *cmd)
{
    long int int_arg;
    float f1, f2;

    if (cmd == NULL) {
        return;
    }

    if (strcmp(cmd, "CMD:FAULT_DETAIL") == 0) {
        DrvUart_UploadImmediate();
        return;
    }
    if (strcmp(cmd, "CMD:JDIAG") == 0) {
        DrvUart_UploadJDiag();
        return;
    }
    if (sscanf(cmd, "CMD:COG_PHASE,%f", &f1) == 1) {
    if (strcmp(cmd, "CMD:COG_CFG?") == 0) {
        DrvUart_QueryCogCfg();
        return;
    }
    if (sscanf(cmd, "CMD:COG_CFG,%f,%f", &f1, &f2) == 2) {
        if (f1 >= 0.0f && f1 <= 1.0f) g_foc_app.cogging_lut.gain = f1;
        g_foc_app.cogging_lut.phase_offset_rad = f2 * FOC_PI / 180.0f;
        return;
    }
        g_foc_app.cogging_lut.phase_offset_rad = f1;
        return;
    }

    /* --- V5 Motion Config: Query --- */
    if (strcmp(cmd, "CMD:MOTION_CFG?") == 0) {
        char resp[128];
        char s_str[20], a_str[20], c_str[20];
        DrvUart_FormatFixed(s_str, sizeof(s_str), g_foc_app.position_speed_limit_radps, 3U);
        DrvUart_FormatFixed(a_str, sizeof(a_str), g_foc_app.position_accel_limit_radps2, 3U);
        DrvUart_FormatFixed(c_str, sizeof(c_str), g_foc_app.position_cruise_speed_radps, 3U);
        (void)snprintf(resp, sizeof(resp),
                 "MOTION_CFG,OK,speed=%s,accel=%s,cruise=%s\r\n", s_str, a_str, c_str);
        UART_CommandSendText(resp);
        return;
    }

    /* --- V5 Motion Config: Reset --- */
    if (strcmp(cmd, "CMD:MOTION_CFG_RESET") == 0) {
        g_foc_app.position_speed_limit_radps  = FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
        g_foc_app.position_accel_limit_radps2 = FOC_MOTION_CFG_ACCEL_LIMIT_DEFAULT;
        g_foc_app.position_cruise_speed_radps = FOC_MOTION_CFG_CRUISE_SPEED_DEFAULT;
        g_foc_app.pos_pd.output_max =  FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
        g_foc_app.pos_pd.output_min = -FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
        {
            float lim = FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
            if (g_foc_app.speed_ref_ramped > lim)  g_foc_app.speed_ref_ramped = lim;
            if (g_foc_app.speed_ref_ramped < -lim) g_foc_app.speed_ref_ramped = -lim;
            g_foc_app.speed_ref_ramped_prev = g_foc_app.speed_ref_ramped;
        }
        UART_CommandSendText("MOTION_CFG,RESET,OK\r\n");
        return;
    }

    /* --- V5 Motion Config: Set --- */
    {
        float s, a, c;
        if (sscanf(cmd, "CMD:MOTION_CFG,%f,%f,%f", &s, &a, &c) == 3) {
            char resp[128];
            if (s < FOC_MOTION_CFG_SPEED_LIMIT_MIN || s > FOC_MOTION_CFG_SPEED_LIMIT_MAX ||
                a < FOC_MOTION_CFG_ACCEL_LIMIT_MIN || a > FOC_MOTION_CFG_ACCEL_LIMIT_MAX ||
                c < 0.0f || c > s) {
                UART_CommandSendText("MOTION_CFG,FAIL,range\r\n");
                return;
            }
            g_foc_app.position_speed_limit_radps  = s;
            g_foc_app.position_accel_limit_radps2 = a;
            g_foc_app.position_cruise_speed_radps = c;
            g_foc_app.pos_pd.output_max =  s;
            g_foc_app.pos_pd.output_min = -s;
            if (g_foc_app.speed_ref_ramped > s)  g_foc_app.speed_ref_ramped = s;
            if (g_foc_app.speed_ref_ramped < -s) g_foc_app.speed_ref_ramped = -s;
            g_foc_app.speed_ref_ramped_prev = g_foc_app.speed_ref_ramped;
            {
                char s_str[20], a_str[20], c_str[20];
                DrvUart_FormatFixed(s_str, sizeof(s_str), s, 3U);
                DrvUart_FormatFixed(a_str, sizeof(a_str), a, 3U);
                DrvUart_FormatFixed(c_str, sizeof(c_str), c, 3U);
                (void)snprintf(resp, sizeof(resp),
                         "MOTION_CFG,OK,speed=%s,accel=%s,cruise=%s\r\n", s_str, a_str, c_str);
                UART_CommandSendText(resp);
            }
            return;
        }
    }

    if (sscanf(cmd, "CMD:UNLOCK,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            g_foc_app.power_unlocked = 1U;
        } else {
            g_foc_app.power_unlocked = 0U;
            g_foc_app.stall_mode_armed = 0U;
            FOC_App_StopIdentify(&g_foc_app);
            FOC_App_Disable(&g_foc_app);
        }
        return;
    }

    if (sscanf(cmd, "CMD:ADC_NOISE,%ld", &int_arg) == 1) {
        if (int_arg < 0) {
            int_arg = 0;
        }
        UART_CommandHandleAdcNoise((uint16_t)int_arg);
        return;
    }

    if (sscanf(cmd, "CMD:ADC_PHASE_SCAN,%ld", &int_arg) == 1) {
        if (int_arg < 0) {
            int_arg = 0;
        }
        UART_CommandHandleAdcPhaseScan((uint16_t)int_arg);
        return;
    }

    if (sscanf(cmd, "CMD:ADC_SECTOR_SCAN,%ld", &int_arg) == 1) {
        if (int_arg < 0) {
            int_arg = 0;
        }
        UART_CommandHandleAdcSectorScan((uint16_t)int_arg);
        return;
    }

    if (sscanf(cmd, "CMD:TLE_GPIO_DIAG,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            if ((g_foc_app.enable_pwm != 0U) ||
                (g_foc_app.state == FOC_STATE_RUNNING) ||
                (g_foc_app.state == FOC_STATE_PARAM_IDENTIFY)) {
                UART_CommandSendText("TLE_GPIO_DIAG,BUSY,pwm_active=1\r\n");
                return;
            }
            TLE5012_GpioDiagStart();
            UART_CommandSendText("TLE_GPIO_DIAG,START,step_ms=500,watch=tle5012_gpio_diag\r\n");
        } else {
            TLE5012_GpioDiagStop();
            UART_CommandSendText("TLE_GPIO_DIAG,STOP\r\n");
        }
        return;
    }

    if (strcmp(cmd, "CMD:TLE_RAW") == 0) {
        char response[256];
        uint32_t angle_cdeg = (uint32_t)((tle5012_sensor.angle * 100.0f) + 0.5f);
        snprintf(response,
                 sizeof(response),
                 "TLE_RAW,raw=0x%04X,safety=0x%04X,status=0x%02X,recv_crc=0x%02X,calc_crc=0x%02X,data_ok=%u,crc_error=%u,valid=%u,angle=%lu.%02lu\r\n",
                 tle5012_sensor.raw_word,
                 tle5012_sensor.safety_word,
                 tle5012_sensor.status,
                 tle5012_sensor.received_crc,
                 tle5012_sensor.calculated_crc,
                 tle5012_sensor.data_ok,
                 tle5012_sensor.crc_error,
                 tle5012_sensor.data_valid,
                 (unsigned long)(angle_cdeg / 100UL),
                 (unsigned long)(angle_cdeg % 100UL));
        UART_CommandSendText(response);
        return;
    }

    if (sscanf(cmd, "CMD:STALL_MODE,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            g_foc_app.stall_mode_armed = 1U;
        } else {
            g_foc_app.stall_mode_armed = 0U;
        }
        return;
    }

    if (sscanf(cmd, "CMD:ENABLE,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            if (!g_foc_app.power_unlocked) {
                return;
            }
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

    if (UART_CommandParseFloat2(cmd, "CMD:IREF,", &f1, &f2)) {
        FOC_App_SetCurrentRef(&g_foc_app, f1, f2);
        return;
    }

    if (UART_CommandParseFloat1(cmd, "CMD:SREF,", &f1)) {
        FOC_App_SetSpeedRef(&g_foc_app, f1);
        return;
    }

    if (UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1)) {
        float pos_before = g_foc_app.pos_ref;
        __disable_irq();
        FOC_App_SetPositionRef(&g_foc_app, f1);
        FOC_App_PositionLoop(&g_foc_app);
        g_foc_app.position_pref_cmd_count_diag++;
        g_foc_app.position_pref_raw_diag = f1;
        g_foc_app.position_pref_mapped_diag = g_foc_app.pos_ref;
        g_foc_app.position_pref_before_diag = pos_before;
        g_foc_app.position_pref_after_diag = g_foc_app.pos_ref;
        g_foc_app.position_pref_user_set_diag = g_foc_app.position_ref_user_set;
        __enable_irq();
        return;
    }

    if (strcmp(cmd, "CMD:HOME") == 0) {
        char response[64];
        if (g_foc_app.enable_pwm != 0U) {
            snprintf(response, sizeof(response), "HOME,FAIL,motor_running\r\n");
            UART_CommandSendText(response);
            return;
        }
        if (!Param_IsValid(&g_foc_app.motor_param)) {
            snprintf(response, sizeof(response), "HOME,FAIL,not_identified\r\n");
            UART_CommandSendText(response);
            return;
        }
        float old_offset = g_foc_app.motor_param.mech_zero_offset;
        MI_SetMechZero(&g_foc_app.motor_param, g_foc_app.theta_mech);
        if (g_foc_app.position_ref_user_set) {
            float dir = (g_foc_app.motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
            g_foc_app.pos_ref = FOC_AngleNormalize(
                g_foc_app.pos_ref + (old_offset - g_foc_app.motor_param.mech_zero_offset) * dir);
        }
        FOC_App_SaveParam(&g_foc_app);
        snprintf(response, sizeof(response),
                 "HOME,OK,%d.%03d\r\n",
                 (int)g_foc_app.motor_param.mech_zero_offset,
                 (int)(fabsf(g_foc_app.motor_param.mech_zero_offset * 1000.0f)) % 1000);
        UART_CommandSendText(response);
        return;
    }

    if (strcmp(cmd, "CMD:CLEAR_HOME") == 0) {
        char response[64];
        if (g_foc_app.enable_pwm != 0U) {
            snprintf(response, sizeof(response), "CLEAR_HOME,FAIL,motor_running\r\n");
            UART_CommandSendText(response);
            return;
        }
        if (!Param_IsValid(&g_foc_app.motor_param)) {
            snprintf(response, sizeof(response), "CLEAR_HOME,FAIL,not_identified\r\n");
            UART_CommandSendText(response);
            return;
        }
        float old_offset = g_foc_app.motor_param.mech_zero_offset;
        g_foc_app.motor_param.mech_zero_offset = 0.0f;
        if (g_foc_app.position_ref_user_set) {
            float dir = (g_foc_app.motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
            g_foc_app.pos_ref = FOC_AngleNormalize(
                g_foc_app.pos_ref + old_offset * dir);
        }
        FOC_App_SaveParam(&g_foc_app);
        snprintf(response, sizeof(response), "CLEAR_HOME,OK,0.000\r\n");
        UART_CommandSendText(response);
        return;
    }

    if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1) {
        if (int_arg != 0) {
            if (!g_foc_app.power_unlocked) {
                return;
            }
            FOC_App_StartIdentify(&g_foc_app);
        } else {
            FOC_App_StopIdentify(&g_foc_app);
        }
        return;
    }

    if (UART_CommandParseFloat2(cmd, "CMD:VBUS_LIMIT,", &f1, &f2)) {
        float oldUndervoltage = g_foc_app.protection.undervoltage_limit_v;
        float oldOvervoltage = g_foc_app.protection.overvoltage_limit_v;

        if ((g_foc_app.enable_pwm == 0U) &&
            (g_foc_app.state != FOC_STATE_RUNNING) &&
            (g_foc_app.state != FOC_STATE_PARAM_IDENTIFY)) {
            FOC_App_SetVoltageThresholds(&g_foc_app, f1, f2);
            if ((fabsf(g_foc_app.protection.undervoltage_limit_v - oldUndervoltage) > 0.0005f) ||
                (fabsf(g_foc_app.protection.overvoltage_limit_v - oldOvervoltage) > 0.0005f)) {
                UART_ReevaluateVoltageFaultAfterThresholdUpdate();
            }
        }
        return;
    }

    if (sscanf(cmd, "CMD:MOTOR_PN,%ld", &int_arg) == 1) {
        if (int_arg >= 1L && int_arg <= 50L) {
            __disable_irq();
            FOC_App_SetPolePairs(&g_foc_app, (uint8_t)int_arg);
            __enable_irq();
        }
        return;
    }

    if (sscanf(cmd, "CMD:ENCODER_DIR,%ld", &int_arg) == 1) {
        char response[64];

        if ((g_foc_app.enable_pwm != 0U) ||
            (g_foc_app.state == FOC_STATE_RUNNING) ||
            (g_foc_app.state == FOC_STATE_PARAM_IDENTIFY)) {
            snprintf(response, sizeof(response), "ENCODER_DIR,FAIL,busy\r\n");
            UART_CommandSendText(response);
            return;
        }

        if ((int_arg == 1L) || (int_arg == -1L)) {
            __disable_irq();
            g_foc_app.motor_param.encoder_dir = (int8_t)int_arg;
            g_foc_app.motor_param.valid_flag = 0U;
            g_foc_app.motor_identified = 0U;
            g_foc_app.mi_handle.pn_last_calc = 0.0f;
            g_foc_app.mi_handle.pn_last_delta_mech = 0.0f;
            g_foc_app.mi_handle.pn_last_delta_elec = 0.0f;
            g_foc_app.mi_handle.pn_observed_dir = 0;
            if (g_foc_app.state == FOC_STATE_READY) {
                g_foc_app.state = FOC_STATE_IDLE;
            }
            __enable_irq();
            snprintf(response, sizeof(response), "ENCODER_DIR,OK,%ld\r\n", int_arg);
        } else {
            snprintf(response, sizeof(response), "ENCODER_DIR,FAIL,invalid=%ld\r\n", int_arg);
        }
        UART_CommandSendText(response);
        return;
    }

    if (strcmp(cmd, "CMD:CLEAR_FAULT") == 0) {
        uint16_t fs1 = 0U, fs2 = 0U, ocp = 0U;
        uint8_t drv_fault_active = 1U;
        uint8_t encoder_ok;
        uint8_t vbus_ok;
        FOC_FaultCode_t voltageFault = FOC_FAULT_NONE;

        (void)DRV8350S_ClearFaults(&drv8350s);

        FOC_App_RefreshTelemetry(&g_foc_app);
        g_foc_app.warning_flags = FOC_App_GetVoltageWarningFlags(&g_foc_app);

        if ((DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_FAULT_STATUS_1, &fs1) == 0) &&
            (DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_VGS_STATUS_2, &fs2) == 0) &&
            (DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_OCP_CTRL, &ocp) == 0)) {
            drv8350s.runtime.regFaultStatus1 = fs1;
            drv8350s.runtime.regVgsStatus2 = fs2;
            drv8350s.runtime.regOcpCtrl = ocp;
            drv8350s.runtime.spiError = 0U;
            drv8350s.readReq.registerAddr = DRV8350S_REG_OCP_CTRL;
            DRV8350S_UpdateFaultState(&drv8350s);
            drv_fault_active = drv8350s.runtime.isFaultActive;
        } else {
            drv8350s.runtime.spiError = 1U;
            drv8350s.readReq.registerAddr = DRV8350S_REG_OCP_CTRL;
            DRV8350S_UpdateFaultState(&drv8350s);
            drv_fault_active = drv8350s.runtime.isFaultActive;
        }

        encoder_ok = g_foc_app.motor_identified ? TLE5012_IsDataValid() : 1U;
        vbus_ok = FOC_App_GetVoltageTripFault(&g_foc_app, &voltageFault) ? 0U : 1U;

        __disable_irq();
        if ((!drv_fault_active) && encoder_ok && vbus_ok) {
            g_foc_app.fault_code = FOC_FAULT_NONE;
            g_foc_app.pending_disable = 0U;
            FOC_App_ResetMotionState(&g_foc_app);
            if (g_foc_app.state == FOC_STATE_FAULT) {
                g_foc_app.state = g_foc_app.motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
            }
        } else {
            if (drv_fault_active) {
                g_foc_app.fault_code = FOC_FAULT_DRV8350S;
            } else if (!encoder_ok) {
                g_foc_app.fault_code = FOC_FAULT_ENCODER;
            } else if (FOC_App_GetVoltageTripFault(&g_foc_app, &voltageFault)) {
                g_foc_app.fault_code = voltageFault;
            }
        }
        __enable_irq();
        return;
    }

    if (UART_CommandParseFloat2(cmd, "CMD:PI_CURRENT,", &f1, &f2)) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            float current_ki_discrete = f2 / (float)FOC_CONTROL_FREQ;
            __disable_irq();
            FOC_PI_Init(&g_foc_app.foc.pi_d, f1, current_ki_discrete, g_foc_app.foc.pi_d.output_max, g_foc_app.foc.pi_d.output_min);
            FOC_PI_Init(&g_foc_app.foc.pi_q, f1, current_ki_discrete, g_foc_app.foc.pi_q.output_max, g_foc_app.foc.pi_q.output_min);
            __enable_irq();
        }
        return;
    }

    if (UART_CommandParseFloat2(cmd, "CMD:PI_SPEED,", &f1, &f2)) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            __disable_irq();
            FOC_PI_Init(&g_foc_app.pi_speed, f1, f2, g_foc_app.pi_speed.output_max, g_foc_app.pi_speed.output_min);
            __enable_irq();
        }
        return;
    }

    if (UART_CommandParseFloat2(cmd, "CMD:PD_POS,", &f1, &f2)) {
        if (f1 > 0.0f && f2 >= 0.0f) {
            __disable_irq();
            FOC_App_SetPositionPDGains(&g_foc_app, f1, f2);
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

    UART_CommandServiceAdcNoise();
    TLE5012_GpioDiagService();
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
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  /* DEBUG: scope on PB0 to measure actual ISR freq */
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
    /* ===== FOC current loop =====
     * PWM: 20kHz center-aligned (ARR=24, PSC=239).
     * TIM1_UP rate: TBD by scope (may be 20kHz or 40kHz depending on update event mode).
     * Effective FOC rate: ADC-frame-gated, ~20kHz (matches PWM/ADC trigger).
     */
    FOC_App_TIM1_IRQHandler(&g_foc_app);
    
    /* TLE5012 encoder read: target ~5kHz. Actual rate depends on TIM1_UP freq (TBD by scope). */
    static uint8_t tle5012_div_counter = 0;
    if (++tle5012_div_counter >= 4)
    {
        tle5012_div_counter = 0;
        TLE5012_StartRead();
    }

    /* Speed loop: target ~2kHz. Actual rate depends on TIM1_UP freq (TBD by scope). */
    static uint8_t speed_loop_div_counter = 0;
    if (++speed_loop_div_counter >= 10) 
    {
        speed_loop_div_counter = 0;
        FOC_App_SpeedLoop(&g_foc_app);
    }
    
    /* Position loop: target ~200Hz. Actual rate depends on TIM1_UP freq (TBD by scope). */
    static uint8_t position_loop_div_counter = 0;
    if (++position_loop_div_counter >= 100) 
    {
        position_loop_div_counter = 0;
        FOC_App_PositionLoop(&g_foc_app);
    }
    
    /* DRV8350S status poll. Rate = TIM1_UP freq (TBD by scope). */
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
        (void)drv8350s.rxBuf[0];  /* Current frame response */
        
        switch (reg) {
            case DRV8350S_REG_FAULT_STATUS_1:
                break;
            case DRV8350S_REG_VGS_STATUS_2:
                break;
            default:
                break;
        }

        if (DRV8350S_ShouldHardShutdown(drv8350s.runtime.faultFlags)) {
            FOC_App_RequestFaultShutdownFromISR(&g_foc_app, FOC_FAULT_DRV8350S);
        }
    } else if (hspi == &hspi3) {
        TLE5012_ProcessData(tle5012_rx_buf);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        TLE5012_HandleTxComplete();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        DRV8350S_DMA_ErrorCallback(&drv8350s);
        drv8350s.runtime.errorCount++;
    }
    
    if (hspi == &hspi3) {
        TLE5012_HandleTransferError();
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
