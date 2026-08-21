/**
 * @file    adc_sampling.c
 * @brief   ADC采样和计算模块实现
 * 
 * 【修复记录】
 * v1.1 - 添加ADC校准超时保护，防止死循环 (修复ADC-001)
 */

#include "adc_sampling.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* 私有变量 */
static ADC_Sampling_t s_adcData;
static volatile uint8_t s_controlWindowOpen = 1U;
static volatile uint32_t s_lastConsumedFrameSequence = 0U;

/* 外部变量声明（在stm32h7xx_it.c中定义） */
extern volatile uint16_t adc_data[8];

typedef struct {
    uint32_t sum;
    uint64_t sumSq;
    uint16_t min;
    uint16_t max;
} ADC_NoiseAccum_t;

static uint16_t ADC_Sampling_IntegerSqrt(uint32_t value)
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

static void ADC_NoiseAccum_Init(ADC_NoiseAccum_t *accum, uint16_t first)
{
    accum->sum = 0U;
    accum->sumSq = 0ULL;
    accum->min = first;
    accum->max = first;
}

static void ADC_NoiseAccum_Add(ADC_NoiseAccum_t *accum, uint16_t raw)
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

static void ADC_NoiseAccum_Finish(const ADC_NoiseAccum_t *accum,
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
    stats->stddev = ADC_Sampling_IntegerSqrt(variance);
}

/**
 * @brief 初始化ADC采样模块
 */
int8_t ADC_Sampling_Init(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL) {
        return -1;
    }
    
    /* 清零数据结构 */
    memset((void*)&s_adcData, 0, sizeof(ADC_Sampling_t));
    
    /* 设置默认零点偏移（理论值为VCC/2对应的ADC值） */
    s_adcData.offsetA = (int16_t)ADC_HALF;
    s_adcData.offsetB = (int16_t)ADC_HALF;
    s_adcData.offsetC = (int16_t)ADC_HALF;
    s_adcData.gain_c = 1.0f;    /* CH_CFG 默认: 无补救 */
    s_adcData.recon_c = 0U;
    ADC_Sampling_ResetTimingState();
    
    return 0;
}

/**
 * @brief 处理ADC采样数据（在DMA中断中调用）
 */
void ADC_Sampling_Process(void)
{
    uint8_t frameValid = s_controlWindowOpen;
    uint32_t frameCycle = s_adcData.controlCycleSequence;

    /* 从全局ADC缓冲区读取数据
     * adc_data[0] = PA1 (INP17) - 电流A相
     * adc_data[1] = PA2 (INP14) - 电流B相
     * adc_data[2] = PA3 (INP15) - 电流C相
     * adc_data[3] = PC4 (INP4)  - 母线电压
     */
    s_adcData.rawCurrentA = adc_data[ADC_CH_CURRENT_A];
    s_adcData.rawCurrentB = adc_data[ADC_CH_CURRENT_B];
    s_adcData.rawCurrentC = adc_data[ADC_CH_CURRENT_C];
    s_adcData.rawVbus = adc_data[ADC_CH_VBUS];
    
    /* 计算物理量 */
    s_adcData.currentA = ADC_CalcCurrent(s_adcData.rawCurrentA, s_adcData.offsetA);
    s_adcData.currentB = ADC_CalcCurrent(s_adcData.rawCurrentB, s_adcData.offsetB);
    s_adcData.currentC = ADC_CalcCurrent(s_adcData.rawCurrentC, s_adcData.offsetC);
    s_adcData.vbus = ADC_CalcVoltage(s_adcData.rawVbus, K_VBUS_DIV);

    /* CH_CFG: C 通道补救 — recon 优先: 丢弃实测用 A+B 重构 (KCL);
     * 否则按需增益垫。默认 gain=1.0/recon=0 即原行为。 */
    if (s_adcData.recon_c != 0U) {
        s_adcData.currentC = -(s_adcData.currentA + s_adcData.currentB);
    } else if (s_adcData.gain_c != 1.0f) {
        s_adcData.currentC = s_adcData.currentC * s_adcData.gain_c;
    }
    
    /* 更新标志 */
    s_adcData.pwmPeriod = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
    s_adcData.pwmCompareA = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    s_adcData.pwmCompareB = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    s_adcData.pwmCompareC = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    s_adcData.triggerCompare = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    s_adcData.timerCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    s_adcData.timerCountingDown = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1) != RESET) ? 1U : 0U;
    s_adcData.lowSideValidA = (s_adcData.pwmCompareA <= s_adcData.triggerCompare) ? 1U : 0U;
    s_adcData.lowSideValidB = (s_adcData.pwmCompareB <= s_adcData.triggerCompare) ? 1U : 0U;
    s_adcData.lowSideValidC = (s_adcData.pwmCompareC <= s_adcData.triggerCompare) ? 1U : 0U;

    if (frameValid) {
        s_adcData.dataReady = 1U;
        s_adcData.lastCommittedCycle = frameCycle;
    } else {
        s_adcData.dataReady = 0U;
        s_adcData.invalidWindowCount++;
    }
    s_adcData.sampleCount++;
    s_adcData.frameSequence++;
}

/**
 * @brief 控制ISR进入时关闭当前采样窗口
 */
void ADC_Sampling_BeginControlCycle(void)
{
    s_controlWindowOpen = 0U;
    s_adcData.controlCycleSequence++;
}

/**
 * @brief 控制ISR退出时打开下一控制周期的采样窗口
 */
void ADC_Sampling_EndControlCycle(void)
{
    s_controlWindowOpen = 1U;
}

/**
 * @brief 重置控制周期相关的采样时序状态
 */
void ADC_Sampling_ResetTimingState(void)
{
    s_controlWindowOpen = 1U;
    s_lastConsumedFrameSequence = s_adcData.frameSequence;
    s_adcData.dataReady = 0U;
    s_adcData.controlCycleSequence = 0U;
    s_adcData.lastCommittedCycle = 0U;
    s_adcData.lastConsumedCycle = 0U;
    s_adcData.frameAgeCycles = 0U;
    s_adcData.sampleMissCount = 0U;
    s_adcData.invalidWindowCount = 0U;
}

/**
 * @brief 仅允许控制环消费本控制周期对应的新ADC帧
 */
uint8_t ADC_Sampling_TryConsumeLatest(void)
{
    uint32_t expectedCycle = (s_adcData.controlCycleSequence == 0U) ?
        0U : (s_adcData.controlCycleSequence - 1U);

    if (s_adcData.dataReady &&
        (s_adcData.frameSequence != s_lastConsumedFrameSequence) &&
        (s_adcData.lastCommittedCycle == expectedCycle)) {
        s_lastConsumedFrameSequence = s_adcData.frameSequence;
        s_adcData.lastConsumedCycle = expectedCycle;
        s_adcData.frameAgeCycles = 0U;
        s_adcData.sampleMissCount = 0U;
        s_adcData.dataReady = 0U;
        return 1U;
    }

    s_adcData.sampleMissCount++;
    if (s_adcData.controlCycleSequence > s_adcData.lastCommittedCycle) {
        s_adcData.frameAgeCycles =
            s_adcData.controlCycleSequence - s_adcData.lastCommittedCycle;
    } else {
        s_adcData.frameAgeCycles = 0U;
    }
    s_adcData.dataReady = 0U;

    return 0U;
}

/**
 * @brief 启动零点校准（在无电流时调用）
 * @param samples 校准采样次数
 * @return 0成功，-1超时失败
 * 
 * 【修复】添加超时保护机制，防止死循环
 * 超时时间：1秒（1000ms）
 * 超时后使用理论默认值(ADC_HALF)
 */
int8_t ADC_Sampling_Calibrate(uint16_t samples)
{
    int32_t sumA = 0, sumB = 0, sumC = 0;
    uint16_t count = 0;
    uint32_t startTick;
    const uint32_t timeoutMs = 1000;  /* 1秒超时 */
    
    /* 参数检查 */
    if (samples == 0) {
        return -1;
    }
    
    /* 获取起始时间 */
    startTick = HAL_GetTick();
    
    /* 等待足够的采样次数 */
    while (count < samples) {
        /* 超时检查 */
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            /* 超时：使用理论默认值 */
            s_adcData.offsetA = (int16_t)ADC_HALF;
            s_adcData.offsetB = (int16_t)ADC_HALF;
            s_adcData.offsetC = (int16_t)ADC_HALF;
            s_adcData.calibStatus = ADC_CALIB_TIMEOUT;
            return -1;  /* 返回错误 */
        }
        
        if (s_adcData.dataReady) {
            sumA += s_adcData.rawCurrentA;
            sumB += s_adcData.rawCurrentB;
            sumC += s_adcData.rawCurrentC;
            count++;
            s_adcData.dataReady = 0;
        }
    }
    
    /* 计算平均值作为零点偏移 */
    s_adcData.offsetA = (int16_t)(sumA / samples);
    s_adcData.offsetB = (int16_t)(sumB / samples);
    s_adcData.offsetC = (int16_t)(sumC / samples);
    s_adcData.calibStatus = ADC_CALIB_OK;
    
    return 0;
}

/**
 * @brief CH_CFG: 设置 C 通道补救 (增益垫 / 两相重构)
 */
void ADC_Sampling_SetChCfg(float gain_c, uint8_t recon_c)
{
    s_adcData.gain_c = gain_c;
    s_adcData.recon_c = recon_c;
}

/**
 * @brief CH_CFG: 查询 C 通道补救状态
 */
void ADC_Sampling_GetChCfg(float *gain_c, uint8_t *recon_c)
{
    if (gain_c != NULL) {
        *gain_c = s_adcData.gain_c;
    }
    if (recon_c != NULL) {
        *recon_c = s_adcData.recon_c;
    }
}

/**
 * @brief 获取采样数据指针
 */
ADC_Sampling_t* ADC_Sampling_GetData(void)
{
    return &s_adcData;
}

/**
 * @brief 获取母线电压
 * @return 母线电压值(V)
 */
float ADC_Sampling_GetVbus(void)
{
    return s_adcData.vbus;
}

/**
 * @brief 获取校准状态
 * @return 校准状态
 */
ADC_CalibStatus_t ADC_Sampling_GetCalibStatus(void)
{
    return s_adcData.calibStatus;
}

/**
 * @brief 检查三相电流不平衡度
 * @param threshold 不平衡阈值(A)
 * @return 0平衡，1不平衡
 * 
 * 【新增】三相电流和检查，用于检测采样异常
 */
uint8_t ADC_Sampling_CheckImbalance(float threshold)
{
    float sum = s_adcData.currentA + s_adcData.currentB + s_adcData.currentC;
    
    if (fabsf(sum) > threshold) {
        s_adcData.imbalanceCount++;
        return 1;  /* 不平衡 */
    }
    
    return 0;  /* 平衡 */
}

uint16_t ADC_Sampling_CaptureNoiseStats(uint16_t requestedSamples, ADC_Sampling_NoiseStats_t *stats)
{
    ADC_NoiseAccum_t accA;
    ADC_NoiseAccum_t accB;
    ADC_NoiseAccum_t accC;
    ADC_NoiseAccum_t accVbus;
    uint16_t target;
    uint16_t count = 0U;
    uint32_t startTick;
    uint32_t lastFrame;
    uint32_t timeoutMs;
    uint16_t rawA;
    uint16_t rawB;
    uint16_t rawC;
    uint16_t rawVbus;

    if (stats == NULL) {
        return 0U;
    }

    memset((void*)stats, 0, sizeof(ADC_Sampling_NoiseStats_t));

    target = requestedSamples;
    if (target < ADC_NOISE_MIN_SAMPLES) {
        target = ADC_NOISE_MIN_SAMPLES;
    }
    if (target > ADC_NOISE_MAX_SAMPLES) {
        target = ADC_NOISE_MAX_SAMPLES;
    }

    startTick = HAL_GetTick();
    lastFrame = s_adcData.frameSequence;
    timeoutMs = 250U + ((uint32_t)target / 4U);

    while (count < target) {
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            return 0U;
        }

        if (s_adcData.frameSequence == lastFrame) {
            continue;
        }
        lastFrame = s_adcData.frameSequence;

        rawA = s_adcData.rawCurrentA;
        rawB = s_adcData.rawCurrentB;
        rawC = s_adcData.rawCurrentC;
        rawVbus = s_adcData.rawVbus;

        if (count == 0U) {
            ADC_NoiseAccum_Init(&accA, rawA);
            ADC_NoiseAccum_Init(&accB, rawB);
            ADC_NoiseAccum_Init(&accC, rawC);
            ADC_NoiseAccum_Init(&accVbus, rawVbus);
        }

        ADC_NoiseAccum_Add(&accA, rawA);
        ADC_NoiseAccum_Add(&accB, rawB);
        ADC_NoiseAccum_Add(&accC, rawC);
        ADC_NoiseAccum_Add(&accVbus, rawVbus);
        count++;
    }

    stats->samples = count;
    ADC_NoiseAccum_Finish(&accA, count, &stats->currentA);
    ADC_NoiseAccum_Finish(&accB, count, &stats->currentB);
    ADC_NoiseAccum_Finish(&accC, count, &stats->currentC);
    ADC_NoiseAccum_Finish(&accVbus, count, &stats->vbus);

    return count;
}
