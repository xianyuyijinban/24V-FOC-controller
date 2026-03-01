/**
 * @file    adc_sampling.c
 * @brief   ADC采样和计算模块实现
 * 
 * 【修复记录】
 * v1.1 - 添加ADC校准超时保护，防止死循环 (修复ADC-001)
 */

#include "adc_sampling.h"
#include <string.h>

/* 私有变量 */
static ADC_HandleTypeDef* s_hadc = NULL;
static ADC_Sampling_t s_adcData;

/* 外部变量声明（在stm32h7xx_it.c中定义） */
extern volatile uint16_t adc_data[8];

/**
 * @brief 初始化ADC采样模块
 */
int8_t ADC_Sampling_Init(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL) {
        return -1;
    }
    
    s_hadc = hadc;
    
    /* 清零数据结构 */
    memset((void*)&s_adcData, 0, sizeof(ADC_Sampling_t));
    
    /* 设置默认零点偏移（理论值为VCC/2对应的ADC值） */
    s_adcData.offsetA = (int16_t)ADC_HALF;
    s_adcData.offsetB = (int16_t)ADC_HALF;
    s_adcData.offsetC = (int16_t)ADC_HALF;
    
    return 0;
}

/**
 * @brief 处理ADC采样数据（在DMA中断中调用）
 */
void ADC_Sampling_Process(void)
{
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
    
    /* 更新标志 */
    s_adcData.dataReady = 1;
    s_adcData.sampleCount++;
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
