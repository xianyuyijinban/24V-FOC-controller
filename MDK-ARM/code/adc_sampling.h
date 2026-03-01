/**
 * @file    adc_sampling.h
 * @brief   ADC采样和计算模块 - 电流/电压采样
 * @note    使用ADC1 DMA多通道扫描模式
 * 
 * 【修复记录】
 * v1.1 - 添加校准状态和不平衡检测 (修复ADC-001)
 */

#ifndef __ADC_SAMPLING_H
#define __ADC_SAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*==================== 硬件参数定义 ====================*/

/* ADC参数 */
#define ADC_VREF            3.3f        /* ADC参考电压(V) */
#define ADC_MAX             4095.0f     /* 12位ADC满量程(2^12 - 1) */
#define ADC_HALF            2048.0f     /* VCC/2对应的ADC值 */

/* 电流采样电路参数（运放差分放大）
 * 电路结构：
 *   - 采样电阻：10mΩ，位于下桥臂源极与地之间
 *   - 运放增益：15倍 (Rf/Rin = 15kΩ/1kΩ)
 *   - 输出偏置：VCC/2 = 1.65V (双极性信号，可测正负电流)
 * 计算公式推导：
 *   Vout = 1.65V + (IC+ - IC-) × 15
 *   反推电流：I = (Vout - 1.65V) / 0.15 = (Vout - 1.65V) × 6.667 A/V
 */
#define R_SHUNT             0.01f       /* 采样电阻 10mΩ */
#define AMP_GAIN            15.0f       /* 运放差分增益 */
#define K_CURRENT           (1.0f / (R_SHUNT * AMP_GAIN))  /* 6.667 A/V */

/* 母线电压采样电路参数（电阻分压）
 * 电路结构：
 *   - R35 = 180kΩ (上分压，接VSENVM/+24V)
 *   - R26 = 20kΩ  (下分压，接地)
 * 分压比 = R26 / (R35 + R26) = 20k / 200k = 0.1
 * VM = VSENVM_采样 × 10
 */
#define K_VBUS_DIV          ((180.0f + 20.0f) / 20.0f)  /* 10.0 */

/* 电流不平衡检测阈值 */
#define CURRENT_IMBALANCE_THRESH    0.5f    /* 三相电流和阈值(A) */

/* ADC通道定义 */
#define ADC_CH_CURRENT_A    0   /* PA1 - ADC1_INP17 - 电流A相 */
#define ADC_CH_CURRENT_B    1   /* PA2 - ADC1_INP14 - 电流B相 */
#define ADC_CH_CURRENT_C    2   /* PA3 - ADC1_INP15 - 电流C相 */
#define ADC_CH_VBUS         3   /* PC4 - ADC1_INP4  - 母线电压 */

#define ADC_CHANNEL_COUNT   4   /* ADC通道数量 */

/*==================== 数据结构 ====================*/

/* 校准状态 */
typedef enum {
    ADC_CALIB_NONE = 0,     /* 未校准 */
    ADC_CALIB_OK,           /* 校准成功 */
    ADC_CALIB_TIMEOUT,      /* 校准超时 */
    ADC_CALIB_ERROR         /* 校准错误 */
} ADC_CalibStatus_t;

/* ADC采样数据结构 */
typedef struct {
    /* 原始ADC值 */
    volatile uint16_t rawCurrentA;      /* A相电流原始值 */
    volatile uint16_t rawCurrentB;      /* B相电流原始值 */
    volatile uint16_t rawCurrentC;      /* C相电流原始值 */
    volatile uint16_t rawVbus;          /* 母线电压原始值 */
    
    /* 计算后的物理量 */
    volatile float currentA;            /* A相电流 (A) */
    volatile float currentB;            /* B相电流 (A) */
    volatile float currentC;            /* C相电流 (A) */
    volatile float vbus;                /* 母线电压 (V) */
    
    /* 零点校准值 */
    int16_t offsetA;                    /* A相零点偏移 */
    int16_t offsetB;                    /* B相零点偏移 */
    int16_t offsetC;                    /* C相零点偏移 */
    
    /* 状态标志 */
    volatile uint8_t dataReady;         /* 数据更新标志 */
    volatile uint32_t sampleCount;      /* 采样计数 */
    
    /* 【新增】校准状态 */
    ADC_CalibStatus_t calibStatus;      /* 校准状态 */
    uint16_t imbalanceCount;            /* 不平衡计数 */
} ADC_Sampling_t;

/*==================== 函数声明 ====================*/

/**
 * @brief 初始化ADC采样模块
 * @param hadc ADC句柄
 * @return 0成功，-1失败
 */
int8_t ADC_Sampling_Init(ADC_HandleTypeDef* hadc);

/**
 * @brief 处理ADC采样数据（在DMA中断中调用）
 */
void ADC_Sampling_Process(void);

/**
 * @brief 启动零点校准（在无电流时调用）
 * @param samples 校准采样次数
 * @return 0成功，-1超时失败
 * 
 * 【修复】添加1秒超时保护，防止死循环
 */
int8_t ADC_Sampling_Calibrate(uint16_t samples);

/**
 * @brief 获取采样数据指针
 * @return 采样数据结构体指针
 */
ADC_Sampling_t* ADC_Sampling_GetData(void);

/**
 * @brief 获取母线电压
 * @return 母线电压值(V)
 */
float ADC_Sampling_GetVbus(void);

/**
 * @brief 获取校准状态
 * @return 校准状态
 */
ADC_CalibStatus_t ADC_Sampling_GetCalibStatus(void);

/**
 * @brief 检查三相电流不平衡度
 * @param threshold 不平衡阈值(A)
 * @return 0平衡，1不平衡
 */
uint8_t ADC_Sampling_CheckImbalance(float threshold);

/**
 * @brief 计算电流（双极性，带VCC/2偏置）
 * @param raw ADC原始值 (0~4095)
 * @param offset 零点偏移校准值 (理论上≈2048)
 * @return 电流值(A)，正为流出，负为流入
 */
static inline float ADC_CalcCurrent(uint16_t raw, int16_t offset)
{
    int16_t adc_centered = (int16_t)raw - offset;
    return ((float)adc_centered) * ADC_VREF / ADC_MAX * K_CURRENT;
}

/**
 * @brief 计算电压（单极性，无偏置）
 * @param raw ADC原始值 (0~4095)
 * @param divider 分压比系数
 * @return 电压值(V)
 */
static inline float ADC_CalcVoltage(uint16_t raw, float divider)
{
    return ((float)raw) * ADC_VREF / ADC_MAX * divider;
}

#ifdef __cplusplus
}
#endif

#endif /* __ADC_SAMPLING_H */
