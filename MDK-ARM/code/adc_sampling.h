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
#define ADC_MAX             4096.0f     /* 12位ADC码宽(2^12)，按满量程码宽换算 */
#define ADC_HALF            2048.0f     /* VCC/2对应的ADC值 */

/* 电流采样电路参数（运放差分放大）
 * 电路结构：
 *   - 采样电阻：10mΩ，位于下桥臂源极与地之间
 *   - 运放增益：15倍 (Rf/Rin = 15kΩ/1kΩ)
 *   - 输出偏置：VCC/2 = 1.65V (双极性信号，可测正负电流)
 * 计算公式推导：
 *   先按ADC码计算幅值，再通过 ADC_CURRENT_POLARITY 统一修正符号。
 *   Bench torque sign test requires +Iq_ref to produce the expected mechanical direction.
 */
#define R_SHUNT             0.01f       /* 采样电阻 10mΩ */
#define AMP_GAIN            15.0f       /* 运放差分增益 */
#define K_CURRENT           (1.0f / (R_SHUNT * AMP_GAIN))  /* 6.667 A/V */
#define ADC_CURRENT_POLARITY (-1.0f)    /* Low-side shunt inverts sense polarity; -1.0 restores correct FOC current sign. */

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

/* 采样时序配置 */
#define ADC_SAMPLING_TRIGGER_SOURCE_TEXT         "TIM1_TRGO2_OC4REF_FALLING"
#define ADC_SAMPLING_CURRENT_SAMPLE_TIME_CYCLES  32.5f
#define ADC_SAMPLING_VBUS_SAMPLE_TIME_CYCLES     16.5f

/* ADC通道定义 */
#define ADC_CH_CURRENT_A    0   /* PA1 - ADC1_INP17 - 电流A相 */
#define ADC_CH_CURRENT_B    1   /* PA2 - ADC1_INP14 - 电流B相 */
#define ADC_CH_CURRENT_C    2   /* PA3 - ADC1_INP15 - 电流C相 */
#define ADC_CH_VBUS         3   /* PC4 - ADC1_INP4  - 母线电压 */

#define ADC_CHANNEL_COUNT   4   /* ADC通道数量 */

/* ADC噪声诊断采样限制：只输出统计值，不通过UART刷原始样本流 */
#define ADC_NOISE_MIN_SAMPLES  16U
#define ADC_NOISE_MAX_SAMPLES  4096U

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
    volatile uint32_t controlCycleSequence; /* 当前控制周期序号 */
    volatile uint32_t frameSequence;    /* 已提交ADC帧序号 */
    volatile uint32_t lastCommittedCycle; /* 最近一次有效提交对应的控制周期 */
    volatile uint32_t lastConsumedCycle; /* 最近一次成功消费对应的控制周期 */
    volatile uint32_t frameAgeCycles;   /* 当前数据相对消费侧的年龄(控制周期) */
    volatile uint32_t sampleMissCount;  /* 控制周期未拿到新帧的次数 */
    volatile uint32_t invalidWindowCount; /* 帧在关闭窗口到达的次数 */

    /* PWM/采样窗口诊断（在ADC DMA完成瞬间抓取） */
    volatile uint16_t pwmPeriod;        /* TIM1 ARR */
    volatile uint16_t pwmCompareA;      /* TIM1 CCR1 */
    volatile uint16_t pwmCompareB;      /* TIM1 CCR2 */
    volatile uint16_t pwmCompareC;      /* TIM1 CCR3 */
    volatile uint16_t triggerCompare;   /* TIM1 CCR4 / ADC触发位置 */
    volatile uint16_t timerCount;       /* ADC DMA完成时TIM1 CNT */
    volatile uint8_t timerCountingDown; /* ADC DMA完成时TIM1计数方向 */
    volatile uint8_t lowSideValidA;     /* A相在该触发位置是否满足低边采样窗口 */
    volatile uint8_t lowSideValidB;     /* B相在该触发位置是否满足低边采样窗口 */
    volatile uint8_t lowSideValidC;     /* C相在该触发位置是否满足低边采样窗口 */
    
    /* 【新增】校准状态 */
    ADC_CalibStatus_t calibStatus;      /* 校准状态 */
    uint16_t imbalanceCount;            /* 不平衡计数 */
} ADC_Sampling_t;

typedef struct {
    uint16_t min;          /* 原始ADC最小值 */
    uint16_t max;          /* 原始ADC最大值 */
    uint16_t mean;         /* 原始ADC均值 */
    uint16_t peakToPeak;   /* 原始ADC峰峰值 */
    uint16_t stddev;       /* 原始ADC标准差 */
} ADC_Sampling_NoiseChannelStats_t;

typedef struct {
    uint16_t samples;      /* 实际统计样本数 */
    ADC_Sampling_NoiseChannelStats_t currentA;
    ADC_Sampling_NoiseChannelStats_t currentB;
    ADC_Sampling_NoiseChannelStats_t currentC;
    ADC_Sampling_NoiseChannelStats_t vbus;
} ADC_Sampling_NoiseStats_t;

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
 * @brief 关闭当前采样窗口并进入新的控制周期
 */
void ADC_Sampling_BeginControlCycle(void);

/**
 * @brief 打开下一控制周期的采样窗口
 */
void ADC_Sampling_EndControlCycle(void);

/**
 * @brief 重置控制周期相关的采样时序状态
 */
void ADC_Sampling_ResetTimingState(void);

/**
 * @brief 尝试消费当前控制周期对应的最新ADC帧
 * @return 1=成功拿到当前周期新帧，0=缺帧/晚帧
 */
uint8_t ADC_Sampling_TryConsumeLatest(void);

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
 * @brief 捕获ADC原始码噪声统计（仅用于未驱动状态下台架诊断）
 * @param requestedSamples 请求样本数，函数内部限制到16~4096
 * @param stats 统计结果
 * @return 实际统计样本数，0表示超时或参数错误
 */
uint16_t ADC_Sampling_CaptureNoiseStats(uint16_t requestedSamples, ADC_Sampling_NoiseStats_t *stats);

/**
 * @brief 计算电流（双极性，带VCC/2偏置）
 * @param raw ADC原始值 (0~4095)
 * @param offset 零点偏移校准值 (理论上≈2048)
 * @return 电流值(A)，正为流出，负为流入
 */
static inline float ADC_CalcCurrent(uint16_t raw, int16_t offset)
{
    int16_t adc_centered = (int16_t)raw - offset;
    return ADC_CURRENT_POLARITY * ((float)adc_centered) * ADC_VREF / ADC_MAX * K_CURRENT;
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
