/**
 * @file    foc_dtcomp.h
 * @brief   逆变器死区补偿模块（电压环 FOC 评估用，方案书 §5.3）
 * @note    Vdt_corr = Vbus × (tdead + ton − toff) × fs × sign(相电流)
 *          逐相补偿，挂在 SVPWM 之前；默认关闭保护基线
 */

#ifndef __FOC_DTCOMP_H
#define __FOC_DTCOMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 死区补偿参数 */
#define FOC_DTCOMP_DEFAULT_AMPLITUDE_V   0.15f  /* 默认补偿幅值 V（Vbus×Tdt×fs 典型值，E7 实测校正） */
#define FOC_DTCOMP_SIGN_HYST_A           0.05f  /* 相电流符号滞环 A（防零流抖动） */

/* 死区补偿状态 */
typedef struct {
    uint8_t enabled;         /* 1 = 补偿生效（默认 0） */
    float   amplitude_v;     /* 补偿幅值 V（运行时 CMD:DT_V 调） */
    float   comp_a;          /* A相最近一次补偿量 V（诊断） */
    float   comp_b;          /* B相补偿量 V */
    float   comp_c;          /* C相补偿量 V */
    /* 相电流符号滞环状态：0=未锁 1=正 2=负 */
    uint8_t sign_a;
    uint8_t sign_b;
    uint8_t sign_c;
} FOC_DtComp_t;

/**
 * @brief 初始化死区补偿（enabled=0, amplitude=default）
 */
void FOC_DtComp_Init(FOC_DtComp_t *dt);

/**
 * @brief 计算三相死区补偿电压（在 SVPWM 前叠加到相电压指令）
 * @param dt      补偿状态
 * @param ia/ib/ic 三相电流（用于符号判断；估计电流也可）
 * @param vabc    [in/out] 三相相电压指令 V，函数内叠加补偿
 * @note  电流绝对值小于滞环时不补偿（保持上一次符号为 0 → 幅值 0）
 */
void FOC_DtComp_Apply(FOC_DtComp_t *dt, float ia, float ib, float ic, float vabc[3]);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DTCOMP_H */
