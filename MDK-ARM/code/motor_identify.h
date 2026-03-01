/**
 * @file    motor_identify.h
 * @brief   电机参数自动识别模块
 * @note    上电自动离线识别 + 运行中Rs在线补偿
 */

#ifndef __MOTOR_IDENTIFY_H
#define __MOTOR_IDENTIFY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "foc_core.h"

/*==================== 配置参数 ====================*/

/* Rs识别参数 */
#define MI_RS_TEST_VOLTAGE      0.05f       /* 测试电压 = 5% Vbus */
#define MI_RS_TEST_DURATION     100         /* 测试时间 ms */
#define MI_RS_SETTLE_TIME       50          /* 稳定时间 ms */
#define MI_RS_CONVERGE_THRESH   0.05f       /* 收敛阈值 0.05Ω */
#define MI_RS_CURRENT_THRESH    0.5f        /* 最小测试电流 A */

/* Ls识别参数 */
#define MI_LS_INJ_FREQUENCY     1000.0f     /* 注入频率 Hz */
#define MI_LS_INJ_AMPLITUDE     0.1f        /* 注入幅值 = 10% Vbus */
#define MI_LS_TEST_DURATION     200         /* 测试时间 ms */
#define MI_LS_CONVERGE_THRESH   0.0001f     /* 收敛阈值 0.1mH */

/* Ke识别参数 */
#define MI_KE_TEST_SPEED_RPM    500.0f      /* 测试转速 rpm */
#define MI_KE_RAMP_TIME         500         /* 加速时间 ms */
#define MI_KE_MEASURE_TIME      100         /* 测量时间 ms */

/* 极对数识别 */
#define MI_PN_TEST_CURRENT      1.0f        /* 测试电流 A */
#define MI_PN_TEST_DURATION     300         /* 测试时间 ms */

/* 转动惯量识别 */
#define MI_J_TEST_SPEED_RPM     300.0f      /* 目标转速 rpm */
#define MI_J_RAMP_TIME          300         /* 加速时间 ms */

/* 机械锁止检测 */
#define MI_THETA_LIMIT_DEG      2.0f        /* 角度变化限制 ±2° */
#define MI_THETA_CHECK_TIME     50          /* 检测时间 ms */

/* 错误代码 */
typedef enum {
    MI_ERR_NONE = 0,
    MI_ERR_IN_PROGRESS,         /* 识别进行中（非错误） */
    MI_ERR_MOTOR_MOVING,        /* 电机移动（机械锁止失败） */
    MI_ERR_RS_NOT_CONVERGED,    /* Rs未收敛 */
    MI_ERR_LS_NOT_CONVERGED,    /* Ls未收敛 */
    MI_ERR_KE_NOT_CONVERGED,    /* Ke未收敛 */
    MI_ERR_PN_NOT_CONVERGED,    /* 极对数识别失败 */
    MI_ERR_J_NOT_CONVERGED,     /* J未收敛 */
    MI_ERR_CURRENT_TOO_LOW,     /* 测试电流太小 */
    MI_ERR_CURRENT_TOO_HIGH,    /* 测试电流太大 */
    MI_ERR_TIMEOUT,             /* 超时 */
} MI_ErrorCode_t;

/*==================== 数据结构 ====================*/

/* 电机参数结构体 */
typedef struct {
    float Rs;               /* 定子电阻 Ω */
    float Ld;               /* d轴电感 H */
    float Lq;               /* q轴电感 H */
    float Ke;               /* 反电动势常数 V/(rad/s) */
    uint8_t Pn;             /* 极对数 */
    float J;                /* 转动惯量 kg·m² */
    float B;                /* 摩擦系数 N·m·s/rad */
    float theta_offset;     /* 编码器零位偏移 rad */
    
    /* 参数有效性标志 */
    uint32_t valid_flag;
} MotorParam_t;

/* Rs在线估计器 */
typedef struct {
    float Rs_estimated;     /* 估计的Rs */
    float alpha;            /* 滤波系数 */
    float voltage_accum;    /* 电压累加 */
    float current_accum;    /* 电流累加 */
    uint32_t sample_count;  /* 采样计数 */
    uint8_t enabled;        /* 使能标志 */
} RsOnlineEstimator_t;

/* 识别状态机状态 */
typedef enum {
    MI_STATE_IDLE = 0,
    MI_STATE_PN_IDENTIFY,       /* 极对数识别 */
    MI_STATE_RS_IDENTIFY,       /* 电阻识别 */
    MI_STATE_LS_IDENTIFY,       /* 电感识别 */
    MI_STATE_KE_IDENTIFY,       /* 反电动势识别 */
    MI_STATE_J_IDENTIFY,        /* 惯量识别 */
    MI_STATE_ENCODER_ALIGN,     /* 编码器对齐 */
    MI_STATE_COMPLETE,          /* 完成 */
    MI_STATE_ERROR              /* 错误 */
} MI_State_t;

/* 识别控制结构体 */
typedef struct {
    MI_State_t state;           /* 当前状态 */
    MI_ErrorCode_t error_code;  /* 错误代码 */
    
    MotorParam_t *param;        /* 电机参数指针 */
    FOC_Handle_t *foc;          /* FOC句柄指针 */
    
    /* 识别过程数据 */
    float theta_start;          /* 起始角度 */
    float theta_max;            /* 最大角度 */
    float theta_min;            /* 最小角度 */
    uint32_t state_start_time;  /* 状态开始时间 */
    uint32_t sample_count;      /* 采样计数 */
    
    /* 中间计算结果 */
    float sum_v;                /* 电压累加 */
    float sum_i;                /* 电流累加 */
    float sum_ii;               /* 电流平方累加 */
    float sum_vi;               /* 电压电流乘积累加 */
    
    /* Rs识别专用 */
    float Rs_positive;          /* 正极性Rs */
    float Rs_negative;          /* 负极性Rs */
    uint8_t polarity;           /* 当前极性 */
    
    /* Ke识别状态 */
    uint8_t ke_state;           /* Ke识别状态机状态 */
    float speed_elec;           /* 电转速缓存 */
    
    /* 【修复MI-001】Pn识别状态（从静态变量移至此处） */
    uint8_t pn_state;           /* Pn识别状态 */
    float pn_theta_start;       /* Pn识别起始机械角度 */
    float pn_theta_accum;       /* Pn识别角度累加 */
    uint32_t pn_elec_cycles;    /* Pn识别电周期计数 */
    float pn_theta_last;        /* Pn识别上次角度 */
    float pn_elec_last;         /* Pn识别上次电角度 */
    
    /* 进度回调 */
    void (*progress_callback)(uint8_t percent, const char *step_name);
    
} MI_Handle_t;

/*==================== 函数声明 ====================*/

/* 主状态机 */
void MI_Init(MI_Handle_t *handle, MotorParam_t *param, FOC_Handle_t *foc);
void MI_StartIdentify(MI_Handle_t *handle);
void MI_Process(MI_Handle_t *handle);  /* 在1ms中断中调用 */
uint8_t MI_IsComplete(MI_Handle_t *handle);
MI_ErrorCode_t MI_GetError(MI_Handle_t *handle);
const char* MI_GetErrorString(MI_ErrorCode_t error);

/* 各参数识别步骤 */
MI_ErrorCode_t MI_IdentifyPn(MI_Handle_t *handle);
MI_ErrorCode_t MI_IdentifyRs(MI_Handle_t *handle);
MI_ErrorCode_t MI_IdentifyLs(MI_Handle_t *handle);
MI_ErrorCode_t MI_IdentifyKe(MI_Handle_t *handle);
MI_ErrorCode_t MI_IdentifyJ(MI_Handle_t *handle);
MI_ErrorCode_t MI_EncoderAlign(MI_Handle_t *handle);

/* Rs在线估计 */
void MI_RsOnlineEstimator_Init(RsOnlineEstimator_t *est, float alpha);
void MI_RsOnlineEstimator_Enable(RsOnlineEstimator_t *est, uint8_t enable);
void MI_RsOnlineEstimator_Update(RsOnlineEstimator_t *est, float Vd, float Vq, float Id, float Iq, float omega_e);
float MI_RsOnlineEstimator_GetRs(RsOnlineEstimator_t *est);

/* 辅助函数 */
uint8_t MI_CheckMechanicalLock(MI_Handle_t *handle);
void MI_UpdatePIWithNewRs(FOC_Handle_t *foc, float Rs_new);
float MI_GetElapsedTime(MI_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_IDENTIFY_H */
