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

/* Rs识别参数 — d轴闭环锁轴法 */
#define MI_RS_LOCK_CURRENT_INITIAL 0.15f /* d轴锁轴起始电流 A，24V下PI增益高需保守起步 */
#define MI_RS_LOCK_CURRENT_STEP    0.1f  /* 电流不足时步进 A */
#define MI_RS_LOCK_CURRENT_MAX     1.0f  /* 最大锁轴电流 A */
#define MI_RS_LOCK_DURATION        150   /* 锁轴等待时间 ms */
#define MI_RS_SAMPLE_DURATION      50    /* 采样时间 ms */
#define MI_RS_CONVERGE_THRESH   0.5f        /* 绝对收敛阈值 0.5Ω */
#define MI_RS_CONVERGE_REL_THRESH 0.5f      /* 正负半周期允许50%相对差 */
#define MI_RS_CURRENT_THRESH    0.15f       /* DC电流绝对值有效阈值 */
#define MI_RS_CURRENT_MAX       10.0f        /* 旁路RS过流检查（ADC偏移导致假报警） */

/* Ls识别参数 */
#define MI_LS_INJ_FREQUENCY     1000.0f     /* 注入频率 Hz */
#define MI_LS_INJ_AMPLITUDE     0.2f        /* 注入幅值 = 20% Vbus，24V高电阻台架下先把Irms拉离噪声底 */
#define MI_LS_INJ_VOLTAGE_MAX_V 2.4f        /* 识别注入绝对电压上限，保持24V台架不超过12V已验证幅值 */
#define MI_LS_TEST_DURATION     200         /* 测试时间 ms */
#define MI_LS_CONVERGE_THRESH   0.0001f     /* 收敛阈值 0.1mH */
#define MI_LS_VALID_MAX_H       0.01f       /* 必须与Param_IsValid的Ld/Lq上限一致 */
#define MI_LS_FALLBACK_DEFAULT  0.0005f     /* Ls未可靠收敛时使用0.5mH保守默认值 */

/* Ke识别参数 — ADC触发修复后降速防DRV过流 */
#define MI_KE_TEST_SPEED_RPM    100.0f      /* 测试转速 rpm */
#define MI_KE_RAMP_TIME         1000        /* 加速时间 ms */
#define MI_KE_MEASURE_TIME      100         /* 测量时间 ms */

/* 极对数方向验证：Pn由上位机/Flash配置，这里只验证编码器方向与真实运动 */
#define MI_PN_ALIGN_CURRENT     0.1f        /* 24V台架锁轴电流再降档，尽量避免PN验证在进入电压步进前先触发DRV保护 */
#define MI_PN_ALIGN_DURATION    100         /* 缩短锁轴保持时间，减少高侧GDUV/VGS锁存窗口 */
#define MI_PN_TEST_CURRENT_INITIAL 0.25f    /* 24V台架更低起步电流，避免VDS过流 */
#define MI_PN_TEST_CURRENT_STEP 0.05f       /* 更小步进 */
#define MI_PN_TEST_CURRENT_MAX  0.5f        /* 24V台架降低上限，1.2A已验证触发VDS_HA */
#define MI_PN_STEP_ELEC_DEG     5.0f        /* Microstep 5 electrical degrees. */
#define MI_PN_STEP_SETTLE_MS    15          /* Microstep settle time. */
#define MI_PN_STEP_COUNT        144         /* 144 * 5deg = 720 electrical degrees. */
#define MI_PN_TEST_VOLTAGE_RATIO 0.15f      /* ADC触发修复后真实电流在走，降电压防VDS */
#define MI_PN_TEST_VOLTAGE_MAX_V 1.8f       /* 12V安全上限，ADC触发修复前高值因无电流未暴露 */
#define MI_PN_AUTO_UPDATE_ENCODER_DIR 0     /* PN direction is diagnostic; keep configured encoder_dir. */
#define MI_PN_MIN_MECH_DELTA_RAD 0.15f      /* Minimum accumulated mechanical travel for PN diagnostics. */

/* 转动惯量识别 — 恒电流加速+滑行法 */
#define MI_J_ACCEL_IQ_A             0.45f   /* 加速电流 A (V4回调: 已验证0.60/1.0可行) */
#define MI_J_ACCEL_SETTLE_MS        100     /* 加速前稳定时间 ms */
#define MI_J_ACCEL_TIMEOUT_MS       5000    /* 加速超时 ms */
#define MI_J_ACCEL_SPEED_LOW_RADPS  1.0f    /* 测量窗下限 rad/s */
#define MI_J_ACCEL_SPEED_HIGH_RADPS 4.0f    /* 测量窗上限 rad/s */
#define MI_J_COAST_TIMEOUT_MS       6000    /* 滑行超时 ms */
#define MI_J_VALID_MIN              1e-7f   /* J最小有效值 kg·m² */
#define MI_J_VALID_MAX              0.1f    /* J最大有效值 kg·m² */
#define MI_B_VALID_MIN              0.0f    /* B最小有效值 */
#define MI_B_VALID_MAX              0.1f    /* B最大有效值 */

/* 机械锁止检测 */
#define MI_THETA_LIMIT_DEG      2.0f        /* 角度变化限制 ±2° */
#define MI_THETA_CHECK_TIME     50          /* 检测时间 ms */

/* 编码器零位对齐 */
#define MI_ALIGN_CURRENT        0.8f        /* 锁轴d轴电流 A */
#define MI_ALIGN_DURATION       200         /* 锁轴时长 ms */

/* 识别完成前单向弱运动认证：仅验证编码器方向无严重错误，不强制高齿槽电机完成完整拖动 */
#define MI_VERIFY_VOLTAGE_RATIO MI_PN_TEST_VOLTAGE_RATIO /* Use PN-proven direct-voltage envelope. */
#define MI_VERIFY_VOLTAGE_MAX_V MI_PN_TEST_VOLTAGE_MAX_V /* Keep verify no stronger than PN microstep drive. */
#define MI_VERIFY_MECH_FREQ_HZ  0.10f       /* Faster drag helps high-cogging motors cross cogging steps. */
#define MI_VERIFY_MIN_MECH_RAD  0.30f       /* 最小可信位移约17°，达到即强通过 */
#define MI_VERIFY_DIR_LOCK_RAD  0.15f       /* 累计约8.6°后锁定实测方向 */
#define MI_VERIFY_REVERSE_FAULT_RAD 0.30f   /* 反向累计超过约17°视为相序/方向异常 */
#define MI_VERIFY_PHASE_TIMEOUT_MS 60000    /* 单向超时60s */
#define MI_VERIFY_NO_MOTION_TIMEOUT_MS 10000 /* Longer startup window for high-cogging motors. */
#define MI_VERIFY_NO_MOTION_MIN_RAD 0.03f   /* 无运动判据：原始累计<0.03rad */

/* 齿槽转矩LUT识别 */
#define MI_COGGING_DRAG_SPEED_MECH_RADPS 0.50f  /* 开环拖动机械角速度 rad/s */
#define MI_COGGING_DRAG_VOLTAGE_RATIO    0.08f   /* 拖动电压/Vbus比例 */
#define MI_COGGING_DRAG_VOLTAGE_MAX_V    1.5f    /* 拖动电压上限 V */
#define MI_COGGING_SETTLE_MS             1500    /* 启动稳定时间 ms */
#define MI_COGGING_RECORD_TIMEOUT_MS     30000   /* 记录超时 (约6转) 30s */

/* 编码器识别期容错：单个TLE帧瞬态无效只等待，连续无效才终止识别 */
#define MI_ENCODER_INVALID_CONSECUTIVE_LIMIT 20U

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
    MI_ERR_ENCODER_INVALID,     /* 编码器反馈无效 */
    MI_ERR_TIMEOUT,             /* 超时 */
    MI_ERR_PHASE_SEQUENCE,       /* 相序或编码器方向异常 */
} MI_ErrorCode_t;

/*==================== 数据结构 ====================*/

/* 电机参数结构体 */
typedef struct {
    float Rs;               /* 定子电阻 Ω */
    float Ld;               /* d轴电感 H */
    float Lq;               /* q轴电感 H */
    float Ke;               /* 反电动势常数 V/(rad/s) */
    uint8_t Pn;             /* 极对数 */
    int8_t encoder_dir;     /* 编码器方向：+1=机械角与正电角同向，-1=反向 */
    float J;                /* 转动惯量 kg·m² */
    float B;                /* 摩擦系数 N·m·s/rad */
    float Tc;               /* 库仑摩擦转矩 N·m */
    float theta_offset;     /* 编码器零位偏移 rad */
    float theta_mech_zero;  /* 识别/对齐得到的机械零位 rad */
    float mech_zero_offset; /* 用户设定机械零位偏置 rad */

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
    MI_STATE_PN_IDENTIFY,       /* 极对数验证：Pn由上位机/Flash配置 */
    MI_STATE_RS_IDENTIFY,       /* 电阻识别 */
    MI_STATE_LS_IDENTIFY,       /* 电感识别 */
    MI_STATE_KE_IDENTIFY,       /* 反电动势识别 */
    MI_STATE_J_IDENTIFY,        /* 惯量识别 */
    MI_STATE_ENCODER_ALIGN,     /* 编码器对齐 */
    MI_STATE_MOTION_VERIFY,     /* 双向真实运动认证 */
    MI_STATE_COGGING_IDENTIFY,  /* 齿槽转矩LUT识别 */
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
    uint8_t encoder_invalid_count; /* 识别期连续编码器无效计数 */
    
    /* 中间计算结果 */
    float sum_v;                /* 电压累加 */
    float sum_vq;               /* q轴电压累加 */
    float sum_i;                /* 电流累加 */
    float sum_ia;               /* A相电流累加 */
    float sum_ib;               /* B相电流累加 */
    float sum_ic;               /* C相电流累加 */
    float sum_ialpha;           /* Ialpha累加 */
    float sum_ibeta;            /* Ibeta累加 */
    float sum_i_mag;            /* dq电流幅值累加，用于识别阶段轴向错配诊断 */
    float sum_ii;               /* 电流平方累加 */
    float sum_vi;               /* 电压电流乘积累加 */
    
    /* Rs识别专用 */
    float Rs_positive;          /* 正极性Rs */
    float Rs_negative;          /* 负极性Rs */
    float rs_current_target;    /* d轴锁轴电流目标 A */
    float rs_last_v_avg;        /* 最近一次Rs窗口平均d轴电压 */
    float rs_last_i_avg;        /* 最近一次Rs窗口平均d轴电流 */
    float rs_last_i_mag_avg;    /* 最近一次Rs窗口平均dq电流幅值 */
    float rs_last_vec_rs;       /* 最近一次Rs窗口矢量投影电阻 */
    uint32_t rs_last_samples;   /* 最近一次Rs窗口采样数 */
    uint8_t polarity;           /* 当前极性 */

    /* Ls识别专用 */
    float ls_last_v_rms;        /* 最近一次Ls识别注入电压RMS */
    float ls_last_i_rms;        /* 最近一次Ls识别电流RMS */
    float ls_last_z;            /* 最近一次Ls识别阻抗幅值 */
    float ls_last_xl;           /* 最近一次Ls识别感抗 */
    float ls_last_l;            /* 最近一次Ls识别计算电感 */
    uint8_t ls_used_fallback;   /* Ls是否回退到默认值 */
    
    /* Ke识别状态 */
    uint8_t ke_state;           /* Ke识别状态机状态 */
    float speed_elec;           /* 电转速缓存 */
    float ke_theta_prev;        /* 上次机械角(Ke识别) */
    float ke_speed_filt;        /* 滤波后的电角速度(Ke识别) */
    uint8_t ke_speed_ready;     /* Ke测速初始化标志 */
    
    /* 【修复MI-001】Pn识别状态（从静态变量移至此处） */
    uint8_t pn_state;           /* Pn识别状态 */
    float pn_theta_start;       /* Pn识别起始机械角度 */
    float pn_theta_accum;       /* Pn识别角度累加 */
    uint32_t pn_elec_cycles;    /* Pn步进计数 */
    float pn_theta_last;        /* Pn识别上次角度 */
    float pn_elec_last;         /* Pn识别上次电角度 */
    float pn_current_target;    /* 当前Pn开环拖动电流目标 */
    float pn_last_delta_mech;   /* 最近一次Pn机械角累计变化 rad */
    float pn_last_delta_elec;   /* 最近一次Pn电角指令变化 rad */
    float pn_last_calc;         /* 最近一次Pn计算值 */
    int8_t pn_observed_dir;     /* 正电角拖动时实测机械方向：+1/-1 */

    /* 识别完成前单向弱运动认证 */
    uint8_t verify_phase;       /* 0=初始化, 1=单向拖动验证 */
    float verify_theta_last;    /* 上次认证机械角 rad */
    float verify_theta_accum;   /* 锁定方向后累计机械角 rad */
    float verify_elec_cmd;      /* 当前认证电角指令 rad */
    float verify_raw_accum;     /* 原始累计（锁定方向前使用） */
    int8_t verify_locked_dir;   /* 锁定后的实测机械方向：+1/-1, 0=未锁定 */
    int8_t verify_expected_dir; /* 期望机械方向：来自pn_observed_dir或电角命令方向 */
    uint8_t motion_verify_weak; /* 单向弱通过标志 */
    uint8_t motion_verify_status; /* 0=not_run, 1=strong_pass, 2=weak_pass, 3=failed */
    uint8_t verify_reverse_fault; /* 反向运动异常标志：1=曾检测到显著反向 */
    
    /* J/B识别专用 */
    uint8_t j_state;            /* J识别子状态: 0=INIT, 1=ACCEL, 2=COAST, 3=COMPLETE */
    float j_accel_iq_sum;       /* Iq累加 (用于平均) */
    float j_accel_iq_count;     /* 采样计数 */
    float j_accel_v_start;      /* 测量窗口起始速度 rad/s */
    float j_accel_v_end;        /* 测量窗口结束速度 rad/s */
    uint32_t j_accel_t_start;   /* 测量窗口起始时间 ms */
    uint32_t j_accel_t_end;     /* 测量窗口结束时间 ms */
    uint32_t j_accel_cycle_start; /* 测量窗口起始控制周期计数 */
    float j_coast_v_start;      /* 滑行起始速度 rad/s */
    float j_coast_v_end;        /* 滑行结束速度 rad/s */
    uint32_t j_coast_t_start;   /* 滑行起始时间 ms */
    uint32_t j_coast_t_end;     /* 滑行结束时间 ms */
    float j_speed_mech;         /* J识别期间缓存机械转速 */
    float j_theta_prev;         /* 上一拍机械角 rad */
    uint8_t j_theta_prev_init;  /* 角度跟踪已初始化 */

    /* 齿槽转矩LUT识别专用 */
    uint8_t cg_state;               /* 齿槽识别子状态 */
    float cg_theta_start;           /* 记录起始机械角 rad */
    float cg_theta_prev;            /* 上一拍机械角 rad */
    float cg_theta_accum;           /* 累计机械角位移 rad */
    float cg_bin_iq_sum[264];       /* 每bin的Iq累加值 */
    uint16_t cg_bin_count[264];     /* 每bin的采样计数 */
    float cg_drag_voltage;          /* 开环拖动电压幅值 V */
    float cg_drag_speed_elec;       /* 开环电角速度 rad/s */
    float cg_drag_theta_elec;       /* 开环合成电角度 rad */

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
MI_ErrorCode_t MI_VerifyMotion(MI_Handle_t *handle);
MI_ErrorCode_t MI_IdentifyCogging(MI_Handle_t *handle);

/* Rs在线估计 */
void MI_RsOnlineEstimator_Init(RsOnlineEstimator_t *est, float alpha);
void MI_RsOnlineEstimator_Enable(RsOnlineEstimator_t *est, uint8_t enable);
void MI_RsOnlineEstimator_Update(RsOnlineEstimator_t *est, float Vd, float Vq, float Id, float Iq, float omega_e);
float MI_RsOnlineEstimator_GetRs(RsOnlineEstimator_t *est);

/* 机械零位设置 */
void MI_SetMechZero(MotorParam_t *param, float current_theta_mech);

/* 辅助函数 */
uint8_t MI_CheckMechanicalLock(MI_Handle_t *handle);
void MI_UpdatePIWithNewRs(FOC_Handle_t *foc, float Rs_new);
float MI_GetElapsedTime(MI_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_IDENTIFY_H */
