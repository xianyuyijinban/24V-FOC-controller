/**
 * @file    foc_core.h
 * @brief   FOC核心算法头文件 - DQ坐标系控制
 * @note    包含坐标变换、SVPWM、PI控制器
 */

#ifndef __FOC_CORE_H
#define __FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <math.h>

/*==================== 数学常量 ====================*/
#define FOC_PI              3.14159265359f
#define FOC_SQRT3           1.73205080757f
#define FOC_SQRT3_DIV2      0.86602540378f   /* √3/2 */
#define FOC_2PI_DIV3        2.09439510239f   /* 2π/3 */

/*==================== 前馈使能开关 ====================*/
#define FOC_FF_ENABLE_BEMF       1   /* P1: BEMF解耦前馈 (Phase D1: enabled) */

/*==================== 自适应 Rs 前馈参数 ====================*/
#ifndef FOC_CONTROL_FREQ
#define FOC_CONTROL_FREQ            10000   /* 电流环控制频率 (Hz)，需与 foc_app.h 保持一致 */
#endif
#define FOC_RS_FF_CONF_INIT         1.0f    /* 初始/空闲置信度 */
#define FOC_RS_FF_DIQDT_THRESH      2000.0f /* dIq/dt 阈值 (A/s)，超出按比例降权 */
#define FOC_RS_FF_SPEED_ERR_THRESH  1.0f    /* 速度误差阈值 (rad/s) */
#define FOC_RS_FF_SPEED_ERR_FACTOR  0.8f    /* 速度误差大时的惩罚乘数 */
#define FOC_RS_FF_SAT_RATIO_THRESH  0.95f   /* 饱和比阈值，低于此值乘 sat_ratio（保留） */
#define FOC_RS_FF_SIGN_REF_THRESH    0.08f   /* sign-kill: |Iq_ref| 最小门限 (A) */
#define FOC_RS_FF_SIGN_FB_THRESH     0.06f   /* sign-kill: |Iq_fb| 最小门限 (A) */
#define FOC_RS_FF_SIGN_HOLD_CYCLES   50U     /* sign-kill: 连续反符号周期数 (~5ms@10kHz) */
#define FOC_RS_FF_SIGN_THRESH        0.01f   /* 符号反转检测 Iq_ref 门限 (A)（保留） */
#define FOC_RS_FF_RECOVER_TAU       0.3f    /* 置信度恢复 LPF 时间常数 (s) */
#define FOC_RS_FF_RECOVER_ALPHA     (1.0f / (1.0f + FOC_RS_FF_RECOVER_TAU * (float)FOC_CONTROL_FREQ))

/* RsFF 路径模式 */
#define FOC_RS_FF_MODE_OFF          0   /* 关闭所有 Rs 前馈 */
#define FOC_RS_FF_MODE_DQ           1   /* DQ 域 RsFF（默认，原有路径） */
#define FOC_RS_FF_MODE_ABC          2   /* ABC 域 RsFF：相电压前馈 + 逐相限幅 */

/* ABC RsFF 逐相限幅 */
#define FOC_RS_FF_ABC_VCLAMP_RATIO  0.25f   /* |Vabc_ff_phase| ≤ ratio * Vbus */

/*==================== 数据结构 ====================*/

/* 三相静止坐标系 (ABC) */
typedef struct {
    float a;
    float b;
    float c;
} FOC_ABC_t;

/* 两相静止坐标系 (Alpha-Beta) */
typedef struct {
    float alpha;
    float beta;
} FOC_AlphaBeta_t;

/* 两相旋转坐标系 (DQ) */
typedef struct {
    float d;
    float q;
} FOC_DQ_t;

/* PI控制器结构体 */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float integral_sep_thresh;
    float output_max;
    float output_min;
    float integral_max;
} FOC_PI_Controller_t;

/* SVPWM输出结构体 */
typedef struct {
    float Ta;
    float Tb;
    float Tc;
    uint8_t sector;
} FOC_SVPWM_t;

/* FOC控制句柄 */
typedef struct {
    /* 电流反馈 */
    FOC_ABC_t Iabc;
    FOC_AlphaBeta_t IalphaBeta;
    FOC_DQ_t Idq;
    
    /* 电压输出 */
    FOC_DQ_t Vdq;
    FOC_AlphaBeta_t ValphaBeta;
    FOC_SVPWM_t svpwm;
    
    /* 角度 */
    float theta_elec;       /* 电角度 (rad) */
    float sin_theta;
    float cos_theta;
    
    /* PI控制器 */
    FOC_PI_Controller_t pi_d;
    FOC_PI_Controller_t pi_q;
    
    /* 参考值 */
    float Id_ref;
    float Iq_ref;
    
    /* 母线电压 */
    float Vbus;

    /* Current-loop feedforward */
    float current_resistance_ohm;
    float rs_ff_scale;          /* Rs前馈基础缩放因子 (0~1, 默认0.50) */
    /* 自适应 Rs 前馈 (P5) */
    float rs_ff_confidence;     /* 有效置信度 [0,1] = 非对称LPF输出 */
    float rs_ff_confidence_lpf; /* 非对称LPF内部状态 */
    float rs_ff_dIq_ref_prev;   /* 上一拍Iq_ref，用于计算dIq/dt */
    float rs_ff_speed_error;    /* 速度环最新速度误差 (rad/s)，2kHz更新 */
    float rs_ff_diag_raw;       /* 诊断：LPF前原始置信度 */
    uint8_t rs_ff_adaptive;     /* 1=自适应模式开启（默认），0=关闭（使用裸rs_ff_scale） */
    uint8_t rs_ff_sign_protect; /* 1=符号反转保护开启（默认），0=关闭（诊断用） */
    uint8_t rs_ff_sign_blocked; /* 诊断：1=sign-kill当前激活中 */
    uint16_t rs_ff_sign_mismatch_count; /* 诊断：连续反符号周期计数 */
    uint8_t rs_ff_mode;         /* RsFF路径: 0=OFF 1=DQ(默认) 2=ABC */

    /* ABC 域诊断（RS_FF_MODE=2 时有效） */
    float diag_Va_pi, diag_Vb_pi, diag_Vc_pi;       /* PI输出 ABC电压 */
    float diag_Va_rs_ff, diag_Vb_rs_ff, diag_Vc_rs_ff; /* RsFF ABC电压 */
    float diag_Va_cmd, diag_Vb_cmd, diag_Vc_cmd;       /* 总指令 ABC电压 */
    float diag_Ia_ref, diag_Ib_ref, diag_Ic_ref;       /* 参考电流 ABC */

    /* BEMF decoupling feedforward (P1) */
    float omega_elec_radps;     /* 电角速度 rad/s */
    float bemf_Ld;              /* d轴电感 H */
    float bemf_Lq;              /* q轴电感 H */
    float bemf_Ke;              /* 反电动势常数 V/(rad/s) — 电角速度基准 */
    float bemf_Ke_temp;         /* 临时Ke覆盖值（0=使用默认bemf_Ke） */
    uint8_t bemf_enabled;       /* 1 = BEMF解耦硬件使能（Ld/Lq>0） */
    uint8_t bemf_user_enable;   /* 1 = 用户运行时使能 */
    uint8_t bemf_blocked;       /* 1 = 保护门禁阻止（过压自动置零） */

    /* 电流环诊断（每个FOC迭代更新） */
    float diag_vd_rs_ff;        /* Rs*Id_ref 电阻前馈 */
    float diag_vq_rs_ff;        /* Rs*Iq_ref 电阻前馈 */
    float diag_vd_pi;           /* PI输出 d轴 */
    float diag_vq_pi;           /* PI输出 q轴 */
    float diag_vd_bemf;         /* BEMF解耦 d轴 */
    float diag_vq_bemf;         /* BEMF解耦 q轴 */
    float diag_vd_cmd;          /* 限幅前总Vd指令 */
    float diag_vq_cmd;          /* 限幅前总Vq指令 */
    float diag_v_mag;           /* 限幅前矢量幅值 */
    float diag_sat_ratio;       /* 饱和比（1.0=未饱和，<1.0=已限幅） */

    /* 状态 */
    uint8_t enabled;
} FOC_Handle_t;

/*==================== 函数声明 ====================*/

/* 坐标变换 */
void FOC_Clarke_Transform(const FOC_ABC_t *abc, FOC_AlphaBeta_t *alphabeta);
void FOC_Inverse_Clarke_Transform(const FOC_AlphaBeta_t *alphabeta, FOC_ABC_t *abc);
void FOC_Park_Transform(const FOC_AlphaBeta_t *alphabeta, float sin_theta, float cos_theta, FOC_DQ_t *dq);
void FOC_Inverse_Park_Transform(const FOC_DQ_t *dq, float sin_theta, float cos_theta, FOC_AlphaBeta_t *alphabeta);

/* SVPWM */
void FOC_SVPWM_Generate(const FOC_AlphaBeta_t *ValphaBeta, float Vbus, FOC_SVPWM_t *svpwm);

/* PI控制器 */
void FOC_PI_Init(FOC_PI_Controller_t *pi, float Kp, float Ki, float output_max, float output_min);
float FOC_PI_Update(FOC_PI_Controller_t *pi, float error);

/* FOC主控制 */
void FOC_Init(FOC_Handle_t *foc, float Kp_d, float Ki_d, float Kp_q, float Ki_q);
void FOC_SetCurrentReference(FOC_Handle_t *foc, float Id_ref, float Iq_ref);
void FOC_SetCurrentResistance(FOC_Handle_t *foc, float resistance_ohm);
void FOC_SetBemfParams(FOC_Handle_t *foc, float Ld, float Lq, float Ke);
void FOC_SetOmegaElec(FOC_Handle_t *foc, float omega_elec_radps);
void FOC_SetAngle(FOC_Handle_t *foc, float theta_elec);
void FOC_SetVbus(FOC_Handle_t *foc, float Vbus);
void FOC_UpdateCurrent(FOC_Handle_t *foc, float Ia, float Ib, float Ic);
void FOC_RegenerateVoltageVector(FOC_Handle_t *foc);
void FOC_Run(FOC_Handle_t *foc);
void FOC_GetPWM(FOC_Handle_t *foc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c, uint16_t pwm_period);
void FOC_GetModulationWave(const FOC_Handle_t *foc, float *ma, float *mb, float *mc);

/* 自适应 Rs 前馈 */
void    FOC_UpdateRsFFConfidence(FOC_Handle_t *foc);
void    FOC_SetRsFFAdaptive(FOC_Handle_t *foc, uint8_t enabled);
uint8_t FOC_GetRsFFAdaptive(const FOC_Handle_t *foc);
void    FOC_SetRsFFSpeedError(FOC_Handle_t *foc, float speed_error);

/* RsFF 符号保护开关 */
void    FOC_SetRsFFSignProtect(FOC_Handle_t *foc, uint8_t enabled);

/* RsFF 路径切换 */
void    FOC_SetRsFFMode(FOC_Handle_t *foc, uint8_t mode);

/* ABC 域 SVPWM */
void    FOC_SVPWM_GenerateFromABC(const FOC_ABC_t *Vabc, float Vbus, FOC_SVPWM_t *svpwm);

/* 辅助函数 */
static inline float FOC_Saturate(float value, float max, float min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static inline float FOC_AngleNormalize(float angle)
{
    while (angle > FOC_PI) angle -= 2.0f * FOC_PI;
    while (angle < -FOC_PI) angle += 2.0f * FOC_PI;
    return angle;
}

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CORE_H */
