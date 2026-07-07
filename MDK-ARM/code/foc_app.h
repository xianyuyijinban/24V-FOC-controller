/**
 * @file    foc_app.h
 * @brief   FOC应用层接口
 * @note    整合FOC核心、硬件驱动、参数识别和存储
 */

#ifndef __FOC_APP_H
#define __FOC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "foc_core.h"
#include "motor_identify.h"
#include "param_storage.h"
#include "adc_sampling.h"
#include "tle5012.h"
#include "drv8350s.h"

/*==================== 配置参数 ====================*/

/* 前馈使能开关 (BEMF解耦在 foc_core.h) */
#define FOC_FF_ENABLE_INERTIA    1   /* P2: 加速度/惯量前馈 (Phase D3: enabled) */
#define FOC_FF_INERTIA_MAX_A     0.50f /* 惯量前馈最大补偿电流 A */
#define FOC_FF_INERTIA_J_MIN     1.0e-6f /* P2门禁: J合理下限 kg·m² */
#define FOC_FF_INERTIA_J_MAX     0.01f  /* P2门禁: J合理上限 kg·m² */
#define FOC_FF_INERTIA_B_MAX     0.05f  /* P2门禁: B合理上限 N·m·s/rad */
#define FOC_FF_ENABLE_FRICTION   1   /* P3: 库仑+粘滞摩擦前馈 (Phase D2: enabled) */
#define FOC_FF_COULOMB_DEADBAND_RADPS 0.05f /* 库仑摩擦死区速度 */
#define FOC_FF_FRICTION_MAX_A    0.50f /* 摩擦前馈最大补偿电流 A */
#define FOC_FF_ENABLE_COGGING    1   /* P0: 齿槽转矩LUT前馈 (Phase P0: collect + diagnose) */
#define FOC_COGGING_LUT_SIZE     264 /* LCM(24,22) for 24N22P motor */
#define FOC_FF_COGGING_MAX_A     0.30f /* 齿槽前馈最大补偿电流 A */
/* cogging gain/phase now runtime via CMD:COG_CFG; defaults in FOC_App_Init */
#define FOC_FF_ENABLE_OBSERVER   0   /* P4: 负载转矩观测器 (默认关闭，需调参) */
#define FOC_FF_OBSERVER_GAIN_L   50.0f /* 观测器收敛率 rad/s */
#define FOC_FF_OBSERVER_LPF_HZ   10.0f /* 观测器输出LPF截止频率 Hz */
#define FOC_FF_OBSERVER_MAX_A    1.0f  /* 观测器前馈最大补偿电流 A */

/* 控制周期 */
#define FOC_PWM_FREQUENCY       20000       /* PWM频率 20kHz */
#define FOC_PWM_PERIOD          12000       /* ARR=11999, center-aligned */
#define FOC_CONTROL_FREQ        10000       /* PWM/ADC/effective current loop freq */
#define FOC_SPEED_LOOP_FREQ     2000        /* Target speed loop freq (TBD by scope) */
#define FOC_POSITION_LOOP_FREQ  200         /* 【新增】位置环频率 200Hz */
#define FOC_SPEED_LPF_CUTOFF_HZ 20.0f       /* 速度估算低通截止频率：低速台架优先抑制编码器微分噪声 */
#define FOC_SPEED_EST_ACCEL_LIMIT_RAD_PER_S2 60.0f /* 速度估算限斜率，抑制低速编码器离群尖峰 */
#define FOC_SPEED_STATIC_FRICTION_POS_COMP_A 0.03f /* Rs前馈启用后降低起动补偿，避免低速冲飞 */
#define FOC_SPEED_STATIC_FRICTION_NEG_COMP_A 0.03f /* Rs前馈启用后正反向使用对称起动补偿 */
#define FOC_SPEED_POSITIVE_IQ_LIMIT_A 2.00f /* V6: 24V台架增流，克服堵转 */
#define FOC_SPEED_NEGATIVE_IQ_LIMIT_A 2.00f /* V6: 24V台架增流，克服堵转 */
#define FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A 2.00f /* V6: 24V台架增流，克服堵转 */
#define FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A 2.00f /* V6: 24V台架增流，克服堵转 */
#define FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 2.0f /* V4 装配基线 */
#define FOC_SPEED_REF_MAX_RAD_PER_S        8.0f /* RAW SPEED SREF safety clamp */
#define FOC_CURRENT_LOOP_KP_12V_BENCH       0.50f /* ARR=11999 post-resolution baseline: clean bidirectional tracking. */
#define FOC_CURRENT_LOOP_KI_12V_BENCH       0.0f  /* P-only baseline; Ki to be reintroduced in small steps later. */
#define FOC_POSITION_USER_POSITIVE_STATIC_FRICTION_COMP_A 0.05f /* 位置末端小误差静摩擦补偿，帮助闭合最后几度 */
#define FOC_POSITION_USER_NEGATIVE_STATIC_FRICTION_COMP_A 0.05f /* 正反向对称补偿，避免零位附近方向偏置 */
#define FOC_POSITION_PD_KP_DEFAULT 4.0f  /* 12V台架位置模式默认刚度 */
#define FOC_POSITION_PD_KD_DEFAULT 0.12f /* 12V台架位置模式默认速度阻尼 */
#define FOC_POSITION_PD_KP_SCALE 1.0f   /* 派生增益 */
#define FOC_POSITION_PD_KP_MIN 4.0f     /* 最小刚度 */
#define FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S 0.05f /* 误差超过该值才加起动偏置 */
#define FOC_SPEED_STATIC_FRICTION_ACTIVE_RAD_PER_S 0.20f /* 实际速度低于该值才加起动偏置 */
#define FOC_POSITION_STATIC_FRICTION_ENTER_RAD 0.052f /* 位置误差超过约3deg才加起动偏置 */
#define FOC_POSITION_STATIC_FRICTION_EXIT_RAD 0.017f /* 位置误差小于约1deg才退出起动偏置 */
#define FOC_NEUTRAL_BOOTSTRAP_VOLTAGE_EPS 0.001f /* 中性PWM起动判定：当前电压近零 */
#define FOC_NEUTRAL_BOOTSTRAP_CURRENT_EPS 0.001f /* 中性PWM起动判定：电流给定非零 */
#define FOC_LOW_SIDE_ZERO_WINDOW_FORCE_INTERVAL 32U  /* 0/0/0窗口饥饿逃逸间隔，低边采样下保守避免启动过流 */

/* 电流采样 */
#define FOC_ADC_RESOLUTION      12          /* ADC分辨率 */
#define FOC_ADC_VREF            3.3f        /* ADC参考电压 */
#define FOC_CURRENT_GAIN        0.01f       /* 采样电阻0.01Ω + 放大倍数 */
#define FOC_ADC_TO_CURRENT(adc) (((float)(adc) - 2048.0f) * FOC_ADC_VREF / 4096.0f / FOC_CURRENT_GAIN)

/* 编码器 */
#define FOC_ENCODER_RESOLUTION  65536       /* TLE5012 16位分辨率 */

/* 保护阈值默认值 */
#define FOC_DEFAULT_OVERCURRENT_LIMIT_A   3.0f    /* 过流保护阈值 A */
#define FOC_DEFAULT_OVERVOLTAGE_LIMIT_V   18.0f   /* 12V系统过压阈值 V */
#define FOC_DEFAULT_UNDERVOLTAGE_LIMIT_V  10.0f   /* 12V系统欠压阈值 V */
#define FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V  1.0f    /* 严重电压故障相对告警阈值的额外裕量 */
#define FOC_VOLTAGE_FAULT_RECOVER_HYSTERESIS_V 0.5f /* 严重电压故障自动恢复滞回 */
#define FOC_ADC_SAMPLE_MISS_FAULT_THRESHOLD 3U /* 连续采样失配升级为故障 */
#define FOC_ENCODER_FAULT_MISS_THRESHOLD 5U /* 编码器连续无效/CRC错误升级为故障 */
#define FOC_CURRENT_REF_LIMIT_RATIO       0.80f /* 电流给定最多使用过流阈值的80%，给保护留余量 */
#define FOC_CURRENT_REF_VOLTAGE_RATIO     0.577f /* SVPWM相电压可用量约 Vbus/sqrt(3) */
#define FOC_CURRENT_REF_VOLTAGE_MARGIN    1.25f  /* 高阻电机按电压/Rs降额，12V台架仍有Vq余量时小步放宽 */
#define FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S      20.0f  /* 堵转开环试转最大机械角速度 */
#define FOC_STALL_OPEN_LOOP_SPEED_RAMP_RAD_PER_S2    200.0f /* 堵转开环试转速度斜率 */
#define FOC_STALL_OPEN_LOOP_CURRENT_MAX_A            2.0f   /* 堵转开环试转电流上限 */
#define FOC_STALL_OPEN_LOOP_DEFAULT_SPEED_RAD_PER_S  5.0f   /* 未显式给speed时的默认试转速度 */
#define FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A             0.5f   /* 未显式给Iq时的默认试转扭矩 */
#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           2.0f   /* V4 装配基线 */
#define FOC_POSITION_CRUISE_SPEED_RAD_PER_S          0.80f  /* V4 巡航速度下限 */
#define FOC_POSITION_CRUISE_HOLD_THRESHOLD_RAD       0.087f /* V4 巡航切PD阈值 (~5 deg) */

/* V5 位置模式运行时运动配置默认值与范围 — 12V 标准基线 */
#define FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT      1.0f   /* 速度上限 rad/s (12V 稳定包络; 极限 ±1.0) */
#define FOC_MOTION_CFG_ACCEL_LIMIT_DEFAULT      2.0f   /* 加速度上限 rad/s^2 */
#define FOC_MOTION_CFG_CRUISE_SPEED_DEFAULT     0.3f   /* 巡航下限 rad/s */
#define FOC_MOTION_CFG_SPEED_LIMIT_MIN          0.2f   /* 速度下限 */
#define FOC_MOTION_CFG_SPEED_LIMIT_MAX          2.0f   /* 速度上限 (12V 安全限幅) */
#define FOC_MOTION_CFG_ACCEL_LIMIT_MIN          0.5f   /* 加速度下限 */
#define FOC_MOTION_CFG_ACCEL_LIMIT_MAX          5.0f   /* 加速度上限 (12V 安全限幅) */

#define FOC_GIMBAL_RAMP_ACCEL_DEFAULT           2.0f   /* GIMBAL SREF斜坡默认加速度 rad/s^2 */
#define FOC_JOINT_SOFT_LIMIT_DEFAULT_ENABLED    1U

#define FOC_SPRING_K_DEFAULT       0.5f   /* 弹簧刚度 A/rad */
#define FOC_SPRING_D_DEFAULT       0.05f  /* 阻尼系数 A/(rad/s) */
#define FOC_SPRING_LIMIT_DEFAULT   0.30f  /* 弹簧力限幅 A */
#define FOC_DETENT_COUNT_DEFAULT   12.0f  /* 卡点数量 (12/圈 = 每30°) */
#define FOC_DETENT_STRENGTH_DEFAULT 1.0f  /* 吸附强度 A/rad */
#define FOC_DETENT_WIDTH_DEFAULT   0.13f  /* 卡点宽度 ~7.5° */
#define FOC_DETENT_LIMIT_DEFAULT   0.25f  /* 卡点力限幅 A */

/* 固件版本信息 */
#define FOC_FW_VERSION          "1.0.0"
#define FOC_PARAM_VERSION       "1"
#define FOC_BASELINE_NAME       "12V_STANDARD"
#ifndef FOC_GIT_HASH
#define FOC_GIT_HASH            "unknown"
#endif

#define FOC_WARNING_VBUS_UNDERVOLTAGE_BIT (1UL << 0)
#define FOC_WARNING_VBUS_OVERVOLTAGE_BIT  (1UL << 1)
/* 注意：CURRENT_IMBALANCE_THRESH 定义在 adc_sampling.h 中 */

/*==================== 数据结构 ====================*/

/* FOC运行状态 */
typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_INIT,
    FOC_STATE_PARAM_IDENTIFY,   /* 参数识别中 */
    FOC_STATE_READY,            /* 准备就绪 */
    FOC_STATE_RUNNING,          /* 运行中 */
    FOC_STATE_FAULT,            /* 故障 */
} FOC_AppState_t;

/* FOC故障代码 */
typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_OVERCURRENT,
    FOC_FAULT_OVERVOLTAGE,
    FOC_FAULT_UNDERVOLTAGE,
    FOC_FAULT_ENCODER,
    FOC_FAULT_DRV8350S,
    FOC_FAULT_PARAM_INVALID,
    FOC_FAULT_ADC_SAMPLING,
} FOC_FaultCode_t;

/* 控制模式 (底层 FOC 模式) */
typedef enum {
    FOC_MODE_TORQUE = 0,        /* 力矩模式：直接控制Iq */
    FOC_MODE_SPEED,             /* 速度模式：速度环控制 */
    FOC_MODE_POSITION,          /* 位置模式：位置环+速度环 */
} FOC_ControlMode_t;

/* 上层应用模式 (Phase 3 — 产品模式外壳) */
typedef enum {
    APP_MODE_RAW = 0,           /* 原始模式：兼容所有旧行为 */
    APP_MODE_JOINT_POS,         /* 关节位置模式：底层POSITION + 软限位 + fault自动STOP */
    APP_MODE_GIMBAL_SPEED,      /* 云台速度模式：底层SPEED + SREF斜坡平滑 */
    APP_MODE_HOLD,              /* 当前位置保持：底层POSITION + 锁定当前角度 */
    APP_MODE_SPRING_DAMPER,     /* 虚拟弹簧阻尼：Iq = K*(theta_ref - theta) - D*speed */
    APP_MODE_DETENT,            /* 虚拟卡点：吸附到最近 detent 位置 */
} AppMode_t;

/* 保护参数 */
typedef struct {
    float overcurrent_limit_a;
    float overvoltage_limit_v;
    float undervoltage_limit_v;
} FOC_ProtectionConfig_t;

/* 位置环 PD 参数 */
typedef struct {
    float kp;
    float kd;
    float output_max;
    float output_min;
} FOC_PositionPD_t;

/* 齿槽转矩LUT (P0 feedforward) */
typedef struct {
    float table[FOC_COGGING_LUT_SIZE];  /* Iq补偿值 vs 机械角bin */
    uint16_t valid_size;                /* 实际有效表项数 */
    uint8_t  valid;                     /* 1 = LUT已加载且有效 */
    uint8_t  pending;                   /* 1 = LUT待持久化到Flash */
    uint8_t  save_attempted;            /* 1 = 已尝试保存LUT到Flash */
    float    gain;                      /* LUT幅值缩放 0.0-1.0, default 0.25 */
    float    phase_offset_rad;          /* 查表相位偏移 rad, default +60deg */
} FOC_CoggingLUT_t;

/* 负载转矩观测器 (P4) — Gopinath型降维扰动观测器 */
typedef struct {
    float z;            /* 观测器状态 */
    float J_hat;        /* 估计惯量 kg·m² */
    float B_hat;        /* 估计粘滞摩擦 N·m·s/rad */
    float Kt;           /* 转矩常数 N·m/A */
    float l;            /* 观测器增益 rad/s */
    float T_est;        /* 估计扰动转矩 N·m */
    float T_lpf;        /* 低通滤波后估计值 */
    uint8_t enabled;    /* 使能标志 */
} FOC_TorqueObserver_t;

/* 前馈诊断结构体 (FFDiag) */
typedef struct {
    float bemf_vd;              /* P1 BEMF解耦 Vd补偿量 V */
    float bemf_vq;              /* P1 BEMF解耦 Vq补偿量 V */
    float inertia_iq;           /* P2 惯量前馈 Iq贡献 A */
    float friction_iq;          /* P3 摩擦前馈 Iq贡献 A */
    float cogging_iq;           /* P0 齿槽前馈 Iq贡献 A */
    float observer_iq;          /* P4 观测器前馈 Iq贡献 A */
    float ff_total_iq;          /* 前馈总Iq贡献 A */
    uint8_t bemf_enabled;       /* P1 使能状态 */
    uint8_t inertia_blocked;    /* P2 被门禁阻止 */
    uint8_t friction_enabled;   /* P3 使能状态 */
    uint8_t cogging_enabled;    /* P0 使能状态 */
    uint8_t observer_enabled;   /* P4 使能状态 */
    uint8_t ff_enc_dir_blocked; /* 因enc_dir != -1阻止所有前馈 */
} FOC_FFDiag_t;

/* FOC应用层句柄 */
typedef struct {
    /* 核心FOC */
    FOC_Handle_t foc;
    
    /* 电机参数 */
    MotorParam_t motor_param;
    
    /* 参数识别 */
    MI_Handle_t mi_handle;
    RsOnlineEstimator_t rs_est;
    FOC_ProtectionConfig_t protection;
    
    /* 状态 */
    FOC_AppState_t state;
    FOC_FaultCode_t fault_code;
    uint32_t warning_flags;            /* 非停机告警位 */
    FOC_ControlMode_t control_mode;  /* 底层FOC模式：力矩/速度/位置 */
    AppMode_t app_mode;              /* 上层应用模式 (Phase 3) */
    float gimbal_ramp_accel_radps2;  /* GIMBAL_SPEED SREF斜坡加速度 */
    uint8_t cal_state;               /* Phase 4: 0=idle 1=running 2=done 3=failed 4=aborted */
    uint8_t cal_last_error;          /* Phase 4: last calibration error code */
    /* Phase 3B: spring-damper / detent config */
    float    spring_K;               /* 弹簧刚度 A/rad */
    float    spring_D;               /* 阻尼系数 A/(rad/s) */
    float    spring_limit_A;         /* 弹簧力限幅 A */
    float    detent_count;           /* 卡点数量 (每圈) */
    float    detent_strength;        /* 卡点吸附强度 A/rad */
    float    detent_width_rad;       /* 卡点宽度 rad */
    float    detent_limit_A;         /* 卡点力限幅 A */

    /* 反馈值 */
    float Ia, Ib, Ic;           /* 三相电流 A */
    float Vbus;                 /* 母线电压 V */
    float theta_mech;           /* 机械角度 rad */
    float theta_elec;           /* 电角度 rad */
    float speed_mech;           /* 机械转速 rad/s */
    float speed_elec;           /* 电转速 rad/s */
    float speed_theta_prev;     /* 速度估算上一拍机械角度 rad */
    uint32_t theta_sample_seq;  /* 机械角度样本序号 */
    
    /* 参考值 */
    float Id_ref;
    float Iq_ref;
    float speed_ref;
    float speed_ref_ramped;     /* 速度模式内部限斜率给定 rad/s */
    float speed_ref_ramped_prev;/* 上一拍速度给定 (惯量前馈加速度计算) */
    float pos_ref;              /* 位置给定 (rad) */

    /* V5 位置模式运行时运动配置 */
    float position_speed_limit_radps;   /* 速度上限 rad/s */
    float position_accel_limit_radps2;  /* 加速度上限 rad/s^2 */
    float position_cruise_speed_radps;  /* 巡航下限 rad/s */
    float joint_pos_limit_min_rad;      /* 关节软限位下限 rad (JOINT_POS) */
    float joint_pos_limit_max_rad;      /* 关节软限位上限 rad (JOINT_POS) */
    uint8_t joint_soft_limit_enabled;   /* 软限位使能 */

    /* 外环控制器 */
    FOC_PI_Controller_t pi_speed;   /* 速度环PI */
    FOC_PositionPD_t pos_pd;        /* 位置环PD */

    /* 前馈数据 */
    FOC_CoggingLUT_t cogging_lut;   /* 齿槽转矩LUT (P0) */
    FOC_TorqueObserver_t torque_obs;/* 负载转矩观测器 (P4) */
    FOC_FFDiag_t ff_diag;           /* 前馈诊断数据 */
    
    /* 运行时计数 */
    uint32_t control_count;
    uint32_t speed_loop_count;
    uint32_t adc_valid_low_side_count;
    uint32_t adc_invalid_low_side_count;
    uint32_t adc_forced_low_side_count;
    uint32_t adc_invalid_low_side_streak;
    float speed_loop_ref_diag;
    float speed_loop_mech_diag;
    float speed_loop_error_diag;
    float speed_loop_iq_mech_diag;
    float speed_loop_friction_diag;
    float speed_loop_iq_cmd_diag;
    float speed_loop_p_iq;          /* 速度环比例贡献 A */
    float speed_loop_i_iq;          /* 速度环积分贡献 A */
    uint8_t speed_i_state;          /* 积分状态: 0=inactive 1=active 2=frozen 3=cleared 4=sat_hold */
    float position_loop_error_diag;     /* 位置环最近一次位置误差 rad */
    float position_loop_pd_out_diag;    /* 位置PD输出到速度给定 rad/s */
    uint8_t position_loop_pd_sat_diag;  /* 位置PD输出是否触及速度上限 */
    uint8_t position_loop_speed_ramp_sat_diag; /* 位置模式速度斜坡是否限制给定 */
    /* V4 轨迹诊断 */
    uint8_t  traj_active_diag;     /* 巡航模式激活标志 */
    float    traj_cmd_diag;        /* 最终速度指令 rad/s */
    uint8_t position_loop_iq_pos_sat_diag;     /* 位置模式正向Iq是否触顶 */
    uint8_t position_loop_iq_neg_sat_diag;     /* 位置模式负向Iq是否触底 */
    uint32_t position_pref_cmd_count_diag;     /* 最近PREF命令计数 */
    float position_pref_raw_diag;              /* 最近PREF用户原始目标 rad */
    float position_pref_mapped_diag;           /* 最近PREF按encoder_dir映射后目标 rad */
    float position_pref_before_diag;           /* PREF执行前pos_ref rad */
    float position_pref_after_diag;            /* PREF执行后pos_ref rad */
    uint8_t position_pref_user_set_diag;       /* PREF执行后user_set状态 */
    
    /* 使能标志 */
    uint8_t motor_identified;      /* 0=未识别，1=已识别 */
    uint8_t ff_blocked_by_enc_dir; /* 1=enc_dir != -1，阻止所有前馈验证 */
    uint8_t stall_mode_armed;      /* 0=未授权，1=允许未识别堵转使能 */
    uint8_t stall_open_loop_active;/* 0=关闭，1=编码器离线开环试转中 */
    uint8_t power_unlocked;        /* 0=锁定，1=允许功率级动作 */
    uint8_t enable_pwm;
    uint8_t enable_identify;
    uint8_t position_ref_user_set; /* 1=用户显式下发过位置目标，不在使能时覆盖 */
    uint8_t speed_loop_ready;   /* 1=速度估算器已用当前角度完成预置 */
    uint8_t position_friction_active; /* 位置模式静摩擦补偿滞回状态 */
    volatile uint8_t pending_disable;   /* ISR中仅做快速下电，阻塞SPI收尾延后到主循环 */
    float stall_theta_elec;        /* 堵转开环试转使用的合成电角度 */
    float stall_speed_ref_mech;    /* 堵转开环试转使用的限幅/斜率后的机械角速度 */
    
} FOC_AppHandle_t;

/*==================== 函数声明 ====================*/

/* 初始化和主循环 */
void FOC_App_Init(FOC_AppHandle_t *handle);
void FOC_App_MainLoop(FOC_AppHandle_t *handle);
void FOC_App_TIM1_IRQHandler(FOC_AppHandle_t *handle);
void FOC_App_TIM2_IRQHandler(FOC_AppHandle_t *handle);
void FOC_App_RequestFaultShutdownFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault);

/* 三环控制（分频调用） */
void FOC_App_SpeedLoop(FOC_AppHandle_t *handle);           /* 速度环 (2kHz) */
void FOC_App_PositionLoop(FOC_AppHandle_t *handle);        /* 位置环 (200Hz) */
void FOC_App_ParamIdentifyLoop(FOC_AppHandle_t *handle);   /* 兼容接口：参数识别已移至TIM1周期 */

/* 兼容旧代码的函数名（已弃用，保留用于兼容性） */
#define FOC_App_PositionSpeedLoop FOC_App_SpeedLoop  /* 旧代码兼容性 */

/* 控制接口 */
void FOC_App_Enable(FOC_AppHandle_t *handle);
void FOC_App_Disable(FOC_AppHandle_t *handle);
void FOC_App_ResetMotionState(FOC_AppHandle_t *handle);
void FOC_App_RefreshTelemetry(FOC_AppHandle_t *handle);
uint32_t FOC_App_GetVoltageWarningFlags(const FOC_AppHandle_t *handle);
uint8_t FOC_App_GetVoltageTripFault(const FOC_AppHandle_t *handle, FOC_FaultCode_t *fault);
uint8_t FOC_App_IsVoltageFaultRecovered(const FOC_AppHandle_t *handle, FOC_FaultCode_t fault);

/* ── Phase 5A: Black Box ───────────────────────────────────── */

#define BLACKBOX_SAMPLES   100
#define BLACKBOX_RATE_HZ   50

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  state;
    uint8_t  control_mode;
    uint8_t  app_mode;
    uint8_t  fault_code;
    uint32_t warning_flags;
    float    Vbus;
    float    theta_mech;
    float    speed_mech;
    float    Id;
    float    Iq;
    float    Id_ref;
    float    Iq_ref;
    float    Vd;
    float    Vq;
    float    speed_ref;
    float    pos_ref;
    uint32_t fault_flags;
    uint16_t drv_fault1;
    uint16_t drv_vgs2;
    uint16_t encoder_crc_count;
    uint8_t  encoder_valid;
} BlackBoxSample_t;

void     BlackBox_Init(void);
void     BlackBox_Sample(void);
void     BlackBox_Freeze(uint8_t reason);
void     BlackBox_Clear(void);
uint8_t  BlackBox_GetCount(void);
uint8_t  BlackBox_IsFrozen(void);
uint8_t  BlackBox_GetFreezeReason(void);
uint32_t BlackBox_GetFreezeTime(void);
const BlackBoxSample_t *BlackBox_GetSample(uint8_t index);  /* 0 = oldest */

void FOC_App_SetCurrentRef(FOC_AppHandle_t *handle, float Id_ref, float Iq_ref);
void FOC_App_SetSpeedRef(FOC_AppHandle_t *handle, float speed_ref);
void FOC_App_SetPositionRef(FOC_AppHandle_t *handle, float pos_ref);
void FOC_App_SetPositionPDGains(FOC_AppHandle_t *handle, float kp, float kd);
void FOC_App_SetControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode);
void FOC_App_SetRawControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode);
void FOC_App_SetAppMode(FOC_AppHandle_t *handle, AppMode_t mode);
void FOC_App_SetJointLimits(FOC_AppHandle_t *handle, float min_rad, float max_rad);
void FOC_App_SetGimbalRamp(FOC_AppHandle_t *handle, float accel_radps2);
void FOC_App_SetSpringCfg(FOC_AppHandle_t *handle, float K, float D, float limit);
void FOC_App_SetDetentCfg(FOC_AppHandle_t *handle, float count, float strength, float width, float limit);
uint8_t FOC_App_CalIsBusy(FOC_AppHandle_t *handle);
uint8_t FOC_App_CalPrecheck(FOC_AppHandle_t *handle);
void FOC_App_SetVoltageThresholds(FOC_AppHandle_t *handle, float undervoltage, float overvoltage);
void FOC_App_SetPolePairs(FOC_AppHandle_t *handle, uint8_t pole_pairs);
float FOC_App_PositionSensorToControlFrame(const FOC_AppHandle_t *handle, float pos_ref_sensor);
float FOC_App_PositionControlToSensorFrame(const FOC_AppHandle_t *handle, float pos_ref_control);

/* 参数管理 */
void FOC_App_LoadParam(FOC_AppHandle_t *handle);
void FOC_App_SaveParam(FOC_AppHandle_t *handle);
void FOC_App_StartIdentify(FOC_AppHandle_t *handle);
void FOC_App_StopIdentify(FOC_AppHandle_t *handle);
uint8_t FOC_App_IsIdentifyComplete(FOC_AppHandle_t *handle);

/* 状态查询 */
FOC_AppState_t FOC_App_GetState(FOC_AppHandle_t *handle);
FOC_FaultCode_t FOC_App_GetFault(FOC_AppHandle_t *handle);
const char* FOC_App_GetStateString(FOC_AppState_t state);
const char* FOC_App_GetFaultString(FOC_FaultCode_t fault);

/* 调试接口 */
void FOC_App_GetDebugInfo(FOC_AppHandle_t *handle, float *Id, float *Iq, float *Vd, float *Vq, 
                          float *theta, float *speed, float *Rs_est);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_APP_H */
