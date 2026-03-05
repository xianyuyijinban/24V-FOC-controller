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

/* 控制周期 */
#define FOC_PWM_FREQUENCY       20000       /* PWM频率 20kHz */
#define FOC_PWM_PERIOD          50          /* PWM周期 50us */
#define FOC_CONTROL_FREQ        20000       /* 电流环频率 20kHz */
#define FOC_SPEED_LOOP_FREQ     2000        /* 【修改】速度环频率 2kHz (原为1kHz) */
#define FOC_POSITION_LOOP_FREQ  200         /* 【新增】位置环频率 200Hz */
#define FOC_SPEED_LPF_CUTOFF_HZ 200.0f      /* 速度估算低通截止频率 */

/* 电流采样 */
#define FOC_ADC_RESOLUTION      12          /* ADC分辨率 */
#define FOC_ADC_VREF            3.3f        /* ADC参考电压 */
#define FOC_CURRENT_GAIN        0.01f       /* 采样电阻0.01Ω + 放大倍数 */
#define FOC_ADC_TO_CURRENT(adc) (((float)(adc) - 2048.0f) * FOC_ADC_VREF / 4096.0f / FOC_CURRENT_GAIN)

/* 编码器 */
#define FOC_ENCODER_RESOLUTION  65536       /* TLE5012 16位分辨率 */

/* 保护阈值 */
#define FOC_OVERCURRENT_THRESH      15.0f   /* 过流保护阈值 A */
#define FOC_OVERVOLTAGE_THRESH      28.0f   /* 过压保护阈值 V */
#define FOC_UNDERVOLTAGE_THRESH     18.0f   /* 欠压保护阈值 V */
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
} FOC_FaultCode_t;

/* 控制模式 */
typedef enum {
    FOC_MODE_TORQUE = 0,        /* 力矩模式：直接控制Iq */
    FOC_MODE_SPEED,             /* 速度模式：速度环控制 */
    FOC_MODE_POSITION,          /* 位置模式：位置环+速度环 */
} FOC_ControlMode_t;

/* FOC应用层句柄 */
typedef struct {
    /* 核心FOC */
    FOC_Handle_t foc;
    
    /* 电机参数 */
    MotorParam_t motor_param;
    
    /* 参数识别 */
    MI_Handle_t mi_handle;
    RsOnlineEstimator_t rs_est;
    
    /* 状态 */
    FOC_AppState_t state;
    FOC_FaultCode_t fault_code;
    FOC_ControlMode_t control_mode;  /* 控制模式：力矩/速度/位置 */
    
    /* 反馈值 */
    float Ia, Ib, Ic;           /* 三相电流 A */
    float Vbus;                 /* 母线电压 V */
    float theta_mech;           /* 机械角度 rad */
    float theta_elec;           /* 电角度 rad */
    float speed_mech;           /* 机械转速 rad/s */
    float speed_elec;           /* 电转速 rad/s */
    
    /* 参考值 */
    float Id_ref;
    float Iq_ref;
    float speed_ref;
    float pos_ref;              /* 位置给定 (rad) */
    
    /* PI控制器 */
    FOC_PI_Controller_t pi_speed;   /* 速度环PI */
    FOC_PI_Controller_t pi_pos;     /* 位置环PI */
    
    /* 运行时计数 */
    uint32_t control_count;
    uint32_t speed_loop_count;
    
    /* 使能标志 */
    uint8_t enable_pwm;
    uint8_t enable_identify;
    volatile uint8_t pending_disable;   /* ISR中仅做快速下电，阻塞SPI收尾延后到主循环 */
    
} FOC_AppHandle_t;

/*==================== 函数声明 ====================*/

/* 初始化和主循环 */
void FOC_App_Init(FOC_AppHandle_t *handle);
void FOC_App_MainLoop(FOC_AppHandle_t *handle);
void FOC_App_TIM1_IRQHandler(FOC_AppHandle_t *handle);
void FOC_App_TIM2_IRQHandler(FOC_AppHandle_t *handle);

/* 三环控制（分频调用） */
void FOC_App_SpeedLoop(FOC_AppHandle_t *handle);           /* 速度环 (2kHz) */
void FOC_App_PositionLoop(FOC_AppHandle_t *handle);        /* 位置环 (200Hz) */
void FOC_App_ParamIdentifyLoop(FOC_AppHandle_t *handle);   /* 兼容接口：参数识别已移至TIM1周期 */

/* 兼容旧代码的函数名（已弃用，保留用于兼容性） */
#define FOC_App_PositionSpeedLoop FOC_App_SpeedLoop  /* 旧代码兼容性 */

/* 控制接口 */
void FOC_App_Enable(FOC_AppHandle_t *handle);
void FOC_App_Disable(FOC_AppHandle_t *handle);
void FOC_App_SetCurrentRef(FOC_AppHandle_t *handle, float Id_ref, float Iq_ref);
void FOC_App_SetSpeedRef(FOC_AppHandle_t *handle, float speed_ref);
void FOC_App_SetPositionRef(FOC_AppHandle_t *handle, float pos_ref);
void FOC_App_SetControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode);

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
