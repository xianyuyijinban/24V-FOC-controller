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
void FOC_SetAngle(FOC_Handle_t *foc, float theta_elec);
void FOC_SetVbus(FOC_Handle_t *foc, float Vbus);
void FOC_UpdateCurrent(FOC_Handle_t *foc, float Ia, float Ib, float Ic);
void FOC_Run(FOC_Handle_t *foc);
void FOC_GetPWM(FOC_Handle_t *foc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c, uint16_t pwm_period);

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
