/**
 * @file    foc_core.c
 * @brief   FOC核心算法实现 - DQ坐标系控制
 * @note    包含坐标变换、SVPWM(零序注入法)、PI控制器
 */

#include "foc_core.h"
#include <string.h>

/* 电流环电压矢量饱和后的反算抗积分饱和增益 */
#define FOC_CURRENT_AW_GAIN    0.2f
#define FOC_EPSILON            1e-6f

/**
 * @brief Clark变换 (三相静止坐标系 → 两相静止坐标系)
 * @param abc 输入：三相电流/电压 (a, b, c)
 * @param alphabeta 输出：两相静止坐标系 (alpha, beta)
 * @note 使用等幅值变换
 * 
 * 等幅值Clark变换公式：
 *   alpha = a
 *   beta  = (a + 2*b) / √3
 * 
 * 推导：利用三相平衡条件 a + b + c = 0
 *   beta = (b - c) / √3 = (b - (-a-b)) / √3 = (a + 2b) / √3
 */
void FOC_Clarke_Transform(const FOC_ABC_t *abc, FOC_AlphaBeta_t *alphabeta)
{
    alphabeta->alpha = abc->a;
    alphabeta->beta = (abc->a + 2.0f * abc->b) / FOC_SQRT3;
}

/**
 * @brief 反Clark变换 (两相静止坐标系 → 三相静止坐标系)
 * @param alphabeta 输入：两相静止坐标系 (alpha, beta)
 * @param abc 输出：三相电流/电压 (a, b, c)
 * 
 * 变换公式：
 *   a = alpha
 *   b = (-alpha + √3*beta) / 2
 *   c = (-alpha - √3*beta) / 2
 */
void FOC_Inverse_Clarke_Transform(const FOC_AlphaBeta_t *alphabeta, FOC_ABC_t *abc)
{
    abc->a = alphabeta->alpha;
    abc->b = (-alphabeta->alpha + FOC_SQRT3 * alphabeta->beta) * 0.5f;
    abc->c = (-alphabeta->alpha - FOC_SQRT3 * alphabeta->beta) * 0.5f;
}

/**
 * @brief Park变换 (两相静止坐标系 → 两相旋转坐标系)
 * @param alphabeta 输入：两相静止坐标系 (alpha, beta)
 * @param sin_theta sin(电角度)
 * @param cos_theta cos(电角度)
 * @param dq 输出：两相旋转坐标系 (d, q)
 * 
 * 变换公式：
 *   d =  alpha * cos(theta) + beta * sin(theta)
 *   q = -alpha * sin(theta) + beta * cos(theta)
 */
void FOC_Park_Transform(const FOC_AlphaBeta_t *alphabeta, float sin_theta, float cos_theta, FOC_DQ_t *dq)
{
    dq->d = alphabeta->alpha * cos_theta + alphabeta->beta * sin_theta;
    dq->q = -alphabeta->alpha * sin_theta + alphabeta->beta * cos_theta;
}

/**
 * @brief 反Park变换 (两相旋转坐标系 → 两相静止坐标系)
 * @param dq 输入：两相旋转坐标系 (d, q)
 * @param sin_theta sin(电角度)
 * @param cos_theta cos(电角度)
 * @param alphabeta 输出：两相静止坐标系 (alpha, beta)
 * 
 * 变换公式：
 *   alpha = d * cos(theta) - q * sin(theta)
 *   beta  = d * sin(theta) + q * cos(theta)
 */
void FOC_Inverse_Park_Transform(const FOC_DQ_t *dq, float sin_theta, float cos_theta, FOC_AlphaBeta_t *alphabeta)
{
    alphabeta->alpha = dq->d * cos_theta - dq->q * sin_theta;
    alphabeta->beta = dq->d * sin_theta + dq->q * cos_theta;
}

/**
 * @brief SVPWM生成 - 零序注入法
 * @param ValphaBeta 输入：两相静止坐标系电压 (alpha, beta)
 * @param Vbus 母线电压
 * @param svpwm 输出：SVPWM占空比
 * 
 * 【算法更新】使用零序注入法替代扇区法
 * 优势：
 * - 计算简单：仅需加减和比较，无需三角函数
 * - 实时性好：计算量小，适合高开关频率
 * - 完全等效：与标准SVPWM输出特性相同
 * 
 * 实现步骤：
 * 1. 反Clark变换得到三相电压指令ua,ub,uc
 * 2. 计算零序分量 u0 = -0.5*(max(ua,ub,uc)+min(ua,ub,uc))
 * 3. 注入零序分量得到调制波
 * 4. 转换为PWM占空比(0~1)
 */
void FOC_SVPWM_Generate(const FOC_AlphaBeta_t *ValphaBeta, float Vbus, FOC_SVPWM_t *svpwm)
{
    FOC_ABC_t v_abc;
    float max_u, min_u, u0;
    float ma, mb, mc;
    float VphaseNorm;
    
    /* 归一化基准：调制波 m=1 对应桥臂相电压幅值 Vbus/2 */
    VphaseNorm = Vbus * 0.5f;
    
    /* 反Clark变换得到三相电压指令(归一化到-1~1) */
    /* 先得到未归一化的三相电压 */
    v_abc.a = ValphaBeta->alpha;
    v_abc.b = (-ValphaBeta->alpha + FOC_SQRT3 * ValphaBeta->beta) * 0.5f;
    v_abc.c = (-ValphaBeta->alpha - FOC_SQRT3 * ValphaBeta->beta) * 0.5f;
    
    /* 归一化到-1~1范围 */
    if (VphaseNorm > 0.0f) {
        v_abc.a /= VphaseNorm;
        v_abc.b /= VphaseNorm;
        v_abc.c /= VphaseNorm;
    }
    
    /* 零序注入算法 */
    /* 步骤1：求取最大值和最小值 */
    max_u = v_abc.a;
    if (v_abc.b > max_u) max_u = v_abc.b;
    if (v_abc.c > max_u) max_u = v_abc.c;
    
    min_u = v_abc.a;
    if (v_abc.b < min_u) min_u = v_abc.b;
    if (v_abc.c < min_u) min_u = v_abc.c;
    
    /* 步骤2：计算零序分量 u0 = -0.5*(max_u + min_u) */
    u0 = -0.5f * (max_u + min_u);
    
    /* 步骤3：注入零序分量 */
    ma = v_abc.a + u0;
    mb = v_abc.b + u0;
    mc = v_abc.c + u0;
    
    /* 步骤4：转换为PWM占空比(0~1)并限幅 */
    /* 调制波范围-1~1，映射到PWM占空比0~1 */
    svpwm->Ta = FOC_Saturate((ma + 1.0f) * 0.5f, 1.0f, 0.0f);
    svpwm->Tb = FOC_Saturate((mb + 1.0f) * 0.5f, 1.0f, 0.0f);
    svpwm->Tc = FOC_Saturate((mc + 1.0f) * 0.5f, 1.0f, 0.0f);
    
    /* 扇区信息保留用于调试（通过调制波符号判断）*/
    /* sector: 1-6对应不同扇区，0表示零矢量区域 */
    if (max_u - min_u < 0.001f) {
        svpwm->sector = 0;  /* 零矢量区域 */
    } else {
        /* 根据最大值的相位判断扇区 */
        if (v_abc.a == max_u) {
            svpwm->sector = (v_abc.b > v_abc.c) ? 1 : 6;
        } else if (v_abc.b == max_u) {
            svpwm->sector = (v_abc.c > v_abc.a) ? 3 : 2;
        } else {
            svpwm->sector = (v_abc.a > v_abc.b) ? 5 : 4;
        }
    }
}

/**
 * @brief PI控制器初始化
 * @param pi PI控制器结构体指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param output_max 输出上限
 * @param output_min 输出下限
 */
void FOC_PI_Init(FOC_PI_Controller_t *pi, float Kp, float Ki, float output_max, float output_min)
{
    float output_span, sep_output;

    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->integral = 0.0f;
    pi->output_max = output_max;
    pi->output_min = output_min;
    pi->integral_max = output_max * 0.9f;  /* 积分限幅略小于输出限幅 */

    /* 各环独立积分分离阈值：按输出限幅和Kp换算到误差域 */
    output_span = fabsf(output_max - output_min);
    sep_output = output_span * 0.2f;
    if (fabsf(Kp) > FOC_EPSILON) {
        pi->integral_sep_thresh = sep_output / fabsf(Kp);
    } else {
        pi->integral_sep_thresh = 1e6f;
    }
    if (pi->integral_sep_thresh < FOC_EPSILON) {
        pi->integral_sep_thresh = FOC_EPSILON;
    }
}

/**
 * @brief PI控制器更新（带积分分离）
 * @param pi PI控制器结构体指针
 * @param error 误差值 (参考值 - 反馈值)
 * @return 控制器输出
 * 
 * 【改进】添加积分分离功能：
 * 当误差较大时暂停积分，防止积分饱和和超调
 */
float FOC_PI_Update(FOC_PI_Controller_t *pi, float error)
{
    /* 积分项更新（带积分分离） */
    if (fabsf(error) < pi->integral_sep_thresh) {
        pi->integral += error;
    }
    /* 积分限幅（抗积分饱和） */
    pi->integral = FOC_Saturate(pi->integral, pi->integral_max, -pi->integral_max);
    
    /* PI计算 */
    float output = pi->Kp * error + pi->Ki * pi->integral;
    
    /* 输出限幅 */
    output = FOC_Saturate(output, pi->output_max, pi->output_min);
    
    return output;
}

/**
 * @brief FOC初始化
 * @param foc FOC句柄指针
 * @param Kp_d D轴PI比例增益
 * @param Ki_d D轴PI积分增益
 * @param Kp_q Q轴PI比例增益
 * @param Ki_q Q轴PI积分增益
 */
void FOC_Init(FOC_Handle_t *foc, float Kp_d, float Ki_d, float Kp_q, float Ki_q)
{
    /* 清零结构体 */
    memset(foc, 0, sizeof(FOC_Handle_t));
    
    /* 初始化PI控制器 */
    /* 电压矢量限幅：|Vdq| <= Vbus/√3 (SVPWM线性区) */
    float Vmax = 24.0f / FOC_SQRT3;  /* 假设默认Vbus=24V */
    FOC_PI_Init(&foc->pi_d, Kp_d, Ki_d, Vmax, -Vmax);
    FOC_PI_Init(&foc->pi_q, Kp_q, Ki_q, Vmax, -Vmax);
    
    /* 默认参考值 */
    foc->Id_ref = 0.0f;
    foc->Iq_ref = 0.0f;
    
    /* 默认母线电压 */
    foc->Vbus = 24.0f;
    
    foc->enabled = 1;
}

/**
 * @brief 设置电流参考值
 * @param foc FOC句柄指针
 * @param Id_ref D轴电流参考值
 * @param Iq_ref Q轴电流参考值
 */
void FOC_SetCurrentReference(FOC_Handle_t *foc, float Id_ref, float Iq_ref)
{
    foc->Id_ref = Id_ref;
    foc->Iq_ref = Iq_ref;
}

/**
 * @brief 设置电角度
 * @param foc FOC句柄指针
 * @param theta_elec 电角度 (rad)
 */
void FOC_SetAngle(FOC_Handle_t *foc, float theta_elec)
{
    foc->theta_elec = FOC_AngleNormalize(theta_elec);
    foc->sin_theta = sinf(foc->theta_elec);
    foc->cos_theta = cosf(foc->theta_elec);
}

/**
 * @brief 设置母线电压
 * @param foc FOC句柄指针
 * @param Vbus 母线电压 (V)
 */
void FOC_SetVbus(FOC_Handle_t *foc, float Vbus)
{
    foc->Vbus = Vbus;
    
    /* 更新PI输出电压矢量限幅（SVPWM线性区） */
    float Vmax = Vbus / FOC_SQRT3;
    foc->pi_d.output_max = Vmax;
    foc->pi_d.output_min = -Vmax;
    foc->pi_q.output_max = Vmax;
    foc->pi_q.output_min = -Vmax;
    foc->pi_d.integral_max = Vmax * 0.9f;
    foc->pi_q.integral_max = Vmax * 0.9f;
}

/**
 * @brief 更新电流反馈
 * @param foc FOC句柄指针
 * @param Ia A相电流 (A)
 * @param Ib B相电流 (A)
 * @param Ic C相电流 (A)
 */
void FOC_UpdateCurrent(FOC_Handle_t *foc, float Ia, float Ib, float Ic)
{
    foc->Iabc.a = Ia;
    foc->Iabc.b = Ib;
    foc->Iabc.c = Ic;
}

/**
 * @brief 执行FOC控制算法
 * @param foc FOC句柄指针
 * 
 * 执行流程：
 * 1. Clark变换 (ABC → AlphaBeta)
 * 2. Park变换 (AlphaBeta → DQ)
 * 3. PI控制器计算 (DQ电压)
 * 4. 反Park变换 (DQ → AlphaBeta)
 * 5. SVPWM生成（零序注入法）
 */
void FOC_Run(FOC_Handle_t *foc)
{
    float error_d, error_q;
    float vd_cmd, vq_cmd;
    float vd_sat, vq_sat;
    float Vmax, v_mag;

    if (!foc->enabled) return;
    
    /* Step 1: Clark变换 */
    FOC_Clarke_Transform(&foc->Iabc, &foc->IalphaBeta);
    
    /* Step 2: Park变换 */
    FOC_Park_Transform(&foc->IalphaBeta, foc->sin_theta, foc->cos_theta, &foc->Idq);
    
    /* Step 3: PI控制器计算DQ轴电压 */
    error_d = foc->Id_ref - foc->Idq.d;
    error_q = foc->Iq_ref - foc->Idq.q;
    vd_cmd = FOC_PI_Update(&foc->pi_d, error_d);
    vq_cmd = FOC_PI_Update(&foc->pi_q, error_q);

    /* Step 3.5: 电压矢量限幅（Vd/Vq联合限幅）+ 反算抗积分饱和 */
    vd_sat = vd_cmd;
    vq_sat = vq_cmd;
    Vmax = foc->pi_d.output_max;
    v_mag = sqrtf(vd_cmd * vd_cmd + vq_cmd * vq_cmd);
    if ((Vmax > FOC_EPSILON) && (v_mag > Vmax)) {
        float scale = Vmax / v_mag;
        vd_sat = vd_cmd * scale;
        vq_sat = vq_cmd * scale;

        if (fabsf(foc->pi_d.Ki) > FOC_EPSILON) {
            foc->pi_d.integral += (FOC_CURRENT_AW_GAIN * (vd_sat - vd_cmd)) / foc->pi_d.Ki;
            foc->pi_d.integral = FOC_Saturate(foc->pi_d.integral, foc->pi_d.integral_max, -foc->pi_d.integral_max);
        }
        if (fabsf(foc->pi_q.Ki) > FOC_EPSILON) {
            foc->pi_q.integral += (FOC_CURRENT_AW_GAIN * (vq_sat - vq_cmd)) / foc->pi_q.Ki;
            foc->pi_q.integral = FOC_Saturate(foc->pi_q.integral, foc->pi_q.integral_max, -foc->pi_q.integral_max);
        }
    }

    foc->Vdq.d = vd_sat;
    foc->Vdq.q = vq_sat;
    
    /* Step 4: 反Park变换 */
    FOC_Inverse_Park_Transform(&foc->Vdq, foc->sin_theta, foc->cos_theta, &foc->ValphaBeta);
    
    /* Step 5: SVPWM生成（零序注入法） */
    FOC_SVPWM_Generate(&foc->ValphaBeta, foc->Vbus, &foc->svpwm);
}

/**
 * @brief 获取PWM占空比寄存器值
 * @param foc FOC句柄指针
 * @param pwm_a A相PWM输出
 * @param pwm_b B相PWM输出
 * @param pwm_c C相PWM输出
 * @param pwm_period PWM周期值 (ARR寄存器值)
 */
void FOC_GetPWM(FOC_Handle_t *foc, uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c, uint16_t pwm_period)
{
    *pwm_a = (uint16_t)(foc->svpwm.Ta * pwm_period);
    *pwm_b = (uint16_t)(foc->svpwm.Tb * pwm_period);
    *pwm_c = (uint16_t)(foc->svpwm.Tc * pwm_period);
}

/**
 * @brief 获取零序注入后的三相调制波
 * @param foc FOC句柄指针
 * @param ma A相调制波输出(-1~1)
 * @param mb B相调制波输出(-1~1)
 * @param mc C相调制波输出(-1~1)
 * 
 * 【新增】用于调试和监控
 */
void FOC_GetModulationWave(const FOC_Handle_t *foc, float *ma, float *mb, float *mc)
{
    /* 将占空比(0~1)转换回调制波(-1~1) */
    *ma = foc->svpwm.Ta * 2.0f - 1.0f;
    *mb = foc->svpwm.Tb * 2.0f - 1.0f;
    *mc = foc->svpwm.Tc * 2.0f - 1.0f;
}
