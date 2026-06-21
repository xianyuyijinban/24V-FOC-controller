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
#define FOC_ZERO_CURRENT_REF_EPS      0.002f
#define FOC_ZERO_CURRENT_FEEDBACK_EPS 0.020f

static float FOC_PI_IntegralLimit(float output_max, float Ki)
{
    if (fabsf(Ki) > FOC_EPSILON) {
        return (fabsf(output_max) * 0.9f) / fabsf(Ki);
    }

    return 0.0f;
}

/**
 * @brief Clark变换 (三相静止坐标系 → 两相静止坐标系)
 * @param abc 输入：三相电流/电压 (a, b, c)
 * @param alphabeta 输出：两相静止坐标系 (alpha, beta)
 * @note 使用等幅值变换
 * 
 * 等幅值Clark变换公式（三电流形式）：
 *   alpha = (2a - b - c) / 3
 *   beta  = (b - c) / √3
 *
 * 与两电流公式在三相平衡时等价，但不会在低边采样窗口异常或
 * C相电流显著而A/B较小时把真实电流丢掉。
 */
void FOC_Clarke_Transform(const FOC_ABC_t *abc, FOC_AlphaBeta_t *alphabeta)
{
    alphabeta->alpha = (2.0f * abc->a - abc->b - abc->c) / 3.0f;
    alphabeta->beta = (abc->b - abc->c) / FOC_SQRT3;
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
 * @brief SVPWM生成 - 从ABC三相电压直接计算（跳过InvClarke）
 * @param Vabc 输入：三相电压指令 (V)，已归一化到相电压幅值
 * @param Vbus 母线电压
 * @param svpwm 输出：SVPWM占空比
 */
void FOC_SVPWM_GenerateFromABC(const FOC_ABC_t *Vabc, float Vbus, FOC_SVPWM_t *svpwm)
{
    FOC_ABC_t v_norm;
    float max_u, min_u, u0;
    float ma, mb, mc;
    float VphaseNorm;

    VphaseNorm = Vbus * 0.5f;
    if (VphaseNorm < 0.01f) { VphaseNorm = 0.01f; }

    /* 归一化：相电压 → 调制波 [-1,1] */
    v_norm.a = Vabc->a / VphaseNorm;
    v_norm.b = Vabc->b / VphaseNorm;
    v_norm.c = Vabc->c / VphaseNorm;

    /* 零序注入 */
    max_u = v_norm.a;
    if (v_norm.b > max_u) max_u = v_norm.b;
    if (v_norm.c > max_u) max_u = v_norm.c;
    min_u = v_norm.a;
    if (v_norm.b < min_u) min_u = v_norm.b;
    if (v_norm.c < min_u) min_u = v_norm.c;
    u0 = -0.5f * (max_u + min_u);
    ma = v_norm.a + u0;
    mb = v_norm.b + u0;
    mc = v_norm.c + u0;

    /* 转换为PWM占空比(0~1) */
    svpwm->Ta = FOC_Saturate((ma + 1.0f) * 0.5f, 1.0f, 0.0f);
    svpwm->Tb = FOC_Saturate((mb + 1.0f) * 0.5f, 1.0f, 0.0f);
    svpwm->Tc = FOC_Saturate((mc + 1.0f) * 0.5f, 1.0f, 0.0f);

    /* 扇区 */
    if (max_u - min_u < 0.001f) {
        svpwm->sector = 0;
    } else {
        if (v_norm.a == max_u) {
            svpwm->sector = (v_norm.b > v_norm.c) ? 1 : 6;
        } else if (v_norm.b == max_u) {
            svpwm->sector = (v_norm.c > v_norm.a) ? 3 : 2;
        } else {
            svpwm->sector = (v_norm.a > v_norm.b) ? 5 : 4;
        }
    }
}

/**
 * @brief 设置 RsFF 路径模式
 * @param foc FOC句柄指针
 * @param mode 0=OFF, 1=DQ(默认), 2=ABC
 */
void FOC_SetRsFFMode(FOC_Handle_t *foc, uint8_t mode)
{
    if (foc == NULL) return;
    if (mode > 2U) return;
    foc->rs_ff_mode = mode;
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
    pi->integral_max = FOC_PI_IntegralLimit(output_max, Ki);

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
    foc->diag_sat_ratio = 1.0f;
    /* 自适应 Rs 前馈默认值 */
    foc->rs_ff_adaptive = 0U;                   /* 默认关闭，待PI基线验证后启用 */
    foc->rs_ff_sign_protect = 0U;                /* 配合RsFF关闭 */
    foc->rs_ff_sign_blocked = 0U;
    foc->rs_ff_sign_mismatch_count = 0U;
    foc->rs_ff_mode = FOC_RS_FF_MODE_DQ;          /* post-ARR baseline: DQ域 RsFF * 0.20 */
    foc->rs_ff_confidence = FOC_RS_FF_CONF_INIT;
    foc->rs_ff_confidence_lpf = FOC_RS_FF_CONF_INIT;
    foc->rs_ff_dIq_ref_prev = 0.0f;
    foc->rs_ff_speed_error = 0.0f;
    foc->rs_ff_diag_raw = FOC_RS_FF_CONF_INIT;
    
    /* 初始化PI控制器 */
    /* 电压矢量限幅：|Vdq| <= Vbus/√3 (SVPWM线性区) */
    float Vmax = foc->Vbus / FOC_SQRT3;
    if (Vmax < 1.0f) { Vmax = 24.0f / FOC_SQRT3; }  /* Vbus未设置时的fallback */
    FOC_PI_Init(&foc->pi_d, Kp_d, Ki_d, Vmax, -Vmax);
    FOC_PI_Init(&foc->pi_q, Kp_q, Ki_q, Vmax, -Vmax);
    
    /* 默认参考值 */
    foc->Id_ref = 0.0f;
    foc->Iq_ref = 0.0f;
    
    /* 默认母线电压 */
    foc->Vbus = 24.0f;
    foc->current_resistance_ohm = 0.0f;
    foc->rs_ff_scale = 0.20f;       /* post-ARR baseline: 最低有效scale（0.10欠跟踪，0.30零回残留大） */
    foc->bemf_Ke_temp = 0.0f;       /* 0 = 使用默认Ke */
    foc->bemf_user_enable = 0U;     /* 默认关闭，需用户显式使能 */
    foc->bemf_blocked = 0U;
    
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

void FOC_SetCurrentResistance(FOC_Handle_t *foc, float resistance_ohm)
{
    if (resistance_ohm < 0.0f) {
        resistance_ohm = 0.0f;
    }
    foc->current_resistance_ohm = resistance_ohm;
}

/**
 * @brief 设置自适应 Rs 前馈开关
 * @param foc FOC句柄指针
 * @param enabled 0=关闭（使用裸rs_ff_scale），1=开启（rs_ff_scale × confidence）
 */
void FOC_SetRsFFAdaptive(FOC_Handle_t *foc, uint8_t enabled)
{
    if (foc == NULL) return;
    foc->rs_ff_adaptive = (enabled != 0U) ? 1U : 0U;
    if (!foc->rs_ff_adaptive) {
        /* 关闭自适应：置信度复位为1.0，行为等同传统裸 rs_ff_scale */
        foc->rs_ff_confidence = 1.0f;
        foc->rs_ff_confidence_lpf = 1.0f;
        foc->rs_ff_diag_raw = 1.0f;
    }
}

/**
 * @brief 查询自适应 Rs 前馈状态
 * @param foc FOC句柄指针
 * @return 0=关闭，1=开启
 */
uint8_t FOC_GetRsFFAdaptive(const FOC_Handle_t *foc)
{
    return (foc != NULL) ? foc->rs_ff_adaptive : 0U;
}

/**
 * @brief 设置速度环误差（供自适应 Rs 前馈置信度计算）
 * @param foc FOC句柄指针
 * @param speed_error 速度环误差 (rad/s)，力矩模式传0.0f
 */
/**
 * @brief 设置 RsFF 符号反转保护开关
 * @param foc FOC句柄指针
 * @param enabled 0=关闭（诊断），1=开启（默认）
 */
void FOC_SetRsFFSignProtect(FOC_Handle_t *foc, uint8_t enabled)
{
    if (foc == NULL) return;
    foc->rs_ff_sign_protect = (enabled != 0U) ? 1U : 0U;
}

void FOC_SetRsFFSpeedError(FOC_Handle_t *foc, float speed_error)
{
    if (foc == NULL) return;
    foc->rs_ff_speed_error = speed_error;
}

/**
 * @brief 更新自适应 Rs 前馈置信度（每个FOC周期调用一次）
 * @param foc FOC句柄指针
 *
 * 4个乘性因子计算原始置信度：
 *   1. |dIq_ref/dt| > 0.5 A/s → 0.5/dIq_dt
 *   2. |speed_error| > 0.1 rad/s → ×0.30
 *   3. 上周期sat_ratio < 0.95 → ×sat_ratio
 *   4. Iq_ref·Iq_fb < 0 且 |Iq_ref| > 0.01A → 立即归零
 * 非对称LPF：瞬时下降，~300ms缓慢恢复。
 */
void FOC_UpdateRsFFConfidence(FOC_Handle_t *foc)
{
    float raw_confidence;
    float dIq_dt_abs;

    if (foc == NULL) return;

    if (!foc->rs_ff_adaptive) {
        foc->rs_ff_confidence = 1.0f;
        foc->rs_ff_confidence_lpf = 1.0f;
        foc->rs_ff_diag_raw = 1.0f;
        return;
    }

    /* Factor 4: Iq_ref与Iq反馈符号相反（幅值门限 + 连续计数 ~3ms） */
    if (foc->rs_ff_sign_protect &&
        (foc->Iq_ref * foc->Idq.q < 0.0f) &&
        (fabsf(foc->Iq_ref) > FOC_RS_FF_SIGN_REF_THRESH) &&
        (fabsf(foc->Idq.q) > FOC_RS_FF_SIGN_FB_THRESH)) {
        foc->rs_ff_sign_mismatch_count++;
        if (foc->rs_ff_sign_mismatch_count >= FOC_RS_FF_SIGN_HOLD_CYCLES) {
            foc->rs_ff_sign_blocked = 1U;
            foc->rs_ff_confidence_lpf = 0.0f;
            foc->rs_ff_confidence = 0.0f;
            foc->rs_ff_diag_raw = 0.0f;
            foc->rs_ff_dIq_ref_prev = foc->Iq_ref;
            return;
        }
    } else {
        foc->rs_ff_sign_mismatch_count = 0U;
        foc->rs_ff_sign_blocked = 0U;
    }

    raw_confidence = 1.0f;

    /* Factor 1: |dIq_ref/dt| 过大 → 按比例降权 */
    dIq_dt_abs = fabsf((foc->Iq_ref - foc->rs_ff_dIq_ref_prev) * (float)FOC_CONTROL_FREQ);
    foc->rs_ff_dIq_ref_prev = foc->Iq_ref;
    if (dIq_dt_abs > FOC_RS_FF_DIQDT_THRESH) {
        raw_confidence *= FOC_RS_FF_DIQDT_THRESH / dIq_dt_abs;
    }

    /* Factor 2: 速度误差过大 → 乘惩罚系数 */
    if (fabsf(foc->rs_ff_speed_error) > FOC_RS_FF_SPEED_ERR_THRESH) {
        raw_confidence *= FOC_RS_FF_SPEED_ERR_FACTOR;
    }

    /* Factor 3: 上周期电压饱和 → 乘 sat_ratio */
    if (foc->diag_sat_ratio < FOC_RS_FF_SAT_RATIO_THRESH) {
        raw_confidence *= foc->diag_sat_ratio;
    }

    /* 钳位 */
    if (raw_confidence < 0.0f) raw_confidence = 0.0f;
    if (raw_confidence > 1.0f) raw_confidence = 1.0f;

    foc->rs_ff_diag_raw = raw_confidence;

    /* 非对称LPF：下降瞬时跟踪，上升按300ms时间常数缓慢恢复 */
    if (raw_confidence < foc->rs_ff_confidence_lpf) {
        foc->rs_ff_confidence_lpf = raw_confidence;
    } else {
        foc->rs_ff_confidence_lpf += FOC_RS_FF_RECOVER_ALPHA *
                                     (raw_confidence - foc->rs_ff_confidence_lpf);
    }

    foc->rs_ff_confidence = foc->rs_ff_confidence_lpf;
}

/**
 * @brief 设置BEMF解耦前馈参数
 * @param foc FOC句柄指针
 * @param Ld d轴电感 (H)
 * @param Lq q轴电感 (H)
 * @param Ke 反电动势常数 V/(rad/s)
 */
void FOC_SetBemfParams(FOC_Handle_t *foc, float Ld, float Lq, float Ke)
{
    foc->bemf_Ld = (Ld > 0.0f) ? Ld : 0.0f;
    foc->bemf_Lq = (Lq > 0.0f) ? Lq : 0.0f;
    foc->bemf_Ke = (Ke >= 0.0f) ? Ke : 0.0f;
    foc->bemf_enabled = (foc->bemf_Ld > 0.0f && foc->bemf_Lq > 0.0f) ? 1U : 0U;
}

/**
 * @brief 设置电角速度（用于BEMF解耦）
 * @param foc FOC句柄指针
 * @param omega_elec_radps 电角速度 (rad/s)
 */
void FOC_SetOmegaElec(FOC_Handle_t *foc, float omega_elec_radps)
{
    foc->omega_elec_radps = omega_elec_radps;
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
    foc->pi_d.integral_max = FOC_PI_IntegralLimit(Vmax, foc->pi_d.Ki);
    foc->pi_q.integral_max = FOC_PI_IntegralLimit(Vmax, foc->pi_q.Ki);
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

void FOC_RegenerateVoltageVector(FOC_Handle_t *foc)
{
    if (!foc->enabled) {
        return;
    }

    FOC_Inverse_Park_Transform(&foc->Vdq, foc->sin_theta, foc->cos_theta, &foc->ValphaBeta);
    FOC_SVPWM_Generate(&foc->ValphaBeta, foc->Vbus, &foc->svpwm);
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

    /* Step 0: 更新自适应 Rs 前馈置信度（基于上一拍状态） */
    FOC_UpdateRsFFConfidence(foc);

    /* Step 1: Clark变换 */
    FOC_Clarke_Transform(&foc->Iabc, &foc->IalphaBeta);
    
    /* Step 2: Park变换 */
    FOC_Park_Transform(&foc->IalphaBeta, foc->sin_theta, foc->cos_theta, &foc->Idq);
    
    /* Step 3: PI控制器计算DQ轴电压 */
    error_d = foc->Id_ref - foc->Idq.d;
    error_q = foc->Iq_ref - foc->Idq.q;
    if ((fabsf(foc->Id_ref) <= FOC_ZERO_CURRENT_REF_EPS) &&
        (fabsf(foc->Iq_ref) <= FOC_ZERO_CURRENT_REF_EPS) &&
        (fabsf(foc->Idq.d) <= FOC_ZERO_CURRENT_FEEDBACK_EPS) &&
        (fabsf(foc->Idq.q) <= FOC_ZERO_CURRENT_FEEDBACK_EPS)) {
        foc->pi_d.integral = 0.0f;
        foc->pi_q.integral = 0.0f;
        foc->Vdq.d = 0.0f;
        foc->Vdq.q = 0.0f;
        foc->ValphaBeta.alpha = 0.0f;
        foc->ValphaBeta.beta = 0.0f;
        /* 清零电流环诊断 */
        foc->diag_vd_rs_ff = 0.0f; foc->diag_vq_rs_ff = 0.0f;
        foc->diag_vd_pi = 0.0f; foc->diag_vq_pi = 0.0f;
        foc->diag_vd_bemf = 0.0f; foc->diag_vq_bemf = 0.0f;
        foc->diag_vd_cmd = 0.0f; foc->diag_vq_cmd = 0.0f;
        foc->diag_v_mag = 0.0f; foc->diag_sat_ratio = 1.0f;
        foc->diag_Va_pi = 0.0f; foc->diag_Vb_pi = 0.0f; foc->diag_Vc_pi = 0.0f;
        foc->diag_Va_rs_ff = 0.0f; foc->diag_Vb_rs_ff = 0.0f; foc->diag_Vc_rs_ff = 0.0f;
        foc->diag_Va_cmd = 0.0f; foc->diag_Vb_cmd = 0.0f; foc->diag_Vc_cmd = 0.0f;
        foc->diag_Ia_ref = 0.0f; foc->diag_Ib_ref = 0.0f; foc->diag_Ic_ref = 0.0f;
        FOC_SVPWM_Generate(&foc->ValphaBeta, foc->Vbus, &foc->svpwm);
        return;
    }

    /* Step 3.1: PI 输出（RsFF在后续分支中处理） */
    {
        float vd_pi = FOC_PI_Update(&foc->pi_d, error_d);
        float vq_pi = FOC_PI_Update(&foc->pi_q, error_q);

        foc->diag_vd_pi = vd_pi;
        foc->diag_vq_pi = vq_pi;

        /* 先不加RsFF，后续按模式分支处理 */
        vd_cmd = vd_pi;
        vq_cmd = vq_pi;
    }

    /* Step 3.2: BEMF解耦前馈（P1，DQ域，各模式共用） */
    {
        float vd_bemf = 0.0f;
        float vq_bemf = 0.0f;
#if FOC_FF_ENABLE_BEMF
        {
            float ke_used = (foc->bemf_Ke_temp > 0.0f) ? foc->bemf_Ke_temp : foc->bemf_Ke;
            float omega_e = foc->omega_elec_radps;
            float bemf_limit = 0.8f * foc->Vbus / FOC_SQRT3;

            foc->bemf_blocked = 0U;

            if (foc->bemf_user_enable && foc->bemf_enabled && (ke_used > FOC_EPSILON)) {
                vd_bemf = -omega_e * foc->bemf_Lq * foc->Iq_ref;
                vq_bemf =  omega_e * (foc->bemf_Ld * foc->Id_ref + ke_used);

                if (fabsf(vq_bemf) > bemf_limit || fabsf(vd_bemf) > bemf_limit) {
                    foc->bemf_blocked = 1U;
                    vd_bemf = 0.0f;
                    vq_bemf = 0.0f;
                }
            }
        }
#endif
        foc->diag_vd_bemf = vd_bemf;
        foc->diag_vq_bemf = vq_bemf;

        /* BEMF 加在 PI 输出上（DQ域，各模式共用） */
        vd_cmd += vd_bemf;
        vq_cmd += vq_bemf;
    }

    /* Step 3.3: RsFF 分支 —— DQ域 或 ABC域 */
    {
        float rs_eff;
        if (foc->rs_ff_mode == FOC_RS_FF_MODE_OFF) {
            rs_eff = 0.0f;
        } else {
            rs_eff = foc->rs_ff_adaptive
                ? (foc->rs_ff_scale * foc->rs_ff_confidence)
                : foc->rs_ff_scale;
        }

        if (foc->rs_ff_mode == FOC_RS_FF_MODE_ABC) {
            /* ====== ABC 域 Rs 前馈 ====== */
            FOC_DQ_t   Vdq_pre;
            FOC_AlphaBeta_t Vab_pi;
            FOC_ABC_t  Vabc_pi, Iabc_ref, Vabc_ff, Vabc_cmd;
            float Rs = foc->current_resistance_ohm;
            float vclamp_ff = FOC_RS_FF_ABC_VCLAMP_RATIO * foc->Vbus;
            float vclamp_total;
            uint8_t any_clamped;

            /* 1. Vdq_pre(PI+BEMF) → ABC */
            Vdq_pre.d = vd_cmd;
            Vdq_pre.q = vq_cmd;
            FOC_Inverse_Park_Transform(&Vdq_pre, foc->sin_theta, foc->cos_theta, &Vab_pi);
            FOC_Inverse_Clarke_Transform(&Vab_pi, &Vabc_pi);

            /* 2. Idq_ref → Iabc_ref */
            {
                FOC_DQ_t   Idq_ref = {foc->Id_ref, foc->Iq_ref};
                FOC_AlphaBeta_t Iab_ref;
                FOC_Inverse_Park_Transform(&Idq_ref, foc->sin_theta, foc->cos_theta, &Iab_ref);
                FOC_Inverse_Clarke_Transform(&Iab_ref, &Iabc_ref);
            }

            /* 3. Vabc_ff = Rs * Iabc_ref * rs_eff，逐相限幅 */
            Vabc_ff.a = FOC_Saturate(Rs * Iabc_ref.a * rs_eff, vclamp_ff, -vclamp_ff);
            Vabc_ff.b = FOC_Saturate(Rs * Iabc_ref.b * rs_eff, vclamp_ff, -vclamp_ff);
            Vabc_ff.c = FOC_Saturate(Rs * Iabc_ref.c * rs_eff, vclamp_ff, -vclamp_ff);

            /* 4. ABC 诊断 */
            foc->diag_Va_pi = Vabc_pi.a; foc->diag_Vb_pi = Vabc_pi.b; foc->diag_Vc_pi = Vabc_pi.c;
            foc->diag_Va_rs_ff = Vabc_ff.a; foc->diag_Vb_rs_ff = Vabc_ff.b; foc->diag_Vc_rs_ff = Vabc_ff.c;
            foc->diag_Ia_ref = Iabc_ref.a; foc->diag_Ib_ref = Iabc_ref.b; foc->diag_Ic_ref = Iabc_ref.c;

            /* DQ RsFF 诊断置零（ABC模式不使用DQ前馈） */
            foc->diag_vd_rs_ff = 0.0f;
            foc->diag_vq_rs_ff = 0.0f;

            /* 5. Vabc_cmd = Vabc_pi + Vabc_ff */
            Vabc_cmd.a = Vabc_pi.a + Vabc_ff.a;
            Vabc_cmd.b = Vabc_pi.b + Vabc_ff.b;
            Vabc_cmd.c = Vabc_pi.c + Vabc_ff.c;

            /* 6. 逐相总限幅：|V_phase| ≤ Vbus/√3 */
            vclamp_total = foc->Vbus / FOC_SQRT3;
            any_clamped = 0U;
            if (fabsf(Vabc_cmd.a) > vclamp_total) { Vabc_cmd.a = (Vabc_cmd.a > 0.0f) ? vclamp_total : -vclamp_total; any_clamped = 1U; }
            if (fabsf(Vabc_cmd.b) > vclamp_total) { Vabc_cmd.b = (Vabc_cmd.b > 0.0f) ? vclamp_total : -vclamp_total; any_clamped = 1U; }
            if (fabsf(Vabc_cmd.c) > vclamp_total) { Vabc_cmd.c = (Vabc_cmd.c > 0.0f) ? vclamp_total : -vclamp_total; any_clamped = 1U; }

            foc->diag_Va_cmd = Vabc_cmd.a; foc->diag_Vb_cmd = Vabc_cmd.b; foc->diag_Vc_cmd = Vabc_cmd.c;

            /* DQ 诊断兼容 */
            foc->diag_vd_cmd = Vdq_pre.d;
            foc->diag_vq_cmd = Vdq_pre.q;
            foc->diag_v_mag = sqrtf(Vdq_pre.d * Vdq_pre.d + Vdq_pre.q * Vdq_pre.q);
            foc->diag_sat_ratio = any_clamped ? 0.9f : 1.0f;

            /* 7. 抗积分饱和：逐相限幅时缩放积分 */
            if (any_clamped && (fabsf(foc->pi_d.Ki) > FOC_EPSILON || fabsf(foc->pi_q.Ki) > FOC_EPSILON)) {
                /* 简化处理：限幅时冻结积分（ABC域不易反算精确退卷量） */
                if (fabsf(foc->pi_d.Ki) > FOC_EPSILON) {
                    foc->pi_d.integral = FOC_Saturate(foc->pi_d.integral * 0.9f,
                                                      foc->pi_d.integral_max, -foc->pi_d.integral_max);
                }
                if (fabsf(foc->pi_q.Ki) > FOC_EPSILON) {
                    foc->pi_q.integral = FOC_Saturate(foc->pi_q.integral * 0.9f,
                                                      foc->pi_q.integral_max, -foc->pi_q.integral_max);
                }
            }

            /* 8. SVPWM from ABC */
            foc->Vdq.d = 0.0f;
            foc->Vdq.q = 0.0f;
            foc->ValphaBeta.alpha = Vab_pi.alpha;
            foc->ValphaBeta.beta = Vab_pi.beta;
            FOC_SVPWM_GenerateFromABC(&Vabc_cmd, foc->Vbus, &foc->svpwm);

        } else {
            /* ====== DQ 域 Rs 前馈（mode=0=OFF, mode=1=DQ） ====== */
            float vd_rs_ff = foc->current_resistance_ohm * foc->Id_ref * rs_eff;
            float vq_rs_ff = foc->current_resistance_ohm * foc->Iq_ref * rs_eff;

            foc->diag_vd_rs_ff = vd_rs_ff;
            foc->diag_vq_rs_ff = vq_rs_ff;

            vd_cmd += vd_rs_ff;
            vq_cmd += vq_rs_ff;

            /* ABC 诊断置零 */
            foc->diag_Va_pi = 0.0f; foc->diag_Vb_pi = 0.0f; foc->diag_Vc_pi = 0.0f;
            foc->diag_Va_rs_ff = 0.0f; foc->diag_Vb_rs_ff = 0.0f; foc->diag_Vc_rs_ff = 0.0f;
            foc->diag_Va_cmd = 0.0f; foc->diag_Vb_cmd = 0.0f; foc->diag_Vc_cmd = 0.0f;
            foc->diag_Ia_ref = 0.0f; foc->diag_Ib_ref = 0.0f; foc->diag_Ic_ref = 0.0f;

            /* 保存限幅前的总指令 */
            foc->diag_vd_cmd = vd_cmd;
            foc->diag_vq_cmd = vq_cmd;

            /* 电压矢量限幅（Vd/Vq联合限幅）+ 反算抗积分饱和 */
            {
                float vd_sat = vd_cmd;
                float vq_sat = vq_cmd;
                float Vmax_dq = foc->pi_d.output_max;
                float v_mag_local = sqrtf(vd_cmd * vd_cmd + vq_cmd * vq_cmd);
                foc->diag_v_mag = v_mag_local;
                if ((Vmax_dq > FOC_EPSILON) && (v_mag_local > Vmax_dq)) {
                    float scale = Vmax_dq / v_mag_local;
                    vd_sat = vd_cmd * scale;
                    vq_sat = vq_cmd * scale;
                    foc->diag_sat_ratio = scale;
                    if (fabsf(foc->pi_d.Ki) > FOC_EPSILON) {
                        foc->pi_d.integral += (FOC_CURRENT_AW_GAIN * (vd_sat - vd_cmd)) / foc->pi_d.Ki;
                        foc->pi_d.integral = FOC_Saturate(foc->pi_d.integral, foc->pi_d.integral_max, -foc->pi_d.integral_max);
                    }
                    if (fabsf(foc->pi_q.Ki) > FOC_EPSILON) {
                        foc->pi_q.integral += (FOC_CURRENT_AW_GAIN * (vq_sat - vq_cmd)) / foc->pi_q.Ki;
                        foc->pi_q.integral = FOC_Saturate(foc->pi_q.integral, foc->pi_q.integral_max, -foc->pi_q.integral_max);
                    }
                } else {
                    foc->diag_sat_ratio = 1.0f;
                }
                foc->Vdq.d = vd_sat;
                foc->Vdq.q = vq_sat;
            }

            /* 反Park变换 */
            FOC_Inverse_Park_Transform(&foc->Vdq, foc->sin_theta, foc->cos_theta, &foc->ValphaBeta);

            /* SVPWM生成 */
            FOC_SVPWM_Generate(&foc->ValphaBeta, foc->Vbus, &foc->svpwm);
        }
    }
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
