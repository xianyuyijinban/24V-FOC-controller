/**
 * @file    motor_identify.c
 * @brief   电机参数自动识别模块实现
 * @note    上电自动离线识别 + 运行中Rs在线补偿
 */

#include "motor_identify.h"
#include "tle5012.h"
#include <string.h>
#include <math.h>

/*==================== 私有宏定义 ====================*/
#define MI_DEG2RAD      (FOC_PI / 180.0f)
#define MI_RAD2DEG      (180.0f / FOC_PI)
#define MI_CONTROL_DT   0.00005f   /* 20kHz控制周期 */

/*==================== 私有函数声明 ====================*/
static void MI_ResetStateData(MI_Handle_t *handle);
static void MI_EnterState(MI_Handle_t *handle, MI_State_t new_state);

/**
 * @brief 初始化参数识别模块
 * @param handle 识别句柄指针
 * @param param 电机参数结构体指针
 * @param foc FOC句柄指针
 */
void MI_Init(MI_Handle_t *handle, MotorParam_t *param, FOC_Handle_t *foc)
{
    memset(handle, 0, sizeof(MI_Handle_t));
    handle->param = param;
    handle->foc = foc;
    handle->state = MI_STATE_IDLE;
    handle->error_code = MI_ERR_NONE;
}

/**
 * @brief 开始参数识别
 * @param handle 识别句柄指针
 */
void MI_StartIdentify(MI_Handle_t *handle)
{
    handle->error_code = MI_ERR_NONE;
    handle->ke_state = 0;
    handle->pn_state = 0;
    MI_EnterState(handle, MI_STATE_PN_IDENTIFY);
}

/**
 * @brief 参数识别主处理函数（在1ms中断中调用）
 * @param handle 识别句柄指针
 */
void MI_Process(MI_Handle_t *handle)
{
    MI_ErrorCode_t err = MI_ERR_NONE;
    
    switch (handle->state) {
        case MI_STATE_IDLE:
            /* 空闲状态，不执行任何操作 */
            break;
            
        case MI_STATE_PN_IDENTIFY:
            err = MI_IdentifyPn(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_RS_IDENTIFY);
            } else if (err != MI_ERR_IN_PROGRESS) {
                handle->error_code = err;
                MI_EnterState(handle, MI_STATE_ERROR);
            }
            break;
            
        case MI_STATE_RS_IDENTIFY:
            err = MI_IdentifyRs(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_LS_IDENTIFY);
            } else if (err != MI_ERR_IN_PROGRESS) {
                handle->error_code = err;
                MI_EnterState(handle, MI_STATE_ERROR);
            }
            break;
            
        case MI_STATE_LS_IDENTIFY:
            err = MI_IdentifyLs(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_KE_IDENTIFY);
            } else if (err != MI_ERR_IN_PROGRESS) {
                handle->error_code = err;
                MI_EnterState(handle, MI_STATE_ERROR);
            }
            break;
            
        case MI_STATE_KE_IDENTIFY:
            err = MI_IdentifyKe(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_J_IDENTIFY);
            } else if (err == MI_ERR_IN_PROGRESS) {
                /* continue */
            } else {
                /* Ke识别失败不认为是致命错误，继续 */
                MI_EnterState(handle, MI_STATE_J_IDENTIFY);
            }
            break;
            
        case MI_STATE_J_IDENTIFY:
            err = MI_IdentifyJ(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_ENCODER_ALIGN);
            } else if (err == MI_ERR_IN_PROGRESS) {
                /* continue */
            } else {
                /* J识别失败不认为是致命错误，继续 */
                MI_EnterState(handle, MI_STATE_ENCODER_ALIGN);
            }
            break;
            
        case MI_STATE_ENCODER_ALIGN:
            err = MI_EncoderAlign(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_COMPLETE);
            } else if (err != MI_ERR_IN_PROGRESS) {
                handle->error_code = err;
                MI_EnterState(handle, MI_STATE_ERROR);
            }
            break;
            
        case MI_STATE_COMPLETE:
        case MI_STATE_ERROR:
            /* 终态，不执行任何操作 */
            break;
            
        default:
            break;
    }
}

/**
 * @brief 检查识别是否完成
 * @param handle 识别句柄指针
 * @return 1完成，0未完成
 */
uint8_t MI_IsComplete(MI_Handle_t *handle)
{
    return (handle->state == MI_STATE_COMPLETE) ? 1 : 0;
}

/**
 * @brief 获取错误代码
 * @param handle 识别句柄指针
 * @return 错误代码
 */
MI_ErrorCode_t MI_GetError(MI_Handle_t *handle)
{
    return handle->error_code;
}

/**
 * @brief 获取错误字符串
 * @param error 错误代码
 * @return 错误描述字符串
 */
const char* MI_GetErrorString(MI_ErrorCode_t error)
{
    switch (error) {
        case MI_ERR_NONE:               return "No Error";
        case MI_ERR_IN_PROGRESS:        return "In Progress";
        case MI_ERR_MOTOR_MOVING:       return "Motor Moving";
        case MI_ERR_RS_NOT_CONVERGED:   return "Rs Not Converged";
        case MI_ERR_LS_NOT_CONVERGED:   return "Ls Not Converged";
        case MI_ERR_KE_NOT_CONVERGED:   return "Ke Not Converged";
        case MI_ERR_PN_NOT_CONVERGED:   return "Pn Not Converged";
        case MI_ERR_J_NOT_CONVERGED:    return "J Not Converged";
        case MI_ERR_CURRENT_TOO_LOW:    return "Current Too Low";
        case MI_ERR_CURRENT_TOO_HIGH:   return "Current Too High";
        case MI_ERR_TIMEOUT:            return "Timeout";
        default:                        return "Unknown Error";
    }
}

/**
 * @brief 定子电阻Rs识别（直流伏安法）
 * @param handle 识别句柄指针
 * @return 错误代码
 * 
 * 步骤：
 * 1. 施加α轴电压 Vα = 0.05 * Vbus
 * 2. 等待电流稳定
 * 3. 采样电压和电流
 * 4. Rs = Vα / Iα
 * 5. 改变电压极性重复测量，取平均消除死区影响
 */
MI_ErrorCode_t MI_IdentifyRs(MI_Handle_t *handle)
{
    float elapsed = MI_GetElapsedTime(handle);
    float Vtest = handle->foc->Vbus * MI_RS_TEST_VOLTAGE;
    
    /* 阶段1：正极性测试 */
    if (handle->polarity == 0) {
        if (elapsed < MI_RS_TEST_DURATION) {
            /* 施加正电压 */
            handle->foc->ValphaBeta.alpha = Vtest;
            handle->foc->ValphaBeta.beta = 0;
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
            
            /* 等待稳定后开始采样 */
            if (elapsed > MI_RS_SETTLE_TIME) {
                handle->sum_v += Vtest;
                /* 直接采样α轴电流（Clark变换后的alpha分量） */
                handle->sum_i += handle->foc->IalphaBeta.alpha;
                handle->sample_count++;
            }
            return MI_ERR_IN_PROGRESS;  /* 继续等待 */
        } else {
            /* 计算正极性Rs */
            if (handle->sample_count > 0) {
                float V_avg = handle->sum_v / handle->sample_count;
                float I_avg = handle->sum_i / handle->sample_count;
                if (fabsf(I_avg) > MI_RS_CURRENT_THRESH) {
                    handle->Rs_positive = V_avg / I_avg;
                } else {
                    return MI_ERR_CURRENT_TOO_LOW;
                }
            }
            
            /* 切换到负极性 */
            handle->polarity = 1;
            MI_ResetStateData(handle);
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }
    }
    /* 阶段2：负极性测试 */
    else {
        if (elapsed < MI_RS_TEST_DURATION) {
            /* 施加负电压 */
            handle->foc->ValphaBeta.alpha = -Vtest;
            handle->foc->ValphaBeta.beta = 0;
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
            
            if (elapsed > MI_RS_SETTLE_TIME) {
                handle->sum_v += (-Vtest);
                /* 直接采样α轴电流 */
                handle->sum_i += handle->foc->IalphaBeta.alpha;
                handle->sample_count++;
            }
            return MI_ERR_IN_PROGRESS;
        } else {
            /* 计算负极性Rs */
            if (handle->sample_count > 0) {
                float V_avg = handle->sum_v / handle->sample_count;
                float I_avg = handle->sum_i / handle->sample_count;
                if (fabsf(I_avg) > MI_RS_CURRENT_THRESH) {
                    handle->Rs_negative = V_avg / I_avg;
                } else {
                    return MI_ERR_CURRENT_TOO_LOW;
                }
            }
            
            /* 计算平均Rs */
            handle->param->Rs = (handle->Rs_positive + handle->Rs_negative) / 2.0f;
            
            /* 检查收敛条件 */
            float Rs_diff = fabsf(handle->Rs_positive - handle->Rs_negative);
            if (Rs_diff < MI_RS_CONVERGE_THRESH) {
                return MI_ERR_NONE;  /* 识别成功 */
            } else {
                return MI_ERR_RS_NOT_CONVERGED;
            }
        }
    }
}

/**
 * @brief 定子电感Ls识别（高频注入法）
 * @param handle 识别句柄指针
 * @return 错误代码
 * 
 * 步骤：
 * 1. 注入高频交流电压（约1kHz）
 * 2. 测量电流响应
 * 3. 计算电感 L = V / (2πf * I)
 */
MI_ErrorCode_t MI_IdentifyLs(MI_Handle_t *handle)
{
    float elapsed = MI_GetElapsedTime(handle);
    float Vamp = handle->foc->Vbus * MI_LS_INJ_AMPLITUDE;
    
    if (elapsed < MI_LS_TEST_DURATION) {
        /* 生成高频注入信号 */
        float t = (float)handle->sample_count * MI_CONTROL_DT;
        float omega = 2.0f * FOC_PI * MI_LS_INJ_FREQUENCY;
        
        /* α轴注入高频电压 */
        handle->foc->ValphaBeta.alpha = Vamp * sinf(omega * t);
        handle->foc->ValphaBeta.beta = 0;
        
        FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
        
        /* 采样电流平方和用于RMS估计 */
        handle->sum_ii += handle->foc->IalphaBeta.alpha * handle->foc->IalphaBeta.alpha;
        handle->sample_count++;
        
        return MI_ERR_IN_PROGRESS;
    } else {
        /* 计算电感 */
        if (handle->sample_count > 100U) {
            /* 正弦注入已知幅值：Vrms = Vamp / sqrt(2) */
            float Vrms = fabsf(Vamp) * 0.70710678f;
            float Irms = sqrtf(handle->sum_ii / (float)handle->sample_count);

            if (Irms > 0.01f) {
                float Z = Vrms / Irms;
                float Rs = handle->param->Rs;
                float xl_sq = Z * Z - Rs * Rs;
                if (xl_sq < 0.0f) {
                    xl_sq = 0.0f;
                }

                /* 计算电感 L = XL / (2πf) */
                float XL = sqrtf(xl_sq);
                float L = XL / (2.0f * FOC_PI * MI_LS_INJ_FREQUENCY);

                if (L > 0.0f && L < 0.1f) {
                    handle->param->Ld = L;
                    handle->param->Lq = L;  /* 假设各向同性 */
                    return MI_ERR_NONE;
                }
            }
        }
        return MI_ERR_LS_NOT_CONVERGED;
    }
}

/**
 * @brief 反电动势常数Ke识别
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 原理：
 * 1. 开环拖动到目标转速
 * 2. 切换到电流环(Id=0, Iq=维持电流)
 * 3. 在αβ静止坐标系估算反电势幅值 |E| ≈ |Vαβ - Rs * Iαβ|
 * 4. Ke = |E| / |ωe_meas|
 */
MI_ErrorCode_t MI_IdentifyKe(MI_Handle_t *handle)
{
    /* 使用句柄中的状态变量，避免静态变量的线程安全问题 */
    uint8_t *state = &handle->ke_state;
    const float speed_lpf_alpha = 0.2f;
    float pole_pairs = (handle->param->Pn > 0U) ? (float)handle->param->Pn : 1.0f;
    float omega_e_target = MI_KE_TEST_SPEED_RPM * 2.0f * FOC_PI / 60.0f * pole_pairs;
    float elapsed = MI_GetElapsedTime(handle);
    float theta_mech_now, delta_theta, omega_mech, omega_e_meas;

    switch (*state) {
        case 0: /* 加速阶段 */
            if (elapsed < MI_KE_RAMP_TIME) {
                /* 斜坡加速 */
                float ramp_ratio = elapsed / MI_KE_RAMP_TIME;
                float omega_e_cmd = omega_e_target * ramp_ratio;

                /* 开环拖动 */
                handle->foc->Id_ref = 0.0f;
                handle->foc->Iq_ref = 0.5f; /* 拖动电流 */
                handle->speed_elec = omega_e_cmd; /* 用于电角度积分 */
                handle->foc->theta_elec = FOC_AngleNormalize(handle->foc->theta_elec + omega_e_cmd * MI_CONTROL_DT);
                handle->foc->sin_theta = sinf(handle->foc->theta_elec);
                handle->foc->cos_theta = cosf(handle->foc->theta_elec);

                return MI_ERR_IN_PROGRESS;
            } else {
                *state = 1;
                handle->ke_speed_ready = 0U;
                MI_ResetStateData(handle); /* 重置计时 */
                return MI_ERR_IN_PROGRESS;
            }

        case 1: /* 测量阶段 */
            if (elapsed < MI_KE_MEASURE_TIME) {
                /* 使用电流环维持转速，Id=0控制 */
                handle->foc->Id_ref = 0;
                handle->foc->Iq_ref = 0.3f; /* 维持电流 */
                handle->speed_elec = omega_e_target;
                handle->foc->theta_elec = FOC_AngleNormalize(handle->foc->theta_elec + handle->speed_elec * MI_CONTROL_DT);
                handle->foc->sin_theta = sinf(handle->foc->theta_elec);
                handle->foc->cos_theta = cosf(handle->foc->theta_elec);

                /* 编码器测速（机械角差分 -> 电角速度） */
                theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;
                if (!handle->ke_speed_ready) {
                    handle->ke_theta_prev = theta_mech_now;
                    handle->ke_speed_filt = 0.0f;
                    handle->ke_speed_ready = 1U;
                    return MI_ERR_IN_PROGRESS;
                }

                delta_theta = theta_mech_now - handle->ke_theta_prev;
                if (delta_theta > FOC_PI) {
                    delta_theta -= 2.0f * FOC_PI;
                } else if (delta_theta < -FOC_PI) {
                    delta_theta += 2.0f * FOC_PI;
                }
                handle->ke_theta_prev = theta_mech_now;

                omega_mech = delta_theta / MI_CONTROL_DT;
                omega_e_meas = omega_mech * pole_pairs;
                handle->ke_speed_filt += speed_lpf_alpha * (omega_e_meas - handle->ke_speed_filt);

                /* 在αβ静止坐标系估算反电势幅值，降低开环dq坐标失配偏差 */
                if (fabsf(handle->ke_speed_filt) > 10.0f) {
                    float e_alpha = handle->foc->ValphaBeta.alpha - handle->param->Rs * handle->foc->IalphaBeta.alpha;
                    float e_beta = handle->foc->ValphaBeta.beta - handle->param->Rs * handle->foc->IalphaBeta.beta;
                    float e_mag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
                    handle->sum_v += e_mag;
                    handle->sum_i += fabsf(handle->ke_speed_filt);
                    handle->sample_count++;
                }

                return MI_ERR_IN_PROGRESS;
            } else {
                *state = 2;
                return MI_ERR_IN_PROGRESS;
            }

        case 2: /* 计算Ke */
            if (handle->sample_count > 10) {
                float E_avg = handle->sum_v / handle->sample_count;
                float omega_avg = handle->sum_i / handle->sample_count;
                if (fabsf(omega_avg) < 1e-3f) {
                    *state = 0;
                    return MI_ERR_KE_NOT_CONVERGED;
                }

                /* Ke = |E| / |ωe_meas| */
                handle->param->Ke = E_avg / omega_avg;

                /* 验证合理性 */
                if (handle->param->Ke > 0.001f && handle->param->Ke < 1.0f) {
                    handle->ke_speed_ready = 0U;
                    *state = 0;
                    return MI_ERR_NONE;
                }
            }
            handle->ke_speed_ready = 0U;
            *state = 0;
            return MI_ERR_KE_NOT_CONVERGED;
    }

    return MI_ERR_KE_NOT_CONVERGED;
}

/**
 * @brief 极对数识别 - 通过开环拖动统计电角度变化
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 原理：
 * 1. 施加固定方向的旋转磁场
 * 2. 统计编码器转过的机械角度和电角度周期数
 * 3. Pn = 电角度周期数 / 机械角度(圈)
 */
/**
 * @brief 极对数识别 - 通过开环拖动统计电角度变化
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 【修复MI-001】将静态变量移至handle结构体，实现线程安全
 * 原理：
 * 1. 施加固定方向的旋转磁场
 * 2. 统计编码器转过的机械角度和电角度周期数
 * 3. Pn = 电角度周期数 / 机械角度(圈)
 */
MI_ErrorCode_t MI_IdentifyPn(MI_Handle_t *handle)
{
    uint8_t *state = &handle->pn_state;
    float *theta_mech_start = &handle->pn_theta_start;
    float *theta_elec_last = &handle->pn_elec_last;
    float elapsed = MI_GetElapsedTime(handle);

    switch (*state) {
        case 0: /* 初始化 */
            *theta_mech_start = TLE5012_GetAngle() * MI_DEG2RAD;
            handle->pn_theta_accum = 0.0f;
            handle->pn_theta_last = *theta_mech_start;
            *theta_elec_last = 0.0f;
            handle->sample_count = 0U;
            *state = 1;
            return MI_ERR_IN_PROGRESS;

        case 1: /* 施加旋转磁场 */
            if (elapsed < MI_PN_TEST_DURATION) {
                float omega_elec = 2.0f * FOC_PI * 2.0f; /* 2Hz电频率 */
                float theta_elec_cmd = omega_elec * ((float)handle->sample_count * MI_CONTROL_DT);
                float theta_mech_now;
                float delta_mech;

                /* 采用开环拖动：固定Iq并推进电角度 */
                handle->foc->Id_ref = 0.0f;
                handle->foc->Iq_ref = MI_PN_TEST_CURRENT;
                handle->foc->theta_elec = theta_elec_cmd;
                handle->foc->sin_theta = sinf(theta_elec_cmd);
                handle->foc->cos_theta = cosf(theta_elec_cmd);

                theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;
                delta_mech = theta_mech_now - handle->pn_theta_last;
                if (delta_mech > FOC_PI) {
                    delta_mech -= 2.0f * FOC_PI;
                } else if (delta_mech < -FOC_PI) {
                    delta_mech += 2.0f * FOC_PI;
                }

                handle->pn_theta_accum += delta_mech;
                handle->pn_theta_last = theta_mech_now;
                *theta_elec_last = theta_elec_cmd;
                handle->sample_count++;
                return MI_ERR_IN_PROGRESS;
            }

            *state = 2;
            return MI_ERR_IN_PROGRESS;

        case 2: /* 计算极对数 */
        {
            float delta_mech = handle->pn_theta_accum;
            float delta_elec = *theta_elec_last;

            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;

            /* 使用累计机械角与指令电角比值，避免“整周期计数不足”导致识别失败 */
            if (fabsf(delta_mech) > 0.05f && fabsf(delta_elec) > 0.5f) {
                float pn_calc = fabsf(delta_elec / delta_mech);
                handle->param->Pn = (uint8_t)(pn_calc + 0.5f);

                if (handle->param->Pn >= 1U && handle->param->Pn <= 50U) {
                    *state = 0;
                    return MI_ERR_NONE;
                }
            }

            *state = 0;
            return MI_ERR_PN_NOT_CONVERGED;
        }

        default:
            *state = 0;
            return MI_ERR_PN_NOT_CONVERGED;
    }
}

/**
 * @brief 转动惯量J识别
 * @param handle 识别句柄指针
 * @return 错误代码
 */
MI_ErrorCode_t MI_IdentifyJ(MI_Handle_t *handle)
{
    /* 简化实现：使用默认值 */
    handle->param->J = 0.0001f;  /* 默认值 kg·m² */
    handle->param->B = 0.001f;   /* 默认值 N·m·s/rad */
    return MI_ERR_NONE;
}

/**
 * @brief 编码器零位对齐
 * @param handle 识别句柄指针
 * @return 错误代码
 */
MI_ErrorCode_t MI_EncoderAlign(MI_Handle_t *handle)
{
    float elapsed = MI_GetElapsedTime(handle);
    float pole_pairs = (handle->param->Pn > 0U) ? (float)handle->param->Pn : 1.0f;

    if (elapsed < MI_ALIGN_DURATION) {
        /* d轴定向锁轴，让转子对齐到已知电角度 */
        handle->foc->Id_ref = MI_ALIGN_CURRENT;
        handle->foc->Iq_ref = 0.0f;
        FOC_SetAngle(handle->foc, 0.0f);
        return MI_ERR_IN_PROGRESS;
    }

    /* 锁轴结束后读取机械角并反推出电角零位偏置 */
    {
        float theta_mech = TLE5012_GetAngle() * MI_DEG2RAD;
        handle->param->theta_offset = FOC_AngleNormalize(-theta_mech * pole_pairs);
    }

    handle->foc->Id_ref = 0.0f;
    handle->foc->Iq_ref = 0.0f;
    handle->param->valid_flag = 0xFFFFFFFF;
    return MI_ERR_NONE;
}

/**
 * @brief 检查电机是否锁定（机械锁止检测）
 * @param handle 识别句柄指针
 * @return 1锁定，0未锁定
 */
uint8_t MI_CheckMechanicalLock(MI_Handle_t *handle)
{
    float theta_current = handle->foc->theta_elec * MI_RAD2DEG;
    float theta_diff = fabsf(theta_current - handle->theta_start);
    
    if (theta_diff > MI_THETA_LIMIT_DEG) {
        return 0;  /* 电机在动 */
    }
    
    return 1;  /* 电机锁定 */
}

/**
 * @brief 更新FOC的PI控制器参数（使用新的Rs）
 * @param foc FOC句柄指针
 * @param Rs_new 新的Rs值
 */
void MI_UpdatePIWithNewRs(FOC_Handle_t *foc, float Rs_new)
{
    /* 根据新Rs更新电流环Ki */
    /* 与FOC_App_UpdatePIParams保持一致：Ki = Rs * 2π * bandwidth * T_sample */
    float bandwidth = 2000.0f;  /* 2kHz电流环带宽 */
    float T_sample = 0.00005f;  /* 50μs采样周期 */
    
    foc->pi_d.Ki = Rs_new * 2.0f * FOC_PI * bandwidth * T_sample;
    foc->pi_q.Ki = Rs_new * 2.0f * FOC_PI * bandwidth * T_sample;
}

/**
 * @brief Rs在线估计器初始化
 * @param est 估计器指针
 * @param alpha 滤波系数 (0~1)
 */
void MI_RsOnlineEstimator_Init(RsOnlineEstimator_t *est, float alpha)
{
    memset(est, 0, sizeof(RsOnlineEstimator_t));
    est->alpha = alpha;
    est->Rs_estimated = 0.5f;  /* 默认初始值 */
    est->enabled = 0;
}

/**
 * @brief 使能/禁用Rs在线估计
 * @param est 估计器指针
 * @param enable 1使能，0禁用
 */
void MI_RsOnlineEstimator_Enable(RsOnlineEstimator_t *est, uint8_t enable)
{
    est->enabled = enable;
    if (enable) {
        est->voltage_accum = 0;
        est->current_accum = 0;
        est->sample_count = 0;
    }
}

/**
 * @brief Rs在线估计器更新
 * @param est 估计器指针
 * @param Vd d轴电压
 * @param Vq q轴电压
 * @param Id d轴电流
 * @param Iq q轴电流
 * @param omega_e 电角速度
 */
void MI_RsOnlineEstimator_Update(RsOnlineEstimator_t *est, float Vd, float Vq, float Id, float Iq, float omega_e)
{
    if (!est->enabled) return;
    
    /* 低速时估计更准确（反电动势影响小） */
    if (fabsf(omega_e) < 100.0f) {  /* 电角速度 < 100 rad/s */
        float I_mag = sqrtf(Id * Id + Iq * Iq);
        
        if (I_mag > 0.5f) {  /* 电流足够大 */
            /* 简单估计：Rs ≈ Vd / Id (稳态时) */
            /* 防止除零：Id必须大于阈值 */
            if (fabsf(Id) > 0.1f) {
                float Rs_inst = Vd / Id;
                
                /* 限制Rs估计范围（防止异常值） */
                if (Rs_inst > 0.01f && Rs_inst < 10.0f) {
                    /* 一阶低通滤波 */
                    est->Rs_estimated = est->alpha * Rs_inst + (1.0f - est->alpha) * est->Rs_estimated;
                    est->sample_count++;
                }
            }
        }
    }
}

/**
 * @brief 获取估计的Rs值
 * @param est 估计器指针
 * @return 估计的Rs值
 */
float MI_RsOnlineEstimator_GetRs(RsOnlineEstimator_t *est)
{
    return est->Rs_estimated;
}

/*==================== 私有函数实现 ====================*/

/**
 * @brief 重置状态数据
 * @param handle 识别句柄指针
 */
static void MI_ResetStateData(MI_Handle_t *handle)
{
    handle->sum_v = 0;
    handle->sum_i = 0;
    handle->sum_ii = 0;
    handle->sum_vi = 0;
    handle->sample_count = 0;
}

/**
 * @brief 进入新状态
 * @param handle 识别句柄指针
 * @param new_state 新状态
 */
static void MI_EnterState(MI_Handle_t *handle, MI_State_t new_state)
{
    handle->state = new_state;
    handle->state_start_time = HAL_GetTick();
    MI_ResetStateData(handle);
    
    /* 记录起始角度（用于机械锁止检测） */
    if (new_state == MI_STATE_RS_IDENTIFY || 
        new_state == MI_STATE_LS_IDENTIFY) {
        handle->theta_start = handle->foc->theta_elec * MI_RAD2DEG;
        handle->theta_max = handle->theta_start;
        handle->theta_min = handle->theta_start;
    }
    
    /* 重置极性标志 */
    handle->polarity = 0;
}

/**
 * @brief 获取当前状态已运行时间
 * @param handle 识别句柄指针
 * @return 已运行时间 ms
 */
float MI_GetElapsedTime(MI_Handle_t *handle)
{
    return (float)(HAL_GetTick() - handle->state_start_time);
}
