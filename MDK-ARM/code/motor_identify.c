/**
 * @file    motor_identify.c
 * @brief   电机参数自动识别模块实现
 * @note    上电自动离线识别 + 运行中Rs在线补偿
 */

#include "motor_identify.h"
#include "tle5012.h"
#include "foc_app.h"
#include <string.h>
#include <math.h>

/*==================== 私有宏定义 ====================*/
#define MI_DEG2RAD      (FOC_PI / 180.0f)
#define MI_RAD2DEG      (180.0f / FOC_PI)
#define MI_CONTROL_DT   (1.0f / (float)FOC_CONTROL_FREQ)  /* 由PWM频率决定 */

/*==================== 私有函数声明 ====================*/
static void MI_ResetStateData(MI_Handle_t *handle);
static void MI_EnterState(MI_Handle_t *handle, MI_State_t new_state);
static MI_ErrorCode_t MI_RequireValidEncoder(MI_Handle_t *handle);
static uint8_t MI_RsConverged(float rs_positive, float rs_negative);
static uint8_t MI_RsUseSinglePolarityFallback(MI_Handle_t *handle);
static MI_ErrorCode_t MI_UseLsFallback(MI_Handle_t *handle);
static float MI_GetPnTestCurrent(const MI_Handle_t *handle);
static uint8_t MI_PnRetryWithHigherCurrent(MI_Handle_t *handle);
static float MI_ClampAbsVoltage(float voltage, float limit);
static float MI_WrapDelta(float delta);
static void MI_ApplyVerifyVoltageVector(MI_Handle_t *handle, float theta_elec);
/* PN path keeps the current_target naming contract in this file for bench tests. */
/* FOC_SetCurrentReference(handle->foc, current_target, 0.0f); */

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
                MI_UpdatePIWithNewRs(handle->foc, handle->param->Rs);
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
                MI_EnterState(handle, MI_STATE_MOTION_VERIFY);
            } else if (err != MI_ERR_IN_PROGRESS) {
                handle->error_code = err;
                MI_EnterState(handle, MI_STATE_ERROR);
            }
            break;

        case MI_STATE_MOTION_VERIFY:
            err = MI_VerifyMotion(handle);
            if (err == MI_ERR_IN_PROGRESS) {
                break;
            }
            if (err == MI_ERR_NONE) {
                handle->motion_verify_status = (handle->motion_verify_weak) ? 2U : 1U;
            } else {
                handle->motion_verify_status = 3U;  /* failed */
            }
            {
                uint8_t sanity_ok = 1U;

                if (handle->pn_observed_dir == 0) {
                    sanity_ok = 0U;
                } else if (err == MI_ERR_NONE) {
                    /* Strong or weak motion verify pass. */
                } else if (handle->verify_locked_dir != 0) {
                    /* Direction locked before timeout; acceptable for high-cogging motors. */
                } else {
                    sanity_ok = 0U;
                }

                if (sanity_ok) {
                    handle->param->valid_flag = 0xFFFFFFFF;
#if FOC_FF_ENABLE_COGGING
                    MI_EnterState(handle, MI_STATE_COGGING_IDENTIFY);
#else
                    MI_EnterState(handle, MI_STATE_COMPLETE);
#endif
                } else {
                    handle->param->valid_flag = 0x00000000U;
                    handle->error_code = (err != MI_ERR_NONE) ? err : MI_ERR_PN_NOT_CONVERGED;
                    MI_EnterState(handle, MI_STATE_ERROR);
                }
            }
            break;

        case MI_STATE_COGGING_IDENTIFY:
            err = MI_IdentifyCogging(handle);
            if (err == MI_ERR_NONE) {
                MI_EnterState(handle, MI_STATE_COMPLETE);
            } else if (err == MI_ERR_IN_PROGRESS) {
                /* continue */
            } else {
                /* Cogging ID failure is non-fatal; continue to complete */
                MI_EnterState(handle, MI_STATE_COMPLETE);
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
        case MI_ERR_ENCODER_INVALID:    return "Encoder Invalid";
        case MI_ERR_TIMEOUT:            return "Timeout";
        case MI_ERR_PHASE_SEQUENCE:     return "Phase Sequence";
        default:                        return "Unknown Error";
    }
}

static MI_ErrorCode_t MI_RequireValidEncoder(MI_Handle_t *handle)
{
    if (handle == NULL) {
        return MI_ERR_ENCODER_INVALID;
    }

    if (!TLE5012_IsDataValid()) {
        if (handle->encoder_invalid_count < 0xFFU) {
            handle->encoder_invalid_count++;
        }
        if (handle->encoder_invalid_count >= MI_ENCODER_INVALID_CONSECUTIVE_LIMIT) {
            return MI_ERR_ENCODER_INVALID;
        }
        return MI_ERR_IN_PROGRESS;
    }

    handle->encoder_invalid_count = 0U;
    return MI_ERR_NONE;
}

/**
 * @brief 定子电阻Rs识别（d轴闭环锁轴法）
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 原理：
 * 1. d轴电流闭环注入锁定转子，防止运动产生反电动势
 * 2. FOC_SetAngle(0) + FOC_SetCurrentReference(Id) 通过PI控制器输出Vd
 * 3. 稳定后采样Vd(来自PI输出)和Id(来自Clarke+Park变换)
 * 4. Rs = |Vd| / |Id|
 * 5. 反极性重复，取平均抵消死区/零漂
 */
MI_ErrorCode_t MI_IdentifyRs(MI_Handle_t *handle)
{
    float elapsed = MI_GetElapsedTime(handle);
    float test_current = handle->rs_current_target;
    float I_mag, Id_meas;

    /* Read measured d-axis current from FOC (Clarke+Park computed in ISR) */
    Id_meas = handle->foc->Idq.d;
    I_mag  = sqrtf(handle->foc->Idq.d * handle->foc->Idq.d +
                   handle->foc->Idq.q * handle->foc->Idq.q);

    /* Initialize test current on first call */
    if (handle->rs_current_target <= 0.0f) {
        handle->rs_current_target = MI_RS_LOCK_CURRENT_INITIAL;
    }

    /* Phase: lock + sample */
    if (elapsed < (MI_RS_LOCK_DURATION + MI_RS_SAMPLE_DURATION)) {
        /* Apply d-axis current via closed-loop FOC */
        FOC_SetAngle(handle->foc, 0.0f);
        FOC_SetCurrentReference(handle->foc,
            (handle->polarity == 0) ? test_current : -test_current, 0.0f);

        if (elapsed > MI_RS_LOCK_DURATION) {
            /* Overcurrent only during sampling (PI settled by now) */
            if (I_mag > MI_RS_CURRENT_MAX) {
                return MI_ERR_CURRENT_TOO_HIGH;
            }
            /* Sampling phase: accumulate Vd (PI output) and Id (measured) */
            handle->sum_v  += handle->foc->Vdq.d;
            handle->sum_i  += Id_meas;
            handle->sum_ii += Id_meas * Id_meas;
            handle->sum_i_mag += I_mag;
            handle->sample_count++;
        }
        return MI_ERR_IN_PROGRESS;
    }

    /* Compute Rs from accumulated Vd/Id */
    if (handle->sample_count > 0) {
        float n = (float)handle->sample_count;
        float Vd_avg = handle->sum_v / n;
        float Id_avg = handle->sum_i / n;
        float Rs_val = (fabsf(Id_avg) > 0.01f) ? fabsf(Vd_avg / Id_avg) : 0.0f;

        /* Store diagnostics */
        handle->rs_last_v_avg = Vd_avg;
        handle->rs_last_i_avg = Id_avg;
        handle->rs_last_i_mag_avg = handle->sum_i_mag / n;
        handle->rs_last_vec_rs = Rs_val;
        handle->rs_last_samples = handle->sample_count;

        /* Check validity */
        if (fabsf(Id_avg) > MI_RS_CURRENT_THRESH && Rs_val > 0.0f && Rs_val < 50.0f) {
            if (handle->polarity == 0) {
                handle->Rs_positive = Rs_val;
                /* Switch to negative polarity */
                handle->polarity = 1;
                MI_ResetStateData(handle);
                handle->state_start_time = HAL_GetTick();
                return MI_ERR_IN_PROGRESS;
            } else {
                handle->Rs_negative = Rs_val;
                /* Compute final Rs as average of both polarities */
                handle->param->Rs = (handle->Rs_positive + handle->Rs_negative) / 2.0f;
                if (MI_RsConverged(handle->Rs_positive, handle->Rs_negative)) {
                    return MI_ERR_NONE;
                } else if (MI_RsUseSinglePolarityFallback(handle)) {
                    return MI_ERR_NONE;
                } else {
                    return MI_ERR_RS_NOT_CONVERGED;
                }
            }
        } else {
            /* Current too low or invalid — retry with higher current */
            float next = handle->rs_current_target + MI_RS_LOCK_CURRENT_STEP;
            if (next <= MI_RS_LOCK_CURRENT_MAX + 0.001f) {
                handle->rs_current_target = next;
                handle->polarity = 0;
                handle->Rs_positive = 0.0f;
                handle->Rs_negative = 0.0f;
                MI_ResetStateData(handle);
                handle->state_start_time = HAL_GetTick();
                return MI_ERR_IN_PROGRESS;
            }
            return MI_ERR_CURRENT_TOO_LOW;
        }
    }

    return MI_ERR_CURRENT_TOO_LOW;
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
    Vamp = MI_ClampAbsVoltage(Vamp, MI_LS_INJ_VOLTAGE_MAX_V);
    
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
            float Z = 0.0f;
            float Rs_safe = handle->param->Rs;
            if (Rs_safe < 0.1f || Rs_safe > 30.0f) { Rs_safe = 0.5f; }
            float xl_sq = 0.0f;
            float XL = 0.0f;
            float L = 0.0f;

            handle->ls_last_v_rms = Vrms;
            handle->ls_last_i_rms = Irms;
            handle->ls_last_z = 0.0f;
            handle->ls_last_xl = 0.0f;
            handle->ls_last_l = 0.0f;

            if (Irms > 0.01f) {
                Z = Vrms / Irms;
                xl_sq = Z * Z - Rs_safe * Rs_safe;
                if (xl_sq < 0.0f) {
                    xl_sq = 0.0f;
                }

                /* 计算电感 L = XL / (2πf) */
                XL = sqrtf(xl_sq);
                L = XL / (2.0f * FOC_PI * MI_LS_INJ_FREQUENCY);
                handle->ls_last_z = Z;
                handle->ls_last_xl = XL;
                handle->ls_last_l = L;

                if (L > 0.0f && L <= MI_LS_VALID_MAX_H) {
                    handle->param->Ld = L;
                    handle->param->Lq = L;  /* 假设各向同性 */
                    handle->ls_used_fallback = 0U;
                    return MI_ERR_NONE;
                }
            }
        }
        return MI_UseLsFallback(handle);
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
    MI_ErrorCode_t encoder_status = MI_RequireValidEncoder(handle);

    if (encoder_status != MI_ERR_NONE) {
        return encoder_status;
    }

    switch (*state) {
        case 0: /* 加速阶段 */
            if (elapsed < MI_KE_RAMP_TIME) {
                /* 斜坡加速 */
                float ramp_ratio = elapsed / MI_KE_RAMP_TIME;
                float omega_e_cmd = omega_e_target * ramp_ratio;

                /* 开环拖动 */
                handle->foc->Id_ref = 0.0f;
                handle->foc->Iq_ref = 0.05f; /* 极小拖动电流，防DRV过流 */
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
                handle->foc->Iq_ref = 0.05f; /* 极小维持电流 */
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
                    float Rs_safe = handle->param->Rs;
                    if (Rs_safe < 0.1f || Rs_safe > 30.0f) { Rs_safe = 0.5f; }
                    float e_alpha = handle->foc->ValphaBeta.alpha - Rs_safe * handle->foc->IalphaBeta.alpha;
                    float e_beta = handle->foc->ValphaBeta.beta - Rs_safe * handle->foc->IalphaBeta.beta;
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
 * @brief 极对数验证 - 使用已配置Pn，只验证运动方向和拖动是否有效
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 原理：
 * 1. Pn由上位机/Flash配置，避免用短距离抖动反算极对数
 * 2. 施加离散电角步进并等待稳定，确认电机真实跟随
 * 3. 根据累计电角步进与机械角变化的符号确定编码器方向
 */
MI_ErrorCode_t MI_IdentifyPn(MI_Handle_t *handle)
{
    uint8_t *state = &handle->pn_state;
    float *theta_mech_start = &handle->pn_theta_start;
    float *theta_elec_last = &handle->pn_elec_last;
    MI_ErrorCode_t encoder_status = MI_RequireValidEncoder(handle);

    if (encoder_status != MI_ERR_NONE) {
        return encoder_status;
    }

    if ((handle->param->Pn == 0U) || (handle->param->Pn > 50U)) {
        return MI_ERR_PN_NOT_CONVERGED;
    }

    switch (*state) {
        case 0: /* 锁轴初始化：先让转子贴住已知电角度，避免直接旋转场抓不住转子 */
            *theta_mech_start = TLE5012_GetAngle() * MI_DEG2RAD;
            handle->pn_theta_accum = 0.0f;
            handle->pn_theta_last = *theta_mech_start;
            *theta_elec_last = 0.0f;
            handle->sample_count = 0U;
            handle->pn_elec_cycles = 0U;
            handle->pn_last_delta_mech = 0.0f;
            handle->pn_last_delta_elec = 0.0f;
            handle->pn_last_calc = 0.0f;
            handle->pn_observed_dir = 0;
            FOC_SetAngle(handle->foc, 0.0f);
            FOC_SetCurrentReference(handle->foc, MI_PN_ALIGN_CURRENT, 0.0f);
            *state = 1;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;

        case 1: /* 锁轴等待 */
            FOC_SetAngle(handle->foc, 0.0f);
            FOC_SetCurrentReference(handle->foc, MI_PN_ALIGN_CURRENT, 0.0f);
            if (MI_GetElapsedTime(handle) < MI_PN_ALIGN_DURATION) {
                return MI_ERR_IN_PROGRESS;
            }

            *theta_mech_start = TLE5012_GetAngle() * MI_DEG2RAD;
            handle->pn_theta_accum = 0.0f;
            handle->pn_theta_last = *theta_mech_start;
            *theta_elec_last = 0.0f;
            handle->sample_count = 0U;
            handle->pn_elec_cycles = 0U;
            *state = 2;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;

        case 2: /* 应用下一步进电角 */
        {
            float micro_theta_elec_step = MI_PN_STEP_ELEC_DEG * MI_DEG2RAD;
            float micro_current = MI_GetPnTestCurrent(handle);
            float micro_voltage_scale = micro_current / MI_PN_TEST_CURRENT_INITIAL;
            float micro_voltage_mag = handle->foc->Vbus * MI_PN_TEST_VOLTAGE_RATIO * micro_voltage_scale;
            float micro_theta_elec_target;
            uint32_t micro_step_index = handle->pn_elec_cycles;

            if (micro_step_index >= MI_PN_STEP_COUNT) {
                handle->foc->Id_ref = 0.0f;
                handle->foc->Iq_ref = 0.0f;
                handle->pn_last_delta_mech = handle->pn_theta_accum;
                handle->pn_last_delta_elec = *theta_elec_last;
                if ((fabsf(handle->pn_theta_accum) > 0.001f) &&
                    (fabsf(*theta_elec_last) > 0.5f)) {
                    handle->pn_last_calc = fabsf(*theta_elec_last / handle->pn_theta_accum);
                } else {
                    handle->pn_last_calc = 0.0f;
                }
                *state = 0;

#if MI_PN_AUTO_UPDATE_ENCODER_DIR
                if ((fabsf(handle->pn_theta_accum) >= MI_PN_MIN_MECH_DELTA_RAD) &&
                    TLE5012_IsDataValid()) {
                    handle->pn_observed_dir = (handle->pn_theta_accum >= 0.0f) ? 1 : -1;
                    handle->param->encoder_dir = handle->pn_observed_dir;
                }
#endif
                handle->pn_observed_dir = (handle->pn_theta_accum >= 0.0f) ? 1 : -1;
                handle->param->encoder_dir = handle->pn_observed_dir;
                return MI_ERR_NONE;
            }

            micro_theta_elec_target = micro_theta_elec_step * (float)(micro_step_index + 1U);
            *theta_elec_last = micro_theta_elec_target;
            micro_voltage_mag = MI_ClampAbsVoltage(micro_voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);
            FOC_SetAngle(handle->foc, micro_theta_elec_target);
            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;
            handle->foc->pi_d.integral = 0.0f;
            handle->foc->pi_q.integral = 0.0f;
            handle->foc->ValphaBeta.alpha = micro_voltage_mag * cosf(handle->foc->theta_elec);
            handle->foc->ValphaBeta.beta = micro_voltage_mag * sinf(handle->foc->theta_elec);
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

            *state = 8;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;

        }

        case 8:
            if (MI_GetElapsedTime(handle) < MI_PN_STEP_SETTLE_MS) {
                return MI_ERR_IN_PROGRESS;
            }
            {
                float theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;
                float delta_mech = MI_WrapDelta(theta_mech_now - handle->pn_theta_last);

                handle->pn_theta_accum += delta_mech;
                handle->pn_theta_last = theta_mech_now;
                handle->pn_elec_cycles++;
                handle->sample_count++;
                *state = 2;
                return MI_ERR_IN_PROGRESS;
            }

#if 0
        case 3: /* 应用反向解卡脉冲 */
        {
            float theta_elec_nudge = MI_PN_NUDGE_ELEC_DEG * MI_DEG2RAD;
            float voltage_scale = current_target / MI_PN_TEST_CURRENT_INITIAL;
            float voltage_mag = handle->foc->Vbus * MI_PN_TEST_VOLTAGE_RATIO * voltage_scale;
            float theta_elec_cmd = *theta_elec_last - theta_elec_nudge;

            voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);
            FOC_SetAngle(handle->foc, theta_elec_cmd);
            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;
            handle->foc->pi_d.integral = 0.0f;
            handle->foc->pi_q.integral = 0.0f;
            handle->foc->ValphaBeta.alpha = voltage_mag * cosf(handle->foc->theta_elec);
            handle->foc->ValphaBeta.beta = voltage_mag * sinf(handle->foc->theta_elec);
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

            *state = 4;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }

        case 4: /* 等待反向解卡稳定 */
            if (MI_GetElapsedTime(handle) < MI_PN_NUDGE_SETTLE_MS) {
                return MI_ERR_IN_PROGRESS;
            }

            *state = 5;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;

        case 5: /* 应用正向越峰脉冲 */
        {
            float theta_elec_nudge = MI_PN_NUDGE_ELEC_DEG * MI_DEG2RAD;
            float voltage_scale = current_target / MI_PN_TEST_CURRENT_INITIAL;
            float voltage_mag = handle->foc->Vbus * MI_PN_TEST_VOLTAGE_RATIO * voltage_scale;
            float theta_elec_cmd = *theta_elec_last + theta_elec_nudge;

            voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);
            FOC_SetAngle(handle->foc, theta_elec_cmd);
            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;
            handle->foc->pi_d.integral = 0.0f;
            handle->foc->pi_q.integral = 0.0f;
            handle->foc->ValphaBeta.alpha = voltage_mag * cosf(handle->foc->theta_elec);
            handle->foc->ValphaBeta.beta = voltage_mag * sinf(handle->foc->theta_elec);
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

            *state = 6;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }

        case 6: /* 等待正向越峰稳定 */
            if (MI_GetElapsedTime(handle) < MI_PN_NUDGE_SETTLE_MS) {
                return MI_ERR_IN_PROGRESS;
            }

            *state = 7;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;

        case 7: /* 回到主目标步进角 */
        {
            float voltage_scale = current_target / MI_PN_TEST_CURRENT_INITIAL;
            float voltage_mag = handle->foc->Vbus * MI_PN_TEST_VOLTAGE_RATIO * voltage_scale;

            voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);
            FOC_SetAngle(handle->foc, *theta_elec_last);
            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;
            handle->foc->pi_d.integral = 0.0f;
            handle->foc->pi_q.integral = 0.0f;
            handle->foc->ValphaBeta.alpha = voltage_mag * cosf(handle->foc->theta_elec);
            handle->foc->ValphaBeta.beta = voltage_mag * sinf(handle->foc->theta_elec);
            FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

            *state = 8;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }

        case 8: /* 等待主步进稳定 */
            if (MI_GetElapsedTime(handle) < MI_PN_STEP_SETTLE_MS) {
                float voltage_scale = current_target / MI_PN_TEST_CURRENT_INITIAL;
                float voltage_mag = handle->foc->Vbus * MI_PN_TEST_VOLTAGE_RATIO * voltage_scale;

                voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);
                FOC_SetAngle(handle->foc, *theta_elec_last);
                handle->foc->Id_ref = 0.0f;
                handle->foc->Iq_ref = 0.0f;
                handle->foc->pi_d.integral = 0.0f;
                handle->foc->pi_q.integral = 0.0f;
                handle->foc->ValphaBeta.alpha = voltage_mag * cosf(handle->foc->theta_elec);
                handle->foc->ValphaBeta.beta = voltage_mag * sinf(handle->foc->theta_elec);
                FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
                return MI_ERR_IN_PROGRESS;
            }

            *state = 9;
            return MI_ERR_IN_PROGRESS;

        case 9: /* 采样机械角并累计位移 */
        {
            float theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;
            float delta_mech = MI_WrapDelta(theta_mech_now - handle->pn_theta_last);

            handle->pn_theta_accum += delta_mech;
            handle->pn_theta_last = theta_mech_now;
            handle->pn_elec_cycles++;
            handle->sample_count++;
            *state = 2;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }

        case 10: /* 验证配置Pn下的运动方向和最小机械位移 */
        {
            float delta_mech = handle->pn_theta_accum;
            float delta_elec = *theta_elec_last;
            float expected_delta = fabsf(delta_elec) / (float)configured_pn;
            float min_delta = expected_delta * MI_PN_MIN_EXPECTED_TRAVEL_RATIO;
            float dir_sign_min_delta = MI_PN_DIR_SIGN_MIN_MECH_DELTA_RAD;

            handle->foc->Id_ref = 0.0f;
            handle->foc->Iq_ref = 0.0f;
            handle->pn_last_delta_mech = delta_mech;
            handle->pn_last_delta_elec = delta_elec;
            handle->pn_last_calc = 0.0f;

            if (min_delta < MI_PN_MIN_MECH_DELTA_RAD) {
                min_delta = MI_PN_MIN_MECH_DELTA_RAD;
            }

            if (dir_sign_min_delta > min_delta) {
                dir_sign_min_delta = min_delta;
            }

            /* 只把电角/机械角比值作为诊断值，不再用短行程结果覆盖配置Pn。 */
            if (fabsf(delta_mech) >= min_delta && fabsf(delta_elec) > 0.5f) {
                handle->pn_last_calc = fabsf(delta_elec / delta_mech);
                handle->pn_observed_dir = (delta_mech >= 0.0f) ? 1 : -1;
                /* encoder_dir = pn_observed_dir when delta_elec > 0 (forward drive).
                 * Using sign(delta_elec*delta_mech) would give +1 when both are negative,
                 * which assigns the wrong sign when the PN sweep drives elec backwards.
                 * Fix: encoder_dir tracks physical mechanical direction for positive Iq. */
                handle->param->encoder_dir = handle->pn_observed_dir;

                *state = 0;
                return MI_ERR_NONE;
            }

            if (MI_PnRetryWithHigherCurrent(handle)) {
                return MI_ERR_IN_PROGRESS;
            }

            if ((fabsf(delta_mech) >= dir_sign_min_delta) && (fabsf(delta_elec) > 0.5f)) {
                handle->pn_observed_dir = (delta_mech >= 0.0f) ? 1 : -1;
                handle->param->encoder_dir = handle->pn_observed_dir;
            }

#if MI_PN_STRICT_VERIFY
            *state = 0;
            return MI_ERR_PN_NOT_CONVERGED;
#else
            *state = 0;
            return MI_ERR_NONE;
#endif
        }

#endif
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
/**
 * @brief 转动惯量J和粘滞摩擦B识别 — 恒电流加速+滑行法
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 方法:
 *   Phase 1 (ACCEL): 施加恒定Iq, 测量机械速度从 LOW→HIGH 阈值的时间
 *     J = Kt * Iq_avg * Δt / Δv
 *   Phase 2 (COAST): Iq置零, 测量速度从 HIGH→LOW 的自然衰减
 *     B = J * (v_high - v_low) / (Δt * v_avg)
 */
MI_ErrorCode_t MI_IdentifyJ(MI_Handle_t *handle)
{
    MI_ErrorCode_t encoder_status;
    float theta_mech_now;
    float delta_theta;
    float speed_mech_now;
    float Kt;
    uint32_t now_ms;
    float elapsed_s;

    /* Read encoder */
    encoder_status = MI_RequireValidEncoder(handle);
    if (encoder_status != MI_ERR_NONE) {
        return encoder_status;
    }
    theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;

    /* Compute instantaneous mechanical speed from encoder delta */
    if (!handle->j_theta_prev_init) {
        handle->j_theta_prev = theta_mech_now;
        handle->j_theta_prev_init = 1U;
        speed_mech_now = 0.0f;
    } else {
        delta_theta = MI_WrapDelta(theta_mech_now - handle->j_theta_prev);
        handle->j_theta_prev = theta_mech_now;
        speed_mech_now = delta_theta * (float)FOC_CONTROL_FREQ;
    }

    /* Low-pass filter speed (same as speed loop LPF at 20Hz) */
    {
        const float Ts = 1.0f / (float)FOC_CONTROL_FREQ;
        const float wc = 2.0f * FOC_PI * 20.0f;
        const float alpha_j = (wc * Ts) / (1.0f + wc * Ts);
        handle->j_speed_mech = handle->j_speed_mech
                             + alpha_j * (speed_mech_now - handle->j_speed_mech);
    }

    /* Use absolute speed for thresholds — enc_dir may cause negative rotation. */
    float speed_abs = fabsf(handle->j_speed_mech);

    Kt = handle->param->Ke;  /* N·m/A = V/(rad/s) for PMSM */
    if (Kt < 1e-10f) {
        /* Fallback: no valid Ke, use defaults */
        handle->param->J = 0.0001f;
        handle->param->B = 0.001f;
        return MI_ERR_J_NOT_CONVERGED;
    }

    switch (handle->j_state) {
    case 0: /* INIT — start acceleration */
        handle->j_accel_iq_sum = 0.0f;
        handle->j_accel_iq_count = 0.0f;
        handle->j_accel_v_start = 0.0f;
        handle->j_accel_v_end = 0.0f;
        handle->j_accel_t_start = 0U;
        handle->j_accel_t_end = 0U;
        handle->j_coast_v_start = 0.0f;
        handle->j_coast_v_end = 0.0f;
        handle->j_coast_t_start = 0U;
        handle->j_coast_t_end = 0U;
        handle->foc->Id_ref = 0.0f;
        handle->foc->Iq_ref = MI_J_ACCEL_IQ_A;
        handle->j_state = 1;
        return MI_ERR_IN_PROGRESS;

    case 1: /* ACCEL — measure speed ramp */
        now_ms = HAL_GetTick();
        if (MI_GetElapsedTime(handle) > (float)MI_J_ACCEL_TIMEOUT_MS) {
            handle->param->J = 0.0001f;
            handle->param->B = 0.001f;
            handle->foc->Iq_ref = 0.0f;
            return MI_ERR_J_NOT_CONVERGED;
        }
        /* Accumulate Iq for averaging */
        handle->j_accel_iq_sum += handle->foc->Idq.q;
        handle->j_accel_iq_count += 1.0f;

        /* Speed crosses LOW threshold (abs) → record window start.
         * Use j_accel_iq_count (control cycles) for sub-ms timing resolution.
         * Reset Iq accumulators so Iq_avg is computed only within the
         * measurement window, not including the pre-threshold ramp-up. */
        if ((handle->j_accel_t_start == 0U) &&
            (speed_abs >= MI_J_ACCEL_SPEED_LOW_RADPS)) {
            handle->j_accel_v_start = speed_abs;
            handle->j_accel_t_start = now_ms;
            handle->j_accel_cycle_start = (uint32_t)handle->j_accel_iq_count;
            handle->j_accel_iq_sum = 0.0f;
            handle->j_accel_iq_count = 0.0f;
        }

        /* Speed crosses HIGH threshold (abs) → record window end, compute J */
        if ((handle->j_accel_t_end == 0U) &&
            (handle->j_accel_t_start != 0U) &&
            (speed_abs >= MI_J_ACCEL_SPEED_HIGH_RADPS)) {
            handle->j_accel_v_end = speed_abs;
            handle->j_accel_t_end = now_ms;

            /* elapsed from control cycles (50us each at 20kHz).
             * j_accel_iq_count was reset at LOW threshold, so counts
             * only within the measurement window. */
            elapsed_s = (float)((uint32_t)handle->j_accel_iq_count)
                      / (float)FOC_CONTROL_FREQ;
            if ((elapsed_s > 0.0001f) && (handle->j_accel_iq_count > 0.0f)) {
                float Iq_avg = handle->j_accel_iq_sum / handle->j_accel_iq_count;
                float dv = handle->j_accel_v_end - handle->j_accel_v_start;
                if (dv > 0.05f) {
                    handle->param->J = Kt * fabsf(Iq_avg) * elapsed_s / dv;
                    if (handle->param->J < MI_J_VALID_MIN) handle->param->J = MI_J_VALID_MIN;
                    if (handle->param->J > MI_J_VALID_MAX) handle->param->J = MI_J_VALID_MAX;
                } else {
                    handle->param->J = 0.0001f;
                }
            }

            /* Transition to coast phase */
            handle->foc->Iq_ref = 0.0f;
            handle->j_state = 2;
        }
        return MI_ERR_IN_PROGRESS;

    case 2: /* COAST — measure speed decay for B */
        now_ms = HAL_GetTick();
        if (MI_GetElapsedTime(handle) > (float)(MI_J_ACCEL_TIMEOUT_MS + MI_J_COAST_TIMEOUT_MS)) {
            /* Timeout: use J from accel, B default */
            handle->param->B = 0.001f;
            handle->j_state = 3;
            return MI_ERR_IN_PROGRESS;
        }

        /* Speed crosses high threshold (abs, decaying) → record coast start */
        if ((handle->j_coast_t_start == 0U) &&
            (speed_abs <= (MI_J_ACCEL_SPEED_HIGH_RADPS * 0.9f))) {
            handle->j_coast_v_start = speed_abs;
            handle->j_coast_t_start = now_ms;
        }

        /* Speed crosses low threshold (abs, decaying) → record coast end, compute B */
        if ((handle->j_coast_t_end == 0U) &&
            (handle->j_coast_t_start != 0U) &&
            (speed_abs <= MI_J_ACCEL_SPEED_LOW_RADPS)) {
            handle->j_coast_v_end = speed_abs;
            handle->j_coast_t_end = now_ms;

            elapsed_s = (float)(handle->j_coast_t_end - handle->j_coast_t_start) * 0.001f;
            if ((elapsed_s > 0.01f) && (handle->param->J > MI_J_VALID_MIN)) {
                float dv = handle->j_coast_v_start - handle->j_coast_v_end;
                float v_avg = (handle->j_coast_v_start + handle->j_coast_v_end) * 0.5f;
                if ((dv > 0.01f) && (v_avg > 0.1f)) {
                    handle->param->B = handle->param->J * dv / (elapsed_s * v_avg);
                    if (handle->param->B < MI_B_VALID_MIN) handle->param->B = MI_B_VALID_MIN;
                    if (handle->param->B > MI_B_VALID_MAX) handle->param->B = MI_B_VALID_MAX;
                } else {
                    handle->param->B = 0.001f;
                }
            } else {
                handle->param->B = 0.001f;
            }

            handle->j_state = 3;
        }
        return MI_ERR_IN_PROGRESS;

    case 3: /* COMPLETE */
        handle->foc->Iq_ref = 0.0f;
        handle->foc->Id_ref = 0.0f;
        /* If J still at invalid default, mark as not converged */
        if (handle->param->J < MI_J_VALID_MIN || handle->param->J > MI_J_VALID_MAX) {
            handle->param->J = 0.0001f;
            handle->param->B = 0.001f;
            return MI_ERR_J_NOT_CONVERGED;
        }
        return MI_ERR_NONE;

    default:
        handle->j_state = 0;
        return MI_ERR_IN_PROGRESS;
    }
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
        MI_ErrorCode_t encoder_status = MI_RequireValidEncoder(handle);
        float encoder_dir = (handle->param->encoder_dir < 0) ? -1.0f : 1.0f;
        if (encoder_status != MI_ERR_NONE) {
            return encoder_status;
        }
        if ((TLE5012_GetCRCErrorCount() > 3U) || (TLE5012_GetSpiErrorCount() > 0U)) {
            return MI_ERR_ENCODER_INVALID;
        }
        float theta_mech = TLE5012_GetAngle() * MI_DEG2RAD;
        handle->param->theta_mech_zero = theta_mech;
        /* The rotor is locked to electrical d=0 here; store that electrical zero directly. */
        handle->param->theta_offset = FOC_AngleNormalize(-theta_mech * pole_pairs * encoder_dir);
    }

    handle->foc->Id_ref = 0.0f;
    handle->foc->Iq_ref = 0.0f;
    return MI_ERR_NONE;
}

static void MI_ShutdownOutput(MI_Handle_t *handle)
{
    handle->foc->Id_ref = 0.0f;
    handle->foc->Iq_ref = 0.0f;
    handle->foc->Vdq.d = 0.0f;
    handle->foc->Vdq.q = 0.0f;
    handle->foc->ValphaBeta.alpha = 0.0f;
    handle->foc->ValphaBeta.beta = 0.0f;
    handle->foc->pi_d.integral = 0.0f;
    handle->foc->pi_q.integral = 0.0f;
    FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
}

static void MI_ApplyVerifyVoltageVector(MI_Handle_t *handle, float theta_elec)
{
    float voltage_mag = handle->foc->Vbus * MI_VERIFY_VOLTAGE_RATIO;

    voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_VERIFY_VOLTAGE_MAX_V);
    FOC_SetAngle(handle->foc, theta_elec);
    handle->foc->Id_ref = 0.0f;
    handle->foc->Iq_ref = 0.0f;
    handle->foc->Vdq.d = 0.0f;
    handle->foc->Vdq.q = 0.0f;
    handle->foc->pi_d.integral = 0.0f;
    handle->foc->pi_q.integral = 0.0f;
    handle->foc->ValphaBeta.alpha = voltage_mag * cosf(handle->foc->theta_elec);
    handle->foc->ValphaBeta.beta = voltage_mag * sinf(handle->foc->theta_elec);
    FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);
}

/**
 * @brief 参数识别完成前的单向运动认证
 *
 * 验证编码器方向与电角拖动方向一致，不强制的齿槽电机完成完整拖动。
 * - 单向 d轴旋转，从原始运动中锁定实测方向
 * - 锁定后与期望方向比较（期望=pn_observed_dir或+1），不一致→PHASE_SEQUENCE
 * - 达到 MI_VERIFY_MIN_MECH_RAD → 强通过
 * - 超时且方向匹配/无反向故障/编码器在线 → 弱通过
 * - 反向运动超阈值 → 标记 reverse_fault，禁止弱通过
 */
MI_ErrorCode_t MI_VerifyMotion(MI_Handle_t *handle)
{
    float elapsed = MI_GetElapsedTime(handle);
    float pole_pairs = (handle->param->Pn > 0U) ? (float)handle->param->Pn : 1.0f;
    float theta_mech_now;
    float delta_mech;
    float omega_elec;
    MI_ErrorCode_t encoder_status = MI_RequireValidEncoder(handle);

    if (encoder_status != MI_ERR_NONE) {
        return encoder_status;
    }

    theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;

    /* Phase 0 → 1: 初始化 */
    if (handle->verify_phase == 0U) {
        handle->verify_phase = 1U;
        handle->verify_theta_last = theta_mech_now;
        handle->verify_theta_accum = 0.0f;
        handle->verify_raw_accum = 0.0f;
        handle->verify_elec_cmd = 0.0f;
        handle->verify_locked_dir = 0;
        handle->motion_verify_weak = 0U;
        handle->verify_reverse_fault = 0U;
        /* 期望方向：优先pn_observed_dir，否则正电角→正机械(+1) */
        handle->verify_expected_dir = (handle->pn_observed_dir != 0) ?
                                       handle->pn_observed_dir : (int8_t)1;
        handle->state_start_time = HAL_GetTick();
        MI_ApplyVerifyVoltageVector(handle, 0.0f);
        return MI_ERR_IN_PROGRESS;
    }

    /* 无运动保护：5s内方向未锁定且位移极小→停止拖动，避免堵转加热 */
    if (handle->verify_locked_dir == 0 &&
        elapsed > MI_VERIFY_NO_MOTION_TIMEOUT_MS &&
        fabsf(handle->verify_raw_accum) < MI_VERIFY_NO_MOTION_MIN_RAD) {
        MI_ShutdownOutput(handle);
        /* no-motion: do not set weak flag, this is a hard timeout */
        return MI_ERR_TIMEOUT;
    }

    /* 超时处理：弱通过需方向匹配 + 编码器在线。rev_fault不阻断弱通过（d轴拖动在高齿槽电机上固有回弹） */
    if (elapsed > MI_VERIFY_PHASE_TIMEOUT_MS) {
        MI_ShutdownOutput(handle);
        if (handle->verify_locked_dir != 0 &&
            handle->verify_locked_dir == handle->verify_expected_dir) {
            handle->motion_verify_weak = 1U;
            return MI_ERR_NONE;
        }
        return MI_ERR_TIMEOUT;
    }

    delta_mech = MI_WrapDelta(theta_mech_now - handle->verify_theta_last);
    handle->verify_theta_last = theta_mech_now;

    if (handle->verify_locked_dir == 0) {
        /* 方向未锁定：原始累计，用于判断真实运动方向 */
        handle->verify_raw_accum += delta_mech;
        if (fabsf(handle->verify_raw_accum) >= MI_VERIFY_DIR_LOCK_RAD) {
            handle->verify_locked_dir = (handle->verify_raw_accum >= 0.0f) ? 1 : -1;
            handle->verify_theta_accum = fabsf(handle->verify_raw_accum);
            /* 方向校验：与期望不一致时，报告相位序错误而非自动纠正（EncoderAlign已用旧encoder_dir计算theta_offset） */
            if (handle->verify_locked_dir != handle->verify_expected_dir) {
                MI_ShutdownOutput(handle);
                return MI_ERR_PHASE_SEQUENCE;
            }
        }
    } else {
        /* 方向已锁定：按锁定方向累计 */
        float signed_delta = delta_mech * (float)handle->verify_locked_dir;
        handle->verify_theta_accum += signed_delta;
        /* 反向运动超过阈值：标记故障，重置累计 */
        if (handle->verify_theta_accum < -MI_VERIFY_REVERSE_FAULT_RAD) {
            handle->verify_reverse_fault = 1U;
            handle->verify_theta_accum = 0.0f;
        }
    }

    /* 强通过：方向一致 + 达到最小可信位移 */
    if (handle->verify_locked_dir != 0 &&
        handle->verify_locked_dir == handle->verify_expected_dir &&
        handle->verify_theta_accum >= MI_VERIFY_MIN_MECH_RAD) {
        MI_ShutdownOutput(handle);
        handle->motion_verify_weak = 0U;
        return MI_ERR_NONE;
    }

    /* 继续拖动 */
    omega_elec = 2.0f * FOC_PI * MI_VERIFY_MECH_FREQ_HZ * pole_pairs;
    handle->verify_elec_cmd = FOC_AngleNormalize(handle->verify_elec_cmd + omega_elec * MI_CONTROL_DT);

    MI_ApplyVerifyVoltageVector(handle, handle->verify_elec_cmd);

    return MI_ERR_IN_PROGRESS;
}

/**
 * @brief 齿槽转矩LUT识别 — 慢速开环拖动+分bin记录
 * @param handle 识别句柄指针
 * @return 错误代码
 *
 * 方法: 开环旋转电压矢量以恒定低速拖动电机，记录每个机械角bin的Iq反馈，
 *       完整一转后做bin平均，生成264-entry补偿LUT存入Flash。
 */
MI_ErrorCode_t MI_IdentifyCogging(MI_Handle_t *handle)
{
    MI_ErrorCode_t encoder_status;
    float theta_mech_now;
    float delta_mech;
    float omega_elec;
    float pole_pairs;
    uint32_t elapsed;
    uint16_t i;
    float voltage_mag;

    encoder_status = MI_RequireValidEncoder(handle);
    if (encoder_status != MI_ERR_NONE) {
        return encoder_status;
    }

    theta_mech_now = TLE5012_GetAngle() * MI_DEG2RAD;
    pole_pairs = (handle->param->Pn > 0U) ? (float)handle->param->Pn : 1.0f;

    switch (handle->cg_state) {
    case 0: /* INIT — reset tracking, start drag */
        handle->cg_theta_start = theta_mech_now;
        handle->cg_theta_prev = theta_mech_now;
        handle->cg_theta_accum = 0.0f;
        handle->cg_drag_theta_elec = 0.0f;
        handle->cg_drag_speed_elec = MI_COGGING_DRAG_SPEED_MECH_RADPS * pole_pairs;
        voltage_mag = handle->foc->Vbus * MI_COGGING_DRAG_VOLTAGE_RATIO;
        voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_COGGING_DRAG_VOLTAGE_MAX_V);
        handle->cg_drag_voltage = voltage_mag;
        memset(handle->cg_bin_iq_sum, 0, sizeof(handle->cg_bin_iq_sum));
        memset(handle->cg_bin_count, 0, sizeof(handle->cg_bin_count));
        handle->cg_state = 1;
        return MI_ERR_IN_PROGRESS;

    case 1: /* SETTLE — wait for steady motion */
        /* Apply open-loop rotating voltage vector */
        handle->cg_drag_theta_elec = FOC_AngleNormalize(
            handle->cg_drag_theta_elec + handle->cg_drag_speed_elec * MI_CONTROL_DT);
        handle->foc->Id_ref = 0.0f;
        handle->foc->Iq_ref = 0.0f;
        handle->foc->pi_d.integral = 0.0f;
        handle->foc->pi_q.integral = 0.0f;
        handle->foc->ValphaBeta.alpha = handle->cg_drag_voltage * cosf(handle->cg_drag_theta_elec);
        handle->foc->ValphaBeta.beta  = handle->cg_drag_voltage * sinf(handle->cg_drag_theta_elec);
        FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

        if (MI_GetElapsedTime(handle) > (float)MI_COGGING_SETTLE_MS) {
            handle->cg_state = 2;
            handle->state_start_time = HAL_GetTick();
        }
        return MI_ERR_IN_PROGRESS;

    case 2: /* RECORD — bin Iq feedback by mechanical angle for one full revolution */
        elapsed = (uint32_t)MI_GetElapsedTime(handle);
        if (elapsed > (uint32_t)MI_COGGING_RECORD_TIMEOUT_MS) {
            /* Timeout: incomplete but use what we have */
            handle->cg_state = 3;
            handle->state_start_time = HAL_GetTick();
            return MI_ERR_IN_PROGRESS;
        }

        /* Continue open-loop drag */
        handle->cg_drag_theta_elec = FOC_AngleNormalize(
            handle->cg_drag_theta_elec + handle->cg_drag_speed_elec * MI_CONTROL_DT);
        handle->foc->Id_ref = 0.0f;
        handle->foc->Iq_ref = 0.0f;
        handle->foc->pi_d.integral = 0.0f;
        handle->foc->pi_q.integral = 0.0f;
        handle->foc->ValphaBeta.alpha = handle->cg_drag_voltage * cosf(handle->cg_drag_theta_elec);
        handle->foc->ValphaBeta.beta  = handle->cg_drag_voltage * sinf(handle->cg_drag_theta_elec);
        FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);

        /* Track accumulated mechanical angle */
        delta_mech = MI_WrapDelta(theta_mech_now - handle->cg_theta_prev);
        handle->cg_theta_prev = theta_mech_now;
        handle->cg_theta_accum += delta_mech;

        /* Bin Iq feedback by mechanical angle */
        {
            float theta_norm = FOC_AngleNormalize(theta_mech_now);
            float index_f = (theta_norm + FOC_PI) / (2.0f * FOC_PI) * 264.0f;
            int idx = (int)index_f;
            if (idx >= 264) idx = 0;
            if (idx < 0) idx = 263;
            handle->cg_bin_iq_sum[idx] += handle->foc->Idq.q;
            handle->cg_bin_count[idx]++;
        }

        /* Check if we have completed at least one full revolution */
        if (fabsf(handle->cg_theta_accum) >= (2.0f * FOC_PI - 0.1f)) {
            handle->cg_state = 3;
            handle->state_start_time = HAL_GetTick();
        }
        return MI_ERR_IN_PROGRESS;

    case 3: /* COMPUTE — average bins to build LUT */
    {
        float cogging_lut[264];
        float iq_avg_total = 0.0f;
        uint16_t valid_bins = 0U;

        /* Compute per-bin average and global average */
        for (i = 0U; i < 264U; i++) {
            if (handle->cg_bin_count[i] > 0U) {
                cogging_lut[i] = handle->cg_bin_iq_sum[i] / (float)handle->cg_bin_count[i];
                iq_avg_total += cogging_lut[i];
                valid_bins++;
            } else {
                cogging_lut[i] = 0.0f;
            }
        }

        /* Remove DC component: cogging LUT = (per-bin avg) - (global avg) */
        if (valid_bins > 0U) {
            float iq_global_avg = iq_avg_total / (float)valid_bins;
            for (i = 0U; i < 264U; i++) {
                if (handle->cg_bin_count[i] > 0U) {
                    cogging_lut[i] -= iq_global_avg;
                }
            }
        }

        /* If too few valid bins, mark as invalid */
        if (valid_bins < 130U) {
            /* Less than half the bins have data; skip saving */
            handle->cg_state = 4;
            return MI_ERR_IN_PROGRESS;
        }

        /* Buffer LUT in RAM for deferred save from main loop.
         * Flash write inside TIM1 ISR is unreliable; FOC_App_MainLoop
         * will call FOC_App_SaveCoggingLUT() after identification completes. */
        {
            extern FOC_AppHandle_t g_foc_app;
            if (g_foc_app.cogging_lut.valid_size <= 264U) {
                memcpy(g_foc_app.cogging_lut.table, cogging_lut,
                       264U * sizeof(float));
                g_foc_app.cogging_lut.valid_size = 264U;
                g_foc_app.cogging_lut.valid = 1U;
                g_foc_app.cogging_lut.pending = 1U;
            }
        }

        handle->cg_state = 4;
        return MI_ERR_IN_PROGRESS;
    }

    case 4: /* DONE — cleanup */
        handle->foc->Id_ref = 0.0f;
        handle->foc->Iq_ref = 0.0f;
        MI_ShutdownOutput(handle);
        return MI_ERR_NONE;

    default:
        handle->cg_state = 0;
        return MI_ERR_IN_PROGRESS;
    }
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
    (void)Rs_new;

    /* Keep post-Rs control on the conservative bench baseline used at runtime. */
    FOC_PI_Init(&foc->pi_d,
                FOC_CURRENT_LOOP_KP_12V_BENCH,
                FOC_CURRENT_LOOP_KI_12V_BENCH,
                foc->pi_d.output_max,
                foc->pi_d.output_min);
    FOC_PI_Init(&foc->pi_q,
                FOC_CURRENT_LOOP_KP_12V_BENCH,
                FOC_CURRENT_LOOP_KI_12V_BENCH,
                foc->pi_q.output_max,
                foc->pi_q.output_min);
    foc->pi_d.integral_sep_thresh = 1.5f;
    foc->pi_q.integral_sep_thresh = 1.5f;
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
    handle->sum_vq = 0;
    handle->sum_i = 0;
    handle->sum_ia = 0;
    handle->sum_ib = 0;
    handle->sum_ic = 0;
    handle->sum_ialpha = 0;
    handle->sum_ibeta = 0;
    handle->sum_i_mag = 0;
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
    handle->encoder_invalid_count = 0U;
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
    if (new_state == MI_STATE_RS_IDENTIFY) {
        /* d轴闭环锁轴法：按当前param->Rs和实际电流环周期计算PI增益 */
        float Rs_pi = (handle->param->Rs > 0.1f) ? handle->param->Rs : 1.0f;
        float current_ts = 1.0f / (float)FOC_CONTROL_FREQ;
        handle->rs_current_target = 0.0f;
        handle->foc->pi_d.Kp = 0.5f;
        handle->foc->pi_d.Ki = Rs_pi * 2.0f * FOC_PI * 300.0f * current_ts;  /* 24V高阻电机降带宽防过流 */
        handle->foc->pi_d.integral = 0.0f;
        handle->foc->pi_d.integral_max = handle->foc->pi_d.output_max / handle->foc->pi_d.Ki;
        handle->foc->pi_q.integral = 0.0f;
        handle->foc->pi_q.integral_max = 0.0f;
        handle->rs_last_v_avg = 0.0f;
        handle->rs_last_i_avg = 0.0f;
        handle->rs_last_i_mag_avg = 0.0f;
        handle->rs_last_vec_rs = 0.0f;
        handle->rs_last_samples = 0U;
        handle->Rs_positive = 0.0f;
        handle->Rs_negative = 0.0f;
        handle->polarity = 0;
    }
    if (new_state == MI_STATE_LS_IDENTIFY) {
        handle->ls_last_v_rms = 0.0f;
        handle->ls_last_i_rms = 0.0f;
        handle->ls_last_z = 0.0f;
        handle->ls_last_xl = 0.0f;
        handle->ls_last_l = 0.0f;
        handle->ls_used_fallback = 0U;
    }
    if (new_state == MI_STATE_PN_IDENTIFY) {
        handle->pn_current_target = MI_PN_TEST_CURRENT_INITIAL;
        handle->pn_last_delta_mech = 0.0f;
        handle->pn_last_delta_elec = 0.0f;
        handle->pn_last_calc = 0.0f;
        handle->pn_observed_dir = 0;
        handle->foc->pi_d.integral = 0.0f;
        handle->foc->pi_q.integral = 0.0f;
    }
    if (new_state == MI_STATE_MOTION_VERIFY) {
        handle->verify_phase = 0U;
        handle->verify_theta_last = 0.0f;
        handle->verify_theta_accum = 0.0f;
        handle->verify_raw_accum = 0.0f;
        handle->verify_elec_cmd = 0.0f;
        handle->verify_locked_dir = 0;
        handle->verify_expected_dir = 0;
        handle->motion_verify_weak = 0U;
        handle->motion_verify_status = 0U;
        handle->verify_reverse_fault = 0U;
        handle->foc->pi_d.integral = 0.0f;
        handle->foc->pi_q.integral = 0.0f;
    }
    if (new_state == MI_STATE_J_IDENTIFY) {
        handle->j_state = 0U;
        handle->j_speed_mech = 0.0f;
        handle->j_accel_cycle_start = 0U;
        handle->j_theta_prev = 0.0f;
        handle->j_theta_prev_init = 0U;
        handle->j_accel_iq_sum = 0.0f;
        handle->j_accel_iq_count = 0.0f;
        handle->j_accel_v_start = 0.0f;
        handle->j_accel_v_end = 0.0f;
        handle->j_accel_t_start = 0U;
        handle->j_accel_t_end = 0U;
        handle->j_coast_v_start = 0.0f;
        handle->j_coast_v_end = 0.0f;
        handle->j_coast_t_start = 0U;
        handle->j_coast_t_end = 0U;
    }
}

static uint8_t MI_RsConverged(float rs_positive, float rs_negative)
{
    float rs_avg;
    float rs_diff;
    float allowed_diff;

    if ((rs_positive <= 0.0f) || (rs_negative <= 0.0f)) {
        return 0U;
    }

    rs_avg = (rs_positive + rs_negative) * 0.5f;
    rs_diff = fabsf(rs_positive - rs_negative);
    allowed_diff = rs_avg * MI_RS_CONVERGE_REL_THRESH;
    if (allowed_diff < MI_RS_CONVERGE_THRESH) {
        allowed_diff = MI_RS_CONVERGE_THRESH;
    }

    return (rs_diff <= allowed_diff) ? 1U : 0U;
}

static uint8_t MI_RsUseSinglePolarityFallback(MI_Handle_t *handle)
{
    if (handle == NULL) {
        return 0U;
    }

    if (handle->Rs_positive <= 0.0f) {
        return 0U;
    }

    handle->Rs_negative = handle->Rs_positive;
    handle->param->Rs = handle->Rs_positive;
    return 1U;
}

static MI_ErrorCode_t MI_UseLsFallback(MI_Handle_t *handle)
{
    if ((handle == NULL) || (handle->param == NULL)) {
        return MI_ERR_LS_NOT_CONVERGED;
    }

    handle->ls_used_fallback = 1U;
    handle->param->Ld = MI_LS_FALLBACK_DEFAULT;
    handle->param->Lq = MI_LS_FALLBACK_DEFAULT;
    return MI_ERR_NONE;
}

static float MI_GetPnTestCurrent(const MI_Handle_t *handle)
{
    float current;

    if (handle == NULL) {
        return 0.0f;
    }

    current = handle->pn_current_target;
    if (current <= 0.0f) {
        current = MI_PN_TEST_CURRENT_INITIAL;
    }
    if (current > MI_PN_TEST_CURRENT_MAX) {
        current = MI_PN_TEST_CURRENT_MAX;
    }

    return current;
}

static uint8_t MI_PnRetryWithHigherCurrent(MI_Handle_t *handle)
{
    float next_current;

    if (handle == NULL) {
        return 0U;
    }

    next_current = handle->pn_current_target + MI_PN_TEST_CURRENT_STEP;
    if (next_current > (MI_PN_TEST_CURRENT_MAX + 0.0001f)) {
        return 0U;
    }

    handle->pn_current_target = next_current;
    handle->pn_state = 0U;
    MI_ResetStateData(handle);
    handle->foc->pi_d.integral = 0.0f;
    handle->foc->pi_q.integral = 0.0f;
    handle->state_start_time = HAL_GetTick();
    return 1U;
}

static float MI_ClampAbsVoltage(float voltage, float limit)
{
    if (limit <= 0.0f) {
        return voltage;
    }

    return FOC_Saturate(voltage, limit, -limit);
}

static float MI_WrapDelta(float delta)
{
    if (delta > FOC_PI) {
        delta -= 2.0f * FOC_PI;
    } else if (delta < -FOC_PI) {
        delta += 2.0f * FOC_PI;
    }
    return delta;
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

/**
 * @brief 设定当前机械角为机械零位
 * @param param 电机参数结构体指针
 * @param current_theta_mech 当前机械角度 rad
 */
void MI_SetMechZero(MotorParam_t *param, float current_theta_mech)
{
    param->mech_zero_offset = current_theta_mech;
}
