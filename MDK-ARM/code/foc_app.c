/**
 * @file    foc_app.c
 * @brief   FOC应用层接口实现
 * @note    整合FOC核心、硬件驱动、参数识别和存储
 */

#include "foc_app.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

/* 外部变量声明（来自其他模块） */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi3;
extern volatile uint16_t adc_data[8];
extern DRV8350S_Handle_t drv8350s;

/* DRV8350S 使能引脚（与原理图 DRV_EN 一致） */
#define DRV_EN_GPIO_Port GPIOE
#define DRV_EN_Pin       GPIO_PIN_14

/* 私有函数前向声明 */
static void FOC_App_UpdateLoopParams(FOC_AppHandle_t *handle);
static void FOC_App_ApplyKnownMotorConstants(FOC_AppHandle_t *handle);
static uint8_t FOC_App_PrecheckPowerStage(FOC_AppHandle_t *handle, FOC_FaultCode_t *fault, uint8_t require_encoder);
static void FOC_App_RequestDisableFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault);
static uint8_t FOC_App_IsValidHandlePointer(const FOC_AppHandle_t *handle);
static void FOC_App_UpdateIdentifyState(FOC_AppHandle_t *handle);
static void FOC_App_EnterFault(FOC_AppHandle_t *handle, FOC_FaultCode_t fault);
static uint8_t FOC_App_IsEncoderFaultActive(void);
static uint8_t FOC_App_IsEncoderReadyForPowerStage(FOC_FaultCode_t *fault);
static float FOC_App_GetCurrentRefLimit(const FOC_AppHandle_t *handle);
static void FOC_App_ClampSpeedPiIntegral(FOC_PI_Controller_t *pi, float output_max, float output_min);
static void FOC_App_PrimeNeutralPwm(void);
static uint8_t FOC_App_ShouldBootstrapNeutralPwm(const FOC_AppHandle_t *handle, const ADC_Sampling_t *adc);
static void FOC_App_RefreshEncoderFeedback(FOC_AppHandle_t *handle);
static void FOC_PositionPD_Init(FOC_PositionPD_t *pd, float kp, float kd, float output_max, float output_min);
static float FOC_PositionPD_Update(const FOC_PositionPD_t *pd, float pos_error, float speed_feedback);

float FOC_App_PositionSensorToControlFrame(const FOC_AppHandle_t *handle, float pos_ref_sensor)
{
    float encoder_dir = 1.0f;

    if (handle != NULL) {
        encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
    }

    return FOC_AngleNormalize(pos_ref_sensor * encoder_dir);
}

float FOC_App_PositionControlToSensorFrame(const FOC_AppHandle_t *handle, float pos_ref_control)
{
    float encoder_dir = 1.0f;
    float pos_ref_sensor;

    if (handle != NULL) {
        encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
    }

    pos_ref_sensor = FOC_AngleNormalize(pos_ref_control * encoder_dir);
    if (pos_ref_sensor < 0.0f) {
        pos_ref_sensor += 2.0f * FOC_PI;
    }
    return pos_ref_sensor;
}

static uint8_t FOC_App_IsValidHandlePointer(const FOC_AppHandle_t *handle)
{
    uintptr_t address = (uintptr_t)handle;

    if (handle == NULL) {
        return 0U;
    }

    if ((address & 0x3U) != 0U) {
        return 0U;
    }

    if ((address >= 0x20000000UL) && (address <= 0x3FFFFFFFUL)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief FOC应用层初始化
 * @param handle FOC应用层句柄指针
 */
void FOC_App_Init(FOC_AppHandle_t *handle)
{
    memset(handle, 0, sizeof(FOC_AppHandle_t));
    
    /* 初始化状态 */
    handle->state = FOC_STATE_INIT;
    handle->fault_code = FOC_FAULT_NONE;
    handle->control_mode = FOC_MODE_SPEED;  /* 默认速度模式 */
    handle->protection.overcurrent_limit_a = FOC_DEFAULT_OVERCURRENT_LIMIT_A;
    handle->protection.overvoltage_limit_v = FOC_DEFAULT_OVERVOLTAGE_LIMIT_V;
    handle->protection.undervoltage_limit_v = FOC_DEFAULT_UNDERVOLTAGE_LIMIT_V;
    handle->stall_mode_armed = 0U;
    
    /* 初始化母线电压 */
    handle->Vbus = 0.0f;

    /* 初始化FOC核心（使用默认PI参数，识别/加载参数后会更新） */
    FOC_Init(&handle->foc, 1.0f, 0.1f, 1.0f, 0.1f);
    FOC_SetCurrentResistance(&handle->foc, handle->motor_param.Rs);
    
    /* 初始化速度环PI控制器 */
    FOC_PI_Init(&handle->pi_speed, 0.30f, 0.0f, 10.0f, -10.0f);

#if FOC_FF_ENABLE_OBSERVER
    /* 初始化负载转矩观测器（参数在UpdateLoopParams中更新） */
    handle->torque_obs.z = 0.0f;
    handle->torque_obs.T_est = 0.0f;
    handle->torque_obs.T_lpf = 0.0f;
    handle->torque_obs.enabled = 0U;
#endif

    /* 初始化位置环PD控制器 - 输出速度给定 */
    FOC_PositionPD_Init(&handle->pos_pd,
                        FOC_POSITION_PD_KP_DEFAULT,
                        FOC_POSITION_PD_KD_DEFAULT,
                        FOC_POSITION_SPEED_LIMIT_RAD_PER_S,
                        -FOC_POSITION_SPEED_LIMIT_RAD_PER_S);
    
    /* 初始化Rs在线估计器 */
    MI_RsOnlineEstimator_Init(&handle->rs_est, 0.01f);
    
    /* 尝试加载参数 */
    FOC_App_LoadParam(handle);
    FOC_App_UpdateIdentifyState(handle);
    
    /* 如果参数有效，更新控制环参数 */
    if (handle->motor_identified) {
        FOC_App_UpdateLoopParams(handle);
        handle->state = FOC_STATE_READY;
    } else {
        /* 参数无效，需要识别 */
        handle->state = FOC_STATE_IDLE;
    }
}

/**
 * @brief FOC主循环（在main的while循环中调用）
 * @param handle FOC应用层句柄指针
 */
void FOC_App_MainLoop(FOC_AppHandle_t *handle)
{
    MI_ErrorCode_t mi_error = MI_ERR_NONE;
    FOC_FaultCode_t voltageFault = FOC_FAULT_NONE;

    if (handle->pending_disable) {
        handle->pending_disable = 0U;
        FOC_App_Disable(handle);
    }

    FOC_App_RefreshTelemetry(handle);

    switch (handle->state) {
        case FOC_STATE_IDLE:
            /* 等待启动参数识别或加载参数 */
            if (handle->enable_identify) {
                FOC_App_StartIdentify(handle);
            }
            break;
            
        case FOC_STATE_PARAM_IDENTIFY:
            /* 参数识别状态机在TIM1中断中运行 */
            if (!handle->enable_identify) {
                /* 外部请求中止识别 */
                MI_Init(&handle->mi_handle, &handle->motor_param, &handle->foc);
                FOC_App_Disable(handle);
                handle->fault_code = FOC_FAULT_NONE;
            } else if (MI_IsComplete(&handle->mi_handle)) {
                /* 识别结束后先关闭功率级，回到安全待机 */
                FOC_App_Disable(handle);

                /* 识别完成，先确认得到的参数能通过最终有效性门槛，再保存。 */
                FOC_App_UpdateIdentifyState(handle);
                if (!handle->motor_identified) {
                    handle->stall_mode_armed = 0U;
                    FOC_App_EnterFault(handle, FOC_FAULT_PARAM_INVALID);
                    return;
                }

                FOC_App_SaveParam(handle);

                /* Persist cogging LUT — sector already erased by Param_Save above.
                 * Write LUT directly without re-erasing to avoid flash timing issues. */
                if (handle->cogging_lut.pending
                    && handle->cogging_lut.valid_size > 0U
                    && handle->cogging_lut.valid_size <= FOC_COGGING_LUT_SIZE) {
                    uint32_t write_size = (uint32_t)handle->cogging_lut.valid_size * sizeof(float);
                    uint32_t crc = Param_CalculateCRC32(handle->cogging_lut.table, write_size);
                    uint32_t header[4];
                    header[0] = 0x434F4747;
                    header[1] = (uint32_t)handle->cogging_lut.valid_size;
                    header[2] = crc;
                    header[3] = 0U;
                    HAL_FLASH_Unlock();
                    Param_WriteFlash(PARAM_COGGING_FLASH_ADDR, header, sizeof(header));
                    Param_WriteFlash(PARAM_COGGING_FLASH_ADDR + sizeof(header),
                                     (const uint32_t *)handle->cogging_lut.table, write_size);
                    HAL_FLASH_Lock();
                    handle->cogging_lut.pending = 0U;
                }

                /* 更新控制环参数 */
                FOC_App_UpdateLoopParams(handle);
                
                handle->stall_mode_armed = 0U;
                handle->fault_code = FOC_FAULT_NONE;
                handle->state = handle->motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
                handle->enable_identify = 0U;
            } else {
                mi_error = MI_GetError(&handle->mi_handle);
            }

            if (mi_error != MI_ERR_NONE) {
                /* 识别出错 */
                FOC_App_Disable(handle);
                if (mi_error == MI_ERR_ENCODER_INVALID) {
                    handle->fault_code = FOC_FAULT_ENCODER;
                    FOC_App_EnterFault(handle, FOC_FAULT_ENCODER);
                } else {
                    FOC_App_EnterFault(handle, FOC_FAULT_PARAM_INVALID);
                }
            }
            break;
            
        case FOC_STATE_READY:
        case FOC_STATE_RUNNING:
            /* 检查故障 */
            if (fabsf(handle->Ia) > handle->protection.overcurrent_limit_a || 
                fabsf(handle->Ib) > handle->protection.overcurrent_limit_a ||
                fabsf(handle->Ic) > handle->protection.overcurrent_limit_a) {
                FOC_App_Disable(handle);
                FOC_App_EnterFault(handle, FOC_FAULT_OVERCURRENT);
            } else if (FOC_App_GetVoltageTripFault(handle, &voltageFault)) {
                FOC_App_Disable(handle);
                FOC_App_EnterFault(handle, voltageFault);
            }
            break;
            
        case FOC_STATE_FAULT:
            if (((handle->fault_code == FOC_FAULT_UNDERVOLTAGE) ||
                 (handle->fault_code == FOC_FAULT_OVERVOLTAGE)) &&
                FOC_App_IsVoltageFaultRecovered(handle, handle->fault_code)) {
                handle->fault_code = FOC_FAULT_NONE;
                FOC_App_ResetMotionState(handle);
                handle->state = handle->motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
            } else if ((handle->fault_code == FOC_FAULT_ENCODER) &&
                       TLE5012_IsDataValid()) {
                handle->fault_code = FOC_FAULT_NONE;
                FOC_App_ResetMotionState(handle);
                handle->state = handle->motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief TIM1中断处理（20kHz，电流环）
 * @param handle FOC应用层句柄指针
 * 
 * 注意：此函数在TIM1更新中断中调用
 */
void FOC_App_TIM1_IRQHandler(FOC_AppHandle_t *handle)
{
    ADC_Sampling_t *adc = NULL;
    float angle_deg;
    float I_sum = 0.0f;
    const float current_loop_ts = 1.0f / (float)FOC_CONTROL_FREQ;
    uint8_t pole_pairs = 1U;
    uint8_t identify_direct_svpwm = 0U;
    uint8_t stall_without_encoder = 0U;
    uint8_t stall_open_loop_active = 0U;
    uint8_t current_feedback_valid = 0U;
    uint8_t low_side_valid_count = 0U;
    float current_a;
    float current_b;
    float current_c;

    if (!FOC_App_IsValidHandlePointer(handle)) {
        return;
    }

    ADC_Sampling_BeginControlCycle();

    if (handle->state != FOC_STATE_RUNNING && handle->state != FOC_STATE_PARAM_IDENTIFY) {
        goto exit_cycle;
    }

    stall_open_loop_active = ((handle->state == FOC_STATE_RUNNING) &&
                              (handle->stall_open_loop_active != 0U));
    stall_without_encoder = ((handle->state == FOC_STATE_RUNNING) &&
                             !stall_open_loop_active &&
                             (handle->stall_mode_armed != 0U) &&
                             !TLE5012_IsDataValid());

    if ((handle->state == FOC_STATE_RUNNING) &&
        !stall_open_loop_active &&
        !stall_without_encoder &&
        FOC_App_IsEncoderFaultActive()) {
        FOC_App_RequestDisableFromISR(handle, FOC_FAULT_ENCODER);
        goto exit_cycle;
    }

    if (stall_open_loop_active) {
        pole_pairs = (handle->motor_param.Pn > 0U) ? handle->motor_param.Pn : 1U;
        handle->theta_mech = FOC_AngleNormalize(handle->theta_mech + handle->stall_speed_ref_mech * current_loop_ts);
        handle->stall_theta_elec = FOC_AngleNormalize(handle->stall_theta_elec +
                                                     handle->stall_speed_ref_mech * pole_pairs * current_loop_ts);
        handle->theta_elec = handle->stall_theta_elec;
        handle->theta_sample_seq++;
        FOC_SetAngle(&handle->foc, handle->theta_elec);
    } else {
        if (stall_without_encoder && FOC_App_IsEncoderFaultActive()) {
            FOC_App_RequestDisableFromISR(handle, FOC_FAULT_ENCODER);
            goto exit_cycle;
        }

        if (TLE5012_IsDataValid()) {
            angle_deg = TLE5012_GetAngle();
            handle->theta_mech = angle_deg * 3.14159f / 180.0f;  /* 转换为弧度 */
            handle->theta_sample_seq++;
        }
        if (handle->state == FOC_STATE_RUNNING) {
            float encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
            pole_pairs = (handle->motor_param.Pn > 0U) ? handle->motor_param.Pn : 1U;
            handle->theta_elec = handle->theta_mech * encoder_dir * pole_pairs + handle->motor_param.theta_offset;
            FOC_SetAngle(&handle->foc, handle->theta_elec);
        } else {
            /* 识别阶段角度由识别状态机决定（开环），不覆盖 */
            handle->theta_elec = handle->foc.theta_elec;
        }
    }

    /* 使用ADC采样模块计算后的物理量（含零点校准） */
    if (!ADC_Sampling_TryConsumeLatest()) {
        adc = ADC_Sampling_GetData();
        if (adc->sampleMissCount >= FOC_ADC_SAMPLE_MISS_FAULT_THRESHOLD) {
            FOC_App_RequestDisableFromISR(handle, FOC_FAULT_ADC_SAMPLING);
        }
        goto exit_cycle;
    }

    adc = ADC_Sampling_GetData();
    low_side_valid_count += (adc->lowSideValidA != 0U) ? 1U : 0U;
    low_side_valid_count += (adc->lowSideValidB != 0U) ? 1U : 0U;
    low_side_valid_count += (adc->lowSideValidC != 0U) ? 1U : 0U;
    current_feedback_valid = (low_side_valid_count >= 2U) ? 1U : 0U;
    /* Identification stages bypass the low-side window gate while they generate
       either closed-loop current commands or direct SVPWM test vectors. */
    if ((handle->state == FOC_STATE_PARAM_IDENTIFY) &&
        handle->enable_identify &&
        (handle->mi_handle.state == MI_STATE_LS_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_RS_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_J_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_COGGING_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_ENCODER_ALIGN ||
         handle->mi_handle.state == MI_STATE_MOTION_VERIFY)) {
        current_feedback_valid = 1U;
    } else if (FOC_App_ShouldBootstrapNeutralPwm(handle, adc)) {
        current_feedback_valid = 1U;
    } else if (low_side_valid_count == 0U) {
        handle->adc_invalid_low_side_streak++;
        if (handle->adc_invalid_low_side_streak >= FOC_LOW_SIDE_ZERO_WINDOW_FORCE_INTERVAL) {
            handle->adc_invalid_low_side_streak = 0U;
            handle->adc_forced_low_side_count++;
            current_feedback_valid = 1U;
        }
    } else {
        handle->adc_invalid_low_side_streak = 0U;
    }
    if (!current_feedback_valid) {
        handle->adc_invalid_low_side_count++;
    } else {
        handle->adc_valid_low_side_count++;
    }

    current_a = adc->currentA;
    current_b = adc->currentB;
    current_c = adc->currentC;
    /* Current reconstruction: skip for stages that bypass window gate. */
    if (!((handle->state == FOC_STATE_PARAM_IDENTIFY) &&
          handle->enable_identify &&
          (handle->mi_handle.state == MI_STATE_LS_IDENTIFY ||
           handle->mi_handle.state == MI_STATE_RS_IDENTIFY ||
           handle->mi_handle.state == MI_STATE_ENCODER_ALIGN ||
           handle->mi_handle.state == MI_STATE_J_IDENTIFY ||
           handle->mi_handle.state == MI_STATE_MOTION_VERIFY)) &&
        (low_side_valid_count == 2U)) {
        if (adc->lowSideValidA == 0U) {
            current_a = -(current_b + current_c);
        } else if (adc->lowSideValidB == 0U) {
            current_b = -(current_a + current_c);
        } else {
            current_c = -(current_a + current_b);
        }
    }

    handle->Ia = current_a;
    handle->Ib = current_b;
    handle->Ic = current_c;
    
    /* 【改进】三相电流不平衡检查 */
    I_sum = handle->Ia + handle->Ib + handle->Ic;
    if (fabsf(I_sum) > CURRENT_IMBALANCE_THRESH) {
        /* 记录不平衡事件，可用于后续故障诊断 */
        /* 注意：这里不直接触发故障，因为轻微不平衡是正常现象 */
    }
    
    /* 读取母线电压 */
    handle->Vbus = adc->vbus;
    
    if (current_feedback_valid) {
        FOC_UpdateCurrent(&handle->foc, handle->Ia, handle->Ib, handle->Ic);
    }
    FOC_SetVbus(&handle->foc, handle->Vbus);

    /* 参数识别在TIM1周期内执行，保证注入波形和采样同步
     * 注意：先刷新最新电流反馈，再执行识别，避免使用旧IalphaBeta
     */
    if (handle->state == FOC_STATE_PARAM_IDENTIFY && handle->enable_identify) {
        FOC_Clarke_Transform(&handle->foc.Iabc, &handle->foc.IalphaBeta);
        /* Keep Idq current for identify stages that read Iq feedback
         * (e.g. cogging binning) even when FOC_Run is not called. */
        FOC_Park_Transform(&handle->foc.IalphaBeta,
                          handle->foc.sin_theta, handle->foc.cos_theta,
                          &handle->foc.Idq);
        MI_Process(&handle->mi_handle);

        /* P0-2: Immediately zero voltage outputs on error/complete to prevent
           dangerous voltage application while main loop catches up and calls disable. */
        if (handle->mi_handle.state == MI_STATE_ERROR ||
            handle->mi_handle.state == MI_STATE_COMPLETE) {
            handle->foc.Vdq.d = 0.0f;
            handle->foc.Vdq.q = 0.0f;
            handle->foc.ValphaBeta.alpha = 0.0f;
            handle->foc.ValphaBeta.beta = 0.0f;
            handle->foc.pi_d.integral = 0.0f;
            handle->foc.pi_q.integral = 0.0f;
            FOC_SVPWM_Generate(&handle->foc.ValphaBeta, handle->foc.Vbus, &handle->foc.svpwm);
            identify_direct_svpwm = 1U;
        }
    }
    
    if ((handle->state == FOC_STATE_PARAM_IDENTIFY) &&
        handle->enable_identify &&
        (handle->mi_handle.state == MI_STATE_LS_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_PN_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_COGGING_IDENTIFY ||
         handle->mi_handle.state == MI_STATE_MOTION_VERIFY)) {
        /* Ls/Pn/Cogging/MotionVerify are generated by the identify module directly. */
        identify_direct_svpwm = 1U;
    }

    if (identify_direct_svpwm) {
        /* Direct-SVPWM identify stages already produced the voltage vector. */
    } else if (!current_feedback_valid) {
        if (handle->state == FOC_STATE_PARAM_IDENTIFY && !identify_direct_svpwm) {
            /* During identify closed-loop stages, don't regenerate old voltage on invalid windows */
            handle->foc.Vdq.d = 0.0f;
            handle->foc.Vdq.q = 0.0f;
            handle->foc.ValphaBeta.alpha = 0.0f;
            handle->foc.ValphaBeta.beta = 0.0f;
            FOC_SVPWM_Generate(&handle->foc.ValphaBeta, handle->foc.Vbus, &handle->foc.svpwm);
        } else {
            FOC_RegenerateVoltageVector(&handle->foc);
        }
    } else {
        /* 执行FOC计算 */
        FOC_Run(&handle->foc);
    }
    
    /* 更新PWM */
    if (handle->enable_pwm) {
        uint16_t pwm_a, pwm_b, pwm_c;
        uint16_t pwm_period = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
        FOC_GetPWM(&handle->foc, &pwm_a, &pwm_b, &pwm_c, pwm_period);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_a);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_b);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_c);
    }
    
    /* Rs在线估计（每20个周期执行一次，即1kHz） */
    handle->control_count++;
    if ((handle->control_count % 20) == 0) {
        MI_RsOnlineEstimator_Update(&handle->rs_est, 
            handle->foc.Vdq.d, handle->foc.Vdq.q,
            handle->foc.Idq.d, handle->foc.Idq.q,
            handle->speed_elec);
    }

exit_cycle:
    ADC_Sampling_EndControlCycle();
}

/**
 * @brief ISR中故障快速下电：仅做无阻塞动作，阻塞SPI收尾延后到主循环
 */
void FOC_App_RequestFaultShutdownFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault)
{
    if (!FOC_App_IsValidHandlePointer(handle)) {
        return;
    }

    if (handle->pending_disable != 0U) {
        return;
    }

    if ((handle->enable_pwm == 0U) &&
        (handle->state != FOC_STATE_RUNNING) &&
        (handle->state != FOC_STATE_PARAM_IDENTIFY)) {
        FOC_App_EnterFault(handle, fault);
        return;
    }

    handle->enable_pwm = 0U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;

    MI_RsOnlineEstimator_Enable(&handle->rs_est, 0U);

    FOC_App_EnterFault(handle, fault);
    handle->pending_disable = 1U;
}

static void FOC_App_RequestDisableFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault)
{
    FOC_App_RequestFaultShutdownFromISR(handle, fault);
}

static void FOC_App_UpdateIdentifyState(FOC_AppHandle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    handle->motor_identified = Param_IsValid(&handle->motor_param) ? 1U : 0U;
}

static void FOC_App_ApplyKnownMotorConstants(FOC_AppHandle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    if (!Param_IsValid(&handle->motor_param)) {
        handle->motor_param.Rs = 8.8f;
        handle->motor_param.Ld = 0.0005f;
        handle->motor_param.Lq = 0.0005f;
        handle->motor_param.Ke = 0.129f;
        handle->motor_param.Pn = 11U;
        if ((handle->motor_param.encoder_dir != 1) && (handle->motor_param.encoder_dir != -1)) {
            handle->motor_param.encoder_dir = -1;  /* V4 baseline: 默认反向 */
        }
        if (handle->motor_param.J < 0.0f || handle->motor_param.J > 1.0f) {
            handle->motor_param.J = 0.0001f;
        }
    }
}

static void FOC_App_EnterFault(FOC_AppHandle_t *handle, FOC_FaultCode_t fault)
{
    if (handle == NULL) {
        return;
    }

    handle->fault_code = fault;
    handle->state = FOC_STATE_FAULT;
    handle->enable_identify = 0U;
    handle->stall_mode_armed = 0U;
}

static uint8_t FOC_App_IsEncoderFaultActive(void)
{
    if (TLE5012_IsDataValid()) {
        return 0U;
    }

    return (TLE5012_GetCRCErrorCount() >= FOC_ENCODER_FAULT_MISS_THRESHOLD) ? 1U : 0U;
}

static uint8_t FOC_App_IsEncoderReadyForPowerStage(FOC_FaultCode_t *fault)
{
    if (TLE5012_IsDataValid()) {
        return 1U;
    }

    if (FOC_App_IsEncoderFaultActive()) {
        if (fault != NULL) {
            *fault = FOC_FAULT_ENCODER;
        }
    }

    return 0U;
}

static float FOC_App_GetCurrentRefLimit(const FOC_AppHandle_t *handle)
{
    float limit;
    float voltage_limit;

    if (handle == NULL) {
        return FOC_DEFAULT_OVERCURRENT_LIMIT_A * FOC_CURRENT_REF_LIMIT_RATIO;
    }

    limit = handle->protection.overcurrent_limit_a * FOC_CURRENT_REF_LIMIT_RATIO;
    if ((handle->motor_identified != 0U) &&
        (handle->motor_param.Rs > 0.05f) &&
        (handle->Vbus > 1.0f)) {
        voltage_limit = (handle->Vbus * FOC_CURRENT_REF_VOLTAGE_RATIO *
                         FOC_CURRENT_REF_VOLTAGE_MARGIN) / handle->motor_param.Rs;
        if (voltage_limit < limit) {
            limit = voltage_limit;
        }
    }
    if (limit < 0.1f) {
        limit = 0.1f;
    }

    return limit;
}

static void FOC_App_ClampSpeedPiIntegral(FOC_PI_Controller_t *pi, float output_max, float output_min)
{
    float integral_max;
    float integral_min;

    if ((pi == NULL) || (fabsf(pi->Ki) <= 1e-6f)) {
        return;
    }

    integral_max = (output_max / pi->Ki) * 0.9f;
    integral_min = (output_min / pi->Ki) * 0.9f;
    if (integral_max < integral_min) {
        float tmp = integral_max;
        integral_max = integral_min;
        integral_min = tmp;
    }
    pi->integral = FOC_Saturate(pi->integral, integral_max, integral_min);
}

static void FOC_App_PrimeNeutralPwm(void)
{
    uint16_t neutral = (uint16_t)(__HAL_TIM_GET_AUTORELOAD(&htim1) / 2U);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, neutral);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, neutral);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, neutral);
}

static uint8_t FOC_App_ShouldBootstrapNeutralPwm(const FOC_AppHandle_t *handle, const ADC_Sampling_t *adc)
{
    if ((handle == NULL) || (adc == NULL)) {
        return 0U;
    }

    if (handle->state != FOC_STATE_RUNNING) {
        return 0U;
    }

    if ((adc->lowSideValidA != 0U) ||
        (adc->lowSideValidB != 0U) ||
        (adc->lowSideValidC != 0U)) {
        return 0U;
    }

    if ((adc->pwmCompareA != adc->pwmCompareB) ||
        (adc->pwmCompareB != adc->pwmCompareC)) {
        return 0U;
    }

    if (handle->foc.svpwm.sector != 0U) {
        return 0U;
    }

    if ((fabsf(handle->foc.Vdq.d) > FOC_NEUTRAL_BOOTSTRAP_VOLTAGE_EPS) ||
        (fabsf(handle->foc.Vdq.q) > FOC_NEUTRAL_BOOTSTRAP_VOLTAGE_EPS)) {
        return 0U;
    }

    if ((fabsf(handle->foc.Id_ref) <= FOC_NEUTRAL_BOOTSTRAP_CURRENT_EPS) &&
        (fabsf(handle->foc.Iq_ref) <= FOC_NEUTRAL_BOOTSTRAP_CURRENT_EPS)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 转速计算和速度环控制（2kHz）
 * @param handle FOC应用层句柄指针
 * 
 * 【重要】此函数在TIM1_UP_IRQHandler中断中通过分频（10分频）调用
 * 执行频率：20kHz / 10 = 2kHz
 * 
 * 处理内容：
 * - 计算电机转速（微分法）
 * - 速度环PI控制（输出Iq_ref）
 * - 在位置模式下，使用位置环输出的速度给定
 * 
 * 注意：位置环在FOC_App_PositionLoop中单独调用（200Hz）
 */
void FOC_App_SpeedLoop(FOC_AppHandle_t *handle)
{
    const float Ts = 1.0f / (float)FOC_SPEED_LOOP_FREQ;
    const float wc = 2.0f * FOC_PI * FOC_SPEED_LPF_CUTOFF_HZ;
    float alpha = (wc * Ts) / (1.0f + wc * Ts);
    float encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;

    if (handle->state != FOC_STATE_RUNNING) {
        handle->speed_loop_ready = 0U;
        handle->speed_theta_prev = handle->theta_mech;
        handle->speed_loop_count++;
        return;
    }

    if (handle->stall_open_loop_active) {
        float speed_target = 0.0f;
        float speed_step = FOC_STALL_OPEN_LOOP_SPEED_RAMP_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ;
        float iq_target = handle->Iq_ref;
        uint8_t pole_pairs = (handle->motor_param.Pn > 0U) ? handle->motor_param.Pn : 1U;

        if (handle->control_mode == FOC_MODE_POSITION) {
            handle->control_mode = FOC_MODE_SPEED;
        }

        if (handle->control_mode == FOC_MODE_SPEED) {
            speed_target = FOC_Saturate(handle->speed_ref,
                                        FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S,
                                        -FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S);
        }

        handle->stall_speed_ref_mech += FOC_Saturate(speed_target - handle->stall_speed_ref_mech,
                                                     speed_step,
                                                     -speed_step);
        if ((fabsf(handle->stall_speed_ref_mech) < 1e-3f) && (fabsf(speed_target) < 1e-3f)) {
            handle->stall_speed_ref_mech = 0.0f;
        }

        handle->speed_mech = handle->stall_speed_ref_mech;
        handle->speed_elec = handle->speed_mech * pole_pairs;
#if FOC_FF_ENABLE_BEMF
        FOC_SetOmegaElec(&handle->foc, handle->speed_elec);
#endif

        iq_target = FOC_Saturate(iq_target,
                                 FOC_STALL_OPEN_LOOP_CURRENT_MAX_A,
                                 -FOC_STALL_OPEN_LOOP_CURRENT_MAX_A);
        if ((handle->control_mode == FOC_MODE_SPEED) &&
            (fabsf(handle->speed_mech) > 1e-3f) &&
            (fabsf(iq_target) < 1e-3f)) {
            iq_target = (handle->speed_mech >= 0.0f) ?
                        FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A :
                        -FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A;
        }

        handle->Id_ref = 0.0f;
        handle->Iq_ref = iq_target;
        FOC_SetCurrentReference(&handle->foc, 0.0f, iq_target);
        handle->speed_loop_count++;
        return;
    }

    /* 速度环（仅在运行状态执行） */
    if (!handle->speed_loop_ready) {
        handle->speed_theta_prev = handle->theta_mech;
        handle->speed_mech = 0.0f;
        handle->speed_elec = 0.0f;
        handle->speed_loop_ready = 1U;
        handle->speed_ref_ramped = 0.0f;
        handle->speed_loop_count++;
        return;
    }

    /* 计算转速（简化：微分法） */
    float delta_theta = handle->theta_mech - handle->speed_theta_prev;
    float speed_raw;
    float speed_next;
    float speed_step;
    float speed_ref_temp;
    float speed_feedback;
    float speed_error;
    float friction_comp;
    float friction_dir;
    float friction_delta;

    /* 处理角度环绕（-π到+π跳变） */
    if (delta_theta > FOC_PI) {
        delta_theta -= 2.0f * FOC_PI;
    } else if (delta_theta < -FOC_PI) {
        delta_theta += 2.0f * FOC_PI;
    }

    /* 一阶低通滤波，抑制编码器量化和采样抖动 */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    speed_raw = delta_theta * FOC_SPEED_LOOP_FREQ;
    speed_next = handle->speed_mech + alpha * (speed_raw - handle->speed_mech);
    speed_step = FOC_SPEED_EST_ACCEL_LIMIT_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ;
    handle->speed_mech += FOC_Saturate(speed_next - handle->speed_mech, speed_step, -speed_step);
    handle->speed_elec = handle->speed_mech * handle->motor_param.Pn * encoder_dir;
#if FOC_FF_ENABLE_BEMF
    FOC_SetOmegaElec(&handle->foc, handle->speed_elec);
#endif
    float speed_mech_user = handle->speed_mech * encoder_dir;
    handle->speed_theta_prev = handle->theta_mech;

    /* 速度给定来源：
     * - 位置模式：使用位置环输出的速度给定（在FOC_App_PositionLoop中更新）
     * - 速度模式：直接使用设定的速度给定
     * - 力矩模式：不经过速度环
     */
    if (handle->control_mode == FOC_MODE_TORQUE) {
        /* 力矩模式：跳过速度环，直接设置电流给定 */
        /* Iq_ref已在FOC_App_SetCurrentRef中设置 */
        handle->speed_loop_count++;
        return;
    } else if (handle->control_mode == FOC_MODE_POSITION) {
        float speed_ramp_delta = handle->speed_ref - handle->speed_ref_ramped;
        float speed_ramp_step = FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ;
        /* 位置模式复用速度给定斜坡，避免位置步进把速度环瞬间推到限幅。 */
        handle->speed_ref_ramped += FOC_Saturate(speed_ramp_delta,
                                                 speed_ramp_step,
                                                 -speed_ramp_step);
        speed_ref_temp = handle->speed_ref_ramped;
        handle->position_loop_speed_ramp_sat_diag = (fabsf(speed_ramp_delta) > speed_ramp_step) ? 1U : 0U;
    } else {
        /* 速度模式：直接使用速度给定 */
        handle->speed_ref_ramped += FOC_Saturate(handle->speed_ref - handle->speed_ref_ramped,
                                                 FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ,
                                                 -FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ);
        speed_ref_temp = handle->speed_ref_ramped;
        handle->position_loop_speed_ramp_sat_diag = 0U;
    }

    /* Speed and position commands are user-frame mechanical values. Keep the
       outer loops in that frame, then map the torque command to the q-axis once. */
    speed_feedback = speed_mech_user;
    speed_error = speed_ref_temp - speed_feedback;
    {
        float iq_limit_pos = FOC_App_GetCurrentRefLimit(handle);
        float iq_limit_neg = -FOC_App_GetCurrentRefLimit(handle);
        float iq_ref_mech;
        if (handle->control_mode == FOC_MODE_SPEED) {
            if (iq_limit_pos > FOC_SPEED_POSITIVE_IQ_LIMIT_A) {
                iq_limit_pos = FOC_SPEED_POSITIVE_IQ_LIMIT_A;
            }
            if ((-iq_limit_neg) > FOC_SPEED_NEGATIVE_IQ_LIMIT_A) {
                iq_limit_neg = -FOC_SPEED_NEGATIVE_IQ_LIMIT_A;
            }
        } else if (handle->control_mode == FOC_MODE_POSITION) {
            iq_limit_pos = FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A;
            iq_limit_neg = -FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A;
        }
        iq_ref_mech = FOC_Saturate(FOC_PI_Update(&handle->pi_speed, speed_error),
                                   iq_limit_pos,
                                   iq_limit_neg);

        /* P2: Acceleration / Inertia Feedforward
         * alpha = d(speed_ref_ramped)/dt, Iq_ff = J * alpha / Kt (Kt ≈ Ke in SI)
         * 门禁: J必须在[FOC_FF_INERTIA_J_MIN, FOC_FF_INERTIA_J_MAX], B<=FOC_FF_INERTIA_B_MAX
         */
#if FOC_FF_ENABLE_INERTIA
        {
            float Kt = handle->motor_param.Ke;
            uint8_t inertia_allowed = 0U;
            if ((fabsf(Kt) > 1e-10f) &&
                (handle->motor_param.J >= FOC_FF_INERTIA_J_MIN) &&
                (handle->motor_param.J <= FOC_FF_INERTIA_J_MAX) &&
                (handle->motor_param.B >= 0.0f) &&
                (handle->motor_param.B <= FOC_FF_INERTIA_B_MAX) &&
                (handle->ff_blocked_by_enc_dir == 0U)) {
                inertia_allowed = 1U;
            }
            handle->ff_diag.inertia_blocked = (inertia_allowed == 0U) ? 1U : 0U;
            if (inertia_allowed) {
                float alpha = (handle->speed_ref_ramped - handle->speed_ref_ramped_prev) * (float)FOC_SPEED_LOOP_FREQ;
                float inertia_ff = handle->motor_param.J * alpha / Kt;
                inertia_ff = FOC_Saturate(inertia_ff, FOC_FF_INERTIA_MAX_A, -FOC_FF_INERTIA_MAX_A);
                handle->ff_diag.inertia_iq = inertia_ff;
                iq_ref_mech += inertia_ff;
                iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
            } else {
                handle->ff_diag.inertia_iq = 0.0f;
            }
        }
        handle->speed_ref_ramped_prev = handle->speed_ref_ramped;
#else
        handle->speed_ref_ramped_prev = handle->speed_ref_ramped;
        handle->ff_diag.inertia_iq = 0.0f;
        handle->ff_diag.inertia_blocked = 1U;
#endif

        /* P3: Coulomb + Viscous Friction Feedforward
         * Iq_coulomb = sign(omega) * Tc / Kt  (with deadband)
         * Iq_viscous = B * omega / Kt
         * 使用用户坐标速度 speed_mech_user，不用裸 speed_mech
         */
#if FOC_FF_ENABLE_FRICTION
        {
            float Kt = handle->motor_param.Ke;
            uint8_t friction_allowed = ((fabsf(Kt) > 1e-10f) &&
                                        (handle->ff_blocked_by_enc_dir == 0U)) ? 1U : 0U;
            handle->ff_diag.friction_enabled = friction_allowed;
            if (friction_allowed) {
                float omega = speed_mech_user;  /* 使用用户坐标速度，不用裸speed_mech */
                float friction_total = 0.0f;
                /* Coulomb: constant magnitude with sign direction, deadband near zero */
                if (fabsf(omega) > FOC_FF_COULOMB_DEADBAND_RADPS) {
                    friction_total += (omega > 0.0f) ?
                        (handle->motor_param.Tc / Kt) : (-handle->motor_param.Tc / Kt);
                }
                /* Viscous: proportional to speed */
                friction_total += handle->motor_param.B * omega / Kt;
                friction_total = FOC_Saturate(friction_total,
                                              FOC_FF_FRICTION_MAX_A,
                                              -FOC_FF_FRICTION_MAX_A);
                handle->ff_diag.friction_iq = friction_total;
                iq_ref_mech += friction_total;
                iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
            } else {
                handle->ff_diag.friction_iq = 0.0f;
            }
        }
#else
        handle->ff_diag.friction_iq = 0.0f;
        handle->ff_diag.friction_enabled = 0U;
#endif

        /* P0: Cogging Torque LUT Feedforward
         * Linear interpolation on 264-bin table indexed by mechanical angle
         * 门禁: LUT必须valid 且 enc_dir == -1
         */
#if FOC_FF_ENABLE_COGGING
        {
            uint8_t cogging_allowed = (handle->cogging_lut.valid &&
                                       (handle->ff_blocked_by_enc_dir == 0U)) ? 1U : 0U;
            handle->ff_diag.cogging_enabled = cogging_allowed;
            if (cogging_allowed) {
                float theta_norm = FOC_AngleNormalize(handle->theta_mech);
                /* Map theta [-PI, PI) to index [0, FOC_COGGING_LUT_SIZE) */
                float index_f = (theta_norm + FOC_PI) / (2.0f * FOC_PI) * (float)FOC_COGGING_LUT_SIZE;
                int idx = (int)index_f;
                float frac = index_f - (float)idx;
                if (idx >= FOC_COGGING_LUT_SIZE) idx = 0;
                if (idx < 0) idx = FOC_COGGING_LUT_SIZE - 1;
                int idx_next = (idx + 1) % FOC_COGGING_LUT_SIZE;
                float cogging_ff = handle->cogging_lut.table[idx] * (1.0f - frac)
                                 + handle->cogging_lut.table[idx_next] * frac;
                cogging_ff = FOC_Saturate(cogging_ff,
                                          FOC_FF_COGGING_MAX_A,
                                          -FOC_FF_COGGING_MAX_A);
                handle->ff_diag.cogging_iq = cogging_ff;
                iq_ref_mech += cogging_ff;
                iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
            } else {
                handle->ff_diag.cogging_iq = 0.0f;
            }
        }
#else
        handle->ff_diag.cogging_iq = 0.0f;
        handle->ff_diag.cogging_enabled = 0U;
#endif

        /* P4: Load Torque Observer — Gopinath reduced-order disturbance observer
         * Model: J * d(omega)/dt = Kt * Iq - T_load - B * omega
         * Observer: T_est estimates the aggregate disturbance torque
         * 门禁: observer必须enabled 且 enc_dir == -1
         */
#if FOC_FF_ENABLE_OBSERVER
        {
            uint8_t observer_allowed = (handle->torque_obs.enabled &&
                                        (handle->ff_blocked_by_enc_dir == 0U)) ? 1U : 0U;
            handle->ff_diag.observer_enabled = observer_allowed;
            if (observer_allowed) {
                const float Ts = 1.0f / (float)FOC_SPEED_LOOP_FREQ;
                FOC_TorqueObserver_t *obs = &handle->torque_obs;
                float omega = handle->speed_mech;
                float Iq = handle->Iq_ref;

                /* Discrete Gopinath observer update (forward Euler) */
                float omega_l = obs->l * omega;
                float dz = (-obs->l / obs->J_hat) * (obs->z + obs->J_hat * omega_l)
                         + (obs->Kt * Iq - obs->B_hat * omega) / obs->J_hat;
                obs->z += Ts * dz;
                obs->T_est = obs->z + obs->J_hat * omega_l;

                /* Low-pass filter on estimated torque */
                {
                    const float wc_obs = 2.0f * FOC_PI * FOC_FF_OBSERVER_LPF_HZ;
                    const float alpha_obs = (wc_obs * Ts) / (1.0f + wc_obs * Ts);
                    obs->T_lpf += alpha_obs * (obs->T_est - obs->T_lpf);
                }

                /* Convert to Iq compensation */
                if (fabsf(obs->Kt) > 1e-10f) {
                    float observer_ff = obs->T_lpf / obs->Kt;
                    observer_ff = FOC_Saturate(observer_ff,
                                               FOC_FF_OBSERVER_MAX_A,
                                               -FOC_FF_OBSERVER_MAX_A);
                    handle->ff_diag.observer_iq = observer_ff;
                    iq_ref_mech += observer_ff;
                    iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
                } else {
                    handle->ff_diag.observer_iq = 0.0f;
                }
            } else {
                handle->ff_diag.observer_iq = 0.0f;
            }
        }
#else
        handle->ff_diag.observer_iq = 0.0f;
        handle->ff_diag.observer_enabled = 0U;
#endif

        friction_dir = 0.0f;
        friction_delta = 0.0f;
        if ((handle->control_mode == FOC_MODE_SPEED) &&
            (fabsf(speed_ref_temp) > FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S) &&
            (fabsf(speed_error) > FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S) &&
            (fabsf(speed_feedback) < FOC_SPEED_STATIC_FRICTION_ACTIVE_RAD_PER_S) &&
            ((speed_ref_temp * speed_error) > 0.0f)) {
            friction_dir = speed_ref_temp;
        } else if (handle->control_mode == FOC_MODE_POSITION) {
            float theta_mech_zeroed = FOC_AngleNormalize(handle->theta_mech - handle->motor_param.mech_zero_offset);
            float theta_mech_control = theta_mech_zeroed * encoder_dir;
            float pos_error_for_friction = FOC_AngleNormalize(handle->pos_ref - theta_mech_control);
            if (fabsf(pos_error_for_friction) > FOC_POSITION_STATIC_FRICTION_ENTER_RAD) {
                handle->position_friction_active = 1U;
            } else if (fabsf(pos_error_for_friction) < FOC_POSITION_STATIC_FRICTION_EXIT_RAD) {
                handle->position_friction_active = 0U;
            }
            if (handle->position_friction_active != 0U) {
                friction_dir = pos_error_for_friction;
            }
        }

        if (friction_dir != 0.0f) {
            if (handle->control_mode == FOC_MODE_POSITION) {
                friction_comp = (friction_dir > 0.0f) ?
                                FOC_POSITION_USER_POSITIVE_STATIC_FRICTION_COMP_A :
                                FOC_POSITION_USER_NEGATIVE_STATIC_FRICTION_COMP_A;
            } else {
                friction_comp = (friction_dir > 0.0f) ?
                                FOC_SPEED_STATIC_FRICTION_POS_COMP_A :
                                FOC_SPEED_STATIC_FRICTION_NEG_COMP_A;
            }
            friction_delta = (friction_dir > 0.0f) ? friction_comp : -friction_comp;
            iq_ref_mech += friction_delta;
            iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
        }
        if (handle->control_mode == FOC_MODE_POSITION) {
            handle->position_loop_iq_pos_sat_diag = (iq_ref_mech >= iq_limit_pos) ? 1U : 0U;
            handle->position_loop_iq_neg_sat_diag = (iq_ref_mech <= iq_limit_neg) ? 1U : 0U;
        } else {
            handle->position_loop_iq_pos_sat_diag = 0U;
            handle->position_loop_iq_neg_sat_diag = 0U;
        }
        FOC_App_ClampSpeedPiIntegral(&handle->pi_speed, iq_limit_pos, iq_limit_neg);
        float iq_cmd = iq_ref_mech;

        /* FFDiag 汇总 */
        handle->ff_diag.ff_total_iq = handle->ff_diag.inertia_iq
                                    + handle->ff_diag.friction_iq
                                    + handle->ff_diag.cogging_iq
                                    + handle->ff_diag.observer_iq;
        handle->ff_diag.ff_enc_dir_blocked = handle->ff_blocked_by_enc_dir;
        /* P1 BEMF诊断值：当前由FOC_FF_ENABLE_BEMF控制，关闭时均为0 */
#if FOC_FF_ENABLE_BEMF
        handle->ff_diag.bemf_enabled = handle->foc.bemf_enabled;
        /* BEMF实际值在FOC_Run中计算，此处诊断用电机参数估算 */
        {
            float omega_e_diag = handle->speed_elec;
            handle->ff_diag.bemf_vd = -omega_e_diag * handle->motor_param.Lq * handle->Iq_ref;
            handle->ff_diag.bemf_vq =  omega_e_diag * (handle->motor_param.Ld * handle->Id_ref + handle->motor_param.Ke);
        }
#else
        handle->ff_diag.bemf_vd = 0.0f;
        handle->ff_diag.bemf_vq = 0.0f;
        handle->ff_diag.bemf_enabled = 0U;
#endif

        handle->speed_loop_ref_diag = speed_ref_temp;
        handle->speed_loop_mech_diag = speed_feedback;
        handle->speed_loop_error_diag = speed_error;
        handle->speed_loop_iq_mech_diag = iq_ref_mech;
        handle->speed_loop_friction_diag = friction_delta;
        handle->speed_loop_iq_cmd_diag = iq_cmd;

        /* 更新电流参考值 */
        FOC_App_SetCurrentRef(handle, 0.0f, iq_cmd);  /* Id=0控制 */
    }

    handle->speed_loop_count++;
}

/**
 * @brief 位置环控制（200Hz）
 * @param handle FOC应用层句柄指针
 * 
 * 【重要】此函数在TIM1_UP_IRQHandler中断中通过分频（100分频）调用
 * 执行频率：20kHz / 100 = 200Hz
 * 
 * 处理内容：
 * - 位置环PD控制（仅在位置模式）
 * - 输出速度给定到handle->speed_ref
 * - 速度环在FOC_App_SpeedLoop中单独执行（2kHz）
 */
void FOC_App_PositionLoop(FOC_AppHandle_t *handle)
{
    if ((handle == NULL) || handle->stall_open_loop_active) {
        return;
    }

    /* 位置环（仅在位置模式且运行状态执行） */
    if (handle->state == FOC_STATE_RUNNING && 
        handle->control_mode == FOC_MODE_POSITION) {
        
        /* 位置误差：pos_ref(用户坐标) - theta_mech_user(用户坐标) */
        float encoder_dir_f = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
        float theta_mech_zeroed = FOC_AngleNormalize(handle->theta_mech - handle->motor_param.mech_zero_offset);
        float theta_mech_user_pos = theta_mech_zeroed * encoder_dir_f;
        float speed_mech_user_pos = handle->speed_mech * encoder_dir_f;
        float pos_error = handle->pos_ref - theta_mech_user_pos;
        
        /* 处理角度环绕（最短路径） */
        pos_error = FOC_AngleNormalize(pos_error);

        /* 位置环PD：位置误差给速度指令，速度反馈提供阻尼 */
        float pos_pd_out = FOC_PositionPD_Update(&handle->pos_pd, pos_error, speed_mech_user_pos);

        /* V4 巡航速度下限：大误差时维持最低巡航速度，避免末端渐近慢尾 */
        float cruise_cmd = pos_pd_out;
        uint8_t cruise_active = 0U;
        float abs_err = (pos_error >= 0.0f) ? pos_error : -pos_error;
        if (abs_err > FOC_POSITION_CRUISE_HOLD_THRESHOLD_RAD) {
            /* 只在 PD 输出与误差同向且 PD 输出绝对值小于巡航速度时才启用 */
            float abs_pd = (pos_pd_out >= 0.0f) ? pos_pd_out : -pos_pd_out;
            if ((pos_error * pos_pd_out) > 0.0f && abs_pd < FOC_POSITION_CRUISE_SPEED_RAD_PER_S) {
                cruise_cmd = (pos_error > 0.0f) ? FOC_POSITION_CRUISE_SPEED_RAD_PER_S
                                                 : -FOC_POSITION_CRUISE_SPEED_RAD_PER_S;
                cruise_active = 1U;
            }
        }

        handle->speed_ref = cruise_cmd;
        handle->position_loop_error_diag = pos_error;
        handle->position_loop_pd_out_diag = pos_pd_out;
        handle->position_loop_pd_sat_diag =
            ((pos_pd_out >= handle->pos_pd.output_max) || (pos_pd_out <= handle->pos_pd.output_min)) ? 1U : 0U;
        handle->traj_active_diag = cruise_active;
        handle->traj_cmd_diag = cruise_cmd;
    }
}

/**
 * @brief 参数识别状态机处理（兼容接口）
 * @param handle FOC应用层句柄指针
 * 
 * 参数识别已移至 FOC_App_TIM1_IRQHandler() 每个PWM周期执行。
 * 保留该函数仅用于兼容旧调用点。
 */
void FOC_App_ParamIdentifyLoop(FOC_AppHandle_t *handle)
{
    (void)handle;
}

/**
 * @brief TIM2中断处理（已禁用）
 * @param handle FOC应用层句柄指针
 * 
 * 【注意】此函数已禁用！速度环、位置环和参数识别已移至TIM1中断
 * 
 * 当前架构（TIM1分频）：
 * - 速度环：2kHz (20kHz/10分频)
 * - 位置环：200Hz (20kHz/100分频)
 * - 参数识别：200Hz (20kHz/100分频)
 * 
 * 如需重新启用TIM2，请：
 * 1. 在 main.c 中启动 TIM2: HAL_TIM_Base_Start_IT(&htim2)
 * 2. 在 stm32h7xx_it.c 中恢复 TIM2_IRQHandler 的处理逻辑
 */
void FOC_App_TIM2_IRQHandler(FOC_AppHandle_t *handle)
{
    /* 所有控制环已移至TIM1中断
     * 在 TIM1_UP_IRQHandler 中通过分频调用
     */
    (void)handle;  /* 避免未使用警告 */
}

/**
 * @brief 使能FOC控制
 * @param handle FOC应用层句柄指针
 */
void FOC_App_Enable(FOC_AppHandle_t *handle)
{
    FOC_FaultCode_t fault = FOC_FAULT_NONE;
    uint8_t encoder_detected = 0U;
    uint8_t requires_stall_mode = 0U;
    uint8_t stall_enable = 0U;

    if ((handle == NULL) || !handle->power_unlocked) {
        return;
    }

    if ((handle->state != FOC_STATE_READY) && (handle->state != FOC_STATE_IDLE)) {
        return;
    }

    drv8350s.runtime.latchedFaultFlags = 0U;
    drv8350s.runtime.latchedFaultStatus1 = 0U;
    drv8350s.runtime.latchedVgsStatus2 = 0U;

    encoder_detected = TLE5012_IsDataValid();
    requires_stall_mode = (uint8_t)((!handle->motor_identified) || (!encoder_detected));

    if (requires_stall_mode) {
        if (!handle->stall_mode_armed) {
            handle->fault_code = FOC_FAULT_NONE;
            handle->enable_pwm = 0U;
            return;
        }

        stall_enable = 1U;
        handle->control_mode = FOC_MODE_SPEED;
    }

    /* Normal precheck contract remains: FOC_App_PrecheckPowerStage(handle, &fault) */
    if (!FOC_App_PrecheckPowerStage(handle, &fault, (uint8_t)(stall_enable == 0U))) {
        if (fault != FOC_FAULT_NONE) {
            FOC_App_EnterFault(handle, fault);
        }
        handle->enable_pwm = 0U;
        return;
    }

    if ((stall_enable == 0U) &&
        (handle->control_mode == FOC_MODE_POSITION) &&
        (handle->position_ref_user_set == 0U)) {
        FOC_App_RefreshEncoderFeedback(handle);
        float theta_mech_zeroed = FOC_AngleNormalize(handle->theta_mech - handle->motor_param.mech_zero_offset);
        float encoder_dir_f = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
        handle->pos_ref = FOC_AngleNormalize(theta_mech_zeroed * encoder_dir_f);
        handle->speed_ref = 0.0f;
        handle->pi_speed.integral = 0.0f;
    }

    /* 先上电驱动芯片，再开栅极，最后启动PWM */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);  /* 给DRV_EN上电留出稳定时间 */

    (void)DRV8350S_ClearFaults(&drv8350s);
    if (DRV8350S_EnableGateDrivers(&drv8350s) != 0) {
        FOC_App_EnterFault(handle, FOC_FAULT_DRV8350S);
        handle->enable_pwm = 0U;
        return;
    }

    ADC_Sampling_ResetTimingState();
    handle->foc.pi_d.integral = 0.0f;  /* 清除识别阶段残留积分 */
    handle->foc.pi_q.integral = 0.0f;
    FOC_App_PrimeNeutralPwm();
    handle->speed_loop_ready = 0U;
    handle->speed_ref_ramped = 0.0f;
    handle->speed_theta_prev = handle->theta_mech;
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    handle->enable_pwm = 1U;
    handle->fault_code = FOC_FAULT_NONE;
    handle->stall_open_loop_active = stall_enable;
    handle->stall_speed_ref_mech = 0.0f;
    if (stall_enable) {
        if (fabsf(handle->speed_ref) < 1e-3f) {
            handle->speed_ref = FOC_STALL_OPEN_LOOP_DEFAULT_SPEED_RAD_PER_S;
        }
        handle->speed_ref = FOC_Saturate(handle->speed_ref,
                                         FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S,
                                         -FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S);
        if (fabsf(handle->Iq_ref) < 1e-3f) {
            handle->Iq_ref = (handle->speed_ref >= 0.0f) ?
                             FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A :
                             -FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A;
        }
        handle->Id_ref = 0.0f;
        handle->Iq_ref = FOC_Saturate(handle->Iq_ref,
                                      FOC_STALL_OPEN_LOOP_CURRENT_MAX_A,
                                      -FOC_STALL_OPEN_LOOP_CURRENT_MAX_A);
        FOC_SetCurrentReference(&handle->foc, 0.0f, handle->Iq_ref);
        handle->theta_mech = encoder_detected ? handle->theta_mech : 0.0f;
        handle->stall_theta_elec = encoder_detected ? handle->theta_elec : 0.0f;
    } else {
        handle->stall_theta_elec = handle->theta_elec;
    }
    handle->state = FOC_STATE_RUNNING;
    
    /* 使能Rs在线估计 */
    MI_RsOnlineEstimator_Enable(&handle->rs_est, 1U);
}

/**
 * @brief 禁用FOC控制
 * @param handle FOC应用层句柄指针
 */
void FOC_App_Disable(FOC_AppHandle_t *handle)
{
    handle->enable_pwm = 0;
    FOC_App_ResetMotionState(handle);
    ADC_Sampling_ResetTimingState();
    
    /* 停止PWM输出 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* 关闭栅极驱动并保持DRV_EN上电，便于继续诊断且MOS仍保持Hi-Z */
    (void)DRV8350S_DisableGateDrivers(&drv8350s);
    
    /* 禁用Rs在线估计 */
    MI_RsOnlineEstimator_Enable(&handle->rs_est, 0);
    
    if (handle->state == FOC_STATE_RUNNING || handle->state == FOC_STATE_PARAM_IDENTIFY) {
        handle->state = handle->motor_identified ? FOC_STATE_READY : FOC_STATE_IDLE;
    }
}

uint32_t FOC_App_GetVoltageWarningFlags(const FOC_AppHandle_t *handle)
{
    uint32_t warningFlags = 0U;

    if (handle == NULL) {
        return 0U;
    }

    if (handle->Vbus < handle->protection.undervoltage_limit_v) {
        warningFlags |= FOC_WARNING_VBUS_UNDERVOLTAGE_BIT;
    }

    if (handle->Vbus > handle->protection.overvoltage_limit_v) {
        warningFlags |= FOC_WARNING_VBUS_OVERVOLTAGE_BIT;
    }

    return warningFlags;
}

uint8_t FOC_App_GetVoltageTripFault(const FOC_AppHandle_t *handle, FOC_FaultCode_t *fault)
{
    float severeUndervoltage;
    float severeOvervoltage;

    if (fault != NULL) {
        *fault = FOC_FAULT_NONE;
    }

    if (handle == NULL) {
        return 0U;
    }

    severeUndervoltage = handle->protection.undervoltage_limit_v - FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V;
    if (severeUndervoltage < 0.0f) {
        severeUndervoltage = 0.0f;
    }
    severeOvervoltage = handle->protection.overvoltage_limit_v + FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V;

    if (handle->Vbus < severeUndervoltage) {
        if (fault != NULL) {
            *fault = FOC_FAULT_UNDERVOLTAGE;
        }
        return 1U;
    }

    if (handle->Vbus > severeOvervoltage) {
        if (fault != NULL) {
            *fault = FOC_FAULT_OVERVOLTAGE;
        }
        return 1U;
    }

    return 0U;
}

uint8_t FOC_App_IsVoltageFaultRecovered(const FOC_AppHandle_t *handle, FOC_FaultCode_t fault)
{
    float severeUndervoltage;
    float severeOvervoltage;

    if (handle == NULL) {
        return 0U;
    }

    severeUndervoltage = handle->protection.undervoltage_limit_v - FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V +
                         FOC_VOLTAGE_FAULT_RECOVER_HYSTERESIS_V;
    if (severeUndervoltage < 0.0f) {
        severeUndervoltage = 0.0f;
    }
    severeOvervoltage = handle->protection.overvoltage_limit_v + FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V -
                        FOC_VOLTAGE_FAULT_RECOVER_HYSTERESIS_V;

    if (fault == FOC_FAULT_UNDERVOLTAGE) {
        return (handle->Vbus >= severeUndervoltage) ? 1U : 0U;
    }

    if (fault == FOC_FAULT_OVERVOLTAGE) {
        return (handle->Vbus <= severeOvervoltage) ? 1U : 0U;
    }

    return 0U;
}

void FOC_App_RefreshTelemetry(FOC_AppHandle_t *handle)
{
    ADC_Sampling_t *adc;

    if (handle == NULL) {
        return;
    }

    adc = ADC_Sampling_GetData();
    if (adc == NULL) {
        return;
    }

    if ((handle->state != FOC_STATE_RUNNING) &&
        (handle->state != FOC_STATE_PARAM_IDENTIFY)) {
        handle->Ia = adc->currentA;
        handle->Ib = adc->currentB;
        handle->Ic = adc->currentC;
    }
    handle->Vbus = adc->vbus;
    handle->warning_flags = FOC_App_GetVoltageWarningFlags(handle);
    if ((handle->state != FOC_STATE_RUNNING) &&
        (handle->state != FOC_STATE_PARAM_IDENTIFY) &&
        (handle->stall_open_loop_active == 0U)) {
        FOC_App_RefreshEncoderFeedback(handle);
    }
    FOC_SetVbus(&handle->foc, handle->Vbus);
}

void FOC_App_ResetMotionState(FOC_AppHandle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    handle->Id_ref = 0.0f;
    handle->Iq_ref = 0.0f;
    handle->speed_ref = 0.0f;
    handle->speed_ref_ramped = 0.0f;
    handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, handle->theta_mech);
    handle->speed_mech = 0.0f;
    handle->speed_elec = 0.0f;
    handle->speed_theta_prev = handle->theta_mech;
    handle->position_ref_user_set = 0U;
    handle->speed_loop_ready = 0U;
    handle->position_friction_active = 0U;
    handle->stall_open_loop_active = 0U;
    handle->stall_theta_elec = 0.0f;
    handle->stall_speed_ref_mech = 0.0f;
    handle->speed_loop_ref_diag = 0.0f;
    handle->speed_loop_mech_diag = 0.0f;
    handle->speed_loop_error_diag = 0.0f;
    handle->speed_loop_iq_mech_diag = 0.0f;
    handle->speed_loop_friction_diag = 0.0f;
    handle->speed_loop_iq_cmd_diag = 0.0f;
    handle->position_loop_error_diag = 0.0f;
    handle->position_loop_pd_out_diag = 0.0f;
    handle->position_loop_pd_sat_diag = 0U;
    handle->position_loop_speed_ramp_sat_diag = 0U;
    handle->position_loop_iq_pos_sat_diag = 0U;
    handle->position_loop_iq_neg_sat_diag = 0U;

    FOC_SetCurrentReference(&handle->foc, 0.0f, 0.0f);
    handle->foc.Iabc.a = 0.0f;
    handle->foc.Iabc.b = 0.0f;
    handle->foc.Iabc.c = 0.0f;
    handle->foc.IalphaBeta.alpha = 0.0f;
    handle->foc.IalphaBeta.beta = 0.0f;
    handle->foc.Idq.d = 0.0f;
    handle->foc.Idq.q = 0.0f;
    handle->foc.Vdq.d = 0.0f;
    handle->foc.Vdq.q = 0.0f;
    handle->foc.ValphaBeta.alpha = 0.0f;
    handle->foc.ValphaBeta.beta = 0.0f;
    FOC_SVPWM_Generate(&handle->foc.ValphaBeta, handle->foc.Vbus, &handle->foc.svpwm);
    handle->foc.pi_d.integral = 0.0f;
    handle->foc.pi_q.integral = 0.0f;
    handle->pi_speed.integral = 0.0f;
}

/**
 * @brief 设置电流参考值
 * @param handle FOC应用层句柄指针
 * @param Id_ref D轴电流参考值
 * @param Iq_ref Q轴电流参考值
 */
void FOC_App_SetCurrentRef(FOC_AppHandle_t *handle, float Id_ref, float Iq_ref)
{
    float current_limit;

    if (handle == NULL) {
        return;
    }

    current_limit = FOC_App_GetCurrentRefLimit(handle);
    Id_ref = FOC_Saturate(Id_ref, current_limit, -current_limit);
    Iq_ref = FOC_Saturate(Iq_ref, current_limit, -current_limit);

    if (handle->stall_open_loop_active || ((!handle->motor_identified) && handle->stall_mode_armed)) {
        Id_ref = 0.0f;
        Iq_ref = FOC_Saturate(Iq_ref,
                              FOC_STALL_OPEN_LOOP_CURRENT_MAX_A,
                              -FOC_STALL_OPEN_LOOP_CURRENT_MAX_A);
    }

    handle->Id_ref = Id_ref;
    handle->Iq_ref = Iq_ref;
    FOC_SetCurrentReference(&handle->foc, Id_ref, Iq_ref);
}

/**
 * @brief 设置速度参考值
 * @param handle FOC应用层句柄指针
 * @param speed_ref 速度参考值（rad/s）
 */
void FOC_App_SetSpeedRef(FOC_AppHandle_t *handle, float speed_ref)
{
    if (handle == NULL) {
        return;
    }

    if (handle->stall_open_loop_active || ((!handle->motor_identified) && handle->stall_mode_armed)) {
        speed_ref = FOC_Saturate(speed_ref,

			FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S,
                                 -FOC_STALL_OPEN_LOOP_SPEED_MAX_RAD_PER_S);
    }

    handle->speed_ref = speed_ref;
}

/**
 * @brief 设置位置参考值
 * @param handle FOC应用层句柄指针
 * @param pos_ref 位置参考值（rad）
 */
void FOC_App_SetPositionRef(FOC_AppHandle_t *handle, float pos_ref)
{
    if (handle == NULL) {
        return;
    }

    handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, pos_ref);
    handle->position_ref_user_set = 1U;
    handle->position_friction_active = 0U;
    handle->speed_ref_ramped = 0.0f;
    handle->pi_speed.integral = 0.0f;
}

void FOC_App_SetPositionPDGains(FOC_AppHandle_t *handle, float kp, float kd)
{
    if (handle == NULL) {
        return;
    }

    FOC_PositionPD_Init(&handle->pos_pd,
                        kp,
                        kd,
                        FOC_POSITION_SPEED_LIMIT_RAD_PER_S,
                        -FOC_POSITION_SPEED_LIMIT_RAD_PER_S);
}

/**
 * @brief 设置控制模式
 * @param handle FOC应用层句柄指针
 * @param mode 控制模式（力矩/速度/位置）
 */
void FOC_App_SetControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode)
{
    if (handle == NULL) {
        return;
    }

    if (handle->stall_open_loop_active && (mode == FOC_MODE_POSITION)) {
        mode = FOC_MODE_SPEED;
    }

    handle->control_mode = mode;
    if ((mode == FOC_MODE_POSITION) && (handle->enable_pwm == 0U)) {
        FOC_App_RefreshEncoderFeedback(handle);
        float theta_mech_zeroed = FOC_AngleNormalize(handle->theta_mech - handle->motor_param.mech_zero_offset);
        float encoder_dir_f = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
        handle->pos_ref = FOC_AngleNormalize(theta_mech_zeroed * encoder_dir_f);
        handle->speed_ref = 0.0f;
        handle->position_ref_user_set = 0U;
    }

    /* 切换模式时清零积分，防止跳变 */
    handle->pi_speed.integral = 0.0f;
    handle->position_friction_active = 0U;
}

void FOC_App_SetVoltageThresholds(FOC_AppHandle_t *handle, float undervoltage, float overvoltage)
{
    if (handle == NULL) {
        return;
    }

    if ((undervoltage <= 0.0f) || (overvoltage <= undervoltage)) {
        return;
    }

    if ((handle->enable_pwm != 0U) ||
        (handle->state == FOC_STATE_RUNNING) ||
        (handle->state == FOC_STATE_PARAM_IDENTIFY)) {
        return;
    }

    handle->protection.undervoltage_limit_v = undervoltage;
    handle->protection.overvoltage_limit_v = overvoltage;
}

void FOC_App_SetPolePairs(FOC_AppHandle_t *handle, uint8_t pole_pairs)
{
    if (handle == NULL) {
        return;
    }

    if ((pole_pairs == 0U) || (pole_pairs > 50U)) {
        return;
    }

    if ((handle->enable_pwm != 0U) ||
        (handle->state == FOC_STATE_RUNNING) ||
        (handle->state == FOC_STATE_PARAM_IDENTIFY)) {
        return;
    }

    handle->motor_param.Pn = pole_pairs;
    handle->motor_param.valid_flag = 0U;
    handle->motor_identified = 0U;
    handle->mi_handle.pn_last_calc = 0.0f;
    handle->mi_handle.pn_last_delta_mech = 0.0f;
    handle->mi_handle.pn_last_delta_elec = 0.0f;
    handle->mi_handle.pn_observed_dir = 0;
    FOC_App_UpdateIdentifyState(handle);
    if (handle->state == FOC_STATE_READY) {
        handle->state = FOC_STATE_IDLE;
    }
}

static uint8_t FOC_App_PrecheckPowerStage(FOC_AppHandle_t *handle, FOC_FaultCode_t *fault, uint8_t require_encoder)
{
    uint16_t fs1 = 0U;
    uint16_t fs2 = 0U;
    uint16_t ocp = 0U;

    if (fault != NULL) {
        *fault = FOC_FAULT_NONE;
    }

    if (handle == NULL) {
        if (fault != NULL) {
            *fault = FOC_FAULT_PARAM_INVALID;
        }
        return 0U;
    }

    FOC_App_RefreshTelemetry(handle);

    if (FOC_App_GetVoltageTripFault(handle, fault)) {
        return 0U;
    }

    if ((require_encoder != 0U) && !FOC_App_IsEncoderReadyForPowerStage(fault)) {
        return 0U;
    }

    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    if ((DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_FAULT_STATUS_1, &fs1) != 0) ||
        (DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_VGS_STATUS_2, &fs2) != 0) ||
        (DRV8350S_ReadRegister(&drv8350s, DRV8350S_REG_OCP_CTRL, &ocp) != 0)) {
        drv8350s.runtime.spiError = 1U;
        drv8350s.readReq.registerAddr = DRV8350S_REG_OCP_CTRL;
        DRV8350S_UpdateFaultState(&drv8350s);
        if (fault != NULL) {
            *fault = FOC_FAULT_DRV8350S;
        }
        return 0U;
    }

    drv8350s.runtime.regFaultStatus1 = fs1;
    drv8350s.runtime.regVgsStatus2 = fs2;
    drv8350s.runtime.regOcpCtrl = ocp;
    drv8350s.runtime.spiError = 0U;
    drv8350s.readReq.registerAddr = DRV8350S_REG_OCP_CTRL;
    DRV8350S_UpdateFaultState(&drv8350s);

    if (drv8350s.runtime.isFaultActive != 0U) {
        if (fault != NULL) {
            *fault = FOC_FAULT_DRV8350S;
        }
        return 0U;
    }

    return 1U;
}

static void FOC_App_RefreshEncoderFeedback(FOC_AppHandle_t *handle)
{
    float angle_deg;
    float encoder_dir;
    uint8_t pole_pairs;

    if ((handle == NULL) || !TLE5012_IsDataValid()) {
        return;
    }

    angle_deg = TLE5012_GetAngle();
    handle->theta_mech = FOC_AngleNormalize(angle_deg * FOC_PI / 180.0f);
    handle->theta_sample_seq++;

    encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
    pole_pairs = (handle->motor_param.Pn > 0U) ? handle->motor_param.Pn : 1U;
    handle->theta_elec = FOC_AngleNormalize((handle->theta_mech * encoder_dir * (float)pole_pairs) +
                                            handle->motor_param.theta_offset);
}

/**
 * @brief 根据电机参数计算控制环参数
 * @param handle FOC应用层句柄指针
 * 
 * 三环带宽分配原则（从外到内，带宽依次增加5-10倍）：
 * - 位置环带宽: 10-20 Hz (机械响应最慢)
 * - 速度环带宽: 5 Hz bench-safe initial value
 * - 电流环带宽: 1000-2000 Hz (比速度环快10倍)
 * 
 * 计算公式（典型工程整定）：
 * - 电流环 Kp = Ld * 2π * fc, Ki = Rs * 2π * fc * T
 * - 速度环 Kp = J * 2π * fs / Kt, Ki = Kp * Rs / Ld (或根据机械时间常数)
 * - 位置环 Kp = 2π * fp，Kd 用于速度反馈阻尼
 */
static void FOC_App_UpdateLoopParams(FOC_AppHandle_t *handle)
{
    /* 12V/8.8R/low-side bench baseline. The previous 2kHz bandwidth formula
       amplified reconstruction spikes into bus-limited voltage at zero current. */
    float Kp_i = FOC_CURRENT_LOOP_KP_12V_BENCH;
    float Ki_i = FOC_CURRENT_LOOP_KI_12V_BENCH;
    
    /* 更新电流环PI */
    FOC_Init(&handle->foc, Kp_i, Ki_i, Kp_i, Ki_i);
    /* 电流环积分分离：误差>1.5A时暂停积分，防止瞬态过流 */
    handle->foc.pi_d.integral_sep_thresh = 1.5f;
    handle->foc.pi_q.integral_sep_thresh = 1.5f;
    FOC_SetCurrentResistance(&handle->foc, handle->motor_param.Rs);
#if FOC_FF_ENABLE_BEMF
    FOC_SetBemfParams(&handle->foc, handle->motor_param.Ld, handle->motor_param.Lq, handle->motor_param.Ke);
#endif

#if FOC_FF_ENABLE_OBSERVER
    /* Configure torque observer from motor params */
    {
        FOC_TorqueObserver_t *obs = &handle->torque_obs;
        obs->J_hat = (handle->motor_param.J > 0.0f) ? handle->motor_param.J : 0.0001f;
        obs->B_hat = (handle->motor_param.B >= 0.0f) ? handle->motor_param.B : 0.001f;
        obs->Kt = (handle->motor_param.Ke > 0.0f) ? handle->motor_param.Ke : 0.129f;
        obs->l = FOC_FF_OBSERVER_GAIN_L;
        obs->enabled = (handle->motor_param.J > 0.0f && handle->motor_param.Ke > 0.0f) ? 1U : 0U;
    }
#endif

#if FOC_FF_ENABLE_COGGING
    /* Load cogging LUT from Flash */
    {
        ParamStatus_t cogging_status;
        uint16_t cogging_size = 0U;
        cogging_status = Param_LoadCoggingLUT(handle->cogging_lut.table, &cogging_size);
        if (cogging_status == PARAM_OK && cogging_size > 0U) {
            handle->cogging_lut.valid_size = cogging_size;
            handle->cogging_lut.valid = 1U;
        } else {
            handle->cogging_lut.valid_size = 0U;
            handle->cogging_lut.valid = 0U;
        }
    }
#endif

    /* 速度环参数 - 固定值，台架测试验证(12V, 24N22P, 74KV) */
    /* 使用台架实测整定值，不使用Ke/J自动计算(默认Ke/J参数不准确) */
    float Kp_s = 0.30f;  /* 固定比例增益 */
    float Ki_s = 0.0f;   /* 2kHz速度积分先禁用，避免Rs前馈下低速冲飞 */
    
    /* 更新速度环PI */
    {
        float speed_current_limit = FOC_App_GetCurrentRefLimit(handle);
        FOC_PI_Init(&handle->pi_speed, Kp_s, Ki_s, speed_current_limit, -speed_current_limit);
    }
    
    /* 位置环参数计算 - 显式PD
     * Kp_pos: 位置误差(rad) -> 速度给定(rad/s)
     * Kd_pos: 实际速度反馈阻尼系数
     */
    float Kp_p = FOC_POSITION_PD_KP_DEFAULT;
    float Kd_p = FOC_POSITION_PD_KD_DEFAULT;

    if (Kp_s > 0.0f) {
        Kp_p = sqrtf(Kp_s) * FOC_POSITION_PD_KP_SCALE;
        if (Kp_p < FOC_POSITION_PD_KP_MIN) Kp_p = FOC_POSITION_PD_KP_MIN;
        if (Kp_p > 100.0f) Kp_p = 100.0f;

        Kd_p = 0.01f * Kp_p;
        if (Kd_p < FOC_POSITION_PD_KD_DEFAULT) Kd_p = FOC_POSITION_PD_KD_DEFAULT;
        if (Kd_p > 0.25f) Kd_p = 0.25f;
    }

    /* 更新位置环PD */
    FOC_PositionPD_Init(&handle->pos_pd,
                        Kp_p,
                        Kd_p,
                        FOC_POSITION_SPEED_LIMIT_RAD_PER_S,
                        -FOC_POSITION_SPEED_LIMIT_RAD_PER_S);

    /* 冷启动门禁：若已识别但 enc_dir != -1，阻止所有前馈验证
     * V4基线要求 enc_dir=-1；enc_dir=1 说明参数不是V4识别的，需重新识别
     */
    handle->ff_blocked_by_enc_dir = 0U;
    if (handle->motor_identified && handle->motor_param.encoder_dir != -1) {
        handle->ff_blocked_by_enc_dir = 1U;
    }

    /* 可选: 打印调试信息 */
    /*
    printf("Loop Params Updated:\r\n");
    printf("  Current: Kp=%.4f, Ki=%.4f\r\n", Kp_i, Ki_i);
    printf("  Speed:   Kp=%.4f, Ki=%.4f\r\n", Kp_s, Ki_s);
    printf("  Position:Kp=%.4f, Kd=%.4f\r\n", Kp_p, Kd_p);
    */
}

static void FOC_PositionPD_Init(FOC_PositionPD_t *pd, float kp, float kd, float output_max, float output_min)
{
    if (pd == NULL) {
        return;
    }

    pd->kp = kp;
    pd->kd = kd;
    pd->output_max = output_max;
    pd->output_min = output_min;
}

static float FOC_PositionPD_Update(const FOC_PositionPD_t *pd, float pos_error, float speed_feedback)
{
    float output;

    if (pd == NULL) {
        return 0.0f;
    }

    output = (pd->kp * pos_error) - (pd->kd * speed_feedback);
    return FOC_Saturate(output, pd->output_max, pd->output_min);
}

/**
 * @brief 从Flash加载参数
 * @param handle FOC应用层句柄指针
 */
void FOC_App_LoadParam(FOC_AppHandle_t *handle)
{
    ParamStatus_t status = Param_Load(&handle->motor_param);
    if (status != PARAM_OK || !Param_IsValid(&handle->motor_param)) {
        /* 加载失败，使用默认参数 */
        Param_SetDefault(&handle->motor_param);
    }
    FOC_App_ApplyKnownMotorConstants(handle);
    FOC_App_UpdateIdentifyState(handle);
}

/**
 * @brief 保存参数到Flash
 * @param handle FOC应用层句柄指针
 */
void FOC_App_SaveParam(FOC_AppHandle_t *handle)
{
    Param_Save(&handle->motor_param);
    /* Flash write stalls CPU → TLE5012 SPI transaction may be interrupted.
       Abort any stuck SPI transfer and reset encoder state machine. */
    HAL_SPI_Abort(&hspi3);
    TLE5012_Init();
}

/**
 * @brief 启动参数识别
 * @param handle FOC应用层句柄指针
 */
void FOC_App_StartIdentify(FOC_AppHandle_t *handle)
{
    FOC_FaultCode_t fault = FOC_FAULT_NONE;

    if (!handle->power_unlocked) {
        return;
    }

    if (handle->state == FOC_STATE_IDLE || handle->state == FOC_STATE_READY) {
        if (!FOC_App_PrecheckPowerStage(handle, &fault, 1U)) {
            if (fault != FOC_FAULT_NONE) {
                FOC_App_EnterFault(handle, fault);
            }
            handle->enable_pwm = 0U;
            if (fault != FOC_FAULT_NONE) {
                handle->enable_identify = 0U;
            }
            return;
        }

        /* 参数识别需要实际激励：确保功率级已上电并允许PWM输出 */
        if (!handle->enable_pwm) {
            drv8350s.runtime.latchedFaultFlags = 0U;
            drv8350s.runtime.latchedFaultStatus1 = 0U;
            drv8350s.runtime.latchedVgsStatus2 = 0U;
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
            HAL_Delay(1);
            (void)DRV8350S_ClearFaults(&drv8350s);
            if (DRV8350S_EnableGateDrivers(&drv8350s) != 0) {
                FOC_App_EnterFault(handle, FOC_FAULT_DRV8350S);
                handle->enable_pwm = 0U;
                return;
            }

            FOC_App_PrimeNeutralPwm();
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            handle->enable_pwm = 1U;
        }

        ADC_Sampling_ResetTimingState();
        TLE5012_ClearDiagnosticCounters();

        /* 初始化参数识别 */
        MI_Init(&handle->mi_handle, &handle->motor_param, &handle->foc);
        MI_StartIdentify(&handle->mi_handle);
        
        handle->state = FOC_STATE_PARAM_IDENTIFY;
        handle->enable_identify = 1U;
    }
}

/**
 * @brief 中止参数识别
 * @param handle FOC应用层句柄指针
 */
void FOC_App_StopIdentify(FOC_AppHandle_t *handle)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    handle->enable_identify = 0;
    ADC_Sampling_ResetTimingState();
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 检查参数识别是否完成
 * @param handle FOC应用层句柄指针
 * @return 1完成，0未完成
 */
uint8_t FOC_App_IsIdentifyComplete(FOC_AppHandle_t *handle)
{
    return MI_IsComplete(&handle->mi_handle);
}

/**
 * @brief 获取FOC状态
 * @param handle FOC应用层句柄指针
 * @return FOC状态
 */
FOC_AppState_t FOC_App_GetState(FOC_AppHandle_t *handle)
{
    return handle->state;
}

/**
 * @brief 获取故障代码
 * @param handle FOC应用层句柄指针
 * @return 故障代码
 */
FOC_FaultCode_t FOC_App_GetFault(FOC_AppHandle_t *handle)
{
    return handle->fault_code;
}

/**
 * @brief 获取状态字符串
 * @param state FOC状态
 * @return 状态描述字符串
 */
const char* FOC_App_GetStateString(FOC_AppState_t state)
{
    switch (state) {
        case FOC_STATE_IDLE:            return "IDLE";
        case FOC_STATE_INIT:            return "INIT";
        case FOC_STATE_PARAM_IDENTIFY:  return "IDENTIFY";
        case FOC_STATE_READY:           return "READY";
        case FOC_STATE_RUNNING:         return "RUNNING";
        case FOC_STATE_FAULT:           return "FAULT";
        default:                        return "UNKNOWN";
    }
}

/**
 * @brief 获取故障字符串
 * @param fault 故障代码
 * @return 故障描述字符串
 */
const char* FOC_App_GetFaultString(FOC_FaultCode_t fault)
{
    switch (fault) {
        case FOC_FAULT_NONE:            return "None";
        case FOC_FAULT_OVERCURRENT:     return "Overcurrent";
        case FOC_FAULT_OVERVOLTAGE:     return "Overvoltage";
        case FOC_FAULT_UNDERVOLTAGE:    return "Undervoltage";
        case FOC_FAULT_ENCODER:         return "Encoder";
        case FOC_FAULT_DRV8350S:        return "DRV8350S";
        case FOC_FAULT_PARAM_INVALID:   return "Param Invalid";
        case FOC_FAULT_ADC_SAMPLING:    return "ADC Sampling";
        default:                        return "Unknown";
    }
}

/**
 * @brief 获取调试信息
 * @param handle FOC应用层句柄指针
 * @param Id D轴电流输出
 * @param Iq Q轴电流输出
 * @param Vd D轴电压输出
 * @param Vq Q轴电压输出
 * @param theta 电角度输出
 * @param speed 转速输出
 * @param Rs_est Rs估计值输出
 */
void FOC_App_GetDebugInfo(FOC_AppHandle_t *handle, float *Id, float *Iq, float *Vd, float *Vq, 
                          float *theta, float *speed, float *Rs_est)
{
    *Id = handle->foc.Idq.d;
    *Iq = handle->foc.Idq.q;
    *Vd = handle->foc.Vdq.d;
    *Vq = handle->foc.Vdq.q;
    *theta = handle->theta_elec;
    *speed = handle->speed_mech;
    *Rs_est = MI_RsOnlineEstimator_GetRs(&handle->rs_est);
}
