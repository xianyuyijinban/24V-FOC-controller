/**
 * @file    foc_app.c
 * @brief   FOC应用层接口实现
 * @note    整合FOC核心、硬件驱动、参数识别和存储
 */

#include "foc_app.h"
#include "current_stream.h"
#include "foc_profiler.h"
#include "wheel_input.h"
#include "tle5012.h"
#include "uart_upload.h"
#include "cogging_lut_cal.h"
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
    handle->app_mode = APP_MODE_RAW;        /* Phase 3: 默认原始模式 */
    handle->gimbal_ramp_accel_radps2 = FOC_GIMBAL_RAMP_ACCEL_DEFAULT;
    handle->cal_state = 0U;                    /* Phase 4: idle */
    handle->cal_last_error = 0U;
    handle->spring_K        = FOC_SPRING_K_DEFAULT;
    handle->spring_D        = FOC_SPRING_D_DEFAULT;
    handle->spring_limit_A  = FOC_SPRING_LIMIT_DEFAULT;
    handle->detent_count    = FOC_DETENT_COUNT_DEFAULT;
    handle->detent_strength = FOC_DETENT_STRENGTH_DEFAULT;
    handle->detent_width_rad= FOC_DETENT_WIDTH_DEFAULT;
    handle->detent_damping  = FOC_DETENT_DAMPING_DEFAULT;
    handle->detent_limit_A  = FOC_DETENT_LIMIT_DEFAULT;
    /* SCROLL_WHEEL independent config */
    handle->wheel_count     = FOC_WHEEL_COUNT_DEFAULT;
    handle->wheel_strength  = FOC_WHEEL_STRENGTH_DEFAULT;
    handle->wheel_width_rad = FOC_WHEEL_WIDTH_DEFAULT;
    handle->wheel_damping   = FOC_WHEEL_DAMPING_DEFAULT;
    handle->wheel_limit_A   = FOC_WHEEL_LIMIT_DEFAULT;
    handle->wheel_cfg_active = 0U;
    BlackBox_Init();                           /* Phase 5A */
    handle->joint_pos_limit_min_rad = -0.524f;  /* -30 deg */
    handle->joint_pos_limit_max_rad =  0.524f;  /* +30 deg */
    handle->joint_soft_limit_enabled = FOC_JOINT_SOFT_LIMIT_DEFAULT_ENABLED;
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
    FOC_PI_Init(&handle->pi_speed, 0.25f, 0.001f, 10.0f, -10.0f);

#if FOC_FF_ENABLE_OBSERVER
    /* 初始化负载转矩观测器（参数在UpdateLoopParams中更新） */
    handle->torque_obs.z = 0.0f;
    handle->torque_obs.T_est = 0.0f;
    handle->torque_obs.T_lpf = 0.0f;
    handle->torque_obs.enabled = 0U;
#endif

    /* V5 位置模式运行时运动配置默认值 */
    handle->position_speed_limit_radps  = FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
    handle->position_accel_limit_radps2 = FOC_MOTION_CFG_ACCEL_LIMIT_DEFAULT;
    handle->position_cruise_speed_radps = FOC_MOTION_CFG_CRUISE_SPEED_DEFAULT;

    /* 初始化位置环PD控制器 - 输出速度给定 */
    FOC_PositionPD_Init(&handle->pos_pd,
                        FOC_POSITION_PD_KP_DEFAULT,
                        FOC_POSITION_PD_KD_DEFAULT,
                        handle->position_speed_limit_radps,
                        -handle->position_speed_limit_radps);

    /* 初始化位置环直连PD控制器 - 输出力矩给定 A（判别实验，默认关闭） */
    FOC_PositionPD_Init(&handle->pos_pd_direct,
                        FOC_POS_DIRECT_KP_DEFAULT,
                        FOC_POS_DIRECT_KD_DEFAULT,
                        FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A,
                        -FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A);
    handle->pos_direct = 0U;
    handle->pos_direct_iq_cmd = 0.0f;
    handle->pos_direct_ki = FOC_POS_DIRECT_KI_DEFAULT;
    handle->pos_integral = 0.0f;
    handle->pos_integral_err_rad = FOC_POS_INTEGRAL_ERR_RAD;
    handle->pos_cmd_dir = 0.0f;
    handle->pos_cmd_dir_hold = 0U;
    handle->pos_ref_prev = handle->pos_ref;
    
    /* 初始化Rs在线估计器 */
    MI_RsOnlineEstimator_Init(&handle->rs_est, 0.01f);

    /* P0 cogging runtime defaults (overridable via CMD:COG_CFG) */
    handle->cogging_lut.gain = 0.25f;
    handle->cogging_lut.phase_offset_rad = FOC_PI / 3.0f;  /* +60 deg */

    /* 静摩擦补偿运行时幅值默认 (overridable via CMD:FRIC_COMP) */
    handle->fric_comp_pos = FOC_POSITION_USER_POSITIVE_STATIC_FRICTION_COMP_A;
    handle->fric_comp_neg = FOC_POSITION_USER_NEGATIVE_STATIC_FRICTION_COMP_A;

    /* 尝试加载参数 */
    FOC_App_LoadParam(handle);
    /* 标定齿槽 LUT 覆盖(替代识别/Flash 低质 LUT; 2026-08-14 台架标定, encoder_dir=-1 已对齐) */
    memcpy(handle->cogging_lut.table, COGGING_LUT_CAL, sizeof(float) * FOC_COGGING_LUT_SIZE);
    handle->cogging_lut.valid_size = FOC_COGGING_LUT_SIZE;
    handle->cogging_lut.valid = 1U;
    handle->cogging_lut.gain = 1.0f;
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
        uint32_t foc_run_start = FOC_Profiler_Begin();
        FOC_Run(&handle->foc);
        FOC_Profiler_End(FOC_PROBE_FOC_RUN, foc_run_start);
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

    /* ── V1.1: Current stream sample push (all FOC states) ── */
    {
        CurStreamSample_t s;
        s.tick_ms = HAL_GetTick();
        s.ia_mA   = (int16_t)(handle->Ia * 1000.0f);
        s.ib_mA   = (int16_t)(handle->Ib * 1000.0f);
        s.ic_mA   = (int16_t)(handle->Ic * 1000.0f);
        /* Id/Iq only valid in RUNNING/IDENTIFY; zero otherwise */
        if (handle->state == FOC_STATE_RUNNING || handle->state == FOC_STATE_PARAM_IDENTIFY) {
            s.id_mA = (int16_t)(handle->foc.Idq.d * 1000.0f);
            s.iq_mA = (int16_t)(handle->foc.Idq.q * 1000.0f);
        } else {
            s.id_mA = 0;
            s.iq_mA = 0;
        }
        s.vbus_mV = (uint16_t)(handle->Vbus * 1000.0f);
        s.flags   = 0U;
        CurStream_PushSample(&s);
    }
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

    BlackBox_Freeze((uint8_t)fault);  /* Phase 5A: freeze on fault */
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

/* Forward declaration: used in SpeedLoop before its full definition below */
static uint8_t FOC_App_IsHapticMode(const FOC_AppHandle_t *handle);

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
    BlackBox_Sample();  /* Phase 5A: 50Hz sampling */

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
        FOC_SetRsFFSpeedError(&handle->foc, 0.0f);  /* 开环堵转：速度不可靠 */
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
        FOC_SetRsFFSpeedError(&handle->foc, 0.0f);  /* 初始化：尚无有效速度误差 */
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
    handle->speed_elec = handle->speed_mech * handle->motor_param.Pn;
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
        FOC_SetRsFFSpeedError(&handle->foc, 0.0f);  /* 无速度误差，不惩罚RsFF */
        handle->speed_loop_count++;
        return;
    } else if (handle->control_mode == FOC_MODE_POSITION) {
        float speed_ramp_delta = handle->speed_ref - handle->speed_ref_ramped;
        float speed_ramp_step = handle->position_accel_limit_radps2 / (float)FOC_SPEED_LOOP_FREQ;
        /* 位置模式复用速度给定斜坡，避免位置步进把速度环瞬间推到限幅。 */
        handle->speed_ref_ramped += FOC_Saturate(speed_ramp_delta,
                                                 speed_ramp_step,
                                                 -speed_ramp_step);
        speed_ref_temp = handle->speed_ref_ramped;
        handle->position_loop_speed_ramp_sat_diag = (fabsf(speed_ramp_delta) > speed_ramp_step) ? 1U : 0U;
    } else {
        /* 速度模式：使用速度给定斜坡 */
        /* Phase 3: GIMBAL_SPEED uses gentler ramp accel for low-speed smoothness */
        float ramp_rate = (handle->app_mode == APP_MODE_GIMBAL_SPEED)
            ? handle->gimbal_ramp_accel_radps2
            : FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2;
        handle->speed_ref_ramped += FOC_Saturate(handle->speed_ref - handle->speed_ref_ramped,
                                                 ramp_rate / (float)FOC_SPEED_LOOP_FREQ,
                                                 -ramp_rate / (float)FOC_SPEED_LOOP_FREQ);
        speed_ref_temp = handle->speed_ref_ramped;
        handle->position_loop_speed_ramp_sat_diag = 0U;
    }

    /* Speed and position commands are user-frame mechanical values. Keep the
       outer loops in that frame, then map the torque command to the q-axis once. */
    if ((handle->obs_use_speed != 0U) && (handle->speed_obs.valid != 0U)) {
        /* 速度环反馈用观测器速度(平滑, 低速 0.1°/s 不被差分量化噪声淹没)。
         * 低速专测; 高速观测器低带宽(w0~12)滞后, 需 obs_use_speed=0 切回差分。 */
        speed_feedback = handle->speed_obs_user;
    } else {
        speed_feedback = speed_mech_user;
    }
    speed_error = speed_ref_temp - speed_feedback;
    /* 将速度误差传给 FOC 核心，用于自适应 Rs 前馈置信度计算 */
    FOC_SetRsFFSpeedError(&handle->foc, speed_error);
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
        /* Haptic modes (SPRING_DAMPER / DETENT): bypass speed PI + all feedforward;
         * torque is computed purely from spring/detent physics below. */
        if (FOC_App_IsHapticMode(handle)) {
            iq_ref_mech = 0.0f;
            goto haptic_torque_injection;
        }
        /* ── 位置环直连电流环 (判别实验): 跳过速度PI，位置环力矩指令
         *    直接进FF层(齿槽LUT/摩擦/惯量)与静摩擦补偿 ── */
        if ((handle->control_mode == FOC_MODE_POSITION) && (handle->pos_direct != 0U)) {
            handle->pi_speed.integral = 0.0f;
            FOC_SetRsFFSpeedError(&handle->foc, 0.0f);  /* 直连：速度误差不参与控制 */
            iq_ref_mech = FOC_Saturate(handle->pos_direct_iq_cmd, iq_limit_pos, iq_limit_neg);
            handle->speed_loop_p_iq = iq_ref_mech;
            handle->speed_loop_i_iq = 0.0f;
            handle->speed_i_state = 5U;  /* 5 = pos_direct bypass */
            goto ff_layers;
        }
        /* ── Speed PI with conditional integration ── */
        {
            float p_iq = handle->pi_speed.Kp * speed_error;
            float i_iq = 0.0f;
            uint8_t i_state = 0U;  /* inactive */

            if (handle->control_mode == FOC_MODE_SPEED && handle->enable_pwm != 0U) {
                float abs_sref = fabsf(speed_ref_temp);

                if (abs_sref < 0.0005f) {
                    /* SREF ~0 (静止): clear integrator to prevent residual creep.
                     * 原阈值0.02rad/s把0.1°/s(=0.0017)也清了积分→速度环P-only→低速无法
                     * 克服静摩擦不动(实测速度模式0.1°/s仅15%)。下调后极低速可积分。 */
                    handle->pi_speed.integral = 0.0f;
                    i_state = 3U;  /* cleared */
                } else if (abs_sref >= 0.0005f && fabsf(speed_error) >= 0.0005f) {
                    /* Active zone: allow integration */
                    handle->pi_speed.integral += speed_error;
                    i_state = 1U;  /* active */
                } else {
                    /* Transition zone or small error: freeze integrator
                     * (holds torque needed to maintain steady speed) */
                    i_state = 2U;  /* frozen */
                }

                /* Integral contribution limit ±0.10A (converted to integral domain) */
                {
                    const float i_contrib_limit_A = 0.10f;
                    float integral_limit;
                    if (handle->pi_speed.Ki > 1e-6f) {
                        integral_limit = i_contrib_limit_A / handle->pi_speed.Ki;
                    } else {
                        integral_limit = 0.0f;
                    }
                    handle->pi_speed.integral = FOC_Saturate(
                        handle->pi_speed.integral, integral_limit, -integral_limit);
                }

                i_iq = handle->pi_speed.Ki * handle->pi_speed.integral;
                float raw_output = p_iq + i_iq;

                /* Anti-windup: when output exceeds limits, undo integration
                 * that pushes further into saturation */
                if (raw_output > iq_limit_pos && speed_error > 0.0f && i_state == 1U) {
                    handle->pi_speed.integral -= speed_error;
                    i_iq = handle->pi_speed.Ki * handle->pi_speed.integral;
                    i_state = 4U;  /* sat_hold */
                } else if (raw_output < iq_limit_neg && speed_error < 0.0f && i_state == 1U) {
                    handle->pi_speed.integral -= speed_error;
                    i_iq = handle->pi_speed.Ki * handle->pi_speed.integral;
                    i_state = 4U;  /* sat_hold */
                }

                iq_ref_mech = FOC_Saturate(p_iq + i_iq, iq_limit_pos, iq_limit_neg);
            } else {
                /* Position mode or disabled: P-only, clear integrator */
                handle->pi_speed.integral = 0.0f;
                iq_ref_mech = FOC_Saturate(p_iq, iq_limit_pos, iq_limit_neg);
            }

            /* Diagnostics */
            handle->speed_loop_p_iq = p_iq;
            handle->speed_loop_i_iq = i_iq;
            handle->speed_i_state = i_state;
        }

ff_layers:
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
                float omega = speed_mech_user;  /* 高速库仑方向用差分速度 */
                float omega_smooth = omega;     /* Stribeck 平滑速度(优先观测器平滑) */
                if ((handle->speed_obs.valid != 0U)) {
                    omega_smooth = handle->speed_obs_user;
                }
                float friction_total = 0.0f;
                /* Coulomb: 高速用速度方向, 低速用指令方向兜底
                 * - POSITION: pos_cmd_dir (方向锁存, 连续斜坡)
                 * - SPEED: SREF 指令符号 (差分速度噪声淹没 0.1°/s 信号, 速度方向不可靠) */
                float coulomb_dir;
                if (fabsf(omega) > FOC_FF_COULOMB_DEADBAND_RADPS) {
                    coulomb_dir = (omega > 0.0f) ? 1.0f : -1.0f;
                } else if ((handle->control_mode == FOC_MODE_POSITION) &&
                           (handle->pos_cmd_dir != 0.0f)) {
                    coulomb_dir = handle->pos_cmd_dir;
                } else if ((handle->control_mode == FOC_MODE_SPEED) &&
                           (fabsf(speed_ref_temp) > 1e-5f)) {
                    coulomb_dir = (speed_ref_temp > 0.0f) ? 1.0f : -1.0f;
                } else {
                    coulomb_dir = 0.0f;
                }
                if (coulomb_dir != 0.0f) {
                    /* 幅值: 先验摩擦 fric_comp (FRIC_COMP 运行时设, 匹配启动电流)。
                     * 速度模式原本用 Tc/Kt, 但 Tc 未识别=0 → 库仑前馈恒 0, 低速全靠
                     * 速度环积分硬扛。统一用 fric_comp 先验。 */
                    float fric = (coulomb_dir > 0.0f) ?
                                 handle->fric_comp_pos : handle->fric_comp_neg;
                    float coulomb = coulomb_dir * fric;
                    /* Stribeck 平滑: 极低速(起动)给满静摩擦, 随速度指数衰减到动摩擦。
                     * 用观测器平滑速度(omega_lpf)而非差分: 差分噪声使 smooth 波动 → 前馈抖动。
                     * fric_vs 运行时校准(0.1°/s 需更小 VS 让运动时衰减到动摩擦, 否则过驱动)。 */
                    float stick = expf(-fabsf(omega_smooth) / handle->fric_vs);
                    float smooth = handle->fric_kin
                                   + (1.0f - handle->fric_kin) * stick;
                    friction_total += coulomb * smooth;
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

        /* 观测器 T_hat 前馈: 速度模式 + obs_use_speed 时, 扰动(摩擦)估计直接补偿,
         * 速度环不需积分硬扛摩擦(低速起动/稳定, Tc 未识别=0 时尤其关键)。
         * 用上一拍 T_hat(观测器在 FF 层尾更新)。 */
        if ((handle->control_mode == FOC_MODE_SPEED) &&
            (handle->obs_use_speed != 0U) &&
            (handle->speed_obs.valid != 0U)) {
            float Kt_obs = handle->motor_param.Ke;
            if (fabsf(Kt_obs) > 1e-10f) {
                float t_ff = handle->speed_obs.T_hat_lpf / Kt_obs;
                t_ff = FOC_Saturate(t_ff, FOC_FF_FRICTION_MAX_A, -FOC_FF_FRICTION_MAX_A);
                handle->ff_diag.friction_iq += t_ff;
                iq_ref_mech += t_ff;
                iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);
            }
        }

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
                float theta_lookup = FOC_AngleNormalize(handle->theta_mech
                                       + handle->cogging_lut.phase_offset_rad);
                /* Map theta [-PI, PI) to index [0, FOC_COGGING_LUT_SIZE) */
                float index_f = (theta_lookup + FOC_PI) / (2.0f * FOC_PI) * (float)FOC_COGGING_LUT_SIZE;
                int idx = (int)index_f;
                float frac = index_f - (float)idx;
                if (idx >= FOC_COGGING_LUT_SIZE) idx = 0;
                if (idx < 0) idx = FOC_COGGING_LUT_SIZE - 1;
                int idx_next = (idx + 1) % FOC_COGGING_LUT_SIZE;
                float cogging_ff = (handle->cogging_lut.table[idx] * (1.0f - frac)
                                  + handle->cogging_lut.table[idx_next] * frac)
                                  * handle->cogging_lut.gain;
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
            /* 低速静摩擦前馈统一由 FF 层库仑处理(fric_comp + pos_cmd_dir方向兜底 + Stribeck),
             * 此处禁用避免双重补偿(实测双份过冲: 匀速从1.8→1.25°/s恶化)。 */
            friction_dir = 0.0f;
        }

        if (friction_dir != 0.0f) {
            if (handle->control_mode == FOC_MODE_POSITION) {
                friction_comp = (friction_dir > 0.0f) ?
                                handle->fric_comp_pos :
                                handle->fric_comp_neg;
            } else {
                friction_comp = (friction_dir > 0.0f) ?
                                FOC_SPEED_STATIC_FRICTION_POS_COMP_A :
                                FOC_SPEED_STATIC_FRICTION_NEG_COMP_A;
            }
            friction_delta = (friction_dir > 0.0f) ? friction_comp : -friction_comp;
            /* Stribeck 平滑: 起动(低速)给满静摩擦, 随速度指数衰减到动摩擦。
             * 恒定 bang-bang 补偿在匀速时持续给满力, 电机冲过头→位置环拉回→
             * 极限环粘滑。平滑后高速衰减, 仅低速补静摩擦, 消除振荡。 */
            {
                float v_mag = fabsf(speed_mech_user);
                float stick = expf(-v_mag / FOC_FRIC_STRIBECK_VS_RADPS);
                float smooth = FOC_FRIC_STRIBECK_KINEMATIC
                               + (1.0f - FOC_FRIC_STRIBECK_KINEMATIC) * stick;
                friction_delta *= smooth;
            }
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

haptic_torque_injection:
        /* ── Phase 3B: Spring-Damper / Detent torque injection ── */
        if (handle->app_mode == APP_MODE_SPRING_DAMPER && handle->motor_identified) {
            float theta_mech_zeroed = FOC_AngleNormalize(
                handle->theta_mech - handle->motor_param.mech_zero_offset);
            float pos_error = FOC_AngleNormalize(
                handle->pos_ref - FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed));
            float spring_torque = handle->spring_K * pos_error
                               - handle->spring_D * speed_feedback;
            spring_torque = FOC_Saturate(spring_torque,
                                         handle->spring_limit_A,
                                        -handle->spring_limit_A);
            iq_ref_mech += spring_torque;
        } else if (handle->app_mode == APP_MODE_DETENT && handle->motor_identified) {
            float theta_mech_zeroed = FOC_AngleNormalize(
                handle->theta_mech - handle->motor_param.mech_zero_offset);
            float pos_rad = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
            float detent_spacing = (2.0f * FOC_PI) / handle->detent_count;
            float nearest = roundf(pos_rad / detent_spacing) * detent_spacing;
            float detent_error = FOC_AngleNormalize(nearest - pos_rad);
            float detent_torque = handle->detent_strength * detent_error;
            /* Width: ramp down outside detent_width */
            {
                float half_w = handle->detent_width_rad * 0.5f;
                float abs_e = fabsf(detent_error);
                if (abs_e > handle->detent_width_rad) {
                    detent_torque = 0.0f;
                } else if (abs_e > half_w) {
                    float ramp = 1.0f - (abs_e - half_w) / half_w;
                    detent_torque *= (ramp > 0.0f) ? ramp : 0.0f;
                }
            }
            detent_torque = FOC_Saturate(detent_torque,
                                         handle->detent_limit_A,
                                        -handle->detent_limit_A);
            /* Detent damping: oppose motion to prevent overshoot oscillation */
            detent_torque -= handle->detent_damping * speed_feedback;
            iq_ref_mech += detent_torque;
        } else if (handle->app_mode == APP_MODE_SCROLL_WHEEL && handle->motor_identified) {
            /* SCROLL_WHEEL: same detent algorithm as DETENT, independent config.
             * Additionally feeds wheel_input for event generation. */
            float theta_mech_zeroed = FOC_AngleNormalize(
                handle->theta_mech - handle->motor_param.mech_zero_offset);
            float pos_rad = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
            float wheel_spacing = (2.0f * FOC_PI) / handle->wheel_count;
            float nearest = roundf(pos_rad / wheel_spacing) * wheel_spacing;
            float wheel_error = FOC_AngleNormalize(nearest - pos_rad);
            float wheel_torque = handle->wheel_strength * wheel_error;
            /* Width: ramp down outside wheel_width */
            {
                float half_w = handle->wheel_width_rad * 0.5f;
                float abs_e = fabsf(wheel_error);
                if (abs_e > handle->wheel_width_rad) {
                    wheel_torque = 0.0f;
                } else if (abs_e > half_w) {
                    float ramp = 1.0f - (abs_e - half_w) / half_w;
                    wheel_torque *= (ramp > 0.0f) ? ramp : 0.0f;
                }
            }
            wheel_torque = FOC_Saturate(wheel_torque,
                                        handle->wheel_limit_A,
                                       -handle->wheel_limit_A);
            wheel_torque -= handle->wheel_damping * speed_feedback;
            iq_ref_mech += wheel_torque;

            /* Feed wheel_input for detent quantization & event generation */
            WheelInput_Update(pos_rad, speed_feedback,
                              (uint8_t)(handle->enable_pwm != 0U),
                              (uint8_t)(1U /* encoder_valid checked on entry */));
        }
        iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);

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

        /* 低速速度观测器 (线性ESO) 更新: 机械帧模型。
         * iq_ref_mech/Iq_ref/Idq.q 均为用户帧电流, 须乘 encoder_dir 转机械帧,
         * 否则 enc_dir=-1 时观测器转矩方向反 (与 Gopinath DOB 门禁同源问题)。
         * 驱动用实测 Idq.q (非指令), 避免"指令自洽"陷阱。 */
        {
            float enc_dir_f_obs = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
            float iq_mech = handle->foc.Idq.q * enc_dir_f_obs;
            FOC_SpeedObserver_Update(&handle->speed_obs,
                                     handle->theta_mech, iq_mech,
                                     1.0f / (float)FOC_SPEED_LOOP_FREQ);
            handle->speed_obs_mech = handle->speed_obs.omega_lpf;
            handle->speed_obs_user = handle->speed_obs.omega_lpf * enc_dir_f_obs;
        }
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

/* ── Haptic mode helper ── */
static uint8_t FOC_App_IsHapticMode(const FOC_AppHandle_t *handle)
{
    return (handle->app_mode == APP_MODE_SPRING_DAMPER ||
            handle->app_mode == APP_MODE_DETENT ||
            handle->app_mode == APP_MODE_SCROLL_WHEEL) ? 1U : 0U;
}

void FOC_App_PositionLoop(FOC_AppHandle_t *handle)
{
    if ((handle == NULL) || handle->stall_open_loop_active) {
        return;
    }

    /* Haptic modes (SPRING_DAMPER, DETENT) bypass normal PositionLoop —
     * they compute torque directly in SpeedLoop, not via position PD. */
    if (FOC_App_IsHapticMode(handle)) {
        handle->speed_ref = 0.0f;
        handle->speed_ref_ramped = 0.0f;
        return;
    }

    /* 非位置模式/非运行时清零低速静摩擦状态（积分+指令方向） */
    if ((handle->state != FOC_STATE_RUNNING) ||
        (handle->control_mode != FOC_MODE_POSITION)) {
        handle->pos_integral = 0.0f;
        handle->pos_cmd_dir = 0.0f;
        handle->pos_cmd_dir_hold = 0U;
    }

    /* 位置环（仅在位置模式且运行状态执行） */
    if (handle->state == FOC_STATE_RUNNING && 
        handle->control_mode == FOC_MODE_POSITION) {
        
        /* 位置误差：pos_ref(用户坐标) - theta_mech_user(用户坐标) */
        float encoder_dir_f = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
        float theta_mech_zeroed = FOC_AngleNormalize(handle->theta_mech - handle->motor_param.mech_zero_offset);
        float theta_mech_user_pos = theta_mech_zeroed * encoder_dir_f;
        /* D项速度源: obs_use_d 时用观测器速度(平滑, 允许更高kd阻尼),
         * 否则差分速度(噪声大, kd受限于0.03)。 */
        float speed_mech_user_pos;
        if ((handle->obs_use_d != 0U) && (handle->speed_obs.valid != 0U)) {
            speed_mech_user_pos = handle->speed_obs.omega_lpf * encoder_dir_f;
        } else {
            speed_mech_user_pos = handle->speed_mech * encoder_dir_f;
        }
        float pos_error = handle->pos_ref - theta_mech_user_pos;
        
        /* 处理角度环绕（最短路径） */
        pos_error = FOC_AngleNormalize(pos_error);

        /* 指令方向锁存(慢摇连续静摩擦补偿方向源, 级联与直连共用):
         * ref 有增量→方向=sign(增量)并重置保持窗口; hold窗口内(ref刚动过,
         * 含PC步进间歇期)方向保持——否则PC每0.2s步进时 ref_delta 间歇为0
         * 会误清方向导致补偿5Hz间歇、长斜坡爬行; hold耗尽后 到位或误差反向→清0。
         * 注意: pos_ref 为 control frame(=用户角×encoder_dir), 方向/误差须转回
         * 用户坐标再判, 否则 encoder_dir=-1 时补偿方向反(正向运动 dir=-1 帮倒忙)。 */
        float enc_dir_f = encoder_dir_f;
        float ref_delta = (handle->pos_ref - handle->pos_ref_prev) * enc_dir_f;
        float pos_err_user = pos_error * enc_dir_f;
        handle->pos_ref_prev = handle->pos_ref;
        if (fabsf(ref_delta) > FOC_FRIC_CMD_DIR_UPDATE_RAD) {
            handle->pos_cmd_dir = (ref_delta > 0.0f) ? 1.0f : -1.0f;
            handle->pos_cmd_dir_hold = FOC_FRIC_CMD_DIR_HOLD_CNT;
        } else if (handle->pos_cmd_dir_hold > 0U) {
            handle->pos_cmd_dir_hold--;
        } else if (fabsf(pos_err_user) < FOC_FRIC_CMD_DIR_CLEAR_RAD) {
            handle->pos_cmd_dir = 0.0f;
        } else if ((pos_err_user * handle->pos_cmd_dir) < 0.0f) {
            handle->pos_cmd_dir = 0.0f;
        }

        /* 位置环PD：位置误差给速度指令，速度反馈提供阻尼 */
        float pos_pd_out = FOC_PositionPD_Update(&handle->pos_pd, pos_error, speed_mech_user_pos);

        /* 位置环直连电流环 (判别实验)：
         * 直连PD输出直接作为力矩指令(A)，跳过速度环PI，速度环只保留
         * FF层(齿槽/摩擦/惯量)与静摩擦补偿。speed_ref 清零避免
         * speed_ref_ramped 斜坡把惯量FF带起来。
         * 叠加低速条件积分(消除静摩擦稳态误差): 仅 |err|<2° 积分, PD饱和
         * 冻结(抗windup), 积分输出限幅±0.10A。 */
        if (handle->pos_direct != 0U) {
            float pd_out = FOC_PositionPD_Update(
                &handle->pos_pd_direct, pos_error, speed_mech_user_pos);
            uint8_t pd_sat = ((pd_out >= handle->pos_pd_direct.output_max) ||
                              (pd_out <= handle->pos_pd_direct.output_min)) ? 1U : 0U;
            if ((pd_sat == 0U) && (fabsf(pos_error) < handle->pos_integral_err_rad)) {
                handle->pos_integral += pos_error * FOC_POS_LOOP_TS;
            }
            float ki_out = handle->pos_direct_ki * handle->pos_integral;
            if (ki_out > FOC_POS_INTEGRAL_LIMIT_A) {
                ki_out = FOC_POS_INTEGRAL_LIMIT_A;
            } else if (ki_out < -FOC_POS_INTEGRAL_LIMIT_A) {
                ki_out = -FOC_POS_INTEGRAL_LIMIT_A;
            }
            handle->pos_direct_iq_cmd = FOC_Saturate(
                pd_out + ki_out,
                handle->pos_pd_direct.output_max,
                handle->pos_pd_direct.output_min);
            handle->speed_ref = 0.0f;
            handle->speed_ref_ramped = 0.0f;
            handle->position_loop_error_diag = pos_error;
            handle->position_loop_pd_out_diag = handle->pos_direct_iq_cmd;
            handle->position_loop_pd_sat_diag = pd_sat;
            handle->traj_active_diag = 0U;
            handle->traj_cmd_diag = handle->pos_direct_iq_cmd;
            return;
        }

        /* V5 巡航速度下限：大误差时维持最低巡航速度，避免末端渐近慢尾
         * 使用运行时配置，若 cruise > speed_limit 则自动夹紧 */
        float cruise_cmd = pos_pd_out;
        uint8_t cruise_active = 0U;
        float effective_cruise = handle->position_cruise_speed_radps;
        if (effective_cruise > handle->position_speed_limit_radps) {
            effective_cruise = handle->position_speed_limit_radps;
        }
        float abs_err = (pos_error >= 0.0f) ? pos_error : -pos_error;
        if (abs_err > FOC_POSITION_CRUISE_HOLD_THRESHOLD_RAD) {
            /* 只在 PD 输出与误差同向且 PD 输出绝对值小于巡航速度时才启用 */
            float abs_pd = (pos_pd_out >= 0.0f) ? pos_pd_out : -pos_pd_out;
            if ((pos_error * pos_pd_out) > 0.0f && abs_pd < effective_cruise) {
                cruise_cmd = (pos_error > 0.0f) ? effective_cruise
                                                 : -effective_cruise;
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

    if ((handle->app_mode == APP_MODE_SCROLL_WHEEL) &&
        !WheelInput_IsSessionActive()) {
        handle->enable_pwm = 0U;
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
        handle->app_mode = APP_MODE_RAW;
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
    handle->speed_obs_mech = 0.0f;
    handle->speed_obs_user = 0.0f;
    FOC_SpeedObserver_Reset(&handle->speed_obs, handle->theta_mech);
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
    } else if (handle->control_mode == FOC_MODE_SPEED) {
        /* RAW SPEED uses the explicit SREF command range. MOTION_CFG remains
         * the conservative trajectory limit for position/joint/gimbal flows. */
        speed_ref = FOC_Saturate(speed_ref,
                                 FOC_SPEED_REF_MAX_RAD_PER_S,
                                 -FOC_SPEED_REF_MAX_RAD_PER_S);
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

    /* Phase 3A: HOLD receives PREF → auto-switch to JOINT_POS
     * to avoid semantic conflict (HOLD = lock current position). */
    if (handle->app_mode == APP_MODE_HOLD) {
        handle->app_mode = APP_MODE_JOINT_POS;
    }

    /* Phase 3: JOINT_POS soft limit clamping */
    if (handle->app_mode == APP_MODE_JOINT_POS && handle->joint_soft_limit_enabled) {
        float pos_deg = pos_ref * 57.29578f;  /* rad → deg for clamping */
        float min_deg = handle->joint_pos_limit_min_rad * 57.29578f;
        float max_deg = handle->joint_pos_limit_max_rad * 57.29578f;
        if (pos_deg < min_deg) pos_deg = min_deg;
        if (pos_deg > max_deg) pos_deg = max_deg;
        pos_ref = pos_deg * 0.0174533f;  /* back to rad */
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
                        handle->position_speed_limit_radps,
                        -handle->position_speed_limit_radps);
}

void FOC_App_SetPosDirectPDGains(FOC_AppHandle_t *handle, float kp, float kd)
{
    if (handle == NULL) {
        return;
    }

    /* 直连PD输出限幅固定为位置模式Iq限幅（力矩域） */
    FOC_PositionPD_Init(&handle->pos_pd_direct,
                        kp,
                        kd,
                        FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A,
                        -FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A);
    handle->pos_direct_iq_cmd = 0.0f;
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

/**
 * @brief 设置底层控制模式并显式切回 RAW 产品模式
 * @param handle FOC应用层句柄指针
 * @param mode 控制模式（力矩/速度/位置）
 *
 * 与 FOC_App_SetControlMode() 的区别：
 * - 本函数同时设置 app_mode = APP_MODE_RAW
 * - 用于用户显式选择底层 RAW 控制模式（如 CMD:MODE,N）
 * - 不应在 APP_MODE 内部切换时调用
 */
void FOC_App_SetRawControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode)
{
    if (handle == NULL) {
        return;
    }

    if (handle->stall_open_loop_active && (mode == FOC_MODE_POSITION)) {
        mode = FOC_MODE_SPEED;
    }

    handle->control_mode = mode;
    handle->app_mode = APP_MODE_RAW;

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

/* ── Phase 3: Application Mode Layer ────────────────────────── */
void FOC_App_SetAppMode(FOC_AppHandle_t *handle, AppMode_t mode)
{
    if (handle == NULL) return;

    handle->pi_speed.integral = 0.0f;
    handle->position_friction_active = 0U;

    switch (mode) {
    case APP_MODE_RAW:
        /* RAW: keep current control_mode, no special behavior */
        break;

    case APP_MODE_JOINT_POS:
        /* JOINT_POS: switch to POSITION, soft limits active */
        handle->control_mode = FOC_MODE_POSITION;
        if (handle->position_ref_user_set == 0U && handle->motor_identified) {
            FOC_App_RefreshEncoderFeedback(handle);
            {
                float theta_mech_zeroed = FOC_AngleNormalize(
                    handle->theta_mech - handle->motor_param.mech_zero_offset);
                handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
            }
        }
        break;

    case APP_MODE_GIMBAL_SPEED:
        /* GIMBAL_SPEED: switch to SPEED, enable SREF ramp */
        handle->control_mode = FOC_MODE_SPEED;
        break;

    case APP_MODE_HOLD:
        /* HOLD: switch to POSITION, lock current position */
        handle->control_mode = FOC_MODE_POSITION;
        if (handle->motor_identified) {
            FOC_App_RefreshEncoderFeedback(handle);
            {
                float theta_mech_zeroed = FOC_AngleNormalize(
                    handle->theta_mech - handle->motor_param.mech_zero_offset);
                handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
                handle->position_ref_user_set = 1U;
            }
        }
        break;

    case APP_MODE_SPRING_DAMPER:
        /* SPRING_DAMPER: always capture current position as equilibrium.
         * Must not inherit a stale PREF target from a previous mode. */
        handle->control_mode = FOC_MODE_POSITION;
        if (handle->motor_identified) {
            FOC_App_RefreshEncoderFeedback(handle);
            {
                float theta_mech_zeroed = FOC_AngleNormalize(
                    handle->theta_mech - handle->motor_param.mech_zero_offset);
                handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
                handle->position_ref_user_set = 1U;
            }
        }
        handle->speed_ref = 0.0f;
        handle->speed_ref_ramped = 0.0f;
        break;

    case APP_MODE_DETENT:
        /* DETENT: always capture the nearest detent on entry.  A previous
         * PREF/JOINT_POS command may leave position_ref_user_set latched,
         * but detent mode must not keep chasing that old position target. */
        handle->control_mode = FOC_MODE_POSITION;
        if (handle->motor_identified) {
            FOC_App_RefreshEncoderFeedback(handle);
            {
                float theta_mech_zeroed = FOC_AngleNormalize(
                    handle->theta_mech - handle->motor_param.mech_zero_offset);
                float pos_rad = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
                /* Snap to nearest detent */
                if (handle->detent_count > 0.5f) {
                    float detent_spacing = (2.0f * FOC_PI) / handle->detent_count;
                    pos_rad = roundf(pos_rad / detent_spacing) * detent_spacing;
                }
                handle->pos_ref = FOC_AngleNormalize(pos_rad);
                handle->position_ref_user_set = 1U;
            }
        }
        break;

    case APP_MODE_SCROLL_WHEEL:
        /* SCROLL_WHEEL: requires identified motor + encoder online.
         * Capture nearest detent as starting position, zero the wheel
         * input state, and turn off current stream. */
        if (!handle->motor_identified) {
            DrvUart_SendTextP0("APP_MODE,FAIL,not_identified\r\n");
            return;
        }
        if (!TLE5012_IsDataValid()) {
            DrvUart_SendTextP0("APP_MODE,FAIL,no_encoder\r\n");
            return;
        }
        /* Disable high-frequency current stream to avoid latency */
        CurStream_SetMode(CUR_STREAM_OFF, 0);
        handle->control_mode = FOC_MODE_POSITION;
        FOC_App_RefreshEncoderFeedback(handle);
        {
            float theta_mech_zeroed = FOC_AngleNormalize(
                handle->theta_mech - handle->motor_param.mech_zero_offset);
            float pos_rad = FOC_App_PositionSensorToControlFrame(handle, theta_mech_zeroed);
            if (handle->wheel_count > 0.5f) {
                float spacing = (2.0f * FOC_PI) / handle->wheel_count;
                pos_rad = roundf(pos_rad / spacing) * spacing;
            }
            handle->pos_ref = FOC_AngleNormalize(pos_rad);
            handle->position_ref_user_set = 1U;
        }
        handle->speed_ref = 0.0f;
        handle->speed_ref_ramped = 0.0f;
        handle->wheel_cfg_active = 1U;
        /* Sync wheel_input config from handle defaults */
        {
            WheelConfig_t wcfg;
            wcfg.count     = handle->wheel_count;
            wcfg.strength  = handle->wheel_strength;
            wcfg.width_rad = handle->wheel_width_rad;
            wcfg.damping   = handle->wheel_damping;
            wcfg.limit_A   = handle->wheel_limit_A;
            WheelInput_ApplyConfig(&wcfg);
        }
        WheelInput_ResetState();
        break;
    }

    /* Only commit app_mode after validation passes (SCROLL_WHEEL may
     * have returned early on prereq failure — see P1-5 fix). */
    handle->app_mode = mode;
}

void FOC_App_SetJointLimits(FOC_AppHandle_t *handle, float min_rad, float max_rad)
{
    if (handle == NULL) return;
    if (min_rad > max_rad) return;
    handle->joint_pos_limit_min_rad = min_rad;
    handle->joint_pos_limit_max_rad = max_rad;
}

void FOC_App_SetGimbalRamp(FOC_AppHandle_t *handle, float accel_radps2)
{
    if (handle == NULL) return;
    if (accel_radps2 < 0.1f) accel_radps2 = 0.1f;
    handle->gimbal_ramp_accel_radps2 = accel_radps2;
}

void FOC_App_SetSpringCfg(FOC_AppHandle_t *handle, float K, float D, float limit)
{
    if (handle == NULL) return;
    if (K < 0.0f) K = 0.0f;
    if (D < 0.0f) D = 0.0f;
    if (limit < 0.01f) limit = 0.01f;
    if (limit > 1.0f) limit = 1.0f;
    handle->spring_K = K;
    handle->spring_D = D;
    handle->spring_limit_A = limit;
}

void FOC_App_SetDetentCfg(FOC_AppHandle_t *handle, float count, float strength, float width, float damping, float limit)
{
    if (handle == NULL) return;
    if (count < 1.0f) count = 1.0f;
    if (strength < 0.0f) strength = 0.0f;
    if (width < 0.01f) width = 0.01f;
    if (damping < 0.0f) damping = 0.0f;
    if (limit < 0.01f) limit = 0.01f;
    if (limit > 1.0f) limit = 1.0f;
    handle->detent_count = count;
    handle->detent_strength = strength;
    handle->detent_width_rad = width;
    handle->detent_damping = damping;
    handle->detent_limit_A = limit;
}

void FOC_App_SetWheelCfg(FOC_AppHandle_t *handle, float count, float strength, float width, float damping, float limit)
{
    WheelConfig_t wcfg;
    if (handle == NULL) return;
    if (count < 1.0f) count = 1.0f;
    if (strength < 0.0f) strength = 0.0f;
    if (width < 0.01f) width = 0.01f;
    if (damping < 0.0f) damping = 0.0f;
    if (limit < 0.01f) limit = 0.01f;
    if (limit > 1.0f) limit = 1.0f;
    handle->wheel_count     = count;
    handle->wheel_strength  = strength;
    handle->wheel_width_rad = width;
    handle->wheel_damping   = damping;
    handle->wheel_limit_A   = limit;
    /* Sync to wheel_input module */
    wcfg.count     = count;
    wcfg.strength  = strength;
    wcfg.width_rad = width;
    wcfg.damping   = damping;
    wcfg.limit_A   = limit;
    WheelInput_ApplyConfig(&wcfg);
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
    {
        /* motor_param.Ke 为机械侧常数(V·s/rad mechanical)，
         * BEMF前馈需电角速度基准: Ke_elec = Ke_mech / Pn */
        float pn_f = (handle->motor_param.Pn > 0U) ? (float)handle->motor_param.Pn : 1.0f;
        float Ke_elec = handle->motor_param.Ke / pn_f;
        FOC_SetBemfParams(&handle->foc, handle->motor_param.Ld, handle->motor_param.Lq, Ke_elec);
    }
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

    /* 低速速度观测器 (线性ESO): 机械帧模型, J/B/Kt 用辨识值 */
    {
        handle->obs_w0 = FOC_OBSERVER_W0_DEFAULT;
        handle->obs_t_gain = FOC_OBSERVER_T_GAIN_DEFAULT;
        handle->obs_use_d = 0U;
        handle->obs_use_speed = 0U;
        handle->fric_vs = FOC_FRIC_STRIBECK_VS_RADPS;
        handle->fric_kin = FOC_FRIC_STRIBECK_KINEMATIC;
        handle->speed_obs_mech = 0.0f;
        handle->speed_obs_user = 0.0f;
        FOC_SpeedObserver_Init(&handle->speed_obs,
                               handle->motor_param.J,
                               handle->motor_param.B,
                               handle->motor_param.Ke,
                               handle->obs_w0,
                               1.0f / (float)FOC_SPEED_LOOP_FREQ);
        handle->speed_obs.t_gain = handle->obs_t_gain;
    }

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

    /* 速度环参数 - 12V 标准基线 (24N22P, 74KV) */
    /* 使用台架实测整定值，不使用Ke/J自动计算(默认Ke/J参数不准确) */
    float Kp_s = 0.25f;  /* Phase 2 定版：0.25 最佳，>=0.30 振荡 */
    float Ki_s = 0.001f; /* Phase 2 gated: 仅速度模式门控积分，位置模式 P-only */
    
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

    /* 重新初始化运行时运动配置默认值（V5不持久化） */
    handle->position_speed_limit_radps  = FOC_MOTION_CFG_SPEED_LIMIT_DEFAULT;
    handle->position_accel_limit_radps2 = FOC_MOTION_CFG_ACCEL_LIMIT_DEFAULT;
    handle->position_cruise_speed_radps = FOC_MOTION_CFG_CRUISE_SPEED_DEFAULT;

    /* 更新位置环PD */
    FOC_PositionPD_Init(&handle->pos_pd,
                        Kp_p,
                        Kd_p,
                        handle->position_speed_limit_radps,
                        -handle->position_speed_limit_radps);

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
    /* If cogging LUT is pending, use combined save that writes both
     * main params and LUT in a single flash erase+write session. */
    if (handle->cogging_lut.pending
        && handle->cogging_lut.valid_size > 0U
        && handle->cogging_lut.valid_size <= FOC_COGGING_LUT_SIZE) {
        {
            ParamStatus_t st = Param_SaveCoggingLUT(handle->cogging_lut.table,
                                       handle->cogging_lut.valid_size,
                                       &handle->motor_param);
            handle->cogging_lut.pending = 0U;
            handle->cogging_lut.save_attempted = (st == PARAM_OK) ? 1U : 2U;
            /* Verify: read back header to confirm persistence */
            if (st == PARAM_OK) {
                const uint32_t *verify_hdr = (const uint32_t *)PARAM_COGGING_FLASH_ADDR;
                if (verify_hdr[0] != 0x434F4747) {
                    handle->cogging_lut.save_attempted = 3U; /* magic mismatch */
                }
            }
        }
    } else {
        Param_Save(&handle->motor_param);
    }
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

/* ── Phase 4: Calibration State Machine ─────────────────────── */

uint8_t FOC_App_CalIsBusy(FOC_AppHandle_t *handle)
{
    if (handle == NULL) return 0U;
    return (handle->cal_state == 1U) ? 1U : 0U;  /* 1=running */
}

uint8_t FOC_App_CalPrecheck(FOC_AppHandle_t *handle)
{
    if (handle == NULL) return 1U;  /* error */

    /* AppFault must be clear */
    if (handle->fault_code != FOC_FAULT_NONE) return 2U;

    /* Vbus must be within thresholds */
    {
        float vbus;
        FOC_App_RefreshTelemetry(handle);
        vbus = handle->Vbus;
        if (vbus < handle->protection.undervoltage_limit_v ||
            vbus > handle->protection.overvoltage_limit_v) return 3U;
    }

    /* Power must be unlocked */
    if (!handle->power_unlocked) return 4U;

    /* DRV communication must be OK */
    {
        extern DRV8350S_Handle_t drv8350s;
        if (drv8350s.runtime.spiError) return 5U;
    }

    /* Encoder must be valid */
    if (!TLE5012_IsDataValid()) return 6U;

    return 0U;  /* OK */
}

/* ── Phase 5A: RAM Fault Black Box ──────────────────────────── */

extern FOC_AppHandle_t g_foc_app;

static BlackBoxSample_t s_bb[BLACKBOX_SAMPLES];
static uint8_t  s_bb_head = 0U;
static uint8_t  s_bb_count = 0U;
static uint8_t  s_bb_frozen = 0U;
static uint8_t  s_bb_freeze_reason = 0U;
static uint32_t s_bb_freeze_time = 0U;
static uint16_t s_bb_decim = 0U;   /* decimation counter for 50Hz from 2kHz */

void BlackBox_Init(void)
{
    s_bb_head = 0U;
    s_bb_count = 0U;
    s_bb_frozen = 0U;
    s_bb_freeze_reason = 0U;
    s_bb_freeze_time = 0U;
    s_bb_decim = 0U;
}

void BlackBox_Sample(void)
{
    BlackBoxSample_t *s;
    extern DRV8350S_Handle_t drv8350s;

    if (s_bb_frozen) return;

    /* Decimate to 50Hz from 2kHz speed loop (40:1) */
    s_bb_decim++;
    if (s_bb_decim < (2000U / BLACKBOX_RATE_HZ)) return;
    s_bb_decim = 0U;

    s = &s_bb[s_bb_head];

    s->timestamp_ms   = HAL_GetTick();
    s->state          = (uint8_t)g_foc_app.state;
    s->control_mode   = (uint8_t)g_foc_app.control_mode;
    s->app_mode       = (uint8_t)g_foc_app.app_mode;
    s->fault_code     = (uint8_t)g_foc_app.fault_code;
    s->warning_flags  = g_foc_app.warning_flags;
    s->Vbus           = g_foc_app.Vbus;
    s->theta_mech     = g_foc_app.theta_mech;
    s->speed_mech     = g_foc_app.speed_mech;
    s->Id             = g_foc_app.foc.Idq.d;
    s->Iq             = g_foc_app.foc.Idq.q;
    s->Id_ref         = g_foc_app.foc.Id_ref;
    s->Iq_ref         = g_foc_app.foc.Iq_ref;
    s->Vd             = g_foc_app.foc.Vdq.d;
    s->Vq             = g_foc_app.foc.Vdq.q;
    s->speed_ref      = g_foc_app.speed_ref;
    s->pos_ref        = g_foc_app.pos_ref;
    s->fault_flags    = drv8350s.runtime.faultFlags;
    s->drv_fault1     = drv8350s.runtime.regFaultStatus1;
    s->drv_vgs2       = drv8350s.runtime.regVgsStatus2;
    s->encoder_crc_count = 0U;
    s->encoder_valid  = TLE5012_IsDataValid() ? 1U : 0U;

    s_bb_head = (uint8_t)((s_bb_head + 1U) % BLACKBOX_SAMPLES);
    if (s_bb_count < BLACKBOX_SAMPLES) s_bb_count++;
}

void BlackBox_Freeze(uint8_t reason)
{
    if (s_bb_frozen) return;
    /* Only freeze if we have enough pre-fault history (>=10 samples = 0.2s) */
    if (s_bb_count < 10U) return;
    s_bb_frozen = 1U;
    s_bb_freeze_reason = reason;
    s_bb_freeze_time = HAL_GetTick();
    /* Capture one final sample at freeze moment */
    BlackBox_Sample();
}

void BlackBox_Clear(void)
{
    s_bb_head = 0U;
    s_bb_count = 0U;
    s_bb_frozen = 0U;
    s_bb_freeze_reason = 0U;
    s_bb_freeze_time = 0U;
}

uint8_t BlackBox_GetCount(void)   { return s_bb_count; }
uint8_t BlackBox_IsFrozen(void)    { return s_bb_frozen; }
uint8_t BlackBox_GetFreezeReason(void) { return s_bb_freeze_reason; }
uint32_t BlackBox_GetFreezeTime(void)  { return s_bb_freeze_time; }

const BlackBoxSample_t *BlackBox_GetSample(uint8_t index)
{
    uint8_t idx;
    if (index >= s_bb_count) return NULL;
    /* index 0 = oldest, index (count-1) = newest */
    idx = (uint8_t)((s_bb_head + BLACKBOX_SAMPLES - s_bb_count + index) % BLACKBOX_SAMPLES);
    return &s_bb[idx];
}
