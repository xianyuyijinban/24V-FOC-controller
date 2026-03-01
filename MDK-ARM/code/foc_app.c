/**
 * @file    foc_app.c
 * @brief   FOC应用层接口实现
 * @note    整合FOC核心、硬件驱动、参数识别和存储
 */

#include "foc_app.h"
#include <string.h>
#include <math.h>

/* 外部变量声明（来自其他模块） */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern volatile uint16_t adc_data[8];
extern DRV8350S_Handle_t drv8350s;

/* DRV8350S 使能引脚（与原理图 DRV_EN 一致） */
#define DRV_EN_GPIO_Port GPIOE
#define DRV_EN_Pin       GPIO_PIN_14

/* 私有函数前向声明 */
static void FOC_App_UpdatePIParams(FOC_AppHandle_t *handle);

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
    
    /* 初始化母线电压 */
    handle->Vbus = 24.0f;
    
    /* 初始化速度环PI控制器 */
    FOC_PI_Init(&handle->pi_speed, 0.1f, 0.01f, 10.0f, -10.0f);
    
    /* 初始化位置环PI控制器 - 位置环输出作为速度给定 */
    /* Kp_pos: 位置误差(rad) -> 速度给定(rad/s), 比如1rad误差对应10rad/s速度 */
    FOC_PI_Init(&handle->pi_pos, 10.0f, 0.0f, 50.0f, -50.0f);  /* 纯P控制或弱I控制 */
    
    /* 初始化Rs在线估计器 */
    MI_RsOnlineEstimator_Init(&handle->rs_est, 0.01f);
    
    /* 尝试加载参数 */
    FOC_App_LoadParam(handle);
    
    /* 初始化FOC核心（使用默认PI参数，识别后会更新） */
    FOC_Init(&handle->foc, 1.0f, 0.1f, 1.0f, 0.1f);
    
    /* 如果参数有效，更新三环PI参数 */
    if (Param_IsValid(&handle->motor_param)) {
        FOC_App_UpdatePIParams(handle);
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
    switch (handle->state) {
        case FOC_STATE_IDLE:
            /* 等待启动参数识别或加载参数 */
            if (handle->enable_identify) {
                FOC_App_StartIdentify(handle);
            }
            break;
            
        case FOC_STATE_PARAM_IDENTIFY:
            /* 参数识别状态机在TIM2中断中运行 */
            if (MI_IsComplete(&handle->mi_handle)) {
                /* 识别完成，保存参数 */
                FOC_App_SaveParam(handle);
                
                /* 更新三环PI参数 */
                FOC_App_UpdatePIParams(handle);
                
                handle->state = FOC_STATE_READY;
                handle->enable_identify = 0;
            } else if (MI_GetError(&handle->mi_handle) != MI_ERR_NONE) {
                /* 识别出错 */
                handle->fault_code = FOC_FAULT_PARAM_INVALID;
                handle->state = FOC_STATE_FAULT;
            }
            break;
            
        case FOC_STATE_READY:
        case FOC_STATE_RUNNING:
            /* 检查故障 */
            if (fabsf(handle->Ia) > FOC_OVERCURRENT_THRESH || 
                fabsf(handle->Ib) > FOC_OVERCURRENT_THRESH ||
                fabsf(handle->Ic) > FOC_OVERCURRENT_THRESH) {
                handle->fault_code = FOC_FAULT_OVERCURRENT;
                FOC_App_Disable(handle);
                handle->state = FOC_STATE_FAULT;
            } else if (handle->Vbus > FOC_OVERVOLTAGE_THRESH) {
                handle->fault_code = FOC_FAULT_OVERVOLTAGE;
                FOC_App_Disable(handle);
                handle->state = FOC_STATE_FAULT;
            } else if (handle->Vbus < FOC_UNDERVOLTAGE_THRESH) {
                handle->fault_code = FOC_FAULT_UNDERVOLTAGE;
                FOC_App_Disable(handle);
                handle->state = FOC_STATE_FAULT;
            }
            break;
            
        case FOC_STATE_FAULT:
            /* 故障状态，等待复位 */
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
    ADC_Sampling_t *adc;
    float angle_deg;
    uint8_t pole_pairs;

    if (handle->state != FOC_STATE_RUNNING && handle->state != FOC_STATE_PARAM_IDENTIFY) {
        return;
    }

    if ((handle->state == FOC_STATE_RUNNING) && !TLE5012_IsDataValid()) {
        handle->fault_code = FOC_FAULT_ENCODER;
        FOC_App_Disable(handle);
        handle->state = FOC_STATE_FAULT;
        return;
    }

    /* 读取编码器角度 */
    angle_deg = TLE5012_GetAngle();
    handle->theta_mech = angle_deg * 3.14159f / 180.0f;  /* 转换为弧度 */
    pole_pairs = (handle->motor_param.Pn > 0U) ? handle->motor_param.Pn : 1U;
    handle->theta_elec = handle->theta_mech * pole_pairs + handle->motor_param.theta_offset;

    /* 使用ADC采样模块计算后的物理量（含零点校准） */
    adc = ADC_Sampling_GetData();
    handle->Ia = adc->currentA;
    handle->Ib = adc->currentB;
    handle->Ic = adc->currentC;
    
    /* 【改进】三相电流不平衡检查 */
    float I_sum = handle->Ia + handle->Ib + handle->Ic;
    if (fabsf(I_sum) > CURRENT_IMBALANCE_THRESH) {
        /* 记录不平衡事件，可用于后续故障诊断 */
        /* 注意：这里不直接触发故障，因为轻微不平衡是正常现象 */
    }
    
    /* 读取母线电压 */
    handle->Vbus = adc->vbus;
    
    /* 更新FOC输入 */
    FOC_UpdateCurrent(&handle->foc, handle->Ia, handle->Ib, handle->Ic);
    FOC_SetAngle(&handle->foc, handle->theta_elec);
    FOC_SetVbus(&handle->foc, handle->Vbus);
    
    /* 执行FOC计算 */
    FOC_Run(&handle->foc);
    
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
    static float theta_prev = 0.0f;
    static uint8_t speed_loop_ready = 0U;

    if (handle->state != FOC_STATE_RUNNING) {
        speed_loop_ready = 0U;
        handle->speed_loop_count++;
        return;
    }

    /* 速度环（仅在运行状态执行） */
    if (!speed_loop_ready) {
        theta_prev = handle->theta_mech;
        handle->speed_mech = 0.0f;
        handle->speed_elec = 0.0f;
        speed_loop_ready = 1U;
        handle->speed_loop_count++;
        return;
    }

    /* 计算转速（简化：微分法） */
    float delta_theta = handle->theta_mech - theta_prev;
    float speed_ref_temp;
    float speed_error;

    /* 处理角度环绕（-π到+π跳变） */
    if (delta_theta > FOC_PI) {
        delta_theta -= 2.0f * FOC_PI;
    } else if (delta_theta < -FOC_PI) {
        delta_theta += 2.0f * FOC_PI;
    }

    handle->speed_mech = delta_theta * FOC_SPEED_LOOP_FREQ;  /* 注意：FOC_SPEED_LOOP_FREQ现在是2000 */
    handle->speed_elec = handle->speed_mech * handle->motor_param.Pn;
    theta_prev = handle->theta_mech;

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
        /* 位置模式：使用位置环输出的速度给定 */
        speed_ref_temp = handle->speed_ref;  /* 位置环更新的速度给定 */
    } else {
        /* 速度模式：直接使用速度给定 */
        speed_ref_temp = handle->speed_ref;
    }

    /* 速度环：速度误差 -> Iq电流给定 */
    speed_error = speed_ref_temp - handle->speed_mech;
    handle->Iq_ref = FOC_PI_Update(&handle->pi_speed, speed_error);

    /* 更新电流参考值 */
    FOC_App_SetCurrentRef(handle, 0, handle->Iq_ref);  /* Id=0控制 */

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
 * - 位置环PI控制（仅在位置模式）
 * - 输出速度给定到handle->speed_ref
 * - 速度环在FOC_App_SpeedLoop中单独执行（2kHz）
 */
void FOC_App_PositionLoop(FOC_AppHandle_t *handle)
{
    /* 位置环（仅在位置模式且运行状态执行） */
    if (handle->state == FOC_STATE_RUNNING && 
        handle->control_mode == FOC_MODE_POSITION) {
        
        /* 位置误差计算 */
        float pos_error = handle->pos_ref - handle->theta_mech;
        
        /* 处理角度环绕（最短路径） */
        while (pos_error > FOC_PI) pos_error -= 2.0f * FOC_PI;
        while (pos_error < -FOC_PI) pos_error += 2.0f * FOC_PI;
        
        /* 位置环PI：输出为速度给定 */
        handle->speed_ref = FOC_PI_Update(&handle->pi_pos, pos_error);
        
        /* 速度限幅（保护） */
        if (handle->speed_ref > 50.0f) handle->speed_ref = 50.0f;
        if (handle->speed_ref < -50.0f) handle->speed_ref = -50.0f;
    }
}

/**
 * @brief 参数识别状态机处理（200Hz）
 * @param handle FOC应用层句柄指针
 * 
 * 【重要】此函数在TIM1_UP_IRQHandler中断中通过分频（100分频）调用
 * 执行频率：20kHz / 100 = 200Hz
 * 
 * 与位置环同频率，确保参数识别的实时性
 */
void FOC_App_ParamIdentifyLoop(FOC_AppHandle_t *handle)
{
    /* 参数识别状态机 */
    if (handle->state == FOC_STATE_PARAM_IDENTIFY) {
        MI_Process(&handle->mi_handle);
    }
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
    if (handle->state == FOC_STATE_READY) {
        /* 先上电驱动芯片，再开栅极，最后启动PWM */
        HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
        HAL_Delay(1);  /* 给DRV_EN上电留出稳定时间 */

        (void)DRV8350S_ClearFaults(&drv8350s);
        if (DRV8350S_EnableGateDrivers(&drv8350s) != 0) {
            handle->fault_code = FOC_FAULT_DRV8350S;
            handle->state = FOC_STATE_FAULT;
            handle->enable_pwm = 0;
            HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
            return;
        }
        
        /* 启动PWM输出 */
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

        handle->enable_pwm = 1;
        handle->state = FOC_STATE_RUNNING;
        
        /* 使能Rs在线估计 */
        MI_RsOnlineEstimator_Enable(&handle->rs_est, 1);
    }
}

/**
 * @brief 禁用FOC控制
 * @param handle FOC应用层句柄指针
 */
void FOC_App_Disable(FOC_AppHandle_t *handle)
{
    handle->enable_pwm = 0;
    
    /* 停止PWM输出 */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* 关闭栅极驱动并关断DRV_EN，确保功率级下电 */
    (void)DRV8350S_DisableGateDrivers(&drv8350s);
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    
    /* 禁用Rs在线估计 */
    MI_RsOnlineEstimator_Enable(&handle->rs_est, 0);
    
    if (handle->state == FOC_STATE_RUNNING) {
        handle->state = FOC_STATE_READY;
    }
}

/**
 * @brief 设置电流参考值
 * @param handle FOC应用层句柄指针
 * @param Id_ref D轴电流参考值
 * @param Iq_ref Q轴电流参考值
 */
void FOC_App_SetCurrentRef(FOC_AppHandle_t *handle, float Id_ref, float Iq_ref)
{
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
    handle->speed_ref = speed_ref;
}

/**
 * @brief 设置位置参考值
 * @param handle FOC应用层句柄指针
 * @param pos_ref 位置参考值（rad）
 */
void FOC_App_SetPositionRef(FOC_AppHandle_t *handle, float pos_ref)
{
    handle->pos_ref = pos_ref;
}

/**
 * @brief 设置控制模式
 * @param handle FOC应用层句柄指针
 * @param mode 控制模式（力矩/速度/位置）
 */
void FOC_App_SetControlMode(FOC_AppHandle_t *handle, FOC_ControlMode_t mode)
{
    handle->control_mode = mode;
    /* 切换模式时清零积分，防止跳变 */
    handle->pi_pos.integral = 0.0f;
    handle->pi_speed.integral = 0.0f;
}

/**
 * @brief 根据电机参数计算三环PI参数
 * @param handle FOC应用层句柄指针
 * 
 * 三环带宽分配原则（从外到内，带宽依次增加5-10倍）：
 * - 位置环带宽: 10-20 Hz (机械响应最慢)
 * - 速度环带宽: 100-200 Hz (比位置环快10倍)
 * - 电流环带宽: 1000-2000 Hz (比速度环快10倍)
 * 
 * 计算公式（典型工程整定）：
 * - 电流环 Kp = Ld * 2π * fc, Ki = Rs * 2π * fc * T
 * - 速度环 Kp = J * 2π * fs / Kt, Ki = Kp * Rs / Ld (或根据机械时间常数)
 * - 位置环 Kp = 2π * fp (纯P控制) 或 Kp = 2π * fp, Ki = Kp / τ
 */
static void FOC_App_UpdatePIParams(FOC_AppHandle_t *handle)
{
    MotorParam_t *mp = &handle->motor_param;
    
    /* 电流环参数计算 - 带宽 2000Hz */
    float current_bw = 2000.0f;  /* 电流环带宽 Hz */
    float T_sample = 0.00005f;   /* 电流环采样周期 50us */
    float Kp_i = mp->Ld * 2.0f * FOC_PI * current_bw;
    float Ki_i = mp->Rs * 2.0f * FOC_PI * current_bw * T_sample;
    
    /* 限制电流环PI参数范围，防止极端值 */
    if (Kp_i < 0.1f) Kp_i = 0.1f;
    if (Kp_i > 100.0f) Kp_i = 100.0f;
    if (Ki_i < 0.001f) Ki_i = 0.001f;
    if (Ki_i > 10.0f) Ki_i = 10.0f;
    
    /* 更新电流环PI */
    FOC_Init(&handle->foc, Kp_i, Ki_i, Kp_i, Ki_i);
    
    /* 速度环参数计算 - 带宽 100Hz (电流环的1/20) */
    float speed_bw = 100.0f;     /* 速度环带宽 Hz */
    float T_speed = 1.0f / (float)FOC_SPEED_LOOP_FREQ;  /* 速度环采样周期 */
    
    /* 速度环PI基于转动惯量和转矩常数
     * 简化的工程整定: Kp = J * ws^2 / Kt, Ki = ws / 10
     * 其中 ws = 2π * speed_bw
     */
    float ws = 2.0f * FOC_PI * speed_bw;
    float Kt = 1.5f * mp->Pn * mp->Ke;  /* 转矩常数 Nm/A (假设使用Iq控制) */
    
    float Kp_s, Ki_s;
    if (Kt > 0.001f && mp->J > 0.0f) {
        /* 有有效参数时使用计算值 */
        Kp_s = mp->J * ws * ws / Kt * 0.5f;  /* 0.5为阻尼系数，可根据需要调整 */
        Ki_s = Kp_s * ws * 0.1f * T_speed;   /* Ki = Kp * ws/10 * T */
    } else {
        /* 使用默认值 */
        Kp_s = 0.1f;
        Ki_s = 0.01f;
    }
    
    /* 限制速度环PI参数 */
    if (Kp_s < 0.01f) Kp_s = 0.01f;
    if (Kp_s > 50.0f) Kp_s = 50.0f;
    if (Ki_s < 0.001f) Ki_s = 0.001f;
    if (Ki_s > 5.0f) Ki_s = 5.0f;
    
    /* 更新速度环PI */
    FOC_PI_Init(&handle->pi_speed, Kp_s, Ki_s, 50.0f, -50.0f);
    
    /* 位置环参数计算 - 带宽 10Hz (速度环的1/10)
     * 位置环PI整定 (纯P控制或弱I控制)
     * Kp_pos: 位置误差(rad) -> 速度给定(rad/s)
     * 对于10Hz带宽，希望位置误差0.1rad时产生约6rad/s速度 (约1转/秒)
     * 因此 Kp_pos ≈ 6 / 0.1 = 60，但通常保守设置，设为20-50
     * 
     * 或者使用: Kp_pos = 2π * fp * 安全系数
     */
    float Kp_p = 20.0f;  /* 位置环Kp: 1rad误差 -> 20rad/s速度给定 */
    float Ki_p = 0.0f;   /* 位置环通常用纯P控制，或用很小的Ki */
    
    /* 也可以根据速度环参数来整定位置环
     * 位置环带宽应低于速度环，通常 wp = ws / 10 ~ ws / 20
     */
    if (Kp_s > 0.0f) {
        /* 基于速度环Kp来估算位置环Kp
         * 经验: Kp_pos = sqrt(Kp_speed) * 5 到 10
         */
        Kp_p = sqrtf(Kp_s) * 8.0f;
        if (Kp_p < 10.0f) Kp_p = 10.0f;
        if (Kp_p > 100.0f) Kp_p = 100.0f;
    }
    
    /* 更新位置环PI */
    FOC_PI_Init(&handle->pi_pos, Kp_p, Ki_p, 50.0f, -50.0f);
    
    /* 可选: 打印调试信息 */
    /*
    printf("PI Params Updated:\r\n");
    printf("  Current: Kp=%.4f, Ki=%.4f\r\n", Kp_i, Ki_i);
    printf("  Speed:   Kp=%.4f, Ki=%.4f\r\n", Kp_s, Ki_s);
    printf("  Position:Kp=%.4f, Ki=%.4f\r\n", Kp_p, Ki_p);
    */
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
}

/**
 * @brief 保存参数到Flash
 * @param handle FOC应用层句柄指针
 */
void FOC_App_SaveParam(FOC_AppHandle_t *handle)
{
    Param_Save(&handle->motor_param);
}

/**
 * @brief 启动参数识别
 * @param handle FOC应用层句柄指针
 */
void FOC_App_StartIdentify(FOC_AppHandle_t *handle)
{
    if (handle->state == FOC_STATE_IDLE || handle->state == FOC_STATE_READY) {
        /* 初始化参数识别 */
        MI_Init(&handle->mi_handle, &handle->motor_param, &handle->foc);
        MI_StartIdentify(&handle->mi_handle);
        
        handle->state = FOC_STATE_PARAM_IDENTIFY;
        handle->enable_identify = 1;
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
