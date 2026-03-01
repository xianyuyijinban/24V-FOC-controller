# 24V FOC Controller 关键问题修复实施计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 修复架构审查中发现的关键问题，使FOC控制系统完整运行，包括TIM2中断实现、FOC应用层集成、参数识别完善和UART上传功能。

**Architecture:** 基于现有分层架构，补充缺失的中断处理和模块集成，保持与Project_Architecture.md定义的一致性。

**Tech Stack:** STM32H743, HAL库, STM32CubeMX, C语言

---

## 问题陈述

### 关键问题清单

| 优先级 | 问题 | 影响 | 状态 |
|--------|------|------|------|
| P0 | TIM2中断未实现 | 速度环和参数识别无法运行 | 🔴 阻塞 |
| P0 | FOC应用层未初始化 | 整个FOC控制逻辑未运行 | 🔴 阻塞 |
| P0 | g_foc_app全局变量未定义 | 编译/链接错误 | 🔴 阻塞 |
| P1 | 参数识别不完整 | Pn/Ke/J返回固定值 | 🟡 重要 |
| P1 | UART上传未集成 | 无法监控运行状态 | 🟡 重要 |
| P2 | Clark变换公式需验证 | 可能影响控制精度 | 🟢 次要 |

### 根本原因分析

1. **TIM2中断缺失**: 架构文档定义了1kHz速度环，但stm32h7xx_it.c中未实现TIM2_IRQHandler
2. **FOC应用层未集成**: main.c中虽然包含了head.h，但未调用FOC_App_Init()和FOC_App_MainLoop()
3. **参数识别简化**: motor_identify.c中的Pn/Ke/J识别函数返回硬编码值，未实现实际测量逻辑
4. **UART模块孤立**: uart_upload.c/h已实现，但main.c中未初始化和调用

---

## 实施计划

### Task 1: 定义g_foc_app全局变量

**Files:**
- Modify: `MDK-ARM/code/head.h` - 添加extern声明
- Modify: `Core/Src/stm32h7xx_it.c` - 定义全局变量

**Step 1: 在head.h中声明外部变量**

head.h已包含声明：
```c
extern FOC_AppHandle_t g_foc_app;
```

**Step 2: 在stm32h7xx_it.c中定义变量**

在`/* USER CODE BEGIN PV */`区域添加：
```c
/* USER CODE BEGIN PV */
/* DRV8350S handle */
DRV8350S_Handle_t drv8350s;

/* FOC应用层句柄 */
FOC_AppHandle_t g_foc_app;

/* UART DMA buffers */
volatile uint16_t urT_data[8];
volatile uint16_t urR_data[8];
/* USER CODE END PV */
```

**Step 3: 验证编译**

运行编译命令检查是否有链接错误。

**Step 4: Commit**

```bash
git add Core/Src/stm32h7xx_it.c
git commit -m "fix: add g_foc_app global variable definition"
```

---

### Task 2: 实现TIM2中断处理

**Files:**
- Modify: `Core/Src/stm32h7xx_it.c` - 添加TIM2_IRQHandler
- Modify: `Core/Src/main.c` - 启动TIM2中断

**Step 1: 在stm32h7xx_it.c中添加TIM2中断处理**

在文件末尾`/* USER CODE BEGIN 1 */`区域添加：
```c
/* USER CODE BEGIN 1 */

/**
  * @brief This function handles TIM2 global interrupt.
  * @note  1kHz - 速度环和参数识别状态机
  */
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */

    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */
    
    /* 调用FOC应用层TIM2中断处理 - 速度环和参数识别 */
    FOC_App_TIM2_IRQHandler(&g_foc_app);
    
    /* USER CODE END TIM2_IRQn 1 */
}

/* SPI DMA Complete Callback */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // ... 现有代码
}
```

**Step 2: 在main.c中启动TIM2**

在`/* USER CODE BEGIN 2 */`区域，在TIM1启动之后添加：
```c
  /* USER CODE BEGIN 2 */
  TLE5012_Init();
  
  /* 初始化FOC应用层 */
  FOC_App_Init(&g_foc_app);
  
  /* 初始化UART上传模块 */
  DrvUart_Init(&huart1, &drv8350s);
  
  HAL_UART_Transmit_DMA(&huart1,(uint8_t*)urT_data,8);
  HAL_UART_Receive_DMA(&huart1,(uint8_t*)urR_data,8);
  
  /* ADC校准 - STM32H7必须在使用前进行校准 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  
  /* 初始化ADC采样模块 */
  ADC_Sampling_Init(&hadc1);
  
  /* 启动ADC DMA采样 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, 4);
  
  if (DRV8350S_Init(&drv8350s, &hspi1, &htim1, 
                      GPIOA, 
                      GPIO_PIN_4) != 0) {
        Error_Handler();
  }
                          
  DRV8350S_Config_t config;
  DRV8350S_SetDefaultConfig(&config);

  /* 根据你的硬件调整 */
  config.pwmMode = DRV8350S_PWM_MODE_6X;           /* 6x PWM 模式 */
  config.idriveP_hs = DRV8350S_IDRIVE_1000MA;      /* 1A source */
  config.idriveN_hs = DRV8350S_IDRIVE_1000MA;      /* 1A sink */
  config.tdrive = DRV8350S_TDRIVE_4000NS;          /* 4us */
  config.ocpMode = DRV8350S_OCP_MODE_RETRY;        /* retry mode */

  if (DRV8350S_Configure(&drv8350s, &config) != 0) {
      Error_Handler();
  }

  DRV8350S_EnableGateDrivers(&drv8350s);

  /* 启动TIM1 - 20kHz电流环 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  
  /* 启动TIM2 - 1kHz速度环 */
  HAL_TIM_Base_Start_IT(&htim2);
  
  /* USER CODE END 2 */
```

**Step 3: 更新main循环**

修改while循环：
```c
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* FOC应用层主循环 - 状态机和故障处理 */
    FOC_App_MainLoop(&g_foc_app);
    
    /* UART数据上传处理 */
    DrvUart_Process();
    
    /* 故障处理 */
    if (drv8350s.runtime.isFaultActive) {
        /* 故障时停止PWM */
        FOC_App_Disable(&g_foc_app);
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```

**Step 4: 验证编译**

确保TIM2相关的HAL函数可以编译。

**Step 5: Commit**

```bash
git add Core/Src/stm32h7xx_it.c Core/Src/main.c
git commit -m "feat: implement TIM2 interrupt and integrate FOC app layer"
```

---

### Task 3: 完善参数识别功能

**Files:**
- Modify: `MDK-ARM/code/motor_identify.c` - 完善识别算法

**Step 1: 实现极对数识别 (MI_IdentifyPn)**

替换现有简化实现：
```c
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
MI_ErrorCode_t MI_IdentifyPn(MI_Handle_t *handle)
{
    static uint8_t state = 0;
    static float theta_mech_start = 0;
    static float theta_elec_start = 0;
    static uint32_t elec_cycles = 0;
    static float theta_mech_last = 0;
    
    float elapsed = MI_GetElapsedTime(handle);
    
    switch (state) {
        case 0: /* 初始化 */
            theta_mech_start = handle->foc->theta_elec / (2.0f * FOC_PI); /* 临时假设Pn=1 */
            theta_elec_start = 0;
            elec_cycles = 0;
            theta_mech_last = theta_mech_start;
            state = 1;
            return MI_ERR_NONE;
            
        case 1: /* 施加旋转磁场 */
            if (elapsed < MI_PN_TEST_DURATION) {
                /* 缓慢旋转电角度 */
                float omega_elec = 2.0f * FOC_PI * 2.0f; /* 2Hz电频率 */
                float theta_elec_cmd = omega_elec * (elapsed / 1000.0f);
                
                /* 使用Iq开环拖动 */
                handle->foc->Idq.d = 0;
                handle->foc->Idq.q = MI_PN_TEST_CURRENT;
                handle->foc->theta_elec = theta_elec_cmd;
                handle->foc->sin_theta = sinf(theta_elec_cmd);
                handle->foc->cos_theta = cosf(theta_elec_cmd);
                
                /* 统计电角度周期 */
                float theta_mech_now = TLE5012_GetAngle() * FOC_PI / 180.0f;
                float delta_mech = theta_mech_now - theta_mech_last;
                
                /* 处理角度环绕 */
                if (delta_mech > FOC_PI) delta_mech -= 2.0f * FOC_PI;
                if (delta_mech < -FOC_PI) delta_mech += 2.0f * FOC_PI;
                
                /* 检测电角度周期 */
                float theta_elec_now = theta_elec_cmd;
                static float theta_elec_last = 0;
                float delta_elec = theta_elec_now - theta_elec_last;
                if (delta_elec < 0) {
                    elec_cycles++;
                }
                theta_elec_last = theta_elec_now;
                theta_mech_last = theta_mech_now;
                
                return MI_ERR_NONE;
            } else {
                state = 2;
                return MI_ERR_NONE;
            }
            
        case 2: /* 计算极对数 */
        {
            float theta_mech_end = TLE5012_GetAngle() * FOC_PI / 180.0f;
            float delta_mech = theta_mech_end - theta_mech_start;
            
            /* 处理角度环绕 */
            if (delta_mech > FOC_PI) delta_mech -= 2.0f * FOC_PI;
            if (delta_mech < -FOC_PI) delta_mech += 2.0f * FOC_PI;
            
            /* 计算极对数 */
            if (fabsf(delta_mech) > 0.1f) { /* 至少转0.1弧度 */
                float pn_calc = (float)elec_cycles * 2.0f * FOC_PI / fabsf(delta_mech);
                /* 四舍五入到最接近的整数 */
                handle->param->Pn = (uint8_t)(pn_calc + 0.5f);
                
                /* 验证合理性 */
                if (handle->param->Pn >= 1 && handle->param->Pn <= 50) {
                    state = 0; /* 重置状态 */
                    return MI_ERR_NONE;
                }
            }
            state = 0;
            return MI_ERR_PN_NOT_CONVERGED;
        }
    }
    
    return MI_ERR_PN_NOT_CONVERGED;
}
```

**Step 2: 实现Ke识别 (MI_IdentifyKe)**

```c
/**
 * @brief 反电动势常数Ke识别
 * @param handle 识别句柄指针
 * @return 错误代码
 * 
 * 原理：
 * 1. 开环拖动到目标转速
 * 2. 切换到电流环(Id=0, Iq=维持电流)
 * 3. 测量d轴电压Vd ≈ -ωe*Ke
 * 4. Ke = |Vd| / ωe
 */
MI_ErrorCode_t MI_IdentifyKe(MI_Handle_t *handle)
{
    static uint8_t state = 0;
    static float omega_e_target = 0;
    
    float elapsed = MI_GetElapsedTime(handle);
    
    switch (state) {
        case 0: /* 加速阶段 */
            if (elapsed < MI_KE_RAMP_TIME) {
                /* 斜坡加速 */
                float ramp_ratio = elapsed / MI_KE_RAMP_TIME;
                omega_e_target = MI_KE_TEST_SPEED_RPM * 2.0f * FOC_PI / 60.0f * handle->param->Pn;
                float omega_e_cmd = omega_e_target * ramp_ratio;
                
                /* 开环拖动 */
                handle->foc->Idq.q = 0.5f; /* 拖动电流 */
                handle->speed_elec = omega_e_cmd; /* 用于电角度积分 */
                
                return MI_ERR_NONE;
            } else {
                state = 1;
                MI_ResetStateData(handle); /* 重置计时 */
                return MI_ERR_NONE;
            }
            
        case 1: /* 测量阶段 */
            if (elapsed < MI_KE_MEASURE_TIME) {
                /* 使用电流环维持转速，Id=0控制 */
                handle->foc->Id_ref = 0;
                handle->foc->Iq_ref = 0.3f; /* 维持电流 */
                
                /* 采样d轴电压和转速 */
                if (fabsf(handle->speed_elec) > 10.0f) { /* 转速足够 */
                    handle->sum_v += handle->foc->Vdq.d;
                    handle->sum_i += handle->speed_elec;
                    handle->sample_count++;
                }
                
                return MI_ERR_NONE;
            } else {
                state = 2;
                return MI_ERR_NONE;
            }
            
        case 2: /* 计算Ke */
            if (handle->sample_count > 10) {
                float Vd_avg = handle->sum_v / handle->sample_count;
                float omega_avg = handle->sum_i / handle->sample_count;
                
                /* Ke = |Vd| / ωe (稳态时Vd = -ωe*Ke) */
                handle->param->Ke = fabsf(Vd_avg) / fabsf(omega_avg);
                
                /* 验证合理性 */
                if (handle->param->Ke > 0.001f && handle->param->Ke < 1.0f) {
                    state = 0;
                    return MI_ERR_NONE;
                }
            }
            state = 0;
            return MI_ERR_KE_NOT_CONVERGED;
    }
    
    return MI_ERR_KE_NOT_CONVERGED;
}
```

**Step 3: Commit**

```bash
git add MDK-ARM/code/motor_identify.c
git commit -m "feat: implement Pn and Ke identification algorithms"
```

---

### Task 4: 验证Clark变换公式

**Files:**
- Verify: `MDK-ARM/code/foc_core.c` - Clark变换

**Step 1: 数学验证**

当前实现：
```c
alphabeta->alpha = abc->a;
alphabeta->beta = (abc->a + 2.0f * abc->b) / FOC_SQRT3;
```

标准Clark变换（等幅值）：
```
alpha = a
beta = (b - c) / sqrt(3)
```

利用 a + b + c = 0，可得 c = -a - b，代入：
```
beta = (b - (-a - b)) / sqrt(3)
     = (b + a + b) / sqrt(3)
     = (a + 2b) / sqrt(3)
```

**结论**: 当前实现数学正确，无需修改。

**Step 2: 添加注释说明**

在foc_core.c中添加数学推导注释：
```c
/**
 * @brief Clark变换 (三相静止坐标系 → 两相静止坐标系)
 * @note 使用等幅值变换
 * 
 * 标准公式：
 *   alpha = a
 *   beta  = (b - c) / √3
 * 
 * 利用 a + b + c = 0，可得 c = -a - b，代入得：
 *   beta = (b - (-a - b)) / √3 = (a + 2b) / √3
 */
```

**Step 3: Commit**

```bash
git add MDK-ARM/code/foc_core.c
git commit -m "docs: add Clark transform mathematical derivation comment"
```

---

### Task 5: 集成UART上传模块

**Files:**
- Modify: `MDK-ARM/code/uart_upload.c` - 添加FOC数据

**Step 1: 扩展数据包结构**

在uart_upload.h中添加FOC数据字段：
```c
/* 上传数据结构 -------------------------------------------------------------*/
typedef struct {
    uint32_t timestamp;         /* 时间戳 (ms) */
    
    /* TLE5012 编码器数据 */
    float    angle;             /* 角度值 (0.0 ~ 360.0 度) */
    uint16_t rawAngle;          /* 原始角度数据 */
    uint8_t  crcError;          /* CRC 错误标志 */
    
    /* DRV8350S 驱动器数据 */
    uint16_t faultStatus1;      /* FAULT_STATUS_1 寄存器 */
    uint16_t vgsStatus2;        /* VGS_STATUS_2 寄存器 */
    uint16_t driverCtrl;        /* DRIVER_CTRL 寄存器 */
    uint16_t ocpCtrl;           /* OCP_CTRL 寄存器 */
    uint32_t faultFlags;        /* 解析后的故障标志 */
    uint8_t  isFaultActive;     /* 是否有故障 */
    
    /* FOC控制数据 - 新增 */
    float    Id;                /* D轴电流 */
    float    Iq;                /* Q轴电流 */
    float    Vd;                /* D轴电压 */
    float    Vq;                /* Q轴电压 */
    float    speed;             /* 转速 (rad/s) */
    float    Id_ref;            /* D轴电流参考 */
    float    Iq_ref;            /* Q轴电流参考 */
    uint8_t  focState;          /* FOC状态 */
    
    uint8_t  packetType;        /* 数据包类型 */
} DrvUart_DataPacket_t;
```

**Step 2: 修改数据收集函数**

在uart_upload.c中添加FOC数据收集：
```c
/* 外部声明 - FOC应用层数据 */
extern FOC_AppHandle_t g_foc_app;

/**
 * @brief 收集 DRV8350S、TLE5012 和 FOC 数据
 */
static void DrvUart_CollectData(DrvUart_DataPacket_t* packet, uint8_t type)
{
    if (packet == NULL || s_drvHandle == NULL) {
        return;
    }
    
    packet->timestamp = HAL_GetTick();
    packet->packetType = type;
    
    /* 从 TLE5012 获取编码器数据 */
    packet->angle = tle5012_sensor.angle;
    packet->rawAngle = tle5012_sensor.raw_angle;
    packet->crcError = tle5012_sensor.crc_error;
    
    /* 从 DRV8350S 句柄获取数据 */
    packet->faultStatus1 = s_drvHandle->runtime.regFaultStatus1;
    packet->vgsStatus2 = s_drvHandle->runtime.regVgsStatus2;
    packet->driverCtrl = s_drvHandle->runtime.regDriverCtrl;
    packet->ocpCtrl = s_drvHandle->runtime.regOcpCtrl;
    packet->faultFlags = s_drvHandle->runtime.faultFlags;
    packet->isFaultActive = s_drvHandle->runtime.isFaultActive;
    
    /* 从 FOC 应用层获取数据 - 新增 */
    packet->Id = g_foc_app.foc.Idq.d;
    packet->Iq = g_foc_app.foc.Idq.q;
    packet->Vd = g_foc_app.foc.Vdq.d;
    packet->Vq = g_foc_app.foc.Vdq.q;
    packet->speed = g_foc_app.speed_mech;
    packet->Id_ref = g_foc_app.Id_ref;
    packet->Iq_ref = g_foc_app.Iq_ref;
    packet->focState = (uint8_t)g_foc_app.state;
}
```

**Step 3: 更新格式化函数**

在DrvUart_FormatNormal中添加FOC数据显示：
```c
static int16_t DrvUart_FormatNormal(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    // ... 现有代码 ...
    
    /* FOC 控制数据 - 新增 */
    len += snprintf((char*)buf + len, bufSize - len,
        "[FOC Control]\r\n");
    len += snprintf((char*)buf + len, bufSize - len,
        "  State:  %s\r\n", FOC_App_GetStateString((FOC_AppState_t)packet->focState));
    len += snprintf((char*)buf + len, bufSize - len,
        "  Id:     %7.3f A (ref: %7.3f)\r\n", packet->Id, packet->Id_ref);
    len += snprintf((char*)buf + len, bufSize - len,
        "  Iq:     %7.3f A (ref: %7.3f)\r\n", packet->Iq, packet->Iq_ref);
    len += snprintf((char*)buf + len, bufSize - len,
        "  Vd:     %7.3f V\r\n", packet->Vd);
    len += snprintf((char*)buf + len, bufSize - len,
        "  Vq:     %7.3f V\r\n", packet->Vq);
    len += snprintf((char*)buf + len, bufSize - len,
        "  Speed:  %7.2f rad/s\r\n\r\n", packet->speed);
    
    // ... 现有代码 ...
}
```

**Step 4: Commit**

```bash
git add MDK-ARM/code/uart_upload.c MDK-ARM/code/uart_upload.h
git commit -m "feat: add FOC data to UART upload module"
```

---

## 测试程序

### 单元测试

#### Test 1: 全局变量定义测试
```c
/* test_globals.c */
#include "head.h"
#include "foc_app.h"

void test_global_variables(void)
{
    /* 测试g_foc_app是否可以访问 */
    FOC_App_Init(&g_foc_app);
    
    /* 验证初始状态 */
    assert(g_foc_app.state == FOC_STATE_INIT);
    assert(g_foc_app.fault_code == FOC_FAULT_NONE);
    
    printf("✓ Global variables test passed\n");
}
```

#### Test 2: TIM2中断测试
```c
/* test_tim2.c */
void test_tim2_interrupt(void)
{
    /* 启动TIM2 */
    HAL_TIM_Base_Start_IT(&htim2);
    
    /* 等待几个周期 */
    uint32_t count_before = g_foc_app.speed_loop_count;
    HAL_Delay(10); /* 应该触发约10次TIM2中断 */
    uint32_t count_after = g_foc_app.speed_loop_count;
    
    assert(count_after > count_before);
    printf("✓ TIM2 interrupt test passed (count: %lu -> %lu)\n", 
           count_before, count_after);
}
```

#### Test 3: 参数识别测试
```c
/* test_identify.c */
void test_param_identify(void)
{
    /* 启动参数识别 */
    FOC_App_StartIdentify(&g_foc_app);
    
    /* 模拟运行一段时间 */
    for (int i = 0; i < 5000; i++) {
        FOC_App_TIM2_IRQHandler(&g_foc_app);
        HAL_Delay(1);
    }
    
    /* 检查是否完成或出错 */
    if (FOC_App_IsIdentifyComplete(&g_foc_app)) {
        printf("✓ Parameter identification completed\n");
        printf("  Rs: %.3f Ohm\n", g_foc_app.motor_param.Rs);
        printf("  Ld: %.6f H\n", g_foc_app.motor_param.Ld);
        printf("  Pn: %d\n", g_foc_app.motor_param.Pn);
        printf("  Ke: %.4f V/(rad/s)\n", g_foc_app.motor_param.Ke);
    } else {
        printf("⚠ Identification did not complete (may need real motor)\n");
    }
}
```

### 集成测试

#### Test 4: FOC控制循环测试
```c
/* test_foc_loop.c */
void test_foc_control_loop(void)
{
    /* 确保参数有效 */
    if (!Param_IsValid(&g_foc_app.motor_param)) {
        printf("⚠ Invalid parameters, using defaults\n");
        Param_SetDefault(&g_foc_app.motor_param);
        g_foc_app.motor_param.valid_flag = 0xFFFFFFFF;
    }
    
    /* 使能FOC */
    FOC_App_Enable(&g_foc_app);
    
    /* 设置速度参考 */
    FOC_App_SetSpeedRef(&g_foc_app, 10.0f); /* 10 rad/s */
    
    /* 运行一段时间 */
    HAL_Delay(1000);
    
    /* 检查状态 */
    assert(FOC_App_GetState(&g_foc_app) == FOC_STATE_RUNNING);
    
    /* 获取调试信息 */
    float Id, Iq, Vd, Vq, theta, speed, Rs_est;
    FOC_App_GetDebugInfo(&g_foc_app, &Id, &Iq, &Vd, &Vq, &theta, &speed, &Rs_est);
    
    printf("✓ FOC control loop test passed\n");
    printf("  Id: %.3f A, Iq: %.3f A\n", Id, Iq);
    printf("  Speed: %.2f rad/s\n", speed);
    
    /* 禁用 */
    FOC_App_Disable(&g_foc_app);
}
```

#### Test 5: UART上传测试
```c
/* test_uart.c */
void test_uart_upload(void)
{
    /* 初始化UART上传 */
    DrvUart_Init(&huart1, &drv8350s);
    
    /* 强制上传一次 */
    DrvUart_UploadImmediate();
    
    printf("✓ UART upload test completed (check serial output)\n");
}
```

---

## 成功标准

### 功能标准

| 检查项 | 成功标准 | 验证方法 |
|--------|---------|---------|
| 编译通过 | 无错误、无警告 | 编译命令 |
| 全局变量 | g_foc_app可访问 | 单元测试 |
| TIM2中断 | 1kHz频率触发 | 示波器/计数器 |
| 速度环 | 转速计算正确 | 调试输出 |
| 参数识别 | 识别完成或合理错误 | 观察状态 |
| FOC运行 | Id/Iq跟随参考值 | 调试输出 |
| UART上传 | 数据正确输出 | 串口助手 |

### 性能标准

| 指标 | 目标值 | 测试条件 |
|------|--------|---------|
| TIM1执行时间 | < 50μs | 20kHz周期内完成 |
| TIM2执行时间 | < 1ms | 1kHz周期内完成 |
| 电流环带宽 | > 1kHz | 阶跃响应测试 |
| 速度环带宽 | > 100Hz | 阶跃响应测试 |
| UART数据率 | 10Hz | 100ms间隔 |

### 稳定性标准

- 连续运行1小时无故障
- 温度变化时参数保持稳定
- 负载突变时系统稳定

---

## 风险评估与缓解

| 风险 | 可能性 | 影响 | 缓解措施 |
|------|--------|------|---------|
| TIM2中断与TIM1冲突 | 中 | 高 | 使用HAL中断优先级管理 |
| 参数识别失败 | 高 | 中 | 提供手动参数输入备选 |
| 电机失控 | 低 | 高 | 完善故障保护和急停逻辑 |
| Flash写入失败 | 低 | 中 | 添加写入验证和重试 |

---

## 实施时间表

| 任务 | 预估时间 | 依赖 |
|------|---------|------|
| Task 1: 全局变量 | 15分钟 | 无 |
| Task 2: TIM2中断 | 30分钟 | Task 1 |
| Task 3: 参数识别 | 60分钟 | Task 2 |
| Task 4: Clark验证 | 15分钟 | 无 |
| Task 5: UART集成 | 30分钟 | Task 1 |
| 测试验证 | 60分钟 | 全部 |
| **总计** | **约3.5小时** | - |

---

## 后续优化建议

1. **添加CAN通信**: 支持工业标准通信协议
2. **实现位置环**: 添加位置控制模式
3. **优化参数识别**: 使用更精确的算法
4. **添加日志系统**: 记录运行历史和故障信息
5. **实现OTA升级**: 支持固件远程更新
