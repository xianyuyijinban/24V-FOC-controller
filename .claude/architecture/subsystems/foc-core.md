# FOC 核心

## 职责

磁场定向控制（FOC）核心算法：Clarke/Park 坐标变换、SVPWM 生成、电流环 PI 控制、BEMF 解耦前馈。

## 内部结构

```mermaid
graph TB
    subgraph FOC核心
        CLARKE[Clarke变换<br/>ABC→αβ]
        PARK[Park变换<br/>αβ→dq]
        INV_PARK[逆Park变换<br/>dq→αβ]
        INV_CLARKE[逆Clarke变换<br/>αβ→ABC]
        SVPWM_DQ[SVPWM生成<br/>从αβ]
        SVPWM_ABC[SVPWM生成<br/>从ABC]
        PI_D[d轴PI控制器]
        PI_Q[q轴PI控制器]
        RSFF_DQ[DQ域Rs前馈<br/>Vdq_rs=Rs×Idq_ref×eff]
        RSFF_ABC[ABC域Rs前馈<br/>Vabc_rs=Rs×Iabc_ref×eff<br/>逐相限幅0.25×Vbus]
        ADAPT[自适应置信度<br/>4因子×非对称LPF]
        BEMF_FF[BEMF前馈<br/>Vd_ff=-ωLq·Iq<br/>Vq_ff=+ω(Ld·Id+Ke)]
        SAT_DQ[DQ矢量限幅]
        SAT_ABC[ABC逐相限幅<br/>|V|≤Vbus/√3]
        DIAG[诊断捕获<br/>DQ+ABC域]
    end

    CLARKE -->|Iα,Iβ| PARK
    PARK -->|Id,Iq| PI_D
    PARK -->|Id,Iq| PI_Q
    PI_D -->|Vd_pi| RSFF_DQ
    PI_Q -->|Vq_pi| RSFF_DQ
    ADAPT -->|confidence| RSFF_DQ
    ADAPT -->|confidence| RSFF_ABC
    RSFF_DQ -->|Vd_cmd| BEMF_FF
    RSFF_ABC -->|Vabc_cmd| SAT_ABC
    BEMF_FF -->|+Vdq_bemf| SAT_DQ
    SAT_DQ -->|Vd_sat,Vq_sat| INV_PARK
    INV_PARK -->|Vα,Vβ| SVPWM_DQ
    SAT_ABC -->|Vabc| SVPWM_ABC
    SAT_DQ -->|捕获| DIAG
    SAT_ABC -->|捕获| DIAG
    SVPWM_DQ -->|Ta,Tb,Tc| PWM输出
    SVPWM_ABC -->|Ta,Tb,Tc| PWM输出
```

## 接口

### 提供的接口

| 接口 | 类型 | 描述 | 消费者 |
|------|------|------|--------|
| `FOC_Init()` | 函数 | 初始化PI控制器、默认参数 | FOC 应用层 |
| `FOC_Run()` | 函数 | 执行一次电流环迭代 | FOC 应用层 (ISR) |
| `FOC_SetCurrentReference()` | 函数 | 设置 Id/Iq 参考值 | FOC 应用层 |
| `FOC_SetVbus()` | 函数 | 更新母线电压+Vmax | FOC 应用层 (每周期) |
| `FOC_SetBemfParams()` | 函数 | 配置 BEMF 前馈参数 | FOC 应用层 |
| `FOC_SetOmegaElec()` | 函数 | 更新电角速度 (BEMF用) | FOC 应用层 |
| `FOC_SetAngle()` | 函数 | 更新电角度 (Park变换用) | FOC 应用层 |
| `FOC_UpdateCurrent()` | 函数 | 更新相电流反馈 | FOC 应用层 |

### 依赖的接口

| 接口 | 类型 | 描述 | 提供者 |
|------|------|------|--------|
| 相电流 Ia,Ib,Ic | 数据 | ADC 采样并校准后的电流 | ADC 采样 |
| 电角度 θ_elec | 数据 | 编码器角度变换后的电角度 | FOC 应用层 |
| 电角速度 ωe | 数据 | 机械速度×Pn | FOC 应用层 |
| 母线电压 Vbus | 数据 | 用于 Vmax 计算 | FOC 应用层 |

## 关键数据结构

```c
typedef struct {
    // PI 控制器
    FOC_PI_Controller_t pi_d;    // d轴电流环
    FOC_PI_Controller_t pi_q;    // q轴电流环

    // 参考值
    float Id_ref, Iq_ref;        // 电流参考
    float Vbus;                  // 母线电压 (V)

    // 电阻前馈
    float current_resistance_ohm; // Rs (Ω)
    float rs_ff_scale;           // Rs前馈缩放 (0~1, 诊断用)

    // BEMF 前馈
    float omega_elec_radps;      // 电角速度 (rad/s)
    float bemf_Ld, bemf_Lq;      // d/q轴电感 (H)
    float bemf_Ke;               // 电角速度基准 Ke (V·s/rad)
    float bemf_Ke_temp;          // 临时 Ke 覆盖 (0=使用默认)
    uint8_t bemf_enabled;        // 硬件使能 (Ld/Lq>0)
    uint8_t bemf_user_enable;    // 用户运行时使能
    uint8_t bemf_blocked;        // 保护门禁阻止

    // 电流环诊断 (每周期更新)
    float diag_vd_rs_ff, diag_vq_rs_ff;  // 电阻前馈分量
    float diag_vd_pi, diag_vq_pi;        // PI输出分量
    float diag_vd_bemf, diag_vq_bemf;    // BEMF前馈分量
    float diag_vd_cmd, diag_vq_cmd;      // 限幅前总指令
    float diag_v_mag;                     // 限幅前矢量幅值
    float diag_sat_ratio;                // 饱和比 (1.0=未饱和)

    // 输出
    FOC_DQ_t Vdq;                // 饱和后 dq 电压
    FOC_AlphaBeta_t ValphaBeta;  // αβ 电压
    FOC_SVPWM_t svpwm;           // SVPWM 占空比

    uint8_t enabled;
} FOC_Handle_t;
```

## 数据流

```mermaid
flowchart LR
    ADC[ADC相电流<br/>Ia,Ib,Ic] -->|Clarke| AB[Iα,Iβ]
    ENC[编码器] -->|θ_elec| PARK[Park变换]
    AB --> PARK
    PARK -->|Id,Iq| ERR[误差计算<br/>err=ref-fb]
    REF[Id_ref,Iq_ref] --> ERR
    ERR -->|err_d,err_q| PI[PI控制器<br/>Kp·err+Ki·∫err]
    PI -->|Vd_pi,Vq_pi| RSFF[Rs前馈<br/>Rs·Iref·scale]
    RSFF -->|V_rsff| BFF[BEMF前馈<br/>-ωLq·Iq_ref<br/>+ω(Ld·Id+Ke)]
    OMEGA[ωe] --> BFF
    BFF -->|Vd_cmd,Vq_cmd| SAT[电压限幅<br/>|V|≤Vbus/√3]
    SAT -->|Vd_sat,Vq_sat| IPARK[逆Park]
    IPARK -->|Vα,Vβ| SVPWM[SVPWM]
    SVPWM --> PWM[PWM占空比]
```

## 设计说明

1. **BEMF 默认关闭**：`bemf_user_enable=0`，每次启动需显式使能，防止参数错误飞车
2. **Ke 量纲**：`bemf_Ke` 是电角速度基准值 (V·s/rad electrical)，由 `motor_param.Ke/Pn` 得到
3. **speed_elec 不乘 encoder_dir**：物理 BEMF 方向与坐标变换约定无关
4. **保护门禁**：`|ωe×Ke| > 0.8×Vbus/√3` 时自动阻止 BEMF，设置 `bemf_blocked=1`
5. **Vmax = Vbus/√3**：通过 `FOC_SetVbus()` 每周期更新，保证抗积分饱和在正确物理阈值触发
6. **Kp 上限**：12V 母线下 J=0.0001 低惯量电机，Kp≥0.60 已出现正向过冲/振荡
7. **自适应 RsFF**：4 因子置信度（dIq/dt@2000A/s, speed_err@1.0/0.8, sat_ratio@0.95, sign_mismatch）+ 非对称 LPF（瞬时降/300ms 恢复），默认 ON
8. **Sign Protect**：幅值门限（|Iq_ref|>0.08A, |Iq_fb|>0.06A）+ 50 周期持续（~5ms），防止正常换向/制动瞬态误触发
9. **ABC RsFF 路径**（RS_FF_MODE=2）：PI+BEMF→InvPark→InvClarke→Vabc_pi, Idq_ref→Iabc_ref→Vabc_ff（逐相≤0.25×Vbus）→ SVPWM from ABC，保留为实验项
10. **DQ 路径**（RS_FF_MODE=1，默认）：Vdq_cmd = Vdq_pi + Rs×Idq_ref×eff + Vdq_bemf → DQ 矢量限幅 → InvPark
11. **RS_FF_MODE=0**：完全关闭 RsFF，仅 PI 驱动
