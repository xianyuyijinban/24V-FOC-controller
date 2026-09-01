# 观测器/Ke 调查（2026-09-01，任务 2）

**对象**: 24V FOC 控制器（STM32H743 + TLE5012B + DRV8350S），分支 foc-public-0817 @ 84de679
**目的**: 20kHz 报告遗留"观测器/Ke 复核"（VESC 共识里低速平滑两个前提项）
**方式**: 代码阅读 + 离线数据盘点，**零烧录零台架实验**

---

## 1. 速度反馈路径（代码事实）

速度环反馈当前走 **编码器差分**，不是观测器：

```
theta_mech ──► delta_theta = theta_mech - speed_theta_prev   (foc_app.c:878)
              ├─ 环绕处理 (foc_app.c:890-893)
              ├─ speed_raw = delta_theta * FOC_SPEED_LOOP_FREQ   (foc_app.c:899)
              ├─ 一阶 LPF  (foc_app.c:900): speed_mech += α(speed_raw - speed_mech)
              └─ 加速度限幅 (foc_app.c:901-902): FOC_SPEED_EST_ACCEL_LIMIT
speed_feedback = speed_mech * enc_dir   (foc_app.c:907/950)
```

低速时该路径的本质：TLE5012B 15-bit 编码器（0.011°/LSB），速度环 2kHz 拍一次，
每拍增量 δθ = ω·Ts。0.1°/s 时 δθ = 0.1/3600·2kHz ≈ 0.056 LSB/拍 —— **低于 1 LSB，量化为纯±1 LSB 抖动 → 抖动源**。
这符合 20kHz 报告表 2.1 观察"0.02-0.1 rad/s 角度纹丝不动、爬行、反向"。

**观测器（ESO, foc_observer.c）**：默认 `obs_use_speed=0`（速度环反馈用差分）、`obs_use_d=0`（POS_DIRECT D 项用差分）、`obs_use_speed_obs=0`。
当前定版下观测器**未接入**速度环/位置环反馈路径，只在 OBS? 诊断中输出。
`FOC_SPEED_OBSERVER` 编译开关默认关闭（foc_app.h:26 附近 `#define FOC_FF_ENABLE_OBSERVER 0`）。

因此：**低速量化抖动是真实存在的，但当前控制路径还没启用能平滑它的观测器**。

---

## 2. Ke 使用路径（代码事实）

Ke 是个**复合常数**，在不同路径有不同含义，需要区分：

| 路径 | 使用方式 | 当前位置 | 是否接入控制 |
|---|---|---|---|
| **BEMF 前馈** (foc_core.c:656) | `vq_bemf = ωe·(Ld·Id_ref + Ke)`，`ωe = speed_elec = speed_mech·Pn`（电角速度基准） | `motor_param.Ke` → `FOC_SetBemfParams` (foc_app.c:2527, Ke_elec=Ke/Pn) | **未接入**（`bemf_user_enable=0` 需显式 `CMD:BEMF,1`） |
| **P4 扰动观测器** (foc_app.c:1226+ 门禁) | `T_est` → 前馈 `T_est/Kt`（Kt 取 Ke） | `FOC_FF_ENABLE_OBSERVER=0` 编译关闭 | **未接入** |
| **ESO 速度观测器** (foc_observer.c) | 模型 `J·dω/dt = Kt·Iq - T_load`，Kt=Ke | `obs_use_speed=0` 未接速度环 | **未接入** |
| **电流环解耦项 ωe·λ** | (foc_core.c) 内 `ωe·λ` (λ=Ke·Ld?) 解耦 | BEMF 前馈路径内 | **未接入**（同上） |
| **参数识别** (motor_identify.c:552) | `Ke = |E| / |ωe_meas|` 实测 | 识别后写入 `param->Ke` | 用识别值 |
| **Kt 补偿** (foc_app.c:1060/1096) | 摩擦/惯量前馈 `Kt = motor_param.Ke` | 参与前馈 | **接入**（前馈用） |
| **诊断输出** (uart_upload.c:835) | `ke_used = bemf_Ke_temp>0 ? : bemf_Ke` | 显示用 | 无 |

**关键结论**: Ke 当前**不在低速平滑路径**上——BEMF 前馈（默认关闭）和观测器（编译/运行时默认关闭）都不影响低速行为。
Ke 只在高速 BEMF 前馈（若用户开启）和高频 Kt 补偿相关前馈里起作用。
**低速爬行不是 Ke 不准导致的**（因为 Ke 没接到低速路径上）。

---

## 3. Ke 离线拟合：数据源盘点（任务 2 第二问）

任务卡要求"用已有高速段数据反解 Vq = R·Iq + Ke·ωe，R 用辨识 Rs，数据源 N 帧 Vd/Vq + speed_sweep JSON 速度"。

**数据源盘点（关键发现）：**

1. **speed_sweep JSON 无 Vd/Vq**：`speed_sweep.py:87` 只落 `(label, tick_2khz, theta_user_rad, pos_err_rad, iq_cmd, ff_total, v_mech_rad_s, iq_act, pos_ref_rad, host_rx_time)` ——无 Vd/Vq，无电流环状态。数据源不可用。
2. **N 帧有 Vq**（uart_upload.c:410 `N,%lu,%u,...,vdText,vqText,...`，p[20]=Vd, p[21]=Vq），但**没有脚本存过原始 N 帧行**到磁盘。历史 JSON 都是聚合统计，原始 N 帧行未落盘。数据源不存在。
3. **bemf_regression.py（6 月）**：有 `PWM_DIAG` 静态测试（Vq_pk_static 51-65mV），但那是低速静态 BEMF 扫描，Vq 被淹没在无关电压里，**非高速匀速点**。不可用于直线拟合。
4. **motor_identify.c:552** 有 `Ke=|E|/|ωe|` 的**实测路径**——可能是唯一准确的 Ke 来源，但它跑低速段，尚未在高速段验证。

**结论：Ke 离线拟合当前无法执行——数据源不满足，不是缺推导，是缺采集**：
- 缺：带 Vd/Vq 的原始 N 帧行（或 PDBBIN 扩展帧含 Vq 字段）的**高速匀速段**数据
- 需要跑：高速段采集（如 speed_sweep 高速配 N 帧原始行采集，或 PWM_DIAG 高速段扫描）
- 这是计划清单中一个**未登记的真实数据缺口**，不是可跳过项

---

## 4. 观测器/Ke 复核对低速爬行的影响（结合现有数据）

| 候选因素 | 代码现状 | 是否影响低速爬行 |
|---|---|---|
| 观测器未接（obs_use_speed=0）| ESO 存在但未接入速度环 | **是**——低速量化抖动未能平滑（第一嫌疑人） |
| Ke 只用于 BEMF/高速前馈 | 低速路径无 Ke | **否**——低速爬行与此无关 |
| 低速段 BEMF 前馈关闭 | `bemf_user_enable=0` | **否**——低速 BEMF 可忽略（死区/Rs 占主导），不开不影响 |
| 速度环反馈差分+LPT | 编码器差分 0.056 LSB/拍 | **是**——量化噪声直达速度环 |

**所以：低速平滑两个 VESC 前提项，当前真正相关的是**：
1. **接入 ESO**（obs_use_speed=1，低速用观测器速度，w0 需调低）
2. **不是 Ke**（Ke 若用于低速 BEMF 前馈会被死区/Rs 淹没，无意义）——但 Ke 对**高速** BEMF 前馈是必要项，若用户计划开启高速 BEMF，需要先精准测量 Ke

---

## 5. 建议下一步（是否执行待定，非本任务范围）

1. **观测器接入实验**（低速平滑主攻方向）：
   - `CMD:OBS_CFG,<w0>,use_d,use_speed` 开启 obs_use_speed=1，w0=12 先试
   - 复跑 0.02/0.05/0.1 rad/s 低速场景，看爬行/抖动是否改善
   - 若 w0=12 太滞后，调低到 8 或 6
2. **Ke 精准测量**（高速 BEMF 前驱前置项，非本任务）：
   - 高速段（如 50-300 rpm）采集 N 帧原始行（Vd/Vq/Iq/ω）
   - 拟合 Ke = (Vq - R_s·Iq)/ωe，多窗取中位
   - 对比配置值 0.129，偏差 >10% 判定为需更新

---

**数据源盘点结论**：本任务调查发现"Ke 离线拟合"计划在**数据源上不成立**——不是缺推导，是缺带 Vd/Vq 原始行的采集。这是 20kHz 报告/任务清单里应被纠正的一个可执行性缺口。按计划优先级（加 20kHz 观测器后高速 BEMF 前驱），建议把 Ke 精准测量排到观测器实验之后做，且所需高速采集需专门做一次（当前所有历史文件都不含所需字段）。
