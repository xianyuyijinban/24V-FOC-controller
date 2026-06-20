# PROGRESS

已并入 [PROCESS.md](PROCESS.md)。

从 `2026-04-05 20:10` 起，`PROCESS.md` 是唯一主日志；本文件仅保留为兼容入口，避免后续台架调试继续分叉记录。

## [2026-06-20] BEMF 量纲/符号修复 + 电流环运行时诊断与开关

### Problem / Task
1. 电机在 12V 母线力矩模式下失控飞转（speed=69 rad/s），Vq 饱和在 ~6.9V
2. 诊断发现 `motor_param.Ke=0.129` 是机械侧常数，BEMF 前馈用 `omega_elec * Ke = 630×0.129 = 81V`，直接打满电压矢量
3. Ke 除以 Pn 后（0.0117），BEMF ON 仍然恶化：Iq 符号反转
4. `speed_elec = speed_mech * Pn * encoder_dir` 导致 encoder_dir=-1 时 BEMF 符号翻转，正反馈

### Resolution
1. Ke 量纲修复：`bemf_Ke = motor_param.Ke / Pn` (0.129/11=0.0117)
2. BEMF 符号修复：`speed_elec = speed_mech * Pn`（不乘 encoder_dir），物理 BEMF 方向与坐标变换约定无关
3. 新增运行时 BEMF 开关：`CMD:BEMF_CFG,0/1`、`CMD:BEMF_CFG?`、`CMD:KE_TEMP,<ke>`，默认关闭
4. BEMF 保护门禁：`|omega_e * ke| > 0.8 * Vbus/sqrt(3)` 自动阻止，设置 `bemf_blocked=1`
5. Vmax 修复：`FOC_Init` 改用 `foc->Vbus/sqrt(3)`，`FOC_SetVbus` 每周期同步
6. 电流环诊断：`FAULT_DETAIL` 新增 `CurrentLoopDiag`（RsFF/PI/BEMF/PreSat/sat_ratio）和 `BEMF Ctrl`（user/hw/blocked/Ke_used/omega_e）
7. 新增诊断命令：`CMD:RS_FF_SCALE,<0~1>`、`CMD:ADC_ZERO,<n>`
8. `CMD:BEMF_CFG?` 修复：strcmp 移除 `\n`

### Prevention / Follow-up
1. 永远不在 BEMF 前馈计算中对 `omega_e` 乘以 `encoder_dir`
2. `motor_param.Ke` 语义固定为机械侧 Kt/Ke_mech，BEMF 前馈使用 `Ke/Pn`
3. BEMF 默认关闭（`bemf_user_enable=0`），每次启动需显式使能
4. 电流环诊断行作为标准 FAULT_DETAIL 输出，调参必查
5. 12V 下 Kp 上限受低惯量（J=0.0001）限制，Kp≥0.60 已正向过冲/振荡
6. 遗留：负向 SREF 不转、正负 Iq 不对称、Ld/Lq 交叉耦合符号

### Verification
- BEMF OFF 速度模式：SREF=0.5→spd=0.36, SREF=1.0→spd=0.85 ✅
- BEMF ON (Ke=0.006-0.0117) 力矩：spd≤3 rad/s，不飞转 ✅
- ADC 零点：raw=offset=(2052,2062,2067) ✅
- RS_FF_SCALE=0 纯 PI：Vq 极性正确 ✅

### Commit
- Commit: `c328c4d` (固件), `84c704d` (文档+脚本)
- Branch: `codex/sync-main-20260519`
- Files:
  - `Core/Src/stm32h7xx_it.c`
  - `MDK-ARM/code/foc_core.h`
  - `MDK-ARM/code/foc_core.c`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/uart_upload.c`
  - `HostComputer/main_window.py`
  - `.claude/architecture/README.md`
  - `.claude/architecture/CHANGELOG.md`
  - `.claude/architecture/subsystems/foc-core.md`
  - `.claude/architecture/subsystems/foc-app.md`
  - `PROGRESS.md`
  - `PROCESS.md`
  - `scripts/current_loop_tune.py`
  - `scripts/verify_current_loop.py`
  - `scripts/verify_speed_current.py`
  - `scripts/diag_check.py`
  - `scripts/serial_diag.py`

## [2026-06-13] V5 Motion Speed Runtime Configuration 正式基线

**Commit `31b261d`** — Add runtime motion speed config for V5 baseline.

### 新增能力

| 项目 | 值 |
|---|---|
| 默认运动配置 | speed=4.0 rad/s, accel=6.0 rad/s², cruise=1.2 rad/s |
| UART命令 | `CMD:MOTION_CFG,<s>,<a>,<c>` / `MOTION_CFG?` / `MOTION_CFG_RESET` |
| 运行时改参 | 立即生效，夹紧ramp避P2尖峰，保留PI积分 |
| 非法参数保护 | `MOTION_CFG,FAIL,range`，旧配置不变 |
| FAULT_DETAIL | 新增 `MotionCfg: speed=... accel=... cruise=...` 行 |
| V4回退档 | `CMD:MOTION_CFG,2.0,2.0,0.8` |

### 功率级验证

| 测试 | 结果 |
|---|---|
| V4回退 ±5° | err≤0.9° |
| V4回退 ±20° | err≤0.9° |
| V5默认 20° | err=0.08° |
| V5默认 40° | err=1.51° |
| V5默认 60° | err=2.55° |
| V5默认 80° | err=1.35° |
| 0↔40° ×20 | max=1.68°, avg=1.28° |
| 运行中改参 2→4→5 | 无突跳、无过流 |
| 非法参数 ×5 | 全部 FAIL,range |
| 0 fault | ✅ |

### 修改文件

- `foc_app.h` — 7新宏 + 3 runtime字段
- `foc_app.c` — 5处宏→运行时字段，位置模式(speed loop + position loop + PD init×3)
- `stm32h7xx_it.c` — 3个MOTION_CFG命令处理器
- `uart_upload.h/c` — FormatFixed导出 + MotionCfg诊断行 + TrajDiag更新

## [2026-06-13] FOC Feedforward Baseline v1 定版

**P1+P2+P3+P0 全部就绪，Phase A-E + P0 A/B 通过。**

### Baseline v1 配置

| 前馈 | 状态 | 参数 |
|---|---|---|
| P1 BEMF | ON | — |
| P2 Inertia | ON | J=3.2e-5±12%, B≈0.002 |
| P3 Friction | ON | B=0.002, Tc=0 |
| **P0 Cogging** | **ON** | **gain=0.25, phase=+60°** |
| P4 Observer | OFF | — |

### 关键指标

| 测试 | 结果 |
|---|---|
| PREF ±10° | err ≤1.2° |
| PREF ±20° | err ≤1.0° |
| 10-cycle 0↔20° | 0 faults |
| 负载 Iq_peak | 0.10A (P0 off: 0.20A) |
| 负载 err | ≤1.2° (P0 off: ≤1.8°) |

### P0 运行时调参

```
CMD:COG_CFG,<gain>,<phase_deg>   # 扫参 (gain 0.0-1.0)
CMD:COG_CFG?                     # 查询当前值
CMD:JDIAG                        # cog_gain/cog_phase/cog_valid/cog_bins/cog_min/cog_max
```

### Commits

- `f0c5218`: J 识别闭环电流 + enc_dir 修复
- `035fcfb`: CMD:JDIAG + 周期计时 + N-frame 抑制
- `3de7ed3`: P0 采集启用 + PROGRESS
- `4ebf2ac`: P0 Flash 32B 对齐持久化
- `20c5ffd`: P0 gain + phase 扫参
- `84dacdb`: CMD:COG_CFG 运行时调参

### 本轮关键修复

- **enc_dir**: `motor_identify.c` 三处PN路径 `encoder_dir = sign(Δelec·Δmech)` → `encoder_dir = pn_observed_dir`，修复FOC换向角90°误差
- **J 识别闭环电流**: `foc_app.c` `current_feedback_valid`旁路加`MI_STATE_J_IDENTIFY`，修复电流环被低边窗口门控清零
- **J 周期计时**: `foc_app.c` 控制周期计数器替代 ms 时钟（电机加速仅1.4ms，ms分辨率不足）
- **CMD:JDIAG**: 短命令输出 J/B/Tc/enc/valid/cog_LUT 状态，不再依赖被N-frame交叠破坏的FAULT_DETAIL长文本
- **N-frame 抑制**: FAULT_DETAIL输出期间暂停普通遥测

### 验证数据

| 阶段 | 结果 |
|---|---|
| Phase A 硬件 | DRV Validated, encoder OK, Vbus=23.9V, FAULT1=VGS2=0 |
| Phase B 扭矩 | +Iq→+angle, 方向正确, enc_dir=-1 稳定 |
| Phase C 位置 | PREF=0/±5/±20 全部 <1.5°, HOME坐标链自洽 |
| Phase D1 P1 BEMF | FFDiag: Vd/Vq非零, en=1, 无退化 |
| Phase D2 P3 Friction | FFDiag: en=1, B=0.002有效贡献 |
| Phase D3 P2 Inertia | FFDiag: blk=0, J=3.2e-5±12%收敛 |
| Phase E 负载 | ALL PASS: ±10° err<1.6°, ±20° err<1.8°, Iq×2, 0 faults |
| P0 Cogging | 采集代码就绪, LUT保存待查(cog_valid=0) |

### Commits

- `9c606a28`: 前馈安全基线/FFDiag/上位机 (之前)
- `f0c5218`: J 识别闭环电流通路 + enc_dir修复
- `035fcfb`: CMD:JDIAG + 周期计时 + N-frame抑制

## [2026-04-16 00:53] 位置环PD改造兼容记录
- Problem: 本仓库的正式日志已迁到 `PROCESS.md`，但这次位置环从 `PI` 语义切到显式 `PD` 属于重要控制架构变更，需要在兼容入口保留可追溯记录，避免后续只看 `PROGRESS.md` 的人漏掉这次调整。
- Resolution: 详细变更已写入 `PROCESS.md`，内容涵盖固件位置环 `PD` 公式、`CMD:PD_POS`、Host GUI 与文档同步更新，以及回归测试与构建验证结果；本条用于在兼容入口标记该事件和对应主提交。
- Prevention: 继续以 `PROCESS.md` 为唯一主日志，但遇到控制架构级变更时，在 `PROGRESS.md` 至少追加一条镜像记录，确保兼容入口不会静默失真。
- Commit: 09ad81d961e1e3b808360e38a6456b38fee74f95
- Recurrence policy: Not allowed to happen again.
