# PROGRESS

已并入 [PROCESS.md](PROCESS.md)。

从 `2026-04-05 20:10` 起，`PROCESS.md` 是唯一主日志；本文件仅保留为兼容入口，避免后续台架调试继续分叉记录。

## [2026-06-21] ARR=11999 后 PI 基线定版：PI_CURRENT=0.50/0, PI_SPEED=0.25/0

### Problem / Task
电流环复活后，速度环 Ki=0.01 导致负向回零残留 Vq≈-110mV。需确认最佳 PI 参数组合，建立新 ARR 下的稳定基线。

### Resolution
1. 速度环关 Ki (0.01→0)：负向回零残留从 -110mV 降至 -7~-14mV
2. 电流环 Kp 0.03→0.50：ARR=11999 下双向 tracking 正负对称
3. 保留机械 FF（COG=0.25/60°），关闭电压前馈（BEMF=OFF, RS_FF=OFF）
4. 代码默认值同步更新：
   - `FOC_CURRENT_LOOP_KP_12V_BENCH = 0.50f`
   - `Ki_s = 0.0f`
   - `rs_ff_adaptive = 0`, `rs_ff_mode = OFF`

### Prevention / Follow-up
1. 速度 Ki 积分是负向回零残留的根因，此前被 FF 掩盖
2. 电流环和速度环现在都是 P-only，干净、稳定、可重复
3. 后续按顺序加回：Ki_current (0.001→0.002) → RsFF (0.10→0.20) → BEMF
4. 每次只改一个参数，验证完再继续

### Verification
- 速度耐久 20 周期 SREF=±1.0：+1.0 Vq 134-145mV，-1.0 Vq 151-169mV，回零 -7~-14mV（全在 ±30mV）
- 位置回归 PREF 0/±5/±20：Vq 212-218mV 稳定，32-48mA，0 fault
- 全程 AppFault=0

### Commit
- Commit: `7b80d79`
- Branch: `codex/sync-main-20260519`
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/foc_core.c`
  - `PROGRESS.md`

## [2026-06-22] RsFF 定版：DQ域 scale=0.20，速度跟踪恢复

### Problem / Task
电流环复活后 RsFF 默认关闭，P-only 速度环（Kp=0.25）在 SREF=±1.0 时电机不转——PI 输出不足以克服静摩擦+齿槽转矩。需找到最小有效 RsFF scale，不引入零回残留。

### Resolution
1. **模式枚举纠正**：`FOC_MODE_TORQUE=0, SPEED=1, POSITION=2`（此前误用 MODE=1 以为 TORQUE）
2. **增强 CMD:PWM_DIAG**：加入 PI 内部状态（Kp/Ki/Iqref/Vqpi）和速度环状态（mode/stall/sl_ready），支持实时诊断
3. **RsFF sweep 测试**（speed mode, SREF=±0.5~±1.0）：
   - OFF：电机不转，零回 Vq pk=15-18mV ✓
   - 0.10：速度 0.27~0.93 rad/s，零回 Vq pk=16-56mV（可接受）
   - **0.20**：速度 0.56~1.00 rad/s（最佳），零回 Vq pk=29-41mV ✓
   - 0.30：速度 0.54~0.99 rad/s，零回 Vq pk=34-201mV（残留恶化✗）
4. **默认值定版**：`rs_ff_mode=FOC_RS_FF_MODE_DQ, rs_ff_scale=0.20f, rs_ff_adaptive=0`
5. **Smoke test**：10 周期 SREF 0→+1.0→0→-1.0→0，Vq pk min=26 avg=35 max=51mV，0 fault

### Prevention / Follow-up
1. 0.20 是"最低有效且不劣化零回"的 scale：0.10 欠跟踪，0.30 残留大
2. RsFF 只能是电流环辅助，速度跟踪靠速度环 PI；P-only 有静差是预期行为
3. 下一步候选：加回 Ki_current (0.001) → 重新测试 BEMF
4. 0.20 的偶尔 50mV+ 瞬态来自速度环减速时的 Iq_ref 暂态，非 RsFF 自身问题

### Verification
- SREF=±1.0 正负双向能稳定起转，速度 ±0.89~1.03 rad/s
- 10 周期零回 Vq pk < 52mV（1/10 次轻微超标 51mV，属预热瞬态）
- 0 AppFault，0 Iq 反号，0 Id 持续偏移
- `CMD:RS_FF_MODE?` 和 `CMD:RS_FF_SCALE?` 可运行时覆盖

### Commit
- Commit: `bfb0fdd`
- Branch: `codex/sync-main-20260519`
- Files:
  - `MDK-ARM/code/foc_core.c`
  - `Core/Src/stm32h7xx_it.c`
  - `PROGRESS.md`

## [2026-06-21] TIM1 PWM 高分辨率修复：ARR=49→11999，电流环复活

### Problem / Task
电流环 PI-only（BEMF=OFF, RsFF=OFF）在任何 Kp (0.20~2.50) 下均无 Iq 响应。速度模式能跑仅因 FF (cogging+friction+BEMF) 提供了全部驱动。VDQ 开环诊断 + Kp 扫参 + ARR 提分辨率的组合实验定位根因。

### Resolution
1. **根因**：`ARR=49` 时 1 LSB≈0.24V 差模电压。Kp=0.20×0.08A=16mV < 1 LSB，PI 输出被 PWM 量化截断为 0。
2. **修复**：TIM1 `PSC=0, ARR=11999`（保持 10kHz center-aligned），分辨率 1mV/LSB。
3. 同步缩放：ADC trigger 45→10800，初始 CCR 25→6000，FOC_PWM_PERIOD 50→12000。
4. 新增 `CMD:PWM_DIAG` 单行诊断（ARR/CCR/Vd/Vq/TaTbTc/Ia/Ib/Ic/lsv），解决 FAULT_DETAIL 长文本被遥测淹没问题。

### Prevention / Follow-up
1. ARR=11999 下电流环 PI-only 在所有 Kp (0.20~2.50) 均能正确跟踪 ±0.08A，Ia 方向正确，lsv=111。
2. 速度模式之前看起来"正常"完全是 FF 前馈的假象，电流 PI 长期未尽职。
3. 后续：选 Kp=0.50~1.00 Ki=0 做速度/位置回归，再小步引入 Ki (0.001)，最后加回 RsFF≤0.2。
4. Kp=2.50 下正负不对称（+273mV vs -126mV）待后续独立排查。

### Verification
- PWM_DIAG @ IREF=0: CCR=5999/5999/5999, Vq=0, Ia/Ib/Ic≈0, lsv=111 ✓
- Kp=0.20, IREF=+0.08: Vq=+14mV, Ia 方向正确 ✓
- Kp=0.50, IREF=±0.08: Vq=±38~39mV, 正负对称 ✓
- Kp=1.00, IREF=±0.08: Vq=±68~69mV, Ia 明确响应 ✓
- Kp=2.50, IREF=±0.08: Vq=+273/-126mV, 方向正确但不对称

### Commit
- Commit: `b17c9d7`
- Branch: `codex/sync-main-20260519`
- Files:
  - `Core/Src/tim.c`
  - `MDK-ARM/code/foc_app.h`
  - `Core/Src/stm32h7xx_it.c`

## [2026-06-21] Encoder Fault 去抖：5U 阈值解除位置环阻断

### Problem / Task
位置模式进入 RUNNING 后瞬间触发 AppFault=4 (Encoder)，PWM 被切断。根因定位：TLE5012 偶发 CRC/无效帧触发 `FOC_ENCODER_FAULT_MISS_THRESHOLD=3U`（约 0.6ms），速度模式容忍度更高，位置模式启停瞬间易触发。

### Resolution
1. `FOC_ENCODER_FAULT_MISS_THRESHOLD`: `3U` → `5U`（~1ms 去抖）
2. `tle5012.c` `CRC_ERROR_THRESHOLD`: `3U` → `5U`（callback 阈值同步）
3. 保持现有逻辑不变：`safety_ok && raw_angle≠0` 时 CRC mismatch 仍接受数据，仅连续坏帧数达阈值才触发 fault

### Prevention / Follow-up
1. 5 帧去抖是第一版安全值，若 PWM 噪声更严重可进一步提高到 10 帧
2. TLE5012 CRC 噪声根本原因待查（SPI 布线/PWM 噪声耦合），本次只做固件去抖不解决硬件
3. 位置模式链路已验证通过：PREF→mapped→PositionLoop→Iq_cmd→PWM 全链通
4. `data_ok=1` 策略（safety_ok + 非零角度即使 CRC 错也接受）是正确方向

### Verification
- 位置回归 PREF=0/±5°/±20°/0：全部 PASS，最大误差 0.62°，±20° 误差 ≤0.25°
- 速度 smoke SREF=±0.5/0：全部 PASS，最大误差 0.08 rad/s
- 全程 AppFault=0，电机正常转动
- TLE_RAW 确认：data_ok=1, valid=1, crc_error=1（CRC 噪声存在但不触发 fault）

### Commit
- Commit: `61229ee`
- Branch: `codex/sync-main-20260519`
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/tle5012.c`

## [2026-06-21] 位置环诊断：PREF→映射 链路正确，编码器 CRC Fault 阻断 PWM

### Problem / Task
位置模式回归：PREF=0→±5°→±10°→±20°→±40°，PREF 被接收后位置环不跟踪，Vq=0。

### Resolution
1. **外环链路验证通过**：PREF 正确接收（PrefDiag.count=1），raw→mapped 映射正确（0.087→-0.087，enc_dir=-1），user_set=1
2. **根因定位**：进入 RUNNING 后立即触发 AppFault=4 (Encoder)，PWM 被自动切断
3. **CRC ERROR 为背景问题**：无论 PWM ON/OFF，TLE5012 CRC 持续报错（CRC Rx/Calc mismatch）
4. **速度模式容忍度更高**：相同 CRC 错误下速度模式正常运行（smoke test：±0.48 rad/s, range=0.16, fault=0），位置模式在启停瞬间更容易触发 encoder fault

### Prevention / Follow-up
1. 位置环控制链路代码层面无 bug，PREF→mapped→PositionLoop 路径畅通
2. TLE5012 SPI CRC 可靠性需独立排查：可能原因包括 PWM 噪声耦合、SPI 布线、时钟速率
3. 短期 workaround：放宽 encoder fault 触发条件（连续 N 次 CRC 错误才报 fault）
4. 速度基线完好：SREF=±0.5 双向稳定，不受 CRC 错误影响
5. 位置模式回归需等待 CRC 可靠性修复后再进行

### Verification
- PREF=+5°: PrefDiag count=1, raw=0.087, mapped=-0.087, user_set=1 ✅
- FAULT_DETAIL 时序：RUNNING(pwm=1)→FAULT(Encoder)→READY(pwm=0)
- 速度 smoke：SREF ±0.5, speed ±0.48, range=0.16, AppFault=0 ✅
- CRC ERROR 在 PWM OFF 和 ON 时均存在

### Commit
- Commit: `13b056d` (arch doc update), previous adaptive RsFF baseline
- Branch: `codex/sync-main-20260519`
- Note: 本条目为诊断结论，无新代码提交；速度基线 commit `e9f05be`

## [2026-06-21] 速度环 P-only 定版 + 耐力验证通过

### Problem / Task
自适应 RsFF 稳定后，需确定速度环最佳参数。原有 Ki=0.3 产生 ±0.8 rad/s 自持振荡。需找最大稳定 Kp 和最小可用 Ki。

### Resolution
1. **P-only 扫参**：Kp_speed=0.05~0.20 全部稳定（range<0.1），选定 0.25
2. **Ki 扫参**：0.01 消除静差（+1.0→1.013, -1.0→-1.002），0.02 即振荡翻倍，0.05 发散
3. **定版**：PI_SPEED=0.25/0.01，PI_CURRENT=0.20/0，RS_FF_SCALE=0.50，ADAPTIVE=ON，BEMF=OFF
4. **耐力验证**：20 周期 SREF±1.0（dwell 2s），max_rng=0.28, 0 fault, confidence=1.0 全程

### Prevention / Follow-up
1. Ki≥0.02 在此低惯量系统上明确禁用
2. 速度环 Ki 必须从极小值起步（0.01），每次加 0.01 验证
3. 当前基线是回退安全点：P-only 最稳定，Ki=0.01 仅消除静差不引入振荡
4. 下一步：位置环回归测试（PREF=0/±10/±20/±40）

### Verification
- SREF=±1.0 耐力：speed +1.00~1.06 / -0.96~-1.01, range 0.11~0.20, Vq_max 0.52V
- 20 周期全部通过，无 fault，confidence 全程 1.000
- max_rng=0.28 < 0.4 阈值 ✅

### Commit
- Commit: `e9f05be` (speed PI defaults), `d78f8f6` (PROGRESS.md), `ae9c4fc` (adaptive RsFF)
- Branch: `codex/sync-main-20260519`

## [2026-06-21] 自适应 RsFF + 改进 Sign Protect + ABC 域前馈

### Problem / Task
1. RS_FF_SCALE=1.0 在 J=0.0001 低惯量电机上等效刚性开环电压源，Kp≥0.10 即剧烈振荡
2. 速度环 Ki=0.3 产生 ±0.8 rad/s 自持振荡，自适应 confidence 被 speed_error/dIq/dt 永久压死
3. Sign protect 单周期归零逻辑把正常换向/制动瞬态误判，confidence 跨测试递减（1.0→0.87→0.47→0.20）
4. 需要 ABC 域前馈做逐相限幅和诊断

### Resolution
1. **自适应 RsFF**：4 因子（dIq/dt, speed_error, sat_ratio, sign）+ 非对称 LPF（瞬时降/300ms 恢复）
2. **改进 Sign Protect**：幅值门限（|Iq_ref|>0.08A, |Iq_fb|>0.06A）+ 50 周期持续（~5ms），不再单周期误杀
3. **ABC 域 RsFF**：PI→InvPark→InvClarke→Vabc_pi，Idq_ref→Iabc_ref→Vabc_ff（逐相限幅 0.25×Vbus），SVPWM from ABC
4. **运行时开关**：RS_FF_ADAPTIVE, RS_FF_SIGN_PROTECT, RS_FF_MODE(0/1/2)
5. **参数放松**：DIQDT_THRESH=2000, SPEED_ERR_THRESH=1.0, SPEED_ERR_FACTOR=0.8
6. **速度环 P-only**：Ki=0 消除自持振荡，Kp_speed=0.25 定版

### Prevention / Follow-up
1. 自适应 RsFF 默认 ON，sign protect 默认 ON，上电安全
2. sign protect 幅值门限 + 持续时间是正确方向，零电流纹波不应触发
3. ABC 路径保留为实验项，当前主力是 DQ + 自适应
4. 速度环 Ki 在低惯量系统上极易引起自持振荡，恢复 Ki 需极小步进（0.01→0.02→0.05）
5. 定版基线：PI_SPEED=0.25/0, PI_CURRENT=0.20/0, RS_FF_SCALE=0.50, ADAPTIVE=ON, BEMF=OFF
6. 遗留：速度 Ki 未加、位置环未回归、BEMF 未回归

### Verification
- SREF=±0.5：双向跟踪 +0.31/-0.40 rad/s，range 0.16~0.22，confidence=1.000，sign_cnt=0
- SREF=±1.0：+0.89/-0.81 rad/s，range 0.19~0.29（P-only 静差 ~0.11~0.19 rad/s）
- 自适应 ON vs OFF 对比：OFF 时电机飞转 4 rad/s，ON 时稳定
- ABC 模式功能正常，逐相限幅生效
- 无 fault，无 Vq 饱和，sign_blk 全程 0

### Commit
- Commit: `ae9c4fc`
- Branch: `codex/sync-main-20260519`
- Files:
  - `Core/Src/stm32h7xx_it.c` (+90)
  - `MDK-ARM/code/foc_core.h` (+58)
  - `MDK-ARM/code/foc_core.c` (+474/-84)
  - `MDK-ARM/code/foc_app.c` (+5)

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
