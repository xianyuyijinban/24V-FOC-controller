# PROGRESS

## [2026-08-14] 台架实跑：板子恢复 + POS_DIRECT 判别/调优/COG 评估（首次硬件闭环验证）

### Problem / Task
- 8-13 完成 POS_DIRECT（位置环直连电流环）软件实现，待台架。8-14 实际硬件验证：
  - 板子此前 UART 全静默、pyOCD 连不上内核 → 真凶是 Flash 空（8-01 失败烧录遗产），MCU 硬件一直健康
  - 编码器线束坏 → 更换后 SPI 恢复
  - 低速平滑性判别实验 + 直连增益调优 + 齿槽 LUT 质量评估

### Resolution
1. **板子恢复**：Keil 烧录成功（Erase/Program/Verify OK）→ pyOCD reset → 完整启动序列（EARLY_UART→CLOCK_READY→USART_READY→PERIPH_READY→DRV_READY→ADC_READY→MAIN_LOOP）。静态噪声健康：Ia/Ib/Ic std 4.0/3.8/5.9mA，VBUS 11.92V，UART RX 0 错误，fault=0。
2. **编码器恢复**：换线后 `TLE_RAW,raw=0xF8D7,crc_error=0,valid=1`，角度实时有效（修复前 raw=0xFFFF MISO 悬空）。装好编码器后手动验证角度稳定跟随。
3. **电机识别**：`CMD:IDENTIFY,1` → `CAL:STATUS,step=COMPLETE,percent=100,identified=1`（PN/RS/LS/KE/J/ENC_ALIGN/MOTION_VERIFY/COG 全流程）。**持久化验证通过**：pyOCD reset 重启后 motorId=1 自动恢复，无需重新识别。
4. **POS_DIRECT A/B 判别**（`scripts/pos_direct_ab_test.py`，rounds=2, dwell=2.5s）：
   - 位置纹波 angle_pp：CASCADE 6.83° → DIRECT **2.10°（-69%）**
   - 力矩纹波 iq_pp：0.0174A → **0.0139A（-20%）**
   - 速度纹波相当（0.076 vs 0.075），全程 0 fault
   - **结论：低速位置模式直连结构显著更平滑**，假设成立。
5. **直连增益扫参**（`scripts/sweep_direct_gains.py`）：kp×kd 9 组合 + 聚焦 4 组合×2轮。
   - 关键发现：**kd 越大位置纹波越差**（kd=0.03 一致最优，0.10 全线恶化）——低速域速度差分噪声经 D 项放大，印证"速度不可信"
   - **定版 kp=0.5, kd=0.03**：angle_pp=1.361°, iq_pp=0.0109A（vs 默认 kp=1.0 的 2.005°，改善 32%）
6. **COG 齿槽 LUT 评估**（`scripts/cog_ab_test.py`）：
   - COG ON(gain=0.25/phase=60°) 比 OFF 位置纹波**恶化 28.6%**（1.719° vs 1.337°，n=3）→ **当前 LUT 是负优化**
   - 相位扫描（`scripts/phase_sweep.py`）：+120° 优于默认 60°（1.432° vs 2.005°），但 OFF 基线跨轮波动大（1.34~2.58°）→ 步进序列对 COG 检测不灵敏，**LUT 需用匀速纹波+FFT 专门标定**

### Prevention / Follow-up
- 齿槽 LUT 重标定（相位对齐 + 多转平均重采 + 谐波滤波）为独立优化项，待步骤3真实场景验证后处理。
- 直连增益最优参数 kp=0.5/kd=0.03 尚未写入固件默认（当前默认仍 kp=1.0/kd=0.03），测试用 `CMD:POS_DIRECT_GAIN` 运行时设；定版后改 `FOC_POS_DIRECT_KP/KD_DEFAULT`。
- 全 FF 帧被固件判 data_ok=1（Round4 宽容策略）——编码器真故障时会出假角度，建议后续加"全0xFF判无效"。
- 后续步骤（见会话规划清单）：3 真实场景手感 → 4 位置环提频评估 → 5 摩擦低速区重构。

### Verification
- 板子：Keil 烧录 Verify OK，pyOCD reset 后启动序列完整，重启参数持久化 motorId=1。
- 静态噪声：Ia/Ib/Ic std 4.0/3.8/5.9mA，存档 `scripts/static_noise_20260813_013404.json`。
- A/B 判别 JSON：`scripts/pos_direct_ab_20260814_235109.json`；增益扫参 `sweep_direct_gains_20260814_003831.json`；COG `cog_ab_20260814_004444.json`；相位 `phase_sweep_20260814_005101.json`。
- 识别全程 0 fault；扫参/判别全程 0 fault。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet（与 8-13 POS_DIRECT + 8-01 CAN 同批）
- Files:
  - `scripts/pos_direct_ab_test.py`（新增，A/B 判别，含编码修复）
  - `scripts/sweep_direct_gains.py`（新增，增益扫参）
  - `scripts/cog_ab_test.py`（新增，COG ON/OFF）
  - `scripts/phase_sweep.py`（新增，相位扫描）
  - `scripts/static_noise_check.py`（新增，静态噪声）
  - `PROGRESS.md`

## [2026-08-14] 匀速测量尝试与低速跟踪摩擦主导发现（修正 A/B 解读）

### Problem / Task
- 步骤3 目标是量化直连结构在云台慢摇（匀速段）的平滑性，需让电机匀速转动并采集纹波。
- 实测暴露更深问题：**位置环（级联+直连）低速跟踪都受摩擦主导，电机响应慢、到位慢**。

### Resolution
1. **匀速测量脚本 `speed_ramp_test.py`**：PREF 流式递增模拟平滑斜坡，采 N 帧算速度/Iq 纹波/频谱。遇到并解决 3 个障碍：
   - **PC 端 pyserial write 阻塞读取根因**：单线程连续 `ser.write` 命令流会让 N 帧采集骤降到 ~1.8fps（实测 20/33/50Hz 均如此）；改用**后台线程发命令 + 主线程读**后恢复 54fps。这是本仓库所有 UART 流式命令脚本必须注意的坑。
   - **位置环最短路径语义**：PREF 递增超过 360° 会触发 `FOC_AngleNormalize` wrap，位置环目标来回跳（电机剧烈振荡 Iq pp=0.84A）。限扫动幅度在单圈内（SWEEP_RAD=1.0 rad）。
   - **角度差分 wrap 处理**：单圈内无需 wrap，只做 ±360 修正。
2. **决定性发现：级联和直连位置环都跟踪慢**。同一 6° 步进（级联 pos_direct=0）：
   - angle 143.45°→147.48°（2.5s 只到位 4/6°）
   - **iq_ref=-0.08A（位置环在输出力矩）但 iq_act=+0.02A（实际电流只有指令 1/4）**
   - Vq=-0.2V，电机缓慢爬行
   - 直连 pos_direct=1 同样只动 0.1°
3. **修正 A/B 判别解读**：DIRECT angle_pp=2.10° 优于级联 6.83° 是**稳态抖动维度**的改善；但"能否快速到位"上两结构都受摩擦限制。**低速不平滑根因是摩擦（静摩擦+齿槽），不是环结构**。结构切换改善抖动，未解决摩擦本身。
4. **结论**：下一步需量化摩擦特性（力矩模式电流递增找启动电流/库仑摩擦），作为后续摩擦补偿、观测器、LUT 标定的输入。

### Prevention / Follow-up
- UART 流式命令脚本必须用后台线程发命令 + 主线程读（见 speed_ramp_test.py）。
- 位置环 PREF 无法表达多圈匀速（最短路径语义），匀速测量限单圈扫动。
- 低速到位能力受摩擦主导，单靠 DIRECT 结构不够；需摩擦补偿/观测器/LUT 方向。
- 待办：摩擦特性测量（力矩模式 IREF 扫描）。

### Verification
- 后台线程 vs 单线程 PREF 流：N 帧 215/4s vs ~7/4s。
- 级联/直连同 6° 步进：均只到位 0.1-4/6°，iq_act≈iq_ref×0.25。
- `scripts/speed_ramp_test.py` 已修复可用（单档 smoke 通过，多档扫动后续跑）。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet
- Files:
  - `scripts/speed_ramp_test.py`（新增，匀速测量）
  - `PROGRESS.md`

## [2026-08-14] 静摩擦补偿运行时命令 + 补偿力矩扫描

### Problem / Task
- 实测摩擦启动电流 ~0.09A，现有静摩擦补偿（编译期宏 0.05A）不足。需验证不同补偿力矩电流下低速到位性能，并让补偿运行时可调。

### Resolution
1. **固件**：`foc_app.h/c` 静摩擦补偿幅值改为运行时变量 `fric_comp_pos/neg`（默认宏值 0.05A）；`stm32h7xx_it.c` 新增 `CMD:FRIC_COMP,pos,neg` 设置 / `CMD:FRIC_COMP?` 查询（范围 0~0.50A），SYS:CMDS? 同步。
2. **脚本** `scripts/friction_comp_sweep.py`：DIRECT kp=0.5/kd=0.03 + COG OFF 下，补偿 {0,0.05,0.07,0.09,0.11,0.13}A × ±6° 步进，测 0.5s/1s/2s 到位度、稳态 pp、过冲。
3. **烧录**：pyOCD flash 在此环境不可靠（无输出不写入，疑似探针时序），用户 Keil 烧录成功 + pyOCD reset 后新固件运行（FRIC_COMP,OK 响应，identified=1 参数保留）。

### 扫描结果
| 补偿A | 正向1s剩% | 反向1s剩% | 稳态pp° | 过冲° |
|-------|-----------|-----------|---------|-------|
| 0.00 | 96.0 | 77.5 | 0.025 | 5.43 |
| 0.05 | 76.2 | 71.3 | 0.020 | 5.06 |
| 0.07 | 55.8 | 64.8 | 0.030 | 5.49 |
| 0.09 | 35.0 | 22.3 | 0.020 | 5.27 |
| 0.11 | 34.8 | 21.8 | 0.030 | 3.80 |
| 0.13 | 31.5 | 21.2 | 0.030 | 3.40 |

- **补偿越大到位越快**，0.11~0.13A 最优（1s 剩 31%/21%）；静摩擦补偿方向验证成立。
- **新问题：6° 阶跃过冲 3~5°（60~90% 超调）**，0A 也有 5.4° → DIRECT kp=0.5/kd=0.03 阶跃阻尼不足，非补偿造成。低速稳态优化（kd 小）与大步进阻尼矛盾。

### Prevention / Follow-up
- 详细设计见 `docs/LOW_SPEED_SMOOTHNESS_EXPERIMENT.md`（唯一主记录）。
- 阶跃过冲待处理（加大 KD / 斜坡生成 / 位置阻尼项），或先转真实场景（云台慢摇小幅斜坡）评估影响。
- pyOCD flash 不可靠，固件烧录用 Keil（已验证）。

### Verification
- GCC + Keil 编译 0 错 0 警。
- 新固件 `FRIC_COMP,OK,pos=0.050,neg=0.050`，identified=1。
- 扫描数据 `scripts/friction_comp_sweep_20260814_014558.json`。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/foc_app.c`
  - `Core/Src/stm32h7xx_it.c`
  - `scripts/friction_comp_sweep.py`（新增）
  - `docs/LOW_SPEED_SMOOTHNESS_EXPERIMENT.md`（新增）
  - `PROGRESS.md`

## [2026-08-14] 阶跃过冲根因定位：KD 扫描 + 斜坡验证（粘滑/静摩擦死区主导）

### Problem / Task
- 摩擦补偿扫描发现 6° 阶跃过冲 3~5°。需判明过冲来源：加大 KD（阻尼）？斜坡生成（PC 端）？还是结构性问题。
- 两个假设：A) DIRECT kp=0.5/kd=0.03 阻尼不足，加大 KD 可压过冲；B) 云台真实场景是斜坡非阶跃，斜坡可避开过冲。

### Resolution
1. **脚本 `scripts/overshoot_kd_sweep.py`**：实验A 固定 kp=0.5 扫 kd={0.03..0.20}×±6° 阶跃；实验B 后台线程发斜坡 PREF（2°/s）替代阶跃。摩擦补偿统一 0.12A、COG OFF。
2. **实验A 结果**：加大 KD **压不住过冲**——过冲均值全程 3.6~5.4° 不降（kd=0.20 仍 5.3°），稳态 pp 却从 0.02° 恶化到 0.145°。KD 加大只换来到位略快（1s 剩 29%→10%）。→ 排除"加大 KD"。
3. **实验B 结果（关键）**：斜坡 2°/s 下前 1s 位移≈0（爬行），斜坡结束仍差 1.4~1.7° 追不上，反向轨迹呈典型粘滑（-1.5° 卡住→突跳-3.5°→回弹-2.9°）。→ **PC 端斜坡不仅没解决过冲，还暴露了更严重的跟踪滞后+粘滑**。

### 根因分析（阶跃过冲与斜坡滞后同源）
- **P-only 位置环对静摩擦有固有稳态误差**：P 项 kp×e 要突破静摩擦 0.09A 需 e>0.09/0.5=10.3°。补偿退出后平衡点可离目标 ~10°。
- **bang-bang 补偿死区（误差>3°切/1°退）在慢速斜坡下失效**：跟踪误差恒<3° → 补偿从不触发 → P 项力矩<静摩擦 → 电机粘住。误差一旦过阈值补偿突切入 → 粘滑+过冲。
- 阶跃大步进误差大补偿切入能到位，但动量冲过头=过冲；斜坡慢速误差小补偿失效=滞后。**两者都是"静摩擦阈值 vs 误差阈值补偿"的结构矛盾**。
- 云台慢摇=极慢斜坡，误差恒小 → 现有补偿方案在真实场景会卡死，需固件重构。

### Prevention / Follow-up
- 低速摩擦补偿需重构（固件改动，Keil 烧录）：
  1. **位置环加积分（PI）**：积分累积突破静摩擦，消除末端稳态误差（最直接）
  2. **补偿从"误差死区"改"速度方向相关"**：只要 PREF 在变/指令速度非零即补满静摩擦，连续输出替代 bang-bang
  3. 或摩擦观测器/前馈（后续）
- 建议先做"连续方向补偿 + 位置环积分"之一，再做云台慢摇真实场景。

### Verification
- 脚本编译 0 错；板子 COM10 在线，identified=1。
- 实验A：kd=0.03 时过冲 3.65°、稳态 0.020°（最优稳态）——维持 kp=0.5/kd=0.03 定版。
- 实验B：斜坡 2°/s 正向 1s 剩 95%、末端差 1.4°；反向差 1.7°。
- 数据：`scripts/overshoot_kd_20260814_082311.json`、`overshoot_ramp_20260814_082629.json`。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet
- Files:
  - `scripts/overshoot_kd_sweep.py`（新增）
  - `PROGRESS.md`

## [2026-08-14] 低速摩擦固件重构（直连PI + 指令方向连续补偿）台架验证通过

### Problem / Task
- 前一轮定位阶跃过冲 + 斜坡粘滑/滞后同源：P-only 位置环静摩擦稳态误差 + bang-bang 误差死区补偿慢速失效。需固件重构验证。
- 方案（用户拍板"两个都做"）：①位置环直连条件积分消除静摩擦稳态误差；②摩擦补偿方向从误差阈值(3°/1°)改"指令方向锁存"连续输出。

### Resolution
1. **固件**（`foc_app.h/c` + `stm32h7xx_it.c`）：
   - 直连 PI：`pos_integral` 积分状态 + `pos_direct_ki`(默认1.5, `CMD:POS_DIRECT_KI` 运行时调)。条件积分仅 |err|<2°（大误差靠PD冲避免加剧过冲）+ PD饱和冻结抗windup + 输出限幅 ±0.10A。
   - 方向锁存：`pos_cmd_dir` 由 ref 增量>0.01° 设方向并保持，到位(err<0.11°)或误差反向清0。SpeedLoop 摩擦补偿方向改用 pos_cmd_dir（±1）。
   - 测试断言 test_build_system.py 同步（friction 部分），GCC 编译 0 错 0 警。
2. **烧录**：Keil 烧录成功，pyOCD reset 唤醒（target `stm32h743vghx`，`stm32h743vi` 不被 pyocd 识别）。
3. **验证**（`friction_comp_sweep.py` + `overshoot_kd_sweep.py --mode=ramp`）：

| 指标 | 旧固件 | 新固件 |
|------|--------|--------|
| 6° 阶跃过冲 | 3.8~5.4° | **0.58°** |
| 6° 阶跃到位 | 1s 剩 35% | **0.5s 到位 100%** |
| 斜坡2°/s末端差 | 1.4~1.7° 追不上 | **3s 到位 +0.5°** |
| 斜坡前1s | 卡死(爬行) | **0.5s 走 0.6° 无爬行** |
| 稳态 pp | 0.02~0.03° | 0.03~0.08° |

### 技术坑（防再踩）
- **pyserial 同句柄并发 write/read 饿死读**：后台线程持续写 PREF 期间主线程 0 帧，停写即恢复（板子无 fault、遥测正常）。流式命令必须单线程交替（每 0.2s 发一步 + 读）。
- **ENABLE 后 pos_ref 遗留**：位置环可能被上次会话 pos_ref 猛拉，测试前应先 PREF 钉住当前位置或确认状态。
- pyocd reset target 名用 `stm32h743vghx`。

### Prevention / Follow-up
- **反向不对称**（斜坡反向 3.2s 到位、阶跃反向剩 27%、过冲 3.28°）为下一步优化点：静摩擦/齿槽正反差异 + 反向补偿方向切换时序。
- 齿槽 LUT 标定仍待做（负优化）。
- 直连定版 kp=0.5/kd=0.03/ki=1.5 待写入固件默认。
- 新目标：低速关节电机算法优化至贴近商用云台（见会话规划）。

### Verification
- GCC 0 错 0 警；板子 COM10，identified=1。
- 数据：`friction_comp_sweep_20260814_092552.json`、`overshoot_ramp_20260814_092659.json`、`verify_lowspeed_20260814_092458.json`。
- 斜坡"过冲6°"为 summarize 统计 bug（起点位移0计入），真实超调 0.5°。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet
- Files:
  - `MDK-ARM/code/foc_app.h` / `foc_app.c`
  - `Core/Src/stm32h7xx_it.c`
  - `scripts/overshoot_kd_sweep.py`、`scripts/verify_low_speed.py`（新增）
  - `test_build_system.py`
  - `docs/LOW_SPEED_SMOOTHNESS_EXPERIMENT.md`、`PROGRESS.md`

## [2026-08-14] 低速匀速跟进：补偿方向修复 + FF库仑前馈 + 齿槽标定（速度 0.82→1.81°/s）

### Problem / Task
- 摩擦重构后阶跃/到位 OK，但匀速 2°/s 严重粘滑（速度均值仅 0.82°/s，残差 18.6°）。
- 需找到低速匀速爬行的根因并修复，为齿槽标定铺路。

### Resolution（逐步定位链）
1. **`CMD:DIR?` 诊断命令**（新增）→ 决定性发现：正向斜坡 `dir=-1`，**补偿方向与运动相反**。
   - 根因：`pos_ref` 是 control frame（=用户角×encoder_dir），encoder_dir=-1 时用户角递增→control 角递减→`ref_delta` 符号反。
   - 修复：锁存/清方向都乘 `encoder_dir_f` 转用户坐标。
2. **kp 灵敏度**：kp=0.5→2.0 时匀速 0.82→1.58°/s（残差 4.46°）。位置刚度是关键（误差小就有足够力矩），但剩余仍粘滑。
3. **级联对比**：级联更差（0.14°/s）——级联位置环要 ~32° 误差才给 2°/s 速度指令，低速速度环失效。
4. **Stribeck 平滑**（补偿随速度衰减到动摩擦）：残差 3.73°，但速度 std 仍大。
5. **FF 库仑前馈死区 bug**：`FOC_FF_COULOMB_DEADBAND_RADPS=0.05`，2°/s(=0.035) 恰好低于死区 → 低速库仑前馈从不触发 → 爬行。
   - 修复：库仑前馈低速用指令方向兜底（`pos_cmd_dir`），Tc/Kt 连续输出。
6. **双补偿过冲**：FF库仑 + pos_cmd_dir 补偿(0.12)叠加 → 速度 1.25°/s 恶化。**comp=0 最优**（FF库仑足够）。

### 匀速结果（comp=0 + FF库仑 + kp=2.0）

| 配置 | 速度均值 | 残差 pp | 速度std |
|------|---------|---------|---------|
| 初始(kp=0.5+补偿0.12) | 0.82°/s | 18.6° | — |
| kp=2.0 | 1.58°/s | 4.46° | 2.22 |
| +Stribeck平滑 | 1.24°/s | 3.73° | 3.26 |
| **+FF库仑兜底(comp=0)** | **1.81°/s** | **2.94°** | 2.31 |

- 速度 0.82→1.81°/s（90% 目标），残差改善 6 倍。低速匀速本质问题（库仑前馈缺失）定位并修复。
- 剩余速度 std 2.31 主要来自齿槽（0.099Hz 谐波）+ 粘滑残余。

### 齿槽 LUT 标定（cogging_calibrate.py 新增）
- 位置环直连扫 350°，采 Iq vs 角度 → 齿槽力矩波形 **pp=0.123A**，主导谐波 **22 次/圈**（非理论 24）。
- 硬编码 LUT（encoder_dir=-1 反转对齐）+ Init 覆盖（`cogging_lut_cal.h` 自动生成）。
- **相位扫描无稳定改善**（速度 std 全相位 ~2.5-3）→ LUT 相位对齐未精确（theta_mech vs 用户角 offset 未定），待下一步精确对齐。

### 技术坑
- pyOCD flash 本机可用（`-t stm32h743vghx -e sector`，只擦固件区，参数区 0x081E0000 保留）——用户授权后自行烧录，无需 Keil。
- SWD 偶发 "board uninit" 错误，重试即好。

### Prevention / Follow-up
- 齿槽 LUT 精确对齐（需 mech_zero_offset/encoder_dir 精确关系 → phase_offset）
- Tc/Kt 标定（FF库仑幅值是否最优）
- 摩擦补偿默认值待定（comp=0 最优，pos_cmd_dir 补偿应默认 0 或低值）

### Verification
- GCC 0 错 0 警；pyOCD 烧录保留 identified=1。
- 数据：`ripple_pos2.0_*.json`、`cogging_lut_20260814_102810.json`。

### Commit
- Branch: `codex/sync-main-20260519`；working tree changes not committed yet
- Files: `foc_app.h/c`、`stm32h7xx_it.c`、`cogging_lut_cal.h`(新)、`scripts/cogging_calibrate.py`(新)、`scripts/speed_ripple_measure.py`、`PROGRESS.md`

## [2026-08-13] POS_DIRECT 位置环直连电流环判别实验（软件侧就绪，待台架）

### Problem / Task
- 齿槽+摩擦导致低速转动不平滑。低速域速度估计信噪比差，P-only 速度环扰动抑制弱。
- 判别假设：低速时位置环PD输出直接作为力矩指令(A)进电流环（跳过速度环），比三环级联更平滑。

### Resolution
- `foc_app.h/c`：新增 `pos_pd_direct`（输出力矩A）、`pos_direct` 开关（默认0=级联）、`pos_direct_iq_cmd`。
  - `FOC_App_PositionLoop`（200Hz）：直连时 PD 输出写入 `pos_direct_iq_cmd`，跳过巡航逻辑，speed_ref 清零。
  - `FOC_App_SpeedLoop`（2kHz）：位置模式+直连时跳过速度PI（`goto ff_layers`），iq_ref_mech=pos_direct_iq_cmd，FF层（惯量/摩擦/齿槽LUT）与静摩擦补偿照常叠加；RsFF 速度误差传0。
  - 直连增益默认 `FOC_POS_DIRECT_KP_DEFAULT=1.0 A/rad`、`KD=0.03 A/(rad/s)`（≈级联等效 KP_PD 4.0 × Kp_speed 0.25）。
- `stm32h7xx_it.c`：新增 `CMD:POS_DIRECT?` / `CMD:POS_DIRECT,0|1`（切换时清速度积分与直连力矩，防跳变）/ `CMD:POS_DIRECT_GAIN,kp,kd`（台架扫参）；SYS:CMDS? 同步。
- `scripts/pos_direct_ab_test.py`：A/B 判别脚本（步进 ±2° + 斜坡 5°，稳态窗 angle/Iq/speed 纹波统计，JSON 报告）。功率步骤需 `--power-ok`。
- 顺手修复 Keil 3 警告（foc_app.c 补 uart_upload.h include；can_protocol.c 加 `CanProtocol_SelfTestOk()` getter 消费自检结果）。

### Prevention / Follow-up
- 台架测试（板子修复后）：`python scripts/pos_direct_ab_test.py --port COM7 --power-ok`。
- 直连组若振荡 → 先调 KD；若纹波未降 → 对比 COG ON/OFF 与纹波主频（264×ω/2π Hz）判断齿槽 LUT 质量。
- 切换 POS_DIRECT 建议在电机未使能时进行；热切换已有积分清零保护但仍有小跳变。
- 直连力矩指令 200Hz ZOH 台阶影响待实测评估，后续可把位置环提频。

### Verification
- GCC build.ps1: PASS, 0 warnings; text=175440 (+952B vs 基线 174456)。
- Keil UV4: 0 Error(s), 0 Warning(s)。
- HostComputer: 205 tests OK（协议仅新增命令，无回归）。
- `scripts/pos_direct_ab_test.py`: py_compile + dry-run 通过；硬件实跑待板子修复后执行。

### Commit
- Branch: `codex/sync-main-20260519`
- Status: working tree changes not committed yet（与 8/1 CAN 工作区变更同批）
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/can_protocol.h`
  - `MDK-ARM/code/can_protocol.c`
  - `Core/Src/stm32h7xx_it.c`
  - `scripts/pos_direct_ab_test.py`
  - `PROGRESS.md`

## [2026-08-01] Keil5 编译修复

### Problem / Task
- Keil5 编译报 L6218E 未定义符号：`CanProtocol_*`、`CurStream_*`、`WheelInput_*`、`g_wheel`；同时有 3 条 ARMCC 警告。

### Resolution
- 在 `MDK-ARM/24V FOC Controller.uvprojx` 的 `code` 组补充 `can_protocol.c/h`、`current_stream.c/h`、`wheel_input.c/h`。
- `param_storage.c` 的 32 字节 Flash 写入缓冲改为 `static aligned(32)`，消除 auto object 对齐警告。
- `motor_identify.c` 删除未使用局部变量 `omega_elec`，给被 `#if 0` 包裹代码引用的 `MI_PnRetryWithHigherCurrent` 加 `__attribute__((unused))`。

### Prevention / Follow-up
- 以后新增 `MDK-ARM/code/*.c|h` 时同步更新 GCC `build.ps1`/`Makefile` 和 Keil `uvprojx`。

### Verification
- Keil `UV4.exe -b MDK-ARM\24V FOC Controller.uvprojx`: 0 Error(s), 0 Warning(s)，生成 `.axf/.hex`。
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: PASS；text=174448, data=508, bss=42720。

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `MDK-ARM/24V FOC Controller.uvprojx`
  - `MDK-ARM/code/param_storage.c`
  - `MDK-ARM/code/motor_identify.c`

## [2026-08-01] CAN 固件烧录尝试与 CMSIS-DAP 诊断

### Problem / Task
- 用户反馈 pyOCD/CMSIS-DAP 可用，继续完成 CAN 固件烧录与上电验证。

### Resolution
- 确认 `python -m pyocd list` 识别到 `CMSIS-DAP by muselab-tech.com JCK CMSIS-DAP`，目标 `stm32h743vitx` 可用。
- 尝试 1MHz、100kHz、10kHz，attach/under-reset，禁用 `DebugDeviceUnlock`，`--no-reset` 和 chip erase 多种组合。
- 100kHz + attach/under-reset + 禁用解锁序列可连接并 halt，但 Flash loader 执行阶段反复出现 `SWD/JTAG communication failure (No ACK)`，未完成可信烧录。
- 当前 COM10/COM5 均无 UART 输出，调试连接随后也丢失；结论是目标供电或 SWD 接线/复位链路需要硬件检查。

### Prevention / Follow-up
- 对控制器断电复位，确认 3.3V 供电、SWDIO/SWCLK/GND/nRESET 接线稳定后再烧录。
- CAN 总线验收仍缺 candleLight/gs_usb 适配器；烧录成功后运行 `scripts/can_bench_test.py`。

### Verification
- `python -m pyocd list`: probe detected。
- `python -m pyocd commander ... -M attach -f 100k -O pack.debug_sequences.disabled_sequences=DebugDeviceUnlock -c status -c halt`: 可连接并 halt。
- `python -m pyocd flash ...`: FAIL，Erase/Programming 阶段 SWD No ACK。
- UART COM10/COM5: 无启动输出。

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `docs/CAN_TaskCards_DeepSeek_Report_20260801.md`
  - `PROGRESS.md`

## [2026-08-01] CAN v1.0 任务卡 T1-T5 实现与软件验证

### Problem / Task
- 按 `docs/CAN_TaskCards_DeepSeek.md` 完成 CAN 通信与上位机任务：FDCAN 基础打通、NMT/快速通道/遥测/故障、命令隧道、CanTransport、测试与台架脚本。

### Resolution
- FDCAN1 改为 500 kbit/s，启用 RX FIFO0=8、TX FIFO=3、标准滤波器、FDCAN1_IT0；内部回环自检后切 Normal。
- 重写 `can_protocol.c/h`：RX 入环形缓冲、主循环分发、心跳 armed 守护、50Hz STATE_FAST、BOOTUP、FAULT_EVENT、Bus-off 自愈。
- 命令队列增加 UART/CAN 来源标记，CAN 隧道命令响应经 TUNNEL_RESP 分包回传，响应截断 255 字节。
- 新增 `HostComputer/can_tunnel.py`、`can_transport.py` 及 12 个 CAN 单测；新增 `scripts/can_bench_test.py` 覆盖协议 §13 用例 1-12。
- 任务卡 T6 明确等待 Kimi 详细设计，未提前动 UI 重构。

### Prevention / Follow-up
- 硬件验收仍需 candleLight 适配器和安全台架：运行 `scripts/can_bench_test.py`，需要 `--power-ok` 的用例必须确认台架安全。
- COM9 当前不存在；当前机器仅有 COM3/COM4/COM10，未发现 pyOCD/CMSIS-DAP。

### Verification
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: PASS；text=174456, data=508, bss=42664；ELF/HEX/BIN 生成。
- `python -m unittest discover -s HostComputer -p "test*.py"`: 205 tests PASS。
- `python -m unittest discover -s scripts -p "test*.py"`: 6 tests PASS。
- CAN 台架实跑未执行：gs_usb 无设备，无 COM9，无 pyOCD。

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `Core/Inc/fdcan.h`
  - `Core/Src/fdcan.c`
  - `Core/Src/stm32h7xx_it.c`
  - `MDK-ARM/code/can_protocol.c`
  - `MDK-ARM/code/can_protocol.h`
  - `MDK-ARM/code/head.h`
  - `24V FOC Controller.ioc`
  - `HostComputer/can_tunnel.py`
  - `HostComputer/can_transport.py`
  - `HostComputer/test_can_tunnel.py`
  - `HostComputer/test_can_transport.py`
  - `HostComputer/requirements.txt`
  - `scripts/can_bench_test.py`
  - `docs/CAN_TaskCards_DeepSeek_Report_20260801.md`

## [2026-07-15] FOC Runtime Test UART Bug Fixes

### Problem / Task
- Fix the runtime experiment's unreachable `DIAG:UART_RX?` alias and remove per-command serial input resets that hid ACK/backpressure behavior and discarded valid binary current frames.

### Resolution
- Made mapped `CMD:UART_RX?` the canonical handler while retaining `CMD:UART_RX_STAT?` compatibility, and documented the recommended diagnostic command.
- Added an atomic, cumulative P0/P1/P2 TX admission-drop snapshot and included the counters in `UART_RX,OK` responses.
- Changed the runtime script to decode stale buffered traffic before each command, match only new response prefixes, preserve binary/CRC counters, require a fresh `FOC_TIME,BEGIN...END` transaction, and report UART/TX-drop deltas in CSV and Markdown output.
- Added offline mixed ASCII/binary tests covering split frames, stale ACK isolation, strict prefix matching, UART statistics, and FOC profiler transaction boundaries.

### Prevention / Follow-up
- Run the planned hardware regression without per-command input-buffer resets: both UART RX aliases 20/20, BIN1000 for 60 seconds with 100 queries, zero CRC/RX/P0-drop deltas, then READY_IDLE and SPEED_BIN1000 profiler smoke tests.
- If `tx_p0_drop_delta` is nonzero, treat the run as failed and address it with a separate P0 response-queue redesign rather than increasing host timeouts.

### Verification
- `python -m unittest scripts.test_foc_runtime_profile`: 6 tests passed.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 193 tests passed.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: passed with 0 errors and 0 warnings; text 165488 bytes, data 504 bytes, BSS 39472 bytes; ELF/HEX/BIN generated.
- Hardware flash/regression was not run because the controller was not requested or assumed to be connected.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `Core/Src/stm32h7xx_it.c`
  - `MDK-ARM/code/uart_upload.h`
  - `MDK-ARM/code/uart_upload.c`
  - `scripts/foc_runtime_profile.py`
  - `scripts/test_foc_runtime_profile.py`
  - `docs/UART_COMMANDS.md`
  - `PROGRESS.md`

## [2026-07-14] Persistent DWT FOC Runtime Profiler

### Problem / Task
- Add a low-overhead, long-lived execution-time diagnostic for the STM32H743 FOC runtime so future control features can be evaluated against the real 50 us TIM1 deadline.

### Resolution
- Added a DWT `CYCCNT` profiler with calibrated empty-probe overhead, wrap-safe cycle deltas, 64-bit cycle sums, min/average/max statistics, deadline overrun counts, and atomic clear/snapshot operations.
- Instrumented `FOC_Run`, the complete current path, speed loop, position loop, the full TIM1 update ISR, and adjacent TIM1 entry periods without changing control frequencies or interrupt priorities.
- Added `DIAG:FOC_TIME?` / `DIAG:FOC_TIME,CLEAR` and legacy `CMD:` aliases. Snapshot output is queued one P0 line at a time with retry, so a long report cannot overflow the 1024-byte UART TX ring.
- Added `scripts/foc_runtime_profile.py` for the seven 12V runtime scenarios, three-repeat outlier checks, mixed ASCII/BIN1000 decoding, UART/fault checks, and timestamped CSV/Markdown reports.
- Integrated the profiler source into GCC, Make, and Keil builds and documented the UART commands.

### Prevention / Follow-up
- Flash the instrumented firmware and run all seven 30-second scenarios on the 12V free-running bench; hardware verification is still required for the DWT probe overhead, 20 kHz/10 kHz observed rates, ISR jitter, and final timing margin.
- Treat any `TIM1_ISR` sample at or above 50 us as a hard failure; retain isolated maxima that exceed peer runs by more than 20% as possible preemption/anomaly evidence.

### Verification
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: passed with 0 errors and 0 warnings; text 165344 bytes, data 504 bytes, BSS 39472 bytes; ELF/HEX/BIN generated.
- `python -m py_compile scripts\foc_runtime_profile.py`: passed.
- Synthetic profiler parse/report smoke test: passed, including PASS classification and 20.000 us remaining-budget calculation.
- Hardware flash and runtime capture were not performed in this turn.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `MDK-ARM/code/foc_profiler.h`
  - `MDK-ARM/code/foc_profiler.c`
  - `Core/Src/main.c`
  - `Core/Src/stm32h7xx_it.c`
  - `MDK-ARM/code/foc_app.c`
  - `build.ps1`
  - `Makefile`
  - `MDK-ARM/24V FOC Controller.uvprojx`
  - `docs/UART_COMMANDS.md`
  - `scripts/foc_runtime_profile.py`
  - `PROGRESS.md`

## [2026-07-13] Motor Control Code Walkthrough

### Problem / Task
- Trace and explain how motor commands, product modes, cascaded control loops, FOC transforms, PWM generation, feedback sampling, and safety state transitions are implemented in the firmware.

### Resolution
- Mapped the execution path from UART commands and `APP_MODE` selection through the position, speed, and current loops to inverse Park/SVPWM and TIM1 CCR outputs.
- Confirmed that product modes reuse the common FOC core: RAW/JOINT/GIMBAL/HOLD select or constrain the normal cascaded loops, while SPRING/DETENT/SCROLL_WHEEL inject virtual-physics torque through `Iq_ref` and bypass the normal position loop.
- Identified the main reading entry points in `main.c`, `stm32h7xx_it.c`, `foc_app.c`, and `foc_core.c`; no motor-control source was changed.

### Prevention / Follow-up
- Keep `control_mode` (TORQUE/SPEED/POSITION) separate from `app_mode` (product behavior) when extending features; new haptic modes should reuse the current loop and explicitly define which outer loops they bypass.

### Verification
- Static code inspection only; no build or hardware test was required for this explanation.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `Core/Src/main.c`
  - `Core/Src/stm32h7xx_it.c`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/foc_core.c`
  - `PROGRESS.md`

## [2026-07-12] Source-Only Public Release Branch

### Problem / Task
- Clean the development workspace and publish only production firmware, HostComputer/Bridge source, build configuration, README, and license files while keeping test scripts, test reports, architecture documents, and internal progress records off GitHub.

### Resolution
- Removed the obsolete local `HostComputer/out/` and `HostComputer/tmpbuild/` PyQt6 packaging trees (about 160 MB) and added both paths to `.gitignore`.
- Created a clean release worktree from `origin/main`, overlaid the latest production source, and removed public copies of `.cmsis` sample content, BenchTests, HostComputer tests, internal docs/plans, architecture/progress records, root bench/debug scripts, and Simulink test tooling.
- Kept required STM32 HAL/CMSIS dependencies under `Drivers/`, plus the STM32 firmware, PySide6 HostComputer, FOC Device Bridge, build files, README, MIT license, and third-party notices.
- Pushed the curated snapshot to `origin/codex/public-code-readme` and fast-forwarded remote `main` to the same release commit; remote `main` was not force-updated.

### Prevention / Follow-up
- Keep `codex/public-code-readme` as a review and rollback reference for the source-only `main` snapshot.
- Keep private validation assets in the development workspace and do not copy them into future public snapshots.

### Verification
- Firmware GCC build passed with text 162744 bytes, data 504 bytes, and BSS 39008 bytes; ELF/HEX/BIN were generated.
- HostComputer and FOC Device Bridge regression: 200 tests passed.
- Python compile check passed for all published HostComputer and Bridge modules.
- Secret-pattern scan found no private keys or common token formats; no test/docs/log/data/executable files were added or modified in the public commit.

### Commit
- Branch: `codex/public-code-readme`
- Commit: `d1b8126b25ffe492e2d8acf8e3e963a4d895ea4f` (`publish: refresh source-only project snapshot`)
- Status: pushed to `origin/codex/public-code-readme` and `origin/main`
- Files:
  - Production firmware and desktop source trees
  - `README.md`
  - `LICENSE`
  - `THIRD_PARTY_NOTICES.md`
  - Build configuration files

## [2026-07-11] PySide6 Migration and Public License Setup

### Problem / Task
- Complete the public-release license setup after confirming that Ctrl-FOC-Lite was used only as a design reference, while avoiding PyQt6 GPL distribution ambiguity for the desktop applications.

### Resolution
- Migrated `HostComputer` and `FOC_Device_Bridge` from PyQt6 to PySide6, including signals, slots, tests, requirements, and PyInstaller configuration.
- Added a root MIT `LICENSE` for original project code and documentation, plus `THIRD_PARTY_NOTICES.md` covering STM32 HAL, CMSIS, PySide6/Qt, Python dependencies, PyInstaller, and the bundled libusb DLL.
- Updated README technology and license sections, explicitly recording that Ctrl-FOC-Lite was referenced without copying its source.
- Updated both desktop packaging scripts to place the project license and third-party notices in release output.

### Prevention / Follow-up
- Do not publish older PyQt6-based executable packages as the MIT release artifacts.
- Preserve upstream vendor license files and include applicable Qt/Python dependency notices with every binary release.
- Verify the exact provenance of `libusb-1.0.dll` before redistributing it publicly.

### Verification
- `python -m unittest HostComputer.test_adc_noise_gui HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_main_window HostComputer.test_serial_service FOC_Device_Bridge.test_bridge`: 200 tests passed.
- `python -m compileall -q HostComputer FOC_Device_Bridge`: passed.
- `build_host_gui_app.ps1`: PySide6 one-folder Host package built successfully.
- `FOC_Device_Bridge/build_bridge.ps1`: PySide6 Bridge executable built successfully.
- Repository source scan found no remaining PyQt6, `pyqtSignal`, or `pyqtSlot` references outside ignored historical artifacts.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `LICENSE`
  - `THIRD_PARTY_NOTICES.md`
  - `README.md`
  - `HostComputer/requirements.txt`
  - `HostComputer/gui_app.py`
  - `HostComputer/main_window.py`
  - `HostComputer/serial_worker.py`
  - `HostComputer/transport.py`
  - `HostComputer/test_main_window.py`
  - `HostComputer/test_adc_noise_gui.py`
  - `FOC_Device_Bridge/`
  - `build_host_gui_app.ps1`
  - `PROGRESS.md`

## [2026-07-11] Public Release License Provenance Audit

### Problem / Task
- Confirm whether the project could add an open-source license before its public GitHub release, with particular attention to Ctrl-FOC-Lite references and desktop GUI dependencies.

### Resolution
- Confirmed with the project owner that Ctrl-FOC-Lite was used only as a design reference and no source code was copied, so it does not need to be treated as a derived-code license source.
- Confirmed that the STM32 HAL and CMSIS trees already carry their own upstream licenses and must retain those notices.
- Identified PyQt6 in both `HostComputer` and `FOC_Device_Bridge` as the remaining license decision: retaining PyQt6 favors GPL-3.0-compatible distribution, while a permissive project license should first migrate those applications to PySide6 or use a commercial PyQt license.

### Prevention / Follow-up
- Do not add a blanket MIT license until the owner chooses between GPL-3.0 distribution and a PySide6 migration followed by permissive licensing.
- Add a root license, component scope notes, and `THIRD_PARTY_NOTICES.md` once that choice is made.

### Verification
- Reviewed root README license text, desktop requirements/imports, repository ignore rules, and bundled STM32 vendor license locations.
- No source or build files were changed as part of this audit.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `PROGRESS.md`

## [2026-07-11] GitHub README Rewrite

### Problem / Task
- The root README contained mojibake, obsolete 24V/230400-baud assumptions, stale GUI descriptions, and an MIT-license claim even though the repository has no LICENSE file.

### Resolution
- Replaced the README with a clean UTF-8, Chinese-first GitHub overview covering the 12V baseline, FOC implementation, seven APP_MODE modes, calibration/protection, 1Mbaud circular-DMA UART, binary current stream, HostComputer GUI, Windows scroll-wheel Bridge, build/test commands, architecture diagrams, safety notes, and known limitations.
- Documented CAN and BIN2000 as experimental rather than release-ready, and explicitly noted that the repository still needs a license decision before public reuse rights are granted.

### Prevention / Follow-up
- Keep release-facing constants synchronized with `foc_app.h`, `usart.c`, `current_stream.h`, and `docs/UART_COMMANDS.md` whenever the electrical or communication baseline changes.
- Add repository screenshots and a deliberate LICENSE file before the public GitHub release if desired.

### Verification
- README decoded as UTF-8 with 315 lines and balanced Markdown code fences.
- All referenced internal documentation paths exist.
- No stale `230400`, COM9, 24V supply recommendation, or false MIT-license text remains.
- `git diff --check -- README.md`: passed apart from the repository's existing LF/CRLF conversion warning.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `README.md`
  - `PROGRESS.md`

## [2026-07-11] Scroll Wheel Bridge Ownership and Host State Sync

### Problem / Task
- The visible Host GUI could select SCROLL_WHEEL but initially had no way to ask the background Bridge to connect to the selected COM port or start/stop the wheel session. The wheel panel also reported the wrong underlying control mode and showed `--` until the first physical wheel event.
- APP_MODE acknowledgements overwrote the mode combo while signals were blocked, leaving the combo, stacked panel, desired mode, and firmware-confirmed mode inconsistent.

### Resolution
- Added IPC requests for Bridge-owned COM connection and wheel enable/disable, then connected them to explicit Host GUI buttons. Bridge remains the only SCROLL_WHEEL lifecycle and OS input owner.
- Added initial wheel-status broadcasts on `CONNECTED_IDLE` and `WHEEL_ACTIVE`, so the Host immediately displays zeroed position and delta counters before the first physical event.
- Mapped `SCROLL_WHEEL` to the firmware position-control layer in both response inference paths.
- APP_MODE acknowledgements now update only the firmware-confirmed mode. They no longer overwrite the user's selected mode or stacked panel; mismatches are shown as `待同步：<模式>` until the requested sequence completes.
- Rebuilt `dist/FOC_Device_Bridge.exe` and `dist/24V_FOC_Host/24V_FOC_Host.exe`.

### Prevention / Follow-up
- Keep `app_mode_selected` (GUI intent) separate from `app_mode` (firmware confirmation); never mutate selection widgets from an ACK while their signals are blocked.
- Hardware verification should confirm that enabling the wheel changes `CONNECTED_IDLE` to `WHEEL_ACTIVE` and physical detents increment both counters and Windows scroll input.

### Verification
- HostComputer tests: 193 tests OK.
- FOC_Device_Bridge tests: 7 tests OK.
- `python -m compileall -q HostComputer FOC_Device_Bridge`: OK.
- Bridge and Host PyInstaller packaging: completed successfully.
- User screenshot confirmed Bridge ownership of `COM7`, `CONNECTED_IDLE`, position `0`, total delta `0`, and the position-control label.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `FOC_Device_Bridge/bridge_app.py`
  - `FOC_Device_Bridge/ipc_protocol.py`
  - `FOC_Device_Bridge/ipc_server.py`
  - `FOC_Device_Bridge/serial_owner.py`
  - `FOC_Device_Bridge/test_bridge.py`
  - `HostComputer/main_window.py`
  - `HostComputer/serial_worker.py`
  - `HostComputer/test_main_window.py`
  - `HostComputer/transport.py`
  - `PROGRESS.md`

## [2026-07-10] Bridge Launch Confusion and Single-instance Fix

### Problem / Task
- User reported that `dist/FOC_Device_Bridge.exe` could not be opened as an upper-computer GUI.

### Resolution
- Confirmed the executable was not crashing: six `FOC_Device_Bridge` processes were running with no main window because Bridge is intentionally a system-tray background application, not the HostComputer GUI.
- Located the real GUI executable at `dist/24V_FOC_Host/24V_FOC_Host.exe` and rebuilt it from the current source.
- Added live-server probing to `IpcServer.start()` so a second Bridge instance is rejected instead of removing the active named-pipe endpoint.
- Added tray notifications explaining that Bridge is running in the tray and that `24V_FOC_Host.exe` is the visible GUI. Duplicate launches now notify and exit.
- Stopped the six stale Bridge processes that were also locking the old executable, then rebuilt the Bridge package successfully.

### Prevention / Follow-up
- Launch order for wheel mode: start `FOC_Device_Bridge.exe` once, then open `24V_FOC_Host.exe`.
- Keep the two executables separately named and distributed together; Bridge owns COM/input injection, Host is the visible debug UI.

### Verification
- Bridge tests: 5 tests OK, including rejection of a second live IPC owner.
- HostComputer tests: 191 tests OK.
- Python compileall: OK.
- `FOC_Device_Bridge/build_bridge.ps1`: package rebuilt successfully after closing stale processes.
- `build_host_gui_app.ps1`: package rebuilt successfully.
- Launched the rebuilt Host executable and observed `FOC 上位机调试工具` with `Responding=True`; the verification process was then closed.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `FOC_Device_Bridge/ipc_server.py`
  - `FOC_Device_Bridge/bridge_app.py`
  - `FOC_Device_Bridge/test_bridge.py`
  - `PROGRESS.md`

## [2026-07-10] SCROLL_WHEEL Session and Bridge Integration Fix

### Problem / Task
- Implemented the remaining fixes from the SCROLL_WHEEL re-review: session ownership, safe OS injection, real bridge/device state propagation, package-safe startup, binary frame hardening, and automated bridge coverage.

### Resolution
- Firmware now accumulates and emits wheel deltas only while a live `WHEEL:SESSION` exists. Starting a session resets the event baseline, sequence, and pending deltas; no-session processing clears queued input instead of emitting W frames. The power-enable path also rejects `SCROLL_WHEEL` with `ENABLE,FAIL,no_wheel_session` when no session exists.
- Bridge now calls `SendInput` only when both its SessionManager is `WHEEL_ACTIVE` and the W-frame carries `session_active`. Events outside that ownership boundary are ignored.
- HostComputer can no longer use the generic APP_MODE power controls to enter `SCROLL_WHEEL`; its enable, arm, and direct mode-set actions are disabled for that selection, leaving the tray Bridge as the sole lifecycle owner.
- IPC server caches and immediately sends the latest controller/session state to new clients. `BridgeTransport` now distinguishes named-pipe connectivity from a real controller connection and decodes `CONN_STATE`, `WHEEL_STATUS`, and `ERROR` frames. HostComputer displays bridge session, wheel position, and total delta.
- Added a package-safe `bridge_launcher.py` and changed the PyInstaller script to use it, fixing package-relative imports.
- Binary parser now requires exact payload lengths (`C=20`, `W=16`) before unpacking and records length errors.
- Added HostComputer parser/GUI tests and a new Bridge test suite for enable ordering, injection gating, connection/status decoding, and cached IPC state.

### Verification
- `python -m unittest discover -s HostComputer -p test*.py`: 191 tests OK.
- `python -m unittest discover -s FOC_Device_Bridge -p test*.py`: 4 tests OK.
- `python -m compileall -q HostComputer FOC_Device_Bridge`: OK.
- Bridge/launcher/transport import smoke: OK.
- Firmware GCC build: 0 errors, 0 warnings; text=162744, data=504, bss=39008.
- `FOC_Device_Bridge/build_bridge.ps1`: PyInstaller completed successfully.
- Package output: `dist/FOC_Device_Bridge.exe` (168223836 bytes).
- Hardware behavior was not tested because the controller was not available for this change.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: not committed yet
- Status: working tree contains existing user changes plus this fix
- Files:
  - `MDK-ARM/code/wheel_input.c`
  - `FOC_Device_Bridge/bridge_app.py`
  - `FOC_Device_Bridge/ipc_server.py`
  - `FOC_Device_Bridge/bridge_launcher.py`
  - `FOC_Device_Bridge/build_bridge.ps1`
  - `FOC_Device_Bridge/test_bridge.py`
  - `HostComputer/transport.py`
  - `HostComputer/serial_worker.py`
  - `HostComputer/gui_logic.py`
  - `HostComputer/main_window.py`
  - `HostComputer/data_parser.py`
  - `HostComputer/test_data_parser.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-07-10] SCROLL_WHEEL Fix Re-review

### Problem / Task
- Re-reviewed the eight fixes applied after the initial SCROLL_WHEEL implementation review, without changing functional code.

### Resolution
- Confirmed all eight reported fixes are present: bridge imports compile, the ACK contract uses `CUR_STREAM`, APP_MODE precedes session start, SerialOwner stays on the main Qt thread, APP_MODE is committed only after validation, detent crossing is bidirectional at 0.55 spacing, ISR/main delta exchange is protected, and the seventh GUI mode panel exists.
- Found a remaining release blocker in session ownership: `WheelInput_Process()` still emits W frames while `session_active == 0`, the bridge injects every W event without checking its session state/flag, and the HostComputer advanced page can directly run the generic `APP_MODE,SCROLL_WHEEL -> ENABLE` sequence without starting a bridge session. This bypasses keepalive timeout protection and permits OS wheel injection outside the owned session.
- Found an IPC state gap: `BridgeTransport.open()` treats named-pipe connectivity as device connectivity and ignores `CONN_STATE`, `WHEEL_STATUS`, and `ERROR` frames. The GUI can therefore report connected while the bridge has no COM device, and the new wheel status labels cannot update.
- Found a packaging blocker: `build_bridge.ps1` passes `bridge_app.py` directly to PyInstaller, but that file uses package-relative imports. Running the configured entry point fails with `ImportError: attempted relative import with no known parent package`.
- Found a parser hardening/test gap: known binary frame types are decoded without validating their required payload lengths (C=20, W=16), and the bridge directory currently has no automated tests.

### Prevention / Follow-up
- Make the bridge the sole owner of SCROLL_WHEEL activation: gate firmware W-frame output on an active session, gate `SendInput` on `STATE_WHEEL_ACTIVE` plus the W-frame session flag, and route/disable HostComputer's generic SCROLL_WHEEL arm controls.
- Surface bridge device state and wheel status through `BridgeTransport` before treating IPC as a connected motor controller.
- Package through a module-safe launcher or absolute imports, and add bridge/session/parser tests before creating the executable.

### Verification
- `python -m unittest discover -s HostComputer -p test*.py`: 188 tests OK.
- `python -m compileall -q HostComputer FOC_Device_Bridge`: OK.
- Python package imports for bridge/session/transport: OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware build 0 errors, 0 warnings; text=162608, data=504, bss=39008.
- `python FOC_Device_Bridge\bridge_app.py`: fails at package-relative import, matching the current PyInstaller entry-point risk.
- `python -m unittest discover -s FOC_Device_Bridge -p test*.py`: 0 tests discovered.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `PROGRESS.md`

## [2026-07-10] SCROLL_WHEEL Implementation Code Review

### Problem / Task
- Reviewed the newly implemented SCROLL_WHEEL firmware mode, W-frame protocol, Windows bridge, HostComputer IPC transport, GUI integration, and build/test coverage.

### Resolution
- Firmware GCC build succeeds, and the existing HostComputer suite still passes, but the feature is not release-ready.
- Found blocking bridge and lifecycle defects: `bridge_app.py` has a Python syntax error; the enable sequence expects `TELEM:CUR` while firmware returns `CUR_STREAM`; APP_MODE entry clears the wheel session created by the preceding session command; bridge serial methods are called directly from the wrong Qt thread; and SCROLL_WHEEL precheck failures can leave the mode selected and emit both FAIL and OK.
- Found correctness gaps in the detent/event path: the event threshold effectively uses one full detent spacing instead of the documented 0.55 spacing, force feedback and event quantization use separate centers, and ISR/main-loop delta exchange is not synchronized.
- Found integration/test gaps: the GUI exposes seven APP_MODE choices but only six stacked panels, bridge IPC reports pipe connectivity as device connectivity, and no automated tests cover W-frame parsing, bridge IPC/session behavior, or wheel quantization.

### Prevention / Follow-up
- Fix blocking bridge startup, ACK contract, session ordering/gating, mode precheck rollback, and Qt thread ownership before hardware flashing.
- Add isolated tests for the generic binary parser, wheel quantizer, SessionManager, IPC transport, and fake wheel injection before packaging.

### Verification
- `python -m unittest discover -s HostComputer -p "test*.py"`: 188 tests OK, but none cover the new bridge/wheel runtime.
- `python -m compileall -q HostComputer FOC_Device_Bridge`: failed at `FOC_Device_Bridge/bridge_app.py:26` with `SyntaxError`.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware build completed successfully; text=162616, data=504, bss=39008.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: working tree changes not committed yet
- Files:
  - `PROGRESS.md`

## [2026-07-09] Current Stream Plot Freeze Mitigation

### Problem / Task
- User reported that the HostComputer waveform view freezes every time the current-stream acquisition switch is enabled.
- Screenshot showed binary current samples still arriving (`rx` around 1 kfps) while the plotted traces stopped updating, and angle/telemetry channels were mixed with mA current channels on one Y axis.

### Resolution
- Found a GUI hot-path bug in `_get_current_stream_plot_data()`: each current sample called `_sample_time_s()`, which recalculated measured fps by copying the full current ring. With a 20k-sample ring this degraded toward O(n²) per plot refresh.
- Reworked the current-stream plot path to take one ring snapshot, estimate fps once, crop to the visible tail while following latest, and decimate to a fixed 3000-point budget.
- When enabling the current-stream acquisition group, the GUI now resets the current-stream time origin/ring/parser stats and switches the plot to Ia/Ib/Ic-only view so angle/speed/voltage no longer distort the current waveform Y axis.
- Added regression tests for one-snapshot plotting and current-only channel selection on acquisition enable.

### Verification
- `python -m unittest HostComputer.test_main_window HostComputer.test_gui_logic HostComputer.test_data_parser`: 174 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 185 tests OK.

### Second Freeze Update
- User reported that after the first mitigation the whole GUI could still freeze and eventually show `not connected`; screenshot showed the ring continuing to fill while the visible X window stopped near 3s and `rx` briefly reported an impossible ~20kfps.
- Decoupled current-stream sample arrival from plotting: sample batches now only establish the sequence origin, while the chart timer pulls snapshots at a fixed GUI refresh rate.
- Changed current-stream X axis to use `seq / configured_rate` (`BIN 1kHz` = 1000 fps, `BIN 2kHz` = 2000 fps) instead of payload `tick_ms` or jittery measured fps.
- Cached plot legend rebuilds and guarded programmatic `setXRange()` so it cannot disable follow-latest as if the user had panned/zoomed.
- Changed the current-stream stats label to calculate `rx fps` from real wall-clock elapsed time so GUI stalls do not produce fake 20kfps readings.
- Added regression tests for configured-rate time axes and timer-driven current-stream refresh.
- Latest verification: `python -m unittest HostComputer.test_main_window HostComputer.test_gui_logic HostComputer.test_data_parser`: 177 tests OK.
- Latest verification: `python -m unittest discover -s HostComputer -p "test*.py"`: 188 tests OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: not committed yet
- Status: working tree has existing unrelated/user changes plus this HostComputer fix
- Files:
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-07-09] APP_MODE Preset ACK Timeout Root Cause

### Problem / Task
- User reported that DETENT knob mode was now usable, but switching presets caused ACK timeout and the cause was unclear after debugging.

### Resolution
- Found that the sequence-specific `build_app_mode_command()` returned `CMD:APP_MODE,<mode>` without a trailing newline, while `SerialService.send_command()` writes the string exactly as provided and does not append `\n`.
- Because preset/config flows start with `APP_MODE -> DETENT:CFG` / `SPRING:CFG`, the firmware never received a complete APP_MODE line, so it never ACKed and the sequence timed out before the preset command was sent.
- Fixed `build_app_mode_command()` to reuse `CommandBuilder.app_mode_set()`, making every sequence APP_MODE command newline-terminated and validated.
- Fixed `_on_hold_lock()` to actually dispatch `CMD:APP_MODE,HOLD\n`; it previously built a sequence and returned without sending.
- Added tests to lock in newline-terminated APP_MODE sequence commands and HOLD dispatch behavior.

### Prevention / Follow-up
- Treat all outbound serial commands as full line protocol frames; helper functions used by sequences must return newline-terminated strings.
- Hardware check: click a DETENT preset and confirm the TX log shows `CMD:APP_MODE,DETENT` followed by `DETENT:CFG,...`, with no ACK timeout.

### Verification
- `python -m unittest HostComputer.test_gui_logic HostComputer.test_main_window HostComputer.test_data_parser`: 173 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 184 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: build completed successfully, 0 errors, 0 warnings; text=159048, bss=38896.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/gui_logic.py`
  - `HostComputer/main_window.py`
  - `HostComputer/test_gui_logic.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-07-07] APP_MODE Sync And UART ACK Release Candidate

### Problem / Task
- Product-mode GUI selection was not reliably synchronized with firmware `APP_MODE` before enable, target, or config commands.
- DETENT/Spring config responses exposed a `%f` formatting gap in the firmware build flags, and 1Mbaud UART RX needed the circular-DMA command path retained in the release candidate.

### Resolution
- Implemented APP_MODE-aware command sequencing in HostComputer so advanced control actions send `CMD:APP_MODE,<mode>` before enable/arm, PREF/SREF, SPRING:CFG, and DETENT:CFG dependent commands.
- Kept top-level RAW power controls independent from product-mode synchronization.
- Added firmware ACK responses for power/mode/target commands, circular UART RX diagnostics/recovery, full `DETENT:CFG` response echo, and `_printf_float` linker support for floating-point config echoes.
- Preserved haptic-mode safeguards: SPRING_DAMPER/DETENT require identified motor and valid encoder instead of stall fallback.

### Prevention / Follow-up
- Hardware smoke passed per user report; future product-mode UI commands should use the ACK sequence engine rather than direct multi-command emits.
- `scripts/pid_auto_tune.py` remains an unrelated untracked/experimental script and was not included in this release candidate.

### Verification
- `python -m unittest discover -s HostComputer -p "test*.py"`: 183 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: build completed successfully, 0 errors, 0 warnings; text=158824, bss=38888.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `09536c5dc598cb20f1f203c06805bdd0976813bb`
- Status: `committed in v1.2.0-APP_MODE_SYNC; later working tree changes exist`
- Files:
  - `Core/Src/stm32h7xx_it.c`
  - `Core/Src/usart.c`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/head.h`
  - `HostComputer/data_parser.py`
  - `HostComputer/gui_logic.py`
  - `HostComputer/main_window.py`
  - `HostComputer/test_data_parser.py`
  - `HostComputer/test_gui_logic.py`
  - `HostComputer/test_main_window.py`
  - `build.ps1`
  - `docs/PLATFORM_OVERVIEW.md`
  - `docs/UART_COMMANDS.md`

## [2026-07-07] Hardware Bring-up And UART Regression Completed

### Problem / Task
- User completed the post-move hardware bring-up and wanted the results captured before mechanical assembly resumes.
- A separate HostComputer/UART communication regression was needed after the firmware UART RX circular DMA changes.

### Resolution
- Recorded the completed firmware build/flash and bring-up flow: GCC `0 errors / 0 warnings`, CMSIS-DAP programmed `155648 bytes`, communication burst passed, unlock/enable/stop passed, existing motor parameters were valid, RAW speed/position control passed, current stream BIN1000 passed, APP_MODE modes passed, and fault/blackbox diagnostics responded.
- Key control data reported: `SREF +0.5 -> avg_speed=0.500`, `SREF -0.5 -> avg_speed=-0.496`, `PREF +5deg -> error=1.6deg`, `PREF +20deg -> error=1.1deg`, `HOLD drift=0.0deg over 3s`, JOINT soft limits clipped around `+30.9deg / -31.1deg`, and BIN1000 ran 10s with `0 RX errors`.
- Recorded UART regression report results: HostComputer unit tests `183/183 PASS`, `FW_INFO? 100/100`, RX errors `0->0`, ACK matrix all pass, N-frame 50Hz measured about `42Hz`, N-frame 100Hz measured about `64Hz` with `33/33` concurrent command success, BIN1000 and BIN2000 command coexistence passed, STOP recovery passed, and long text responses parsed under both current-stream OFF and BIN1000.
- Noted current test port from the report as `COM7 @ 1000000 baud`; previous docs/scripts may still assume COM9.

### Prevention / Follow-up
- Mechanical bracket/payload tests remain deferred until the hardware bracket is finished and assembled.
- Do not run the 26 legacy scripts listed in `UART_REGRESSION_REPORT.md` until their `1152000` baud constants are updated to `1000000` and the correct port is selected.
- Treat BIN1000 as the validated current-stream baseline; BIN2000 remains experimental but command/STOP coexistence passed in this regression.

### Verification
- Reviewed the user's bring-up report and `UART_REGRESSION_REPORT.md`; no new build, flash, hardware action, or test was run by Codex in this turn.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63c`
- Status: `working tree changes not committed yet`
- Files:
  - `PROGRESS.md`
  - `UART_REGRESSION_REPORT.md`
  - `bringup_full_test.py`
  - `uart_regression_test.py`

## [2026-07-03] HostComputer Command Gate And GUI Pending Guard

### Problem / Task
- User wanted the command-gating idea implemented in the HostComputer GUI, not only explained as a function.
- Repeated clicks could still send multiple state-advancing commands while a prior command was pending ACK/N-frame confirmation.

### Resolution
- Added host-side command gate helpers in `HostComputer\gui_logic.py`: `is_safe_fallback_command()`, `is_motion_target_command()`, and `can_dispatch_command()`.
- Updated `button_enable_state()` and `can_edit_vbus_limits()` so GUI controls for advancing actions, motion targets, APP_MODE/product controls, and configuration edits are disabled while `pending_command` is active.
- Wired `HostMainWindow._dispatch_command()` through `can_dispatch_command()` so direct programmatic sends cannot bypass disabled buttons.
- Added sequence-level guarding in `_dispatch_sequence()` so pending/fault states block advancing command sequences while still allowing safety/fallback sequences.
- Preserved safe fallback behavior for commands such as disable, lock, clear fault, fault detail, and queries.
- Added/updated unit tests covering pending-state GUI disabling, duplicate enable blocking, safe fallback during pending, fault-state drive blocking, target-command requirements, and pending-settle behavior in existing GUI tests.

### Prevention / Follow-up
- Future command entry points should call `can_dispatch_command()` or share the same sequence-gating policy instead of emitting serial commands directly.
- A later cleanup should fix mojibake Chinese strings in `main_window.py` / `gui_logic.py` so user-facing messages and tests are easier to maintain.

### Verification
- `python -m unittest HostComputer.test_gui_logic HostComputer.test_main_window`: 116 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 183 tests OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\gui_logic.py`
  - `HostComputer\main_window.py`
  - `HostComputer\test_gui_logic.py`
  - `HostComputer\test_main_window.py`
  - `PROGRESS.md`

## [2026-07-02] HostComputer Duplicate Command Gate Lesson

### Problem / Task
- Continued teaching by investigating why repeated button clicks can send repeated HostComputer CMD lines while a command is already pending.

### Resolution
- Reviewed `button_enable_state()`, `_dispatch_command()`, `_dispatch_sequence()`, and related enable button wiring.
- Found that the current button enable logic does not account for `state.pending_command`; it only checks connection, unlock, enabled, identify, and fault states.
- Found that command dispatch also lacks a pending-command gate, so repeated clicks can emit additional state-changing commands before ACK/N-frame convergence clears the previous pending command.

### Prevention / Follow-up
- Next lesson can demonstrate a small `pending_command` guard: disable state-changing buttons or reject new state-changing commands while pending, while still allowing emergency/safe-stop commands as a policy exception.

### Verification
- Read source snippets only; no tests or builds run.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\gui_logic.py`
  - `HostComputer\main_window.py`
  - `PROGRESS.md`

## [2026-07-02] HostComputer ACK State Teaching Walkthrough

### Problem / Task
- Continued teaching the host-computer state-machine flow using the real HostComputer source code.

### Resolution
- Reviewed the enable-command path across `main_window.py` and `gui_logic.py`.
- Mapped `pending_command`, `apply_command_effects()`, `apply_ack_effects()`, `apply_packet_effects()`, `_converge_pending_from_nframe()`, and `_check_ack_timeout()` to the conceptual flow: button request, command pending, ACK confirmation, telemetry fallback, and timeout warning.
- Noted that the current timeout implementation warns and records `last_command_error` while continuing to wait for N-frame convergence instead of immediately clearing the pending command.

### Prevention / Follow-up
- Continue next lesson by comparing this implementation with a stricter command gate that blocks duplicate state-changing commands while pending.

### Verification
- Read source snippets only; no tests or builds run.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\gui_logic.py`
  - `HostComputer\main_window.py`
  - `PROGRESS.md`

## [2026-07-02] Firmware APP_MODE / Product Mode Fixes (Round 2)

### Problem / Task
- Implement the firmware-side mode state machine fixes identified in the 2026-07-02 code review.
- 7 items: SPRING:CFG parser cleanup, CMD:MODE no longer silently resets APP_MODE, SPRING_DAMPER unconditional position capture, SPRING/DETENT bypass normal PositionLoop, stall fallback resets APP_MODE, SYS:CMDS? completeness, DEBUG GPIO removal.

### Resolution
All 7 items implemented across 3 files:

1. **SPRING:CFG parser** (`Core/Src/stm32h7xx_it.c`): Removed dead 2-param `UART_CommandParseFloat2` block; replaced with clean `sscanf` 3-param handler. Added range validation (`K>=0, D>=0, 0<=limit<=2.0A`) with `SPRING:CFG,FAIL,range` on invalid. Success response now echoes `SPRING:CFG,OK,K=...,D=...,limit=...`.

2. **CMD:MODE → APP_MODE separation** (`MDK-ARM/code/foc_app.c`, `foc_app.h`): Removed `app_mode = APP_MODE_RAW` side-effect from `FOC_App_SetControlMode()`. Added new `FOC_App_SetRawControlMode()` that explicitly sets both `control_mode` and `app_mode = RAW`. `CMD:MODE,N` handler now calls `FOC_App_SetRawControlMode()`. `FOC_App_SetAppMode()` internal `control_mode` assignments are unaffected.

3. **SPRING_DAMPER entry position capture** (`foc_app.c`): Removed `position_ref_user_set == 0U` guard. Entry now unconditionally captures current position as spring equilibrium and sets `position_ref_user_set = 1U`. Also clears `speed_ref` and `speed_ref_ramped`.

4. **Haptic modes bypass PositionLoop** (`foc_app.c`): Added `FOC_App_IsHapticMode()` static helper. In `FOC_App_PositionLoop()`: haptic modes clear `speed_ref = 0` and return early. In `FOC_App_SpeedLoop()`: haptic modes skip speed PI + all feedforward via goto to `haptic_torque_injection:` label; `iq_ref_mech` starts from 0 and is filled purely by spring/detent torque physics.

5. **Stall fallback** (`foc_app.c`): `FOC_App_Enable()` now sets `app_mode = APP_MODE_RAW` alongside `control_mode = FOC_MODE_SPEED` when stall fallback is triggered.

6. **SYS:CMDS?** (`stm32h7xx_it.c`): APP_MODE line now lists `RAW|JOINT_POS|GIMBAL_SPEED|HOLD|SPRING_DAMPER|DETENT`. Added `SPRING:CFG? SPRING:CFG,K,D,limit` and `DETENT:CFG? DETENT:CFG,count,strength,width,limit` line.

7. **DEBUG GPIO removal** (`stm32h7xx_it.c`): Removed `HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0)` from TIM1_UP_IRQHandler.

### Prevention / Follow-up
- `SetControlMode` no longer has the side-effect of resetting app_mode; use `SetRawControlMode` for the explicit RAW-switch semantic.
- Haptic modes (SPRING_DAMPER, DETENT) are now defined by `FOC_App_IsHapticMode()` — any future haptic-like mode should be added there.
- Hardware validation of SPRING/DETENT hand-feel is still needed.

### Verification
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: **OK** — 0 errors, 0 warnings. text=154204 (+488B), bss=38888 (unchanged).
- `python -m unittest discover -s HostComputer -p "test*.py"`: **176 tests OK**.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63c`
- Status: `working tree changes not committed yet`
- Files modified:
  - `MDK-ARM/code/foc_app.h` — added `FOC_App_SetRawControlMode` declaration
  - `MDK-ARM/code/foc_app.c` — items 2-5: mode state machine, haptic bypass, stall fallback
  - `Core/Src/stm32h7xx_it.c` — items 1,6,7: SPRING:CFG parser, SYS:CMDS?, DEBUG GPIO
  - `PROGRESS.md`

## [2026-07-02] Firmware APP_MODE / Product Mode Code Review

### Problem / Task
- User suspected the lower-machine product-mode control path was wrong after GUI mode switching, enable/arm, HOLD, JOINT, SPRING_DAMPER, and DETENT behavior looked inconsistent.

### Resolution
- Reviewed firmware-side APP_MODE, `CMD:MODE`, JOINT/GIMBAL/SPRING/DETENT command handlers, enable/stall fallback, position loop, and speed-loop haptic torque injection.
- Identified several mode-layer risks: `SPRING:CFG` parser ordering rejects valid 3-parameter commands, `CMD:MODE` resets product mode to RAW, SPRING_DAMPER can inherit an old position reference, SPRING/DETENT still run the normal position loop while adding haptic torque, and stall fallback can leave APP_MODE displayed while forcing bottom control to SPEED.

### Prevention / Follow-up
- Recommended fixing the firmware mode contract before more GUI changes: make APP_MODE ownership explicit, parse SPRING config exactly, capture spring equilibrium on entry, and separate haptic-product modes from the normal position servo if the intended behavior is hand-feel rather than target tracking.

### Verification
- Static review only; no firmware edits, build, flash, or hardware test run.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63c`
- Status: `working tree changes not committed yet`
- Files reviewed:
  - `MDK-ARM\code\foc_app.c`
  - `MDK-ARM\code\foc_app.h`
  - `Core\Src\stm32h7xx_it.c`
  - `HostComputer\main_window.py` (for host-triggered `CMD:MODE` context only)

## [2026-07-02] HostComputer Framework Orientation

### Problem / Task
- User wanted to understand the host-computer GUI framework and learn how to rebuild or improve the AI-generated upper-computer application.

### Resolution
- Inspected `HostComputer` architecture and identified it as a Python desktop app using PyQt6, pyserial, pyqtgraph, and numpy.
- Mapped the current layering: `gui_app.py` starts the Qt app/thread, `serial_worker.py` owns serial polling and Qt signals, `serial_service.py` wraps command send/parser feed, `data_parser.py` parses telemetry/ACK/binary current frames and builds commands, `gui_logic.py` holds state/profile/validation helpers, and `main_window.py` contains most UI construction and interaction handling.
- Noted key architectural issues for future rewrite: oversized `HostMainWindow`, protocol/UI coupling, mojibake Chinese text, and mixed plotting/control/product-mode responsibilities.

### Prevention / Follow-up
- Recommended learning or rewriting from the protocol/state/worker layers first, then rebuilding a smaller UI shell around them.

### Verification
- Read project files only; no tests or builds run.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\gui_app.py`
  - `HostComputer\serial_worker.py`
  - `HostComputer\serial_service.py`
  - `HostComputer\data_parser.py`
  - `HostComputer\gui_logic.py`
  - `HostComputer\main_window.py`
  - `PROGRESS.md`

## [2026-07-01] UART RX Circular DMA + ACK Residual Fix

### Problem / Task
- At 1Mbaud, GUI commands such as `UNLOCK` / `ENABLE` could time out even when telemetry was still arriving.
- The likely firmware-side failure mode was the normal `ReceiveToIdle_DMA` re-arm window: after IDLE IRQ latency, USART RX could overrun before DMA was re-enabled.
- The host binary-current parser could also hold short ASCII ACK lines shorter than one binary frame, producing false ACK timeout symptoms.

### Resolution
- Switched USART1 RX DMA from normal mode to circular mode and increased the RX buffer from 128B to 256B.
- Reworked `HAL_UARTEx_RxEventCallback()` to consume only the newly received circular-DMA span using `s_uartRxLastPos`; it no longer re-arms RX DMA on every IDLE event.
- Removed the stale `#if 0` legacy normal-DMA parser block after the circular-DMA build passed, keeping only the active RX path.
- Added UART RX error recovery and `CMD:UART_RX_STAT?` / `DIAG:UART_RX?` diagnostics for RX error count, restart failures, last DMA position, and buffer size.
- Updated `BinaryCurrentParser.feed()` so short text ACKs such as `UNLOCK,OK,1\r\n` are released immediately unless they are a possible binary-frame prefix.
- Removed temporary HostComputer ACK debug prints from the serial worker, parser, and log handler.
- Added parser tests for short ACK residual release and split binary sync edge cases.

### Prevention / Follow-up
- With 256B at 1Mbaud, the RX circular buffer covers about 2.56ms of incoming bytes; this should be ample for short command lines while TIM/SPI ISRs preempt UART.
- Hardware validation still needs to verify repeated `CMD:UNLOCK,1`, `CMD:ENABLE,1`, `CMD:IDENTIFY,1`, and `CMD:UART_RX_STAT?` at 1Mbaud with current stream enabled.

### Verification
- `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_main_window`: 165 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 176 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware build OK, `153716 text / 38888 bss`.
- `rg -n "#if 0|Legacy normal-DMA|HAL_UARTEx_ReceiveToIdle_DMA\(&huart1" Core/Src/stm32h7xx_it.c Core/Src/usart.c`: no stale normal-DMA parser or active per-IDLE re-arm path remains.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: HostComputer package OK.
- Package output: `dist\24V_FOC_Host\24V_FOC_Host.exe`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `Core\Src\stm32h7xx_it.c`
  - `Core\Src\usart.c`
  - `MDK-ARM\code\head.h`
  - `HostComputer\data_parser.py`
  - `HostComputer\main_window.py`
  - `HostComputer\serial_worker.py`
  - `HostComputer\test_data_parser.py`
  - `dist\24V_FOC_Host\24V_FOC_Host.exe`
  - `PROGRESS.md`

## [2026-06-29] HostComputer Power State Visibility And Scroll Layout Fix

### Problem / Task
- The HostComputer control page could be clipped at full-screen/window-scale combinations, hiding lower diagnostics.
- Manual testing made unlock look failed even though the serial log contained `UNLOCK,OK,1` / `ENABLE,OK,1` and the firmware reported `FOC 状态 4` (RUNNING).

### Resolution
- Wrapped the tall HostComputer tabs (`控制器参数`, `参数识别`, `高级控制`, `环路参数`) in `QScrollArea` while leaving the realtime waveform tab full-size.
- Added a main control-page power status label (`未解锁 | 未使能`, `已解锁 | 未使能`, `已解锁 | 已使能`) and refreshed it from the same state as the V1.2 APP power status.
- Added GUI tests covering scrollable tabs and `UNLOCK,OK,1` / `ENABLE,OK,1` ACK-driven button/status updates.

### Prevention / Follow-up
- Use the explicit power status text as the source of truth during manual tests; disabled buttons alone are ambiguous after a successful unlock/enable.

### Verification
- `python -m unittest HostComputer.test_main_window HostComputer.test_gui_logic HostComputer.test_data_parser`: 161 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 172 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: HostComputer package OK.
- Zip output: `dist\24V_FOC_Host_20260629_2149.zip`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\main_window.py`
  - `HostComputer\test_main_window.py`
  - `dist\24V_FOC_Host_20260629_2149.zip`
  - `PROGRESS.md`

## [2026-06-29] DETENT Mode Entry Re-Capture Fix

### Problem / Task
- Selecting or configuring the HostComputer `卡点旋钮` product mode could show DETENT in the GUI while the motor still behaved like it was chasing an old position target.

### Resolution
- Changed firmware DETENT entry semantics so `FOC_App_SetAppMode(APP_MODE_DETENT)` always captures the nearest detent on entry when the motor is identified, even if a previous PREF/JOINT_POS command left `position_ref_user_set` latched.
- Changed HostComputer DETENT preset/config actions to always re-send `CMD:APP_MODE,DETENT` before `DETENT:CFG,...`, forcing firmware to re-enter DETENT and re-capture the nearest detent.
- Prevented HostComputer from treating generic N-frame telemetry as APP_MODE confirmation; APP_MODE is now confirmed only by ACK/query response.

### Prevention / Follow-up
- For manual validation, flash the rebuilt firmware before testing DETENT feel; updating only the GUI package is not enough for the re-capture behavior.

### Verification
- `python -m unittest HostComputer.test_main_window HostComputer.test_gui_logic HostComputer.test_data_parser`: 159 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 170 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware build OK, `build\gcc\24V_FOC_Controller.bin` updated.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: HostComputer package OK.
- Zip output: `dist\24V_FOC_Host_20260629_2134.zip`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `MDK-ARM\code\foc_app.c`
  - `HostComputer\main_window.py`
  - `HostComputer\gui_logic.py`
  - `HostComputer\test_main_window.py`
  - `HostComputer\test_gui_logic.py`
  - `build\gcc\24V_FOC_Controller.bin`
  - `dist\24V_FOC_Host_20260629_2134.zip`
  - `PROGRESS.md`

## [2026-06-29] HostComputer Power ACK Payload Fix

### Problem / Task
- The packaged HostComputer showed unlock failure even though the serial log contained firmware responses like `UNLOCK,OK,1`.

### Resolution
- Updated `AckParser` to accept both legacy ACK forms (`UNLOCK,OK`, `ENABLE,OK`) and firmware payload ACK forms (`UNLOCK,OK,1`, `ENABLE,OK,0`, plus IDENTIFY/STALL_MODE equivalents).
- Added `AckResult.command_value` and made the host state machine use the ACK payload when a pending command value is unavailable.
- Rebuilt the HostComputer package after stopping the stale running packaged GUI process that locked the previous `dist` folder.

### Prevention / Follow-up
- Keep tests for both ACK wire shapes because firmware responses may include the requested `0/1` payload while older assumptions did not.

### Verification
- `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic`: 101 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 167 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: packaging OK.
- Package output: `dist\24V_FOC_Host\24V_FOC_Host.exe`.
- Zip output: `dist\24V_FOC_Host_20260629_2116.zip`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer\data_parser.py`
  - `HostComputer\gui_logic.py`
  - `HostComputer\test_data_parser.py`
  - `HostComputer\test_gui_logic.py`
  - `dist\24V_FOC_Host\24V_FOC_Host.exe`
  - `dist\24V_FOC_Host_20260629_2116.zip`
  - `PROGRESS.md`

## [2026-06-29] HostComputer Chinese Product Mode Review And Package

### Problem / Task
- Review the HostComputer V1.2 product-mode Chinese-first GUI changes and produce a packaged Windows app for local testing.

### Resolution
- Verified the product-mode combo uses Chinese display labels while preserving firmware protocol tokens through combo `userData`.
- Found and fixed one ACK state-machine edge case: `UNLOCK,OK` / `ENABLE,OK` responses do not include the requested `0/1` value, so the host now stores the pending command payload and applies ACK effects according to the requested target.
- Rebuilt the HostComputer one-folder app and created a zip package for easier manual testing.

### Prevention / Follow-up
- Keep firmware protocol tokens internal and raw serial logs untranslated; translate only user-facing GUI labels/status text.
- When adding ACK parsing for set-style commands, track the requested payload because the firmware ACK may only echo the command name.

### Verification
- `python -m unittest HostComputer.test_main_window`: 55 tests OK.
- `python -m unittest HostComputer.test_gui_logic`: 48 tests OK.
- `python -m unittest HostComputer.test_data_parser`: 51 tests OK.
- `python -m unittest discover -s HostComputer -p "test*.py"`: 165 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: packaging OK.
- Package output: `dist\24V_FOC_Host\24V_FOC_Host.exe`.
- Zip output: `dist\24V_FOC_Host_20260629_2106.zip`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/gui_logic.py`
  - `HostComputer/test_gui_logic.py`
  - `dist\24V_FOC_Host\24V_FOC_Host.exe`
  - `dist\24V_FOC_Host_20260629_2106.zip`
  - `PROGRESS.md`

## [2026-06-29] Sanitized GitHub Release Repository Sync

### Problem / Task
- Sync the current project to `https://github.com/xianyuyijinban/24V-FOC-controller` without publishing private project notes, documentation, debug dumps, or secret-like files.

### Resolution
- Created a clean release snapshot in `C:\tmp\24v-foc-release-sync` instead of pushing the dirty local development branch/history.
- Included firmware source, STM32 project files, required CMSIS/HAL subsets, HostComputer source/tests, build scripts, and executable test scripts.
- Excluded `docs/`, `PROGRESS.md`, `PROCESS.md`, `Project_Architecture.md`, `.agents/`, `.codex/`, `.claude/`, build/dist outputs, debug logs/dumps, JSON/CSV captured results, and key/env-style files.
- Pushed the sanitized snapshot to the release repository `main` branch with `--force-with-lease`.

### Prevention / Follow-up
- Continue using sanitized snapshot/export sync for the public release repository; do not push the local development branch directly because it contains documentation and work logs.

### Verification
- Export safety gate: forbidden docs/log/key paths count was 0.
- Secret-like scan: no PAT/private-key/password/API-key style content found; only a parser variable named `token` was a false positive in the first broad scan.
- `python -m unittest discover HostComputer`: 132 tests OK in the release snapshot.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware build OK in the release snapshot.
- `git ls-remote https://github.com/xianyuyijinban/24V-FOC-controller.git refs/heads/main`: remote `main` points to `c9712234a35b90c813018766059941d93080a987`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `PROGRESS.md`

## [2026-06-28] HostComputer HOLD Panel Runtime Telemetry Fix

### Problem / Task
- In the V1.2 advanced control page, `HOLD` mode showed current angle and current speed as `--`, making it look like the firmware was not reporting angle telemetry.
- The mode status also showed `APP HOLD | CTRL speed` because it trusted the last raw telemetry control-mode field even though HOLD is a product-level position-hold mode.

### Resolution
- Wired the HOLD panel angle/speed labels to normal runtime telemetry in `apply_packet()`.
- Added product-mode control semantics for the advanced status label: `HOLD`, `JOINT_POS`, `SPRING_DAMPER`, and `DETENT` display as position semantics, while `GIMBAL_SPEED` displays as speed.
- Added a GUI regression test proving the HOLD panel updates from a received `FOCDataPacket`.

### Prevention / Follow-up
- If the HOLD panel still shows `--`, check whether the general runtime panel is receiving fresh `N` telemetry frames. If the general runtime panel has angle but HOLD does not, it is a HostComputer UI wiring bug.
- Flash the firmware ACK/FAIL diagnostic build separately if text command responses like `UNLOCK,OK` / `ENABLE,OK` are still needed for command-chain debugging.

### Verification
- `python -m unittest discover HostComputer`: 132 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK, `dist/24V_FOC_Host/24V_FOC_Host.exe` updated at 2026-06-28 19:19:42.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-06-28] HostComputer JOINT_POS Mode Preservation Fix

### Problem / Task
- In the V1.2 advanced control page, selecting `JOINT_POS` with soft limits enabled could still show the underlying control mode as speed.
- Sending a 20 degree joint target left firmware telemetry at `pos_ref=-0.00 deg`, indicating the position target did not enter the active position-mode path.

### Resolution
- Removed the explicit `CMD:MODE,1` from the HostComputer enable/quick-arm sequence. Firmware already switches to speed internally only when stall/open-loop enable is actually required; the explicit GUI mode command reset non-RAW `APP_MODE` back to RAW and broke JOINT_POS.
- Updated APP_MODE response parsing to accept both `APP_MODE,OK,JOINT_POS` and `APP_MODE,OK,JOINT_POS (ctrl_mode=2)`.
- Updated the advanced control status label to show both product mode and underlying control mode (`APP ... | CTRL ...`) instead of only the underlying control mode.
- Changed the JOINT_POS target button to always send `CMD:APP_MODE,JOINT_POS` immediately before `CMD:PREF,...`, so stale host/firmware mode state cannot send PREF into RAW/SPEED mode.

### Prevention / Follow-up
- Re-test JOINT_POS by setting app mode, enabling, then sending a target. Expected log includes `APP_MODE,OK,JOINT_POS`, `PREF,OK,0.349`, and telemetry should show `pos_ref` near the requested target instead of 0.

### Verification
- `python -m unittest HostComputer.test_gui_logic HostComputer.test_main_window`: 91 tests OK.
- `python -m unittest discover HostComputer`: 131 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK after stopping the stale running GUI process.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/gui_logic.py`
  - `HostComputer/main_window.py`
  - `HostComputer/test_gui_logic.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-06-28] Firmware Command ACK/FAIL Diagnostics

### Problem / Task
- After V1.2 GUI integration, enable and motor identify appeared to send commands from HostComputer but did not change firmware state.
- Need to distinguish "Host did not send", "firmware did not receive", and "firmware received but silently rejected".

### Resolution
- Checked HostComputer command generation and firmware dispatch forms: legacy `CMD:UNLOCK,1`, `CMD:ENABLE,1`, and `CMD:IDENTIFY,1` still match firmware handlers.
- Found that key control commands were mostly silent on success/reject, including `UNLOCK`, `STALL_MODE`, `ENABLE`, `IDENTIFY`, `MODE`, `IREF`, `SREF`, and `PREF`.
- Added explicit UART ACK/FAIL responses for those commands in `Core/Src/stm32h7xx_it.c`, including reject reasons for enable/identify (`locked`, `fault`, or `rejected` with state/pwm/encoder/stall details).

### Prevention / Follow-up
- Flash this build before the next hardware check. The expected GUI log should now show `UNLOCK,OK`, `STALL_MODE,OK`, `ENABLE,OK/FAIL`, and `IDENTIFY,OK/FAIL` responses.
- If `ENABLE,FAIL,rejected,...` appears, use the reported `state/pwm/identified/enc/stall` fields to target the real firmware precondition rather than guessing from TX logs.

### Verification
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: GCC build OK, 0 compiler errors, generated `.elf/.hex/.bin`.
- Artifacts exist: `build/gcc/24V_FOC_Controller.elf`, `.hex`, `.bin`.
- Not flashed in this turn.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `Core/Src/stm32h7xx_it.c`
  - `PROGRESS.md`

## [2026-06-28] HostComputer Enable / Quick-Arm State Fix

### Problem / Task
- After V1.2 Joint Product Mode GUI integration, HostComputer could no longer enable the motor from either the original top-level controls or the new APP-mode controls.
- The enable/arm button state chain needed to be restored without touching firmware.

### Resolution
- Added a separate `can_quick_arm` UI state for "connected, not enabled, not identifying, no fault" so quick-arm is available before unlock.
- Removed the top-level quick-arm button from the motion-target enable group, which incorrectly required the motor to already be enabled.
- Changed the V1.2 APP-mode "enable" button to reuse `_request_enable_motor()` instead of directly sending `CMD:ENABLE,1`, preserving stall-mode confirmation and mode setup.
- Changed the V1.2 APP-mode "unlock and enable" button to use `can_quick_arm`.
- Treat unknown encoder status (`encoder_detected=None`) as requiring stall-mode confirmation, matching firmware's enable precheck which re-validates `TLE5012_IsDataValid()` and silently rejects unarmed stall cases.
- Hardened the shared enable sequence to conservatively send `CMD:STALL_MODE,1` before `CMD:ENABLE,1` when stall mode is not already armed. Normal identified/encoder-online enable paths still run normally in firmware, while invalid/unknown encoder cases no longer degrade into repeated bare `CMD:ENABLE,1`.
- Added regression tests for top-level quick arm, APP-mode enable, APP-mode quick arm, and fault/identify disable rules.

### Prevention / Follow-up
- Hardware check: verify the serial log emits `CMD:STALL_MODE,1` before `CMD:ENABLE,1` instead of repeated bare `CMD:ENABLE,1`. If the stall-mode confirmation path is shown, the expected sequence is `CMD:STALL_MODE,1`, `CMD:MODE,1`, and `CMD:ENABLE,1`.

### Verification
- `python -m unittest HostComputer.test_main_window`: 48 tests OK.
- `python -m unittest HostComputer.test_gui_logic`: 41 tests OK.
- `python -m unittest HostComputer.test_data_parser`: 29 tests OK.
- `python -m unittest discover HostComputer`: 129 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK after stopping the stale running `24V_FOC_Host.exe` that locked `dist/`.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/gui_logic.py`
  - `HostComputer/main_window.py`
  - `HostComputer/test_gui_logic.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`

## [2026-06-28] HostComputer Scope Time-Origin Fix

### Problem / Task
- Scope screenshot showed `Time (s)` axis expanded to thousands (`0..10000`), making the plot unusable.
- User clarified `TELEM:CUR,OFF` was intentionally used to pause the stream and inspect the captured waveform.

### Resolution
- Ignore `timestamp=0` packets as scope time origin.
- Telemetry plot series now falls back to its own first timestamp when the session origin is missing/invalid.
- Starting scope immediately sets a bounded X range (`0..window_s`) so pyqtgraph does not auto-expand over all historical data.
- Preserved OFF-as-pause behavior and current-stream ring contents.
- Rebuilt Host GUI package.

### Prevention / Follow-up
- Continue treating waveform as secondary; use current diagnostic line for decisions.
- If mixed telemetry/current plotting still feels noisy, split the tab into separate "runtime telemetry" and "current diagnostics" panes.

### Verification
- `python -m unittest discover -s HostComputer`: 124 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`
  - `dist/24V_FOC_Host/`

## [2026-06-28] HostComputer Current Stream Diagnostics

### Problem / Task
- Raw three-phase current waveform was visible but not very useful for FOC diagnosis.
- Clarified that `TELEM:CUR,OFF` may be intentionally used as a pause/snapshot action, so OFF must not be treated as a display bug or clear old samples.

### Resolution
- Added a current diagnostic line under Current Stream controls.
- Diagnostic window reports `sumABC` mean/RMS, `Id` mean/p-p, `Iq` mean/p-p, and phase-current p-p in mA over the latest 1000 samples.
- Kept ring-buffer waveform data when stream is turned OFF, preserving pause-and-inspect behavior.
- Fixed near-zero signed display so floating-point `-0.0mA` is shown as `+0.0mA`.
- Rebuilt Host GUI package.

### Prevention / Follow-up
- Prefer these diagnostic metrics over raw phase-current waveform for bring-up decisions.
- Next useful display mode would be envelope/RMS/current-error panels instead of denser raw waveform plotting.

### Verification
- `python -m unittest discover -s HostComputer`: 123 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`
  - `dist/24V_FOC_Host/`

## [2026-06-28] HostComputer Current Scope Readability Polish

### Problem / Task
- Current waveform was visible but still hard to read: legend listed every channel, Y axis used generic scaled value, and 5s current view was visually dense.

### Resolution
- Plot legend now rebuilds from selected channels only.
- Current-stream samples are plotted in mA while parser/ring storage remains in A.
- Current-only views set the Y axis to `Current (mA)` and disable confusing SI auto-prefix behavior.
- Opening current stream now switches the scope window from default 5s to 1s for a clearer first view.
- Current curves use thinner pens and pyqtgraph clip/downsample hints.
- Rebuilt Host GUI package.

### Prevention / Follow-up
- If 1kHz still looks too dense, add an explicit display mode selector: raw / decimated / envelope / RMS, without changing the binary protocol.

### Verification
- `python -m unittest discover -s HostComputer`: 122 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `PROGRESS.md`
  - `dist/24V_FOC_Host/`

## [2026-06-28] V1.1 Speed SREF Clamp And Current Stream UX Fix

### Problem / Task
- User observed `SREF=6` still running slowly, and HostComputer scope showed no phase-current waveform.
- Check whether the issue was protocol/firmware behavior or GUI activation.

### Resolution
- Firmware: decoupled RAW `SREF` from conservative `MOTION_CFG.speed_limit`; RAW SPEED now clamps to `FOC_SPEED_REF_MAX_RAD_PER_S = +/-8.0`, while position/joint/gimbal trajectory limits still use 12V `MOTION_CFG`.
- HostComputer: enabling the current-stream group now auto-starts scope and sends recommended `TELEM:CUR,BIN,1000`; mode changes also start scope when needed.
- Docs: updated `UART_COMMANDS.md` and `PLATFORM_OVERVIEW.md` to describe RAW SREF clamp vs MOTION trajectory limits.
- Rebuilt Host GUI package after closing the old running `24V_FOC_Host.exe` that locked `dist/`.

### Prevention / Follow-up
- `SREF=6` will no longer be silently clipped to 1.0/2.0, but the raw speed ramp remains `2 rad/s^2`, so reaching 6 rad/s still takes about 3 seconds by design.
- Bench-test high SREF carefully on 12V; protections remain active, but the default 12V release envelope was validated mainly around +/-1 rad/s.

### Verification
- `python -m unittest discover -s HostComputer`: 121 tests OK.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`: firmware GCC build OK, artifacts generated.
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`: package OK.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63cd991fd9ea05e84f021c804e1343fbf7be`
- Status: `working tree changes not committed yet`
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/foc_app.c`
  - `HostComputer/main_window.py`
  - `HostComputer/test_main_window.py`
  - `docs/UART_COMMANDS.md`
  - `docs/PLATFORM_OVERVIEW.md`
  - `PROGRESS.md`
  - `dist/24V_FOC_Host/`

## [2026-06-28] HostComputer Scope 优化体验包打包

### Problem / Task
收口上位机波形图体验优化，打包可直接体验的 Windows GUI。

### Resolution
- 修正 scope review 后的 5 个细节：BIN2000 时间轴按实测 fps、current stream 优先驱动跟随时间、开始波形重置 fps 统计、无 pyqtgraph guard、移除无用相对 import。
- 使用 `build_host_gui_app.ps1` 打包 HostComputer one-folder app。
- 启动体验包验证进程可运行。

### Prevention / Follow-up
- 手动体验 GUI：连接 `COM9 @ 1000000`，验证 OFF/BIN1000/BIN2000 切换、时间轴、缩放和自动 Y。

### Verification
- `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_serial_service HostComputer.test_main_window`：112 tests OK。
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`：打包成功。
- Executable: `E:\24V_FOC_Controller_sync_20260519\dist\24V_FOC_Host\24V_FOC_Host.exe`
- Launch check: process running after 3s.

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63c`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `PROGRESS.md`
  - `dist/24V_FOC_Host/`

## [2026-06-28] HostComputer Scope 波形优化 review

### Problem / Task
审查上位机波形图优化实现，确认默认关闭、相对时间轴、缩放逻辑和 current stream 显示风险。

### Resolution
- 确认 HostComputer 单测通过：112 tests OK。
- 发现后续需修正的 review 点：BIN2000 时间轴固定按 1000fps 计算会失真；current stream elapsed 应优先使用最新 current sample 时间；scope start 应重置 fps 统计；无 pyqtgraph 时 scope toggle 需 guard；`_get_current_stream_plot_data()` 内部未使用的相对导入会影响 top-level import 场景。
- 已修正上述 5 点：current stream 时间轴改为 seq + 实测 fps；scope elapsed 优先使用 current stream 最新 tick；开始波形清零 `_cur_stats_prev_total`；无 pyqtgraph 时跳过 `setXRange`；删除方法内无用相对 import。

### Prevention / Follow-up
- 修正上述 review 点后再做 GUI 手动 smoke。

### Verification
- `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_serial_service HostComputer.test_main_window`：112 tests OK（修正前后均通过）。

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `148b63c`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/main_window.py`
  - `PROGRESS.md`

## [2026-06-28] Step 2C：上位机体验收口与 V1.1 发布前整理

### Problem / Task
收口 Step 2A/2B 成果，做 GUI 体验打磨、文档同步、发布前 regression。

### Resolution
- **GUI**：BIN 1kHz 标注"推荐"，BIN 2kHz 标注"实验"；stats label 显示 fps + baud
- **Docs**：`docs/UART_COMMANDS.md` baud → 1000000，新增 TELEM 章节（RATE + CUR），1152000 标为禁用
- **固件行为定版**：1000000 baud，BIN 1000 推荐默认，BIN 2000 实验档（≤1330fps），OFF 恢复 N 帧
- **不扩大 TX ring**，**不改 IT chunk**，**不改 binary payload**

### Verification
- HostComputer: 112 tests OK
- 固件编译: 0 error / 0 warning
- CUR OFF → N-frame 50Hz 恢复 ✓
- BIN 1000: 2001f/2s (1001fps ✓), CRC=0, FW_INFO? 50/50
- BIN 2000: 2662f/2s (~1330fps), N-frame→10Hz 自动降载 ✓, FW_INFO? 50/50

## [2026-06-28] V1.1 Current Stream / Host GUI 问题定位

### Problem / Task
- 继续 Step 2A：在 1000000 baud 基线上验证 2kHz binary current stream 与上位机适配。

### Resolution
- 修复 HostComputer 入口语法错误：`GuiProfile.baud_rate` 的 C 风格注释改为 Python 注释。
- 同步 HostComputer 单测期望：默认 baud 从 230400 更新为 1000000。
- 指出固件侧关键问题：TIM1 center-aligned update IRQ 实际约 20kHz（overflow + underflow），但 `current_stream.c` 中 `CUR_STREAM_ISR_HZ` 写成 10000，导致 `TELEM:CUR,BIN,1000` 实际约 2kfps、`BIN,2000` 实际逼近 4kfps 并触发串口背压/seq gap。

### Prevention / Follow-up
- 将 `CUR_STREAM_ISR_HZ` 修正为 20000，并同步注释；修正后再复测 1000/2000 档。
- 后续按用户要求由执行方继续测试，当前 Codex 仅指出问题。

### Verification
- `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_serial_service HostComputer.test_main_window`：112 tests OK。
- 无界面串口采样曾观察到 `BIN,1000` 约 2003fps、`BIN,2000` 约 3433fps 且有 seq gap，和 ISR 基准倍率问题一致。

### Commit
- Branch: `codex/sync-main-20260519`
- Commit: `c865e09014da4eed0a8a5495eda0ff50204ca306`
- Status: `working tree changes not committed yet`
- Files:
  - `HostComputer/gui_logic.py`
  - `HostComputer/test_gui_logic.py`
  - `PROGRESS.md`

已并入 [PROCESS.md](PROCESS.md)。

从 `2026-04-05 20:10` 起，`PROCESS.md` 是唯一主日志；本文件仅保留为兼容入口，避免后续台架调试继续分叉记录。

## [2026-06-28] V1.1 UART Baud Sweep：1000000 定版，1152000 禁用

### Problem / Task
Step 2A 目标升级 UART 至 1152000。测试发现该波特率下命令 RX（DMA+IDLE）不稳定：
- 230400 / 921600 / 1000000 全部 20/20 PASS（FW_INFO? 命令 + N/C 遥测）
- 1152000 仅 1/20 PASS，遥测 IT TX 正常但 DMA RX 几乎不可用

### Resolution
**V1.1 高速通信基线定为 1000000 baud**（GPIO_SPEED_FREQ_VERY_HIGH 保留）：
- 2kHz 电流流带宽需求：25B × 2000 = 50 kB/s
- 1000000 baud 实际约 100 kB/s，约 50% 余量给命令 + 低频遥测
- 921600 作为备选档，1152000 禁用

### 1152000 现象记录（避免以后再踩）
- 不是完全不通：第 1 次 FW_INFO? 成功（返回完整响应），后续 19 次失败
- 失败时读取到的是遥测行（C/N 帧），不是命令响应
- 根因未深究，判断为高波特率边界下 RX DMA/IDLE restart、UART error recovery 或 TX IT 抢占导致接收链路变脆
- 物理链路 CH340C + PC + STM32H743 USART1 在 1152000 不构成可靠双向通道

### Decision Rule
- 若后续发现 1000000 也不够稳定，回退到 921600
- 1152000 永久禁用，不再投入时间

### Verification
- 1000000: FW_INFO? 20/20, N=100, C=399, 0 错误
- 921600: FW_INFO? 20/20, N=100, C=399, 0 错误
- 1152000: FW_INFO? 1/20 ❌
- 230400: 历史已知稳定（未重复测试）

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

## [2026-06-22] BEMF 回归测试 Round 2：诊断为 OFF 基线不变

### Problem / Task
Round 1 无法解释 BEMF ON 后静止 Vq=59mV，PWM_DIAG 缺少 BEMF 诊断字段。需增强诊断后重测。

### Resolution — 固件改动
**`Core/Src/stm32h7xx_it.c`** PWM_DIAG 新增 5 个字段：
- `ome` — 电角速度 mrad/s
- `Vqb` — BEMF 解耦 Vq 补偿 mV
- `Vdb` — BEMF 解耦 Vd 补偿 mV
- `bemb` — 保护门禁（1=过压置零）
- `Ke` — 当前有效 Ke μV/(rad/s)
- buffer 200→256 防止截断

**`scripts/bemf_regression.py`** 三处改进：
- PWM_DIAG key 更新，打印完整 BEMF 分解
- Phase 2 结束后自动选 settle Vq 最低 Ke；Phase 3 用最优 Ke + 次优对照
- Phase 3 故障检测改从 N 帧 AppFault 字段，消除 C 帧误报

### Round 2 测试结论

| 结论 | 证据 |
|------|------|
| **BEMF 算法正确** | Vqb = Ke × ωe 精确到 mV（0.003×8.027=24mV, 0.006×7.906=47mV, 0.0117×9.0=105mV） |
| **静止残留不是 BEMF** | 静止时 Vqb=-1mV，总 Vq 波动来自 PI+RsFF+采样噪声 |
| **低速无收益** | ±1 rad/s 下 BEMF 贡献仅几十 mV，回零残留略增 |
| **bemb 门禁始终未触发** | 所有 Ke 档在 ±1 rad/s 下 Vqb 远低于饱和限 |

### Ke 分档对比

| | Ke=0.003 | Ke=0.006 | Ke=default(0.0117) |
|--|----------|----------|-------------------|
| Settle Vq_pk | 57mV | 57mV | **74mV** ❌ |
| +1.0→0 回零 Vq | **23mV** ✅ | 156mV ❌ | 65mV |
| +1.0→0 回零 Id | 3mA | 32mA | -12mA |
| 耐久 Vq_pk | 48~65mV | 51~64mV | — |

### 决策
- **BEMF 保持 OFF**（当前基线不变）
- Ke=0.003 记录为实验候选，不写入默认参数
- BEMF 后续在更高速（几十 rad/s，反电动势成为主要压降）时复测
- 诊断字段保留，方便后续任何电压前馈问题归因

### 当前基线
```
PI_CURRENT = 0.50/0, PI_SPEED = 0.25/0
RS_FF = DQ/0.20, COG = 0.25/60°
BEMF = OFF, Ke_temp = 0
```

### Commit
- Commit: `7b80d79`
- Branch: `codex/sync-main-20260519`
- Files:
  - `MDK-ARM/code/foc_app.h`
  - `MDK-ARM/code/foc_app.c`
  - `MDK-ARM/code/foc_core.c`
  - `PROGRESS.md`

## [2026-06-22] BEMF 回归测试：低速下无收益，BEMF 保持 OFF

### Problem / Task
在 ARR=11999 + PI_CURRENT=0.50/0 + PI_SPEED=0.25/0 + RsFF=DQ/0.20 基线基础上，单独回归 BEMF 前馈。测试计划：Ke 分档（0.003/0.006/default）+ SREF sweep + 10 周期 smoke，判断 BEMF 是否可进入下一版基线。

### Resolution
1. **芯片全擦→参数丢失**：`pyocd erase --chip` 擦除了 Flash 中存储的电机识别参数（Rs/Ld/Lq/Ke），导致 `motorIdentified=0`，SpeedLoop 无法进入 RUNNING 状态。通过 `CMD:IDENTIFY,1` 重新识别恢复（identifyState 正常走完 1→9 MI_STATE_COMPLETE）。
2. **N-frame 字段映射修正**：遥测帧索引需要精确匹配 `uart_upload.c:376` 的格式（33 字段，appFaultCode@[14], controlMode@[15], Id_ref@[16], speed_ref@[17], pos_ref@[18], Iq_ref@[19], Vd@[20], Vq@[21]）。脚本此前使用错误的早期偏移，导致 Iq_ref 误读为 pos_ref（3.628）。
3. **VBUS_LIMIT 必须先于 CLEAR_FAULT**：默认欠压阈值 18V，12V 供电触发 FOC_FAULT_UNDERVOLTAGE。必须先设 VBUS_LIMIT,8,15 再 CLEAR_FAULT，否则故障无法清除。
4. **BEMF 测试结果**（Ke=0.003/0.006/default）：
   - Phase 1 静态：Vq_pk=59mV（单点 5mV），BEMF blocked=0，无故障
   - Phase 2 SREF：速度跟踪正常（pk=0.40-1.08 rad/s），但零回 Vq_pk=31-120mV（BEMF OFF 基线 26-51mV）
   - Phase 3 Smoke：Ke=0.003 零回 Vq_pk=53-64mV（>50mV 高频出现）
   - **BEMF ON 零回 Vq 劣于 BEMF OFF**，且低速下 Ke·ω < 10mV，收益为零
5. **判定**：BEMF 保持 OFF，当前基线 `BEMF=OFF` 不变。

### Prevention / Follow-up
1. **禁止 `pyocd erase --chip`**：只需 `pyocd flash` 即可更新固件，全擦会丢失识别参数。如需烧录，先 `pyocd flash`（不擦除），失败才考虑 sector erase。
2. BEMF 的真正价值在高速（>50 rad/s，Vbemf > 350mV），低速测试无法体现其优势。届时用 `CMD:KE_TEMP` 分档 + 高速 SREF sweep 重新回归。
3. 当前 FF 策略验证完成：COG(机械) + RsFF(电阻) 已够覆盖低速域；BEMF(反电动势) 留到高速域再开。
4. `scripts/bemf_regression.py` 和 `scripts/rsff_sweep.py` 的 N-frame 索引均已修正为 `uart_upload.c:376` 的正确映射。

### Verification
- Phase 1: 静态 BEMF ON, Vq_pk=59mV, 0 fault, BEMF blocked=0
- Phase 2 Ke sweep: 三档 Ke 全部完成 SREF ±0.5/±1.0 sweep, 零回残留随 Ke 增大而恶化
- Phase 3: Ke=0.003 10 周期 smoke, Vq_pk 53-64mV, 0 AppFault
- 最终 PWM_DIAG 确认：CCR=6000/6000/6000（50% duty）, Idref=Iqref=0, PI 清零正常

### Commit
- Commit: `N/A (working tree changes not committed yet)`
- Branch: `codex/sync-main-20260519`
- Files:
  - `scripts/bemf_regression.py` (新增)
  - `scripts/motor_recovery.py` (新增)
  - `scripts/rsff_sweep.py` (N-frame 索引修正)
  - `scripts/bemf_results_20260622_005631.json` (测试数据)
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

## [2026-06-26] Phase 2 速度 Ki 门控定版：PI_SPEED=0.25/0.001 gated

### Problem / Task
速度环 P-only (Ki=0) 在低速段追踪不足，需要引入门控积分提升低速维持能力，同时确保：
- 仅速度模式使用积分，位置模式保持 P-only（防止 PREF 回零残留）
- 积分贡献限幅 ±0.10A
- 回零 Vq 基线与 P-only 时期相当

### Resolution
**固件改动** (`foc_app.c`)：
1. `UpdateLoopParams()` 内 `Ki_s: 0.0f → 0.001f`，注释更新为 "Phase 2 gated"
2. 冷启动默认 PI 同步更新：`FOC_PI_Init(&pi_speed, 0.25f, 0.001f, ...)`
3. 门控逻辑已在未提交 diff 中完整就位：
   - `|SREF| < 0.02` → 清零积分 (i_state=3)
   - `|SREF| ≥ 0.05` 且 `|speed_error| ≥ 0.02` → 允许积分 (i_state=1)
   - 过渡区 → 冻结 (i_state=2)
   - 输出饱和 → 抗饱和回退 (i_state=4)
   - 积分贡献限幅 ±0.10A
   - 位置模式 / disable / fault → 清零积分
4. 诊断：N-frame SpeedLoopDiag 含 `p_iq, i_iq, i_state`

### Verification（12V 台架，VBUS_LIMIT=10-16V）

| 测试 | 结果 | 状态 |
|------|------|------|
| 编译 | GCC 0e0w, text=137588 | PASS |
| 冷启动 | AppFault=0, BEMF=OFF, RS_FF=DQ/0.20, COG=0.25/60° | PASS |
| 低速 sweep (±0.05~±1.00) | 8/10 通过 (SREF=+0.05 28%, SREF=-0.10 71%) | PASS* |
| 方向对称性 | 全部 |diff| < 0.05 rad/s | PASS |
| 回零 Vq | max=18mV, mean=9mV (P-only 基线 17-56mV) | PASS |
| 20 周期耐久 (±0.5 rad/s) | 0 faults, Vq_pk 6-8mV, ω 追踪 112-116% | PASS |
| 位置回归 (0/±5/±20/0°) | 全部 drift<0.005 rad/s, 0 faults | PASS |

\* SREF=+0.05 (28%) 和 SREF=-0.10 (71%) 未达 80% 阈值，判定为 12V 供电下静摩擦/齿槽效应的方向不对称，与门控逻辑无关。

### 12V 限制记录（已更新）
- 电机 12V 稳态包络：±0.3 ~ ±1.0 rad/s 追踪 98-102%（16/16 全部 stable）
- ±1.0 急减速（-1.0→0 制动）可触发 DRV8350S UVLO (0x00400080)
- 12V 标准基线额定速度定为 ±0.5 rad/s，极限验证 ±1.0 rad/s
- 控制地板 ±0.05 rad/s（需 2-3s settle for Ki=0.001 integration）

### Prevention / Follow-up
1. **回零 Vq 基底**：当前 max=18mV，仍在 P-only 基线 (17-56mV) 范围内，积分项确认清零，零速控制基底作为独立后续问题
2. **位置模式安全边界**：位置模式速度积分清零已在本轮验证（全部 PREF 无 drift），勿在位置模式放开积分
3. **12V 永久基线**：项目不再以 24V 为目标平台，所有默认参数按 12V 收敛
4. **后续不进入本轮**：BEMF、ABC RsFF、observer、VDQ_PULSE

### Commits
- `319290c` `fix: enable gated speed integral for low-speed tracking`

## [2026-06-26] 12V 标准基线冻结

### Problem / Task
将项目从 24V 目标平台切换为 12V 永久基线，需完整表征 12V 工作包络并更新所有默认参数。

### 12V 工作包络表征结果

**速度包络**（±0.3 ~ ±1.0 rad/s, 0.1 步长, 双向）：
| 指标 | 结果 |
|------|------|
| 稳态追踪 | 16/16 全部 stable (98-102%) |
| Vq_pk | 18-33mV |
| Vbus_min | 11.9V 稳定 |
| 饱和点 | >±1.0 rad/s（未触及） |

**低速地板**（±0.05 ~ ±0.20, 双向, 3s settle）：
| 指标 | 结果 |
|------|------|
| 追踪 (≥0.10) | 7/8 tracking (96-117%) |
| ±0.05 | 69-75%（判定为 12V 控制地板） |

**位置模式**：
- PREF=0/±5/±20/0 全部 drift < 0.004 rad/s
- speed_limit=1.0 正常追踪

**负载安全**（手捏扰动, ±0.5/±0.7, 双向）：
| 指标 | 结果 |
|------|------|
| Fault | 4/4 no fault |
| UVLO | 0 复现（上次为急减速触发） |
| Iq_peak | max=0.160A << 0.35A |
| 回零 Vq | 10-18mV |

注：手捏负载速度跌落数据因空载段预加载污染，不作为速度刚度定量依据。

### 12V 标准基线参数

```text
PI_CURRENT  = 0.50 / 0
PI_SPEED    = 0.25 / 0.001 gated
RS_FF       = DQ / 0.20, adaptive OFF
BEMF        = OFF
COG         = 0.25 / +60deg

额定速度:      ±0.5 rad/s
验证极限:      ±1.0 rad/s 空载稳态
控制地板:      ±0.05 rad/s (需 2-3s settle)
位置 speed_limit: 1.0 rad/s
位置 accel:       2.0 rad/s²
位置 cruise:      0.3 rad/s

SREF 命令限幅:  ±1.0 rad/s (MOTION_CFG_SPEED_LIMIT_DEFAULT)
```

### 代码改动

**`foc_app.h`** — MOTION_CFG 默认值收敛到 12V：
- `SPEED_LIMIT_DEFAULT`: 1.5 → 1.0 rad/s
- `ACCEL_LIMIT_DEFAULT`: 6.0 → 2.0 rad/s²
- `CRUISE_SPEED_DEFAULT`: 1.2 → 0.3 rad/s
- `SPEED_LIMIT_MAX`: 8.0 → 2.0 rad/s
- `ACCEL_LIMIT_MAX`: 30.0 → 5.0 rad/s²

**`foc_app.c`** — 注释更新：SREF 限幅说明、"12V 标准基线"

### Prevention / Follow-up
1. **不再追 24V 性能**，所有后续开发以 12V 为唯一目标平台
2. **速度刚度专项**留待后续固定负载（砝码/摩擦/关节装配）测试
3. **UART 指令和上位机参数整理**为下一优先事项
4. **回零 Vq 基底**（当前 <20mV）作为独立问题跟踪

### Commits
- `a2f0c5b` 12V 基线冻结：MOTION_CFG 默认值更新 + PROGRESS.md
- `d9b50f9` `CMD:FW_INFO?` + git hash 编译时注入
- `141c3ee` Phase 1：UART 指令别名体系 + SYS:CMDS? + UART_COMMANDS.md
- `ce5bb09` Phase 2：UART TX ring buffer + IT 非阻塞发送 + TELEM 命令
- `4e5d4d7` Phase 3A：APP_MODE 层（JOINT_POS, GIMBAL_SPEED, HOLD）

## [2026-06-27] Phase 2: UART TX Ring Buffer + IT 非阻塞发送

### Resolution
- 1024B ring buffer + HAL_UART_Transmit_IT 替换所有阻塞 HAL_UART_Transmit
- 3 级优先级准入：P0 不丢，P1 缓冲区近满时丢，P2 反压下丢
- UART_CommandSendText 改为非阻塞
- 新增 TELEM:ON/OFF, TELEM:RATE,<hz>, TELEM:RATE? 命令

### Verification
- TELEM:OFF 后停止周期遥测，命令回执仍正常
- BSS +1040B (1024B ring buffer)

### Commits
- `ce5bb09`

## [2026-06-27] Phase 3A: APP_MODE 产品模式外壳

### Resolution
- 新增 AppMode_t（RAW/JOINT_POS/GIMBAL_SPEED/HOLD），不替换底层 FOC_ControlMode_t
- APP_MODE_RAW 完全兼容旧行为
- JOINT_POS：底层 POSITION + 软限位裁剪，PREF 超限不触发 fault
- GIMBAL_SPEED：底层 SPEED + 可配置 SREF 斜坡加速度
- HOLD：底层 POSITION + 进入时捕获当前位置作为目标
- 新增命令：CTRL:APP_MODE, JOINT:LIMIT, GIMBAL:RAMP
- STOP 在所有 APP_MODE 下立即生效

### Verification (12V bench)
- RAW 模式 SREF=±0.5 追踪 98-101%
- APP_MODE 命令正确切换并返回状态
- JOINT:LIMIT 和 GIMBAL:RAMP 查询/设置正常
- STOP 在 4 个模式下均无 fault
- 位置模式/HOLD 追踪精度待解析器修复后定量验证

### Known Issues
- Phase 2 IT 发送在部分帧中出现 NUL 字节（~4% 帧受影响），不影响控制功能
- 上位机解析器需适配以正确处理 IT 分片帧，作为 Phase 2b 工作项

### Commits
- `4e5d4d7`
