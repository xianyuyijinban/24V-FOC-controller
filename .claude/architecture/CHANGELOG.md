# 架构变更日志

追踪所有架构层面的变更。

---

### 2026-06-21: 自适应 RsFF + ABC 域前馈 + 速度环 P-only 定版
- **子系统**: FOC 核心, FOC 应用层, UART 上传
- **文件**: `foc_core.h`, `foc_core.c`, `foc_app.c`, `stm32h7xx_it.c`
- **摘要**:
  1. 自适应 RsFF：4 因子置信度（dIq/dt, speed_error, sat_ratio, sign_mismatch）+ 非对称 LPF
  2. 改进 Sign Protect：幅值门限 + 50 周期持续时间，消除正常换向误判
  3. ABC 域 RsFF 前馈路径（RS_FF_MODE=2）：逐相 Vabc 限幅 + SVPWM from ABC
  4. 运行时开关：CMD:RS_FF_ADAPTIVE, CMD:RS_FF_SIGN_PROTECT, CMD:RS_FF_MODE
  5. 速度环 P-only 定版：Kp=0.25 Ki=0.01，20 周期耐力验证通过
  6. 自适应参数基线：DIQDT=2000, SPEED_ERR=1.0/0.8, SIGN_REF=0.08, SIGN_FB=0.06
- **原因**: RS_FF_SCALE=1.0 在低惯量电机上产生硬前馈振荡；速度环 Ki 引起自持振荡；sign protect 单周期归零误杀正常瞬态；需要 ABC 域逐相诊断能力。

### 2026-06-20: BEMF 量纲/符号修复 + 电流环运行时诊断与开关
- **子系统**: FOC 核心, FOC 应用层, UART 上传
- **文件**: `foc_core.h`, `foc_core.c`, `foc_app.c`, `stm32h7xx_it.c`, `uart_upload.c`
- **摘要**:
  1. BEMF 前馈新增运行时开关 (`CMD:BEMF_CFG,0/1`, `CMD:BEMF_CFG?`, `CMD:KE_TEMP`)
  2. BEMF Ke 量纲修复：`bemf_Ke = motor_param.Ke / Pn` (电角速度基准)
  3. BEMF 符号修复：`speed_elec` 不再乘 `encoder_dir`
  4. 新增保护门禁：`|ωe×Ke| > 0.8×Vbus/√3` 自动阻止
  5. FAULT_DETAIL 新增 `CurrentLoopDiag` 和 `BEMF Ctrl` 诊断行
  6. 新增 `CMD:RS_FF_SCALE` 和 `CMD:ADC_ZERO` 诊断命令
  7. Vmax 修复为 `Vbus/√3`（原硬编码 24V/√3）
- **原因**: Ke=0.129 直接乘 ωe 产生 81V BEMF 导致电压饱和飞车；encoder_dir 乘入 speed_elec 翻转 BEMF 符号；缺少运行时 BEMF 控制手段和电流环诊断

### 2026-06-13: V5 Motion Speed 运行时配置
- **子系统**: FOC 应用层, UART 上传
- **文件**: `foc_app.h`, `foc_app.c`, `stm32h7xx_it.c`, `uart_upload.h`, `uart_upload.c`
- **摘要**: Motion speed/accel/cruise 从编译宏改为运行时字段，支持 UART 实时修改和查询
- **原因**: 12V/24V 不同母线电压需要不同速度上限

### 2026-06-13: FOC Feedforward Baseline v1
- **子系统**: FOC 核心, FOC 应用层
- **文件**: `foc_app.c`, `foc_core.c`, `motor_identify.c`
- **摘要**: P1 BEMF + P2 Inertia + P3 Friction + P0 Cogging 前馈全部就绪，位置模式误差 ≤1.0°
- **原因**: 完整前馈链路降低位置环负载

### 2026-04-02: 初始架构捕获
- **子系统**: 全部
- **文件**: `.claude/architecture/README.md`, `.claude/architecture/subsystems/*.md`
- **摘要**: 通过 project-arch skill 初始化架构文档
- **原因**: 项目架构文档化
