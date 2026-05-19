# 24V FOC Controller - STM32H743 Firmware
# 24V FOC控制器 - STM32H743 固件

Field-Oriented Control (FOC) motor driver for joint servo applications based on STM32H743VIT6.

基于 STM32H743VIT6 的关节伺服 FOC 电机驱动固件。

---

## Features / 特性

- **MCU**: STM32H743VIT6 (ARM Cortex-M7, 480MHz)
- **Control Algorithm**: Field-Oriented Control (FOC) / 磁场定向控制
- **Control Modes / 控制模式**:
  - Torque Control (Current Loop) / 力矩控制（电流环）
  - Speed Control (Speed + Current Loop) / 速度控制（速度+电流环）
  - Position Control (Position + Speed + Current Loop) / 位置控制（位置+速度+电流环）
- **Motor Parameter Identification / 电机参数识别**: configured Pn + Rs, Ld/Lq fallback, Ke, J, encoder direction/zero verification / 手动配置极对数，并识别/验证 Rs、电感默认值、Ke、J、编码器方向与零位
- **Communication / 通信**: UART1 (230400 bps), Compact `N/F` ASCII frames + detailed fault text / 紧凑 `N/F` ASCII 帧 + 详细故障文本
- **Encoder**: TLE5012B (SPI interface) / TLE5012B 磁编码器
- **Driver**: DRV8350S (Three-phase gate driver) / DRV8350S 三相栅极驱动
- **PWM Frequency / PWM频率**: 20kHz
- **Control Loop Frequency / 控制频率**:
  - Current Loop: 20kHz
  - Speed Loop: 2kHz
  - Position Loop: 200Hz

---

## Hardware Specifications / 硬件规格

| Component / 组件 | Model / 型号 | Specification / 规格 |
|-----------------|-------------|---------------------|
| MCU | STM32H743VIT6 | 480MHz, Cortex-M7 |
| Gate Driver | DRV8350S | 100V, 1.5A/2.5A gate drive |
| MOSFETs | NCEP15T14D | 150V, 140A |
| Encoder | TLE5012B | 15-bit, SPI interface |
| Current Sensors | Low-side shunt resistors | 0.005Ω |
| Power Supply | 24V DC | 5-30V input range |

---

## Current Baseline / 当前基线

- Baseline date: 2026-05-06.
- Bench motor constants: 12V supply, 8.8 ohm phase resistance, 11 pole pairs, 74KV.
- Current sampling contract: low-side sampling frontend is inverted in firmware through `ADC_CURRENT_POLARITY = -1.0f`.
- Control baseline: torque, speed, and position modes share the same user-frame direction convention; speed and position defaults are conservative for the 12V bench motor.
- Current-loop tuning command contract: `CMD:PI_CURRENT,kp,ki` treats `ki` as continuous-time Ki and firmware divides it by `FOC_CONTROL_FREQ` before storing the per-sample PI gain.
- Position command ordering: UART command queue keeps `CMD:MODE,2` and `CMD:PREF,<rad>` in FIFO order, so a position reference cannot jump ahead of mode selection and be overwritten by position-mode hold seeding.
- Host GUI baseline: default pole-pair input is 11, 12V voltage limits are 9.0V/16.0V, and boot/diagnostic text lines are shown without blocking compact telemetry parsing.
- Verified locally with pytest and Keil ARMCC5. Keil flash log for this baseline reported `Erase Done`, `Programming Done`, `Verify OK`, and `Application running`.

---

## 2026-05-16 Sync Notes

- Host parser now accepts detailed diagnostic snapshots headed by either `FOC Diagnostic Snapshot` or the legacy `FAULT DETECTED` banner, and it only marks a packet as an active fault when the payload contains a real fault source.
- Detailed snapshots can populate motor parameters in the Identify tab: Rs, Ld, Lq, Ke, pole pairs, and encoder direction. The GUI falls back to the known 12V bench motor constants when firmware has not reported fresh values yet.
- The Identify tab uses a compact status/parameter/action layout and requests `CMD:FAULT_DETAIL` after connection and after identification completes, so the latest firmware-side parameter snapshot is pulled into the host view.
- Runtime telemetry packets no longer flood the RX log. Log rendering is batched on a timer, while fault details remain separated in the fault log.
- Current, speed, and position target commands are blocked unless the motor is both unlocked and enabled, reducing accidental motion commands during setup.
- Firmware fault-detail formatting now emits a non-fault diagnostic title when no shutdown fault is active, while preserving the fault banner and action checklist for real faults.
- Repository hygiene: generated DOCX render images and local Keil output text files are ignored; project documentation for upload should be consolidated into this root `README.md`.

---

## Bench Bring-Up Note / 台架启动说明

- 当前主固件优先使用 `25MHz HSE + PLL1`；若外部晶振启动失败，会自动回退到 `HSI 64MHz + PLL1`，目标仍保持 `SYSCLK = 480MHz`，优先保证复位后能进入主循环并保持 `UART1` 可调试。
- `FDCAN` 继续通过 `FOC_DEBUG_DISABLE_FDCAN_INIT=1U` 临时跳过初始化；本轮台架重点只看 `SPI1 / SPI3 / UART1` 主链路，`I2C/CAN` 不参与联调。
- 当前板的 `USART1 <-> CH340` 连接需要开启 `USART1` advanced-feature `SWAP`，不要在未改板前把该配置关掉。
- 若 `DRV8350S` 启动期 SPI 读写失败，主固件现在会保留 `UART1` 和主循环继续运行，并通过故障上传报告 `DRV8350S` 通信故障；不要再把这类启动期外设异常直接做成 `Error_Handler()` 死循环，否则上位机将完全收不到包。
- 同样地，`DrvUart_Init()` 之后若 `USART1 RX DMA/IDLE`、`ADC` 校准、`TIM1 Base/OC4` 触发链或 `ADC DMA` 启动失败，主固件也不再直接卡死；会继续保留 `UART1 TX` 和主循环，用故障包把启动失败原因上传到上位机。
- 当前推荐烧录/调试探头：`CMSIS-DAP`（SWD）。

---

## Pinout / 引脚定义

| Function / 功能 | Pin / 引脚 | Description / 说明 |
|----------------|-----------|-------------------|
| TIM1_CH1 | PE9 | PWM Phase U High-side |
| TIM1_CH1N | PE8 | PWM Phase U Low-side |
| TIM1_CH2 | PE11 | PWM Phase V High-side |
| TIM1_CH2N | PE10 | PWM Phase V Low-side |
| TIM1_CH3 | PE13 | PWM Phase W High-side |
| TIM1_CH3N | PE12 | PWM Phase W Low-side |
| DRV8350S_nSCS | PA4 | DRV8350S SPI Chip Select |
| DRV8350S_ENABLE | PE14 | DRV8350S Enable (DRV_EN) |
| SPI3_SCK | PC10 | TLE5012 Clock |
| SPI3_MISO / DATA | PC11 | TLE5012 shared DATA receive phase |
| SPI3_MOSI / DATA | PC12 | TLE5012 shared DATA command phase |
| TLE5012_NSS | PA15 | TLE5012 software chip select (active low) |
| MOD2 | PB12 | Local identify start/abort button (active low) |
| MOD1 | PB13 | Local demo-mode toggle button (active low) |
| USART1_TX | PB14 | Serial TX |
| USART1_RX | PB15 | Serial RX |
| ADC1_INP17 | PA1 | Phase U Current |
| ADC1_INP14 | PA2 | Phase V Current |
| ADC1_INP15 | PA3 | Phase W Current |
| ADC1_INP4 | PC4 | Bus Voltage |

### ADC Sampling Timing / ADC采样时序

- `TIM1_CH4` 仅作为内部采样参考，不驱动外部引脚。
- `TIM1_TRGO2 = OC4REF`，`ADC1` 常规组触发源改为 `TIM1_TRGO2`，避免继续使用粗粒度 `UPDATE` 触发。
- 上电启动顺序已改为：`ADC_Sampling_Init()` 后先启动 `TIM1 Base + CH4(OC4REF)` 供 ADC 零点校准取样，再在校准完成后单独打开 `TIM1 UPDATE IRQ`；整个预触发阶段不启动 PWM 输出，不会驱动功率级。
- 三相电流通道采样时间为 `32.5 cycles`，母线电压通道采样时间为 `16.5 cycles`。
- NVIC 使用 `NVIC_PRIORITYGROUP_4`，并保持 `DMA1_Stream2_IRQn < TIM1_UP_IRQn < SPI DMA/IRQ`，设计目标是 `TIM1_CH4 -> ADC DMA完成 -> TIM1控制ISR`。
- `TIM1` 控制环现在只消费“当前控制周期内完成”的 ADC 帧；单次缺帧会计数并上报，连续缺帧会升级为 `FOC_FAULT_ADC_SAMPLING`。
- UART 状态/故障上传现在包含 ADC 帧序号、帧年龄、缺帧计数、无效窗口计数、原始电流 ADC 值以及换算后的 `Ia/Ib/Ic/Vbus`。
- UART 故障首报路径使用 `1536B` 发送缓冲区，且故障格式化改为整数快路径，避免故障态浮点 `printf` 把首个大故障快照卡在串口发送前。
- TLE5012 编码器板 `CN2.5/CN2.6` 共用同一根 `DATA` 网；`PC12` 只在命令阶段驱动，`PC11` 只在响应阶段接收，方向切换通过直接修改 `PC11/PC12` 的 `MODER` 完成。
- UART 正常/故障文本会额外输出编码器 `Safety` 字节和 `Reset` 状态，用来暴露 TLE5012 `Safety Word bit15` 的复位/看门狗异常。

---

## Build Instructions / 编译说明

### Prerequisites / 前提条件

- GNU Arm Embedded Toolchain 14.2.Rel1 (or later)
- Make (Windows: via MinGW or MSYS2)
- PowerShell (Windows)

### Build Commands / 编译命令

```powershell
# Build using PowerShell script / 使用 PowerShell 脚本编译
.\build.ps1

# Or use Makefile directly / 或直接使用 Makefile
make all

# Clean build files / 清理编译文件
make clean

# Test build / 测试编译
.\build_test.ps1

# HostComputer parser unit tests / 上位机解析单测（可在仓库根目录直接执行）
python -m unittest HostComputer/test_data_parser.py

# Local host GUI / 本地上位机 GUI
python -m pip install -r HostComputer/requirements.txt
python -m HostComputer.gui_app

# Package the Host GUI into a Windows app / 将上位机GUI打包为Windows应用
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1
```

- 若使用仓库内 `MDK-ARM/24V FOC Controller.uvprojx` 通过 `Keil/ARMCC5` 重编译，不要再向 linker misc 人工追加 `--scanf_support=...`；当前工程直接使用标准 `sscanf()` 解析 `CMD:VBUS_LIMIT`、`IREF/SREF/PREF`、`PI_CURRENT/PI_SPEED/PD_POS` 等浮点串口命令，错误的 linker 选项只会触发 `L3900U` 导致目标文件无法生成。

---

## Project Structure / 项目结构

```
24V FOC Controller/
├── Core/
│   ├── Inc/              # Header files / 头文件 (STM32CubeMX生成)
│   │   ├── main.h        # 主程序头文件
│   │   ├── adc.h         # ADC配置
│   │   ├── tim.h         # 定时器配置
│   │   ├── spi.h         # SPI配置
│   │   └── ...
│   └── Src/              # Source files / 源文件 (STM32CubeMX生成)
│       ├── main.c        # 主程序入口
│       ├── adc.c
│       ├── tim.c
│       └── ...
├── MDK-ARM/
│   └── code/             # FOC核心代码
│       ├── head.h        # 头文件整合
│       ├── foc_core.h/c  # FOC核心算法
│       ├── foc_app.h/c   # FOC应用层
│       ├── motor_identify.h/c  # 电机参数识别
│       ├── param_storage.h/c   # 参数存储
│       ├── adc_sampling.h/c    # ADC采样处理
│       ├── demo_button_control.h/c  # 本地演示按钮控制
│       ├── tle5012.h/c   # TLE5012B编码器驱动
│       ├── drv8350s.h/c  # DRV8350S栅极驱动
│       └── uart_upload.h/c     # UART数据上传
├── Drivers/              # HAL and CMSIS drivers / HAL和CMSIS驱动
├── HostComputer/         # Host PC software / 上位机软件
│   ├── __init__.py       # HostComputer package入口
│   ├── data_parser.py    # 数据解析器与命令构建
│   ├── gui_app.py        # 本地GUI启动入口
│   ├── gui_logic.py      # GUI显示/状态映射逻辑
│   ├── main_window.py    # PyQt6主窗口 (HostMainWindow)
│   ├── serial_service.py # 可测试的串口服务核心
│   ├── serial_worker.py  # QThread串口worker
│   └── requirements.txt  # Python依赖
├── Makefile              # GNU Make build file / GNU Make编译文件
├── build.ps1             # PowerShell build script / PowerShell编译脚本
└── README.md             # This file / 本文件
```

---

## Host Computer Software / 上位机软件

### Local bench GUI / 仓库内本地调试GUI

本仓库现在包含一个可直接运行的本地 PyQt6 上位机调试工具，适合台架联调和故障诊断：

- 安装依赖：`python -m pip install -r HostComputer/requirements.txt`
- 启动 GUI：`python -m HostComputer.gui_app`
- 主要入口文件：`HostComputer/gui_app.py`
- 主窗口实现：`HostComputer/main_window.py`
- 串口线程与协议接线：`HostComputer/serial_worker.py`

当前本地 GUI 提供：
- `Debug Panel`
  - 串口端口枚举、连接、断开
  - `UNLOCK` / `LOCK` / `ENABLE` / `DISABLE` / `CLEAR FAULT`
  - 一键 `Unlock + Enable`、`Disable + Lock`、`Clear Fault + Re-arm Hint`
  - 模式切换、运行状态卡片、故障摘要、串口日志、可折叠实时曲线
- `Identify`
  - 连接/上锁状态提醒
  - `START IDENTIFY` / `STOP IDENTIFY` / `CLEAR FAULT`
  - 识别事件与状态快照日志
  - `已识别 / 未识别`、`堵转授权`、`开环试转激活态` 三个状态位
- `Advanced Control`
  - 力矩模式 `Id_ref / Iq_ref`
  - 速度模式 `speed`
  - 位置模式 `position`
  - 保护阈值：欠压/过压阈值下发与固件确认显示
  - ADC噪声测试：发送 `CMD:ADC_NOISE,n`，显示 `A/B/C/VBUS` 原始码 `min/max/mean/pp/std`
  - 本地 preset 保存/加载
- `Loop Parameters`
  - 电流环 / 速度环 `Kp / Ki`
  - `Position Loop PD`: 位置环 `Kp / Kd`
  - 本地默认参数加载与 preset 保存/加载

当前 GUI 使用用户本地 JSON 配置文件保存：
- 最近使用的串口、波特率、模式
- 常用目标值
- 电流/速度 PI 与位置 PD 默认值

默认配置文件路径：
- Windows: `%USERPROFILE%\\.24v_foc_host_gui.json`

已知限制：
- 识别进度目前基于已有命令与状态快照做页面占位，尚未解析更细粒度的固件识别阶段数据包。
- 曲线导出仅导出当前滚动缓存，不会自动长期录波。
- preset 目前是单份本地配置文件，不含多命名配置管理。

### Package as a Windows app / 打包为 Windows 应用

- 打包脚本：`build_host_gui_app.ps1`
- 打包工具：`PyInstaller`（由脚本自动安装）
- 输出目录：`dist/24V_FOC_Host/`
- 主程序：`dist/24V_FOC_Host/24V_FOC_Host.exe`
- 环境检查：如果 `PyInstaller / PyQt6 / pyqtgraph / pyserial / numpy` 已安装，脚本会直接 skip `pip install`
- 旧包清理：脚本在新包构建成功后，会自动删除历史目录 `dist_rebuilt/24V_FOC_Host/`
- 推荐流程：
  - `python -m unittest discover -s HostComputer -p "test_*.py" -v`
  - `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`

### Local data parser / 本地数据解析器

项目同时保留一个 Python 数据解析器 (`HostComputer/data_parser.py`)，用于解析下位机上传的文本格式数据：

```python
from data_parser import FOCDataParser, FOCDataPacket

parser = FOCDataParser()
# 设置数据包接收回调
parser.set_packet_callback(on_packet_received)
# 喂入串口数据
parser.feed_data(serial_data)
```

### External host repository / 外部上位机仓库

独立的 host 仓库仍可作为后续功能扩展参考，但不再是明天台架调试的唯一入口：

🔗 **https://github.com/xianyuyijinban/24V-FOC-Controller-Host**

---

## Communication Protocol / 通信协议

### UART Settings / 串口设置
- **Baud Rate / 波特率**: 230400 bps
- **Data Bits / 数据位**: 8
- **Parity / 校验**: None
- **Stop Bits / 停止位**: 1

### Upload Frames / 上传帧格式

- 正常实时遥测使用紧凑单行帧，normal telemetry interval is 20ms / 50Hz：`N,timestamp_ms,foc_state,angle_deg,speed_rad_s,Id,Iq,Vbus,fault_flags,encoder_detected,motor_identified,stall_mode_armed,stall_open_loop_active,app_warning_flags,app_fault_code,control_mode,Id_ref,speed_ref,pos_ref_rad,Iq_ref,Vd,Vq,Ia,Ib,Ic,identify_state,identify_error,undervoltage_limit_v,overvoltage_limit_v`
- 三相电流高速曲线使用轻量单行帧，phase current telemetry interval is 5ms / 200Hz：`C,timestamp_ms,Ia,Ib,Ic`
- 故障摘要使用紧凑单行帧：`F,timestamp_ms,foc_state,fault_flags,drv_comm_fault,encoder_detected,stall_open_loop_active,fault1,vgs2,last_rx,app_warning_flags,app_fault_code,identify_state,identify_error,Vbus,undervoltage_limit_v,overvoltage_limit_v`
- 详细故障报告仍保留原有人类可读文本格式，但只在新的故障边沿发送一次，并按小块分片上传，避免堵塞实时链路。
- 正常状态包也会额外上报 `app_warning_flags`；目前 bit0=`欠压告警`、bit1=`过压告警`。普通电压越限只作为告警显示，不再直接停机。
- `app_fault_code` 对应 `FOC_FaultCode_t`；即使 `DRV fault_flags == 0`，只要应用层进入 `FOC_STATE_FAULT`，上位机也必须把它显示为故障激活，而不是“正常”。
- 上位机解析 `N/F` 紧凑帧时，`undervoltage_limit_v / overvoltage_limit_v` 固定取末尾两个十进制字段；即使中间插入新的布尔状态位，也不允许把 UV/OV 读串位。

Examples / 示例:

```text
N,2612,3,12.34,5.67,0.120,0.456,11.98,0x00000000,1,0,1,0,18.00,28.00
F,2650,7,0x00000020,1,0,1,0x07FF,0x07FF,0xFFFF,11.98,18.00,28.00
```

### Command Format / 命令格式

| Command / 命令 | Format / 格式 | Description / 说明 |
|---------------|--------------|-------------------|
| Unlock | `CMD:UNLOCK,1` | Unlock power-stage arming / 解锁功率级动作许可 |
| Lock | `CMD:UNLOCK,0` | Relock and force disable / 重新上锁并强制下电 |
| Enable | `CMD:ENABLE,1` | Enable motor / 使能电机 |
| Disable | `CMD:ENABLE,0` | Disable motor / 禁用电机 |
| Set Mode | `CMD:MODE,n` | 0=Torque, 1=Speed, 2=Position |
| Stall Authorize | `CMD:STALL_MODE,0/1` | Authorize / cancel bench stall open-loop startup / 授权或取消堵转开环试转 |
| Set Current | `CMD:IREF,id,iq` | Set Id_ref, Iq_ref |
| Set Speed | `CMD:SREF,speed` | Set speed target (rad/s) |
| Set Position | `CMD:PREF,pos` | Set position target (rad) |
| Set Pole Pairs | `CMD:MOTOR_PN,pn` | Configure motor pole pairs before identification; valid range 1~50 |
| Identify | `CMD:IDENTIFY,1` | Start parameter identification and direction/zero verification |
| Set Vbus Limits | `CMD:VBUS_LIMIT,uv,ov` | Update runtime undervoltage / overvoltage warning thresholds |
| ADC Noise Test | `CMD:ADC_NOISE,n` | Capture raw ADC noise statistics while motor is not working; `n` is clamped to 16~4096 |
| TLE GPIO Diag | `CMD:TLE_GPIO_DIAG,0/1` | Stop / start slow GPIO toggling for TLE5012 CS/SCK/DATA line probing |
| Fault Detail | `CMD:FAULT_DETAIL` | Re-send the current detailed fault report, including active motor-identification diagnostics while identification is still running |
| Clear Fault | `CMD:CLEAR_FAULT` | Clear fault status |
| Set Current PI | `CMD:PI_CURRENT,kp,ki` | Set current loop PI; `ki` is continuous-time Ki and is converted to per-sample Ki in firmware |
| Set Speed PI | `CMD:PI_SPEED,kp,ki` | Set speed loop PI |
| Set Position PD | `CMD:PD_POS,kp,kd` | Set position loop PD |

Notes / 说明:
- Each command must end with newline (`\n` or `\r\n`) for parsing.
- 命令需以换行结束（`\n` 或 `\r\n`）才能被固件解析。
- Power stage is locked after boot; send `CMD:UNLOCK,1` first, then `CMD:ENABLE,1` or `CMD:IDENTIFY,1`.
- 上电后功率级默认锁定；需先发送 `CMD:UNLOCK,1`，再发送 `CMD:ENABLE,1` 或 `CMD:IDENTIFY,1`。
- Before unlock, `CMD:ENABLE,1` and `CMD:IDENTIFY,1` are ignored by firmware.
- 在解锁前，固件会忽略 `CMD:ENABLE,1` 与 `CMD:IDENTIFY,1`。
- 若电机未识别或编码器离线，上位机会在 `ENABLE` 前提示是否进入 `堵转模式（开环试转）`；确认后会下发 `CMD:STALL_MODE,1`。
- `堵转模式（开环试转）` 仍受欠压、过压、过流、DRV 故障和 ADC 采样故障保护；该模式下位置环不可用，建议先给小 `Iq_ref`，再给小 `speed` 试转。
- 若进入堵转模式前未显式下发 `speed_ref / Iq_ref`，固件会自动注入台架默认试转值：`speed_ref = 5.0 rad/s`，`Iq_ref = 0.5 A`，避免“已授权但零指令不转”的假死观感。
- `CMD:VBUS_LIMIT,uv,ov` 仅允许在电机未运行、未处于参数识别、PWM 未使能时修改阈值；这对值现在定义的是“普通电压告警阈值”。
- `CMD:MOTOR_PN,pn` 仅用于停机/未识别流程前配置极对数；固件不再通过短距离开环抖动自动反算 `Pn`，参数识别阶段只验证编码器方向、零位和双向真实运动。
- `CMD:ADC_NOISE,n` 仅允许在电机未运行、未处于参数识别、PWM 未使能时执行；固件不会驱动电机，只统计 `A/B/C/VBUS` 四路原始 ADC 码的 `min/max/mean/pp/std` 并一次性返回，例如 `CMD:ADC_NOISE,4096`。
- `CMD:TLE_GPIO_DIAG,1` 仅在 PWM 未使能且不处于运行/识别状态时接管 TLE5012 引脚：`PA15=CSQ`、`PC10=SCK`、`PC12=DATA_OUT` 以 500ms 步进低速翻转，`PC11=DATA_IN` 持续采样到 `tle5012_gpio_diag.data_in`；发送 `CMD:TLE_GPIO_DIAG,0` 后恢复 SPI3 三线 SSC 正常读取。
- TLE GPIO 诊断用于万用表/Keil Watch 排查线序：编码器板 `CN2.5/CN2.6` 共 DATA，因此 `PC12` 输出高低变化时，`PC11` 应跟随变化；若不跟随，优先查控制板连接器、线缆、编码器板 DATA 共网和 `R2 -> U1.4` 路径。
- ADC 噪声诊断输出的是原始码值：电流通道约 `5.37 mA/LSB`，母线电压约 `8.06 mV/LSB`；台架初筛可先看电流峰峰值是否小于 `5~15 LSB`，若超过 `20 LSB` 优先查模拟地、VDDA、采样时刻和运放噪声。
- 固件会在告警阈值之外再派生一组内部“严重停机阈值”：`严重欠压 = uv - 1.0V`，`严重过压 = ov + 1.0V`。只有越过严重阈值时才会真正下电进入 `FAULT`。
- 严重电压故障带 `0.5V` 自动恢复滞回：`严重欠压` 恢复到 `uv - 0.5V` 以上、`严重过压` 回落到 `ov + 0.5V` 以下后，固件会自动退出电压故障态并回到 `READY/IDLE`。
- `CMD:CLEAR_FAULT` 会先刷新实时 `Vbus`、编码器有效位和 `DRV8350S` 读回，再决定是否允许退出故障态；普通电压告警不会再阻止清故障或重新使能。
- 上位机默认也应使用 `230400` 波特率；旧的 `115200` 配置不再作为当前默认值。
- 上位机“高级控制 -> 保护阈值”面板中的输入框是待下发值；“当前阈值”标签显示固件实时上传的实际 `undervoltage_limit_v / overvoltage_limit_v`。
- 若已连接但尚未收到任何固件包，标签显示“等待固件回传”；若已收到包但阈值字段仍缺失，则显示“未上报阈值（请确认已烧录最新固件）”。

### Local Demo Buttons / 板载演示按钮

- `PB13 = MOD1`, `PB12 = MOD2`，两者均为低电平按下、主循环轮询消抖。
- 两个按钮都遵守与串口一致的解锁门禁；`power_unlocked == 0` 时不会触发功率级动作。
- `MOD1` 在两种本地 demo 模式间切换：
  - 第一次有效按下进入 `10 deg/s` 匀速旋转（`0.174533 rad/s`）。
  - 再次按下进入“回零弹簧”模式，目标为最近一次参数识别/对齐记录的机械零位。
- 回零弹簧模式使用力矩模式按角度误差线性增大恢复力，误差达到约 `120 deg`（`2.0943951 rad`）后饱和到最大演示电流。
- 若从失能/待机状态进入回零弹簧模式，固件会先等待一次新的上电后编码器角度样本，再输出第一笔恢复力矩，避免首拍使用旧角度。
- `MOD2` 在解锁后用于启动参数识别；若当前已在识别，则再次按下会中止识别。
- 参数识别/对齐结束后，固件会同时保存电角零位补偿 `theta_offset` 和该次对齐时的机械零位 `theta_mech_zero`；回零弹簧模式始终以 `theta_mech_zero` 为回零基准。

---

## Safety Notes / 安全注意事项

⚠️ **WARNING / 警告**

- This project involves high voltage (24V) and high current. Please ensure proper safety precautions.
- 本项目涉及高压(24V)和大电流，请确保安全操作。

- Always use appropriate PPE and follow electrical safety guidelines.
- 请使用适当的个人防护设备并遵循电气安全指南。

- Motor can spin at high speed. Keep clear during testing.
- 电机可能高速旋转，测试时请保持安全距离。

---

## Documentation / 文档

- [Project Architecture](Project_Architecture.md) - 详细的项目架构文档
- [Motor Parameter Identification Primer (ZH)](docs/Motor_Parameter_Identification_Primer_zh.md) - 参数识别原理与调试说明

### Recent Technical Updates / 近期技术更新 (2026-04-02)

- 低边分流采样重构：`ADC1` 触发源从 `TIM1 TRGO=UPDATE` 切换为 `TIM1 TRGO2=OC4REF`，将采样点移动到可控的低边导通窗口。
- ADC 帧交接加固：`adc_sampling` 新增控制周期窗口、帧序号、帧年龄、缺帧计数与无效窗口计数，`TIM1` 电流环只消费当前周期已提交的 ADC 帧。
- 采样故障策略落地：连续采样缺失会触发 `FOC_FAULT_ADC_SAMPLING`，避免闭环在不可信电流反馈上继续运行。
- UART 诊断增强：正常/故障上传包新增采样触发源、采样时间、帧序号、原始 ADC 三相电流和换算后的 `Ia/Ib/Ic/Vbus`。

### Bench Bring-Up Update / 台架启动更新 (2026-04-23)

- 时钟启动策略改为“`HSE` 优先，失败自动回退 `HSI 64MHz + PLL1`”，避免外部晶振偶发不起振时，固件在 `SystemClock_Config()` 内直接卡死而导致复位后完全无串口。
- `FOC_DEBUG_DISABLE_FDCAN_INIT` 继续保留，当前 bench 启动仍然只验证核心时钟与 `TIM1 / ADC / SPI / UART / GPIO` 主链路。
- 由于 `SYSCLK/AHB/APB/TIM1/SPI123` 目标频率保持不变，`TIM1 PSC/ARR` 和 `SPI1/SPI3` 波特率分频保持原值，不做额外补偿修改。
- `ADC` 零点校准前会先启动 `TIM1` 基计数器和 `CH4/OC4REF` 触发链，校准完成后再单独使能 `TIM1 UPDATE IRQ`；这样能让 ADC 先拿到采样帧，同时不提前驱动 PWM。
- UART 故障上报路径已加大缓冲区并移除故障态浮点格式化，保证首次故障和最坏情况故障快照也能稳定送到上位机。

### Recent Technical Updates / 近期技术更新 (2026-03-04)

- SPI1 访问互斥：阻塞式寄存器读写与TIM1异步DMA轮询已加总线互斥，避免并发竞争导致寄存器访问错乱。
- 故障中断路径去阻塞：TIM1故障分支改为快速下电，阻塞式DRV收尾移至主循环执行。
- Ke识别模型更新：改用静止坐标系反电势幅值 `|Vαβ - Rs*Iαβ|` 与实测 `ωe` 估算，提高开环识别阶段鲁棒性。
- FOC电压链路常数统一：`Vdq` 矢量限幅为 `Vbus/√3`，SVPWM归一化使用 `Vbus/2`，提升母线利用率一致性。
- UART故障首报修复：仅在故障包成功进入DMA发送后更新故障边沿标志，避免首次故障上报丢失。
- 串口命令复制改为“有界长度函数 + memcpy”，规避 `-Werror` 下 `strncpy truncation` 告警。
- FOC调制波接口声明补齐：`FOC_GetModulationWave()` 已在头文件导出。
- 上位机单测入口兼容：`HostComputer/test_data_parser.py` 现可在仓库根目录直接运行。
- DRV8350S 异步读失败路径补齐 `readReq.pending` 清理（DMA启动失败和DMA错误回调），避免后续阻塞式 SPI 因等待 `pending==0` 超时。

---

## License / 许可证

MIT License - See LICENSE file for details.

---

## Author / 作者

- **GitHub**: [@xianyuyijinban](https://github.com/xianyuyijinban)
- **Project**: 24V FOC Controller

---

## Acknowledgments / 致谢

- STMicroelectronics for HAL drivers
- SimpleFOC community for algorithm references
