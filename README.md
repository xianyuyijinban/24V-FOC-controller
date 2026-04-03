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
- **Motor Parameter Identification / 电机参数识别**: Rs, Ld, Lq, Ke, J, Pn
- **Communication / 通信**: UART1 (115200 bps), Text Protocol / 文本协议
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

## Bench Bring-Up Note / 台架启动说明

- 当前固件处于临时台架恢复配置：`SystemClock_Config()` 已从外部 `HSE 25MHz` 切换到内部 `HSI 64MHz + PLL1`，目标仍保持 `SYSCLK = 480MHz`。
- 切换原因是当前硬件上 `HSE` 未就绪，固件如果继续依赖外部晶振会在时钟初始化阶段直接停在 `Error_Handler()`，后续外设根本起不来。
- `FDCAN` 目前通过 `FOC_DEBUG_DISABLE_FDCAN_INIT=1U` 临时跳过初始化，避免 bench 启动链继续依赖外部晶振；本轮台架调试保留 `TIM1 / ADC / SPI / UART / GPIO` 主链路。
- 待外部晶振链路修复并重新验证 CAN 位时序后，再恢复 `HSE + FDCAN` 正常配置。
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
| SPI3_MISO | PC11 | TLE5012 Data Out |
| SPI3_MOSI | PC12 | TLE5012 Data In |
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
- 三相电流通道采样时间为 `32.5 cycles`，母线电压通道采样时间为 `16.5 cycles`。
- NVIC 使用 `NVIC_PRIORITYGROUP_4`，并保持 `DMA1_Stream2_IRQn < TIM1_UP_IRQn < SPI DMA/IRQ`，设计目标是 `TIM1_CH4 -> ADC DMA完成 -> TIM1控制ISR`。
- `TIM1` 控制环现在只消费“当前控制周期内完成”的 ADC 帧；单次缺帧会计数并上报，连续缺帧会升级为 `FOC_FAULT_ADC_SAMPLING`。
- UART 状态/故障上传现在包含 ADC 帧序号、帧年龄、缺帧计数、无效窗口计数、原始电流 ADC 值以及换算后的 `Ia/Ib/Ic/Vbus`。

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
- `Advanced Control`
  - 力矩模式 `Id_ref / Iq_ref`
  - 速度模式 `speed`
  - 位置模式 `position`
  - 本地 preset 保存/加载
- `PI Parameters`
  - 电流环 / 速度环 / 位置环 `Kp / Ki`
  - 本地默认参数加载与 preset 保存/加载

当前 GUI 使用用户本地 JSON 配置文件保存：
- 最近使用的串口、波特率、模式
- 常用目标值
- PI 参数默认值

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
- **Baud Rate / 波特率**: 115200 bps
- **Data Bits / 数据位**: 8
- **Parity / 校验**: None
- **Stop Bits / 停止位**: 1

### Command Format / 命令格式

| Command / 命令 | Format / 格式 | Description / 说明 |
|---------------|--------------|-------------------|
| Unlock | `CMD:UNLOCK,1` | Unlock power-stage arming / 解锁功率级动作许可 |
| Lock | `CMD:UNLOCK,0` | Relock and force disable / 重新上锁并强制下电 |
| Enable | `CMD:ENABLE,1` | Enable motor / 使能电机 |
| Disable | `CMD:ENABLE,0` | Disable motor / 禁用电机 |
| Set Mode | `CMD:MODE,n` | 0=Torque, 1=Speed, 2=Position |
| Set Current | `CMD:IREF,id,iq` | Set Id_ref, Iq_ref |
| Set Speed | `CMD:SREF,speed` | Set speed target (rad/s) |
| Set Position | `CMD:PREF,pos` | Set position target (rad) |
| Identify | `CMD:IDENTIFY,1` | Start parameter identification |
| Clear Fault | `CMD:CLEAR_FAULT` | Clear fault status |
| Set Current PI | `CMD:PI_CURRENT,kp,ki` | Set current loop PI |
| Set Speed PI | `CMD:PI_SPEED,kp,ki` | Set speed loop PI |
| Set Position PI | `CMD:PI_POS,kp,ki` | Set position loop PI |

Notes / 说明:
- Each command must end with newline (`\n` or `\r\n`) for parsing.
- 命令需以换行结束（`\n` 或 `\r\n`）才能被固件解析。
- Power stage is locked after boot; send `CMD:UNLOCK,1` first, then `CMD:ENABLE,1` or `CMD:IDENTIFY,1`.
- 上电后功率级默认锁定；需先发送 `CMD:UNLOCK,1`，再发送 `CMD:ENABLE,1` 或 `CMD:IDENTIFY,1`。
- Before unlock, `CMD:ENABLE,1` and `CMD:IDENTIFY,1` are ignored by firmware.
- 在解锁前，固件会忽略 `CMD:ENABLE,1` 与 `CMD:IDENTIFY,1`。

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

### Bench Bring-Up Update / 台架启动更新 (2026-04-03)

- 时钟启动临时改为 `HSI 64MHz + PLL1`，保持 `SYSCLK 480MHz`，绕开当前 `HSE` 不起振导致的上电死停问题。
- 新增 `FOC_DEBUG_DISABLE_FDCAN_INIT` 启动门控，在外部晶振恢复前跳过 `FDCAN` 初始化，避免无关外设拖垮 bench 启动链。
- `test_build_system.py` 已新增源码契约，锁定 `HSI` 启动和 `FDCAN` 门控，防止后续改动把这条恢复路径悄悄回退。

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
