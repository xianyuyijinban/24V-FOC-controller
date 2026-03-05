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
| USART1_TX | PB14 | Serial TX |
| USART1_RX | PB15 | Serial RX |
| ADC1_INP17 | PA1 | Phase U Current |
| ADC1_INP14 | PA2 | Phase V Current |
| ADC1_INP15 | PA3 | Phase W Current |
| ADC1_INP4 | PC4 | Bus Voltage |

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
│       ├── tle5012.h/c   # TLE5012B编码器驱动
│       ├── drv8350s.h/c  # DRV8350S栅极驱动
│       └── uart_upload.h/c     # UART数据上传
├── Drivers/              # HAL and CMSIS drivers / HAL和CMSIS驱动
├── HostComputer/         # Host PC software / 上位机软件
│   ├── data_parser.py    # 数据解析器
│   └── requirements.txt  # Python依赖
├── Makefile              # GNU Make build file / GNU Make编译文件
├── build.ps1             # PowerShell build script / PowerShell编译脚本
└── README.md             # This file / 本文件
```

---

## Host Computer Software / 上位机软件

The host computer software is in a separate repository:

上位机软件在单独的仓库中：

🔗 **https://github.com/xianyuyijinban/24V-FOC-Controller-Host**

Features / 特性:
- Real-time waveform display / 实时波形显示
- Serial communication / 串口通信
- Motor parameter identification / 电机参数识别
- Three-loop control configuration / 三环控制配置
- English/Chinese language support / 中英文切换

### Local Data Parser / 本地数据解析器

项目包含一个Python数据解析器 (`HostComputer/data_parser.py`)，用于解析下位机上传的文本格式数据：

```python
from data_parser import FOCDataParser, FOCDataPacket

parser = FOCDataParser()
# 设置数据包接收回调
parser.set_packet_callback(on_packet_received)
# 喂入串口数据
parser.feed_data(serial_data)
```

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
- Power stage is disabled after boot; send `CMD:ENABLE,1` to start PWM/gate drive.
- 固件上电默认关闭功率级；需发送 `CMD:ENABLE,1` 才会启动PWM与栅极驱动。

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
