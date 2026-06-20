# 24V FOC Controller 架构

## 概述

基于 STM32H743 + TLE5012B + DRV8350S 的磁场定向控制（FOC）电机驱动器。支持力矩/速度/位置三环控制、自动参数识别（Rs/Ls/Ke/J/Pn/encoder_dir）、前馈补偿（BEMF/惯量/摩擦/齿槽），通过 UART 与上位机（PyQt6）通信。

## 系统上下文

```mermaid
graph TB
    USER[用户 / 操作者]
    GUI[上位机 PyQt6 GUI<br/>COM9 230400bps]
    FOC[24V FOC Controller<br/>STM32H743VIT6]
    MOTOR[永磁同步电机<br/>Pn=11, 24V]
    ENC[TLE5012B 磁编码器<br/>SPI3]
    DRV[DRV8350S 栅极驱动<br/>SPI1]
    PSU[12V/24V 直流电源]

    USER -->|操作指令| GUI
    GUI -->|UART CMD: N/C/F帧| FOC
    FOC -->|遥测数据| GUI
    FOC -->|SPI 配置/回读| DRV
    DRV -->|三相 PWM| MOTOR
    MOTOR -->|旋转磁场| ENC
    ENC -->|SPI 角度数据| FOC
    PSU -->|Vbus| FOC
```

## 高层组件图

```mermaid
graph TB
    subgraph 上位机层
        GUI_APP[gui_app.py<br/>启动入口]
        MAIN_WIN[main_window.py<br/>PyQt6 主窗口]
        GUI_LOGIC[gui_logic.py<br/>状态管理/命令构建]
        PARSER[data_parser.py<br/>N/F/C帧解析]
        SERIAL_WORKER[serial_worker.py<br/>QThread 串口]
    end

    subgraph 应用层
        FOC_APP[foc_app.c/h<br/>状态机/速度环/位置环/保护]
        MI[motor_identify.c/h<br/>参数识别状态机]
        UART[uart_upload.c/h<br/>遥测编码/命令分发]
        PARAM[param_storage.c/h<br/>Flash参数持久化]
    end

    subgraph 核心算法层
        FOC_CORE[foc_core.c/h<br/>Clarke/Park/SVPWM<br/>电流环PI/BEMF前馈]
        ADC_SAMP[adc_sampling.c/h<br/>ADC时序/低边采样<br/>电流零点校准]
    end

    subgraph 硬件驱动层
        TLE[tle5012.c/h<br/>TLE5012B SPI读取<br/>CRC校验/角度计算]
        DRV8350[drv8350s.c/h<br/>DRV8350S SPI配置<br/>故障回读/栅极控制]
    end

    subgraph HAL层
        STM32_CORE[Core/Src/<br/>main/gpio/adc/tim<br/>spi/usart/dma]
        HAL_DRV[Drivers/STM32H7xx<br/>HAL库]
    end

    GUI_APP --> MAIN_WIN
    MAIN_WIN --> GUI_LOGIC
    GUI_LOGIC -->|CommandBuilder| SERIAL_WORKER
    SERIAL_WORKER -->|UART| UART
    UART -->|FOCDataPacket| PARSER
    PARSER -->|packet_callback| MAIN_WIN
    MAIN_WIN -->|更新显示| GUI_LOGIC

    UART -->|CMD解析| FOC_APP
    UART -->|CMD解析| MI
    FOC_APP -->|控制循环| FOC_CORE
    FOC_APP -->|速度/位置环| FOC_APP
    MI -->|注入波形| FOC_CORE
    FOC_APP -->|读写参数| PARAM
    MI -->|读写参数| PARAM
    FOC_CORE -->|相电流| ADC_SAMP
    FOC_CORE -->|电角度/电角速度| TLE
    FOC_APP -->|配置/回读| DRV8350
    FOC_APP -->|PWM输出| STM32_CORE
    ADC_SAMP -->|ADC触发| STM32_CORE
    TLE -->|SPI| STM32_CORE
    DRV8350 -->|SPI| STM32_CORE
```

## 子系统索引

| 子系统 | 文件 | 描述 |
|--------|------|------|
| FOC 核心 | `subsystems/foc-core.md` | Clarke/Park/SVPWM、电流环PI、BEMF前馈 |
| FOC 应用层 | `subsystems/foc-app.md` | 状态机、速度/位置环、故障保护 |
| 参数识别 | `subsystems/motor-identify.md` | Rs/Ls/Ke/J/Pn/encoder_dir 自动识别 |
| ADC 采样 | `subsystems/adc-sampling.md` | ADC时序控制、低边采样、电流校准 |
| UART 上传 | `subsystems/uart-upload.md` | N/C/F帧编码、FAULT_DETAIL、命令分发 |
| TLE5012 编码器 | `subsystems/tle5012.md` | SPI读取、CRC校验、角度/速度计算 |
| DRV8350S 驱动 | `subsystems/drv8350s.md` | SPI配置、故障回读、栅极控制 |
| 参数存储 | `subsystems/param-storage.md` | Flash读写、版本管理、CRC校验 |
| 上位机 | `subsystems/host-computer.md` | PyQt6 GUI、数据解析、实时波形 |
| STM32 HAL | `subsystems/stm32-hal.md` | 外设配置、ISR处理、DMA传输 |

## 技术栈

| 层 | 技术 | 用途 |
|----|------|------|
| MCU | STM32H743VIT6 (Cortex-M7, 480MHz) | FOC 实时控制 |
| 固件语言 | C11 (GCC ARM 10.3) | 控制算法、驱动 |
| 构建 | PowerShell build.ps1 + pyOCD CMSIS-DAP | 编译、烧录 |
| 编码器 | TLE5012B (SPI, 16bit) | 角度/速度反馈 |
| 栅极驱动 | DRV8350S (SPI) | 三相逆变器控制 |
| 上位机 | Python 3 + PyQt6 + pyqtgraph | GUI 监控与控制 |
| 串口 | UART 230400bps, DMA | 上下位机通信 |
| 参数存储 | STM32 Flash Bank2 Sector7 (0x081E0000) | 电机参数持久化 |

## 关键架构决策

### 2026-06-20: BEMF 量纲/符号修复 + 运行时诊断开关
- `motor_param.Ke` 语义固定为机械侧 Kt/Ke_mech (V·s/rad mechanical)
- BEMF 前馈使用 `Ke/Pn` 电角速度基准值
- `speed_elec` 不乘 `encoder_dir`（物理 BEMF 方向与坐标变换独立）
- BEMF 默认关闭，需显式使能
- 电流环电压分解诊断（CurrentLoopDiag/BEMF Ctrl）作为标准 FAULT_DETAIL 输出

### 2026-06-13: V5 Motion Speed Runtime Configuration
- 运动配置（speed/accel/cruise）改为运行时字段，支持 CMD:MOTION_CFG 即时修改
- 默认 speed=4.0 rad/s（24V）/ 1.5 rad/s（12V 安全上限）

### 2026-06-13: FOC Feedforward Baseline v1
- P1 BEMF + P2 Inertia + P3 Friction + P0 Cogging 全部就绪
- 位置模式 PREF ±20° 误差 ≤1.0°

### 2026-04-02: 初始架构捕获
通过 project-arch skill 初始创建。
