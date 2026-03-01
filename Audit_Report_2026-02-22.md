# 24V FOC Controller 审查与调试报告（2026-02-22）

## 1. 执行范围与说明
- 目标目录（原始）：`D:\STM32CubeMXProject\item\24V FOC Controller`
- 受当前执行环境写权限限制，无法直接写入 `D:`，本次在副本目录执行并验证：
  - `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222`
- 已完成：代码审查、可执行测试、缺陷修复、回归验证。

## 2. 审查发现与处理结果

### 严重问题（已修复）
1. 构建脚本“假成功”，实际链接失败未被检测  
   - 现象：`build.ps1` 显示 `Link [OK]`，但无 `.elf`。  
   - 根因：仅靠 `try/catch`，未检查外部命令退出码；同时使用了不兼容 GNU 的 Keil startup 文件和重复 HAL 模板源。  
   - 修复：重写 `build.ps1`，统一命令退出码检查、改用 GNU startup、移除模板源、编译失败时输出真实错误。  
   - 位置：`build.ps1:26`, `build.ps1:145`, `build.ps1:203`

2. Flash 写入存在越界读取（OOB Read）  
   - 现象：`Param_WriteFlash` 按 32 字节块写入，最后一块可能从结构体边界外读取。  
   - 根因：按块写入时直接用源指针偏移，未处理尾块不足 32 字节。  
   - 修复：尾块使用 32 字节临时缓冲（`0xFF` 填充）并复制有效数据后写入。  
   - 位置：`MDK-ARM/code/param_storage.c:264`, `MDK-ARM/code/param_storage.c:282`

3. 参数识别状态机逻辑错误，步骤会被跳过  
   - 现象：识别函数“进行中”也返回 `MI_ERR_NONE`，状态机立即跳转到下一步。  
   - 根因：缺少“进行中”状态码。  
   - 修复：新增 `MI_ERR_IN_PROGRESS`，并在状态机中单独处理；仅在真正完成时跳转。  
   - 位置：`MDK-ARM/code/motor_identify.h:54`, `MDK-ARM/code/motor_identify.c:51`

4. UART 文本格式化存在潜在缓冲区越界风险  
   - 现象：多次 `snprintf` 直接累加长度，截断时可能导致剩余长度计算错误。  
   - 根因：未统一检查每次追加后的边界状态。  
   - 修复：引入 `DrvUart_Append` 安全追加函数，格式化路径全部走边界检查。  
   - 位置：`MDK-ARM/code/uart_upload.c:292`, `MDK-ARM/code/uart_upload.c:127`, `MDK-ARM/code/uart_upload.c:180`

### 中等问题（已修复）
1. FOC 电流采样链路与校准链路不一致  
   - 现象：控制中断直接读原始 `adc_data` 并用固定公式换算，绕过 `ADC_Sampling` 校准数据。  
   - 修复：改为读取 `ADC_Sampling_GetData()` 中的校准后三相电流和母线电压；增加编码器数据有效性保护。  
   - 位置：`MDK-ARM/code/foc_app.c:128`, `MDK-ARM/code/foc_app.c:152`

2. 速度环重启首周期可能出现尖峰  
   - 现象：`theta_prev` 未在非运行态重置，重启后首个微分值可能异常。  
   - 修复：加入速度环“就绪”逻辑，运行首周期仅同步角度不计算速度。  
   - 位置：`MDK-ARM/code/foc_app.c:208`

3. 上位机解析器与固件输出格式不一致  
   - 现象：`Id_ref/Iq_ref`、故障包起始标记、故障状态字段解析不完整。  
   - 修复：支持正常包+故障包双起始标记，补齐 `Id_ref/Iq_ref` 与 `FAULT1/VGS2/Status` 解析。  
   - 位置：`HostComputer/data_parser.py:46`, `HostComputer/data_parser.py:126`, `HostComputer/data_parser.py:153`

4. `DrvUart_UploadImmediate` 存在死等风险  
   - 现象：等待 DMA 忙标志时无超时。  
   - 修复：增加 100ms 超时退出并计入错误计数。  
   - 位置：`MDK-ARM/code/uart_upload.c:403`, `MDK-ARM/code/uart_upload.c:414`

### 残留风险（未在本轮实现）
1. 串口命令协议在固件侧未发现接收解析实现  
   - 证据：仅见 DMA 接收启动，未见 `HAL_UART_RxCpltCallback` 命令分发路径。  
   - 位置：`Core/Src/main.c:121`, `Core/Src/stm32h7xx_it.c:478`  
   - 影响：README/架构文档中列出的 `CMD:*` 控制路径可能无法实际闭环。  
   - 建议：新增命令接收状态机（含帧边界、校验、容错）并补回归测试。

## 3. 功能测试与结果

### 3.1 固件编译测试
1. 快速编译测试  
   - 命令：`./build_test.ps1`  
   - 结果：通过（8/8 成功）

2. 全量构建测试  
   - 命令：`./build.ps1`  
   - 结果：通过，生成：
     - `build/gcc/24V_FOC_Controller.elf`
     - `build/gcc/24V_FOC_Controller.hex`
     - `build/gcc/24V_FOC_Controller.bin`
   - 体积：`text=55184, data=112, bss=5360, dec=60656 (0xECF0)`

3. 编译告警扫描（项目核心源码，`-Wall -Wextra`）  
   - 结果：`TOTAL_WARNINGS:0`

### 3.2 上位机协议测试
1. 单元测试  
   - 命令：`python -m unittest -v test_data_parser.py`  
   - 用例：4  
   - 结果：全部通过  
   - 覆盖：
     - 正常包解析
     - 故障包解析
     - 分片输入拼包
     - 命令构建器输出
   - 测试文件：`HostComputer/test_data_parser.py:41`

### 3.3 边界/限制说明
- 本轮未进行上电实机测试（电流环动态、过流保护触发、电机参数识别实转验证），需在硬件台架完成。
- 本轮未验证 `make` 路径（当前环境无 `make` 可执行），但 `Makefile` 已按 GNU startup 与源清单修正。

## 4. 修复清单（代码改动）
- `build.ps1`
- `Makefile`
- `Core/Src/main.c`
- `MDK-ARM/code/foc_app.c`
- `MDK-ARM/code/motor_identify.h`
- `MDK-ARM/code/motor_identify.c`
- `MDK-ARM/code/param_storage.c`
- `MDK-ARM/code/uart_upload.c`
- `HostComputer/data_parser.py`
- `HostComputer/test_data_parser.py`（新增）

## 5. 同步建议
- 将上述文件从副本目录同步回原目录 `D:\STM32CubeMXProject\item\24V FOC Controller`。
- 同步后优先执行：
  1. `.\build_test.ps1`
  2. `.\build.ps1`
  3. 台架实测（使能/禁用、速度模式、故障注入、参数识别）
