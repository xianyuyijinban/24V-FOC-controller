# Process Log

记录本轮台架恢复过程中确认过的代码级问题、修复动作和仍待继续定位的运行日志。

## [2026-04-05 19:10] ADC 零点校准启动死锁
- 类型: 已修复的代码逻辑问题
- 现象: 复位后约 `200 ms` 抓到 `PC=0x08009dc8`，固件停在 `ADC_Sampling_Calibrate()` 内等待 ADC 帧，主循环无法继续进入外设联调阶段。
- 根因: `ADC_Sampling_Calibrate()` 运行时，`ADC1` 触发源已经切到 `TIM1_TRGO2=OC4REF`，但 `TIM1` 计数器和 `CH4` 比较输出尚未启动，导致校准阶段永远收不到有效 DMA 采样帧。
- 修复: 在 `ADC_Sampling_Init()` 后立即启动 `HAL_TIM_Base_Start(&htim1)` 和 `HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4)`，仅用于提供 ADC 触发；零点校准完成后不再调用 `HAL_TIM_Base_Start_IT()`，而是改为 `__HAL_TIM_CLEAR_FLAG()` + `__HAL_TIM_ENABLE_IT()` 单独打开 `TIM1 UPDATE IRQ`，避免 HAL TIM 状态机二次启动报错。
- 验证: `test_build_system.py::test_startup_primes_tim1_oc4_before_adc_zero_calibration` 已覆盖启动顺序；板上通过 `pyocd commander` 复位后继续运行，程序不再卡死在 ADC 零点校准阶段。

## [2026-04-05 19:20] UART 故障快照可能被截断或卡死
- 类型: 已修复的代码逻辑问题
- 现象: 最坏情况下故障文本长度达到 `1075 bytes`，超过原 `1024 bytes` 发送缓冲区；在线抓调时，故障路径还会长时间停在 `_printf_i` / `memmove`，导致 `s_lastFaultFlags` 和上传统计长时间不更新。
- 根因: 故障文本缓冲区容量不足，且 `DrvUart_FormatFault()` 在故障态使用大量浮点 `printf`，在高频中断和 DMA 压力下会拖慢首包发送。
- 修复: 将 `DRV_UART_BUF_SIZE` 扩大到 `1536`，并把故障文本格式化改成整数快路径，保留关键原始值和诊断字段，去掉故障态浮点格式化热点。
- 验证: `test_build_system.py::test_uart_fault_packet_buffer_covers_worst_case_fault_dump` 和 `test_uart_fault_formatter_avoids_float_printf_in_fault_path` 已覆盖缓冲区与格式化契约；板上观测到 `totalUploads=2`、`faultUploads=1`、`txErrors=0`，说明故障包已实际发送。

## [2026-04-05 19:35] 当前未解决的台架运行日志
- 类型: 待继续定位
- UART1: 固件侧寄存器与 DMA 接收状态正常，`BRR=0x412`，`gState=0x20`，`RxState=0x22`，当前没有发现 UART1 初始化级异常。
- SPI3 / TLE5012E: 传输在跑，但 `crc_error=1`、`data_valid=0`、`raw_angle=0`；历史抓取 `tle5012_rx_buf=[0x8021, 0x0000, 0x8021]`，更像读回无效帧或 MOSI 回显，当前编码器数据仍不可用。
- SPI1 / DRV8350S: DMA 和轮询计数持续增长，`commCount=19840`、`errorCount=0`，但关键寄存器读回长期为 `0x07ff`，`faultFlags=0x00ff07ff`，属于明显的无效状态读回，不像真实驱动状态。
- 系统影响: `g_foc_app` 当前落在 `FOC_STATE_FAULT`，`fault_code=FOC_FAULT_DRV8350S`，编码器相关角度/速度字段仍为零，电机闭环无法进入有效运行。
- 下一步建议: 优先继续比对 `SPI1/SPI3` 的片选时序、帧格式、字长、CPOL/CPHA 和器件回包语义；从现有日志看，当前主阻塞点已集中在两条 SPI 外设链路，而不是 `UART1` 或 `ADC/TIM1` 启动链。
