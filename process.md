# PROCESS

## Mandatory Rules

1. Before each act/execution, read this file and remember prior mistakes and prevention controls.
2. Log an entry after any incident/problem or any major code/config/architecture change.
3. Every entry must include a real git commit SHA.
4. Recurrence of the same issue is not allowed; add concrete prevention controls.
5. Do not close the task before this file is updated.
6. Existing log content is append-only by default; do not delete, rewrite, truncate, or silently replace older entries unless the user explicitly asks for that exact change.

## Entry Template

```markdown
## [YYYY-MM-DD HH:MM] <short title>
- Problem: <what happened>
- Resolution: <how it was fixed>
- Prevention: <how to prevent recurrence>
- Commit: <full git commit SHA>
- Recurrence policy: Not allowed to happen again.
```

## Entries

## [2026-04-02 16:36] Local demo button modes and build wrapper closure
- Problem: The demo board needed unlocked-only local button control for `MOD1`/`MOD2`, but the firmware lacked button GPIO plumbing, a dedicated demo button module, and the minimal unlock/reset hooks required to switch modes or start/abort identify safely from the board. During final verification, `build.ps1` also falsely reported linker failure when native tools wrote non-fatal text to stderr.
- Resolution: Added `PB13/PB12` button definitions and GPIO input setup, introduced `demo_button_control.[ch]`, wired `MOD1` to toggle between `10 deg/s` speed mode and a zero-spring torque mode with `120 deg` saturation, wired `MOD2` to start/abort identify behind `power_unlocked`, added source-contract tests for the new behavior, updated `README.md`, and fixed `build.ps1` so `Invoke-AndCheck` tolerates native stderr when the exit code is zero.
- Prevention: Keep the new `test_build_system.py` contracts for button GPIO/behavior/spring mode plus the `build.ps1` stderr-wrapper regression in the repo, require fresh `python -m unittest -v test_build_system.py` and `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` evidence after touching local demo control or build plumbing, and preserve the unlock gate as a defense-in-depth check in both command and button paths.
- Commit: 2e74ccf09dca24b299dd7d81408ffd9f3d576085
- Recurrence policy: Not allowed to happen again.

## [2026-04-02 17:08] Zero-spring first torque waits for fresh angle sample
- Problem: The local zero-spring demo mode could command its first restoring `Iq_ref` immediately after `FOC_App_Enable()`, before `TIM1` had refreshed `theta_mech`, so the first torque update after enable could use a stale disabled-state angle sample.
- Resolution: Added a new source-contract test for post-enable angle freshness, introduced `theta_sample_seq` in `FOC_AppHandle_t`, incremented it on each TIM1 angle refresh, and gated zero-spring output in `demo_button_control.c` until the sequence advances past the pre-enable snapshot. Updated `README.md` to document that spring mode waits for a fresh post-enable encoder sample before outputting its first restoring torque.
- Prevention: Keep the new `test_zero_spring_waits_for_fresh_angle_sample_after_enable` contract in `test_build_system.py`, and whenever local demo torque behavior is touched, rerun both `python -m unittest -v test_build_system.py` and `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` before claiming the mode is deterministic.
- Commit: c2641a63db886423baff61774cfb8853896164be
- Recurrence policy: Not allowed to happen again.

## [2026-04-02 17:49] Zero-spring anchors to identified mechanical zero
- Problem: The local zero-spring demo mode used raw mechanical `0 rad` as its return target, but parameter identification only persisted electrical `theta_offset`. After encoder align, spring mode could therefore pull toward an arbitrary absolute encoder origin instead of the identified/aligned mechanical home.
- Resolution: Added persisted `theta_mech_zero` to `MotorParam_t`, stored it during `MI_EncoderAlign()`, bumped `PARAM_VERSION` so old flash payloads are rejected cleanly, switched `demo_button_control.c` to compute spring error from `theta_mech_zero - theta_mech`, and updated `README.md` plus `Project_Architecture.md` to document the separate electrical/mechanical zero semantics.
- Prevention: Keep `test_zero_spring_uses_identify_aligned_mechanical_zero` in `test_build_system.py`, and whenever demo spring behavior or identify/alignment persistence changes, rerun `python -m unittest -v test_build_system.py` and `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` before closing the task.
- Commit: 02410121151d0821761712cb6fbf9a133c4e26ab
- Recurrence policy: Not allowed to happen again.

## [2026-04-02 23:01] Local host debug GUI milestone
- Problem: The repository had only a parser-side host utility, so tomorrow's bench session still lacked a runnable in-repo GUI for serial connection management, command dispatch, runtime status visibility, and fault inspection.
- Resolution: Added a local PyQt6 host GUI package with `HostMainWindow`, a worker-backed serial path, pure-Python GUI presentation logic, command helpers for `CMD:UNLOCK`, host-side unit tests, and README/architecture updates documenting `python -m HostComputer.gui_app` as the local startup path.
- Prevention: Keep the new `HostComputer/test_gui_logic.py`, `HostComputer/test_serial_service.py`, `HostComputer/test_main_window.py`, `test_build_system.py::test_host_gui_docs_and_entry_exist`, and the offscreen `HostMainWindow` smoke launch in the verification routine whenever host GUI wiring, docs, or startup paths change.
- Commit: 49560693973c4792e88bf74859b9830884ea2233

## [2026-04-02 23:32] Low-side ADC sampling timing and freshness contract
- Problem: `ADC1` was still triggered by `TIM1 TRGO=UPDATE` with short `8.5-cycle` sample times, while `TIM1_UP_IRQHandler()` could run before `DMA1_Stream2_IRQHandler()` committed the latest frame. That let the 20kHz FOC loop read stale one-cycle-old current data and gave the host no explicit visibility into timing misses for the low-side shunt path.
- Resolution: Retargeted `ADC1` regular conversions to `TIM1_TRGO2` sourced from internal `TIM1_CH4/OC4REF`, increased current/Vbus sample times to `32.5/16.5 cycles`, raised `DMA1_Stream2_IRQn` above `TIM1_UP_IRQn`, added control-cycle-aware ADC frame sequencing and miss/invalid-window counters in `adc_sampling`, gated `FOC_App_TIM1_IRQHandler()` on a fresh current-cycle frame with escalation to `FOC_FAULT_ADC_SAMPLING`, expanded UART status/fault uploads with ADC diagnostics, added regression tests for the timing/freshness contract, and updated the implementation plan plus architecture/README docs.
- Prevention: Keep the new timing/freshness/UART contracts in `test_build_system.py`, rerun `python -m unittest test_build_system.py`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` after any future TIM1/ADC/DMA/FOC/UART sampling changes, and preserve the rule that the control loop may only consume a frame explicitly validated for the current PWM cycle.
- Commit: 8d44727b34c6a0810c1e3dcf23dc4d02c1b03bdb
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 11:05] ADC miss semantics and IRQ ladder follow-up
- Problem: The first low-side ADC timing redesign left two regressions: `sampleMissCount` accumulated across the full uptime instead of representing consecutive misses, and `TIM1_UP_IRQn` was demoted below unrelated SPI DMA preemption levels under `NVIC_PRIORITYGROUP_3`, reintroducing current-loop jitter risk.
- Resolution: Added `ADC_Sampling_ResetTimingState()` plus success-path miss-counter clearing so ADC sampling faults only escalate after consecutive misses within the active run/identify session, switched the global NVIC grouping to `NVIC_PRIORITYGROUP_4`, kept `DMA1_Stream2_IRQn` ahead of `TIM1_UP_IRQn`, and pushed SPI DMA/IRQ priorities behind the control loop. Updated source-contract tests plus README/architecture notes to lock the priority ladder and consecutive-miss semantics.
- Prevention: Keep the strengthened `test_build_system.py` checks for `NVIC_PRIORITYGROUP_4`, `DMA1_Stream2_IRQn < TIM1_UP_IRQn < SPI DMA/IRQ`, and ADC timing-state resets; rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` after any future TIM1/ADC/DMA/FOC interrupt or sampling-state changes.
- Commit: f19240fba8ccaa196ee459e2209ba4e4cf6373e8
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 11:14] ADC sampling design doc tracked and generated docs ignored
- Problem: After the ADC timing follow-up fixes, the approved design doc `docs/plans/2026-04-02-adc-low-side-sampling-design.md` was still untracked and `docs/_generated/` remained as noisy generated workspace output, leaving the repository state ambiguous even though the implementation work had been landed.
- Resolution: Added `docs/_generated/` to `.gitignore` so generated documentation artifacts stay local, and committed the ADC low-side sampling design document so the design baseline now lives in version control next to the implementation and follow-up fixes.
- Prevention: Keep generated documentation outputs under ignored paths, and when a design doc is used to drive implementation, commit it in the same task so review, implementation, and follow-up fixes all reference the same tracked source document.
- Commit: f28db495dd3c0f858325a6174eec2f8ddfad687d
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 12:00] Host GUI roadmap completed in-repo
- Problem: The in-repo host GUI still stopped at a bench-safe shell with placeholder tabs, fragmented state handling, no reusable validation/notification path, and no dedicated GUI coverage for advanced target entry, PI tuning, identify workflow, plotting, preset persistence, or operator session helpers.
- Resolution: Added a shared host-side GUI state/validation/persistence layer, rewired the PyQt main window into functional `Debug Panel / Identify / Advanced Control / PI Parameters` pages, covered all current firmware command paths with dedicated controls, added bounded live plotting plus CSV export, preset/settings persistence, stale-data indication, structured log filtering, quick actions, and refreshed README/architecture docs plus host-side tests/build contracts.
- Prevention: Keep the expanded host-side unit tests (`HostComputer/test_data_parser.py`, `HostComputer/test_gui_logic.py`, `HostComputer/test_serial_service.py`, `HostComputer/test_main_window.py`), the GUI doc/source contract in `test_build_system.py`, and the fresh verification routine (`python -m unittest -v ...`, `python -m unittest -v test_build_system.py`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1`) as mandatory evidence whenever host GUI architecture, command coverage, docs, or persistence behavior changes.
- Commit: 63479e600df3ed8e2075d5582d8fdbac8f1851ae
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 13:29] Host GUI packet-state reconciliation and disconnect reset
- Problem: The first host GUI roadmap implementation latched `identify_active` and `motor_enabled` from transmitted commands without reconciling them against incoming firmware `foc_state`, so READY packets after identify completion or disable still left Start/Stop and Enable/Disable buttons in the wrong state. The disconnect path also preserved the last telemetry snapshot, causing stale runtime values to appear as current data after reconnect.
- Resolution: Added explicit GUI-side FOC state constants plus packet-driven state reconciliation in `gui_logic.py`, blocked enable/identify actions when the last known state is faulted, and split action-button refresh from connection transitions so incoming packets no longer trigger disconnect-style clearing. On disconnect, the window now clears runtime/fault widgets, drops the last packet timestamps, and resets the plot buffer before any reconnect.
- Prevention: Keep the new regression tests in `HostComputer/test_gui_logic.py` and `HostComputer/test_main_window.py` that exercise READY-after-identify, READY-after-enable, fault gating, and disconnect/reconnect stale-data handling; whenever GUI button rules or runtime state plumbing change, rerun `python -m unittest -v HostComputer/test_data_parser.py HostComputer/test_gui_logic.py HostComputer/test_serial_service.py HostComputer/test_main_window.py` and `python -m unittest -v test_build_system.py` before closing the task.
- Commit: d8c5715b06810fd04c3feb7eda24d9367e381010
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 15:27] Host GUI Windows packaging path
- Problem: The in-repo Host GUI still required a live Python environment, so there was no bench-ready Windows application bundle. During packaging bring-up, the new PowerShell wrapper also failed under `powershell.exe` when `PSNativeCommandUseErrorActionPreference` was undefined.
- Resolution: Added a PyInstaller one-folder packaging path with `build_host_gui_app.ps1`, `HostComputer/host_gui_app.spec`, and a launcher entrypoint, documented the Windows app workflow in `README.md`, ignored `dist/` outputs, and fixed the build wrapper so native stderr handling remains compatible with shells that do not define `PSNativeCommandUseErrorActionPreference`. Verified the packaged output by building `dist/24V_FOC_Host/24V_FOC_Host.exe` and smoke-launching it successfully.
- Prevention: Keep the new `test_build_system.py` contracts for packaging assets and PowerShell wrapper compatibility, and whenever Host GUI packaging/build plumbing changes, rerun `python -m unittest discover -s HostComputer -p "test_*.py" -v`, `python -m unittest test_build_system.py -v`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_host_gui_app.ps1`, and a short packaged `24V_FOC_Host.exe` smoke launch before closing the task.
- Commit: 47cbc19e88683fb73a873f75ac7e230b41aa7055
- Recurrence policy: Not allowed to happen again.

## [2026-04-03 22:31] Bench boot path switched to HSI
- Problem: The STM32H743 board still stalled in `SystemClock_Config()` because `HSEON=1` while `HSERDY=0`, so the firmware never got past clock init and bench debugging could not continue. `FDCAN` startup also still depended on the external crystal path, which would have kept the bring-up chain fragile even after switching the core clock.
- Resolution: Switched the runtime system clock path to `HSI 64MHz + PLL1` while keeping `SYSCLK=480MHz`, added a bench-only `FOC_DEBUG_DISABLE_FDCAN_INIT` gate so startup skips `FDCAN` init until the crystal path is repaired, updated the CubeMX `.ioc` clock source to match the firmware, added a source-contract test for the HSI/FDCAN recovery path, and refreshed `README.md` plus `Project_Architecture.md` to document the temporary bench configuration and `CMSIS-DAP` debug flow.
- Prevention: Keep `test_build_system.py::test_debug_boot_path_uses_hsi_and_gates_fdcan_init` in the repo, and whenever startup clocks, external crystal usage, or early peripheral init are changed, rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` before closing the task.
- Commit: 171179a3a3e890b8965dbb4dc6d688b4026ac235
- Recurrence policy: Not allowed to happen again.

## [2026-04-04 10:10] TLE5012 NSS driven from PA15
- Problem: The hardware had moved the TLE5012 encoder `NSS` line to `PA15`, but the firmware still assumed the encoder chip-select was hard-tied low. As a result, `SPI3` transactions never actually asserted/deasserted the encoder select line, and the error path also lacked any recovery to release `NSS` on transfer failure.
- Resolution: Added explicit `TLE5012_NSS` pin definitions in `main.h`, switched `tle5012.[ch]` to software chip-select control on `PA15`, asserted `NSS` before each DMA read and released it on DMA completion, timeout recovery, and SPI error callback, then updated the source-contract tests plus `README.md` and `Project_Architecture.md` to document the `SPI3 + DMA + software NSS` path.
- Prevention: Keep `test_build_system.py::test_tle5012_uses_pa15_as_software_nss` in the repo, and whenever encoder SPI wiring, DMA callbacks, or GPIO ownership changes, rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` before closing the task.
- Commit: b6a00c5e27c52c02b0a0f74cf9aec5c37a61f108
- Recurrence policy: Not allowed to happen again.

## [2026-04-05 16:21] Bench boot path switched back to HSE
- Problem: After the earlier fallback to `HSI`, the board needed a clean retest of the repaired external crystal path. Keeping firmware on the internal RC would have hidden whether the reworked `HSE 25MHz` network could now start reliably.
- Resolution: Switched `SystemClock_Config()` and the CubeMX `.ioc` clock model back to `HSE + PLL1` while keeping `SYSCLK=480MHz`, updated the startup source-contract test from `HSI` to `HSE`, and refreshed `README.md` plus `Project_Architecture.md` to document that this bench build now re-tests the external crystal while still keeping `FOC_DEBUG_DISABLE_FDCAN_INIT` enabled to isolate the clock path.
- Prevention: Keep `test_build_system.py::test_debug_boot_path_uses_hse_and_gates_fdcan_init` in the repo, and whenever the startup clock source or external crystal recovery path is changed, rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` before closing the task.
- Commit: ea0ab41a3cbebceaa8ff3c9f3b79a463b4501f21
- Recurrence policy: Not allowed to happen again.

## [2026-04-05 16:48] HSE retest still stalls in SystemClock_Config
- Problem: After reflashing the new `HSE` bench build over SWD, the target still failed to boot. A reset-and-run capture showed `PC=0x0800820a`, which is the tight loop immediately after `HAL_RCC_OscConfig()` fails in `SystemClock_Config()`, and `RCC_CR=0x00014025`, meaning the firmware had enabled `HSE` but `HSERDY` still never asserted.
- Resolution: Verified the new image by programming it with `pyocd commander`, then halted after reset and mapped the stopped PC back to the `SystemClock_Config()` failure loop. This confirms the current blocker is still the external crystal startup path, not `FDCAN`, encoder SPI, or later peripheral init.
- Prevention: After any future crystal rework, always reflash the `HSE` bench image and capture both `PC` and `RCC_CR` over SWD before assuming the hardware fix worked; if `RCC_CR` again shows `HSEON=1` with `HSERDY=0`, stay focused on the oscillator network instead of moving on to downstream firmware modules.
- Commit: ea0ab41a3cbebceaa8ff3c9f3b79a463b4501f21
- Recurrence policy: Not allowed to happen again.

## [2026-04-05 19:10] HSI台架恢复与故障上传加固
- Problem: The repaired `HSE 25MHz` path still could not bring the board past `SystemClock_Config()`, so bench bring-up had to fall back to `HSI` first. After that fallback, firmware still had two runtime blockers: `ADC_Sampling_Calibrate()` could deadlock before `TIM1_TRGO2/OC4REF` was actually running, and worst-case UART fault snapshots could be truncated or delayed by a too-small buffer plus float-heavy fault formatting.
- Resolution: Switched the bench startup path back to `HSI 64MHz + PLL1` while keeping the original target bus/peripheral frequencies, so `TIM1 PSC/ARR` and `SPI1/SPI3` prescalers stayed unchanged. Started `TIM1 base + CH4/OC4REF` before ADC zero calibration and later enabled only the `TIM1` update interrupt, enlarged `DRV_UART_BUF_SIZE` to `1536`, rewrote the fault formatter to avoid float `printf`, added source-contract tests for the startup and UART constraints, and recorded the remaining live SPI findings in the merged bench appendix within `PROCESS.md`.
- Prevention: Keep `test_debug_boot_path_uses_hsi_and_gates_fdcan_init`, `test_startup_primes_tim1_oc4_before_adc_zero_calibration`, `test_uart_fault_packet_buffer_covers_worst_case_fault_dump`, and `test_uart_fault_formatter_avoids_float_printf_in_fault_path` in `test_build_system.py`; after any future clock-source, TIM1/ADC startup, or UART fault-path change, rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` before closing the task.
- Commit: a0ef1278ad23ceb64da6e2ad9838edf805741fd6
- Recurrence policy: Not allowed to happen again.

## [2026-04-05 20:20] PCB网表确认SPI问题边界
- Problem: The live SPI failures still had an unresolved hardware-vs-firmware split: `DRV8350S` kept returning `0x07ff`, and `TLE5012` kept producing CRC-invalid frames. Without checking the actual PCB netlists, it was still possible to chase the wrong side of the interface.
- Resolution: Read both `Netlist_PCB1_2026-04-05.tel` and `Netlist_PCB2_2026-04-05.tel`. Confirmed the control board already provides the required `DRV8350S SDO` pull-up (`R34=4.7K`) and also holds `DRV_EN` low by default through `R11`, so the SPI1 issue remains in firmware sequencing/protocol. Also confirmed the encoder PCB ties `SPI3_MOSI` and `SPI3_MISO` through `R1/R2=100R` into the same TLE5012 `DATA` pin, proving the current `SPI3` full-duplex DMA implementation does not match the hardware's single-data-line SSC topology.
- Prevention: Whenever a peripheral bring-up failure could plausibly be either wiring or firmware, check the board netlist before changing protocol code. For this project specifically, preserve the rule that `DRV8350S` SPI changes must be checked against the device frame timing, and `TLE5012` changes must be checked against the single-line `DATA` topology and `twr_delay` requirement before implementation.
- Commit: 08af84a6b932e953bb2cb0816e33f7a24be82cd8
- Recurrence policy: Not allowed to happen again.

## [2026-04-05 18:38] SPI bring-up aligned to real TLE5012/DRV8350S protocols
- Problem: Bench evidence plus PCB netlists showed both SPI links were still using the wrong transaction model: `TLE5012` was being polled with full-duplex `TransmitReceive DMA` even though the board ties `MOSI/MISO` onto one SSC `DATA` line, and `DRV8350S` was still configured/read while `DRV_EN` was low and then parsed as a two-frame `read + NOP` protocol even though the device returns register data in the current 16-bit frame. That combination could only yield encoder CRC failures, `0x07ff` reads, and misleading fault handling.
- Resolution: Switched `SPI3` to `SPI_DIRECTION_1LINE`, rewired `tle5012.c` to a staged `Transmit DMA -> twr_delay -> Receive DMA` flow with dedicated TX/RX callbacks, and changed the CRC check to cover the command plus returned data word. Reworked `DRV8350S` sync/async register reads to use a single 16-bit frame and parse `rxBuf[0]`, powered `DRV_EN` before initial configuration, kept diagnostics alive by leaving `DRV_EN` high while forcing gate drivers into `COAST`, and set `config.vdsLvl = 0x01` for the requested `0.07 V typ` threshold. Added source-contract tests to lock these protocol assumptions.
- Prevention: Keep `test_tle5012_uses_three_wire_staged_transfer_and_command_crc` and `test_drv8350_uses_single_frame_reads_and_keeps_diagnostics_powered` in `test_build_system.py`, and rerun `python -m pytest test_build_system.py -q`, `powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`, and `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` after any future SPI, `DRV_EN`, or fault-diagnostics changes. Preserve the rule that non-driving states must keep gates disabled but may keep diagnostics power alive when the system still needs fault visibility.
- Commit: 41200eb7a3fd8418d8897cafa9f938950f4e182e
- Recurrence policy: Not allowed to happen again.

## Bench Runtime Appendix

下列内容从历史 `process.md` 合并而来，保留本轮台架调试的现场记录。

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

## [2026-04-05 20:20] 网表确认的硬件边界与协议问题
- 类型: 已定位的根因线索
- 控制板网表 `Netlist_PCB1_2026-04-05.tel` 显示：`DRV8350S` 的 `MISO/SDO` 网 `MISO` 通过 `R34` 上拉到 `VCC`，而 `R34` 的阻值在封装表中是 `4.7K`；因此 `SPI1` 读回全 `1` 不是因为板上缺少 `SDO` 上拉。相同网表还显示 `DRV_EN` 通过 `R11` 下拉到 `GND`，说明驱动默认硬件态就是失能。
- 结合固件，当前 `main.c` 在 `DRV_EN` 仍然为低时就执行 `DRV8350S_Configure()`，随后 `drv8350s.c` 的异步读路径又把“读命令 + NOP”放在同一次 `nSCS` 拉低周期内，并把 `rxBuf[1]` 当作目标寄存器数据。这与 DRV8350S 手册“单个 16bit 帧即返回当前寄存器数据，且帧与帧之间 `nSCS` 必须回到高电平”不一致，是当前 `SPI1` 软件侧的主嫌疑。
- 编码器板网表 `Netlist_PCB2_2026-04-05.tel` 显示：`SPI3_MOSI` 通过 `R1(100R)`、`SPI3_MISO` 通过 `R2(100R)` 同时汇到 `U1.4`；`SPI3_SCK` 通过 `R3(100R)` 到 `U1.3`，`SPI3_NSS` 通过 `R4(100R)` 到 `U1.2`。这说明 TLE5012 接口在硬件上就是把 MCU 的 `MOSI/MISO` 通过串阻并到传感器单 `DATA` 线上，不是标准四线全双工 SPI。
- 结合当前 `tle5012.c` 的实现，`HAL_SPI_TransmitReceive_DMA()` 按两线全双工连续发送 `0x8021, 0x0000, 0x0000`，主机在数据阶段仍持续驱动 `MOSI`，没有切换为高阻/单线接收，也没有按器件协议插入方向切换延时 `twr_delay`。这与板上单线 `DATA` 适配方式直接冲突，和现场抓到的 `tle5012_rx_buf = [0x8021, 0x0000, 0x8021]` 现象一致。
- 附带问题: 当前 `TLE5012_CalculateCRC8()` 仅对 `raw_data` 一字做 CRC，而器件安全字的 CRC 依赖完整传输字节序列；即使物理层修通，现有 CRC 校验也会误判。

## [2026-04-05 18:38] TLE5012 / DRV8350S SPI 协议与诊断上电修复
- 类型: 已修复的代码逻辑问题
- TLE5012: `SPI3` 改为 `1-line` 半双工，读取流程改成“命令 DMA 发送 -> `twr_delay` -> Data/Safety DMA 接收”，`stm32h7xx_it.c` 中拆分为 `HAL_SPI_TxCpltCallback()` 和 `HAL_SPI_RxCpltCallback()` 两段处理，避免再走错误的全双工 `TransmitReceive` 路径。
- TLE5012 CRC: 参考 Infineon 官方库的 SSC 安全字检查方式，CRC 改为覆盖“命令字 + 数据字”字节序列，不再只对 `raw_data` 单字计算；同时把 `SYSTEM / INTERFACE / INVALID_ANGLE` 三个安全位一起纳入 `data_valid` 判定。
- DRV8350S: `main.c` 中先拉高 `DRV_EN` 并等待 `1 ms`，再执行 `DRV8350S_Configure()`；`drv8350s.c` 的同步/异步读统一改为单个 `16-bit` 帧直接取当前帧回包，不再发送额外 `NOP`，也不再解析不存在的 `rxBuf[1]`。
- 诊断策略: 保持 `DRV_EN` 上电，只通过 `DRV8350S_DisableGateDrivers()` 让功率级进入 `COAST/Hi-Z`，这样故障态和待机态都能继续轮询驱动寄存器并实时上传，而不会驱动 MOS。
- 保护阈值: 启动配置里把 `DRV8350S vdsLvl` 改为 `0x01`，对应 `0.07 V typ` 的压降阈值，匹配你前面要求的过流检测基线。
- 验证:
  - `python -m pytest test_build_system.py -q` -> `20 passed`
  - `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1` -> `Successful: 8, Failed: 0`
  - `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1` -> 链接成功并生成 `24V_FOC_Controller.hex/.bin`
  - 代码提交: `41200eb7a3fd8418d8897cafa9f938950f4e182e`

## [2026-04-05 21:xx] 切换到 Keil + CMSIS-DAP 烧录调试链
- 类型: 当前执行中的台架流程调整
- 背景: `pyocd load/reset/list` 在本机和当前 `CMSIS-DAP` 连接上出现阻塞/超时，导致命令行烧录链不稳定，即使探针重新枚举后也会卡住。
- 已确认条件:
  - 本机存在 `C:\\Keil_v5\\UV4\\UV4.exe`
  - 工程为 [MDK-ARM/24V FOC Controller.uvprojx](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\24V FOC Controller.uvprojx)
  - [MDK-ARM/24V FOC Controller.uvoptx](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\24V FOC Controller.uvoptx) 当前绑定 `CMSIS_AGDI.dll`，即 `CMSIS-DAP` 调试器链
  - Keil 官方命令行文档支持 `UV4 -b` 构建、`UV4 -f` 下载到 Flash
- 执行策略:
  - 先用 `UV4 -b` 做一次工程构建，确认 Keil 工程本身可用
  - 再用 `UV4 -f` 走 Keil Flash Download，不再与 `pyocd` 混用
  - 若 Keil 仍失败，则保留失败日志并把问题边界收敛到“CMSIS-DAP/Keil 下载链”而非固件编译链

## [2026-04-05 20:02] Keil 在线调试确认两条 SPI 链路都在读全高回包
- 类型: 已完成的在线调试结论
- 烧录/启动: `UV4 -b` 和 `UV4 -f` 已成功执行；SWD 复位后在 `0x080055c8 (FOC_App_MainLoop)` 断点稳定命中，说明当前固件已经穿过 `SystemClock_Config()`、`ADC_Sampling_Calibrate()` 和 `DRV8350S_Configure()`，不再卡在早期启动阶段。
- FOC / UART1: 主循环首个断点时 `g_foc_app.state=IDLE`、`fault_code=NONE`、`control_mode=1`、`Vbus=24.0V`；自由运行约 `1.5 s` 后变为 `FOC_STATE_FAULT + FOC_FAULT_DRV8350S`，但 `Vbus` 仍保持 `24.0V`。`USART1 BRR=0x412`，`huart1.gState=0x21`、`RxState=0x22`、`ErrorCode=0`，当前没有发现 UART 初始化或波特率级异常。
- SPI1 / DRV8350S: 在 `0x08003b08 (DRV8350S_DMA_CompleteCallback)` 断点处，`PA4(nSCS)` 的 `ODR/IDR` 已确认为低电平，说明片选确实被软件拉下；此时 `txBuf=0x8000`（读 `FAULT_STATUS_1`），而 `rxBuf=0xFFFF`。继续自由运行后，`regFaultStatus1/regVgsStatus2/regDriverCtrl=0x07FF`、`faultFlags=0x00FF07FF`，属于标准“总线只读到高电平”现象。同步抓到 `hspi1.State=5(BUSY_TX_RX)`、`hspi1.ErrorCode=0`，说明这不是 HAL 级 SPI 错误，而是总线层回包无效。
- SPI3 / TLE5012: 在 `0x0800e3b0 (TLE5012_ProcessData)` 断点处，`PA15(NSS)` 已经被软件拉低；`GPIOC MODER=0xFCAFFFFF`，对应 `PC11=AF`、`PC12=Input`，说明单线 SSC 的收发方向切换已经执行到接收相位。但原始 `tle5012_rx_buf=[0xFFFF, 0xFFFF]`，稳态 `tle5012_sensor` 变成 `raw_angle=0x7FFF`、`status=0x7F`、`crc_error=1`、`data_valid=0`，同样是物理层/协议层只读到高电平。
- 额外观察: `drv8350s.runtime.commCount` 与 `errorCount` 近似相等，但 `hspi1.ErrorCode=0`。结合代码路径，这个 `errorCount` 现在主要由 `dmaBusy` 跨周期未清时在 `TIM1` 中断里重复累加，不能直接当作“真实 SPI 硬错误次数”解释。
- 结论: 当前板级主阻塞点已经收敛为 `SPI1(DRV8350S)` 和 `SPI3(TLE5012)` 两条链路的原始回包全高，`UART1` 不是当前异常源；下一步应优先继续核查 DRV 的 `SDO` 实际驱动条件，以及 TLE5012 的 SSC 时序/数据线驱动，而不是继续怀疑主时钟或 UART 配置。

## [2026-04-08 22:05] Simulink Task 6 observer/load architecture
- Problem: The average-value FOC Simulink baseline still fed the position and speed loops from ideal plant outputs and hard-coded the mechanical load torque inside the plant, so the closed-loop behavior remained too ideal for the next realism step.
- Resolution: Added a dedicated design document and execution plan for Task 6, defining an encoder-style observer layer (`theta_mech_fb`, filtered `speed_mech_fb`, projected electrical angle/speed) plus an external `Tload_cmd` plant input that starts at `0.2 N·m`. The implemented external MATLAB artifacts now follow that design in `D:\matlab`, and the repo tracks the approved architecture and verification contract.
- Prevention: Keep the Task 6 design and implementation plan in version control before extending the Simulink model further, and require fresh MATLAB batch checks proving the plant exposes `Tload_cmd`, the loops read `feedback`, and the new model simulates before calling the realism upgrade complete.
- Commit: f43ebd6a3f375e3d35cf3ca80804bbc83282d48d
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 21:54] CMSIS-DAP台架复核确认两路SPI仍为全高回包
- Problem: 继续烧录调试时，板上没有接 UART/USB 数据链路，必须改走 SWD 观测。此前如果只看“无串口输出”，容易误判为固件没跑起来；同时需要重新确认当前板上是否真的是最新镜像，以及 SPI1/SPI3 的现场故障是否仍然存在。
- Resolution: 使用 `CMSIS-DAP + pyOCD` 重新识别探针、用当前 `24V FOC Controller.axf` 重新烧录，并在无 `nRESET` 条件下改用 `attach` 模式附着调试。软件复位后分时读取 `g_foc_app`、`drv8350s`、`tle5012_sensor` 的实时值，确认固件稳定运行但持续停在 `FOC_STATE_FAULT + FOC_FAULT_DRV8350S`，`Vbus≈11.73V`。同时再次抓到 `DRV8350S` 读回 `FAULT1/VGS2/OCP = 0x07FF`、`lastRx=0xFFFF`、`commFault=1`，以及 `TLE5012` 读回 `raw=0x7FFF`、`status=0xFF`、`crc_error=1`、`data_valid=0`。结合控制板网表中 `MISO` 由 `R34=4.7K` 上拉到 `VCC` 的事实，可确认当前两条 SPI 链路都仍然表现为“外设未有效驱动回包，总线只读到高电平”，而不是 UART 或主时钟问题。
- Prevention: 今后在没有串口链路的台架调试里，先用 SWD 重新烧录当前镜像，再用 `attach` 采样 `g_foc_app/drv8350s/tle5012_sensor`，不要把“无串口输出”等同于“固件未启动”。如果再次看到 `DRV8350S=0x07FF/0xFFFF` 或 `TLE5012=0x7FFF/0xFF`，优先回到片外供电、连线、片选、连接器/编码器板，而不是先怀疑 UART 或控制环软件。
- Commit: 627242bcd4ca1d655f01a92635a0abbfd0595ec3
- Recurrence policy: Not allowed to happen again.

## [2026-04-08 21:54] TLE5012 ISR时序与Safety Reset可观测性修复
- Problem: `TLE5012` 在 `TIM1` 高优先级路径里通过 `HAL_GPIO_Init()` 反复重配 `PC11/PC12`，把编码器数据线方向切换变成了高开销 HAL 路径；同时 `Safety Word bit15`（编码器复位/看门狗异常）在驱动里被掩掉，UART 也无法把这个故障原因上传给上位机。
- Resolution: 将 `tle5012.c` 的命令/响应阶段切换改为直接修改 `GPIOC->MODER`，把方向切换收敛为常量级寄存器写；保留完整 `Safety Word` 高8位到 `tle5012_sensor.status`，新增 `reset_fault` 标志，并在 `uart_upload` 的正常/故障文本里追加 `Safety` 与 `Reset` 诊断字段。同步更新 `README.md`、`Project_Architecture.md` 和源码约束测试。
- Prevention: 保留 `test_build_system.py` 中关于 `GPIOC->MODER` 常量开销切换、禁止 `HAL_GPIO_Init()` 回到 `TIM1` 路径、以及 `Safety Word bit15` 必须进入 UART 上传的回归约束；后续若再改编码器 SPI/故障上报，必须重跑 `python -m pytest test_build_system.py -q`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1` 和 `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`。
- Commit: c8b7c0ba4d3be587121637e9d6d58a632a280415
- Recurrence policy: Not allowed to happen again.

## [2026-04-08 23:40] Simulink Task 7 abc average-value plant
- Problem: The Task 6 Simulink baseline still kept the motor electrical state entirely in dq coordinates, so the simulation could not expose real `ia/ib/ic` phase-current behavior even though the controller already had an observer and average-value bridge.
- Resolution: Added Task 7 design and implementation docs plus red/green MATLAB verification scripts in the repo, then used the external `D:\matlab` upgrade flow to create `minimal_foc_controller_task7_abcplant.slx`. The new model rebuilds `plant` as an average-value SPMSM `abc` current model, rebuilds `feedback` to convert `ia/ib/ic` back into `id/iq` through Clarke/Park, keeps the existing observer and loop structure, and adds direct three-phase viewing/logging blocks.
- Prevention: Before any future Simulink realism upgrade is called complete, run the red check proving the prior baseline lacks the new interface, then run `tools/simulink/check_task7_green.m` to verify ports, rewiring, successful batch simulation, and non-flat three-phase currents.
- Commit: abab0238f0a8c3452d2da7e66c2000eec27cb443
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 16:12] Live encoder bench snapshot still shows all-high SSC replies
- Problem: With `CMSIS-DAP` attached and the current firmware baseline running on hardware, live SWD snapshots still show the encoder SSC path returning invalid all-high data. At both `150 ms` and `1500 ms` after reset, `tle5012_rx_buf = [0xffff, 0xffff]`, `tle5012_sensor.raw_angle = 0x7fff`, `status = 0x7f`, `reset_fault = 1`, `crc_error = 1`, and `data_valid = 0`. In the same snapshots the application is already in `FOC_STATE_FAULT + FOC_FAULT_DRV8350S`, while `DRV8350S` runtime registers remain `0x07ff` and `faultFlags = 0x00ff07ff`.
- Resolution: Rebuilt the Keil project, reattached over `CMSIS-DAP`, brought up `pyocd` for scripted SWD access, and captured repeatable runtime snapshots from `tle5012_rx_buf`, `tle5012_sensor`, `drv8350s.runtime`, and `g_foc_app` without changing firmware. This confirms the bench blocker is still the live `SPI3/TLE5012` and `SPI1/DRV8350S` data path, not a host-side UART visibility issue.
- Prevention: For every future bench session, preserve the `pyocd commander` snapshot routine against the exact runtime addresses/offsets before changing control logic, and require one reset-immediate plus one delayed snapshot so "instant fault" vs "fault after running" is not guessed from symptoms.
- Commit: 88445e46352fb5182168faba5e5903d4bd86ec6d
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 16:26] TLE5012响应阶段事务模型与官方3-wire参考实现不一致
- Problem: After narrowing the live failure to `SPI3/TLE5012`, the staged firmware flow was still using `HAL_SPI_Transmit_DMA(command)` followed by `HAL_SPI_Receive_DMA(data+safety)`. The Infineon reference library for 3-wire SSC does not do a receive-only second stage: it keeps `CS` low, waits `5 us`, flips the data-line drive direction, then clocks out the response by sending `0x0000` dummy words while receiving `Data + Safety`. On STM32H7, `HAL_SPI_Receive_DMA()` switches the peripheral to `2LINES_RXONLY`, which does not match that reference transaction model and is the most likely software root cause for the stable `0xffff/0xffff` replies.
- Resolution: Pulled the Infineon `TLE5012-Magnetic-Angle-Sensor` reference library, traced `readFromSensor()` and the Arduino `spi3w` PAL, and confirmed that the command word/CRC assumptions in the current firmware are fine (`READ_SENSOR | REG_AVAL | SAFE_high = 0x8021`), while the response phase must be modeled as dummy-clocked transfer rather than pure receive-only DMA.
- Prevention: Before changing the encoder transport again, keep the rule that any future `TLE5012` SSC refactor must be checked against the official 3-wire `sendReceive(command, 1, received, 2)` behavior, including the `5 us` trigger delay, data-line direction swap, and explicit dummy clocks for every returned word.
- Commit: 56037dfc3776f870448f386381facc01c01249f8
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 16:52] TLE5012三线响应阶段改为dummy-clock DMA收发
- Problem: `tle5012.c` 仍在响应阶段调用 `HAL_SPI_Receive_DMA()`，即使前面的根因调查已经确认 STM32H7 上这会切入 `2LINES_RXONLY`，与 TLE5012 官方 3-wire SSC 的“dummy word 打时钟 + 同步接收 Data/Safety”模型不一致。对应地，`stm32h7xx_it.c` 仍把 `SPI3` 的完成处理挂在 `HAL_SPI_RxCpltCallback()`，也和新的事务模型不匹配。
- Resolution: 在 `tle5012.c` 中新增两字 `dummy` 发包缓冲区，把响应阶段改成保持 `CS` 有效、`PC11=AF`/`PC12=Input` 后调用 `HAL_SPI_TransmitReceive_DMA()`，用两个 `0x0000` dummy word 为 TLE5012 回包提供时钟；同时把 `SPI3` 的收尾处理迁移到 `HAL_SPI_TxRxCpltCallback()`，保留命令阶段的 `HAL_SPI_TxCpltCallback()` 只负责进入响应相位。同步更新 `test_build_system.py`，将源码契约固定为 dummy-clocked 3-wire 事务。
- Prevention: 保留 `test_tle5012_uses_three_wire_staged_transfer_with_dummy_clocked_response`，并在每次修改 `tle5012.c` / `stm32h7xx_it.c` 后至少重跑 `python -m pytest test_build_system.py -q -k tle5012`、`python -m pytest test_build_system.py -q`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1` 和 `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`，避免再把 TLE5012 响应相位退回纯接收模式或挂错 DMA 完成回调。
- Commit: 63b265ccf6f8a6ed408a3d3bb0c9dc57440048d8
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 17:05] Dummy-clock修复已上板但TLE5012稳态仍回全高
- Problem: 新的 dummy-clock `SPI3/TLE5012` 固件已经重新编译并成功烧录到板上，但上机快照显示问题没有完全消失。复位后约 `150 ms` 抓到 CPU 停在 `HAL_SPI_IRQHandler`，此时 `tle5012_rx_buf = [0x0000, 0x0000]`，说明编码器事务正处在早期处理中；继续运行到约 `1500 ms` 后再次抓取，`tle5012_rx_buf = [0xffff, 0xffff]`，`tle5012_sensor = { angle≈359.99°, raw_angle=0x7fff, status=0xff, reset_fault=0, crc_error=1, update_flag=1, data_valid=0 }`。同时 `g_foc_app.state=5`、`fault_code=5`、`power_unlocked=1`、`Vbus=24.0V`，以及 `DRV8350S regFaultStatus1/regVgsStatus2=0x07ff`、`faultFlags=0x00ff07ff`，说明这次修复已经真实运行在板上，但编码器和驱动两条 SPI 链路在稳态仍然都表现为“读到高电平”。
- Resolution: 使用 `pyocd load -M attach -t stm32h743xx` 成功烧录 [build/gcc/24V_FOC_Controller.elf](C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/build/gcc/24V_FOC_Controller.elf)，避开了当前探针上 `safe_reset_and_halt` 的断言失败；随后通过两次 `pyocd commander` 复位/运行/暂停快照，确认“dummy-clock 代码已上板”与“稳态回包仍全高”这两个事实同时成立，排除了“只是没烧进去旧固件”的可能性。
- Prevention: 之后每次上机验证都保留这条双快照流程：先用 `attach` 模式烧录，再抓一次复位后短延时快照和一次稳态快照；只有当两次快照都显示 `tle5012_rx_buf` 已脱离 `0x0000/0xffff` 异常模式，才能判定编码器链路真正修通。当前探针链路默认 `load` 的 reset 流程不稳定，后续继续台架调试时优先使用 `pyocd load -M attach`。
- Commit: 21de43c9ea8c97de8373ca93f75f3efe42d2a7ce
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 20:18] DRV8350S无效SPI回包被误判为真实驱动故障
- Problem: 基于当前台架固件 `a3e2c35fb05d71c9302b097add7ad1f61ace57a0` 的在线快照，系统运行时 `SPI1/DRV8350S` 仍稳定读回 `rxBuf=0xffff`、`regFaultStatus1=0x07ff`、`regVgsStatus2=0x07ff`、`regDriverCtrl=0x07ff`，且 `drv8350s.runtime.isFaultActive=1`、`faultFlags=0x00ff07ff`、`errorCount=0`。同时 `g_foc_app` 已处于 `FOC_STATE_FAULT + FOC_FAULT_DRV8350S`，说明当前主阻塞点不是 HAL SPI 报错，而是软件把明显无效的全高回包按真实 DRV 故障位解析，随后在 `main.c` 主循环里强制打进整机故障态。
- Resolution: 重新核对了 [MDK-ARM/code/drv8350s.c](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\drv8350s.c) 与 [Core/Src/main.c](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\Core\Src\main.c) 的故障链路，并用 `pyocd commander` 在线读取 `g_foc_app` 与 `drv8350s.runtime` 的关键地址，确认当前路径是“无效 SPI 回包 -> `DRV8350S_ParseFaultStatus()` 置位 `isFaultActive` -> `main()` 把 `fault_code/state` 强制切到 `DRV8350S/FAULT`”。这次结论不涉及功能代码改动，只是把故障边界收敛到 `DRV8350S` 通信有效性判定模型。
- Prevention: 后续处理 `DRV8350S` 故障链路时，必须先把“SPI 传输有效”与“DRV 自身故障位有效”分离，至少增加源码/回归约束，禁止 `0xffff` / `0x07ff` 这类明显无效读回直接映射成 `isFaultActive=1`；同时继续保留 `pyocd` 在线快照例程，先验证原始寄存器回包是否合法，再根据故障位讨论功率级保护。
- Commit: a3e2c35fb05d71c9302b097add7ad1f61ace57a0
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 17:16] 在线断点证实SPI软件时序已到位但两条总线都无人拉低回包
- Problem: 仅凭稳态 `0xffff` 回包还不能排除“软件没有真正拉低片选/没有真正切换引脚模式”的可能性，特别是 `TLE5012` 的 3-wire DATA 线在 `PC11/PC12` 之间动态切换，`DRV8350S` 也需要同时满足 `DRV_EN` 高和 `nSCS` 低才能回数据。如果这些前提条件有任何一个没满足，继续怀疑板级会太早。
- Resolution: 通过 `pyocd gdbserver + arm-none-eabi-gdb` 在函数入口打硬件断点，分别抓取 `TLE5012_HandleTxComplete()`、`TLE5012_ProcessData()` 和 `DRV8350S_DMA_CompleteCallback()` 时的 GPIO/SPI 寄存器状态。结果显示：`TLE5012` 命令相位时 `PA15(IDR/ODR)=0`、`PC11=input`、`PC12=AF`、`SPI3 CFG2=0x05420000`；响应相位时 `PA15` 仍保持低，`PC11=AF`、`PC12=input`、`SPI3 CFG2=0x05400000`，说明软件片选和 3-wire 方向切换都已实际生效。与此同时 `GPIOC IDR=0x00001c00`，即 `PC10/11/12` 全高，且 `tle5012_rx_buf=[0xffff,0xffff]`。`DRV8350S` 侧则确认 `PE14(DRV_EN)=1`、`PA4(nSCS)=0`、`txBuf=0x8000`、`rxBuf=0xffff`。这说明两条 SPI 链路在软件侧都已经把关键时序条件满足了，但外设侧仍没有把数据线拉出全高状态。
- Prevention: 后续继续查 SPI 现场时，先抓函数入口寄存器证据，再决定是否改代码。对于本项目，若再遇到外设回包全高，必须先验证 `CS/EN` 电平、GPIO `MODER`、以及外设寄存器 `CFG2/SR` 是否已经进入预期事务相位；只有这些都正确后，才允许把问题归到器件供电、板级连线、焊接、时钟或外设自身状态上。
- Commit: 01dae382516fae927bb0afb7023f8d9def8c70f2
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 19:24] 硬件返修后复测：TLE快照窗口改变但TLE/DRV断点回包仍为全高
- Problem: 你反馈已经对硬件做了 debug，需要确认返修后现场是否真的改善。仅靠普通定时快照很容易把“缓冲区被清零等待下一次 DMA”误判成“总线读到 0x0000”，所以必须同时看稳态快照和函数入口断点。
- Resolution: 重新用 `pyocd load -M attach` 验证当前固件仍在板上，然后重复了两组抓取。普通时间窗快照里，`tle5012_rx_buf` 从以前的稳态 `[0xffff, 0xffff]` 变成了两次都看到 `[0x0000, 0x0000]`，说明硬件调整确实改变了空闲窗口里的总线行为；但在 `TLE5012_ProcessData()` 入口重新下断点后，实际传给驱动处理的数据仍是 `tle5012_rx_buf=[0xffff,0xffff]`，而 `TLE5012_HandleTxComplete()` 时的 GPIO/SPI 状态与之前一致。`DRV8350S_DMA_CompleteCallback()` 断点也再次确认 `PE14(DRV_EN)=1`、`PA4(nSCS)=0`、`txBuf=0x8000`、`rxBuf=0xffff`。结论是：这次硬件返修改变了采样到的“窗口外观”，但没有改变两条 SPI 在有效回包时刻仍然读全高这一事实，尤其 `DRV8350S` 侧问题完全未改善。
- Prevention: 后续台架复测不能只看定时快照，必须把“普通时间窗快照”和“驱动处理入口断点快照”配对保存；只有当入口断点处的 `tle5012_rx_buf` / `drv8350s.rxBuf[0]` 真正脱离 `0xffff`，才算外围器件开始有效驱动总线。对 `DRV8350S`，继续排查时优先验证 `EP/GND`、`VM/VDRAIN`、`VCP/CPH/CPL` 与 `DVDD` 电源链，而不是再先改 SPI 软件。
- Commit: 34ede6f744fcda4cf7e5a89bcd4897e92673c8e9
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 21:08] DRV8350S全高回包改为通信故障归因并完成板上复测
- Problem: `DRV8350S` 的 `SPI1` 回包在台架上仍然稳定为 `0xffff/0x07ff`，但固件之前会把这些全高值展开成“VDS过流 + VGS故障”等真实驱动故障位，导致上位机看到误导性故障原因，`CMD:CLEAR_FAULT` 也会继续按假故障位复位失败。
- Resolution: 在 [MDK-ARM/code/drv8350s.h](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\drv8350s.h) / [MDK-ARM/code/drv8350s.c](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\drv8350s.c) 中新增 `DRV8350S_COMM_FAULT_BIT`、`commFaultActive/commValidated/lastRxFrame` 和统一的 `DRV8350S_UpdateFaultState()`，把原始 `0xffff` 读回与 `OCP_CTRL` 配置回读校验一起作为“通信无效”判断，只有通信已验证通过时才解析真实 DRV 故障位；同步更新 [Core/Src/stm32h7xx_it.c](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\Core\Src\stm32h7xx_it.c) 的 `CMD:CLEAR_FAULT` 路径，以及 [MDK-ARM/code/uart_upload.h](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\uart_upload.h) / [MDK-ARM/code/uart_upload.c](C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\uart_upload.c) 的上传文本，让上位机明确看到 `DRV SPI readback invalid` 和最后一帧原始回读。新固件烧录后，复位 `150 ms / 1500 ms` 双快照都显示 `faultFlags=0x80000000`、`isFaultActive=1`、`commFaultActive=1`、`commValidated=0`、`lastRxFrame=0xffff`，不再出现旧的 `0x00ff07ff` 假功率级故障展开。
- Prevention: 保留 `test_build_system.py` 中新增的通信归因与 UART 上传契约，并在每次修改 `drv8350s*`、`stm32h7xx_it.c` 或 `uart_upload*` 后重跑 `python -m pytest test_build_system.py -q`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1`，再用 `pyocd` 做一次复位后短延时和稳态双快照，确认全高回包只落成 `DRV8350S_COMM_FAULT_BIT`，不再回归成假过流/假栅极故障。
- Commit: b821d58bf7c9a361db24f79be376123984c12332
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 21:12] 运行时保护阈值与识别前预检闭环
- Problem: 固件仍把欠压/过压/过流阈值写死在 `foc_app.h` 宏里，`IDLE/READY/FAULT` 阶段也不会刷新实时 `Vbus`，导致上位机无法按 12V 台架条件调整阈值，且使能/清故障可能基于占位电压 `24.0V` 做出错误判断。与此同时，电机识别阶段直接使用 `TLE5012_GetAngle()`，在编码器数据无效时仍可能继续计算极对数、Ke 和零位偏置。
- Resolution: 在 `foc_app` 中新增运行时 `FOC_ProtectionConfig_t`、`FOC_App_RefreshTelemetry()` 和 `FOC_App_PrecheckPowerStage()`，让 `Enable/StartIdentify/CLEAR_FAULT` 都先刷新实时 `Ia/Ib/Ic/Vbus` 并联合检查欠压/过压、编码器有效位和 `DRV8350S` 阻塞读回；新增 `CMD:VBUS_LIMIT,uv,ov` 仅在电机未工作时修改欠压/过压阈值；识别状态机新增 `MI_ERR_ENCODER_INVALID` 并在应用层映射为 `FOC_FAULT_ENCODER`。同步更新 `README.md`、`Project_Architecture.md` 和 `test_build_system.py`，并完成 `pytest + build_test + build` 验证。
- Prevention: 保留 `test_protection_thresholds_are_runtime_configurable`、`test_precheck_refreshes_live_telemetry_before_enable_and_clear_fault`、`test_motor_identification_requires_valid_encoder_feedback` 三条源码约束；后续凡是改 `foc_app*`、`motor_identify*`、`stm32h7xx_it.c` 或上位机命令表，必须重跑 `python -m pytest test_build_system.py -q`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\build_test.ps1`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`，并确保 `Enable/Identify/CLEAR_FAULT` 仍只基于实时遥测做决策。
- Commit: 201fd79662289f9c32cb42648f1713f20df2e040
- Recurrence policy: Not allowed to happen again.

## [2026-04-12 21:22] 清理工作区生成物并补齐遗漏计划文档
- Problem: 仓库长期把 `MDK-ARM/24V FOC Controller` 下的 `Keil` 生成物纳入版本控制，近期 bench 调试又在根目录留下大量 `.tmp_*` 和 `Simulink` 缓存，导致工作区持续处于脏状态；同时有 3 份实际的计划文档只留在本地未入库。
- Resolution: 更新 `.gitignore`，新增 `Keil` 生成物、`.tmp*`、`*.slxc` 和 `slprj/` 忽略规则；从版本控制中移除 `axf/build_log/htm/lnp/dep/crf` 等 `Keil` 生成物并删除本地缓存；补提 `docs/plans/2026-04-03-complete-simulink-average-foc-plan.md`、`docs/plans/2026-04-03-host-computer-gui-full-plan.md`、`docs/plans/2026-04-05-constant-speed-three-phase-view.md`；同时把 `PROGRESS.md` 固化为指向 `PROCESS.md` 的兼容入口。
- Prevention: 后续 `Keil`、`pyocd/gdb`、`Simulink` 运行产生的临时文件默认应落入忽略规则；提交前先看一次 `git status --short`，若再次出现生成物污染，优先修 `.gitignore` 或把误跟踪的生成物从索引中摘掉，不再让它们长期留在仓库历史里。
- Commit: 6c624939de194a487ce40fa714fc4dcf4a4921d8
- Recurrence policy: Not allowed to happen again.

## [2026-04-13 16:22] Standalone HSE LED bench clock isolation
- Problem: The board had conflicting bench evidence around the external 25MHz HSE path. Previous main-firmware bring-up had stalled in `SystemClock_Config()` with `HSEON=1` and `HSERDY=0`, but hardware rework later changed the board condition and we needed a low-risk way to prove whether the crystal path itself could now boot without the full FOC stack.
- Resolution: Added a standalone `BenchTests/HSE_LED_Test` project plus build script, two plan docs, and a source-contract test. The bench image initializes `PB8/PB9` before clock setup, uses `HSE + PLL1`, routes all clock/init failures into a `PB9` fast-blink loop, and leaves success in a `PB8` slow-blink loop. After building and loading the image over `pyocd load -M attach`, live SWD evidence showed `PC=0x08000510`, `RCC_CR=0x03034025`, and `GPIOB ODR=0x00000100`, proving the minimal HSE path now reaches the success loop with `HSERDY=1`. The implementation commit is `4ae9956b8798a1613b8f4f279cad98af66cefee9`.
- Prevention: Keep the standalone HSE bench project and `test_standalone_hse_led_test_project_contract` in the repo, and whenever future hardware rework reopens the clock question, verify with the minimal image first and capture both `PC` and `RCC_CR` over SWD before blaming downstream firmware modules.
- Commit: 4ae9956b8798a1613b8f4f279cad98af66cefee9
- Recurrence policy: Not allowed to happen again.

## [2026-04-15 20:29] Task8顶层三环参数外置并完成官方PMSM模型整理
- Problem: `task8` 的官方 PMSM 模型虽然已经能仿真，但三环 PI 增益仍埋在各子系统内部，顶层不方便直接调参；同时升级脚本把顶层参考源块名写死成 `command source`，而实际模型里该块名是 `commend sorse`，导致顶层重连线和布局对现有模型不稳。
- Resolution: 新增 Task8 顶层 PI 外置设计/实现文档，更新 [tools/simulink/upgrade_add_official_pmsm.m](C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/upgrade_add_official_pmsm.m) 以在顶层创建 `Kp_pos`、`Ki_pos`、`Kp_speed`、`Ki_speed`、`Kp_iq`、`Ki_iq` 六个真实调参块，并重建 `position loop`、`speed loop`、`current loop` 使这些参数从顶层输入进入控制环。顶层重连线改成自动解析真实参考源与可选观察块，不再依赖固定块名，同时恢复 `pos_ref`、`speed_ref`、`Iq_ref` 和 `foc_debug_scope` 观察线。再用 [tools/simulink/check_task8_green.m](C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/check_task8_green.m) 校验新接口；用户在本机 MATLAB 上实际跑通升级与绿灯检查，得到 `TASK8_POSITION_LOOP_IN=4 OUT=1`、`TASK8_SPEED_LOOP_IN=4 OUT=1`、`TASK8_CURRENT_LOOP_IN=6 OUT=2`，并最终输出 `SIM_OK_TASK8`。
- Prevention: 后续任何 Simulink 顶层整理都不要再写死块名，必须优先从现有连线或实际存在的顶层块解析信号来源；同时保留像 `check_task8_green.m` 这样的结构+连线+仿真联合校验，至少验证端口数、顶层参数源、关键观察信号和一次真实仿真绿灯后，才能声称模型升级完成。
- Commit: 150cda0cfdd8e555a0f773d218ceacbdd88a184e
- Recurrence policy: Not allowed to happen again.
## [2026-04-16 00:53] 位置环从PI语义切换为显式PD
- Problem: 位置模式此前仍沿用 `PI` 形态的接口，固件默认虽然把 `Ki` 设为 `0`，但结构体、UART命令、上位机 GUI 和文档仍把位置环表述为 `PI`，导致用户可以继续下发 `CMD:PI_POS`，并把第二个参数当积分项使用。这会让控制语义和实际实现脱节，也增加关节电机上机整定时的误判风险。
- Resolution: 将固件位置环改为显式 `FOC_PositionPD_t`，在 `FOC_App_PositionLoop()` 中使用 `speed_ref = Kp * pos_error - Kd * speed_mech`，新增 `FOC_App_SetPositionPDGains()`，UART 命令改为 `CMD:PD_POS`；同步更新 Host GUI 的命令构建、preset 持久化、页签文案和位置环输入标签；补充 `test_build_system.py` 与 HostComputer 单测，验证位置环 PD 语义、旧 `position_pi` 本地配置兼容路径、GUI 标签和命令接口；更新 `README.md`、`Project_Architecture.md` 与实施计划文档。
- Prevention: 后续凡是修改位置环、上位机命令或控制模式文档，必须同时检查固件结构体命名、UART 命令名、GUI 标签和文档是否一致，并重跑 `python -m unittest HostComputer.test_data_parser HostComputer.test_gui_logic HostComputer.test_main_window -v`、`python -m unittest -v test_build_system.py`、`powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`，禁止再次出现“名字还是PI、实际已经不是PI”的语义漂移。
- Commit: 09ad81d961e1e3b808360e38a6456b38fee74f95
- Recurrence policy: Not allowed to happen again.

## [2026-06-12 01:30] 前馈v4_safe_baseline验证 + 电磁刹车阻塞确认
- Problem: 6月11日测试发现前馈补齐后Iq≈3A但机械角几乎不动，需要先确认是前馈参数问题还是硬件阻塞。
- Resolution: 通过COM9串口执行分阶段验证：
  - Phase A: 硬件链路确认。DRV8350S SPI通信已恢复（Comm: Validated），TLE5012角度连续(257-260°)，Vbus=23.9V，ADC正常，FAULT1=VGS2=0x0000。与四月份台架全高回包不同，当前SPI链路已正常工作。
  - Phase B: v4_safe_baseline扭矩测试。0.05A→0.10A→0.20A→0.50A→1.0A→2.0A递增。电机对扭矩响应方向正确(+Iq→+angle)，但总移动量<1°，实际Iq显著高于指令值(0.5A→0.84A, 1.0A→1.79A)。2.0A触发过流保护(FAULT→Overcurrent)。行为与电磁刹车抱死一致：电流能注入但转子无法自由旋转。
  - 结论: 不是前馈问题，是堵转测试环境。代码v4_safe_baseline已就绪(P0-P4全部关闭，FFDiag已集成UART)，编译0错0告。
- Prevention: 刹车释放前不应再推>0.5A堵转电流。释放后重做低电流方向测试(0.03→0.05→0.10A)，验收角度能连续运动、Iq跟踪误差<30%、无fault，再进入位置闭环和逐项前馈验证。
- Scripts: `C:\Users\xiangyu\Desktop\phase_bcd_postbrake.py` (刹车释放后一键执行B→C→D)
- Recurrence policy: 台架调试前先确认执行器自由度，不把硬件阻塞当软件问题调。

## [2026-06-12 21:30] enc_dir符号修复 + 位置闭环验证通过
- Problem: 电机能自由转动(角度从258°变到341°证明无刹车)，但0.05A~1.0A电流下仅转动<1°。速度模式测试中电机在113°和104.5°两位置间跳变(~8.5°差)，像步进电机卡在零转矩死点。根因是 `enc_dir=1` 但 `pn_dir=-1`——PN识别时微步路径驱动电角负向旋转，`sign(Δθ_elec×Δθ_mech)` 公式因两者同号给出 +1，不代表"正Iq→正机械角"。FOC换向角在特定位置误差~90°，cos(90°)≈0导致零转矩。
- Resolution:
  1. **motor_identify.c 三处修复**: 将 `encoder_dir = sign(Δθ_elec×Δθ_mech)` 改为 `encoder_dir = pn_observed_dir`（基于实际机械运动方向）。最关键的微步PN路径(line 658后)原本 `encoder_dir` 赋值在 `#if MI_PN_AUTO_UPDATE_ENCODER_DIR`(默认0)内，从不执行。现移到 `#endif` 后无条件执行。
  2. 重编译烧录、重识别 → `enc_dir=-1, pn_dir=-1` 一致。扭矩测试0.05A旋转91.6°（修复前仅0.68°）。
  3. **CMD:HOME** 设机械零位 → PREF=0/±5°/±20°全部通过，误差<2°，Iq≈0。
- Prevention: PN识别中enc_dir应始终从pn_observed_dir推断，不依赖δθ_elec符号。识别后必须执行CMD:HOME设零位。任何修改motor_identify.c PN路径后需重跑 `build.ps1 + pyocd load + reidentify + HOME + PREF序列` 验证。
- Commit: (pending)
- Recurrence policy: Not allowed to happen again.
