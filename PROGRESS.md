# PROGRESS

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
