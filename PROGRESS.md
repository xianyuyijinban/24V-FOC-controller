# PROGRESS

## Mandatory Rules

1. Before each act/execution, read this file and remember prior mistakes and prevention controls.
2. Log an entry after any incident/problem or any major code/config/architecture change.
3. Every entry must include a real git commit SHA.
4. Recurrence of the same issue is not allowed; add concrete prevention controls.
5. Do not close the task before this file is updated.

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
