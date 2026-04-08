# Task 7 ABC Plant Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a new Simulink model that upgrades the Task 6 observer/load baseline to an average-value `abc` SPMSM plant with visible `ia`, `ib`, and `ic` phase-current outputs.

**Architecture:** Copy the validated Task 6 model into a new Task 7 model, replace the dq electrical plant with a phase-domain MATLAB Function plus discrete integrators, rebuild the feedback subsystem so it converts `ia/ib/ic` into `id_fb/iq_fb` through Clarke/Park using observed electrical angle, and expose the three-phase currents for scopes and batch verification.

**Tech Stack:** MATLAB R2025a, Simulink, MATLAB scripts, PowerShell `matlab -batch`, external model files under `D:\matlab`

---

### Task 1: Freeze the approved Task 7 design in repo

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-08-task7-abcplant-design.md`
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-08-task7-abcplant-implementation.md`

**Step 1: Record the chosen realism boundary**

Document that Task 7:

- keeps the average-value inverter bridge
- replaces the dq plant with an `abc` average-value motor model
- reconstructs `id/iq` in `feedback`
- exposes `ia/ib/ic`

**Step 2: Record explicit non-goals**

Document that Task 7 still excludes:

- switch-level PWM
- deadtime
- current reconstruction timing
- sensorless observer logic

### Task 2: Write and run a failing pre-check against Task 6

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task7_red.m`

**Step 1: Write a red check for missing phase-current outputs**

The script should:

- load `D:\matlab\minimal_foc_controller_task6_observer_load.slx`
- assert that `plant` already has `5` outputs
- assert that `feedback` already has `6` inputs

These assertions must fail on Task 6.

**Step 2: Verify the red state**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/check_task7_red.m')"
```

Expected:

- MATLAB exits with an assertion failure proving Task 6 does not yet provide the Task 7 interfaces.

### Task 3: Create the Task 7 upgrade script and output model

**Files:**
- Create: `D:\matlab\upgrade_add_abc_plant.m`
- Create: `D:\matlab\minimal_foc_controller_task7_abcplant.slx`

**Step 1: Copy the Task 6 model**

Use `minimal_foc_controller_task6_observer_load.slx` as the base model.

**Step 2: Rebuild the plant subsystem**

Plant target interface:

- Inputs: `v_alpha`, `v_beta`, `Tload_cmd`
- Outputs: `ia_meas`, `ib_meas`, `ic_meas`, `speed_mech`, `theta_mech`

Inside the plant:

- keep discrete integrators for `ia`, `ib`, `ic`, `speed_mech`, `theta_mech`
- use a MATLAB Function to compute `dia`, `dib`, `dic`, `domega`, `dtheta`, and optional debug torque terms
- enforce balanced three-phase current behavior

**Step 3: Rebuild the feedback subsystem**

Feedback target interface:

- Inputs: `theta_mech`, `speed_mech`, `ia_meas`, `ib_meas`, `ic_meas`, `Vbus`
- Outputs: `theta_elec`, `speed_elec`, `theta_mech_fb`, `speed_mech_fb`, `id_fb`, `iq_fb`, `Vbus_fb`

Inside feedback:

- keep the Task 6 encoder-style observer
- add Clarke transform
- add Park transform
- output reconstructed `id_fb`, `iq_fb`

**Step 4: Rewire the top level**

Required rewiring:

- `avg inverter bridge/1 -> plant/2`
- `avg inverter bridge/2 -> plant/1`
- `Tload_cmd/1 -> plant/3`
- `plant/1, plant/2, plant/3 -> feedback phase-current inputs`
- `plant/4 -> speed_mech`
- `plant/4 -> feedback speed input`
- `plant/5 -> theta_mech`
- `plant/5 -> feedback theta input`
- `feedback/id_fb -> current loop/id_meas`
- `feedback/iq_fb -> current loop/iq_meas`
- `feedback/theta_elec -> FOC math/theta_elec`
- `feedback/theta_elec -> avg inverter bridge/theta_elec`

**Step 5: Add three-phase observability**

Ensure `ia`, `ib`, and `ic` are available for grouped scope/debug output at the top level.

### Task 4: Write and run green checks against Task 7

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task7_green.m`

**Step 1: Verify the new interfaces**

The script should:

- load `D:\matlab\minimal_foc_controller_task7_abcplant.slx`
- assert `plant` has `3` inputs and `5` outputs
- assert `feedback` has `6` inputs and `7` outputs
- verify the current loop still receives `feedback` outputs

**Step 2: Verify the model simulates**

The script should:

- run `D:\matlab\minimal_foc_init.m`
- simulate the model for `0.2 s`
- report success markers for load, structure, and simulation

**Step 3: Verify the phase currents are non-trivial**

The script should log `ia`, `ib`, `ic` from the simulation and assert:

- each current has non-zero variation
- at least two phases differ over time

### Task 5: Record the completed architecture change

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\PROCESS.md`

**Step 1: Append a Task 7 entry**

Record:

- the problem with the old dq-only internal plant
- the new `abc` average-value motor architecture
- the new guardrail: always prove `ia/ib/ic` exist in batch before claiming realism upgrades are done

### Task 6: Commit the documentation and verification trail

**Files:**
- Stage only the new Task 7 repo docs and verification scripts

**Step 1: Commit the repo-side Task 7 artifacts**

Use a focused commit that includes:

- Task 7 design doc
- Task 7 implementation plan
- Task 7 red/green check scripts
- `PROCESS.md`

```bash
git add docs/plans/2026-04-08-task7-abcplant-design.md docs/plans/2026-04-08-task7-abcplant-implementation.md tools/simulink/check_task7_red.m tools/simulink/check_task7_green.m PROCESS.md
git commit -m "docs: record task7 abc plant simulink upgrade"
```
