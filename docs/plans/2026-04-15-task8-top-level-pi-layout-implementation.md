# Task 8 Top-Level PI Layout Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Expose the Task 8 three-loop PI gains at the top level and reorganize the Task 8 top-level Simulink layout without breaking the official PMSM simulation path.

**Architecture:** Keep the current Task 8 official-PMSM model generation flow, but extend the upgrade script so it rebuilds the three loop subsystems with explicit gain inputs, adds a top-level PI parameter zone, rewires the top level, and repositions major blocks into a readable signal-flow layout.

**Tech Stack:** MATLAB R2025a, Simulink, repository-side MATLAB generation scripts, external model files under `D:\matlab`

---

### Task 1: Freeze the approved layout-and-tuning design

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-15-task8-top-level-pi-layout-design.md`
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-15-task8-top-level-pi-layout-implementation.md`

**Step 1: Record the user-visible goal**

Document:

- top-level editable PI gains
- no hidden tuning-only workflow
- top-level readability over cosmetic perfection

**Step 2: Record the chosen wiring contract**

Document:

- `position loop` gains from top level
- `speed loop` gains from top level
- `current loop` gains from top level
- official PMSM path unchanged

### Task 2: Add a failing structural check for the new tuning interface

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task8_green.m`

**Step 1: Add structural assertions for the new top-level PI blocks**

Check for:

- `Kp_pos`
- `Ki_pos`
- `Kp_speed`
- `Ki_speed`
- `Kp_iq`
- `Ki_iq`

**Step 2: Add loop port-count assertions**

Check for:

- `position loop`: `4` inputs
- `speed loop`: `4` inputs
- `current loop`: `6` inputs

**Step 3: Verify the check would fail on the pre-change model**

Expected:
- the new assertions fail until the upgrade script is updated and rerun

### Task 3: Rebuild the loop subsystems with explicit gain inputs

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`

**Step 1: Rebuild `position loop`**

Create a simple discrete PI subsystem with ports:

- `pos_ref`
- `theta_mech_fb`
- `Kp_pos`
- `Ki_pos`
- output `speed_ref`

Use:

- proportional path: `Kp_pos * error`
- integral path: `Ki_pos * error` into a discrete integrator with `Ts_position`
- output saturation to `±speed_limit`

**Step 2: Rebuild `speed loop`**

Create ports:

- `speed_ref`
- `speed_mech_fb`
- `Kp_speed`
- `Ki_speed`
- output `iq_ref`

Use:

- proportional path: `Kp_speed * error`
- integral path: `Ki_speed * error` into a discrete integrator with `Ts_speed`
- output saturation to `±iq_limit`

**Step 3: Rebuild `current loop`**

Create ports:

- `id_ref`
- `iq_ref`
- `id_meas`
- `iq_meas`
- `Kp_iq`
- `Ki_iq`
- outputs `vd_cmd`, `vq_cmd`

Use:

- identical discrete PI structure on both `d` and `q`
- `Ts_current`
- output saturation to `±vq_limit`

### Task 4: Add a top-level PI parameter zone

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`

**Step 1: Add six top-level constant blocks**

Create:

- `Kp_pos`
- `Ki_pos`
- `Kp_speed`
- `Ki_speed`
- `Kp_iq`
- `Ki_iq`

Initialize each block with the corresponding workspace variable name so the existing `minimal_foc_init.m` defaults still apply.

**Step 2: Rewire the gains into the loop subsystem inputs**

Ensure:

- position-loop gain ports are driven by `Kp_pos` and `Ki_pos`
- speed-loop gain ports are driven by `Kp_speed` and `Ki_speed`
- current-loop gain ports are driven by `Kp_iq` and `Ki_iq`

### Task 5: Clean up the top-level layout

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`

**Step 1: Reposition the major functional blocks**

Move:

- `feedback`
- command source
- PI parameter blocks
- `position loop`
- `speed loop`
- `current loop`
- `FOC math`
- `avg inverter bridge`
- `plant`

**Step 2: Reposition observation blocks away from the main chain**

Move:

- `phase_currents`
- `phase_currents_mux`
- `phase_currents_log`
- `speed_mech`
- `theta_mech`
- `foc_debug_scope`

**Step 3: Keep the functional rewiring explicit**

Do not rely on old inherited lines for the main chain. Reconnect the main top-level path in the upgrade script so reruns remain deterministic.

### Task 6: Extend the green check to cover the new operator interface

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task8_green.m`

**Step 1: Verify the PI blocks exist**

Assert the top-level model contains the six new PI parameter blocks.

**Step 2: Verify gain wiring**

Assert:

- `position loop/3` comes from `Kp_pos`
- `position loop/4` comes from `Ki_pos`
- `speed loop/3` comes from `Kp_speed`
- `speed loop/4` comes from `Ki_speed`
- `current loop/5` comes from `Kp_iq`
- `current loop/6` comes from `Ki_iq`

**Step 3: Keep the existing simulation check**

Retain:

- official PMSM presence
- feedback wiring
- `phase_currents_log`
- three-phase current quality gates

### Task 7: Regenerate and verify externally

**Files:**
- Run externally from workspace script: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`

**Step 1: Regenerate Task 8**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/upgrade_add_official_pmsm.m')"
```

Expected:

- `D:\matlab\minimal_foc_controller_task8_official_pmsm.slx` is recreated successfully

**Step 2: Run the updated green check**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/check_task8_green.m')"
```

Expected:

- structural checks pass
- simulation passes
- `SIM_OK_TASK8` is printed
