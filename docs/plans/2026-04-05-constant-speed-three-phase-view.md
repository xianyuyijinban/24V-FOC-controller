# Constant Speed Three-Phase View Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a Simulink demo model that runs the motor at a constant speed and exposes the average three-phase voltage waveforms `Va/Vb/Vc` on a dedicated scope.

**Architecture:** Start from the validated average-value bridge model, switch the outer-loop operating mode from position-driven to direct constant-speed command, expand the average inverter bridge to output reconstructed three-phase average phase voltages, and route those signals to a dedicated top-level scope. Keep the dq plant and average-value inverter abstraction unchanged.

**Tech Stack:** MATLAB R2025a, Simulink, MATLAB batch verification, external model files in `D:\matlab`

---

### Task 1: Define the demo contract with a failing check

**Files:**
- Verify: `D:\matlab\minimal_foc_controller_task5_avgbridge.slx`

**Step 1: Check for the future three-phase scope**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_task5_avgbridge.slx'); mdl='minimal_foc_controller_task5_avgbridge'; hasScope=~isempty(find_system(mdl,'SearchDepth',1,'Name','three_phase_scope')); if ~hasScope, error('three_phase_scope missing'); end;"
```

Expected:
- FAIL with `three_phase_scope missing`

### Task 2: Create the constant-speed three-phase demo model

**Files:**
- Create: `D:\matlab\upgrade_constant_speed_three_phase_view.m`
- Create: `D:\matlab\minimal_foc_controller_constspeed_3phase.slx`

**Step 1: Copy the Task 5 model**

Base the new model on `minimal_foc_controller_task5_avgbridge.slx`.

**Step 2: Switch to direct speed command**

- Disconnect `position loop` from `speed loop`
- Add a `speed_cmd_const` block
- Feed `speed loop` directly from the constant speed command
- Set the mechanical speed integrator initial condition to the same speed so the waveform is immediately close to steady-state

**Step 3: Expand the average bridge outputs**

Make `avg inverter bridge` output:

- `vd_applied`
- `vq_applied`
- `va_avg`
- `vb_avg`
- `vc_avg`

**Step 4: Add a dedicated three-phase scope**

At the top level, add:

- `three_phase_mux`
- `three_phase_scope`

and route `va_avg/vb_avg/vc_avg` into it.

### Task 3: Verify the demo model

**Files:**
- Verify only

**Step 1: Confirm the new scope exists**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_constspeed_3phase.slx'); mdl='minimal_foc_controller_constspeed_3phase'; assert(~isempty(find_system(mdl,'SearchDepth',1,'Name','three_phase_scope'))); disp('SCOPE_OK');"
```

Expected:
- `SCOPE_OK`

**Step 2: Simulate the new model**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('D:/matlab/minimal_foc_init.m'); load_system('D:/matlab/minimal_foc_controller_constspeed_3phase.slx'); sim('minimal_foc_controller_constspeed_3phase','StopTime','0.2'); disp('SIM_OK_CONST_SPEED_3PH');"
```

Expected:
- `SIM_OK_CONST_SPEED_3PH`
