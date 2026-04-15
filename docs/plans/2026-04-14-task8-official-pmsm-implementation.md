# Task 8 Official PMSM Plant Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Upgrade the current Simulink FOC model from the custom Task 7 `abc` motor plant to an official `Surface Mount PMSM` block while keeping the current controller structure.

**Architecture:** Preserve the position/speed/current loops and `FOC math`, replace the custom `plant` with a subsystem centered on the official `Surface Mount PMSM` block, and feed measured `ia/ib/ic`, `speed_mech`, and `theta_mech` back through the existing `feedback` subsystem.

**Tech Stack:** MATLAB R2025a, Simulink, Motor Control Blockset, optional average-value inverter block, PowerShell `matlab -batch`, external model files under `D:\matlab`

---

### Task 1: Freeze the approved Task 8 design

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-14-task8-official-pmsm-design.md`
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\docs\plans\2026-04-14-task8-official-pmsm-implementation.md`

**Step 1: Record the root cause of Task 7 waveform distortion**

Document:

- simplified custom phase model
- numerical current balancing
- observer phase lag
- mixed transient test window

**Step 2: Record the new architecture**

Document:

- official `Surface Mount PMSM` plant
- phase-voltage `Mux`, phase-current `Demux`, and `MtrPos` bus extraction
- average-value three-phase voltage actuation
- existing FOC controller retained

### Task 2: Prove Task 7 still uses the custom plant

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task8_red.m`

**Step 1: Write the red check**

The script should:

- load `D:\matlab\minimal_foc_controller_task7_abcplant.slx`
- assert that the plant already contains an official `Surface Mount PMSM` or equivalent official PMSM block
- assert that the plant already exposes official phase-current output paths

These assertions must fail on Task 7.

**Step 2: Run the red check**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/check_task8_red.m')"
```

Expected:

- MATLAB fails with assertions proving Task 7 still uses the custom plant.

### Task 3: Create the Task 8 upgrade script and output model

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`
- Create externally when run: `D:\matlab\minimal_foc_controller_task8_official_pmsm.slx`

**Step 1: Copy the Task 7 model**

Use `D:\matlab\minimal_foc_controller_task7_abcplant.slx` as the base.

**Step 2: Rebuild the inverter output contract**

Change the average-value bridge so it outputs:

- `Va`
- `Vb`
- `Vc`

If an official average-value inverter block is available, prefer it. Otherwise retain the existing bridge math and only change its outputs.

**Step 3: Rebuild the plant around the official PMSM block**

Inside `plant`, add:

- `Surface Mount PMSM`
- phase-voltage `Mux`
- phase-current `Demux`
- `Bus Selector` for `MtrPos`

**Step 4: Map initialization parameters**

Configure the official `Surface Mount PMSM` with:

- `Rs`
- `Ld`
- `psi_f`
- `pole_pairs`
- `[J, B, 0]`
- `Ts_current`

**Step 5: Rewire feedback**

Ensure `feedback` inputs now come from:

- measured `ia`
- measured `ib`
- measured `ic`
- measured `speed_mech`
- measured `theta_mech`
- `Vbus`

### Task 4: Add verification for the official PMSM path

**Files:**
- Create: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\check_task8_green.m`

**Step 1: Verify structure**

The script should:

- load `D:\matlab\minimal_foc_controller_task8_official_pmsm.slx`
- verify the plant contains an official `Surface Mount PMSM`
- verify `feedback` still feeds the current loop

**Step 2: Verify simulation**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/check_task8_green.m')"
```

The script should:

- run the init script
- simulate the model
- log `ia`, `ib`, `ic`
- report success markers

**Step 3: Verify waveform quality gates**

Assert:

- `max(abs(ia + ib + ic))` stays small
- each phase has non-zero variation
- a selected constant-speed window is smoother than the Task 7 baseline

### Task 5: Record the architecture upgrade

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\process.md`

**Step 1: Append a Task 8 entry**

Record:

- why the custom Task 7 plant was insufficient
- why the official `Surface Mount PMSM` plant replaced it
- the new verification guardrail for `ia+ib+ic`, `id_fb`, and constant-speed waveform quality

### Task 6: Commit the Task 8 planning and verification scaffolding

**Files:**
- Stage only the Task 8 repo-side artifacts

**Step 1: Commit the plan-side files**

```bash
git add docs/plans/2026-04-14-task8-official-pmsm-design.md docs/plans/2026-04-14-task8-official-pmsm-implementation.md tools/simulink/check_task8_red.m tools/simulink/check_task8_green.m process.md
git commit -m "docs: plan task8 official pmsm upgrade"
```

### Task 7: Execute the external model upgrade

**Files:**
- Run externally from workspace script: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\tools\simulink\upgrade_add_official_pmsm.m`

**Step 1: Generate the model**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('C:/Users/xiangyu/24V_FOC_Controller_audit_20260222/tools/simulink/upgrade_add_official_pmsm.m')"
```

Expected:

- `D:\matlab\minimal_foc_controller_task8_official_pmsm.slx` is created or updated.

**Step 2: Run the green check**

Run the `check_task8_green.m` script immediately after generation and do not claim completion without fresh evidence.
