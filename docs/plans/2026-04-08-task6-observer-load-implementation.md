# Task 6 Observer And Load Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a new Simulink model that adds an encoder-style observer layer and an external load torque input on top of the current average-value FOC baseline.

**Architecture:** Copy the validated Task 5 average-bridge model into a new Task 6 model, upgrade the `feedback` subsystem into a quantized-angle plus filtered-speed observer, rewire the position/speed loops to consume observer outputs, and add a third plant input `Tload_cmd` with a top-level constant initialized to `0.2 N·m`.

**Tech Stack:** MATLAB R2025a, Simulink, MATLAB scripts, batch verification through `matlab -batch`, external model files under `D:\matlab`

---

### Task 1: Extend the initialization script

**Files:**
- Modify: `D:\matlab\minimal_foc_init.m`

**Step 1: Add observer and external-load parameters**

Add:

```matlab
Tload_init = 0.2;
speed_obs_tau = 2e-3;
speed_obs_alpha = exp(-Ts_current / speed_obs_tau);
```

**Step 2: Verify the new parameters load**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('D:/matlab/minimal_foc_init.m'); whos Tload_init speed_obs_tau speed_obs_alpha"
```

Expected:
- MATLAB prints the three variables with no errors.

### Task 2: Prove the current model is missing the new interfaces

**Files:**
- Verify only: `D:\matlab\minimal_foc_controller_task5_avgbridge.slx`

**Step 1: Check that the current plant still has only two inputs**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_task5_avgbridge.slx'); mdl='minimal_foc_controller_task5_avgbridge'; ports=get_param([mdl '/plant'],'Ports'); assert(ports(1)==3,'plant should already expose Tload_cmd');"
```

Expected:
- FAIL because the current plant does not yet expose `Tload_cmd`.

**Step 2: Check that the speed loop is still fed directly from the plant**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_task5_avgbridge.slx'); mdl='minimal_foc_controller_task5_avgbridge'; ph=get_param([mdl '/speed loop'],'PortHandles'); lh=get_param(ph.Inport(1),'Line'); sp=get_param(lh,'SrcPortHandle'); src=getfullname(get_param(sp,'Parent')); assert(strcmp(src,[mdl '/feedback']),'speed loop should already be fed by feedback');"
```

Expected:
- FAIL because the speed loop still reads the plant directly.

### Task 3: Create the Task 6 upgrade script and model

**Files:**
- Create: `D:\matlab\upgrade_add_observer_and_load.m`
- Create: `D:\matlab\minimal_foc_controller_task6_observer_load.slx`

**Step 1: Copy the Task 5 model**

Use `minimal_foc_controller_task5_avgbridge.slx` as the base.

**Step 2: Upgrade the feedback subsystem**

Inside `feedback`:

- quantize `theta_mech` using `encoder_cpr`
- compute discrete angle difference
- divide by `Ts_current` for raw speed estimate
- filter it with a discrete first-order low-pass using `speed_obs_alpha`
- emit observed mechanical/electrical angle and speed

**Step 3: Rewire top-level loops**

Top-level rewiring must be:

- `feedback/theta_mech_fb -> position loop`
- `feedback/speed_mech_fb -> speed loop`
- `feedback/id_fb -> current loop`
- `feedback/iq_fb -> current loop`
- `feedback/theta_elec -> FOC math`
- `feedback/theta_elec -> avg inverter bridge`

**Step 4: Add the plant load input**

- add top-level constant block `Tload_cmd`
- set it to `Tload_init`
- add plant inport 3
- modify the plant MATLAB Function to accept `Tload_cmd`

### Task 4: Verify the Task 6 model

**Files:**
- Verify only

**Step 1: Check the plant input count**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_task6_observer_load.slx'); mdl='minimal_foc_controller_task6_observer_load'; ports=get_param([mdl '/plant'],'Ports'); assert(ports(1)==3); disp('PLANT_PORTS_OK');"
```

Expected:
- `PLANT_PORTS_OK`

**Step 2: Check feedback rewiring**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('D:/matlab/minimal_foc_controller_task6_observer_load.slx'); mdl='minimal_foc_controller_task6_observer_load'; ph=get_param([mdl '/speed loop'],'PortHandles'); lh=get_param(ph.Inport(1),'Line'); sp=get_param(lh,'SrcPortHandle'); src=getfullname(get_param(sp,'Parent')); assert(strcmp(src,[mdl '/feedback'])); disp('FEEDBACK_REWIRE_OK');"
```

Expected:
- `FEEDBACK_REWIRE_OK`

**Step 3: Simulate the model**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('D:/matlab/minimal_foc_init.m'); load_system('D:/matlab/minimal_foc_controller_task6_observer_load.slx'); sim('minimal_foc_controller_task6_observer_load','StopTime','0.5'); disp('SIM_OK_TASK6');"
```

Expected:
- `SIM_OK_TASK6`
