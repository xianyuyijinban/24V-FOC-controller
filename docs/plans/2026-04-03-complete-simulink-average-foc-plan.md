# Complete Simulink Average-Value FOC Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a time-efficient “complete enough” Simulink simulation for this repository that covers the full control chain and realistic motor behavior without dropping into transistor-level switching detail.

**Architecture:** Start from the user's working `minimal_foc_controller` model and upgrade it into an average-value FOC simulation. Keep fixed-step multi-rate timing aligned to firmware (`20 kHz / 2 kHz / 200 Hz`), use dq-domain PMSM plant dynamics plus `Clarke/Park/Inverse Park/SVPWM` math blocks, and add sensor/limit models that are strong enough for controller validation but still fast to run.

**Tech Stack:** MATLAB R2025a, Simulink, repository firmware references in `MDK-ARM/code`, MATLAB scripts, PowerShell `matlab -batch` verification.

---

### Task 1: Freeze the Scope and Parameters

**Files:**
- Modify: `minimal_foc_init.m`
- Create: `docs/simulink/complete-average-foc-scope.md`

**Step 1: Write down the simulation boundary**

Document the target model contents:

- full three-loop control
- dq PMSM plant
- electrical angle generation
- `Clarke/Park/Inverse Park`
- average-value `SVPWM`
- current/angle/speed feedback
- voltage/current/speed saturation

Explicitly exclude:

- switch-level inverter detail
- deadtime
- SPI/UART protocol timing
- full parameter identification workflow

**Step 2: Expand the parameter script**

Add or verify:

```matlab
Ts_current = 50e-6;
Ts_speed = 500e-6;
Ts_position = 5e-3;

Rs = 0.3;
Ld = 1e-3;
Lq = 1e-3;
pole_pairs = 7;
J = 5e-4;
B = 1e-3;
Kt = 0.08;
psi_f = Kt / (1.5 * pole_pairs);
Tload = 0;
Vbus = 24;
theta_offset = 0;
sensor_direction = 1;
encoder_cpr = 65536;

Kp_pos = 10; Ki_pos = 0;
Kp_speed = 0.1; Ki_speed = 0.01;
Kp_id = Ld * 2*pi*2000;
Ki_id = Rs * 2*pi*2000 * Ts_current;
Kp_iq = Lq * 2*pi*2000;
Ki_iq = Rs * 2*pi*2000 * Ts_current;

speed_limit = 50;
iq_limit = 10;
vd_limit = 12;
vq_limit = 12;
```

**Step 3: Verify the script loads**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('minimal_foc_init.m'); whos Ts_current Ts_speed Ts_position Rs Ld Lq pole_pairs J B Kt psi_f Vbus Kp_id Ki_id Kp_iq Ki_iq"
```

Expected:
- MATLAB prints all requested variables with no errors.

### Task 2: Upgrade the Current Loop to dq Dual PI

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model
- Modify: `docs/simulink/complete-average-foc-scope.md`

**Step 1: Refactor `Current Loop` ports**

Change the `Current Loop` subsystem to:

- Inputs: `id_ref`, `iq_ref`, `id_meas`, `iq_meas`
- Outputs: `vd_cmd`, `vq_cmd`

**Step 2: Build the dual PI structure**

Inside `Current Loop`, create:

```text
id_ref - id_meas -> PI_d -> Saturation -> vd_cmd
iq_ref - iq_meas -> PI_q -> Saturation -> vq_cmd
```

Use:

- `PI_d`: `Kp_id`, `Ki_id`, `Ts_current`
- `PI_q`: `Kp_iq`, `Ki_iq`, `Ts_current`

**Step 3: Feed `id_ref = 0`**

At the top level, add a `Constant` block with value `0` and connect it to `Current Loop.id_ref`.

**Step 4: Verify port wiring**

Confirm:

- `Current Loop.vd_cmd -> Plant.vd_cmd`
- `Current Loop.vq_cmd -> Plant.vq_cmd`
- `Plant.id_meas -> Current Loop.id_meas`
- `Plant.iq_meas -> Current Loop.iq_meas`

**Step 5: Run a smoke simulation**

Run the model manually for `1 s`.

Expected:
- no compile errors
- `id_meas` stays near zero
- `iq_meas` still drives acceleration and braking

### Task 3: Add Electrical Angle and Sensor Feedback

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model
- Modify: `docs/simulink/complete-average-foc-scope.md`

**Step 1: Create a `Sensors` subsystem**

Inputs:

- `theta_mech`
- `speed_mech`
- `id_meas`
- `iq_meas`
- `Vbus`

Outputs:

- `theta_elec`
- `speed_elec`
- `theta_mech_fb`
- `speed_mech_fb`
- `id_fb`
- `iq_fb`
- `Vbus_fb`

**Step 2: Implement electrical angle projection**

Inside `Sensors`, compute:

```text
theta_elec = sensor_direction * pole_pairs * theta_mech + theta_offset
speed_elec = sensor_direction * pole_pairs * speed_mech
```

**Step 3: Add minimal realism**

Keep the first sensor model simple:

- pass through `id` and `iq`
- pass through `speed`
- optionally quantize `theta_mech` using `encoder_cpr`

**Step 4: Rewire loops to consume feedback outputs**

Position, speed, and current loops should read feedback from `Sensors`, not directly from `Plant`.

### Task 4: Add FOC Math Blocks

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model
- Reference: `MDK-ARM/code/foc_core.c`
- Reference: `MDK-ARM/code/foc_core.h`

**Step 1: Create a `FOC Math` subsystem**

Inputs:

- `id_meas`
- `iq_meas`
- `theta_elec`
- `vd_cmd`
- `vq_cmd`
- `Vbus_fb`

Outputs:

- `Valpha`
- `Vbeta`
- `duty_a`
- `duty_b`
- `duty_c`

**Step 2: Add inverse Park**

Implement the same equations as firmware:

```text
alpha = d * cos(theta) - q * sin(theta)
beta  = d * sin(theta) + q * cos(theta)
```

**Step 3: Add average-value SVPWM**

Mirror the zero-sequence injection logic from `foc_core.c` and produce:

- `duty_a`
- `duty_b`
- `duty_c`

Use average values, not switch-level PWM pulses.

**Step 4: Add voltage saturation**

Constrain dq voltage commands to the linear modulation region before inverse Park:

```text
sqrt(vd^2 + vq^2) <= Vbus / sqrt(3)
```

### Task 5: Bridge Average-Value Inverter to the dq Plant

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model
- Modify: `plant/MATLAB Function` logic inside the model

**Step 1: Decide the abstraction**

Keep the plant in dq form, but drive it with dq voltage commands that have already passed through:

- dual current PI
- inverse Park
- average-value SVPWM

**Step 2: Avoid over-modeling**

Do not yet reconstruct switch-level phase currents. Instead:

- keep dq plant dynamics
- use the `FOC Math` subsystem primarily for controller-side realism and observability

**Step 3: Expose debug quantities**

At the top level, log:

- `vd_cmd`
- `vq_cmd`
- `Valpha`
- `Vbeta`
- `duty_a`
- `duty_b`
- `duty_c`

### Task 6: Align Multi-Rate Scheduling to Firmware

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model
- Reference: `MDK-ARM/code/foc_app.h`
- Reference: `MDK-ARM/code/foc_app.c`

**Step 1: Keep solver fixed-step**

Model settings:

- fixed-step
- discrete
- `step size = 5e-5`

**Step 2: Enforce subsystem sample times**

Use:

- position loop: `Ts_position`
- speed loop: `Ts_speed`
- current loop and plant: `Ts_current`

**Step 3: Verify timing behavior**

Run the simulation and confirm:

- `position loop` updates every `5 ms`
- `speed loop` updates every `0.5 ms`
- current loop and plant update every `50 us`

### Task 7: Add Test Scenarios

**Files:**
- Create: `docs/simulink/complete-average-foc-test-cases.md`
- Modify: the existing `minimal_foc_controller.slx` model

**Step 1: Add a position step case**

Scenario:

- `pos_ref: 0 -> 1 rad at 0.05 s`

Expected:

- `theta_mech` tracks target
- bounded overshoot
- `iq_meas` accelerates then brakes

**Step 2: Add a speed step case**

Scenario:

- bypass position loop
- command `speed_ref = 10 rad/s`

Expected:

- stable speed tracking
- `id_meas` remains near `0`

**Step 3: Add a load disturbance case**

Scenario:

- `Tload = 0.02 N·m` step at `0.3 s`

Expected:

- speed dips
- current rises to compensate

**Step 4: Add a bus variation case**

Scenario:

- reduce `Vbus` from `24 V` to `20 V`

Expected:

- voltage saturation becomes visible
- tracking degrades gracefully, not catastrophically

### Task 8: Create a Single Operator Dashboard

**Files:**
- Modify: the existing `minimal_foc_controller.slx` model

**Step 1: Replace scattered scopes with grouped views**

Create grouped observation channels:

- `pos_ref`, `theta_mech`
- `speed_ref`, `speed_mech`
- `id_ref`, `id_meas`
- `iq_ref`, `iq_meas`
- `vd_cmd`, `vq_cmd`
- `Te`, `omega_e`
- `duty_a`, `duty_b`, `duty_c`

**Step 2: Use one or two main scopes**

Prefer:

- one large scope with multiple panes
- or two grouped scopes:
  - motion/current
  - voltage/modulation/debug

### Task 9: Verify from MATLAB Batch

**Files:**
- Verify only

**Step 1: Load the parameter script**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('minimal_foc_init.m'); disp('INIT_OK')"
```

Expected:
- `INIT_OK`

**Step 2: Open the model**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "load_system('minimal_foc_controller'); disp('MODEL_OK')"
```

Expected:
- `MODEL_OK`

**Step 3: Simulate once**

Run:

```powershell
& 'C:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('minimal_foc_init.m'); load_system('minimal_foc_controller'); simOut = sim('minimal_foc_controller'); disp('SIM_OK')"
```

Expected:
- `SIM_OK`

### Task 10: Freeze a Deliverable Version

**Files:**
- Modify: `docs/simulink/complete-average-foc-scope.md`
- Modify: `docs/simulink/complete-average-foc-test-cases.md`
- Modify: the existing `minimal_foc_controller.slx` model

**Step 1: Save a clean model version**

Save the model after:

- dq dual current loop works
- sensor feedback is wired
- FOC math blocks exist
- grouped scopes exist

**Step 2: Record what is still intentionally omitted**

Document:

- no switch-level inverter
- no deadtime
- no communication timing
- no full identification workflow

**Step 3: Commit**

```bash
git add minimal_foc_init.m minimal_foc_controller.slx docs/simulink/complete-average-foc-scope.md docs/simulink/complete-average-foc-test-cases.md docs/plans/2026-04-03-complete-simulink-average-foc-plan.md
git commit -m "feat: add complete average-value simulink foc plan"
```
