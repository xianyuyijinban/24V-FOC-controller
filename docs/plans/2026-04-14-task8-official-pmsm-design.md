# Task 8 Official PMSM Plant Design

**Goal:** Replace the hand-written average-value `abc` motor plant in Task 7 with an official MathWorks `Surface Mount PMSM` plant so the simulation produces more realistic three-phase current waveforms while preserving the existing FOC controller structure.

**Current Baseline:** The current model is [minimal_foc_controller_task7_abcplant.slx](D:/matlab/minimal_foc_controller_task7_abcplant.slx). It already exposes `ia`, `ib`, and `ic`, but the plant is still a custom MATLAB Function model with simplified electrical dynamics and observer-fed feedback.

## Why Task 7 Looks Distorted

The visible waveform distortion in Task 7 is not just a plotting issue. The root causes are structural:

1. The plant uses a hand-written phase model with `Ls = 0.5 * (Ld + Lq)` instead of a full PMSM electromagnetic model.
2. The phase-current balance is enforced numerically by subtracting the average derivative rather than by a physical machine model.
3. The controller observes phase currents through quantized mechanical angle feedback and a low-pass differentiated speed estimate, so fast transient segments carry additional phase lag.
4. The test scenario is not a clean constant-speed steady state, so startup, deceleration, and holding torque all appear in one scope.

This is acceptable for a teaching transition model, but not for the next “closer to real hardware” milestone.

## Chosen Approach

Use the official MathWorks `Surface Mount PMSM` plant and keep the controller side largely unchanged:

1. Keep the existing position, speed, and current loops.
2. Keep the existing `FOC math`.
3. Replace the custom `plant` subsystem with an official motor block plus lightweight signal-routing blocks.
4. Keep average-value voltage actuation for now instead of switching to a transistor-level inverter.

This gives a better machine model without paying the full complexity of switch-level PWM and current reconstruction.

## Alternatives Considered

### Option 1: Official PMSM + average-value three-phase voltage drive

This is the selected approach.

- Best balance of realism and complexity
- Preserves the existing controller structure
- Improves `ia/ib/ic` realism without requiring slow switch-level simulation

### Option 2: Keep the custom Task 7 plant and refine its equations

- Smaller immediate change
- Still leaves us maintaining a fragile custom machine model
- Harder to trust than a validated official block

### Option 3: Official PMSM + switch-level inverter + PWM

- Most realistic electrically
- Significantly more blocks, slower simulation, harder debugging
- Too much complexity for the next step

## Architecture

### Controller Side

The following subsystems remain:

- `position loop`
- `speed loop`
- `current loop`
- `FOC math`
- `feedback` observer shell, although its signal source changes

The controller continues to work in:

- `theta_mech`
- `speed_mech`
- `id/iq`
- `vd/vq`
- `duty_a/duty_b/duty_c`

### Inverter Abstraction

The inverter remains average-value in Task 8.

The target behavior is:

- inputs: `duty_a`, `duty_b`, `duty_c`, `Vbus`
- outputs: average-value phase voltages `Va`, `Vb`, `Vc`

If the installed toolboxes provide an official average-value inverter block, prefer it. Otherwise keep the existing custom bridge math but change its output contract to `Va`, `Vb`, `Vc`.

### Official PMSM Plant Assembly

The custom Task 7 `plant` is replaced by a subsystem centered on the official `Surface Mount PMSM` block.

The supporting structure is standard Simulink signal routing:

- phase-voltage `Mux`
- phase-current `Demux`
- `Bus Selector` for `MtrPos`
- direct torque input `Tload_cmd`

Outputs returned to Simulink:

- `ia_meas`
- `ib_meas`
- `ic_meas`
- `speed_mech`
- `theta_mech`

### Feedback Path

The `feedback` subsystem still performs:

- encoder-style angle quantization
- speed estimation / low-pass observation
- Clarke transform
- Park transform

The difference is that its `ia/ib/ic` inputs now come from measured currents of the official PMSM plant instead of the custom `abc_dynamics` block.

## Parameter Mapping

The current initialization script already contains the core PMSM parameters:

- `Rs`
- `Ld`
- `pole_pairs`
- `psi_f`
- `J`
- `B`
- `Tload_init`
- `Vbus`

Task 8 maps them directly into the official `Surface Mount PMSM` block configuration.

The intended first-pass mapping is:

- stator resistance <- `Rs`
- stator inductance <- `Ld`
- permanent magnet flux linkage <- `psi_f`
- pole pairs <- `pole_pairs`
- mechanical vector <- `[J, B, 0]`

Because this is the surface-mount PMSM case, the isotropic inductance assumption aligns with the current script where `Ld = Lq`. This keeps controller tuning and machine parameters consistent with the earlier tasks.

## Data Flow

The Task 8 signal flow becomes:

`position loop -> speed loop -> current loop -> FOC math -> average-value three-phase voltage source -> official Surface Mount PMSM -> current/speed/angle outputs -> feedback -> loops`

This preserves the firmware-style controller view while replacing the machine core with a more trustworthy plant.

## Verification Criteria

Task 8 is considered successful when:

1. The new model loads in MATLAB without missing-block errors.
2. The official `Surface Mount PMSM` block is present in `plant`.
3. `ia + ib + ic` stays close to zero.
4. In a constant-speed window, `ia/ib/ic` look visibly smoother than Task 7.
5. `id_fb` stays near zero in the standard FOC case.
6. `iq_fb` rises appropriately with external load torque.
7. Existing loops still close through `feedback`, not directly from the plant.

## Explicit Non-Goals

Task 8 still does not add:

- switch-level PWM
- transistor deadtime
- current sampling timing
- ADC reconstruction logic
- sensorless estimation

Those remain later upgrades after the official PMSM plant is stable.
