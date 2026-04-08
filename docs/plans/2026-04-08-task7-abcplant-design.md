# Task 7 ABC Plant Design

**Goal:** Upgrade the current Task 6 average-value FOC Simulink model from a dq-domain internal motor plant to an SPMSM-style `abc` average-value plant so the simulation exposes real three-phase currents while keeping the existing position, speed, and current-loop architecture intact.

**Current Baseline:** The working baseline is [minimal_foc_controller_task6_observer_load.slx](D:/matlab/minimal_foc_controller_task6_observer_load.slx). It already contains the average-value inverter bridge, observer-based feedback layer, and external load torque input, but the motor electrical dynamics are still simulated directly in dq coordinates and therefore do not expose `ia`, `ib`, and `ic`.

## Chosen Approach

Use an average-value `abc` SPMSM plant while keeping the existing controller-side abstraction:

1. Keep the current `FOC math` and average-value inverter bridge.
2. Replace the plant electrical model with phase-current dynamics in `abc`.
3. Reconstruct `id` and `iq` inside `feedback` from measured `ia`, `ib`, `ic` plus observed electrical angle.
4. Expose `ia`, `ib`, and `ic` at the top level for scope/debug use.

This is the shortest path to “more realistic than dq-only” without paying the complexity and runtime cost of switch-level PWM, deadtime, and current-reconstruction timing.

## Why This Approach

Three implementation directions were considered:

1. Keep the dq plant and synthesize fake `abc` currents only for display.
2. Move to an average-value `abc` plant.
3. Jump directly to a switch-level inverter plus phase-domain plant.

Option 2 is the right step now.

- Option 1 is fast but misleading because the three-phase waveforms are only post-processed visuals, not the real internal plant state.
- Option 3 is closer to hardware, but it adds PWM carrier effects, deadtime, switching ripple, and solver/runtime cost before the user has a stable intermediate model.
- Option 2 adds genuine phase-current dynamics and observable `ia/ib/ic` while preserving the current teaching-friendly workflow.

## Architecture

### Top Level

The target output model is [minimal_foc_controller_task7_abcplant.slx](D:/matlab/minimal_foc_controller_task7_abcplant.slx).

Top-level structure remains:

`pos_ref -> position loop -> speed_ref -> speed loop -> iq_ref -> current loop -> FOC math -> avg inverter bridge -> plant`

The differences are:

- `plant` now accepts phase-voltage-equivalent inputs rather than dq state variables.
- `plant` outputs `ia`, `ib`, `ic`, `speed_mech`, and `theta_mech`.
- `feedback` consumes `ia`, `ib`, `ic` and converts them to `id_fb`, `iq_fb`.
- `ia`, `ib`, `ic` are logged or sent to grouped scopes for waveform viewing.

### Plant Subsystem

Inputs:

- `v_alpha`
- `v_beta`
- `Tload_cmd`

Outputs:

- `ia_meas`
- `ib_meas`
- `ic_meas`
- `speed_mech`
- `theta_mech`

Internal behavior:

- Compute electrical angle `theta_elec = pole_pairs * theta_mech`
- Transform `v_alpha`, `v_beta` into phase voltages `va`, `vb`, `vc`
- Compute back-EMF phase terms from rotor electrical angle and speed
- Integrate `ia`, `ib`, `ic` with a simplified SPMSM phase model
- Compute electromagnetic torque from the reconstructed `iq`
- Update mechanical speed and angle using `J`, `B`, and external `Tload_cmd`

This remains an average-value model: no switching ripple, no deadtime, no sampling skew.

### Feedback Subsystem

Inputs become:

- `theta_mech`
- `speed_mech`
- `ia_meas`
- `ib_meas`
- `ic_meas`
- `Vbus`

Outputs remain compatible with the controller:

- `theta_elec`
- `speed_elec`
- `theta_mech_fb`
- `speed_mech_fb`
- `id_fb`
- `iq_fb`
- `Vbus_fb`
- optional pass-through phase outputs for debug taps if convenient

Internal behavior:

- Keep the encoder quantization plus low-pass speed observer from Task 6
- Derive observed `theta_elec` and `speed_elec` from the mechanical observer
- Run Clarke transform on `ia`, `ib`, `ic`
- Run Park transform using observed `theta_elec`
- Feed the resulting `id_fb` and `iq_fb` back into the existing current loop

This keeps the controller topology unchanged even though the plant is now phase-domain.

## Parameters

The existing parameter script [minimal_foc_init.m](D:/matlab/minimal_foc_init.m) already contains the motor constants needed for the first `abc` version:

- `Rs`
- `Ld`
- `Lq`
- `psi_f`
- `pole_pairs`
- `J`
- `B`
- `Ts_current`
- `Tload_init`

For the first pass we assume a surface PMSM with:

- `Ld = Lq`
- sinusoidal back-EMF
- balanced three-phase set
- isolated neutral with `ia + ib + ic = 0` enforced numerically

No new mandatory user-facing parameters are required unless the implementation needs an explicit `phase_offset` term later.

## Data Flow

Controller side:

`id_ref/iq_ref -> current loop -> vd_cmd/vq_cmd -> FOC math -> v_alpha/v_beta -> avg inverter bridge`

Plant side:

`v_alpha/v_beta + theta_mech + speed_mech -> abc plant -> ia/ib/ic + theta_mech + speed_mech`

Feedback side:

`theta_mech/speed_mech -> observer`

`ia/ib/ic + theta_elec -> Clarke/Park -> id_fb/iq_fb`

This preserves the firmware-like controller view while making the electrical machine state phase-domain.

## Verification

The design is complete when all of the following are true:

1. `minimal_foc_controller_task7_abcplant.slx` loads in MATLAB batch.
2. `plant` exposes `5` outputs with `ia`, `ib`, `ic`, `speed_mech`, `theta_mech`.
3. `feedback` accepts `ia`, `ib`, `ic` and still outputs usable `id_fb`, `iq_fb`.
4. The model simulates in batch with no compile/runtime errors.
5. Logged `ia`, `ib`, `ic` are non-trivial and phase-shifted rather than flat lines.
6. The existing loops still run using reconstructed `id/iq`, not direct plant dq states.

## Explicit Non-Goals

This task does not add:

- switch-level PWM pulses
- MOSFET deadtime
- current-sampling windows and low-side reconstruction
- sensorless back-EMF or PLL observers
- saturation, saliency, or detailed iron-loss models

Those belong to later realism upgrades after the `abc` average-value path is stable and easy to use.
