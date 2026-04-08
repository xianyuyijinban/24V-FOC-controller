# Task 6 Observer And Load Design

**Goal:** Upgrade the current average-value FOC Simulink model so the control loops consume observer-style mechanical feedback and the plant accepts an external load torque input.

**Current Baseline:** The working baseline is [minimal_foc_controller_task5_avgbridge.slx](D:/matlab/minimal_foc_controller_task5_avgbridge.slx). It already includes the average-value inverter bridge, dq-domain plant, and FOC math, but the position and speed loops still rely on ideal plant outputs and the load torque is still hard-coded inside the plant dynamics.

## Scope

This design adds two realism layers without jumping to a switch-level inverter or sensorless control:

1. A mechanical observer layer that mimics an encoder-style feedback chain.
2. An external load torque input that enters the plant mechanical equation directly.

The target output model is [minimal_foc_controller_task6_observer_load.slx](D:/matlab/minimal_foc_controller_task6_observer_load.slx).

## Architecture

### Observer Layer

The existing `feedback` subsystem will become a real observer/measurement layer rather than a pure signal pass-through.

Inputs remain:

- `theta_mech`
- `speed_mech`
- `id_meas`
- `iq_meas`
- `Vbus`

Outputs remain:

- `theta_elec`
- `speed_elec`
- `theta_mech_fb`
- `speed_mech_fb`
- `id_fb`
- `iq_fb`
- `Vbus_fb`

The internal behavior changes:

- `theta_mech_fb` is derived from quantized encoder counts based on `encoder_cpr`
- `speed_mech_fb` is computed from discrete angle difference and filtered by a first-order low-pass observer
- `theta_elec` and `speed_elec` are computed from the observed mechanical values, not the ideal plant values
- `id_fb`, `iq_fb`, and `Vbus_fb` stay as pass-through signals for now

This keeps the model aligned with a sensored FOC system while introducing realistic delay, quantization, and differentiation noise.

### External Load Input

The plant subsystem gains a new top-level input:

- `Tload_cmd`

The plant dynamics change from using a fixed workspace parameter `Tload` to using the external signal `Tload_cmd` in the mechanical equation:

`domega = (Te - B * speed_mech - Tload_cmd) / J`

At the top level, a new constant block drives `Tload_cmd` with an initial value of `0.2`. The user can later edit that block directly.

## Data Flow

The closed-loop structure after this task becomes:

`pos_ref -> position loop -> speed_ref -> speed loop -> iq_ref -> current loop -> FOC math -> avg inverter bridge -> plant`

Observed feedback path:

`plant(theta_mech, speed_mech, id, iq) -> feedback observer -> theta_mech_fb, speed_mech_fb, theta_elec, speed_elec, id_fb, iq_fb, Vbus_fb`

Consumers:

- `position loop` uses `theta_mech_fb`
- `speed loop` uses `speed_mech_fb`
- `current loop` continues to use `id_fb` and `iq_fb`
- `FOC math` and `avg inverter bridge` use `theta_elec`

## Parameters

The initial parameter script [minimal_foc_init.m](D:/matlab/minimal_foc_init.m) will be extended with a small observer parameter set:

- `Tload_init = 0.2`
- `speed_obs_tau = 2e-3`
- `speed_obs_alpha = exp(-Ts_current / speed_obs_tau)` or an equivalent discrete low-pass coefficient

These values are intentionally simple and tunable rather than physically over-fitted.

## Verification

The design is considered complete when these checks pass:

1. The new model loads and simulates in MATLAB batch.
2. The plant exposes a third input for `Tload_cmd`.
3. The position and speed loops are rewired to `feedback` outputs instead of direct `plant` outputs.
4. `speed_mech_fb` follows `speed_mech` with observable filtering behavior.
5. A `0.2 N·m` load step can be applied without immediate compile/runtime failure.

## Explicit Non-Goals

This task does not add:

- sensorless back-EMF or PLL observers
- abc current-state motor dynamics
- switch-level PWM or deadtime
- ADC current reconstruction timing

Those belong to later realism upgrades after this feedback/load layer is stable.
