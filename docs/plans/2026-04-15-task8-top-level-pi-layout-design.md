# Task 8 Top-Level PI Layout Design

**Goal:** Make the Task 8 official-PMSM Simulink model easier to tune and read by exposing the three-loop PI gains at the top level and cleaning up the top-level block layout.

**Current Baseline:** `D:/matlab/minimal_foc_controller_task8_official_pmsm.slx` now runs with the official MathWorks `Surface Mount PMSM`, but the top level is visually crowded and the PI gains are still effectively buried inside the loop implementations or workspace variables.

## User Need

The immediate need is not more plant realism. The user needs a model that is practical to tune under time pressure:

1. `Kp_pos`, `Ki_pos`, `Kp_speed`, `Ki_speed`, `Kp_iq`, and `Ki_iq` must be visible and editable from the top level.
2. The main control flow should be readable without tracing wires through a cluttered canvas.
3. The official PMSM path introduced in Task 8 must keep working after the cleanup.

## Chosen Approach

Use explicit top-level parameter blocks and route them into the three loop subsystems as signal inputs.

This means:

1. Add six top-level constant parameter blocks named after the active PI gains.
2. Expand the loop subsystem interfaces so the gains are no longer implicit.
3. Rebuild only the loop subsystems and top-level layout needed for readability.
4. Preserve the current official PMSM plant, feedback shell, and average-value bridge.

This is the most direct way to give the user a model that can be tuned interactively without hunting through subsystem internals.

## Alternatives Considered

### Option 1: Top-level parameter blocks wired into loop subsystems

This is the selected approach.

- Directly supports tuning from the top level
- Keeps the active gain values visible on the main canvas
- Makes future gain scheduling or test overrides easier

### Option 2: Keep loop internals unchanged and only add a visual parameter panel

- Less intrusive
- Does not actually make tuning easier unless the user still edits workspace variables
- Too easy for the top-level display to drift away from the real active gains

### Option 3: Use masks or model workspace only

- Cleaner looking canvas
- Worse for quick teaching/debug use
- Hides the active signal path and makes ad-hoc tuning slower

## Architecture Change

### Top-Level Parameter Zone

Add six visible top-level parameter blocks:

- `Kp_pos`
- `Ki_pos`
- `Kp_speed`
- `Ki_speed`
- `Kp_iq`
- `Ki_iq`

These blocks should default to the existing workspace variables of the same name so the current initialization script remains the source of the initial values.

### Loop Interface Expansion

Update the loop subsystem interfaces to:

- `position loop`: `pos_ref`, `theta_mech_fb`, `Kp_pos`, `Ki_pos` -> `speed_ref`
- `speed loop`: `speed_ref`, `speed_mech_fb`, `Kp_speed`, `Ki_speed` -> `iq_ref`
- `current loop`: `id_ref`, `iq_ref`, `id_meas`, `iq_meas`, `Kp_iq`, `Ki_iq` -> `vd_cmd`, `vq_cmd`

For the current loop, the same gain pair remains the active tuning pair for both `d` and `q` paths in this teaching-stage model.

### Top-Level Layout Cleanup

Re-layout the main canvas around a readable left-to-right flow:

1. left: `feedback`, command source, `PI Params`
2. center: `position loop`, `speed loop`, `current loop`
3. right: `FOC math`, `avg inverter bridge`, `plant`
4. far right / bottom: scopes, logs, debug observation blocks

The goal is readability, not aesthetic perfection.

## Verification Criteria

The change is successful when:

1. The Task 8 upgrade script recreates the model without compile errors.
2. The top level contains all six PI parameter blocks.
3. The three loop subsystems expose the new gain input ports.
4. `check_task8_green.m` still passes simulation.
5. The top-level canvas is visibly easier to read than the current dense layout.

## Non-Goals

This change does not:

- retune the controller gains
- redesign the observer
- change the official PMSM plant
- add switch-level inverter detail
- add dashboard knobs or App Designer controls
