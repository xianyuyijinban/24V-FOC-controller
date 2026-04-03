# ADC Low-Side Sampling Timing Redesign

**Date:** 2026-04-02  
**Scope:** STM32H743 FOC current sampling path redesign for low-side shunt sensing

## Goal

Redesign the ADC trigger and data handoff path so the three phase-current channels are sampled inside a valid low-side conduction window, and so the 20kHz control loop consumes the current PWM cycle's completed ADC frame instead of stale data from the previous cycle.

## Background

The current project uses three low-side shunt current channels plus one bus-voltage channel:

- Current A: `PA1 / ADC1_INP17`
- Current B: `PA2 / ADC1_INP14`
- Current C: `PA3 / ADC1_INP15`
- Vbus: `PC4 / ADC1_INP4`

Current firmware behavior:

- `ADC1` regular conversion is triggered by `TIM1 TRGO = UPDATE`
- All four regular channels use `ADC_SAMPLETIME_8CYCLES_5`
- `TIM1_UP_IRQHandler()` runs the 20kHz control loop
- `DMA1_Stream2_IRQHandler()` runs later and commits ADC results through `ADC_Sampling_Process()`

That means the control loop can execute before the latest ADC frame is committed, so the loop may consume one-cycle-old current data. For low-side shunt sensing, increasing ADC sample-and-hold time alone does not fix this if the trigger still lands near switching edges or outside a valid conduction window.

## Constraints

- Hardware topology is low-side shunt sensing, not inline phase-current sensing.
- PWM remains `TIM1`, center-aligned, 20kHz.
- The design should stay close to the current HAL/CubeMX structure and avoid a large architecture rewrite.
- The design must support both normal FOC running state and motor-identification state.
- Fault causes and sampling diagnostics must be uploaded to the host in real time.
- If sampling becomes invalid, motor drive must not silently continue on untrusted current feedback.

## Recommended Approach

Use `TIM1 CH4 output compare` as an internal sampling reference, route it to `TIM1 TRGO2`, and retarget `ADC1` regular conversions to `TIM1_TRGO2`.

This keeps the existing `ADC regular scan + DMA circular + TIM1 main ISR` architecture, but replaces the coarse `UPDATE` trigger with a controllable compare event. It also gives the firmware a clean way to place sampling in the valid low-side window and to enforce that the control ISR only consumes a frame that has completed in the current PWM cycle.

## Alternatives Considered

### Option 1: Only increase ADC `SamplingTime`

- Smallest code change
- Lowest schedule risk
- Does not solve wrong trigger phase
- Does not solve one-cycle data handoff skew

Conclusion: rejected. It may reduce noise slightly but cannot guarantee valid low-side sampling.

### Option 2: Use `TIM1 output compare` plus data-handoff cleanup

- Moderate change size
- Fits current architecture
- Fixes trigger phase and stale-frame consumption together
- Supports later diagnostics and sampling validity checks

Conclusion: selected.

### Option 3: Full dynamic sampling-window reconstruction

- Highest theoretical sampling quality
- Much more code and bench-tuning effort
- Too much risk for the current project stage

Conclusion: not selected for this round.

## Architecture

### Current Timing Path

Current path:

1. `TIM1 UPDATE` triggers `ADC1`
2. `ADC1` scans 4 channels
3. DMA writes raw data
4. `DMA1_Stream2_IRQHandler()` runs and calls `ADC_Sampling_Process()`
5. `TIM1_UP_IRQHandler()` runs the control loop and reads `ADC_Sampling_GetData()`

Observed issue:

- The control ISR and the ADC frame-commit point are not explicitly ordered for same-cycle consumption.
- The trigger event is too generic for low-side shunt sensing and is not explicitly tied to the stable low-side window.

### Target Timing Path

Target path:

1. `TIM1` continues to generate 20kHz center-aligned PWM
2. `TIM1 CH4` acts only as an internal output-compare event
3. `TIM1 TRGO2 = OC4REF`
4. `ADC1 regular conversion trigger = TIM1_TRGO2`
5. `ADC1` scans 4 channels and DMA writes the frame
6. `DMA1_Stream2_IRQHandler()` commits raw/physical values and increments a frame sequence counter
7. `TIM1_UP_IRQHandler()` consumes only a frame marked complete for the current cycle

This design keeps one main control ISR but moves the ADC trigger earlier by a controlled offset so that ADC conversion and DMA completion occur before the control ISR reads the frame.

## Sampling Window Strategy

### Why Trigger Phase Matters

For low-side shunt sensing, the measured current is reliable only when the corresponding low-side current path is observable and has settled. Sampling too close to switching edges can corrupt readings due to:

- MOSFET switching transients
- diode reverse recovery
- op-amp recovery and settling
- ground bounce and current-path commutation

Therefore, the trigger must be placed inside a stable conduction window, not simply near the PWM boundary.

### Sampling Point Placement

The sampling event should be placed near the middle of the valid low-side conduction window, with two explicit margins:

- `t_blank`: blanking time after the switching edge
- `t_guard`: end margin before the window closes

Design rule:

`t_sample = t_low_on_start + t_blank + margin_to_window_center`

And it must satisfy:

`t_sample + t_adc_scan < t_low_on_end - t_guard`

### Initial ADC Sampling-Time Recommendation

Initial recommendation:

- Current channels: `ADC_SAMPLETIME_32CYCLES_5`
- Vbus channel: `ADC_SAMPLETIME_16CYCLES_5`

Rationale:

- `8.5 cycles` is likely too short for this low-side shunt + amplifier front-end when sampling is moved away from the switching edge but still expected to be quiet.
- `32.5 cycles` gives more analog settling margin without immediately consuming too much of the low-side window.
- `64.5 cycles` remains a valid fallback if bench data still shows settling problems, but it is not the preferred first default because it reduces timing margin.

## Data Flow

### ADC Producer Path

`ADC1` remains in regular scan mode with DMA circular transfer. The DMA handler remains responsible for lightweight frame commit work:

- copy raw channel values
- convert raw values into physical `Ia`, `Ib`, `Ic`, `Vbus`
- update a monotonically increasing frame sequence
- record a ready flag or cycle tag for the current frame

The DMA handler should not run heavy control logic.

### Control-Loop Consumer Path

The 20kHz control ISR in `TIM1_UP_IRQHandler()` remains the main consumer:

- read encoder position
- verify that a fresh ADC frame is available for the current control cycle
- consume `Ia`, `Ib`, `Ic`, `Vbus`
- run current loop or identification logic
- update PWM compare values

If a fresh frame is not available in time, the cycle must be recorded as a sampling timing failure rather than silently reusing data with no diagnostic.

### Identification Path

Motor identification runs in the same 20kHz timing domain. The redesign must ensure:

- ADC sampling remains synchronized with injected excitation
- identification logic also consumes only validated current frames
- invalid sampling windows can abort or degrade identification cleanly with an explicit reason code

## Interrupt and DMA Ordering

This redesign does not remove the need for correct interrupt prioritization. The intended timing relationship is:

1. `TIM1 CH4 / TRGO2` triggers ADC conversion
2. `ADC1 + DMA` complete the frame
3. `DMA1_Stream2_IRQHandler()` commits frame data
4. `TIM1_UP_IRQHandler()` consumes the committed frame

The firmware should treat this ordering as a design contract. If any higher-latency interrupt path breaks it, the system must emit a diagnostic rather than silently proceeding.

## Fault Handling and Diagnostics

### Sampling Diagnostics

The firmware should add and publish diagnostics such as:

- ADC trigger source in use
- configured ADC sample time for current channels
- configured ADC sample time for Vbus channel
- latest ADC frame sequence
- ADC frame age in control cycles
- ADC sample-miss counter
- invalid-window counter
- current-consistency fault counter
- raw ADC current channels
- computed `Ia`, `Ib`, `Ic`

### Sampling Fault Policy

Recommended policy:

- Single missed or late frame: count and report a timing/diagnostic event
- Repeated late frames or invalid windows: escalate to a fault
- While sampling fault is active, the driver must not continue closed-loop motor drive

This matches the project's broader requirement that faults remain visible to the host and prevent unsafe drive behavior.

## Bench Validation Plan

### Stage 1: Static Timing Validation

Verify the sequence:

`TIM1 CH4 trigger -> ADC DMA complete -> TIM1_UP control ISR`

Recommended method:

- add lightweight GPIO or counter instrumentation for the three timing points
- observe the order with a logic analyzer or oscilloscope

### Stage 2: Analog Quality Validation

Observe:

- `adc_raw_a/b/c`
- `Ia`, `Ib`, `Ic`
- `Ia + Ib + Ic`
- sample-miss counter
- invalid-window counter

Success criteria:

- zero-current offset remains stable
- low-current excitation does not show strong edge-correlated spikes
- three-phase current sum remains near zero on average

### Stage 3: 12V / 2A Bench Bring-Up

Given the immediate bench constraints:

- bus voltage: `12V`
- max phase current: `2A`

Recommended sequence:

1. power stage off, verify diagnostics only
2. enable power stage with no motion command
3. small open-loop excitation
4. low-speed closed-loop run
5. identification run

### Stage 4: Fault Path Validation

Inject or simulate:

- ADC frame late/missing
- invalid encoder
- undervoltage/overvoltage edges
- repeated current-consistency anomalies

Expected result:

- host receives clear real-time fault reason
- driver does not keep driving the motor on invalid current feedback

## Files Affected by the Future Implementation

Expected code touch points:

- `Core/Src/adc.c`
- `Core/Src/tim.c`
- `Core/Src/stm32h7xx_it.c`
- `MDK-ARM/code/adc_sampling.c`
- `MDK-ARM/code/adc_sampling.h`
- `MDK-ARM/code/foc_app.c`
- `MDK-ARM/code/uart_upload.c`
- `MDK-ARM/code/uart_upload.h`
- `Project_Architecture.md`
- `README.md`

## Final Decision

Approved design decisions:

- Use `TIM1 CH4 output compare` as the ADC sampling reference
- Route `TIM1 TRGO2 = OC4REF`
- Retarget `ADC1` regular trigger to `TIM1_TRGO2`
- Increase current-channel ADC sample time to `32.5 cycles`
- Increase Vbus-channel ADC sample time to `16.5 cycles`
- Require the control loop to consume only a validated current-cycle ADC frame
- Upload all sampling failure reasons and counters to the host in real time
- Escalate repeated sampling invalidity into a motor-drive-blocking fault
