# ADC Low-Side Sampling Timing Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Retarget ADC1 sampling to a controllable TIM1 compare event, guarantee the 20kHz loop consumes a same-cycle ADC frame, and surface sampling diagnostics/faults to the host.

**Architecture:** Keep the existing TIM1 PWM + ADC regular scan + DMA circular pipeline, but move the trigger source from `TIM1 TRGO=UPDATE` to `TIM1 TRGO2=OC4REF`. Add explicit ADC frame sequencing and control-cycle tagging in `adc_sampling`, then require `FOC_App_TIM1_IRQHandler()` to use only a frame that was completed for the current control cycle and fault if repeated misses occur.

**Tech Stack:** STM32H7 HAL/CubeMX-generated C, application-layer C in `MDK-ARM/code`, Python `unittest`, PowerShell build scripts, ARM GCC sanity build.

---

### Task 1: Lock the trigger-routing and ISR-order contract with tests

**Files:**
- Modify: `test_build_system.py`
- Test: `python -m unittest test_build_system.py`

**Step 1: Write the failing test**

Add tests that assert:
- `Core/Src/tim.c` configures `TIM1` with `MasterOutputTrigger2 = TIM_TRGO2_OC4REF`.
- `Core/Src/tim.c` configures `TIM_CHANNEL_4` as an internal compare channel.
- `Core/Src/adc.c` uses `ADC_EXTERNALTRIG_T1_TRGO2`.
- `Core/Src/adc.c` uses `ADC_SAMPLETIME_32CYCLES_5` for current channels and `ADC_SAMPLETIME_16CYCLES_5` for Vbus.
- `Core/Src/dma.c` gives `DMA1_Stream2_IRQn` a higher priority than `TIM1_UP_IRQn`.

**Step 2: Run test to verify it fails**

Run: `python -m unittest test_build_system.py`
Expected: FAIL because the source still uses `TIM_TRGO_UPDATE`, `TIM_TRGO2_RESET`, `ADC_EXTERNALTRIG_T1_TRGO`, `ADC_SAMPLETIME_8CYCLES_5`, and the old NVIC priorities.

**Step 3: Write minimal implementation**

Update the CubeMX-generated source while staying inside user-code-safe sections where practical:
- `Core/Src/tim.c`
- `Core/Src/adc.c`
- `Core/Src/dma.c`

**Step 4: Run test to verify it passes**

Run: `python -m unittest test_build_system.py`
Expected: PASS for the new timing contract.

**Step 5: Commit**

Run:
```bash
git add test_build_system.py Core/Src/tim.c Core/Src/adc.c Core/Src/dma.c
git commit -m "feat: retarget adc trigger to tim1 compare"
```

### Task 2: Add ADC frame sequencing and freshness tracking

**Files:**
- Modify: `MDK-ARM/code/adc_sampling.h`
- Modify: `MDK-ARM/code/adc_sampling.c`
- Modify: `Core/Src/stm32h7xx_it.c`
- Test: `test_build_system.py`

**Step 1: Write the failing test**

Add tests that assert `adc_sampling` exposes:
- frame sequence tracking
- committed control-cycle tag / age tracking
- sample-miss and invalid-window counters
- helper APIs for producer/consumer handoff

Concrete symbols to require in source:
- `frameSequence`
- `lastCommittedCycle`
- `sampleMissCount`
- `invalidWindowCount`
- `ADC_Sampling_BeginControlCycle`
- `ADC_Sampling_TryConsumeLatest`

**Step 2: Run test to verify it fails**

Run: `python -m unittest test_build_system.py`
Expected: FAIL because the bookkeeping fields and helper functions do not exist yet.

**Step 3: Write minimal implementation**

Implement the producer/consumer contract:
- `ADC_Sampling_Process()` copies the DMA frame, computes physical values, increments a frame sequence, and tags the frame with the most recent announced control cycle.
- `ADC_Sampling_BeginControlCycle()` increments/announces the current control cycle from the TIM1 ISR.
- `ADC_Sampling_TryConsumeLatest()` returns success only when a fresh frame is committed for the announced cycle and records misses otherwise.
- Keep DMA work lightweight and avoid moving control logic into the DMA IRQ.

**Step 4: Run test to verify it passes**

Run: `python -m unittest test_build_system.py`
Expected: PASS for the new handoff API and counters.

**Step 5: Commit**

Run:
```bash
git add test_build_system.py MDK-ARM/code/adc_sampling.h MDK-ARM/code/adc_sampling.c Core/Src/stm32h7xx_it.c
git commit -m "feat: track adc frame freshness by control cycle"
```

### Task 3: Gate the FOC loop on fresh ADC data and fault on repeated misses

**Files:**
- Modify: `MDK-ARM/code/foc_app.h`
- Modify: `MDK-ARM/code/foc_app.c`
- Modify: `test_build_system.py`
- Test: `python -m unittest test_build_system.py`
- Verify: `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`

**Step 1: Write the failing test**

Add tests that assert:
- `FOC_FAULT_ADC_SAMPLING` exists.
- `FOC_App_TIM1_IRQHandler()` announces a control cycle before consuming ADC data.
- `FOC_App_TIM1_IRQHandler()` calls `ADC_Sampling_TryConsumeLatest`.
- repeated sample misses escalate through `FOC_App_RequestDisableFromISR`.

**Step 2: Run test to verify it fails**

Run: `python -m unittest test_build_system.py`
Expected: FAIL because the FOC app still reads `ADC_Sampling_GetData()` directly and has no sampling fault code.

**Step 3: Write minimal implementation**

Update `FOC_App_TIM1_IRQHandler()` so it:
- begins the cycle
- attempts to consume a fresh ADC frame
- records frame age and current values only on success
- counts timing failures and faults after repeated misses
- blocks both normal running and identification from silently continuing on stale currents

**Step 4: Run tests to verify they pass**

Run:
- `python -m unittest test_build_system.py`
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`

Expected:
- Python tests PASS
- ARM GCC compile sanity PASS

**Step 5: Commit**

Run:
```bash
git add test_build_system.py MDK-ARM/code/foc_app.h MDK-ARM/code/foc_app.c
git commit -m "feat: fault on stale adc frames in foc loop"
```

### Task 4: Publish sampling diagnostics to the host

**Files:**
- Modify: `MDK-ARM/code/uart_upload.h`
- Modify: `MDK-ARM/code/uart_upload.c`
- Modify: `test_build_system.py`
- Test: `python -m unittest test_build_system.py`
- Verify: `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`

**Step 1: Write the failing test**

Add tests that assert the UART upload packet and formatter include:
- trigger source description
- current and Vbus sample-time metadata
- frame sequence / frame age
- sample-miss and invalid-window counters
- raw ADC A/B/C channels
- computed `Ia`, `Ib`, `Ic`

**Step 2: Run test to verify it fails**

Run: `python -m unittest test_build_system.py`
Expected: FAIL because the upload packet currently exposes only encoder, driver, and limited FOC loop data.

**Step 3: Write minimal implementation**

Extend `DrvUart_DataPacket_t`, `DrvUart_CollectData()`, and `DrvUart_FormatNormal()` / `DrvUart_FormatFault()` to publish the sampling diagnostics in both steady-state and fault cases.

**Step 4: Run tests to verify they pass**

Run:
- `python -m unittest test_build_system.py`
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`

Expected: PASS

**Step 5: Commit**

Run:
```bash
git add test_build_system.py MDK-ARM/code/uart_upload.h MDK-ARM/code/uart_upload.c
git commit -m "feat: upload adc sampling diagnostics"
```

### Task 5: Refresh docs and final verification

**Files:**
- Modify: `Project_Architecture.md`
- Modify: `README.md`
- Modify: `PROGRESS.md`
- Verify: `python -m unittest test_build_system.py`
- Verify: `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`
- Verify: `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1`

**Step 1: Update docs**

Document:
- `TIM1 CH4 -> TRGO2 -> ADC1` sampling path
- same-cycle ADC frame contract
- sampling fault behavior
- host-visible diagnostics

**Step 2: Log progress**

Add a `PROGRESS.md` entry with:
- problem
- fix
- prevention / follow-up validation
- commit placeholder if final commit is not created yet

**Step 3: Run full verification**

Run:
- `python -m unittest test_build_system.py`
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build_test.ps1`
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\\build.ps1`

Expected:
- tests PASS
- compile sanity PASS
- full firmware build PASS

**Step 4: Commit**

Run:
```bash
git add Project_Architecture.md README.md PROGRESS.md
git commit -m "docs: record adc sampling timing redesign"
```
