# Position Loop PD Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the top-level firmware position loop from the old PI-shaped interface with an explicit PD design, and keep the host GUI/command/docs aligned with the new semantics.

**Architecture:** Keep the existing cascaded control stack `position -> speed -> current`, but make the position loop generate `speed_ref` with `Kp * pos_error - Kd * speed_mech` instead of reusing the PI controller abstraction. Update the UART command path and host GUI so the operator sees `PD` for position and `PI` only for current/speed.

**Tech Stack:** STM32H7 firmware in `MDK-ARM/code` and `Core/Src`, Python host GUI in `HostComputer`, repo source-contract tests in `test_build_system.py`

---

### Task 1: Lock the PD interface with failing tests

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\test_build_system.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\test_data_parser.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\test_gui_logic.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\test_main_window.py`

**Step 1:** Assert firmware uses a dedicated position-PD config and `CMD:PD_POS`.

**Step 2:** Assert host command builders and GUI labels use `PD` for the position loop.

### Task 2: Implement the firmware PD loop

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\foc_app.h`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\MDK-ARM\code\foc_app.c`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\Core\Src\stm32h7xx_it.c`

**Step 1:** Add a small explicit `PositionPD` config/state shape.

**Step 2:** Replace the old position PI update with `Kp * pos_error - Kd * speed_mech`.

**Step 3:** Replace the UART command parser from `CMD:PI_POS` to `CMD:PD_POS`.

### Task 3: Align host GUI and documentation

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\data_parser.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\gui_logic.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\HostComputer\main_window.py`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\README.md`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\Project_Architecture.md`

**Step 1:** Update host command builders and local preset schema to position PD.

**Step 2:** Rename position UI labels from PI to PD and the tab from `PI Parameters` to `Loop Parameters`.

**Step 3:** Update operator docs and architecture notes so they match the shipped behavior.

### Task 4: Verify and log

**Files:**
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\PROGRESS.md`
- Modify: `C:\Users\xiangyu\24V_FOC_Controller_audit_20260222\PROCESS.md`

**Step 1:** Run the focused Python/unit/source-contract verification commands.

**Step 2:** Append the mandatory change log entry with the real commit SHA.
