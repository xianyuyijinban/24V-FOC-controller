# Host Computer Full GUI Roadmap

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Expand the current local host GUI from a bench-safe debug shell into a full-featured control application that GUI-covers all existing firmware commands and the main debug workflow.

**Architecture:** Keep the existing split between parser/command builder, GUI presentation logic, and serial worker. Grow the UI by filling the already reserved tabs (`Identify`, `Advanced Control`, `PI Parameters`) and add reusable service/state layers before adding richer views such as plotting or parameter persistence. Every phase must keep the firmware build and existing host parser tests green.

**Tech Stack:** Python 3, PyQt6, pyserial, pyqtgraph, unittest

---

## Current Baseline

The repository already has:

- A runnable local GUI entry: `HostComputer/gui_app.py`
- A `Debug Panel` with:
  - port selection
  - connect/disconnect
  - unlock/lock
  - enable/disable
  - clear fault
  - identify start/stop
  - mode selection
  - single target send
  - runtime status cards
  - fault summary
  - serial log
- Placeholder tabs for:
  - `Identify`
  - `Advanced Control`
  - `PI Parameters`
- Host-side tests for:
  - parser
  - GUI logic
  - serial service
  - main window smoke behavior

This roadmap starts from that baseline and turns the remaining firmware-facing functions into full GUI workflows.

## End-State Scope

When this roadmap is complete, the GUI should cover:

- serial connection lifecycle
- power-stage lock/unlock workflow
- enable/disable workflow
- fault clear workflow
- identification start/stop plus progress/status presentation
- control-mode switching
- torque/speed/position targets with dedicated controls
- PI parameter editing for current/speed/position loops
- richer live status display
- live signal plotting for the main runtime channels
- command validation and operator feedback
- optional save/load of GUI-side presets

## Implementation Checklist

### Phase 1: Stabilize the GUI core before adding more controls

**Goal:** Make the current shell easier to extend without rewriting every page later.

- [ ] Extract a shared application-state model that the tabs can read instead of each tab tracking packet fields separately.
- [ ] Normalize all command send paths through one dispatcher so logging, validation, and error handling are consistent.
- [ ] Add a reusable notification/status-bar path for:
  - connection errors
  - invalid numeric input
  - command send failures
  - successful command sends
- [ ] Add unit tests for:
  - input validation
  - command dispatch routing
  - button enable/disable rules
- [ ] Keep `HostComputer/test_data_parser.py`, GUI tests, and `test_build_system.py` green after refactor.

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/gui_logic.py`
- `HostComputer/serial_service.py`
- `HostComputer/serial_worker.py`
- `HostComputer/test_gui_logic.py`
- `HostComputer/test_main_window.py`

### Phase 2: Finish the `Advanced Control` tab

**Goal:** Replace the current single-value target widget with a real control page that exposes all runtime command paths cleanly.

- [ ] Build an `Advanced Control` tab widget.
- [ ] Add dedicated grouped controls for:
  - torque mode with `Id_ref` / `Iq_ref`
  - speed mode with `speed`
  - position mode with `position`
- [ ] Add mode-aware validation:
  - reject malformed floats
  - reject empty sends
  - clearly mark the active mode
- [ ] Add "apply" buttons per control group instead of forcing all modes through one field.
- [ ] Keep the `Debug Panel` fast and simple by moving advanced entry forms out of it.
- [ ] Add tests that assert:
  - correct `CommandBuilder` method is selected
  - current-reference page sends `CMD:IREF`
  - speed page sends `CMD:SREF`
  - position page sends `CMD:PREF`

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/gui_logic.py`
- `HostComputer/test_main_window.py`
- `HostComputer/test_gui_logic.py`

### Phase 3: Finish the `PI Parameters` tab

**Goal:** GUI-cover all three PI update commands that already exist in firmware.

- [ ] Build a `PI Parameters` tab widget.
- [ ] Add three sections:
  - current-loop PI
  - speed-loop PI
  - position-loop PI
- [ ] Each section should expose:
  - `Kp`
  - `Ki`
  - send/apply button
- [ ] Add numeric-range validation and formatted display.
- [ ] Add a "load defaults from local preset" hook so later preset storage can be layered in without redesigning the page.
- [ ] Add tests for:
  - current PI -> `CMD:PI_CURRENT`
  - speed PI -> `CMD:PI_SPEED`
  - position PI -> `CMD:PI_POS`

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/data_parser.py`
- `HostComputer/gui_logic.py`
- `HostComputer/test_main_window.py`
- `HostComputer/test_gui_logic.py`
- `HostComputer/test_data_parser.py`

### Phase 4: Finish the `Identify` tab

**Goal:** Turn identify from two raw buttons into an operator-friendly workflow page.

- [ ] Build an `Identify` tab widget.
- [ ] Add an identify state panel showing:
  - current connection state
  - power lock state reminder
  - current FOC state
  - fault-active summary
- [ ] Add primary actions:
  - start identify
  - stop identify
  - clear fault
- [ ] Add a progress/log section specialized for identification events.
- [ ] If firmware currently lacks detailed identify progress packets, add GUI placeholders that consume what exists now and leave a clean extension point for later finer-grained progress.
- [ ] Add tests for:
  - identify page command wiring
  - state text updates from incoming packets
  - identify action disable rules when disconnected

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/gui_logic.py`
- `HostComputer/test_main_window.py`
- `README.md`

### Phase 5: Add live plotting

**Goal:** Make the GUI useful for tuning, not just command entry.

- [ ] Add a plotting area using `pyqtgraph`.
- [ ] Start with these channels:
  - angle
  - speed
  - `Id`
  - `Iq`
  - `Id_ref`
  - `Iq_ref`
  - `Vd`
  - `Vq`
- [ ] Add channel toggles so the plot does not become unreadable.
- [ ] Add a rolling buffer with a bounded history.
- [ ] Keep plotting optional or collapsible so low-power systems still run the GUI comfortably.
- [ ] Add tests around plot-buffer logic in a pure-Python helper instead of trying to snapshot plot widgets.

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/gui_logic.py`
- `HostComputer/test_gui_logic.py`
- `HostComputer/requirements.txt`

### Phase 6: Improve operator feedback and session usability

**Goal:** Remove friction during repeated bench sessions.

- [ ] Remember last-used:
  - serial port
  - baud rate
  - selected control mode
- [ ] Add one-click actions or shortcuts for common debug flows:
  - unlock + enable
  - disable + lock
  - clear fault + re-arm prompt
- [ ] Add connection heartbeat or stale-data indication when packets stop updating.
- [ ] Add structured log filters:
  - INFO
  - TX
  - RX
  - ERROR
- [ ] Add "copy recent log" and "clear log" actions.
- [ ] Add tests for settings persistence and stale-data indicator logic.

**Files likely involved:**

- `HostComputer/main_window.py`
- `HostComputer/gui_logic.py`
- `HostComputer/test_gui_logic.py`
- `HostComputer/test_main_window.py`

### Phase 7: Add preset/export support

**Goal:** Make repeated tuning sessions reproducible.

- [ ] Add local preset save/load for:
  - PI parameters
  - common target values
  - preferred GUI settings
- [ ] Use a simple local JSON file in `HostComputer/` or a user config path.
- [ ] Add CSV export for plotted/received runtime samples if the rolling buffer is already present.
- [ ] Make export optional and bounded so it does not interfere with live tuning.
- [ ] Add tests for preset serialization and CSV formatting helpers.

**Files likely involved:**

- `HostComputer/gui_logic.py`
- `HostComputer/main_window.py`
- `HostComputer/test_gui_logic.py`
- `README.md`

### Phase 8: Finish documentation and polish

**Goal:** Make the GUI discoverable and maintainable by the next person.

- [ ] Update `README.md` with:
  - install steps
  - launch command
  - current feature map by tab
  - any known limitations
- [ ] Update `Project_Architecture.md` host-side section so the GUI files and flow are reflected accurately.
- [ ] Add screenshots only if they can be regenerated easily.
- [ ] Add a final source-contract test that checks the expected GUI entry files and doc references still exist.

**Files likely involved:**

- `README.md`
- `Project_Architecture.md`
- `test_build_system.py`

## Recommended Execution Order

If the goal is “尽快补齐全部 GUI 功能，但不把明天可用性搞坏”, the order should be:

1. Phase 1: Core stabilization
2. Phase 2: Advanced Control tab
3. Phase 3: PI Parameters tab
4. Phase 4: Identify tab
5. Phase 5: Live plotting
6. Phase 6: Session usability
7. Phase 7: Presets/export
8. Phase 8: Final docs/polish

This order matches the current firmware command surface and gives the fastest path to "all existing commands have real GUI entry points" before adding nicer extras.

## Definition of Done

This GUI roadmap is done when:

- every currently supported firmware command has a dedicated GUI control path
- all placeholder tabs are replaced with functional pages
- the GUI can run a full debug session without dropping to raw serial tools
- host-side tests cover parser, logic, serial service, and major window workflows
- `python -m unittest -v HostComputer/test_data_parser.py HostComputer/test_gui_logic.py HostComputer/test_serial_service.py HostComputer/test_main_window.py` passes
- `python -m unittest -v test_build_system.py` passes
- `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1` still passes
