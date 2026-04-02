# Host Computer GUI Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a runnable local PyQt6 host GUI that can connect to the board tomorrow, show core runtime/fault status, send the common debug commands, and keep placeholder tabs for later advanced features.

**Architecture:** Keep protocol parsing in the existing `HostComputer/data_parser.py`, add a tested pure-Python presentation layer for labels/status formatting, and isolate serial I/O inside a worker-backed service so the GUI thread never blocks on serial reads. The first version only implements the `Debug Panel`; `Identify`, `Advanced Control`, and `PI Parameters` are explicit placeholder tabs so later work can extend the shell without re-laying out the app.

**Tech Stack:** Python 3, PyQt6, pyserial, unittest

---

### Task 1: Create the GUI logic contract

**Files:**
- Create: `HostComputer/gui_logic.py`
- Create: `HostComputer/test_gui_logic.py`
- Reference: `HostComputer/data_parser.py`

**Step 1: Write the failing test**

Create `HostComputer/test_gui_logic.py` with contract tests for the non-Qt logic:

```python
import unittest
from data_parser import FOCDataPacket
from gui_logic import (
    connection_command_state,
    fault_summary_text,
    mode_target_label,
    packet_snapshot,
)


class TestGuiLogic(unittest.TestCase):
    def test_mode_target_label_matches_control_mode(self):
        self.assertEqual(mode_target_label(0), "Iq_ref (A)")
        self.assertEqual(mode_target_label(1), "Speed (rad/s)")
        self.assertEqual(mode_target_label(2), "Position (rad)")

    def test_connection_command_state_requires_link(self):
        state = connection_command_state(is_connected=False)
        self.assertFalse(state["can_unlock"])
        self.assertFalse(state["can_send_target"])

    def test_packet_snapshot_formats_runtime_values(self):
        packet = FOCDataPacket(
            timestamp=123,
            angle=12.5,
            speed=2.5,
            Id=0.1,
            Iq=0.2,
            Id_ref=0.0,
            Iq_ref=0.3,
            Vd=0.4,
            Vq=0.5,
            foc_state=4,
        )
        snapshot = packet_snapshot(packet)
        self.assertEqual(snapshot["timestamp"], "123 ms")
        self.assertEqual(snapshot["angle"], "12.50 deg")
        self.assertEqual(snapshot["speed"], "2.50 rad/s")
        self.assertIn("0.10", snapshot["currents"])

    def test_fault_summary_text_highlights_registers(self):
        packet = FOCDataPacket(
            timestamp=456,
            is_fault_active=True,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
        )
        summary = fault_summary_text(packet)
        self.assertIn("ACTIVE", summary["state"])
        self.assertIn("0x0640", summary["fault1"])
        self.assertIn("0x00C0", summary["vgs2"])
```

**Step 2: Run test to verify it fails**

Run:

```bash
python -m unittest -v HostComputer/test_gui_logic.py
```

Expected: FAIL with `ModuleNotFoundError` or missing symbol errors because `gui_logic.py` does not exist yet.

**Step 3: Write minimal implementation**

Create `HostComputer/gui_logic.py` with a small pure-Python API:

```python
from dataclasses import dataclass
from data_parser import FOCDataPacket


MODE_LABELS = {
    0: "Iq_ref (A)",
    1: "Speed (rad/s)",
    2: "Position (rad)",
}


def mode_target_label(mode: int) -> str:
    return MODE_LABELS.get(mode, "Target")


def connection_command_state(is_connected: bool) -> dict[str, bool]:
    enabled = bool(is_connected)
    return {
        "can_unlock": enabled,
        "can_lock": enabled,
        "can_enable": enabled,
        "can_disable": enabled,
        "can_clear_fault": enabled,
        "can_identify": enabled,
        "can_send_target": enabled,
    }


def packet_snapshot(packet: FOCDataPacket) -> dict[str, str]:
    return {
        "timestamp": f"{packet.timestamp} ms",
        "angle": f"{packet.angle:.2f} deg",
        "speed": f"{packet.speed:.2f} rad/s",
        "currents": f"Id {packet.Id:.2f} A / Iq {packet.Iq:.2f} A",
        "refs": f"Id_ref {packet.Id_ref:.2f} / Iq_ref {packet.Iq_ref:.2f}",
        "voltages": f"Vd {packet.Vd:.2f} V / Vq {packet.Vq:.2f} V",
        "state": str(packet.foc_state),
    }


def fault_summary_text(packet: FOCDataPacket) -> dict[str, str]:
    return {
        "state": "ACTIVE" if packet.is_fault_active else "NORMAL",
        "fault1": f"FAULT1 0x{packet.fault_status1:04X}",
        "vgs2": f"VGS2 0x{packet.vgs_status2:04X}",
        "timestamp": f"{packet.timestamp} ms",
    }
```

**Step 4: Run test to verify it passes**

Run:

```bash
python -m unittest -v HostComputer/test_gui_logic.py
```

Expected: PASS

**Step 5: Commit**

```bash
git add HostComputer/gui_logic.py HostComputer/test_gui_logic.py
git commit -m "test: add host gui logic contracts"
```

### Task 2: Build a serial service core that is testable without Qt

**Files:**
- Create: `HostComputer/serial_service.py`
- Create: `HostComputer/test_serial_service.py`
- Reference: `HostComputer/data_parser.py`

**Step 1: Write the failing test**

Create `HostComputer/test_serial_service.py`:

```python
import unittest
from serial_service import SerialService


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.is_open = True

    def write(self, payload: bytes):
        self.writes.append(payload)


class FakeParser:
    def __init__(self):
        self.payloads = []

    def feed_data(self, data: bytes):
        self.payloads.append(data)


class TestSerialService(unittest.TestCase):
    def test_send_command_encodes_text(self):
        fake_serial = FakeSerial()
        service = SerialService(serial_port=fake_serial, parser=FakeParser())
        service.send_command("CMD:ENABLE,1\n")
        self.assertEqual(fake_serial.writes, [b"CMD:ENABLE,1\n"])

    def test_handle_bytes_delegates_to_parser(self):
        parser = FakeParser()
        service = SerialService(serial_port=FakeSerial(), parser=parser)
        service.handle_bytes(b"abc")
        self.assertEqual(parser.payloads, [b"abc"])
```

**Step 2: Run test to verify it fails**

Run:

```bash
python -m unittest -v HostComputer/test_serial_service.py
```

Expected: FAIL with `ModuleNotFoundError` because `serial_service.py` does not exist yet.

**Step 3: Write minimal implementation**

Create `HostComputer/serial_service.py`:

```python
class SerialService:
    def __init__(self, serial_port, parser):
        self.serial_port = serial_port
        self.parser = parser

    def send_command(self, command: str):
        self.serial_port.write(command.encode("utf-8"))

    def handle_bytes(self, payload: bytes):
        self.parser.feed_data(payload)
```

Do not add threading or Qt here. Keep it as the testable core the worker can call later.

**Step 4: Run test to verify it passes**

Run:

```bash
python -m unittest -v HostComputer/test_serial_service.py
```

Expected: PASS

**Step 5: Commit**

```bash
git add HostComputer/serial_service.py HostComputer/test_serial_service.py
git commit -m "test: add host serial service contracts"
```

### Task 3: Create the main window shell and debug panel

**Files:**
- Create: `HostComputer/main_window.py`
- Create: `HostComputer/test_main_window.py`
- Modify: `HostComputer/gui_logic.py`

**Step 1: Write the failing test**

Create `HostComputer/test_main_window.py` with an offscreen Qt smoke test:

```python
import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from PyQt6.QtWidgets import QApplication
from main_window import HostMainWindow


class TestHostMainWindow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_window_has_required_tabs(self):
        window = HostMainWindow()
        self.assertEqual(window.tabs.tabText(0), "Debug Panel")
        self.assertEqual(window.tabs.tabText(1), "Identify")
        self.assertEqual(window.tabs.tabText(2), "Advanced Control")
        self.assertEqual(window.tabs.tabText(3), "PI Parameters")

    def test_mode_change_updates_target_label(self):
        window = HostMainWindow()
        window.speed_mode_button.setChecked(True)
        window.apply_mode_selection(1)
        self.assertEqual(window.target_label.text(), "Speed (rad/s)")
```

**Step 2: Run test to verify it fails**

Run:

```bash
python -m unittest -v HostComputer/test_main_window.py
```

Expected: FAIL because `main_window.py` and `HostMainWindow` do not exist yet.

**Step 3: Write minimal implementation**

Create `HostComputer/main_window.py` with:

- A `QMainWindow` subclass named `HostMainWindow`
- A public `tabs` widget
- A `Debug Panel` tab
- Placeholder tabs for `Identify`, `Advanced Control`, and `PI Parameters`
- Public widgets needed by the test:
  - `target_label`
  - `speed_mode_button`
- A method `apply_mode_selection(mode: int)` that updates the target label via `mode_target_label(mode)`

Keep the first implementation simple; do not wire serial yet. Focus on stable layout primitives.

**Step 4: Run test to verify it passes**

Run:

```bash
python -m unittest -v HostComputer/test_main_window.py
```

Expected: PASS

**Step 5: Commit**

```bash
git add HostComputer/main_window.py HostComputer/test_main_window.py HostComputer/gui_logic.py
git commit -m "feat: add host gui window shell"
```

### Task 4: Wire serial worker, command actions, and live status updates

**Files:**
- Create: `HostComputer/serial_worker.py`
- Create: `HostComputer/gui_app.py`
- Create: `HostComputer/__init__.py`
- Modify: `HostComputer/main_window.py`
- Modify: `HostComputer/data_parser.py`

**Step 1: Write the failing test**

Add a new test case to `HostComputer/test_main_window.py`:

```python
from data_parser import FOCDataPacket

    def test_packet_update_refreshes_fault_and_status_text(self):
        window = HostMainWindow()
        packet = FOCDataPacket(
            timestamp=1000,
            angle=45.0,
            speed=3.0,
            Id=0.1,
            Iq=0.2,
            Vd=1.0,
            Vq=2.0,
            Id_ref=0.0,
            Iq_ref=0.5,
            foc_state=4,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            is_fault_active=True,
        )
        window.apply_packet(packet)
        self.assertIn("45.00 deg", window.angle_value.text())
        self.assertIn("ACTIVE", window.fault_state_value.text())
        self.assertIn("0x0640", window.fault_registers_value.text())
```

**Step 2: Run test to verify it fails**

Run:

```bash
python -m unittest -v HostComputer/test_main_window.py
```

Expected: FAIL because the live status widgets and `apply_packet` method are not implemented yet.

**Step 3: Write minimal implementation**

Implement:

- `serial_worker.py`
  - A `QObject` worker with Qt signals for:
    - connection state
    - log lines
    - parsed packets
  - Internally use `SerialService`, `serial.Serial`, and the existing `FOCDataParser`
- `gui_app.py`
  - Entry point runnable as `python -m HostComputer.gui_app`
- `main_window.py`
  - Toolbar with ports, baud rate, connect/disconnect
  - Left-side action groups for unlock/enable/fault clear/identify/mode/target send
  - Center status cards
  - Right fault summary and bounded log panel
  - `apply_packet(packet)` to update all runtime displays
  - action handlers that use `CommandBuilder`
- `data_parser.py`
  - Add `unlock_power` and `lock_power` helpers if they do not already exist:

```python
    @staticmethod
    def unlock_power(unlock: bool) -> str:
        return f"CMD:UNLOCK,{1 if unlock else 0}\n"
```

Keep the GUI launch path minimal and bench-oriented. Do not add waveform plotting yet.

**Step 4: Run tests to verify they pass**

Run:

```bash
python -m unittest -v HostComputer/test_data_parser.py HostComputer/test_gui_logic.py HostComputer/test_serial_service.py HostComputer/test_main_window.py
```

Expected: PASS

**Step 5: Commit**

```bash
git add HostComputer/__init__.py HostComputer/data_parser.py HostComputer/gui_app.py HostComputer/main_window.py HostComputer/serial_worker.py HostComputer/test_main_window.py
git commit -m "feat: add host debug gui shell"
```

### Task 5: Document startup path and verify a local smoke launch

**Files:**
- Modify: `README.md`
- Modify: `Project_Architecture.md`

**Step 1: Write the failing test**

Extend `test_build_system.py` with a new source-contract test:

```python
    def test_host_gui_docs_and_entry_exist(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        architecture = (ROOT / "Project_Architecture.md").read_text(encoding="utf-8")
        gui_entry = ROOT / "HostComputer" / "gui_app.py"

        self.assertTrue(gui_entry.exists())
        self.assertIn("python -m HostComputer.gui_app", readme)
        self.assertIn("HostComputer/main_window.py", readme)
        self.assertIn("HostMainWindow", architecture)
```

**Step 2: Run test to verify it fails**

Run:

```bash
python -m unittest -v test_build_system.TestBuildSystemConsistency.test_host_gui_docs_and_entry_exist
```

Expected: FAIL because the docs do not mention the new local GUI path yet.

**Step 3: Write minimal implementation**

Update:

- `README.md`
  - Add local GUI install/run instructions
  - Update the `HostComputer/` section to list the GUI files
  - Keep the external host repository reference only as optional/future context
- `Project_Architecture.md`
  - Update the host-side section so the local GUI is reflected in the software architecture

**Step 4: Run verification**

Run:

```bash
python -m unittest -v test_build_system.TestBuildSystemConsistency.test_host_gui_docs_and_entry_exist
python -m unittest -v HostComputer/test_data_parser.py HostComputer/test_gui_logic.py HostComputer/test_serial_service.py HostComputer/test_main_window.py
```

Then run a smoke launch without keeping the window open:

```bash
powershell -NoProfile -Command "$env:QT_QPA_PLATFORM='offscreen'; @'
from PyQt6.QtWidgets import QApplication
from HostComputer.main_window import HostMainWindow
app = QApplication([])
window = HostMainWindow()
print(window.windowTitle())
window.close()
'@ | python -"
```

Expected:

- All tests PASS
- Smoke launch prints the window title and exits cleanly

**Step 5: Commit**

```bash
git add README.md Project_Architecture.md test_build_system.py
git commit -m "docs: add host gui startup path"
```

### Task 6: Final verification and progress log

**Files:**
- Modify: `PROGRESS.md`

**Step 1: Run full verification**

Run:

```bash
python -m unittest -v test_build_system.py
python -m unittest -v HostComputer/test_data_parser.py HostComputer/test_gui_logic.py HostComputer/test_serial_service.py HostComputer/test_main_window.py
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
```

Expected:

- All Python tests PASS
- Firmware build still succeeds

**Step 2: Update progress log with the real implementation commit SHA**

Append a `PROGRESS.md` entry that records:

- Problem: no local host GUI for bench debug
- Resolution: added local PyQt6 GUI shell, serial worker, debug panel, docs
- Prevention: keep GUI logic/serial/main-window tests and launch smoke check
- Commit: full SHA of the implementation commit

**Step 3: Commit**

```bash
git add PROGRESS.md
git commit -m "docs: log host gui milestone"
```
