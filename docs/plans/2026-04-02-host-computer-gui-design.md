# Host Computer GUI Design

**Date:** 2026-04-02  
**Scope:** First local host GUI for tomorrow's bench debugging

## Goal

Build a local Python GUI inside this repository so the demo board can be connected and debugged tomorrow without depending on a separate host repository. The first version must prioritize reliable serial connection, clear runtime status visibility, fault visibility, and a small set of high-frequency debug commands. It must also leave clean extension points for later identification, advanced control, and PI tuning pages.

## Constraints

- Runtime form: Python script, not packaged executable.
- GUI stack: PyQt6, matching the existing `HostComputer/requirements.txt`.
- Data source: existing `HostComputer/data_parser.py` and `CommandBuilder`.
- Priority: stable debug workflow over feature completeness.
- Future growth: reserve interfaces for identification, advanced control, and PI parameter pages.

## Recommended Approach

Use `PyQt6 + QThread + serial worker + existing parser`.

This approach fits the current dependency stack, keeps serial I/O off the UI thread, and gives the project a maintainable structure instead of a fast but fragile single-file tool. A polling-only or single-file design would be slightly faster to start but would create avoidable risk right before hardware debug.

## Architecture

The first version will be split into four modules:

- `HostComputer/gui_app.py`
  - Application entry point.
  - Creates the `QApplication` and launches the main window.
- `HostComputer/main_window.py`
  - Owns the top-level window and tab layout.
  - Hosts the first working "Debug Panel" tab plus placeholder tabs for later expansion.
- `HostComputer/serial_worker.py`
  - Runs in a dedicated `QThread`.
  - Handles serial port discovery, open/close, background read loop, command transmit, and parser hookup.
- `HostComputer/data_parser.py`
  - Remains the packet-parsing and command-building layer.
  - GUI code consumes parsed packets instead of re-implementing protocol parsing.

The main window talks to the serial worker through Qt signals/slots only. It must not access the serial port directly.

## Window Structure

The window will use a tab skeleton so later features can be added without restructuring the application:

- `Debug Panel`
- `Identify` (placeholder)
- `Advanced Control` (placeholder)
- `PI Parameters` (placeholder)

Only `Debug Panel` is functional in the first version. The placeholder tabs will include a short "reserved for future implementation" message so the extension points are visible and intentional.

## Debug Panel Layout

### Top toolbar

The top area will provide connection management:

- Serial port combo box
- Baud rate combo box, default `115200`
- `Refresh Ports`
- `Connect`
- `Disconnect`
- Visual connection status indicator

### Left column: primary actions

This column will prioritize the commands most likely to be used during bench debug:

- Power-stage group
  - `UNLOCK`
  - `LOCK`
  - `ENABLE`
  - `DISABLE`
  - `CLEAR FAULT`
- Identify group
  - `START IDENTIFY`
  - `STOP IDENTIFY`
- Mode group
  - `Torque`
  - `Speed`
  - `Position`
- Target group
  - One editable target field
  - One send button
  - Label text changes with selected mode:
    - Torque mode: `Iq_ref`
    - Speed mode: `speed`
    - Position mode: `position`

The first version intentionally avoids multiple simultaneous target-entry blocks. A single mode-aware target entry reduces UI clutter and lowers operator confusion during early debug.

### Center column: runtime status cards

The center area will display the values needed for immediate diagnosis:

- `FOC State`
- `Fault Active`
- `Angle`
- `Speed`
- `Id / Iq`
- `Id_ref / Iq_ref`
- `Vd / Vq`
- `FAULT1 / VGS2`

Design requirements:

- Text must remain readable from a bench distance.
- Fault state must use obvious warning coloring.
- `READY`, `RUNNING`, and `IDENTIFY` should be visually distinguishable.
- Non-zero fault registers should be highlighted and rendered in hexadecimal.

### Right column: fault summary and serial log

The right side will contain:

- Fault summary card
  - Active/inactive state
  - `FAULT1`
  - `VGS2`
  - Last packet timestamp
- Serial log panel
  - Connection events
  - Command transmit lines
  - Packet summary lines
  - Error lines

The log will keep only a bounded recent history to prevent long sessions from degrading responsiveness.

## Data Flow

### Receive path

1. `serial_worker.py` reads raw bytes from the serial port.
2. Bytes are fed into `FOCDataParser.feed_data(...)`.
3. Parsed `FOCDataPacket` objects are emitted from the worker thread.
4. `main_window.py` updates cards, fault summary, and log output.

### Transmit path

1. User clicks a control.
2. GUI maps the action to an existing `CommandBuilder` method.
3. The resulting command string is emitted to the worker thread.
4. The worker writes the command to the serial port.
5. The GUI appends a `TX:` log line.

The GUI must not construct protocol strings ad hoc in widget callbacks.

## Threading Model

All serial I/O will run in a dedicated `QThread`.

Reasons:

- Prevent GUI freezes if serial reads block.
- Keep parser exceptions away from the UI event loop.
- Make future waveform and advanced-panel expansion easier.

The worker thread is responsible for:

- Listing ports
- Opening ports
- Closing ports
- Background reads
- Sending commands
- Reporting connection errors
- Publishing parsed packets

## Connection and Default Behavior

- The GUI does not auto-connect on launch.
- Connection is always a user action.
- Controls that require an active connection stay disabled until connected.
- Before the first packet arrives, numeric cards show placeholder values such as `--`.
- Default fault register display is `0x0000`.
- Firmware packet timestamps are used directly for display.

## Error Handling

The first version must explicitly handle these cases:

- Serial open failure
  - Show a user-facing error
  - Write an error log line
- Serial disconnect during operation
  - Mark the UI as disconnected
  - Disable active command controls
  - Log the event
- Command send failure
  - Log the failure
- Packet parse exception
  - Do not crash the GUI
  - Log the issue and continue receiving
- Fault packet arrival
  - Highlight the fault summary immediately

## Logging Policy

The serial log will keep a bounded history, targeted at short bench sessions:

- Retain only the most recent 300 log lines
- Distinguish log classes:
  - `INFO`
  - `TX`
  - `RX`
  - `ERROR`

## Testing Strategy

The first version will focus on testing logic around the GUI instead of brittle pixel-level automation.

### Keep existing tests

- `HostComputer/test_data_parser.py`

### Add new GUI-focused tests

- Mode selection updates the target label correctly
- Connection state updates button enable/disable rules correctly
- Packet-to-display formatting returns the expected state text and values
- Fault summary formatting highlights non-zero `FAULT1` and `VGS2`

The test target is the control logic and presentation mapping, not full end-to-end GUI automation.

## Manual Acceptance Criteria

Before tomorrow's bench session, the following must work:

- Enumerate serial ports
- Connect and disconnect successfully
- See runtime values update from live packets
- See fault summary react immediately to fault packets
- Send `UNLOCK`, `ENABLE`, `DISABLE`, `CLEAR_FAULT`, and identify commands
- Switch modes and send the mode-appropriate target value
- Survive unplug/replug or disconnect without freezing the GUI

## Documentation Changes

The implementation must update repository docs so the local GUI is discoverable:

- `README.md`
  - Add local GUI startup instructions
  - Update the `HostComputer/` directory description
  - Stop implying that the separate host repository is the only GUI path
- Optional architecture note
  - Refresh host-side sections in `Project_Architecture.md` if the local GUI becomes the primary recommended path for bench debug

## Deliverables

The first GUI milestone is complete when the repository contains:

- A runnable local GUI entry script
- A working `Debug Panel`
- Reserved placeholder tabs for `Identify`, `Advanced Control`, and `PI Parameters`
- Updated README instructions
- Automated tests for the new GUI control logic

## Non-Goals for First Version

The following are explicitly deferred:

- Real-time waveform plotting
- Full identification workflow UI
- PI parameter editing UI
- Executable packaging
- Persistent application settings
