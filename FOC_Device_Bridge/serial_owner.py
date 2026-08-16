"""
FOC Device Bridge — Serial Port Owner

Runs on the bridge Qt event loop. Owns the COM port exclusively.
Feeds incoming bytes through BinaryCurrentParser and FOCDataParser.
Routes W frames to main thread via signals; routes all raw bytes via serial_rx for IPC broadcast.
Parses ACK lines from text stream and emits ack_received.
"""

import re
import time

import serial
from PySide6.QtCore import QObject, QTimer, Signal, Slot

try:
    from HostComputer.data_parser import BinaryCurrentParser, FOCDataParser, WheelEvent
except ImportError:
    import sys
    import os
    _parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _parent not in sys.path:
        sys.path.insert(0, _parent)
    from HostComputer.data_parser import BinaryCurrentParser, FOCDataParser, WheelEvent

from .config_manager import BridgeConfig

# ── ACK line patterns ──────────────────────────────────────────────
# Matches any ACK line the firmware emits. Broader than AckParser because
# we need to catch ALL forms (including the simplest "OK\n" from ENABLE,0).
_ACK_PATTERN = re.compile(
    r'^('
    r'OK|'
    r'UNLOCK,(?:OK|FAIL).*|'
    r'ENABLE,(?:OK|FAIL).*|'
    r'IDENTIFY,(?:OK|FAIL).*|'
    r'STALL_MODE,(?:OK|FAIL).*|'
    r'APP_MODE,(?:OK|FAIL).*|'
    r'CUR_STREAM,(?:OK|FAIL).*|'
    r'TELEM:CUR,(?:OK|FAIL).*|'
    r'WHEEL:(?:SESSION|CFG),(?:OK|FAIL).*|'
    r'MODE,(?:OK|FAIL).*|'
    r'CLEAR_FAULT,(?:OK|FAIL).*|'
    r'SPRING:CFG,(?:OK|FAIL).*|'
    r'DETENT:CFG,(?:OK|FAIL).*|'
    r'CMD:CFG,(?:OK|FAIL).*'
    r')'
)


class SerialOwner(QObject):
    """Owns the COM port and polls it with a non-blocking QTimer."""

    # ── Signals (thread-safe, marshalled to main thread) ─────────
    wheel_event = Signal(object)        # WheelEvent
    ack_received = Signal(str)         # raw ACK line (no CR/LF)
    connection_changed = Signal(bool)  # True=connected
    log_line = Signal(str, str)        # level, message
    serial_rx = Signal(bytes)          # all raw bytes from device → IPC broadcast
    wheel_status_update = Signal(dict) # periodic stats snapshot

    def __init__(self, config: BridgeConfig):
        super().__init__()
        self._config = config
        self._serial: serial.Serial | None = None
        self._poll_timer: QTimer | None = None
        self._bin_parser = BinaryCurrentParser()
        self._text_parser = FOCDataParser()
        self._port_name: str = ""
        self._baud_rate: int = 1000000

        # Accumulated wheel stats
        self._wheel_position: int = 0
        self._wheel_total_delta: int = 0
        self._wheel_events_received: int = 0
        self._last_wheel_status_emit_s: float = 0.0

    # ── Public API (thread-safe via Qt slots) ────────────────────

    @Slot(str, int)
    def connect_port(self, port_name: str, baud_rate: int):
        self.disconnect()
        self._port_name = port_name
        self._baud_rate = baud_rate
        try:
            self._serial = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                timeout=0.05,
                exclusive=True,
            )
            self._bin_parser = BinaryCurrentParser()
            self._text_parser = FOCDataParser()
            self._wheel_position = 0
            self._wheel_total_delta = 0
            self._wheel_events_received = 0
            self._last_wheel_status_emit_s = 0.0

            # Start 10ms poll timer (runs in the owner's thread)
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(10)
            self._poll_timer.timeout.connect(self._poll_serial)
            self._poll_timer.start()

            self.connection_changed.emit(True)
            self.log_line.emit("INFO", f"Bridge connected to {port_name} @ {baud_rate}")
        except Exception as exc:
            self._serial = None
            self.connection_changed.emit(False)
            self.log_line.emit("ERROR", f"Failed to open {port_name}: {exc}")

    @Slot()
    def disconnect(self):
        if self._poll_timer is not None:
            self._poll_timer.stop()
            self._poll_timer = None
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
        was_open = self._serial is not None
        self._serial = None
        if was_open:
            self.connection_changed.emit(False)
            self.log_line.emit("INFO", f"Disconnected from {self._port_name}")

    @Slot(str)
    def send_text(self, text: str) -> bool:
        """Encode text as ASCII + newline and write to serial."""
        return self.send_bytes((text + "\n").encode("ascii", errors="replace"))

    @Slot(bytes)
    def send_bytes(self, data: bytes) -> bool:
        """Write raw bytes to serial port."""
        if self._serial is None or not self._serial.is_open:
            self.log_line.emit("ERROR", "Cannot send: not connected")
            return False
        try:
            self._serial.write(data)
            self._serial.flush()
            return True
        except Exception as exc:
            self.log_line.emit("ERROR", f"Serial write failed: {exc}")
            return False

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def wheel_status_snapshot(self, session_active: bool) -> dict:
        """Return the latest wheel counters for IPC/UI synchronization."""
        return {
            "position_steps": self._wheel_position,
            "total_delta": self._wheel_total_delta,
            "events_received": self._wheel_events_received,
            "session_active": bool(session_active),
        }

    # ── Internal poll loop ────────────────────────────────────────

    @Slot()
    def _poll_serial(self):
        if self._serial is None or not self._serial.is_open:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return

            raw = self._serial.read(waiting)
            if not raw:
                return

            # Broadcast ALL raw bytes for IPC (GUI spectator)
            self.serial_rx.emit(bytes(raw))

            # Feed binary parser — extract W events and residual text
            _cur_samples, wheel_events, text_bytes = self._bin_parser.feed_all(raw)

            for evt in wheel_events:
                self._wheel_events_received += 1
                self._wheel_position = evt.position_steps
                self._wheel_total_delta += abs(evt.delta_steps)
                self.wheel_event.emit(evt)

            # Human wheel input is sparse, so update the GUI by elapsed time
            # instead of waiting for hundreds of detents.
            now_s = time.monotonic()
            if wheel_events and (now_s - self._last_wheel_status_emit_s) >= 0.1:
                self._last_wheel_status_emit_s = now_s
                self.wheel_status_update.emit(
                    self.wheel_status_snapshot(wheel_events[-1].session_active)
                )

            # Feed residual text bytes to ASCII parser, scan for ACK lines
            if text_bytes:
                self._scan_acks(text_bytes)

            # Also scan the raw bytes for ACK lines that might have been
            # consumed by the binary parser (belt and suspenders)
            # Actually, text_bytes are the residual bytes NOT consumed by binary parser.
            # ACKs are plain ASCII lines — they'll be in text_bytes.

        except (OSError, serial.SerialException) as exc:
            self.log_line.emit("ERROR", f"Serial read error: {exc}")
            self.disconnect()

    def _scan_acks(self, text_bytes: bytes):
        """Scan text bytes for ACK lines and emit ack_received."""
        text = text_bytes.decode("ascii", errors="replace")
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            if _ACK_PATTERN.match(line):
                self.ack_received.emit(line)
