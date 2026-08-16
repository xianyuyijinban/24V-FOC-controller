"""
FOC Device Bridge — Session Manager

Implements the ACK-waiting enable/disable sequence for SCROLL_WHEEL mode,
plus 250ms keepalive timer while wheel is active.

Enable sequence (6 steps, 500ms timeout each):
  1. TELEM:CUR,OFF
  2. CMD:APP_MODE,SCROLL_WHEEL   ← before SESSION (ResetState called here)
  3. WHEEL:SESSION,<id>,1000
  4. WHEEL:CFG,<count>,<strength>,<width>,<damping>,<limit>
  5. CMD:UNLOCK,1
  6. CMD:ENABLE,1

Disable sequence (3 steps):
  1. CMD:ENABLE,0
  2. CMD:UNLOCK,0
  3. CMD:APP_MODE,RAW

Any FAIL → sends emergency STOP sequence, emits error, returns to IDLE.
"""

import random
import re

from PySide6.QtCore import QObject, QTimer, Signal

from .config_manager import BridgeConfig

# ── States ─────────────────────────────────────────────────────────
STATE_DISCONNECTED    = "DISCONNECTED"
STATE_CONNECTING      = "CONNECTING"
STATE_CONNECTED_IDLE  = "CONNECTED_IDLE"
STATE_WHEEL_ENABLING  = "WHEEL_ENABLING"
STATE_WHEEL_ACTIVE    = "WHEEL_ACTIVE"
STATE_WHEEL_DISABLING = "WHEEL_DISABLING"

ACK_TIMEOUT_MS = 500
KEEPALIVE_INTERVAL_MS = 250
SESSION_TIMEOUT_MS = 1000


class SessionManager(QObject):
    """Manages the wheel enable/disable lifecycle."""

    # Signals
    state_changed = Signal(str)         # new state name
    command_to_send = Signal(str)       # text command (no trailing \n)
    wheel_active_changed = Signal(bool) # True when WHEEL_ACTIVE
    error_occurred = Signal(str)        # human-readable error

    def __init__(self, config: BridgeConfig):
        super().__init__()
        self._config = config
        self._state: str = STATE_DISCONNECTED
        self._session_id: int = 0
        self._seq_step: int = 0
        self._seq_commands: list[str] = []
        self._seq_expected_acks: list[str] = []

        # ACK timeout timer
        self._ack_timer = QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)

        # Keepalive timer
        self._keepalive_timer = QTimer(self)
        self._keepalive_timer.setInterval(KEEPALIVE_INTERVAL_MS)
        self._keepalive_timer.timeout.connect(self._on_keepalive)

    # ── Public API ──────────────────────────────────────────────────

    def current_state(self) -> str:
        return self._state

    def set_connected(self, connected: bool):
        """Called by bridge_app when serial connection state changes."""
        if connected:
            if self._state == STATE_DISCONNECTED:
                self._transition(STATE_CONNECTED_IDLE)
        else:
            self._cancel_sequence()
            self._keepalive_timer.stop()
            self._transition(STATE_DISCONNECTED)

    def start_enable(self):
        """Begin the 6-step wheel enable sequence."""
        if self._state not in (STATE_CONNECTED_IDLE,):
            self.error_occurred.emit(f"Cannot enable wheel in state {self._state}")
            return

        self._session_id = random.randint(1, 0x7FFFFFFF)

        cfg = self._config
        wcfg = cfg.wheel_config_dict()

        self._seq_step = 0
        self._seq_commands = [
            "TELEM:CUR,OFF",
            "CMD:APP_MODE,SCROLL_WHEEL",
            f"WHEEL:SESSION,{self._session_id},{SESSION_TIMEOUT_MS}",
            f"WHEEL:CFG,{wcfg['count']},{wcfg['strength']:.3f},{wcfg['width']:.3f},{wcfg['damping']:.3f},{wcfg['limit']:.3f}",
            "CMD:UNLOCK,1",
            "CMD:ENABLE,1",
        ]
        self._seq_expected_acks = [
            "CUR_STREAM",
            "APP_MODE",
            "WHEEL:SESSION",
            "WHEEL:CFG",
            "UNLOCK",
            "ENABLE",
        ]
        self._transition(STATE_WHEEL_ENABLING)
        self._send_next()

    def start_disable(self):
        """Begin the 3-step wheel disable sequence."""
        if self._state not in (STATE_WHEEL_ACTIVE, STATE_WHEEL_ENABLING):
            self.error_occurred.emit(f"Cannot disable wheel in state {self._state}")
            return

        self._cancel_sequence()
        self._keepalive_timer.stop()

        self._seq_step = 0
        self._seq_commands = [
            "CMD:ENABLE,0",
            "CMD:UNLOCK,0",
            "CMD:APP_MODE,RAW",
        ]
        self._seq_expected_acks = [
            "ENABLE",
            "UNLOCK",
            "APP_MODE",
        ]
        self._transition(STATE_WHEEL_DISABLING)
        self._send_next()

    def on_ack(self, text: str):
        """Feed an ACK line from the serial stream.
        Returns True if the ACK was consumed by the sequence FSM."""
        text = text.strip()

        if self._state == STATE_WHEEL_ACTIVE:
            # Not in a sequence — check for session expiry or other spontaneous ACKs
            return False

        if self._state not in (STATE_WHEEL_ENABLING, STATE_WHEEL_DISABLING):
            return False

        if self._seq_step <= 0 or self._seq_step > len(self._seq_expected_acks):
            return False

        expected = self._seq_expected_acks[self._seq_step - 1]

        # Check if this ACK matches the expected command prefix
        if not text.startswith(expected):
            # Could be an OK without prefix
            if text == "OK":
                pass  # Accept bare OK
            else:
                return False  # Not our ACK

        # Check for FAIL
        if "FAIL" in text:
            self._ack_timer.stop()
            reason = text.split(",", 2)[-1] if text.count(",") >= 2 else "unknown"
            self._abort_sequence(f"Step {self._seq_step} ({expected}) FAILED: {reason}")
            return True

        # ACK OK → advance to next step
        self._ack_timer.stop()

        if self._seq_step >= len(self._seq_commands):
            # Sequence complete!
            self._on_sequence_complete()
        else:
            self._send_next()

        return True

    def stop(self):
        """Emergency stop — send ENABLE,0 immediately, cancel everything."""
        self._cancel_sequence()
        self._keepalive_timer.stop()
        self.command_to_send.emit("CMD:ENABLE,0")
        if self._state not in (STATE_DISCONNECTED, STATE_CONNECTED_IDLE):
            self._transition(STATE_CONNECTED_IDLE)

    # ── Internal ────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        if self._state == new_state:
            return
        self._state = new_state
        self.state_changed.emit(new_state)
        self.wheel_active_changed.emit(new_state == STATE_WHEEL_ACTIVE)

    def _send_next(self):
        """Emit the next command in the sequence."""
        if self._seq_step >= len(self._seq_commands):
            return

        cmd = self._seq_commands[self._seq_step]
        self._seq_step += 1
        self.command_to_send.emit(cmd)

        # Start ACK timeout
        self._ack_timer.start(ACK_TIMEOUT_MS)

    def _on_sequence_complete(self):
        if self._state == STATE_WHEEL_ENABLING:
            self._transition(STATE_WHEEL_ACTIVE)
            self._keepalive_timer.start()
        elif self._state == STATE_WHEEL_DISABLING:
            self._transition(STATE_CONNECTED_IDLE)

    def _abort_sequence(self, reason: str):
        """Send emergency cleanup and return to IDLE."""
        self._cancel_sequence()

        # Best-effort cleanup
        self.command_to_send.emit("CMD:ENABLE,0")
        self.command_to_send.emit("CMD:UNLOCK,0")
        self.command_to_send.emit("CMD:APP_MODE,RAW")

        self.error_occurred.emit(reason)
        if self._state != STATE_CONNECTED_IDLE:
            self._transition(STATE_CONNECTED_IDLE)

    def _cancel_sequence(self):
        self._ack_timer.stop()
        self._seq_step = 0
        self._seq_commands.clear()
        self._seq_expected_acks.clear()

    def _on_ack_timeout(self):
        if self._state not in (STATE_WHEEL_ENABLING, STATE_WHEEL_DISABLING):
            return
        expected = self._seq_expected_acks[self._seq_step - 1] if self._seq_step > 0 else "?"
        self._abort_sequence(f"ACK timeout waiting for {expected} (step {self._seq_step})")

    def _on_keepalive(self):
        if self._state != STATE_WHEEL_ACTIVE:
            self._keepalive_timer.stop()
            return
        if self._session_id > 0:
            self.command_to_send.emit(f"WHEEL:KEEPALIVE,{self._session_id}")
