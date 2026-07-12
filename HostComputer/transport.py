"""
HostComputer — Transport Abstraction Layer

Allows SerialWorker to work with either:
  - DirectSerialTransport:  opens COM port directly (existing behavior, always available)
  - BridgeTransport:        connects to FOC_Device_Bridge via QLocalSocket IPC
"""

import json
import struct
from typing import Optional

import serial
from PySide6.QtCore import QObject, Signal
from PySide6.QtNetwork import QLocalSocket

# Reuse the FB frame codec from FOC_Device_Bridge
# (imported lazily to avoid hard dependency — DirectSerialTransport doesn't need it)
_MAGIC = b'FB'
_HEADER_LEN = 8
_KIND_SERIAL_RX = 0x01
_KIND_SERIAL_TX = 0x02
_KIND_CONN_STATE = 0x03
_KIND_CONNECT_REQUEST = 0x04
_KIND_WHEEL_ENABLE = 0x10
_KIND_WHEEL_STATUS = 0x11
_KIND_ERROR = 0xFF


# ══════════════════════════════════════════════════════════════════════
# Transport ABC
# ══════════════════════════════════════════════════════════════════════

class Transport:
    """Abstract interface for serial data transport.

    Subclasses must implement all methods. Not a formal ABC to avoid
    metaclass conflicts when combined with QObject.
    """

    def open(self) -> bool:
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def is_open(self) -> bool:
        raise NotImplementedError

    def write(self, data: bytes) -> int:
        raise NotImplementedError

    def read(self, max_bytes: int) -> bytes:
        raise NotImplementedError

    def bytes_available(self) -> int:
        raise NotImplementedError


# ══════════════════════════════════════════════════════════════════════
# DirectSerialTransport — legacy COM port
# ══════════════════════════════════════════════════════════════════════

class DirectSerialTransport(Transport):
    """Opens a COM port directly using pyserial."""

    def __init__(self, port: str, baud: int = 1000000, timeout: float = 0.05):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._serial: serial.Serial | None = None

    def open(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._timeout,
            )
            return True
        except serial.SerialException:
            self._serial = None
            return False

    def close(self):
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
        self._serial = None

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def write(self, data: bytes) -> int:
        if self._serial is None or not self._serial.is_open:
            return 0
        return self._serial.write(data)

    def read(self, max_bytes: int) -> bytes:
        if self._serial is None or not self._serial.is_open:
            return b''
        return self._serial.read(max_bytes)

    def bytes_available(self) -> int:
        if self._serial is None or not self._serial.is_open:
            return 0
        return self._serial.in_waiting


# ══════════════════════════════════════════════════════════════════════
# BridgeTransport — IPC to FOC_Device_Bridge
# ══════════════════════════════════════════════════════════════════════

class BridgeTransport(QObject, Transport):
    """Connects to FOC_Device_Bridge via QLocalSocket.

    All serial bytes flow through the bridge's IPC server.
    HostComputer is a spectator that can send commands and receive telemetry.
    """

    connected = Signal()
    disconnected = Signal()
    data_received = Signal(bytes)
    device_connection_changed = Signal(bool)
    connection_state_changed = Signal(dict)
    wheel_status_changed = Signal(dict)
    bridge_error = Signal(str)

    def __init__(self, server_name: str = "FOC_Device_Bridge_v1"):
        QObject.__init__(self)
        self._server_name = server_name
        self._socket: QLocalSocket | None = None
        self._rx_buffer = bytearray()
        self._write_buffer = bytearray()
        self._device_connected = False
        self._connection_state: dict = {
            "state": "DISCONNECTED",
            "port": "",
            "baud": 0,
        }
        self._wheel_status: dict = {}

    # ── Transport interface ─────────────────────────────────────

    def open(self) -> bool:
        """Try to connect to the bridge IPC server."""
        if self._socket is not None:
            self.close()

        self._socket = QLocalSocket()
        self._socket.readyRead.connect(self._on_ready_read)
        self._socket.disconnected.connect(self._on_disconnected)

        # QLocalSocket::connectToServer is the non-blocking Qt5+ method
        # Wait for connection (up to 2s)
        self._socket.connectToServer(self._server_name)
        if self._socket.waitForConnected(2000):
            if self._socket.bytesAvailable() == 0:
                self._socket.waitForReadyRead(250)
            self._on_ready_read()
            self.connected.emit()
            return True
        self._socket.close()
        self._socket = None
        return False

    def close(self):
        self._rx_buffer.clear()
        self._write_buffer.clear()
        self._device_connected = False
        self._connection_state = {"state": "DISCONNECTED", "port": "", "baud": 0}
        self._wheel_status = {}
        if self._socket is not None:
            try:
                self._socket.disconnectFromServer()
                if self._socket.state() != QLocalSocket.LocalSocketState.UnconnectedState:
                    self._socket.waitForDisconnected(500)
            except Exception:
                pass
            self._socket.close()
            self._socket = None

    def is_open(self) -> bool:
        return (self._socket is not None
                and self._socket.state() == QLocalSocket.LocalSocketState.ConnectedState
                and self._device_connected)

    def is_ipc_connected(self) -> bool:
        return (self._socket is not None
                and self._socket.state() == QLocalSocket.LocalSocketState.ConnectedState)

    def device_connected(self) -> bool:
        return self._device_connected

    def connection_state(self) -> dict:
        return dict(self._connection_state)

    def wheel_status(self) -> dict:
        return dict(self._wheel_status)

    def write(self, data: bytes) -> int:
        """Encode data as a SERIAL_TX FB frame and send to bridge."""
        if not self.is_open() or self._socket is None:
            return 0

        # Encode as FB frame
        frame = _encode_fb_frame(_KIND_SERIAL_TX, data)
        try:
            written = self._socket.write(frame)
            self._socket.flush()
            return len(data)  # report original data length
        except Exception:
            return 0

    def request_device_connection(self, port: str, baud: int) -> bool:
        payload = json.dumps({"port": port, "baud": int(baud)}).encode("utf-8")
        return self._send_control_frame(_KIND_CONNECT_REQUEST, payload)

    def request_wheel_enable(self, enable: bool) -> bool:
        return self._send_control_frame(_KIND_WHEEL_ENABLE, b"\x01" if enable else b"\x00")

    def _send_control_frame(self, kind: int, payload: bytes) -> bool:
        if not self.is_ipc_connected() or self._socket is None:
            return False
        try:
            self._socket.write(_encode_fb_frame(kind, payload))
            self._socket.flush()
            return True
        except Exception:
            return False

    def read(self, max_bytes: int) -> bytes:
        """Drain from the internal RX buffer."""
        if not self._rx_buffer:
            return b''
        count = min(max_bytes, len(self._rx_buffer))
        result = bytes(self._rx_buffer[:count])
        del self._rx_buffer[:count]
        return result

    def bytes_available(self) -> int:
        return len(self._rx_buffer)

    # ── Qt socket handlers ──────────────────────────────────────

    def _on_ready_read(self):
        if self._socket is None:
            return

        data = bytes(self._socket.readAll())
        if not data:
            return

        buf = self._write_buffer + data
        self._write_buffer = bytearray()

        while len(buf) >= _HEADER_LEN:
            try:
                magic, version, kind, length = struct.unpack_from('<2s B B I', bytes(buf), 0)
            except struct.error:
                break

            if magic != _MAGIC:
                # Bad magic — skip one byte and retry
                buf.pop(0)
                continue

            total = _HEADER_LEN + length
            if len(buf) < total:
                # Partial frame — keep in write_buffer for next read
                self._write_buffer = buf
                return

            payload = bytes(buf[_HEADER_LEN:total])
            del buf[:total]

            if kind == _KIND_SERIAL_RX:
                # Append SERIAL_RX payload to our RX buffer
                self._rx_buffer.extend(payload)
                self.data_received.emit(bytes(payload))
            else:
                self._handle_control_frame(kind, payload)
            # Other frame types (CONN_STATE, WHEEL_STATUS, ERROR) are
            # ignored by the transport layer — they're for the GUI layer.

        self._write_buffer = buf

    def _on_disconnected(self):
        self._rx_buffer.clear()
        self._write_buffer.clear()
        was_connected = self._device_connected
        self._device_connected = False
        self._connection_state = {"state": "DISCONNECTED", "port": "", "baud": 0}
        self._wheel_status = {}
        if was_connected:
            self.device_connection_changed.emit(False)
        self.disconnected.emit()

    def _handle_control_frame(self, kind: int, payload: bytes):
        if kind == _KIND_CONN_STATE:
            try:
                state = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.bridge_error.emit("Invalid CONN_STATE frame from bridge")
                return
            state_name = str(state.get("state", "DISCONNECTED"))
            port = str(state.get("port", ""))
            connected = state_name != "DISCONNECTED" and bool(port)
            changed = connected != self._device_connected
            self._device_connected = connected
            self._connection_state = state
            self.connection_state_changed.emit(dict(state))
            if changed:
                self.device_connection_changed.emit(connected)
            return

        if kind == _KIND_WHEEL_STATUS:
            try:
                status = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.bridge_error.emit("Invalid WHEEL_STATUS frame from bridge")
                return
            self._wheel_status = status
            self.wheel_status_changed.emit(status)
            return

        if kind == _KIND_ERROR:
            self.bridge_error.emit(payload.decode("utf-8", errors="replace"))


def _encode_fb_frame(kind: int, payload: bytes) -> bytes:
    """Minimal FB frame encoder (avoids importing FOC_Device_Bridge from HostComputer)."""
    header = struct.pack('<2s B B I', _MAGIC, 1, kind, len(payload))
    return header + payload
