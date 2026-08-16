"""
FOC Device Bridge — IPC Server

QLocalServer on named pipe "FOC_Device_Bridge_v1".
Accepts connections from HostComputer GUI spectators.
Broadcasts SERIAL_RX, CONN_STATE, WHEEL_STATUS to all clients.
Receives SERIAL_TX, WHEEL_ENABLE, WHEEL_CONFIG from clients.
"""

import json
import struct

from PySide6.QtCore import QObject, Signal
from PySide6.QtNetwork import QLocalServer, QLocalSocket

from .ipc_protocol import (
    encode_frame,
    decode_frame,
    HEADER_LEN,
    KIND_SERIAL_RX,
    KIND_SERIAL_TX,
    KIND_CONN_STATE,
    KIND_CONNECT_REQUEST,
    KIND_WHEEL_ENABLE,
    KIND_WHEEL_STATUS,
    KIND_WHEEL_CONFIG,
    KIND_ERROR,
)


class IpcServer(QObject):
    """QLocalServer for HostComputer GUI spectator connections."""

    # Signals to bridge_app
    serial_tx = Signal(bytes)            # GUI wants to send data to device
    wheel_enable_request = Signal(bool)  # GUI wants to enable/disable wheel
    wheel_config_request = Signal(dict)  # GUI wants to change wheel config
    connect_request = Signal(str, int)    # GUI asks Bridge to own a COM port

    def __init__(self, server_name: str = "FOC_Device_Bridge_v1"):
        super().__init__()
        self._server_name = server_name
        self._server: QLocalServer | None = None
        self._clients: list[QLocalSocket] = []
        self._client_buffers: dict[QLocalSocket, bytearray] = {}
        self._last_conn_state = json.dumps(
            {"state": "DISCONNECTED", "port": "", "baud": 0}
        ).encode("utf-8")
        self._last_wheel_status: bytes | None = None

    # ── Public API ──────────────────────────────────────────────────

    def start(self) -> bool:
        """Start listening. Returns False if server name is already in use."""
        probe = QLocalSocket()
        probe.connectToServer(self._server_name)
        if probe.waitForConnected(150):
            probe.disconnectFromServer()
            return False
        probe.abort()

        self._server = QLocalServer(self)
        self._server.newConnection.connect(self._on_new_connection)

        # No live owner answered, so a remaining endpoint is stale.
        QLocalServer.removeServer(self._server_name)

        if not self._server.listen(self._server_name):
            return False
        return True

    def stop(self):
        """Stop listening and disconnect all clients."""
        for client in list(self._clients):
            self._remove_client(client)
        if self._server is not None:
            self._server.close()
            self._server = None

    def broadcast(self, kind: int, payload: bytes):
        """Send an FB frame to all connected clients."""
        frame = encode_frame(kind, payload)
        dead: list[QLocalSocket] = []
        for client in self._clients:
            try:
                client.write(frame)
                client.flush()
            except Exception:
                dead.append(client)
        for d in dead:
            self._remove_client(d)

    # ── Convenience broadcast methods ───────────────────────────────

    def broadcast_serial_rx(self, data: bytes):
        self.broadcast(KIND_SERIAL_RX, data)

    def broadcast_conn_state(self, state: str, port: str = "", baud: int = 0):
        payload = json.dumps({"state": state, "port": port, "baud": baud}).encode("utf-8")
        self._last_conn_state = payload
        self.broadcast(KIND_CONN_STATE, payload)

    def broadcast_wheel_status(self, stats: dict):
        payload = json.dumps(stats).encode("utf-8")
        self._last_wheel_status = payload
        self.broadcast(KIND_WHEEL_STATUS, payload)

    def broadcast_error(self, message: str):
        self.broadcast(KIND_ERROR, message.encode("utf-8"))

    # ── Internal ────────────────────────────────────────────────────

    def _on_new_connection(self):
        while self._server is not None and self._server.hasPendingConnections():
            client = self._server.nextPendingConnection()
            if client is None:
                continue
            client.readyRead.connect(lambda c=client: self._on_client_data(c))
            client.disconnected.connect(lambda c=client: self._on_client_disconnected(c))
            self._clients.append(client)
            self._client_buffers[client] = bytearray()
            client.write(encode_frame(KIND_CONN_STATE, self._last_conn_state))
            if self._last_wheel_status is not None:
                client.write(encode_frame(KIND_WHEEL_STATUS, self._last_wheel_status))
            client.flush()

    def _on_client_data(self, client: QLocalSocket):
        buf = self._client_buffers.get(client, bytearray())
        data = bytes(client.readAll())
        buf.extend(data)

        while len(buf) >= HEADER_LEN:
            try:
                result = decode_frame(bytes(buf))
            except ValueError:
                # Bad magic or version — skip one byte and retry
                buf.pop(0)
                continue

            if result is None:
                # Partial frame — wait for more
                break

            kind, payload = result
            # Remove the consumed frame from buffer
            try:
                _magic, _version, _kind_byte, length = struct.unpack_from('<2s B B I', bytes(buf), 0)
                consumed = HEADER_LEN + length
            except Exception:
                consumed = HEADER_LEN
            del buf[:consumed]

            self._dispatch(kind, payload)

        self._client_buffers[client] = buf

    def _on_client_disconnected(self, client: QLocalSocket):
        self._remove_client(client)
        # IMPORTANT: Do NOT stop motor — GUI is a spectator.

    def _remove_client(self, client: QLocalSocket):
        if client in self._clients:
            self._clients.remove(client)
        self._client_buffers.pop(client, None)
        try:
            client.close()
        except Exception:
            pass

    def _dispatch(self, kind: int, payload: bytes):
        if kind == KIND_SERIAL_TX:
            self.serial_tx.emit(payload)
        elif kind == KIND_CONNECT_REQUEST:
            try:
                request = json.loads(payload.decode("utf-8"))
                port = str(request.get("port", "")).strip()
                baud = int(request.get("baud", 1000000))
                if port:
                    self.connect_request.emit(port, baud)
            except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
                self.broadcast_error("Invalid bridge connect request")
        elif kind == KIND_WHEEL_ENABLE:
            enable = len(payload) > 0 and payload[0] == 1
            self.wheel_enable_request.emit(enable)
        elif kind == KIND_WHEEL_CONFIG:
            try:
                cfg = json.loads(payload.decode("utf-8"))
                self.wheel_config_request.emit(cfg)
            except (json.JSONDecodeError, UnicodeDecodeError):
                pass
        # KIND_SERIAL_RX, CONN_STATE, WHEEL_STATUS are broadcast-only; we don't receive them
