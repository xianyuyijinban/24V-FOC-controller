"""CanTransport: CAN byte-pipe Transport implementation.

The transport exposes the same interface as ``DirectSerialTransport`` while
using the CAN v1.0 command tunnel internally. ``write()`` accepts the usual
``command\\n`` bytes from SerialWorker, strips the line ending for the wire
format, and splits the command into tunnel frames. ``read()`` returns
reassembled response payloads.

Telemetry, fault, NMT, and FAST_CTRL frames never enter the byte pipe; they are
exposed through optional callbacks for later application layers.
"""

from __future__ import annotations

from typing import Callable, Optional

try:
    from .can_tunnel import TunnelDecoder, encode_tunnel_payload, strip_line_endings
    from .transport import Transport
except ImportError:
    from can_tunnel import TunnelDecoder, encode_tunnel_payload, strip_line_endings
    from transport import Transport

try:
    import can
except ImportError:
    can = None


def _to_can_message(frame) -> "can.Message":
    if can is None:
        raise RuntimeError("python-can is required for CanTransport")
    return can.Message(
        arbitration_id=frame.arbitration_id,
        data=frame.data,
        is_extended_id=frame.is_extended_id,
    )


class CanTransport(Transport):
    """Transport-compatible tunnel over python-can (gs_usb/candleLight)."""

    def __init__(
        self,
        channel: str | int = 0,
        interface: str = "gs_usb",
        bitrate: int = 500000,
        node_id: int = 1,
        bus=None,
        line_mode: bool = True,
        timeout_ms: int = 300,
        on_telemetry: Optional[Callable] = None,
        on_fault: Optional[Callable] = None,
        on_event: Optional[Callable] = None,
    ):
        self._channel = str(channel)
        self._interface = interface
        self._bitrate = int(bitrate)
        self._node_id = int(node_id) & 0x3F
        self._bus = bus
        self._owns_bus = bus is None
        self._line_mode = bool(line_mode)
        self._rx_buffer = bytearray()
        self._decoder = TunnelDecoder(
            node_id=self._node_id,
            group_base=0x400,
            timeout_ms=timeout_ms,
        )
        self.on_telemetry = on_telemetry
        self.on_fault = on_fault
        self.on_event = on_event

    def open(self) -> bool:
        if self._bus is not None:
            return True
        if can is None:
            return False
        try:
            self._bus = can.Bus(
                interface=self._interface,
                channel=self._channel,
                bitrate=self._bitrate,
            )
            self._owns_bus = True
            return True
        except Exception:
            self._bus = None
            return False

    def close(self):
        self._rx_buffer.clear()
        if self._bus is not None and self._owns_bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        self._bus = None

    def is_open(self) -> bool:
        return self._bus is not None

    def write(self, data: bytes) -> int:
        if self._bus is None:
            return 0

        payload = strip_line_endings(bytes(data))
        if not payload:
            return 0

        try:
            frames = encode_tunnel_payload(
                payload,
                node_id=self._node_id,
                direction="req",
            )
            for frame in frames:
                self._bus.send(_to_can_message(frame))
        except Exception:
            return 0
        return len(data)

    def read(self, max_bytes: int) -> bytes:
        self._drain()
        if not self._rx_buffer or max_bytes <= 0:
            return b""
        count = min(max_bytes, len(self._rx_buffer))
        result = bytes(self._rx_buffer[:count])
        del self._rx_buffer[:count]
        return result

    def bytes_available(self) -> int:
        self._drain()
        return len(self._rx_buffer)

    def _drain(self):
        if self._bus is None:
            return
        try:
            while True:
                message = self._bus.recv(timeout=0)
                if message is None:
                    break
                self._handle_message(message)
        except Exception:
            return

    def _handle_message(self, message):
        if getattr(message, "is_extended_id", 0):
            return

        msg_id = int(message.arbitration_id)
        if (msg_id & 0x3F) != self._node_id:
            return

        group = msg_id & 0x700
        if group == 0x200:
            if self.on_telemetry is not None:
                self.on_telemetry(message)
            return
        if group == 0x500:
            if self.on_fault is not None:
                self.on_fault(message)
            return
        if group in (0x000, 0x100):
            if self.on_event is not None:
                self.on_event(message)
            return
        if group != 0x400:
            return

        for payload in self._decoder.feed(message):
            if self._line_mode and not payload.endswith((b"\r", b"\n")):
                payload += b"\r\n"
            self._rx_buffer.extend(payload)
