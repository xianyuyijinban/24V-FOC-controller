"""CAN v1.0 tunnel framing codec (shared reference implementation).

The wire format follows `docs/CAN_PROTOCOL.md` section 8:

* single frame (SF): data[0] = 0x00 | len (1..7)
* first frame (FF):  data[0] = 0x40, data[1] = total_len (8..255)
* consecutive frame (CF): data[0] = 0x80 | seq, seq = 0,1,2,...
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterable, List, Optional


@dataclass
class TunnelFrame:
    """Minimal CAN frame representation independent of python-can."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False


def tunnel_id(group_base: int, node_id: int) -> int:
    return (group_base | (node_id & 0x3F)) & 0x7FF


def strip_line_endings(data: bytes) -> bytes:
    while data.endswith((b"\r", b"\n")):
        data = data[:-1]
    return data


def encode_tunnel_payload(
    payload: bytes,
    node_id: int = 1,
    direction: str = "req",
) -> List[TunnelFrame]:
    """Encode a command/response payload into tunnel frames."""

    if direction == "req":
        base = 0x300
    elif direction == "resp":
        base = 0x400
    else:
        raise ValueError("direction must be 'req' or 'resp'")

    payload = bytes(payload)
    if not 1 <= len(payload) <= 255:
        raise ValueError("tunnel payload length must be 1..255")

    msg_id = tunnel_id(base, node_id)
    frames: List[TunnelFrame] = []

    if len(payload) <= 7:
        data = bytes([0x00 | len(payload)]) + payload
        frames.append(TunnelFrame(msg_id, data))
        return frames

    ff = bytearray(8)
    ff[0] = 0x40
    ff[1] = len(payload)
    ff[2:8] = payload[:6]
    frames.append(TunnelFrame(msg_id, bytes(ff)))

    pos = 6
    seq = 0
    while pos < len(payload):
        chunk = min(7, len(payload) - pos)
        data = bytes([0x80 | seq]) + payload[pos : pos + chunk]
        frames.append(TunnelFrame(msg_id, data))
        pos += chunk
        seq += 1

    return frames


class TunnelDecoder:
    """Stateful decoder for tunneled byte streams.

    Feed CAN frames with ``arbitration_id`` and ``data`` attributes. Complete
    payloads are returned from :meth:`feed`; partial or invalid sequences are
    discarded according to protocol section 8.3.
    """

    def __init__(
        self,
        node_id: int = 1,
        group_base: int = 0x400,
        timeout_ms: int = 300,
        now=None,
    ):
        self._node_id = node_id & 0x3F
        self._group_base = group_base
        self._timeout_ms = float(timeout_ms)
        self._now = now or time.monotonic

        self._buffer = bytearray()
        self._expected_total = 0
        self._next_seq = 0
        self._active = False
        self._last_frame_at = 0.0

    def _reset(self) -> None:
        self._buffer.clear()
        self._expected_total = 0
        self._next_seq = 0
        self._active = False
        self._last_frame_at = 0.0

    def feed(self, message) -> List[bytes]:
        """Feed one frame and return any completed payloads."""

        if message is None:
            return []

        msg_id = int(getattr(message, "arbitration_id", 0))
        if int(getattr(message, "is_extended_id", 0)):
            return []
        if (msg_id & ~0x3F) != self._group_base:
            return []
        if (msg_id & 0x3F) != self._node_id:
            return []

        data = bytes(getattr(message, "data", b""))
        if not data:
            return []

        now = self._now()
        if self._active and (now - self._last_frame_at) > self._timeout_ms:
            self._reset()

        first = data[0]
        completed: List[bytes] = []

        if (first & 0xC0) == 0x00:
            # SF starts a new message and replaces any old reassembly.
            length = first & 0x3F
            self._reset()
            if 1 <= length <= 7 and len(data) >= length + 1:
                completed.append(bytes(data[1 : 1 + length]))
            return completed

        if first == 0x40:
            # FF starts a new multi-frame message.
            self._reset()
            if len(data) >= 2 and 8 <= data[1] <= 255:
                self._buffer.extend(data[2:8])
                self._expected_total = data[1]
                self._next_seq = 0
                self._active = True
                self._last_frame_at = now
                if len(self._buffer) >= self._expected_total:
                    completed.append(bytes(self._buffer[: self._expected_total]))
                    self._reset()
            return completed

        if (first & 0x80) == 0x80:
            if not self._active or (first & 0x7F) != self._next_seq:
                self._reset()
                return []

            chunk = bytes(data[1:8])
            if len(chunk) > 7:
                chunk = chunk[:7]
            self._buffer.extend(chunk)
            self._next_seq += 1
            self._last_frame_at = now
            if len(self._buffer) >= self._expected_total:
                completed.append(bytes(self._buffer[: self._expected_total]))
                self._reset()
            return completed

        self._reset()
        return []


def decode_tunnel_messages(
    messages: Iterable,
    node_id: int = 1,
    group_base: int = 0x400,
    timeout_ms: int = 300,
) -> List[bytes]:
    """One-shot decoder for tests and small offline use cases."""

    decoder = TunnelDecoder(
        node_id=node_id,
        group_base=group_base,
        timeout_ms=timeout_ms,
    )
    result: List[bytes] = []
    for message in messages:
        result.extend(decoder.feed(message))
    return result
