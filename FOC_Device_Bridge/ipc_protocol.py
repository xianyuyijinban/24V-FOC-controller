"""
FOC Device Bridge — IPC Protocol (FB Frame Codec)

Binary frame format for QLocalSocket communication between
FOC_Device_Bridge (COM port owner) and HostComputer GUI (spectator).

Frame: 'F' 'B' | version:u8 | kind:u8 | length:u32 LE | payload

Constants:
  MAGIC = b'FB' (0x46, 0x42)
  HEADER_LEN = 8  (magic(2) + version(1) + kind(1) + length(4))

Kinds:
  0x01 SERIAL_RX     Bridge→GUI   raw bytes from device
  0x02 SERIAL_TX     GUI→Bridge   raw bytes to send to device
  0x03 CONN_STATE    Bridge→GUI   JSON status
  0x04 CONNECT       GUI→Bridge   JSON {port, baud}
  0x10 WHEEL_ENABLE  GUI→Bridge   enable(\\x01) / disable(\\x00)
  0x11 WHEEL_STATUS  Bridge→GUI   JSON stats
  0x12 WHEEL_CONFIG  GUI→Bridge   JSON cfg (set)
  0x13 WHEEL_CONFIG  Bridge→GUI   JSON cfg (query response)
  0xFF ERROR         either       error string
"""

import struct
from typing import Optional, Tuple

MAGIC = b'FB'
HEADER_LEN = 8  # magic(2) + version(1) + kind(1) + length(4)
VERSION = 1

# ── Kind constants ──────────────────────────────────────────────────
KIND_SERIAL_RX     = 0x01
KIND_SERIAL_TX     = 0x02
KIND_CONN_STATE    = 0x03
KIND_CONNECT_REQUEST = 0x04
KIND_WHEEL_ENABLE  = 0x10
KIND_WHEEL_STATUS  = 0x11
KIND_WHEEL_CONFIG  = 0x12  # set (GUI→Bridge) or response (Bridge→GUI, kind 0x13)
KIND_WHEEL_CONFIG_RESP = 0x13
KIND_ERROR         = 0xFF

# ── Public API ──────────────────────────────────────────────────────

def encode_frame(kind: int, payload: bytes) -> bytes:
    """Encode a single FB frame. Returns MAGIC(2) + version(1) + kind(1) + length_u32le(4) + payload."""
    header = struct.pack('<2s B B I', MAGIC, VERSION, kind, len(payload))
    return header + payload


def decode_frame(data: bytes) -> Optional[Tuple[int, bytes]]:
    """Try to decode one FB frame from the start of `data`.

    Returns (kind, payload) if a complete valid frame is found.
    Returns None if the buffer contains only a partial frame header.
    The caller MUST remove the consumed bytes from its buffer.
    """
    if len(data) < HEADER_LEN:
        return None

    magic, version, kind, length = struct.unpack_from('<2s B B I', data, 0)
    if magic != MAGIC:
        raise ValueError(f"Bad magic: {magic!r} (expected {MAGIC!r})")

    if version != VERSION:
        raise ValueError(f"Unsupported version: {version} (expected {VERSION})")

    total = HEADER_LEN + length
    if len(data) < total:
        return None  # partial frame — wait for more data

    payload = data[HEADER_LEN:total]
    return kind, payload


def frame_size(data: bytes) -> Optional[int]:
    """Return the total frame size in bytes given the header prefix, or None if incomplete."""
    if len(data) < HEADER_LEN:
        return None
    magic = data[:2]
    if magic != MAGIC:
        return None
    version = data[2]
    if version != VERSION:
        return None
    kind = data[3]
    length = struct.unpack_from('<I', data, 4)[0]
    return HEADER_LEN + length
