"""Tests for CanTransport as a Transport byte-pipe implementation."""

import time
import unittest
import uuid

import can

from can_tunnel import TunnelFrame, encode_tunnel_payload
from can_transport import CanTransport


class FakeBus:
    def __init__(self):
        self.sent = []
        self.rx = []

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout=0):
        if self.rx:
            return self.rx.pop(0)
        return None

    def shutdown(self):
        pass


class TestCanTransport(unittest.TestCase):
    def test_transport_contract_methods_exist(self):
        transport = CanTransport(bus=FakeBus())
        self.assertTrue(hasattr(transport, "open"))
        self.assertTrue(hasattr(transport, "close"))
        self.assertTrue(hasattr(transport, "is_open"))
        self.assertTrue(hasattr(transport, "write"))
        self.assertTrue(hasattr(transport, "read"))
        self.assertTrue(hasattr(transport, "bytes_available"))

    def test_write_strips_newline_and_returns_original_length(self):
        bus = FakeBus()
        transport = CanTransport(bus=bus, node_id=1)
        transport.open()
        result = transport.write(b"GAIN:PI_SPEED,0.25,0.001\n")
        self.assertEqual(result, len(b"GAIN:PI_SPEED,0.25,0.001\n"))

        decoded = []
        from can_tunnel import TunnelDecoder

        decoder = TunnelDecoder(node_id=1, group_base=0x300)
        for message in bus.sent:
            decoded.extend(decoder.feed(message))
        self.assertEqual(decoded, [b"GAIN:PI_SPEED,0.25,0.001"])

    def test_read_reassembles_response_without_line_mode(self):
        bus = FakeBus()
        transport = CanTransport(bus=bus, node_id=1, line_mode=False)
        transport.open()
        payload = b"FW_INFO,OK,version=1.0.0,param=1,baseline=12V_STANDARD,git=deadbeef"
        bus.rx.extend(encode_tunnel_payload(payload, node_id=1, direction="resp"))
        self.assertEqual(transport.bytes_available(), len(payload))
        self.assertEqual(transport.read(4096), payload)

    def test_read_line_mode_keeps_parser_contract(self):
        bus = FakeBus()
        transport = CanTransport(bus=bus, node_id=1, line_mode=True)
        transport.open()
        bus.rx.extend(encode_tunnel_payload(b"UNLOCK,OK,1", node_id=1, direction="resp"))
        self.assertEqual(transport.read(4096), b"UNLOCK,OK,1\r\n")

    def test_telemetry_does_not_enter_byte_pipe(self):
        bus = FakeBus()
        telemetry = []
        transport = CanTransport(bus=bus, node_id=1, on_telemetry=telemetry.append)
        transport.open()
        bus.rx.append(TunnelFrame(0x201, bytes(8)))
        self.assertEqual(transport.bytes_available(), 0)
        self.assertEqual(len(telemetry), 1)

    def test_python_can_virtual_bus_roundtrip(self):
        channel = f"codex_can_transport_{uuid.uuid4().hex}"
        peer = can.Bus(interface="virtual", channel=channel)
        transport_bus = can.Bus(interface="virtual", channel=channel)
        transport = CanTransport(bus=transport_bus, node_id=1, line_mode=False)
        transport.open()

        transport.write(b"SYS:FW_INFO?\n")
        frames = []
        for _ in range(3):
            frame = peer.recv(timeout=0.2)
            if frame is not None:
                frames.append(frame)
        decoded = []
        from can_tunnel import TunnelDecoder

        decoder = TunnelDecoder(node_id=1, group_base=0x300)
        for frame in frames:
            decoded.extend(decoder.feed(frame))
        self.assertEqual(decoded, [b"SYS:FW_INFO?"])

        response = encode_tunnel_payload(b"FW_INFO,OK", node_id=1, direction="resp")
        for frame in response:
            peer.send(can.Message(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
                is_extended_id=False,
            ))
        deadline = time.time() + 1.0
        while transport.bytes_available() == 0 and time.time() < deadline:
            time.sleep(0.01)
        self.assertEqual(transport.read(4096), b"FW_INFO,OK")

        peer.shutdown()
        transport.close()
        transport_bus.shutdown()


if __name__ == "__main__":
    unittest.main()
