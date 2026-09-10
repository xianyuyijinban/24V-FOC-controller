"""Tests for the shared CAN tunnel framing reference implementation."""

import random
import unittest

from can_tunnel import TunnelDecoder, TunnelFrame, decode_tunnel_messages, encode_tunnel_payload


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TestEncodeDecode(unittest.TestCase):
    def roundtrip(self, payload: bytes):
        frames = encode_tunnel_payload(payload, node_id=1, direction="resp")
        decoded = decode_tunnel_messages(frames, node_id=1, group_base=0x400)
        self.assertEqual(decoded, [payload])

    def test_single_frame_boundaries(self):
        for length in (1, 7):
            payload = bytes(range(length))
            frames = encode_tunnel_payload(payload, node_id=2, direction="req")
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0].arbitration_id, 0x302)
            self.assertEqual(frames[0].data[0], length)
            self.assertEqual(frames[0].data[1:], payload)
            self.roundtrip(payload)

    def test_multi_frame_boundaries(self):
        for length in (8, 255):
            payload = bytes((i * 7) % 256 for i in range(length))
            frames = encode_tunnel_payload(payload, node_id=1, direction="resp")
            self.assertGreater(len(frames), 1)
            self.assertEqual(frames[0].arbitration_id, 0x401)
            self.assertEqual(frames[0].data[0], 0x40)
            self.assertEqual(frames[0].data[1], length)
            self.roundtrip(payload)

    def test_random_roundtrip_lengths(self):
        rng = random.Random(20260801)
        for _ in range(200):
            length = rng.randint(1, 255)
            payload = bytes(rng.randrange(256) for _ in range(length))
            self.roundtrip(payload)

    def test_sequence_break_discards_buffer(self):
        payload = bytes(range(20))
        frames = encode_tunnel_payload(payload, node_id=1, direction="resp")
        broken = [frames[0], frames[1], TunnelFrame(0x401, bytes([0x82]) + bytes(7))]
        decoder = TunnelDecoder(node_id=1, group_base=0x400)
        complete = []
        for frame in broken:
            complete.extend(decoder.feed(frame))
        self.assertEqual(complete, [])

    def test_new_ff_preempts_old_message(self):
        old = encode_tunnel_payload(bytes(range(20)), node_id=1, direction="resp")
        new = encode_tunnel_payload(b"NEW!", node_id=1, direction="resp")
        decoder = TunnelDecoder(node_id=1, group_base=0x400)
        complete = []
        for frame in old[:2] + new:
            complete.extend(decoder.feed(frame))
        self.assertEqual(complete, [b"NEW!"])

    def test_timeout_discards_partial_message(self):
        clock = FakeClock()
        frames = encode_tunnel_payload(bytes(range(20)), node_id=1, direction="resp")
        decoder = TunnelDecoder(node_id=1, group_base=0x400, timeout_ms=1, now=clock)
        self.assertEqual(decoder.feed(frames[0]), [])
        self.assertEqual(decoder.feed(frames[1]), [])
        clock.t = 2.0
        self.assertEqual(decoder.feed(frames[2]), [])


if __name__ == "__main__":
    unittest.main()
