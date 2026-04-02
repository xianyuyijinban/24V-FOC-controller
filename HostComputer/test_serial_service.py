import sys
import unittest
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from serial_service import SerialService


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.is_open = True

    def write(self, payload: bytes):
        self.writes.append(payload)


class FakeParser:
    def __init__(self):
        self.payloads = []

    def feed_data(self, data: bytes):
        self.payloads.append(data)


class TestSerialService(unittest.TestCase):
    def test_send_command_encodes_text(self):
        fake_serial = FakeSerial()
        service = SerialService(serial_port=fake_serial, parser=FakeParser())
        service.send_command("CMD:ENABLE,1\n")
        self.assertEqual(fake_serial.writes, [b"CMD:ENABLE,1\n"])

    def test_handle_bytes_delegates_to_parser(self):
        parser = FakeParser()
        service = SerialService(serial_port=FakeSerial(), parser=parser)
        service.handle_bytes(b"abc")
        self.assertEqual(parser.payloads, [b"abc"])


if __name__ == "__main__":
    unittest.main()
