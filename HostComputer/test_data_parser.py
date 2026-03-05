import unittest
import sys
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from data_parser import CommandBuilder, FOCDataParser


NORMAL_PACKET = (
    "\r\n========== FOC Controller Status ==========\r\n"
    "Time: 1234 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Angle:    12.34 deg\r\n"
    "  Raw:      5678 (0x162E)\r\n"
    "  CRC:    OK\r\n\r\n"
    "[DRV8350S Driver]\r\n"
    "  FAULT1: 0x0000\r\n"
    "  VGS2:   0x0000\r\n"
    "  Status: Normal\r\n"
    "\r\n[FOC Control]\r\n"
    "  State:  4\r\n"
    "  Id:       0.111 A (ref:   0.000)\r\n"
    "  Iq:       1.234 A (ref:   1.500)\r\n"
    "  Vd:       0.010 V\r\n"
    "  Vq:       2.220 V\r\n"
    "  Speed:    9.87 rad/s\r\n"
    "===========================================\r\n"
)

FAULT_PACKET = (
    "\r\n========== !!! FAULT DETECTED !!! ==========\r\n"
    "Time: 3456 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Angle:    22.22 deg\r\n"
    "  Raw:      2222 (0x08AE)\r\n"
    "  CRC:    ERROR!\r\n\r\n"
    "[DRV8350S Fault Details]\r\n"
    "  FAULT1: 0x0640 | VGS2: 0x00C0\r\n\r\n"
    "  [CRIT]  VDS Overcurrent!\r\n"
    "=============================================\r\n"
)


class TestFOCDataParser(unittest.TestCase):
    def test_parse_normal_packet(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(NORMAL_PACKET.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 1234)
        self.assertAlmostEqual(pkt.angle, 12.34, places=2)
        self.assertEqual(pkt.raw_angle, 5678)
        self.assertFalse(pkt.crc_error)
        self.assertEqual(pkt.foc_state, 4)
        self.assertAlmostEqual(pkt.Id, 0.111, places=3)
        self.assertAlmostEqual(pkt.Iq, 1.234, places=3)
        self.assertAlmostEqual(pkt.Id_ref, 0.0, places=3)
        self.assertAlmostEqual(pkt.Iq_ref, 1.5, places=3)
        self.assertFalse(pkt.is_fault_active)
        self.assertEqual(pkt.fault_status1, 0x0000)
        self.assertEqual(pkt.vgs_status2, 0x0000)

    def test_parse_fault_packet(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(FAULT_PACKET.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 3456)
        self.assertTrue(pkt.crc_error)
        self.assertTrue(pkt.is_fault_active)
        self.assertEqual(pkt.fault_status1, 0x0640)
        self.assertEqual(pkt.vgs_status2, 0x00C0)

    def test_fragmented_input(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))

        data = NORMAL_PACKET.encode("utf-8")
        parser.feed_data(data[:40])
        parser.feed_data(data[40:120])
        parser.feed_data(data[120:])

        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].timestamp, 1234)

    def test_variable_length_separator_is_fully_consumed(self):
        parser = FOCDataParser()
        parser.feed_data(NORMAL_PACKET.encode("utf-8"))
        self.assertEqual(parser.buffer, "")

    def test_noise_without_packet_end_has_bounded_buffer(self):
        parser = FOCDataParser()
        noise = b"NOISE-1234567890\r\n"

        for _ in range(10000):
            parser.feed_data(noise)

        self.assertLessEqual(len(parser.buffer), 8192)


class TestCommandBuilder(unittest.TestCase):
    def test_command_builder_outputs(self):
        self.assertEqual(CommandBuilder.enable_motor(True), "CMD:ENABLE,1\n")
        self.assertEqual(CommandBuilder.enable_motor(False), "CMD:ENABLE,0\n")
        self.assertEqual(CommandBuilder.set_mode(2), "CMD:MODE,2\n")
        self.assertEqual(CommandBuilder.set_current_ref(0.1234, -1.5678), "CMD:IREF,0.123,-1.568\n")
        self.assertEqual(CommandBuilder.set_speed_ref(10.0), "CMD:SREF,10.000\n")
        self.assertEqual(CommandBuilder.set_position_ref(3.14), "CMD:PREF,3.140\n")
        self.assertEqual(CommandBuilder.start_identify(), "CMD:IDENTIFY,1\n")
        self.assertEqual(CommandBuilder.stop_identify(), "CMD:IDENTIFY,0\n")
        self.assertEqual(CommandBuilder.clear_fault(), "CMD:CLEAR_FAULT\n")


if __name__ == "__main__":
    unittest.main()
