import unittest
import sys
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from data_parser import CommandBuilder, FOCDataParser, AckParser, AckResult, BinaryCurrentParser

COMPACT_NORMAL_FRAME = "N,1234,4,12.34,9.87,0.111,1.234,11.98,0x00000000,1,1,0,1,18.00,28.00\n"
COMPACT_FAULT_FRAME = "F,3456,5,0x00000020,1,0,1,0x0640,0x00C0,0xFFFF,11.98,18.00,28.00\n"
COMPACT_NORMAL_FRAME_WITH_EXTRA_FIELD = "N,1234,4,12.34,9.87,0.111,1.234,11.98,0x00000000,1,1,0,1,0,9.00,16.00\n"
LEGACY_COMPACT_FAULT_FRAME = "F,3456,5,0x00000020,1,0,0x0640,0x00C0,0xFFFF,18.00,28.00\n"
LEGACY_COMPACT_NORMAL_FRAME = "N,1234,4,12.34,9.87,0.111,1.234,11.98,0x00000000,1,1,0\n"
LEGACY_APP_STATE_FAULT_NORMAL_FRAME = "N,2234,5,12.34,0.00,0.000,0.000,11.90,0x00000000,1,1,0,18.00,28.00\n"
APP_FAULT_NORMAL_FRAME = "N,2234,5,12.34,0.00,0.000,0.000,11.90,0x00000000,1,1,0,0,3,18.00,28.00\n"
APP_FAULT_COMPACT_FRAME = "F,4456,5,0x00000000,0,1,0,0x0000,0x0000,0x0000,3,11.90,18.00,28.00\n"
APP_WARNING_NORMAL_FRAME = "N,5234,3,12.34,0.00,0.000,0.000,8.80,0x00000000,1,1,0,0,1,0,9.00,16.00\n"
APP_WARNING_FAULT_FRAME = "F,5456,5,0x00000000,0,1,0,0x0000,0x0000,0x0000,1,3,8.40,9.00,16.00\n"
IDENTIFY_FAULT_COMPACT_FRAME = "F,6456,5,0x00000000,0,1,0,0x0000,0x0000,0x0000,0,6,9,12,11.90,9.00,16.00\n"
COMPACT_POSITION_RUNTIME_FRAME = "N,7234,4,109.90,0.26,-0.810,-1.250,11.87,0x00000000,1,1,0,0,0,0,2,0.000,-1.900,0.000,-0.800,0.120,-0.450,0.010,1.230,-0.040,9.00,16.00\n"
COMPACT_IDENTIFY_RUNTIME_FRAME = "N,8234,2,117.39,0.00,0.720,-0.110,11.90,0x00000000,1,0,0,0,0x00000000,0,1,1.500,0.000,0.000,0.000,5.800,-3.900,-1.200,0.020,-0.300,7,1,9.00,16.00\n"
COMPACT_RUNTIME_FRAME_WITH_ADC_OFFSETS = "N,9234,4,12.00,0.10,0.010,0.020,11.95,0x00000000,1,1,0,0,0x00000000,0,1,0.000,0.000,0.000,0.000,0.100,0.200,0.300,-0.100,0.050,0,0,9.00,16.00,1,2056,2055,2332\n"
COMPACT_FAULT_FRAME_WITH_ADC_OFFSETS = "F,9456,5,0x10000000,1,1,0,0x0000,0x0000,0x0000,0x00000000,4,0,0,11.95,9.00,16.00,2,2048,2048,2048\n"


NORMAL_PACKET = (
    "\r\n========== FOC Controller Status ==========\r\n"
    "Time: 1234 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Detected: YES\r\n"
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
    "  Identified: YES\r\n"
    "  StallMode: OFF\r\n"
    "  StallOpenLoop: OFF\r\n"
    "===========================================\r\n"
)

FAULT_PACKET = (
    "\r\n========== !!! FAULT DETECTED !!! ==========\r\n"
    "Time: 3456 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Detected: NO\r\n"
    "  Angle:    22.22 deg\r\n"
    "  Raw:      2222 (0x08AE)\r\n"
    "  CRC:    ERROR!\r\n\r\n"
    "[DRV8350S Fault Details]\r\n"
    "  FAULT1: 0x0640 | VGS2: 0x00C0\r\n\r\n"
    "  Identified: NO\r\n"
    "  StallMode: ARMED\r\n"
    "  StallOpenLoop: ACTIVE\r\n"
    "  [CRIT]  VDS Overcurrent!\r\n"
    "=============================================\r\n"
)

FAULT_PACKET_ANGLE_RAW_ONLY = (
    "\r\n========== !!! FAULT DETECTED !!! ==========\r\n"
    "Time: 70902 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Detected: YES\r\n"
    "  AngleRaw: 13487 (0x34AF)\r\n"
    "  CRC:      OK\r\n"
    "  Safety:   0x7E\r\n"
    "  Reset:    FAULT!\r\n\r\n"
    "[FOC Application]\r\n"
    "  State:    FAULT\r\n"
    "  AppFault: 6 (Param Invalid)\r\n"
    "=============================================\r\n"
)

DETAILED_STATUS_PACKET_WITH_PARAM_DIAG = (
    "\r\n========== !!! FAULT DETECTED !!! ==========\r\n"
    "Time: 2595 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Detected: YES\r\n"
    "  AngleRaw: 13125 (0x3345)\r\n"
    "  CRC:      OK\r\n\r\n"
    "[FOC Application]\r\n"
    "  State:    READY\r\n"
    "  AppFault: 0 (None)\r\n\r\n"
    "[Motor Parameters]\r\n"
    "  ParamDiag: invalid=0x00000000 | valid=0xFFFFFFFF | Rs=8.800 Ohm | "
    "Ld=0.000500 H | Lq=0.000500 H | Ke=0.129000 | Pn=11 | enc_dir=-1 | J=0.000100\r\n"
    "=============================================\r\n"
)

RUNNING_DIAGNOSTIC_PACKET_WITH_LEGACY_FAULT_TITLE = (
    "\r\n========== !!! FAULT DETECTED !!! ==========\r\n"
    "Time: 2338354 ms\r\n\r\n"
    "[TLE5012 Encoder]\r\n"
    "  Detected: YES\r\n"
    "  AngleRaw: 23822 (0x5D0E)\r\n"
    "  CRC:      OK\r\n"
    "  Safety:   0x7E\r\n"
    "  Reset:    FAULT!\r\n\r\n"
    "[DRV8350S Communication]\r\n"
    "  Identified: YES\r\n"
    "  StallMode:  OFF\r\n\r\n"
    "[FOC Application]\r\n"
    "  State:    RUNNING\r\n"
    "  Power:   pwm=1 moe=1 mode=1\r\n"
    "  AppFault: 0 (None)\r\n\r\n"
    "[DRV8350S Fault Details]\r\n"
    "  FAULT1: 0x0000 | VGS2: 0x0000\r\n"
    "=============================================\r\n"
)

RUNNING_DIAGNOSTIC_PACKET = RUNNING_DIAGNOSTIC_PACKET_WITH_LEGACY_FAULT_TITLE.replace(
    "========== !!! FAULT DETECTED !!! ==========",
    "========== FOC Diagnostic Snapshot ==========",
)


class TestFOCDataParser(unittest.TestCase):
    def test_parse_high_rate_phase_current_frame(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(b"C,1205,0.125,-0.250,0.125\n")

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 1205)
        self.assertAlmostEqual(pkt.Ia, 0.125, places=3)
        self.assertAlmostEqual(pkt.Ib, -0.250, places=3)
        self.assertAlmostEqual(pkt.Ic, 0.125, places=3)
        self.assertTrue(pkt.phase_current_only)
        self.assertEqual(pkt.raw_text, "C,1205,0.125,-0.250,0.125")

    def test_parse_compact_normal_frame(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_NORMAL_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 1234)
        self.assertEqual(pkt.foc_state, 4)
        self.assertAlmostEqual(pkt.angle, 12.34, places=2)
        self.assertAlmostEqual(pkt.speed, 9.87, places=2)
        self.assertAlmostEqual(pkt.Id, 0.111, places=3)
        self.assertAlmostEqual(pkt.Iq, 1.234, places=3)
        self.assertAlmostEqual(getattr(pkt, "vbus", 0.0), 11.98, places=2)
        self.assertEqual(pkt.fault_flags, 0x00000000)
        self.assertTrue(pkt.encoder_detected)
        self.assertTrue(pkt.motor_identified)
        self.assertFalse(pkt.stall_mode_armed)
        self.assertTrue(pkt.stall_open_loop_active)
        self.assertFalse(pkt.is_fault_active)
        self.assertAlmostEqual(pkt.undervoltage_limit, 18.0, places=2)
        self.assertAlmostEqual(pkt.overvoltage_limit, 28.0, places=2)

    def test_parse_compact_fault_frame(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_FAULT_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 3456)
        self.assertEqual(pkt.foc_state, 5)
        self.assertEqual(pkt.fault_flags, 0x00000020)
        self.assertEqual(pkt.fault_status1, 0x0640)
        self.assertEqual(pkt.vgs_status2, 0x00C0)
        self.assertFalse(pkt.encoder_detected)
        self.assertTrue(pkt.stall_open_loop_active)
        self.assertTrue(pkt.is_fault_active)
        self.assertAlmostEqual(getattr(pkt, "vbus", 0.0), 11.98, places=2)
        self.assertAlmostEqual(pkt.undervoltage_limit, 18.0, places=2)
        self.assertAlmostEqual(pkt.overvoltage_limit, 28.0, places=2)

    def test_parse_compact_normal_frame_uses_tail_threshold_fields(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_NORMAL_FRAME_WITH_EXTRA_FIELD.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertTrue(pkt.stall_open_loop_active)
        self.assertAlmostEqual(pkt.undervoltage_limit, 9.0, places=2)
        self.assertAlmostEqual(pkt.overvoltage_limit, 16.0, places=2)

    def test_parse_legacy_compact_fault_frame_without_vbus(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(LEGACY_COMPACT_FAULT_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertAlmostEqual(getattr(pkt, "vbus", 0.0), 0.0, places=2)
        self.assertAlmostEqual(pkt.undervoltage_limit, 18.0, places=2)
        self.assertAlmostEqual(pkt.overvoltage_limit, 28.0, places=2)

    def test_parse_legacy_compact_normal_frame_without_thresholds(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(LEGACY_COMPACT_NORMAL_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertIsNone(pkt.undervoltage_limit)
        self.assertIsNone(pkt.overvoltage_limit)

    def test_parse_compact_normal_frame_marks_fault_when_state_is_fault(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(APP_FAULT_NORMAL_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 5)
        self.assertEqual(pkt.app_fault_code, 3)
        self.assertTrue(pkt.is_fault_active)

    def test_parse_legacy_compact_normal_frame_marks_fault_when_state_is_fault(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(LEGACY_APP_STATE_FAULT_NORMAL_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 5)
        self.assertEqual(pkt.app_fault_code, 0)
        self.assertTrue(pkt.is_fault_active)

    def test_parse_compact_fault_frame_preserves_application_fault_code(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(APP_FAULT_COMPACT_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 5)
        self.assertEqual(pkt.app_fault_code, 3)
        self.assertTrue(pkt.is_fault_active)
        self.assertAlmostEqual(pkt.vbus, 11.90, places=2)

    def test_parse_compact_normal_frame_preserves_warning_flags_without_marking_fault(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(APP_WARNING_NORMAL_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 3)
        self.assertEqual(pkt.app_warning_flags, 1)
        self.assertEqual(pkt.app_fault_code, 0)
        self.assertFalse(pkt.is_fault_active)
        self.assertAlmostEqual(pkt.vbus, 8.80, places=2)

    def test_parse_compact_position_runtime_references(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_POSITION_RUNTIME_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.control_mode, 2)
        self.assertAlmostEqual(pkt.Id_ref, 0.0, places=3)
        self.assertAlmostEqual(pkt.Iq_ref, -0.8, places=3)
        self.assertAlmostEqual(pkt.Vd, 0.12, places=3)
        self.assertAlmostEqual(pkt.Vq, -0.45, places=3)
        self.assertAlmostEqual(pkt.Ia, 0.010, places=3)
        self.assertAlmostEqual(pkt.Ib, 1.230, places=3)
        self.assertAlmostEqual(pkt.Ic, -0.040, places=3)
        self.assertAlmostEqual(pkt.speed_ref, -1.9, places=3)
        self.assertAlmostEqual(pkt.pos_ref, 0.0, places=3)
        self.assertAlmostEqual(pkt.undervoltage_limit, 9.0, places=3)
        self.assertAlmostEqual(pkt.overvoltage_limit, 16.0, places=3)
        self.assertIsNone(pkt.identify_state)
        self.assertIsNone(pkt.identify_error)

    def test_parse_compact_identification_runtime_state(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_IDENTIFY_RUNTIME_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 2)
        self.assertEqual(pkt.identify_state, 7)
        self.assertEqual(pkt.identify_error, 1)
        self.assertAlmostEqual(pkt.Id_ref, 1.5, places=3)
        self.assertAlmostEqual(pkt.Ia, -1.2, places=3)
        self.assertAlmostEqual(pkt.undervoltage_limit, 9.0, places=3)
        self.assertAlmostEqual(pkt.overvoltage_limit, 16.0, places=3)

    def test_parse_compact_runtime_frame_preserves_adc_calibration_fields(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_RUNTIME_FRAME_WITH_ADC_OFFSETS.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertAlmostEqual(pkt.undervoltage_limit, 9.0, places=3)
        self.assertAlmostEqual(pkt.overvoltage_limit, 16.0, places=3)
        self.assertEqual(pkt.adc_calib_status, 1)
        self.assertEqual(pkt.adc_offset_a, 2056)
        self.assertEqual(pkt.adc_offset_b, 2055)
        self.assertEqual(pkt.adc_offset_c, 2332)

    def test_parse_compact_fault_frame_preserves_adc_calibration_fields(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(COMPACT_FAULT_FRAME_WITH_ADC_OFFSETS.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.adc_calib_status, 2)
        self.assertEqual(pkt.adc_offset_a, 2048)
        self.assertEqual(pkt.adc_offset_b, 2048)
        self.assertEqual(pkt.adc_offset_c, 2048)

    def test_parse_compact_fault_frame_preserves_warning_and_fault_flags_together(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(APP_WARNING_FAULT_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.app_warning_flags, 1)
        self.assertEqual(pkt.app_fault_code, 3)
        self.assertTrue(pkt.is_fault_active)
        self.assertAlmostEqual(pkt.vbus, 8.40, places=2)

    def test_emits_boot_diagnostic_lines_without_packet(self):
        packets = []
        diagnostics = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.set_diagnostic_callback(diagnostics.append)

        parser.feed_data(b"BOOT,USART_READY\r\nBOOT,PERIPH_READY\r\n")

        self.assertEqual(packets, [])
        self.assertEqual(diagnostics, ["BOOT,USART_READY", "BOOT,PERIPH_READY"])

    def test_preserves_partial_diagnostic_line_until_newline(self):
        diagnostics = []
        parser = FOCDataParser()
        parser.set_diagnostic_callback(diagnostics.append)

        parser.feed_data(b"BOOT,USART")
        self.assertEqual(diagnostics, [])

        parser.feed_data(b"_READY\r\n")
        self.assertEqual(diagnostics, ["BOOT,USART_READY"])

    def test_tle_raw_safety_line_is_diagnostic_not_compact_fault_packet(self):
        packets = []
        diagnostics = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.set_diagnostic_callback(diagnostics.append)

        parser.feed_data(
            b"F,safety=0x7E45,status=0x7E,recv_crc=0x45,calc_crc=0x45,"
            b"data_ok=1,crc_error=0,valid=1,angle=160.30\r\n"
        )

        self.assertEqual(packets, [])
        self.assertEqual(
            diagnostics,
            [
                "F,safety=0x7E45,status=0x7E,recv_crc=0x45,calc_crc=0x45,"
                "data_ok=1,crc_error=0,valid=1,angle=160.30"
            ],
        )

    def test_parse_compact_fault_frame_preserves_identification_error(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(IDENTIFY_FAULT_COMPACT_FRAME.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.app_fault_code, 6)
        self.assertEqual(pkt.identify_state, 9)
        self.assertEqual(pkt.identify_error, 12)
        self.assertAlmostEqual(pkt.vbus, 11.90, places=2)

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
        self.assertTrue(pkt.encoder_detected)
        self.assertEqual(pkt.foc_state, 4)
        self.assertAlmostEqual(pkt.Id, 0.111, places=3)
        self.assertAlmostEqual(pkt.Iq, 1.234, places=3)
        self.assertAlmostEqual(pkt.Id_ref, 0.0, places=3)
        self.assertAlmostEqual(pkt.Iq_ref, 1.5, places=3)
        self.assertFalse(pkt.is_fault_active)
        self.assertEqual(pkt.fault_status1, 0x0000)
        self.assertEqual(pkt.vgs_status2, 0x0000)
        self.assertTrue(pkt.motor_identified)
        self.assertFalse(pkt.stall_mode_armed)
        self.assertFalse(pkt.stall_open_loop_active)

    def test_parse_fault_packet(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(FAULT_PACKET.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 3456)
        self.assertTrue(pkt.crc_error)
        self.assertFalse(pkt.encoder_detected)
        self.assertTrue(pkt.is_fault_active)
        self.assertEqual(pkt.fault_status1, 0x0640)
        self.assertEqual(pkt.vgs_status2, 0x00C0)
        self.assertFalse(pkt.motor_identified)
        self.assertTrue(pkt.stall_mode_armed)
        self.assertTrue(pkt.stall_open_loop_active)

    def test_parse_fault_packet_angle_raw_only_derives_angle_degrees(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(FAULT_PACKET_ANGLE_RAW_ONLY.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.raw_angle, 13487)
        self.assertTrue(pkt.encoder_detected)
        self.assertAlmostEqual(pkt.angle, 13487.0 * 360.0 / 65536.0, places=2)
        self.assertTrue(pkt.is_fault_active)

    def test_parse_detailed_packet_extracts_motor_param_diag(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(DETAILED_STATUS_PACKET_WITH_PARAM_DIAG.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.timestamp, 2595)
        self.assertEqual(pkt.foc_state, 3)
        self.assertAlmostEqual(pkt.motor_param_rs, 8.8, places=3)
        self.assertAlmostEqual(pkt.motor_param_ld, 0.0005, places=6)
        self.assertAlmostEqual(pkt.motor_param_lq, 0.0005, places=6)
        self.assertAlmostEqual(pkt.motor_param_ke, 0.129, places=6)
        self.assertEqual(pkt.motor_param_pn, 11)
        self.assertEqual(pkt.motor_param_encoder_dir, -1)
        self.assertFalse(pkt.is_fault_active)

    def test_parse_running_diagnostic_with_legacy_fault_title_is_not_active_fault(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(RUNNING_DIAGNOSTIC_PACKET_WITH_LEGACY_FAULT_TITLE.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        pkt = packets[0]
        self.assertEqual(pkt.foc_state, 4)
        self.assertEqual(pkt.control_mode, 1)
        self.assertTrue(pkt.encoder_detected)
        self.assertTrue(pkt.motor_identified)
        self.assertFalse(pkt.is_fault_active)

    def test_parse_running_diagnostic_snapshot_title(self):
        packets = []
        parser = FOCDataParser()
        parser.set_packet_callback(lambda pkt: packets.append(pkt))
        parser.feed_data(RUNNING_DIAGNOSTIC_PACKET.encode("utf-8"))

        self.assertEqual(len(packets), 1)
        self.assertFalse(packets[0].is_fault_active)
        self.assertEqual(packets[0].control_mode, 1)

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
        self.assertEqual(CommandBuilder.unlock_power(True), "CMD:UNLOCK,1\n")
        self.assertEqual(CommandBuilder.unlock_power(False), "CMD:UNLOCK,0\n")
        self.assertEqual(CommandBuilder.enable_motor(True), "CMD:ENABLE,1\n")
        self.assertEqual(CommandBuilder.enable_motor(False), "CMD:ENABLE,0\n")
        self.assertEqual(CommandBuilder.set_mode(2), "CMD:MODE,2\n")
        self.assertEqual(CommandBuilder.set_current_ref(0.1234, -1.5678), "CMD:IREF,0.123,-1.568\n")
        self.assertEqual(CommandBuilder.set_speed_ref(10.0), "CMD:SREF,10.000\n")
        self.assertEqual(CommandBuilder.set_position_ref(3.14), "CMD:PREF,3.140\n")
        self.assertEqual(CommandBuilder.set_motion_cfg(4.0, 6.0, 1.2), "CMD:MOTION_CFG,4.000,6.000,1.200\n")
        self.assertEqual(CommandBuilder.query_motion_cfg(), "CMD:MOTION_CFG?\n")
        self.assertEqual(CommandBuilder.reset_motion_cfg(), "CMD:MOTION_CFG_RESET\n")
        self.assertEqual(CommandBuilder.set_cogging_cfg(0.25, 60.0), "CMD:COG_CFG,0.250,60.000\n")
        self.assertEqual(CommandBuilder.query_cogging_cfg(), "CMD:COG_CFG?\n")
        self.assertEqual(CommandBuilder.start_identify(), "CMD:IDENTIFY,1\n")
        self.assertEqual(CommandBuilder.stop_identify(), "CMD:IDENTIFY,0\n")
        self.assertEqual(CommandBuilder.clear_fault(), "CMD:CLEAR_FAULT\n")
        self.assertEqual(CommandBuilder.set_stall_mode(True), "CMD:STALL_MODE,1\n")
        self.assertEqual(CommandBuilder.set_stall_mode(False), "CMD:STALL_MODE,0\n")
        self.assertEqual(CommandBuilder.set_motor_pn(7), "CMD:MOTOR_PN,7\n")
        self.assertEqual(CommandBuilder.set_vbus_limits(9.0, 15.0), "CMD:VBUS_LIMIT,9.000,15.000\n")
        self.assertEqual(CommandBuilder.adc_phase_scan(128), "CMD:ADC_PHASE_SCAN,128\n")
        self.assertEqual(CommandBuilder.adc_sector_scan(256), "CMD:ADC_SECTOR_SCAN,256\n")
        self.assertEqual(CommandBuilder.tle_gpio_diag(True), "CMD:TLE_GPIO_DIAG,1\n")
        self.assertEqual(CommandBuilder.tle_gpio_diag(False), "CMD:TLE_GPIO_DIAG,0\n")
        self.assertEqual(CommandBuilder.tle_raw(), "CMD:TLE_RAW\n")
        self.assertEqual(CommandBuilder.fault_detail(), "CMD:FAULT_DETAIL\n")
        self.assertEqual(CommandBuilder.set_current_pi(0.2, 0.01), "CMD:PI_CURRENT,0.200000,0.010000\n")
        self.assertEqual(CommandBuilder.set_speed_pi(1.0, 0.1), "CMD:PI_SPEED,1.000000,0.100000\n")
        self.assertEqual(CommandBuilder.set_position_pd(3.0, 0.5), "CMD:PD_POS,3.000000,0.500000\n")

    def test_telem_cur_commands(self):
        self.assertEqual(CommandBuilder.telem_cur_off(), "TELEM:CUR,OFF\n")
        self.assertEqual(CommandBuilder.telem_cur_ascii(200), "TELEM:CUR,ASCII,200\n")
        self.assertEqual(CommandBuilder.telem_cur_bin(1000), "TELEM:CUR,BIN,1000\n")
        self.assertEqual(CommandBuilder.telem_cur_bin(2000), "TELEM:CUR,BIN,2000\n")


class TestBinaryCurrentParserTextResidual(unittest.TestCase):
    def test_short_ascii_ack_is_released_immediately(self):
        parser = BinaryCurrentParser()

        samples, text = parser.feed(b"UNLOCK,OK,1\r\n")

        self.assertEqual(samples, [])
        self.assertEqual(text, b"UNLOCK,OK,1\r\n")

    def test_single_sync_prefix_is_kept_for_next_chunk(self):
        parser = BinaryCurrentParser()

        samples, text = parser.feed(b"abc\xA5")

        self.assertEqual(samples, [])
        self.assertEqual(text, b"abc")

        samples, text = parser.feed(b"not_binary\r\n")
        self.assertEqual(samples, [])
        self.assertEqual(text, b"\xA5not_binary\r\n")

    def test_text_before_binary_sync_is_released(self):
        parser = BinaryCurrentParser()

        samples, text = parser.feed(b"ENABLE,OK,1\r\n\xA5\x5A")

        self.assertEqual(samples, [])
        self.assertEqual(text, b"ENABLE,OK,1\r\n")


class TestAckParser(unittest.TestCase):
    def test_parse_unlock_ok(self):
        parser = AckParser()
        result = parser.parse_line("UNLOCK,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "UNLOCK")
        self.assertIsNone(result.reason)
        self.assertIsNone(result.command_value)

    def test_parse_power_ok_with_value(self):
        parser = AckParser()

        unlock = parser.parse_line("UNLOCK,OK,1")
        self.assertIsNotNone(unlock)
        self.assertTrue(unlock.ok)
        self.assertEqual(unlock.command, "UNLOCK")
        self.assertEqual(unlock.command_value, "1")

        enable = parser.parse_line("ENABLE,OK,0")
        self.assertIsNotNone(enable)
        self.assertTrue(enable.ok)
        self.assertEqual(enable.command, "ENABLE")
        self.assertEqual(enable.command_value, "0")

    def test_parse_unlock_fail(self):
        parser = AckParser()
        result = parser.parse_line("UNLOCK,FAIL,busy")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.command, "UNLOCK")
        self.assertEqual(result.reason, "busy")

    def test_parse_enable_ok(self):
        parser = AckParser()
        result = parser.parse_line("ENABLE,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "ENABLE")

    def test_parse_enable_fail(self):
        parser = AckParser()
        result = parser.parse_line("ENABLE,FAIL,not unlocked")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.command, "ENABLE")
        self.assertEqual(result.reason, "not unlocked")

    def test_parse_identify_ok(self):
        parser = AckParser()
        result = parser.parse_line("IDENTIFY,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "IDENTIFY")

    def test_parse_identify_fail(self):
        parser = AckParser()
        result = parser.parse_line("IDENTIFY,FAIL,motor not idle")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "motor not idle")

    def test_parse_mode_ok_with_value(self):
        parser = AckParser()
        result = parser.parse_line("MODE,OK,2")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "MODE")
        self.assertEqual(result.mode_value, 2)

    def test_parse_mode_fail(self):
        parser = AckParser()
        result = parser.parse_line("MODE,FAIL,invalid mode")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "invalid mode")

    def test_parse_app_mode_ok(self):
        parser = AckParser()
        result = parser.parse_line("APP_MODE,OK,JOINT_POS (ctrl_mode=2)")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "APP_MODE")
        self.assertEqual(result.app_mode_name, "JOINT_POS")
        self.assertEqual(result.app_mode_ctrl, 2)

    def test_parse_app_mode_ok_no_ctrl(self):
        parser = AckParser()
        result = parser.parse_line("APP_MODE,OK,GIMBAL_SPEED")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.app_mode_name, "GIMBAL_SPEED")
        self.assertIsNone(result.app_mode_ctrl)

    def test_parse_app_mode_fail(self):
        parser = AckParser()
        result = parser.parse_line("APP_MODE,FAIL,unknown mode")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "unknown mode")

    def test_parse_stall_mode_ok(self):
        parser = AckParser()
        result = parser.parse_line("STALL_MODE,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "STALL_MODE")

    def test_parse_clear_fault_ok(self):
        parser = AckParser()
        result = parser.parse_line("CLEAR_FAULT,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "CLEAR_FAULT")

    def test_parse_iref_ok(self):
        parser = AckParser()
        result = parser.parse_line("IREF,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "IREF")

    def test_parse_pref_fail(self):
        parser = AckParser()
        result = parser.parse_line("PREF,FAIL,out of range")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.command, "PREF")
        self.assertEqual(result.reason, "out of range")

    def test_parse_motor_pn_ok(self):
        parser = AckParser()
        result = parser.parse_line("MOTOR_PN,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "MOTOR_PN")

    def test_parse_encoder_dir_fail(self):
        parser = AckParser()
        result = parser.parse_line("ENCODER_DIR,FAIL,must be +1 or -1")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "must be +1 or -1")

    def test_parse_home_ok(self):
        parser = AckParser()
        result = parser.parse_line("HOME,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "HOME")

    def test_parse_clear_home_fail(self):
        parser = AckParser()
        result = parser.parse_line("CLEAR_HOME,FAIL,no home set")
        self.assertIsNotNone(result)
        self.assertFalse(result.ok)
        self.assertEqual(result.command, "CLEAR_HOME")
        self.assertEqual(result.reason, "no home set")

    def test_parse_non_ack_line_returns_none(self):
        parser = AckParser()
        self.assertIsNone(parser.parse_line("N,1234,4,12.34,..."))
        self.assertIsNone(parser.parse_line(""))
        self.assertIsNone(parser.parse_line("JOINT:LIMIT,OK,min=-30.0deg,max=30.0deg"))
        self.assertIsNone(parser.parse_line("GIMBAL:RAMP,OK,accel=2.0radps2"))
        self.assertIsNone(parser.parse_line("random noise"))
        self.assertIsNone(parser.parse_line("   "))

    def test_parse_sref_ok(self):
        parser = AckParser()
        result = parser.parse_line("SREF,OK")
        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "SREF")


class TestBinaryCurrentParser(unittest.TestCase):
    def test_short_ascii_ack_not_held_for_binary_frame(self):
        """Short ASCII ACK lines (e.g. UNLOCK,OK,1\\r\\n) must be returned
        as residual text, not held waiting for a 25-byte binary frame."""
        parser = BinaryCurrentParser()
        samples, text = parser.feed(b"UNLOCK,OK,1\r\n")
        self.assertEqual(samples, [])
        self.assertEqual(text, b"UNLOCK,OK,1\r\n")


if __name__ == "__main__":
    unittest.main()
