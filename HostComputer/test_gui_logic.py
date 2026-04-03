import sys
import tempfile
import unittest
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from data_parser import FOCDataPacket
from gui_logic import (
    GuiProfile,
    HostAppState,
    LoopTuning,
    RollingPlotBuffer,
    apply_command_effects,
    build_current_ref_command,
    build_pi_command,
    build_position_ref_command,
    build_speed_ref_command,
    button_enable_state,
    fault_summary_text,
    format_plot_csv,
    is_data_stale,
    load_gui_profile,
    mode_target_label,
    packet_snapshot,
    parse_float_field,
    save_gui_profile,
)


class TestGuiLogic(unittest.TestCase):
    def test_mode_target_label_matches_control_mode(self):
        self.assertEqual(mode_target_label(0), "Iq_ref (A)")
        self.assertEqual(mode_target_label(1), "Speed (rad/s)")
        self.assertEqual(mode_target_label(2), "Position (rad)")

    def test_parse_float_field_rejects_empty_input(self):
        with self.assertRaisesRegex(ValueError, "Speed is required"):
            parse_float_field("", "Speed")

    def test_reference_dispatch_routes_to_expected_commands(self):
        self.assertEqual(build_current_ref_command("0.25", "1.5"), "CMD:IREF,0.250,1.500\n")
        self.assertEqual(build_speed_ref_command("12.5"), "CMD:SREF,12.500\n")
        self.assertEqual(build_position_ref_command("-3.0"), "CMD:PREF,-3.000\n")

    def test_pi_dispatch_routes_to_expected_commands(self):
        self.assertEqual(build_pi_command("current", "0.100", "0.002"), "CMD:PI_CURRENT,0.100000,0.002000\n")
        self.assertEqual(build_pi_command("speed", "1.250", "0.015"), "CMD:PI_SPEED,1.250000,0.015000\n")
        self.assertEqual(build_pi_command("position", "3.000", "0.250"), "CMD:PI_POS,3.000000,0.250000\n")

    def test_button_enable_state_tracks_connection_and_power_workflow(self):
        disconnected = button_enable_state(HostAppState(is_connected=False))
        self.assertFalse(disconnected["can_unlock"])
        self.assertFalse(disconnected["can_send_target"])
        self.assertFalse(disconnected["can_identify_start"])

        connected = button_enable_state(HostAppState(is_connected=True, power_unlocked=False))
        self.assertTrue(connected["can_unlock"])
        self.assertFalse(connected["can_enable"])
        self.assertFalse(connected["can_identify_start"])

        armed = button_enable_state(
            HostAppState(is_connected=True, power_unlocked=True, motor_enabled=True, identify_active=True)
        )
        self.assertFalse(armed["can_unlock"])
        self.assertTrue(armed["can_lock"])
        self.assertFalse(armed["can_enable"])
        self.assertTrue(armed["can_disable"])
        self.assertFalse(armed["can_identify_start"])
        self.assertTrue(armed["can_identify_stop"])

    def test_apply_command_effects_updates_runtime_state(self):
        state = HostAppState(is_connected=True)
        apply_command_effects(state, "CMD:UNLOCK,1")
        self.assertTrue(state.power_unlocked)
        apply_command_effects(state, "CMD:ENABLE,1")
        self.assertTrue(state.motor_enabled)
        apply_command_effects(state, "CMD:IDENTIFY,1")
        self.assertTrue(state.identify_active)
        apply_command_effects(state, "CMD:UNLOCK,0")
        self.assertFalse(state.power_unlocked)
        self.assertFalse(state.motor_enabled)
        self.assertFalse(state.identify_active)

    def test_packet_snapshot_formats_runtime_values(self):
        packet = FOCDataPacket(
            timestamp=123,
            angle=12.5,
            speed=2.5,
            Id=0.1,
            Iq=0.2,
            Id_ref=0.0,
            Iq_ref=0.3,
            Vd=0.4,
            Vq=0.5,
            foc_state=4,
        )
        snapshot = packet_snapshot(packet)
        self.assertEqual(snapshot["timestamp"], "123 ms")
        self.assertEqual(snapshot["angle"], "12.50 deg")
        self.assertEqual(snapshot["speed"], "2.50 rad/s")
        self.assertIn("0.10", snapshot["currents"])

    def test_fault_summary_text_highlights_registers(self):
        packet = FOCDataPacket(
            timestamp=456,
            is_fault_active=True,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
        )
        summary = fault_summary_text(packet)
        self.assertIn("ACTIVE", summary["state"])
        self.assertIn("0x0640", summary["fault1"])
        self.assertIn("0x00C0", summary["vgs2"])

    def test_rolling_plot_buffer_caps_history_and_exports_series(self):
        buffer = RollingPlotBuffer(max_samples=2)
        buffer.append_packet(FOCDataPacket(timestamp=10, speed=1.0, Iq=0.2))
        buffer.append_packet(FOCDataPacket(timestamp=20, speed=2.0, Iq=0.4))
        buffer.append_packet(FOCDataPacket(timestamp=30, speed=3.0, Iq=0.6))

        times, values = buffer.series("speed")
        self.assertEqual(times, [20.0, 30.0])
        self.assertEqual(values, [2.0, 3.0])

        csv_text = format_plot_csv(buffer.export_rows(["speed", "Iq"]))
        self.assertIn("timestamp_ms,speed,Iq", csv_text)
        self.assertIn("30,3.0,0.6", csv_text)

    def test_profile_round_trip_persists_settings_and_presets(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = Path(temp_dir) / "profile.json"
            profile = GuiProfile(
                last_port="COM9",
                baud_rate=460800,
                selected_mode=2,
                log_filters=["ERROR", "TX"],
                current_target=(0.1, 0.8),
                speed_target=20.0,
                position_target=1.57,
                current_pi=LoopTuning(kp=0.2, ki=0.01),
                speed_pi=LoopTuning(kp=1.0, ki=0.1),
                position_pi=LoopTuning(kp=2.0, ki=0.2),
            )
            save_gui_profile(profile_path, profile)
            loaded = load_gui_profile(profile_path)

        self.assertEqual(loaded.last_port, "COM9")
        self.assertEqual(loaded.baud_rate, 460800)
        self.assertEqual(loaded.selected_mode, 2)
        self.assertEqual(loaded.log_filters, ["ERROR", "TX"])
        self.assertEqual(loaded.current_target, (0.1, 0.8))
        self.assertEqual(loaded.speed_pi.kp, 1.0)

    def test_stale_data_detection_uses_threshold(self):
        self.assertFalse(is_data_stale(last_packet_received_at_ms=1500, now_ms=2200, threshold_ms=1000))
        self.assertTrue(is_data_stale(last_packet_received_at_ms=1500, now_ms=2601, threshold_ms=1000))


if __name__ == "__main__":
    unittest.main()
