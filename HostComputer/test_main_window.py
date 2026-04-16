import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from PyQt6.QtWidgets import QApplication

from data_parser import FOCDataPacket
from main_window import HostMainWindow


class TestHostMainWindow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _window(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        return HostMainWindow(profile_path=Path(self.temp_dir.name) / "profile.json")

    def tearDown(self):
        temp_dir = getattr(self, "temp_dir", None)
        if temp_dir is not None:
            temp_dir.cleanup()

    def test_window_has_required_tabs(self):
        window = self._window()
        self.assertEqual(window.tabs.tabText(0), "Debug Panel")
        self.assertEqual(window.tabs.tabText(1), "Identify")
        self.assertEqual(window.tabs.tabText(2), "Advanced Control")
        self.assertEqual(window.tabs.tabText(3), "Loop Parameters")
        self.assertFalse(window.plot_group.isChecked())
        self.assertEqual(window.current_apply_button.text(), "Apply Current Refs")
        self.assertEqual(window.current_pi_apply_button.text(), "Apply Current PI")
        self.assertEqual(window.position_pd_apply_button.text(), "Apply Position PD")

    def test_mode_change_updates_target_label(self):
        window = self._window()
        window.speed_mode_button.setChecked(True)
        window.apply_mode_selection(1)
        self.assertEqual(window.target_label.text(), "Speed (rad/s)")
        self.assertIn("Speed", window.advanced_mode_value.text())

    def test_packet_update_refreshes_fault_and_status_text(self):
        window = self._window()
        packet = FOCDataPacket(
            timestamp=1000,
            angle=45.0,
            speed=3.0,
            Id=0.1,
            Iq=0.2,
            Vd=1.0,
            Vq=2.0,
            Id_ref=0.0,
            Iq_ref=0.5,
            foc_state=4,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            is_fault_active=True,
        )
        window.apply_packet(packet)
        self.assertIn("45.00 deg", window.angle_value.text())
        self.assertIn("ACTIVE", window.fault_state_value.text())
        self.assertIn("0x0640", window.fault_registers_value.text())
        self.assertIn("ACTIVE", window.identify_fault_value.text())

    def test_invalid_advanced_control_input_surfaces_notification(self):
        window = self._window()
        window.update_connection_state(True)
        window.speed_ref_input.setText("abc")
        window.speed_apply_button.click()
        self.assertIn("valid number", window.statusBar().currentMessage())

    def test_advanced_control_apply_buttons_emit_expected_commands(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        window.current_id_input.setText("0.10")
        window.current_iq_input.setText("1.50")
        window.current_apply_button.click()

        window.speed_ref_input.setText("12.0")
        window.speed_apply_button.click()

        window.position_ref_input.setText("-2.5")
        window.position_apply_button.click()

        self.assertIn("CMD:IREF,0.100,1.500\n", commands)
        self.assertIn("CMD:SREF,12.000\n", commands)
        self.assertIn("CMD:PREF,-2.500\n", commands)

    def test_identify_panel_updates_state_and_disable_rules(self):
        window = self._window()
        window.update_connection_state(False)
        self.assertFalse(window.identify_start_page_button.isEnabled())

        window.update_connection_state(True)
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self.assertTrue(window.identify_start_page_button.isEnabled())
        self.assertIn("Connected", window.identify_connection_value.text())
        self.assertIn("Unlocked", window.identify_power_value.text())

    def test_ready_packet_clears_identify_and_enable_latched_button_state(self):
        window = self._window()
        window.update_connection_state(True)
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window.handle_log_line("TX", "CMD:IDENTIFY,1")
        window.apply_packet(FOCDataPacket(foc_state=3, is_fault_active=False))
        self.assertTrue(window.identify_start_page_button.isEnabled())
        self.assertFalse(window.identify_stop_page_button.isEnabled())

        window.handle_log_line("TX", "CMD:ENABLE,1")
        window.apply_packet(FOCDataPacket(foc_state=3, is_fault_active=False))
        self.assertTrue(window.enable_button.isEnabled())
        self.assertFalse(window.disable_button.isEnabled())

    def test_disconnect_clears_stale_runtime_snapshot_before_reconnect(self):
        window = self._window()
        window.update_connection_state(True)
        window.apply_packet(FOCDataPacket(timestamp=1000, angle=45.0, foc_state=4))
        self.assertIn("45.00 deg", window.angle_value.text())

        window.update_connection_state(False)
        self.assertEqual(window.angle_value.text(), "--")
        self.assertEqual(window.data_status_value.text(), "Idle")

        window.update_connection_state(True)
        self.assertEqual(window.angle_value.text(), "--")
        self.assertEqual(window.data_status_value.text(), "Waiting for packets")

    def test_log_filters_and_clear_actions_refresh_log_view(self):
        window = self._window()
        window.handle_log_line("INFO", "hello")
        window.handle_log_line("ERROR", "boom")
        self.assertIn("[INFO] hello", window.log_view.toPlainText())

        window.log_filter_checks["INFO"].setChecked(False)
        self.assertNotIn("[INFO] hello", window.log_view.toPlainText())
        self.assertIn("[ERROR] boom", window.log_view.toPlainText())

        window.clear_log_button.click()
        self.assertEqual(window.log_view.toPlainText(), "")

    def test_quick_actions_emit_expected_sequences(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        window.quick_arm_button.click()
        self.assertEqual(commands[:2], ["CMD:UNLOCK,1\n", "CMD:ENABLE,1\n"])

        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window.handle_log_line("TX", "CMD:ENABLE,1")
        window.quick_safe_stop_button.click()
        self.assertEqual(commands[-2:], ["CMD:ENABLE,0\n", "CMD:UNLOCK,0\n"])


if __name__ == "__main__":
    unittest.main()
