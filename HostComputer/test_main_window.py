import os
import sys
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

    def test_window_has_required_tabs(self):
        window = HostMainWindow()
        self.assertEqual(window.tabs.tabText(0), "Debug Panel")
        self.assertEqual(window.tabs.tabText(1), "Identify")
        self.assertEqual(window.tabs.tabText(2), "Advanced Control")
        self.assertEqual(window.tabs.tabText(3), "PI Parameters")

    def test_mode_change_updates_target_label(self):
        window = HostMainWindow()
        window.speed_mode_button.setChecked(True)
        window.apply_mode_selection(1)
        self.assertEqual(window.target_label.text(), "Speed (rad/s)")

    def test_packet_update_refreshes_fault_and_status_text(self):
        window = HostMainWindow()
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


if __name__ == "__main__":
    unittest.main()
