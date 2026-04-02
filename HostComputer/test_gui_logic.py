import sys
import unittest
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from data_parser import FOCDataPacket
from gui_logic import (
    connection_command_state,
    fault_summary_text,
    mode_target_label,
    packet_snapshot,
)


class TestGuiLogic(unittest.TestCase):
    def test_mode_target_label_matches_control_mode(self):
        self.assertEqual(mode_target_label(0), "Iq_ref (A)")
        self.assertEqual(mode_target_label(1), "Speed (rad/s)")
        self.assertEqual(mode_target_label(2), "Position (rad)")

    def test_connection_command_state_requires_link(self):
        state = connection_command_state(is_connected=False)
        self.assertFalse(state["can_unlock"])
        self.assertFalse(state["can_send_target"])

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


if __name__ == "__main__":
    unittest.main()
