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

from data_parser import CommandBuilder, FOCDataPacket
from gui_logic import build_adc_noise_command, parse_adc_noise_response
from main_window import HostMainWindow


class TestAdcNoiseGui(unittest.TestCase):
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

    def test_adc_noise_command_builder_formats_sample_count(self):
        self.assertEqual(CommandBuilder.adc_noise_test(4096), "CMD:ADC_NOISE,4096\n")
        self.assertEqual(build_adc_noise_command("128"), "CMD:ADC_NOISE,128\n")

    def test_adc_noise_command_rejects_invalid_sample_count(self):
        with self.assertRaisesRegex(ValueError, "样本数"):
            build_adc_noise_command("abc")
        with self.assertRaisesRegex(ValueError, "16"):
            build_adc_noise_command("1")
        with self.assertRaisesRegex(ValueError, "4096"):
            build_adc_noise_command("5000")

    def test_adc_noise_response_parser_formats_ok_result(self):
        result = parse_adc_noise_response(
            "ADC_NOISE,OK,n=4096,"
            "A:min=2040,max=2050,mean=2045,pp=10,std=2,"
            "B:min=2041,max=2051,mean=2046,pp=10,std=2,"
            "C:min=2039,max=2049,mean=2044,pp=10,std=3,"
            "VBUS:min=1480,max=1488,mean=1484,pp=8,std=1"
        )

        self.assertIsNotNone(result)
        self.assertTrue(result.ok)
        self.assertEqual(result.samples, 4096)
        self.assertIn("A  min=2040", result.display_text)
        self.assertIn("VBUS min=1480", result.display_text)
        self.assertIn("pp=8", result.display_text)

    def test_adc_noise_response_parser_formats_busy_and_error(self):
        started = parse_adc_noise_response("ADC_NOISE,START,n=4096")
        self.assertIsNotNone(started)
        self.assertTrue(started.ok)
        self.assertEqual(started.status, "START")
        self.assertIn("已开始采样", started.display_text)

        busy = parse_adc_noise_response("ADC_NOISE,BUSY,state=4,pwm=1")
        self.assertIsNotNone(busy)
        self.assertFalse(busy.ok)
        self.assertIn("不能测试", busy.display_text)

        err = parse_adc_noise_response("ADC_NOISE,ERR,timeout")
        self.assertIsNotNone(err)
        self.assertFalse(err.ok)
        self.assertIn("失败", err.display_text)

    def test_adc_noise_button_sends_command(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        window.adc_noise_samples_input.setText("4096")
        window.adc_noise_button.click()

        self.assertIn("CMD:ADC_NOISE,4096\n", commands)
        self.assertIn("等待固件返回", window.adc_noise_result_value.text())

    def test_adc_noise_rx_updates_fixed_result_area(self):
        window = self._window()
        window.update_connection_state(True)

        window.handle_log_line(
            "RX",
            "ADC_NOISE,OK,n=64,"
            "A:min=2040,max=2050,mean=2045,pp=10,std=2,"
            "B:min=2041,max=2051,mean=2046,pp=10,std=2,"
            "C:min=2039,max=2049,mean=2044,pp=10,std=3,"
            "VBUS:min=1480,max=1488,mean=1484,pp=8,std=1",
        )

        self.assertIn("n=64", window.adc_noise_result_value.text())
        self.assertIn("A  min=2040", window.adc_noise_result_value.text())
        self.assertIn("VBUS min=1480", window.adc_noise_result_value.text())

    def test_adc_noise_button_is_disabled_while_motor_running(self):
        window = self._window()
        window.update_connection_state(True)
        self.assertTrue(window.adc_noise_button.isEnabled())

        window.apply_packet(FOCDataPacket(foc_state=4, is_fault_active=False))

        self.assertFalse(window.adc_noise_button.isEnabled())


if __name__ == "__main__":
    unittest.main()
