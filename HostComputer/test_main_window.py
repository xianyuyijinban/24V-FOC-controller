import os
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from PyQt6.QtWidgets import QApplication, QScrollArea

from data_parser import CurrentSample, FOCDataPacket, CommandBuilder
from main_window import HostMainWindow


class TestHostMainWindow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _window(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        return HostMainWindow(profile_path=Path(self.temp_dir.name) / "profile.json")

    def _settle_pending(self, window):
        window._state.pending_command = None
        window._state.pending_command_value = None
        window._state.pending_command_sent_at_ms = None
        window._apply_control_enable_state()

    def tearDown(self):
        temp_dir = getattr(self, "temp_dir", None)
        if temp_dir is not None:
            temp_dir.cleanup()

    def test_window_has_required_tabs(self):
        window = self._window()
        self.assertEqual(window.windowTitle(), "FOC 上位机调试工具")
        self.assertEqual(window.tabs.tabText(0), "控制器参数")
        self.assertEqual(window.tabs.tabText(1), "实时波形")
        self.assertEqual(window.tabs.tabText(2), "参数识别")
        self.assertEqual(window.tabs.tabText(3), "高级控制")
        self.assertEqual(window.tabs.tabText(4), "环路参数")
        self.assertEqual(window.current_apply_button.text(), "应用电流给定")
        self.assertEqual(window.current_pi_apply_button.text(), "应用电流环 PI")
        self.assertEqual(window.position_pd_apply_button.text(), "应用位置环 PD")

    def test_tall_configuration_tabs_are_scrollable(self):
        window = self._window()

        self.assertIsInstance(window.tabs.widget(0), QScrollArea)
        self.assertNotIsInstance(window.tabs.widget(1), QScrollArea)
        self.assertIsInstance(window.tabs.widget(2), QScrollArea)
        self.assertIsInstance(window.tabs.widget(3), QScrollArea)
        self.assertIsInstance(window.tabs.widget(4), QScrollArea)

    def test_angle_plot_channel_is_enabled_by_default(self):
        window = self._window()
        self.assertTrue(window.plot_channel_checks["Ia"].isChecked())

    def test_motor_pn_defaults_to_11_for_12v_bench_motor(self):
        window = self._window()

        self.assertEqual(window.motor_pn_input.text(), "11")

    def test_identify_tab_uses_compact_status_and_param_group(self):
        window = self._window()

        self.assertEqual(window.identify_param_group.title(), "识别状态 / 电机参数")
        self.assertFalse(window.identify_motor_base_group.isHidden())
        self.assertFalse(window.identify_encoder_dir_group.isHidden())
        # identify_param_group unchanged: status + params + actions = 3 widgets
        self.assertEqual(window.identify_param_group.layout().count(), 3)
        self.assertIs(window.identify_connection_value.parent(), window.identify_status_panel)
        self.assertIs(window.motor_param_rs_value.parent(), window.identify_params_panel)
        self.assertIs(window.identify_start_page_button.parent(), window.identify_actions_panel)

    def test_identify_param_defaults_use_known_motor_config(self):
        window = self._window()

        self.assertEqual(window.motor_param_rs_value.text(), "8.800")
        self.assertEqual(window.motor_param_ld_value.text(), "0.000500")
        self.assertEqual(window.motor_param_lq_value.text(), "0.000500")
        self.assertEqual(window.motor_param_ke_value.text(), "0.129000")
        self.assertEqual(window.motor_param_pn_value.text(), "11")
        self.assertEqual(window.motor_param_encoder_dir_value.text(), "-1")
        # KNOWN_MOTOR_THETA_OFFSET and KNOWN_MOTOR_THETA_ZERO updated to 0.0 (firmware reports actual values)
        self.assertEqual(window.motor_param_theta_offset_value.text(), "0.000000")
        self.assertEqual(window.motor_param_theta_zero_value.text(), "0.000000")

    def test_vbus_plot_channel_is_available(self):
        window = self._window()
        self.assertIn("Vbus", window.plot_channel_checks)
        self.assertEqual(window.plot_channel_checks["Vbus"].text(), "母线电压")

    def test_mode_change_updates_target_label(self):
        window = self._window()
        window.speed_mode_button.setChecked(True)
        window.apply_mode_selection(1)
        self.assertEqual(window.target_label.text(), "速度 (rad/s)")
        self.assertIn("速度", window.advanced_mode_value.text())

    def test_packet_update_refreshes_fault_and_status_text(self):
        window = self._window()
        packet = FOCDataPacket(
            timestamp=1000,
            encoder_detected=False,
            angle=45.0,
            speed=3.0,
            Id=0.1,
            Iq=0.2,
            Vd=1.0,
            Vq=2.0,
            vbus=11.8,
            Id_ref=0.0,
            Iq_ref=0.5,
            foc_state=4,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            is_fault_active=True,
        )
        window.apply_packet(packet)
        self.assertIn("45.00 deg", window.angle_value.text())
        self.assertIn("故障激活", window.fault_state_value.text())
        self.assertIn("0x0640", window.fault_registers_value.text())
        self.assertIn("故障激活", window.identify_fault_value.text())
        self.assertIn("未检测到", window.identify_encoder_value.text())
        self.assertIn("Vbus 11.80 V", window.voltages_value.text())

    def test_packet_update_marks_fault_when_only_application_fault_is_present(self):
        window = self._window()
        packet = FOCDataPacket(
            timestamp=1000,
            encoder_detected=True,
            angle=45.0,
            speed=3.0,
            Id=0.1,
            Iq=0.2,
            Vd=1.0,
            Vq=2.0,
            vbus=11.8,
            foc_state=5,
            fault_status1=0x0000,
            vgs_status2=0x0000,
            is_fault_active=False,
            app_fault_code=7,
        )
        window.apply_packet(packet)
        self.assertIn("故障激活", window.fault_state_value.text())
        self.assertIn("ADC采样", window.fault_state_value.text())
        self.assertIn("故障激活", window.identify_fault_value.text())

    def test_invalid_advanced_control_input_surfaces_notification(self):
        window = self._window()
        window.update_connection_state(True)
        window.speed_ref_input.setText("abc")
        window.speed_apply_button.clicked.emit()
        self.assertIn("有效数字", window.statusBar().currentMessage())

    def test_vbus_limit_apply_emits_expected_command(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        window.vbus_uv_input.setText("9.0")
        window.vbus_ov_input.setText("15.0")
        window.vbus_limit_apply_button.click()

        self.assertIn("CMD:VBUS_LIMIT,9.000,15.000\n", commands)

    def test_vbus_limit_apply_marks_waiting_for_firmware_confirmation(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        window.vbus_uv_input.setText("9.0")
        window.vbus_ov_input.setText("16.0")
        window.vbus_limit_apply_button.click()

        self.assertIn("CMD:VBUS_LIMIT,9.000,16.000\n", commands)
        self.assertIn("等待固件确认", window.vbus_limit_actual_value.text())

    def test_vbus_limit_apply_confirms_when_firmware_echoes_requested_limits(self):
        window = self._window()
        window.update_connection_state(True)

        window.vbus_uv_input.setText("9.0")
        window.vbus_ov_input.setText("16.0")
        window.vbus_limit_apply_button.click()

        window.apply_packet(
            FOCDataPacket(
                foc_state=3,
                is_fault_active=False,
                undervoltage_limit=9.0,
                overvoltage_limit=16.0,
            )
        )

        self.assertIn("UV 9.000 V", window.vbus_limit_actual_value.text())
        self.assertIn("OV 16.000 V", window.vbus_limit_actual_value.text())
        self.assertIn("固件确认", window.statusBar().currentMessage())

    def test_vbus_limit_apply_warns_when_firmware_keeps_old_limits(self):
        window = self._window()
        window.update_connection_state(True)

        window.vbus_uv_input.setText("9.0")
        window.vbus_ov_input.setText("16.0")
        window.vbus_limit_apply_button.click()
        window._pending_vbus_limit_requested_at_ms = 0

        with mock.patch("main_window.time.monotonic", return_value=2.0):
            window.apply_packet(
                FOCDataPacket(
                    foc_state=5,
                    is_fault_active=True,
                    undervoltage_limit=18.0,
                    overvoltage_limit=28.0,
                    app_fault_code=3,
                )
            )

        self.assertIn("未生效", window.vbus_limit_actual_value.text())
        self.assertIn("18.000 V", window.vbus_limit_actual_value.text())
        self.assertIn("未生效", window.statusBar().currentMessage())

    def test_vbus_limit_apply_is_disabled_while_motor_is_active(self):
        window = self._window()
        window.update_connection_state(True)
        self.assertTrue(window.vbus_limit_apply_button.isEnabled())

        window.apply_packet(FOCDataPacket(foc_state=4, is_fault_active=False))
        self.assertFalse(window.vbus_limit_apply_button.isEnabled())

        window.apply_packet(FOCDataPacket(foc_state=3, is_fault_active=False))
        self.assertTrue(window.vbus_limit_apply_button.isEnabled())

    def test_vbus_limit_actual_label_updates_from_packet_telemetry(self):
        window = self._window()
        window.apply_packet(
            FOCDataPacket(
                foc_state=3,
                is_fault_active=False,
                undervoltage_limit=9.0,
                overvoltage_limit=15.0,
            )
        )

        self.assertIn("UV 9.000 V", window.vbus_limit_actual_value.text())
        self.assertIn("OV 15.000 V", window.vbus_limit_actual_value.text())

    def test_vbus_limit_actual_label_shows_legacy_firmware_hint(self):
        window = self._window()
        window.apply_packet(
            FOCDataPacket(
                foc_state=3,
                is_fault_active=False,
                vbus=11.98,
            )
        )

        self.assertIn("未上报阈值", window.vbus_limit_actual_value.text())

    def test_vbus_limit_actual_label_shows_waiting_hint_before_first_packet(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)

        self.assertIn("等待固件回传", window.vbus_limit_actual_value.text())
        self.assertEqual(commands, ["CMD:FAULT_DETAIL\n"])

    def test_identify_panel_updates_motor_param_placeholders_from_packet(self):
        window = self._window()
        window.apply_packet(
            FOCDataPacket(
                foc_state=3,
                is_fault_active=False,
                motor_identified=True,
                motor_param_rs=8.8,
                motor_param_ld=0.0005,
                motor_param_lq=0.0005,
                motor_param_ke=0.129,
                motor_param_pn=11,
                motor_param_encoder_dir=-1,
            )
        )

        self.assertEqual(window.motor_param_rs_value.text(), "8.800")
        self.assertEqual(window.motor_param_ld_value.text(), "0.000500")
        self.assertEqual(window.motor_param_lq_value.text(), "0.000500")
        self.assertEqual(window.motor_param_ke_value.text(), "0.129000")
        self.assertEqual(window.motor_param_pn_value.text(), "11")
        self.assertEqual(window.motor_param_encoder_dir_value.text(), "-1")

    def test_identify_complete_requests_parameter_snapshot(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.identify_active = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:IDENTIFY,1")
        self._settle_pending(window)

        window.apply_packet(FOCDataPacket(foc_state=3, is_fault_active=False, motor_identified=True))

        self.assertEqual(commands, ["CMD:FAULT_DETAIL\n"])

    def test_motion_target_commands_require_unlocked_and_enabled_motor(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()

        window.position_ref_input.setText("90")
        window.position_apply_button.clicked.emit()

        self.assertEqual(commands, [])
        self.assertIn("先解锁并使能", window.statusBar().currentMessage())

    def test_pending_command_disables_advancing_controls(self):
        window = self._window()
        window.update_connection_state(True)
        window._state.power_unlocked = True
        window._state.pending_command = "ENABLE"

        window._apply_control_enable_state()

        self.assertFalse(window.enable_button.isEnabled())
        self.assertFalse(window.quick_arm_button.isEnabled())
        self.assertFalse(window.speed_apply_button.isEnabled())
        self.assertFalse(window.app_mode_set_button.isEnabled())
        self.assertTrue(window.lock_button.isEnabled())
        self.assertTrue(window.clear_fault_button.isEnabled())

    def test_dispatch_command_blocks_duplicate_enable_while_pending(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True
        window._state.pending_command = "ENABLE"

        window._dispatch_command(CommandBuilder.enable_motor(True))

        self.assertEqual(commands, [])
        self.assertIn("ENABLE", window.statusBar().currentMessage())

    def test_dispatch_command_allows_safe_fallback_while_pending(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True
        window._state.pending_command = "ENABLE"

        window._dispatch_command(CommandBuilder.enable_motor(False))

        self.assertEqual(commands, [CommandBuilder.enable_motor(False)])

    def test_advanced_control_apply_buttons_emit_expected_commands(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window._state.motor_enabled = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:ENABLE,1")
        self._settle_pending(window)

        window.current_id_input.setText("0.10")
        window.current_iq_input.setText("1.50")
        window.current_apply_button.clicked.emit()

        window.speed_ref_input.setText("5.0")
        window.speed_apply_button.clicked.emit()

        window.position_ref_input.setText("90")
        window.position_apply_button.clicked.emit()

        self.assertIn("CMD:IREF,0.100,1.500\n", commands)
        self.assertIn("CMD:SREF,5.000\n", commands)
        self.assertIn("CMD:PREF,1.571\n", commands)

    def test_current_stream_toggle_starts_recommended_binary_scope(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)

        window.cur_stream_group.setChecked(True)

        self.assertTrue(window._scope_enabled)
        self.assertTrue(window.scope_toggle_button.isChecked())
        self.assertEqual(window.scope_window_combo.currentText(), "1s")
        self.assertEqual(window.cur_mode_combo.currentIndex(), 2)
        self.assertIn(CommandBuilder.telem_cur_bin(1000), commands)

    def test_current_stream_mode_change_starts_scope_when_enabled(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)

        window.cur_stream_group.blockSignals(True)
        window.cur_stream_group.setChecked(True)
        window.cur_stream_group.blockSignals(False)
        window.cur_mode_combo.setCurrentIndex(1)

        self.assertTrue(window._scope_enabled)
        self.assertIn(CommandBuilder.telem_cur_ascii(200), commands)

    def test_current_stream_plot_data_is_scaled_to_milliamps(self):
        window = self._window()

        class FakeRing:
            def __init__(self):
                self._samples = [
                    CurrentSample(seq=10, tick_ms=1000, ia=0.123, ib=-0.045, ic=-0.078),
                    CurrentSample(seq=11, tick_ms=1001, ia=0.125, ib=-0.047, ic=-0.079),
                ]

            def get_all(self):
                return list(self._samples)

        class FakeWorker:
            def __init__(self):
                self._ring = FakeRing()

            def current_ring(self):
                return self._ring

        window._serial_worker = FakeWorker()
        window._scope_start_seq = 10
        window._scope_start_tick_ms = 1000

        _, series = window._get_current_stream_plot_data(["Ia", "Ib", "Ic"])

        self.assertEqual(series["Ia"], [123.0, 125.0])
        self.assertEqual(series["Ib"], [-45.0, -47.0])
        self.assertEqual(series["Ic"], [-78.0, -79.0])

    def test_current_stream_diagnostics_report_window_metrics(self):
        samples = [
            CurrentSample(seq=1, tick_ms=1000, ia=0.100, ib=-0.040, ic=-0.060, id=0.010, iq=0.020),
            CurrentSample(seq=2, tick_ms=1001, ia=0.120, ib=-0.050, ic=-0.070, id=0.020, iq=0.030),
        ]

        text = HostMainWindow._format_current_diagnostics(samples)

        self.assertIn("diag n=2", text)
        self.assertIn("sumABC mean=+0.0mA", text)
        self.assertIn("Id mean=+15.0mA", text)
        self.assertIn("Iq mean=+25.0mA", text)
        self.assertIn("phase p-p=20/10/10mA", text)

    def test_scope_ignores_zero_timestamp_as_time_origin(self):
        window = self._window()
        window._scope_enabled = True

        window.apply_packet(FOCDataPacket(timestamp=0, angle=1.0))
        self.assertIsNone(window._scope_start_timestamp)

        window.apply_packet(FOCDataPacket(timestamp=10000, angle=2.0))
        self.assertEqual(window._scope_start_timestamp, 10000.0)

    def test_unidentified_enable_confirms_stall_mode_before_enabling(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.motor_identified = False
        window._confirm_stall_mode_enable = lambda: True

        window.enable_button.click()

        self.assertEqual(
            commands,
            ["CMD:STALL_MODE,1\n", "CMD:ENABLE,1\n"],
        )

    def test_encoder_offline_enable_confirms_stall_mode_before_enabling(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.motor_identified = True
        window._state.encoder_detected = False
        window._confirm_stall_mode_enable = lambda: True

        window.enable_button.click()

        self.assertEqual(
            commands,
            ["CMD:STALL_MODE,1\n", "CMD:ENABLE,1\n"],
        )

    def test_encoder_unknown_enable_confirms_stall_mode_before_enabling(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.motor_identified = True
        window._state.encoder_detected = None
        window._confirm_stall_mode_enable = lambda: True

        window.enable_button.clicked.emit()

        self.assertEqual(
            commands,
            ["CMD:STALL_MODE,1\n", "CMD:ENABLE,1\n"],
        )

    def test_identify_panel_exposes_stall_open_loop_runtime_state(self):
        window = self._window()
        self.assertEqual(window.identify_stall_open_loop_value.text(), "未激活")

    def test_unidentified_enable_cancel_does_not_emit_commands(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.motor_identified = False
        window._confirm_stall_mode_enable = lambda: False

        window.enable_button.click()

        self.assertEqual(commands, [])

    def test_identify_panel_updates_state_and_disable_rules(self):
        window = self._window()
        window.update_connection_state(False)
        self.assertFalse(window.identify_start_page_button.isEnabled())

        window.update_connection_state(True)
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        self.assertTrue(window.identify_start_page_button.isEnabled())
        self.assertIn("已连接", window.identify_connection_value.text())
        self.assertIn("已解锁", window.identify_power_value.text())

    def test_ready_packet_clears_identify_and_enable_latched_button_state(self):
        window = self._window()
        window.update_connection_state(True)
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.identify_active = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:IDENTIFY,1")
        self._settle_pending(window)
        window.apply_packet(FOCDataPacket(foc_state=3, is_fault_active=False))
        self.assertTrue(window.identify_start_page_button.isEnabled())
        self.assertFalse(window.identify_stop_page_button.isEnabled())

        window._state.motor_enabled = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:ENABLE,1")
        self._settle_pending(window)
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
        self.assertEqual(window.data_status_value.text(), "空闲")

        window.update_connection_state(True)
        self.assertEqual(window.angle_value.text(), "--")
        self.assertEqual(window.data_status_value.text(), "等待数据包")

    def test_session_status_reports_rx_activity_when_packets_stop_refreshing(self):
        window = self._window()
        window.update_connection_state(True)
        window._state.last_packet_received_at_ms = 1000
        window._last_rx_activity_at_ms = 2400

        with mock.patch("main_window.time.monotonic", return_value=2.6):
            window._refresh_session_status()

        self.assertIn("串口活跃", window.data_status_value.text())

    def test_phase_current_only_packet_updates_plot_without_overwriting_status(self):
        window = self._window()
        window.apply_packet(FOCDataPacket(timestamp=1000, angle=45.0, foc_state=4, speed=2.0))

        window.apply_packet(FOCDataPacket(timestamp=1005, Ia=0.4, Ib=0.5, Ic=-0.9, phase_current_only=True))

        self.assertIn("45.00 deg", window.angle_value.text())
        self.assertEqual(window._state.foc_state, 4)
        times, ia_values = window._plot_buffer.series("Ia")
        self.assertEqual(times[-2:], [1000.0, 1005.0])
        self.assertEqual(ia_values[-1], 0.4)

    def test_phase_current_packets_defer_plot_refresh_to_throttle_timer(self):
        window = self._window()
        window.tabs.setCurrentIndex(1)

        with mock.patch.object(window, "_refresh_plot") as refresh_plot:
            for index in range(200):
                window.apply_packet(
                    FOCDataPacket(
                        timestamp=1000 + index * 5,
                        Ia=0.1,
                        Ib=-0.2,
                        Ic=0.1,
                        phase_current_only=True,
                    )
                )

        refresh_plot.assert_not_called()
        self.assertTrue(window._plot_refresh_pending)

        with mock.patch.object(window, "_refresh_plot") as refresh_plot:
            window._flush_pending_plot_refresh()

        refresh_plot.assert_called_once()
        self.assertFalse(window._plot_refresh_pending)

    def test_runtime_packets_do_not_append_rx_log_lines(self):
        window = self._window()
        baseline = len(window._log_entries)

        window.apply_packet(FOCDataPacket(timestamp=1000, foc_state=3, vbus=12.0, speed=0.0, is_fault_active=False))
        window.apply_packet(FOCDataPacket(timestamp=1005, Ia=0.1, Ib=-0.2, Ic=0.1, phase_current_only=True))

        self.assertEqual(len(window._log_entries), baseline)

    def test_handle_log_line_batches_log_render_until_flush_timer(self):
        window = self._window()

        with mock.patch.object(window, "_render_logs") as render_logs:
            window.handle_log_line("INFO", "hello")
            window.handle_log_line("ERROR", "boom")

        render_logs.assert_not_called()
        self.assertTrue(window._log_refresh_pending)

        with mock.patch.object(window, "_render_logs") as render_logs:
            window._flush_pending_log_refresh()

        render_logs.assert_called_once()
        self.assertFalse(window._log_refresh_pending)

    def test_session_status_remains_waiting_without_rx_or_packet_activity(self):
        window = self._window()
        window.update_connection_state(True)
        window._state.last_packet_received_at_ms = None
        window._last_rx_activity_at_ms = None
        window._refresh_session_status()
        self.assertEqual(window.data_status_value.text(), "等待数据包")

    def test_log_filters_and_clear_actions_refresh_log_view(self):
        window = self._window()
        window.handle_log_line("INFO", "hello")
        window.handle_log_line("ERROR", "boom")
        window._flush_pending_log_refresh()
        self.assertIn("[信息] hello", window.log_view.toPlainText())

        window.log_filter_checks["INFO"].setChecked(False)
        self.assertNotIn("[信息] hello", window.log_view.toPlainText())
        self.assertIn("[错误] boom", window.log_view.toPlainText())

        window.clear_log_button.click()
        self.assertEqual(window.log_view.toPlainText(), "")

    def test_fault_log_panel_keeps_fault_entries_separate_and_persistent(self):
        window = self._window()
        window.handle_log_line("RX", "runtime sample")
        window._flush_pending_log_refresh()
        self.assertIn("[接收] runtime sample", window.log_view.toPlainText())
        self.assertEqual(window.fault_log_view.toPlainText(), "")

        fault_packet = FOCDataPacket(
            timestamp=3456,
            foc_state=5,
            fault_flags=0x20,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            encoder_detected=False,
            is_fault_active=True,
            raw_text="F,3456,5,0x00000020,1,0,0x0640,0x00C0,0xFFFF",
        )
        window.apply_packet(fault_packet)
        window.apply_packet(fault_packet)

        fault_log = window.fault_log_view.toPlainText()
        self.assertIn("故障摘要", fault_log)
        self.assertEqual(fault_log.count("故障摘要"), 1)
        self.assertNotIn("runtime sample", fault_log)

    def test_fault_log_clear_is_independent_from_serial_log_clear(self):
        window = self._window()
        window.handle_log_line("INFO", "hello")
        window.handle_log_line("ERROR", "故障：母线过压")
        window._flush_pending_log_refresh()

        self.assertIn("[信息] hello", window.log_view.toPlainText())
        self.assertIn("母线过压", window.fault_log_view.toPlainText())

        window.clear_log_button.click()
        self.assertEqual(window.log_view.toPlainText(), "")
        self.assertIn("母线过压", window.fault_log_view.toPlainText())

        window.clear_fault_log_button.click()
        self.assertEqual(window.fault_log_view.toPlainText(), "")

    def test_quick_actions_emit_expected_sequences(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()

        window._state.motor_identified = False
        window._confirm_stall_mode_enable = lambda: True
        window.quick_arm_button.click()
        self.assertEqual(
            commands[:3],
            ["CMD:UNLOCK,1\n", "CMD:STALL_MODE,1\n", "CMD:ENABLE,1\n"],
        )

        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window._state.motor_enabled = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:ENABLE,1")
        self._settle_pending(window)
        window.quick_safe_stop_button.click()
        self.assertEqual(commands[-2:], ["CMD:ENABLE,0\n", "CMD:UNLOCK,0\n"])

    def test_power_ack_payload_updates_buttons_and_status_text(self):
        window = self._window()
        window.update_connection_state(True)

        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window.handle_log_line("RX", "UNLOCK,OK,1")

        self.assertTrue(window._state.power_unlocked)
        self.assertFalse(window.unlock_button.isEnabled())
        self.assertTrue(window.lock_button.isEnabled())
        self.assertTrue(window.enable_button.isEnabled())
        self.assertEqual(window.power_status_label.text(), "已解锁 | 未使能")
        self.assertEqual(window.app_power_status.text(), "已解锁 | 未使能")

        window.handle_log_line("TX", "CMD:ENABLE,1")
        window.handle_log_line("RX", "ENABLE,OK,1")

        self.assertTrue(window._state.motor_enabled)
        self.assertFalse(window.enable_button.isEnabled())
        self.assertTrue(window.disable_button.isEnabled())
        self.assertEqual(window.power_status_label.text(), "已解锁 | 已使能")
        self.assertEqual(window.app_power_status.text(), "已解锁 | 已使能")

    def test_app_power_buttons_use_enable_state_machine(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()

        self.assertTrue(window.app_arm_button.isEnabled())
        self.assertTrue(window.quick_arm_button.isEnabled())
        self.assertFalse(window.app_enable_button.isEnabled())

        window._state.motor_identified = True
        window._state.encoder_detected = True
        window.app_arm_button.clicked.emit()
        # V1.2+: sequence emits APP_MODE first; rest waits for ACK advancement
        self.assertEqual(commands, ["CMD:APP_MODE,RAW"])

        commands.clear()
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window.enable_button.clicked.emit()
        window.app_enable_button.clicked.emit()
        # Top enable stays legacy; app enable emits APP_MODE first
        self.assertEqual(
            commands,
            ["CMD:STALL_MODE,1\n", "CMD:ENABLE,1\n", "CMD:APP_MODE,RAW"],
        )

    def test_app_enable_preserves_stall_mode_confirmation_sequence(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()

        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)
        window._state.motor_identified = False
        window._confirm_stall_mode_enable = lambda: True

        window.app_enable_button.clicked.emit()

    def test_joint_position_target_reasserts_app_mode_before_pref(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        window._state.motor_enabled = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:ENABLE,1")
        self._settle_pending(window)
        commands.clear()
        window._state.app_mode = "JOINT_POS"
        window._state.control_mode = 1
        window.joint_pos_target_spin.setValue(20.0)

        window.joint_pos_start_btn.clicked.emit()
        self.assertEqual(commands, ["CMD:APP_MODE,JOINT_POS"])

    def test_detent_cfg_reasserts_app_mode_even_when_host_thinks_detent(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.app_mode = "DETENT"
        window.detent_count_spin.setValue(12)
        window.detent_strength_spin.setValue(1.0)
        window.detent_width_spin.setValue(0.13)
        window.detent_limit_spin.setValue(0.25)

        window.detent_cfg_set_btn.clicked.emit()


    def test_detent_preset_reasserts_app_mode_even_when_host_thinks_detent(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        window._state.app_mode = "DETENT"

        window.detent_preset_std_btn.clicked.emit()
        self.assertEqual(commands, ["CMD:APP_MODE,DETENT"])
        self.assertEqual(commands, ["CMD:APP_MODE,DETENT"])

    def test_hold_panel_status_uses_runtime_telemetry(self):
        window = self._window()
        window.update_connection_state(True)

        window.handle_log_line("RX", "APP_MODE,OK,HOLD")
        window.apply_packet(FOCDataPacket(foc_state=4, angle=295.7, speed=0.02, control_mode=1))

        self.assertIn("产品：位置保持", window.advanced_mode_value.text())
        self.assertIn("底层：位置", window.advanced_mode_value.text())
        self.assertEqual(window.hold_angle_label.text(), "295.70 deg")
        self.assertEqual(window.hold_speed_label.text(), "0.02 rad/s")

    # ── V1.2 Chinese display tests ──

    def test_app_mode_combo_shows_chinese_labels_with_protocol_userdata(self):
        window = self._window()
        combo = window.app_mode_combo
        self.assertEqual(combo.count(), 6)
        labels = [combo.itemText(i) for i in range(combo.count())]
        self.assertEqual(labels, ["原始控制", "关节位置", "云台速度", "位置保持", "弹簧阻尼", "卡点旋钮"])
        tokens = [combo.itemData(i) for i in range(combo.count())]
        self.assertEqual(tokens, ["RAW", "JOINT_POS", "GIMBAL_SPEED", "HOLD", "SPRING_DAMPER", "DETENT"])

    def test_app_mode_set_button_uses_currentData(self):
        window = self._window()
        commands = []
        window.command_requested.connect(commands.append)
        window.update_connection_state(True)
        commands.clear()
        idx = window.app_mode_combo.findData("HOLD")
        window.app_mode_combo.setCurrentIndex(idx)
        window.app_mode_set_button.clicked.emit()
        self.assertIn("CMD:APP_MODE,HOLD\n", commands)

    def test_app_mode_ack_reselects_combo_via_findData(self):
        window = self._window()
        window.update_connection_state(True)
        self.assertEqual(window.app_mode_combo.currentIndex(), 0)
        window.handle_log_line("RX", "APP_MODE,OK,HOLD")
        self.assertEqual(window.app_mode_combo.currentText(), "位置保持")
        self.assertEqual(window.app_mode_combo.currentData(), "HOLD")

    def test_app_control_status_shows_chinese_format_raw(self):
        window = self._window()
        window._state.app_mode = "RAW"
        window._state.control_mode = 0
        text = window._app_control_status_text()
        self.assertIn("产品：原始控制", text)
        self.assertIn("底层：力矩", text)

    def test_app_control_status_shows_chinese_format_non_raw(self):
        window = self._window()
        window._state.app_mode = "SPRING_DAMPER"
        text = window._app_control_status_text()
        self.assertIn("产品：弹簧阻尼", text)
        self.assertIn("底层：位置", text)

    def test_enable_and_arm_buttons_disable_during_fault_or_identify(self):
        window = self._window()
        window.update_connection_state(True)
        window._state.power_unlocked = True  # V1.2: simulate ACK
        window.handle_log_line("TX", "CMD:UNLOCK,1")
        self._settle_pending(window)

        window._state.identify_active = True
        window._state.fault_active = False
        window._state.motor_enabled = False
        window._apply_control_enable_state()
        self.assertFalse(window.enable_button.isEnabled())
        self.assertFalse(window.quick_arm_button.isEnabled())
        self.assertFalse(window.app_enable_button.isEnabled())
        self.assertFalse(window.app_arm_button.isEnabled())

        window._state.identify_active = False
        window._state.fault_active = True
        window._apply_control_enable_state()
        self.assertFalse(window.enable_button.isEnabled())
        self.assertFalse(window.quick_arm_button.isEnabled())
        self.assertFalse(window.app_enable_button.isEnabled())
        self.assertFalse(window.app_arm_button.isEnabled())


if __name__ == "__main__":
    unittest.main()
