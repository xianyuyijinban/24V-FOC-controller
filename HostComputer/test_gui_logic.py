import sys
import tempfile
import unittest
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from data_parser import FOCDataPacket, AckResult
from gui_logic import (
    FOC_STATE_FAULT,
    FOC_STATE_PARAM_IDENTIFY,
    FOC_STATE_READY,
    FOC_STATE_RUNNING,
    GuiProfile,
    HostAppState,
    LoopTuning,
    PLOT_CHANNELS,
    PositionLoopTuning,
    RollingPlotBuffer,
    apply_ack_effects,
    apply_command_effects,
    apply_packet_effects,
    build_current_ref_command,
    build_app_mode_command,
    build_loop_gain_command,
    build_motor_pn_command,
    build_position_ref_command,
    build_speed_ref_command,
    build_vbus_limit_command,
    button_enable_state,
    can_dispatch_command,
    can_edit_vbus_limits,
    fault_error_log_entry,
    fault_packet_log_entry,
    fault_summary_text,
    format_plot_csv,
    is_data_stale,
    load_gui_profile,
    mode_target_label,
    packet_snapshot,
    parse_app_mode_response,
    parse_float_field,
    save_gui_profile,
    stall_mode_confirmation_text,
    should_confirm_stall_mode_enable,
)


class TestGuiLogic(unittest.TestCase):
    def test_gui_profile_defaults_to_1000000_baud(self):
        profile = GuiProfile()
        self.assertEqual(profile.baud_rate, 1000000)
        self.assertEqual(profile.undervoltage_limit, 18.0)
        self.assertEqual(profile.overvoltage_limit, 30.0)
        self.assertEqual(profile.motor_pn, 11)

    def test_mode_target_label_matches_control_mode(self):
        self.assertEqual(mode_target_label(0), "Iq_ref (A)")
        self.assertEqual(mode_target_label(1), "速度 (rad/s)")
        self.assertEqual(mode_target_label(2), "位置 (deg)")

    def test_parse_float_field_rejects_empty_input(self):
        with self.assertRaisesRegex(ValueError, "速度.*必填"):
            parse_float_field("", "速度")

    def test_parse_app_mode_response_accepts_set_and_query_forms(self):
        self.assertEqual(
            parse_app_mode_response("APP_MODE,OK,JOINT_POS"),
            {"mode": "JOINT_POS", "ctrl": None},
        )
        self.assertEqual(
            parse_app_mode_response("APP_MODE,OK,JOINT_POS (ctrl_mode=2)"),
            {"mode": "JOINT_POS", "ctrl": 2},
        )

    def test_reference_dispatch_routes_to_expected_commands(self):
        self.assertEqual(build_current_ref_command("0.25", "1.5"), "CMD:IREF,0.250,1.500\n")
        self.assertEqual(build_speed_ref_command("5.0"), "CMD:SREF,5.000\n")
        self.assertEqual(build_position_ref_command("180"), "CMD:PREF,3.142\n")
        self.assertEqual(build_app_mode_command("DETENT"), "CMD:APP_MODE,DETENT\n")
        self.assertEqual(build_vbus_limit_command("9.0", "15.5"), "CMD:VBUS_LIMIT,9.000,15.500\n")
        self.assertEqual(build_motor_pn_command("7"), "CMD:MOTOR_PN,7\n")

    def test_position_ref_rejects_out_of_single_turn_degree_range(self):
        with self.assertRaisesRegex(ValueError, "位置.*>= 0"):
            build_position_ref_command("-1")
        with self.assertRaisesRegex(ValueError, "位置.*<= 360"):
            build_position_ref_command("361")

    def test_motor_pn_command_validates_integer_range(self):
        with self.assertRaisesRegex(ValueError, "极对数.*整数"):
            build_motor_pn_command("7.5")
        with self.assertRaisesRegex(ValueError, "极对数.*>= 1"):
            build_motor_pn_command("0")
        with self.assertRaisesRegex(ValueError, "极对数.*<= 50"):
            build_motor_pn_command("51")

    def test_pi_dispatch_routes_to_expected_commands(self):
        self.assertEqual(build_loop_gain_command("current", "0.100", "0.002"), "CMD:PI_CURRENT,0.100000,0.002000\n")
        self.assertEqual(build_loop_gain_command("speed", "1.250", "0.015"), "CMD:PI_SPEED,1.250000,0.015000\n")
        self.assertEqual(build_loop_gain_command("position", "3.000", "0.250"), "CMD:PD_POS,3.000000,0.250000\n")

    def test_button_enable_state_tracks_connection_and_power_workflow(self):
        disconnected = button_enable_state(HostAppState(is_connected=False))
        self.assertFalse(disconnected["can_unlock"])
        self.assertFalse(disconnected["can_send_target"])
        self.assertFalse(disconnected["can_identify_start"])

        connected = button_enable_state(HostAppState(is_connected=True, power_unlocked=False))
        self.assertTrue(connected["can_unlock"])
        self.assertFalse(connected["can_enable"])
        self.assertTrue(connected["can_quick_arm"])
        self.assertFalse(connected["can_identify_start"])

        armed = button_enable_state(
            HostAppState(is_connected=True, power_unlocked=True, motor_enabled=True, identify_active=True)
        )
        self.assertFalse(armed["can_unlock"])
        self.assertTrue(armed["can_lock"])
        self.assertFalse(armed["can_enable"])
        self.assertFalse(armed["can_quick_arm"])
        self.assertTrue(armed["can_disable"])
        self.assertFalse(armed["can_identify_start"])
        self.assertTrue(armed["can_identify_stop"])

    def test_button_enable_state_blocks_advancing_actions_while_pending(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_enabled=False,
            pending_command="ENABLE",
        )
        button_state = button_enable_state(state)
        self.assertFalse(button_state["can_unlock"])
        self.assertTrue(button_state["can_lock"])
        self.assertFalse(button_state["can_enable"])
        self.assertFalse(button_state["can_quick_arm"])
        self.assertFalse(button_state["can_send_target"])
        self.assertFalse(button_state["app_mode_selector"])
        self.assertFalse(button_state["motion_target"])

    def test_can_dispatch_command_blocks_pending_advancement_but_allows_fallback(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            pending_command="ENABLE",
        )
        allowed, reason = can_dispatch_command(state, "CMD:ENABLE,1\n")
        self.assertFalse(allowed)
        self.assertIn("ENABLE", reason)

        allowed, _ = can_dispatch_command(state, "CMD:ENABLE,0\n")
        self.assertTrue(allowed)

        allowed, _ = can_dispatch_command(state, "CMD:CLEAR_FAULT\n")
        self.assertTrue(allowed)

    def test_can_dispatch_command_blocks_fault_drive_commands(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            fault_active=True,
        )
        self.assertFalse(can_dispatch_command(state, "CMD:ENABLE,1\n")[0])
        self.assertFalse(can_dispatch_command(state, "CMD:SREF,0.500\n")[0])
        self.assertTrue(can_dispatch_command(state, "CMD:CLEAR_FAULT\n")[0])
        self.assertTrue(can_dispatch_command(state, "CMD:UNLOCK,0\n")[0])

    def test_can_dispatch_command_requires_enabled_motor_for_targets(self):
        state = HostAppState(is_connected=True, power_unlocked=True, motor_enabled=False)
        self.assertFalse(can_dispatch_command(state, "CMD:SREF,0.500\n")[0])
        state.motor_enabled = True
        self.assertTrue(can_dispatch_command(state, "CMD:SREF,0.500\n")[0])

    def test_apply_command_effects_sets_pending_not_state(self):
        """V1.2: key commands set pending_command, not direct state booleans."""
        state = HostAppState(is_connected=True)
        apply_command_effects(state, "CMD:UNLOCK,1")
        self.assertFalse(state.power_unlocked)  # NOT set yet
        self.assertEqual(state.pending_command, "UNLOCK")
        self.assertEqual(state.pending_command_value, "1")

        apply_command_effects(state, "CMD:ENABLE,1")
        self.assertFalse(state.motor_enabled)  # NOT set yet
        self.assertEqual(state.pending_command, "ENABLE")
        self.assertEqual(state.pending_command_value, "1")

        apply_command_effects(state, "CMD:IDENTIFY,1")
        self.assertFalse(state.identify_active)  # NOT set yet
        self.assertEqual(state.pending_command, "IDENTIFY")
        self.assertEqual(state.pending_command_value, "1")

        apply_command_effects(state, "CMD:STALL_MODE,1")
        self.assertFalse(state.stall_mode_armed)  # NOT set yet
        self.assertEqual(state.pending_command, "STALL_MODE")
        self.assertEqual(state.pending_command_value, "1")

    def test_apply_command_effects_lock_is_destructive_immediate(self):
        """V1.2: UNLOCK,0 cascades immediately because destructive."""
        state = HostAppState(is_connected=True, power_unlocked=True, motor_enabled=True,
                             identify_active=True, stall_mode_armed=True,
                             stall_open_loop_active=True)
        apply_command_effects(state, "CMD:UNLOCK,0")
        self.assertFalse(state.power_unlocked)
        self.assertFalse(state.motor_enabled)
        self.assertFalse(state.identify_active)
        self.assertFalse(state.stall_mode_armed)
        self.assertFalse(state.stall_open_loop_active)
        self.assertEqual(state.pending_command_value, "0")

    def test_apply_ack_effects_updates_state_on_ok(self):
        state = HostAppState(is_connected=True, pending_command="UNLOCK")
        ack = AckResult(command="UNLOCK", ok=True, raw="UNLOCK,OK")
        apply_ack_effects(state, ack, now_ms=1000)
        self.assertTrue(state.power_unlocked)
        self.assertIsNone(state.pending_command)
        self.assertIsNone(state.pending_command_value)
        self.assertEqual(state.last_ack, "UNLOCK,OK")
        self.assertEqual(state.last_ack_at_ms, 1000)
        self.assertIsNone(state.last_command_error)

    def test_apply_ack_effects_honors_pending_zero_payloads(self):
        state = HostAppState(
            is_connected=True,
            pending_command="ENABLE",
            pending_command_value="0",
            motor_enabled=True,
            stall_open_loop_active=True,
        )
        apply_ack_effects(state, AckResult(command="ENABLE", ok=True, raw="ENABLE,OK"), now_ms=1000)
        self.assertFalse(state.motor_enabled)
        self.assertFalse(state.stall_open_loop_active)

        state = HostAppState(
            is_connected=True,
            pending_command="UNLOCK",
            pending_command_value="0",
            power_unlocked=True,
            motor_enabled=True,
            identify_active=True,
            stall_mode_armed=True,
            stall_open_loop_active=True,
        )
        apply_ack_effects(state, AckResult(command="UNLOCK", ok=True, raw="UNLOCK,OK"), now_ms=1000)
        self.assertFalse(state.power_unlocked)
        self.assertFalse(state.motor_enabled)
        self.assertFalse(state.identify_active)
        self.assertFalse(state.stall_mode_armed)
        self.assertFalse(state.stall_open_loop_active)

    def test_apply_ack_effects_uses_ack_payload_when_pending_value_missing(self):
        state = HostAppState(is_connected=True, pending_command="UNLOCK")
        apply_ack_effects(
            state,
            AckResult(command="UNLOCK", ok=True, command_value="1", raw="UNLOCK,OK,1"),
            now_ms=1000,
        )
        self.assertTrue(state.power_unlocked)

        state = HostAppState(is_connected=True, pending_command="ENABLE", motor_enabled=True)
        apply_ack_effects(
            state,
            AckResult(command="ENABLE", ok=True, command_value="0", raw="ENABLE,OK,0"),
            now_ms=1000,
        )
        self.assertFalse(state.motor_enabled)

    def test_apply_ack_effects_reports_failure(self):
        state = HostAppState(is_connected=True, pending_command="UNLOCK",
                             power_unlocked=False)
        ack = AckResult(command="UNLOCK", ok=False, reason="busy", raw="UNLOCK,FAIL,busy")
        apply_ack_effects(state, ack, now_ms=2000)
        self.assertFalse(state.power_unlocked)  # unchanged on failure
        self.assertIsNone(state.pending_command)  # pending cleared
        self.assertEqual(state.last_command_error, "busy")
        self.assertEqual(state.last_ack, "UNLOCK,FAIL,busy")

    def test_apply_ack_effects_mode_updates_control_mode(self):
        state = HostAppState(is_connected=True, pending_command="MODE")
        ack = AckResult(command="MODE", ok=True, mode_value=2, raw="MODE,OK,2")
        apply_ack_effects(state, ack, now_ms=3000)
        self.assertEqual(state.control_mode, 2)

    def test_apply_ack_effects_app_mode_with_ctrl(self):
        state = HostAppState(is_connected=True, pending_command="APP_MODE")
        ack = AckResult(command="APP_MODE", ok=True, app_mode_name="JOINT_POS",
                        app_mode_ctrl=2, raw="APP_MODE,OK,JOINT_POS (ctrl_mode=2)")
        apply_ack_effects(state, ack, now_ms=4000)
        self.assertEqual(state.app_mode, "JOINT_POS")
        self.assertEqual(state.app_mode_ctrl, 2)

    def test_apply_packet_effects_does_not_confirm_app_mode_pending(self):
        state = HostAppState(
            is_connected=True,
            pending_command="APP_MODE",
            pending_command_value="DETENT",
        )
        apply_packet_effects(state, FOCDataPacket(foc_state=FOC_STATE_RUNNING, control_mode=2))
        self.assertEqual(state.pending_command, "APP_MODE")
        self.assertEqual(state.pending_command_value, "DETENT")
        self.assertIsNone(state.app_mode)

    def test_apply_packet_effects_reconciles_runtime_flags_from_foc_state(self):
        state = HostAppState(is_connected=True, power_unlocked=True, motor_enabled=True, identify_active=True)
        apply_packet_effects(state, FOCDataPacket(foc_state=FOC_STATE_READY, is_fault_active=False))
        self.assertFalse(state.motor_enabled)
        self.assertFalse(state.identify_active)
        self.assertEqual(state.foc_state, FOC_STATE_READY)

        apply_packet_effects(state, FOCDataPacket(foc_state=FOC_STATE_PARAM_IDENTIFY, is_fault_active=False))
        self.assertFalse(state.motor_enabled)
        self.assertTrue(state.identify_active)
        self.assertEqual(state.foc_state, FOC_STATE_PARAM_IDENTIFY)

        apply_packet_effects(
            state,
            FOCDataPacket(
                foc_state=FOC_STATE_RUNNING,
                is_fault_active=False,
                motor_identified=True,
                stall_mode_armed=True,
            ),
        )
        self.assertTrue(state.motor_enabled)
        self.assertFalse(state.identify_active)
        self.assertEqual(state.foc_state, FOC_STATE_RUNNING)
        self.assertTrue(state.motor_identified)
        self.assertTrue(state.stall_mode_armed)

    def test_button_enable_state_blocks_start_and_enable_when_fault_is_active(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_enabled=True,
            foc_state=FOC_STATE_FAULT,
            fault_active=True,
        )
        button_state = button_enable_state(state)
        self.assertFalse(button_state["can_enable"])
        self.assertFalse(button_state["can_identify_start"])
        self.assertFalse(button_state["can_disable"])
        self.assertTrue(button_state["can_clear_fault"])

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
        self.assertIn("故障激活", summary["state"])
        self.assertIn("0x0640", summary["fault1"])
        self.assertIn("0x00C0", summary["vgs2"])

    def test_fault_summary_text_marks_application_fault_without_drv_flags(self):
        packet = FOCDataPacket(
            timestamp=456,
            foc_state=FOC_STATE_FAULT,
            is_fault_active=False,
            app_fault_code=3,
        )

        summary = fault_summary_text(packet)

        self.assertIn("故障激活", summary["state"])
        self.assertIn("欠压", summary["state"])
        self.assertIn("应用故障", summary["fault1"])

    def test_fault_summary_text_surfaces_voltage_warning_without_fault_gating(self):
        packet = FOCDataPacket(
            timestamp=456,
            foc_state=FOC_STATE_READY,
            is_fault_active=False,
            app_warning_flags=1,
            app_fault_code=0,
        )

        summary = fault_summary_text(packet)

        self.assertIn("告警激活", summary["state"])
        self.assertIn("欠压", summary["state"])
        self.assertIn("应用告警", summary["fault1"])

    def test_button_enable_state_keeps_enable_available_when_only_warning_is_present(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_enabled=False,
            foc_state=FOC_STATE_READY,
            fault_active=False,
        )

        button_state = button_enable_state(state)

        self.assertTrue(button_state["can_enable"])

    def test_fault_packet_log_entry_builds_summary_for_compact_fault_frame(self):
        packet = FOCDataPacket(
            timestamp=3456,
            foc_state=FOC_STATE_FAULT,
            fault_flags=0x00000020,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            encoder_detected=False,
            is_fault_active=True,
            raw_text="F,3456,5,0x00000020,1,0,0x0640,0x00C0,0xFFFF",
        )

        entry = fault_packet_log_entry(packet)

        self.assertIsNotNone(entry)
        signature, text = entry
        self.assertIn("summary", signature)
        self.assertIn("故障摘要", text)
        self.assertIn("0x0640", text)
        self.assertIn("未检测到", text)

    def test_fault_packet_log_entry_logs_application_fault_without_drv_fault_bits(self):
        packet = FOCDataPacket(
            timestamp=3456,
            foc_state=FOC_STATE_FAULT,
            fault_flags=0x00000000,
            fault_status1=0x0000,
            vgs_status2=0x0000,
            encoder_detected=True,
            is_fault_active=False,
            app_fault_code=4,
            raw_text="N,3456,5,12.34,0.00,0.000,0.000,11.90,0x00000000,1,1,0,0,4,18.00,28.00",
        )

        entry = fault_packet_log_entry(packet)

        self.assertIsNotNone(entry)
        signature, text = entry
        self.assertIn("summary", signature)
        self.assertIn("应用故障", text)
        self.assertIn("编码器", text)

    def test_fault_packet_log_entry_keeps_detailed_fault_text(self):
        detail = (
            "========== !!! FAULT DETECTED !!! ==========\n"
            "Time: 3456 ms\n"
            "FAULT1: 0x0640 | VGS2: 0x00C0\n"
            "[CRIT] VDS Overcurrent!\n"
            "============================================="
        )
        packet = FOCDataPacket(
            timestamp=3456,
            foc_state=FOC_STATE_FAULT,
            fault_status1=0x0640,
            vgs_status2=0x00C0,
            is_fault_active=True,
            raw_text=detail,
        )

        entry = fault_packet_log_entry(packet)

        self.assertIsNotNone(entry)
        signature, text = entry
        self.assertIn("detail", signature)
        self.assertEqual(text, detail)

    def test_fault_packet_log_entry_ignores_non_fault_diagnostic_snapshot(self):
        detail = (
            "========== !!! FAULT DETECTED !!! ==========\n"
            "Time: 2338354 ms\n"
            "[TLE5012 Encoder]\n"
            "  Detected: YES\n"
            "  Reset:    FAULT!\n"
            "[FOC Application]\n"
            "  State:    RUNNING\n"
            "  AppFault: 0 (None)\n"
            "[DRV8350S Fault Details]\n"
            "  FAULT1: 0x0000 | VGS2: 0x0000\n"
            "============================================="
        )
        packet = FOCDataPacket(
            timestamp=2338354,
            foc_state=FOC_STATE_RUNNING,
            fault_status1=0x0000,
            vgs_status2=0x0000,
            app_fault_code=0,
            is_fault_active=False,
            raw_text=detail,
        )

        self.assertIsNone(fault_packet_log_entry(packet))

    def test_fault_packet_detail_signature_ignores_live_sample_values(self):
        detail_a = (
            "========== !!! FAULT DETECTED !!! ==========\n"
            "Time: 212183 ms\n"
            "  AngleRaw: 21007 (0x520F)\n"
            "  AppFault: 6 (Param Invalid)\n"
            "[Motor Identification]\n"
            "  State:  8 (ERROR)\n"
            "  Error:  8 (MI_ERR_CURRENT_TOO_LOW / 识别电流过低)\n"
            "  RsDiag: target=2.000 A | Vd_avg=5.540 V | Id_avg=-0.020 A\n"
            "============================================="
        )
        detail_b = (
            "========== !!! FAULT DETECTED !!! ==========\n"
            "Time: 220183 ms\n"
            "  AngleRaw: 20850 (0x5172)\n"
            "  AppFault: 6 (Param Invalid)\n"
            "[Motor Identification]\n"
            "  State:  8 (ERROR)\n"
            "  Error:  8 (MI_ERR_CURRENT_TOO_LOW / 识别电流过低)\n"
            "  RsDiag: target=2.000 A | Vd_avg=5.541 V | Id_avg=-0.021 A\n"
            "============================================="
        )

        entry_a = fault_packet_log_entry(
            FOCDataPacket(
                timestamp=212183,
                foc_state=FOC_STATE_FAULT,
                app_fault_code=6,
                is_fault_active=True,
                raw_text=detail_a,
            )
        )
        entry_b = fault_packet_log_entry(
            FOCDataPacket(
                timestamp=220183,
                foc_state=FOC_STATE_FAULT,
                app_fault_code=6,
                is_fault_active=True,
                raw_text=detail_b,
            )
        )

        self.assertIsNotNone(entry_a)
        self.assertIsNotNone(entry_b)
        self.assertEqual(entry_a[0], entry_b[0])

    def test_fault_summary_includes_identification_failure_when_available(self):
        entry = fault_packet_log_entry(
            FOCDataPacket(
                timestamp=6456,
                foc_state=FOC_STATE_FAULT,
                app_fault_code=6,
                identify_state=9,
                identify_error=12,
                encoder_detected=True,
                raw_text="F,6456,5,0x00000000,0,1,0,0x0000,0x0000,0x0000,0,6,9,12,11.90,9.00,16.00",
            )
        )

        self.assertIsNotNone(entry)
        self.assertIn("识别状态=9", entry[1])
        self.assertIn("识别错误=12", entry[1])

    def test_fault_packet_log_entry_ignores_normal_packet(self):
        packet = FOCDataPacket(timestamp=100, foc_state=FOC_STATE_READY, is_fault_active=False, raw_text="N,100,3")
        self.assertIsNone(fault_packet_log_entry(packet))

    def test_fault_error_log_entry_only_accepts_fault_like_errors(self):
        self.assertIsNone(fault_error_log_entry("INFO", "已连接到 COM16"))
        self.assertIsNone(fault_error_log_entry("ERROR", "串口读取失败"))

        entry = fault_error_log_entry("ERROR", "故障：母线过压，请检查电源。")

        self.assertIsNotNone(entry)
        signature, text = entry
        self.assertIn("error", signature)
        self.assertIn("母线过压", text)

    def test_build_vbus_limit_command_validates_order_and_range(self):
        with self.assertRaisesRegex(ValueError, "欠压阈值"):
            build_vbus_limit_command("", "15.0")

        with self.assertRaisesRegex(ValueError, "过压阈值必须 > 欠压阈值"):
            build_vbus_limit_command("12.0", "12.0")

        with self.assertRaisesRegex(ValueError, "过压阈值必须 > 欠压阈值"):
            build_vbus_limit_command("15.0", "12.0")

    def test_can_edit_vbus_limits_only_when_motor_is_not_active(self):
        self.assertFalse(can_edit_vbus_limits(HostAppState(is_connected=False)))
        self.assertTrue(can_edit_vbus_limits(HostAppState(is_connected=True)))
        self.assertFalse(can_edit_vbus_limits(HostAppState(is_connected=True, motor_enabled=True)))
        self.assertFalse(can_edit_vbus_limits(HostAppState(is_connected=True, identify_active=True)))
        self.assertFalse(can_edit_vbus_limits(HostAppState(is_connected=True, pending_command="ENABLE")))

    def test_should_confirm_stall_mode_enable_only_when_unidentified_and_unarmed(self):
        state = HostAppState(is_connected=True, power_unlocked=True)
        self.assertTrue(should_confirm_stall_mode_enable(state))

        state.motor_identified = True
        state.encoder_detected = True
        self.assertFalse(should_confirm_stall_mode_enable(state))

        state.motor_identified = False
        state.stall_mode_armed = True
        self.assertFalse(should_confirm_stall_mode_enable(state))

    def test_should_confirm_stall_mode_enable_when_encoder_is_offline(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_identified=True,
            encoder_detected=False,
        )
        self.assertTrue(should_confirm_stall_mode_enable(state))

    def test_should_confirm_stall_mode_enable_when_encoder_state_unknown(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_identified=True,
            encoder_detected=None,
        )
        self.assertTrue(should_confirm_stall_mode_enable(state))

    def test_stall_mode_confirmation_text_mentions_encoder_warning(self):
        state = HostAppState(
            is_connected=True,
            power_unlocked=True,
            motor_identified=True,
            encoder_detected=False,
        )
        self.assertIn("TLE5012", stall_mode_confirmation_text(state))
        self.assertIn("堵转模式", stall_mode_confirmation_text(state))
        self.assertIn("开环试转", stall_mode_confirmation_text(state))

    def test_host_state_tracks_stall_open_loop_runtime_flag(self):
        state = HostAppState()
        self.assertFalse(state.stall_open_loop_active)

    def test_rolling_plot_buffer_caps_history_and_exports_series(self):
        buffer = RollingPlotBuffer(max_samples=2)
        buffer.append_packet(FOCDataPacket(timestamp=10, speed=1.0, Iq=0.2, Ia=0.1, Ib=0.2, Ic=-0.3, vbus=11.7, speed_ref=0.5, pos_ref=1.0))
        buffer.append_packet(FOCDataPacket(timestamp=20, speed=2.0, Iq=0.4, Ia=0.4, Ib=0.5, Ic=-0.9, vbus=11.8, speed_ref=1.5, pos_ref=2.0))
        buffer.append_packet(FOCDataPacket(timestamp=30, speed=3.0, Iq=0.6, Ia=0.7, Ib=0.8, Ic=-1.5, vbus=11.9, speed_ref=2.5, pos_ref=3.0))

        times, values = buffer.series("speed")
        self.assertEqual(times, [20.0, 30.0])
        self.assertEqual(values, [2.0, 3.0])

        _, ia_values = buffer.series("Ia")
        self.assertEqual(ia_values, [0.4, 0.7])

        _, vbus_values = buffer.series("Vbus")
        self.assertEqual(vbus_values, [11.8, 11.9])

        _, pos_ref_values = buffer.series("pos_ref_deg")
        # pos_ref is stored in firmware control frame (multiplied by encoder_dir=-1).
        # control_to_user_angle inverts: 2.0*(-1)+2π=4.283→245.41°, 3.0*(-1)+2π=3.283→188.11°
        self.assertEqual(len(pos_ref_values), 2)
        self.assertAlmostEqual(pos_ref_values[0], 245.40844097383535, places=4)
        self.assertAlmostEqual(pos_ref_values[1], 188.11266146075303, places=4)
        self.assertIn("pos_ref_deg", PLOT_CHANNELS)
        self.assertNotIn("pos_ref", PLOT_CHANNELS)
        self.assertIn("speed_ref", PLOT_CHANNELS)

        csv_text = format_plot_csv(buffer.export_rows(["speed", "speed_ref", "pos_ref_deg", "Iq", "Ia", "Ib", "Ic", "Vbus"]))
        self.assertIn("timestamp_ms,speed,speed_ref,pos_ref_deg,Iq,Ia,Ib,Ic,Vbus", csv_text)
        self.assertIn("30,3.0,2.5,188.11266146075303,0.6,0.7,0.8,-1.5,11.9", csv_text)

    def test_rolling_plot_buffer_defaults_to_30_second_history(self):
        buffer = RollingPlotBuffer()

        self.assertEqual(buffer.history_window_ms, 30000)

        buffer.append_packet(FOCDataPacket(timestamp=0, speed=1.0))
        buffer.append_packet(FOCDataPacket(timestamp=29999, speed=2.0))
        buffer.append_packet(FOCDataPacket(timestamp=30001, speed=3.0))

        times, values = buffer.series("speed")
        self.assertEqual(times, [29999.0, 30001.0])
        self.assertEqual(values, [2.0, 3.0])

    def test_rolling_plot_buffer_merges_phase_current_only_samples(self):
        buffer = RollingPlotBuffer()

        buffer.append_packet(FOCDataPacket(timestamp=1000, angle=45.0, speed=2.0, Iq=0.4, Ia=0.1, Ib=0.2, Ic=-0.3, vbus=11.8))
        buffer.append_packet(FOCDataPacket(timestamp=1005, Ia=0.4, Ib=0.5, Ic=-0.9, phase_current_only=True))

        times, ia_values = buffer.series("Ia")
        _, speed_values = buffer.series("speed")
        _, vbus_values = buffer.series("Vbus")
        self.assertEqual(times, [1000.0, 1005.0])
        self.assertEqual(ia_values, [0.1, 0.4])
        self.assertEqual(speed_values, [2.0, 2.0])
        self.assertEqual(vbus_values, [11.8, 11.8])

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
                motor_pn=11,
                current_pi=LoopTuning(kp=0.2, ki=0.01),
                speed_pi=LoopTuning(kp=1.0, ki=0.1),
                position_pd=PositionLoopTuning(kp=2.0, kd=0.2),
            )
            save_gui_profile(profile_path, profile)
            loaded = load_gui_profile(profile_path)

        self.assertEqual(loaded.last_port, "COM9")
        self.assertEqual(loaded.baud_rate, 460800)
        self.assertEqual(loaded.selected_mode, 2)
        self.assertEqual(loaded.log_filters, ["ERROR", "TX"])
        self.assertEqual(loaded.current_target, (0.1, 0.8))
        self.assertEqual(loaded.motor_pn, 11)
        self.assertEqual(loaded.speed_pi.kp, 1.0)
        self.assertEqual(loaded.position_pd.kd, 0.2)

    def test_legacy_position_pi_profile_keeps_safe_default_kd(self):
        legacy_payload = {
            "position_pi": {
                "kp": 6.0,
                "ki": 0.8,
            }
        }

        loaded = GuiProfile.from_dict(legacy_payload)

        self.assertEqual(loaded.position_pd.kp, 6.0)
        self.assertEqual(loaded.position_pd.kd, 0.08)

    def test_legacy_gui_profile_migrates_old_24v_defaults_to_12v_bench_defaults(self):
        legacy_payload = {
            "undervoltage_limit": 18.0,
            "overvoltage_limit": 28.0,
            "current_pi": {"kp": 0.2, "ki": 0.01},
            "speed_pi": {"kp": 1.0, "ki": 0.1},
            "position_pd": {"kp": 10.0, "kd": 0.10},
        }

        loaded = GuiProfile.from_dict(legacy_payload)

        self.assertEqual(loaded.motor_pn, 11)
        self.assertEqual(loaded.current_pi, LoopTuning(kp=0.03, ki=0.5))
        self.assertEqual(loaded.speed_pi, LoopTuning(kp=0.10, ki=0.0))
        self.assertEqual(loaded.position_pd, PositionLoopTuning(kp=2.0, kd=0.08))

    def test_v3_gui_profile_migrates_stale_bringup_defaults_to_v18_safe_defaults(self):
        stale_payload = {
            "schema_version": 3,
            "current_pi": {"kp": 0.3, "ki": 0.0},
            "speed_pi": {"kp": 0.3, "ki": 0.0},
            "position_pd": {"kp": 4.0, "kd": 0.12},
        }

        loaded = GuiProfile.from_dict(stale_payload)

        self.assertEqual(loaded.current_pi, LoopTuning(kp=0.03, ki=0.5))
        self.assertEqual(loaded.position_pd, PositionLoopTuning(kp=2.0, kd=0.08))

    def test_profile_migration_preserves_user_tuned_loop_values(self):
        tuned_payload = {
            "schema_version": 3,
            "current_pi": {"kp": 0.04, "ki": 0.25},
            "speed_pi": {"kp": 0.12, "ki": 0.01},
            "position_pd": {"kp": 2.5, "kd": 0.06},
        }

        loaded = GuiProfile.from_dict(tuned_payload)

        self.assertEqual(loaded.current_pi, LoopTuning(kp=0.04, ki=0.25))
        self.assertEqual(loaded.speed_pi, LoopTuning(kp=0.12, ki=0.01))
        self.assertEqual(loaded.position_pd, PositionLoopTuning(kp=2.5, kd=0.06))

    def test_default_loop_tuning_matches_v18_safe_bringup_baseline(self):
        profile = GuiProfile()

        self.assertEqual(profile.current_pi.kp, 0.03)
        self.assertEqual(profile.current_pi.ki, 0.5)
        self.assertEqual(profile.motor_pn, 11)
        self.assertEqual(profile.speed_pi.kp, 0.10)
        self.assertEqual(profile.speed_pi.ki, 0.0)
        self.assertEqual(profile.position_pd.kp, 2.0)
        self.assertEqual(profile.position_pd.kd, 0.08)

    def test_stale_data_detection_uses_threshold(self):
        self.assertFalse(is_data_stale(last_packet_received_at_ms=1500, now_ms=2200, threshold_ms=1000))
        self.assertTrue(is_data_stale(last_packet_received_at_ms=1500, now_ms=2601, threshold_ms=1000))


if __name__ == "__main__":
    unittest.main()
