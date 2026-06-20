from __future__ import annotations

import time
from pathlib import Path

from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

try:
    import pyqtgraph as pg
except Exception:  # pragma: no cover
    pg = None

PLOT_CHANNEL_LABELS = {
    "angle": "角度",
    "speed": "速度",
    "speed_ref": "速度给定",
    "pos_ref_deg": "位置给定",
    "Vbus": "母线电压",
    "Ia": "Ia相电流",
    "Ib": "Ib相电流",
    "Ic": "Ic相电流",
    "Id": "Id",
    "Iq": "Iq",
    "Id_ref": "Id_ref",
    "Iq_ref": "Iq_ref",
    "Vd": "Vd",
    "Vq": "Vq",
}

try:
    from .data_parser import CommandBuilder, FOCDataPacket
    from .gui_logic import (
        DEFAULT_PROFILE_PATH,
        LOG_LEVELS,
        PLOT_CHANNELS,
        GuiProfile,
        HostAppState,
        LoopTuning,
        PositionLoopTuning,
        RollingPlotBuffer,
        apply_command_effects,
        apply_packet_effects,
        build_adc_noise_command,
        build_current_ref_command,
        build_encoder_dir_command,
        build_loop_gain_command,
        build_motor_pn_command,
        build_position_ref_command,
        build_speed_ref_command,
        build_vbus_limit_command,
        button_enable_state,
        can_edit_vbus_limits,
        fault_summary_text,
        fault_error_log_entry,
        fault_packet_log_entry,
        format_plot_csv,
        is_data_stale,
        load_gui_profile,
        log_line_text,
        mode_name,
        mode_target_label,
        packet_snapshot,
        packet_has_active_fault,
        parse_adc_noise_response,
        save_gui_profile,
        stall_mode_confirmation_text,
        should_confirm_stall_mode_enable,
    )
except ImportError:
    from data_parser import CommandBuilder, FOCDataPacket
    from gui_logic import (
        DEFAULT_PROFILE_PATH,
        LOG_LEVELS,
        PLOT_CHANNELS,
        GuiProfile,
        HostAppState,
        LoopTuning,
        PositionLoopTuning,
        RollingPlotBuffer,
        apply_command_effects,
        apply_packet_effects,
        build_adc_noise_command,
        build_current_ref_command,
        build_encoder_dir_command,
        build_loop_gain_command,
        build_motor_pn_command,
        build_position_ref_command,
        build_speed_ref_command,
        build_vbus_limit_command,
        button_enable_state,
        can_edit_vbus_limits,
        fault_summary_text,
        fault_error_log_entry,
        fault_packet_log_entry,
        format_plot_csv,
        is_data_stale,
        load_gui_profile,
        log_line_text,
        mode_name,
        mode_target_label,
        packet_snapshot,
        packet_has_active_fault,
        parse_adc_noise_response,
        save_gui_profile,
        stall_mode_confirmation_text,
        should_confirm_stall_mode_enable,
    )


class HostMainWindow(QMainWindow):
    command_requested = pyqtSignal(str)
    refresh_ports_requested = pyqtSignal()
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    LOG_LIMIT = 400
    IDENTIFY_LOG_LIMIT = 200
    STALE_THRESHOLD_MS = 1500
    VBUS_LIMIT_CONFIRM_TIMEOUT_MS = 600
    VBUS_LIMIT_CONFIRM_EPSILON_V = 0.01
    PLOT_REFRESH_INTERVAL_MS = 50
    LOG_REFRESH_INTERVAL_MS = 150
    KNOWN_MOTOR_RS_OHM = 8.8
    KNOWN_MOTOR_LD_H = 0.0005
    KNOWN_MOTOR_LQ_H = 0.0005
    KNOWN_MOTOR_KE = 0.129
    KNOWN_MOTOR_ENCODER_DIR = -1
    KNOWN_MOTOR_THETA_OFFSET = 0.0
    KNOWN_MOTOR_THETA_ZERO = 0.0

    def __init__(self, profile_path: Path | str | None = None):
        super().__init__()
        self._profile_path = Path(profile_path) if profile_path else DEFAULT_PROFILE_PATH
        self._profile = load_gui_profile(self._profile_path)
        self._state = HostAppState(selected_mode=self._profile.selected_mode)
        self._serial_worker = None
        self._loading_profile = False
        self._log_entries: list[tuple[str, str]] = []
        self._fault_log_entries: list[str] = []
        self._fault_log_signatures: set[str] = set()
        self._identify_entries: list[str] = []
        self._plot_buffer = RollingPlotBuffer()
        self._plot_curves: dict[str, object] = {}
        self._mode_button_map: dict[int, list[QRadioButton]] = {0: [], 1: [], 2: []}
        self._last_encoder_warning_state: bool | None = None
        self._last_rx_activity_at_ms: int | None = None
        self._pending_vbus_limit_request: tuple[float, float] | None = None
        self._pending_vbus_limit_requested_at_ms: int | None = None
        self._plot_refresh_pending = False
        self._log_refresh_pending = False

        self.setWindowTitle("FOC 上位机调试工具")
        self.resize(1500, 920)

        self._build_toolbar()
        self._build_status_bar()

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._build_debug_panel(), "控制器参数")
        self.tabs.addTab(self._build_chart_tab(), "实时波形")
        self.tabs.addTab(self._build_identify_tab(), "参数识别")
        self.tabs.addTab(self._build_advanced_control_tab(), "高级控制")
        self.tabs.addTab(self._build_pi_tab(), "环路参数")
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.setInterval(250)
        self._heartbeat_timer.timeout.connect(self._refresh_session_status)
        self._heartbeat_timer.start()

        self._plot_refresh_timer = QTimer(self)
        self._plot_refresh_timer.setInterval(self.PLOT_REFRESH_INTERVAL_MS)
        self._plot_refresh_timer.timeout.connect(self._flush_pending_plot_refresh)

        self._log_refresh_timer = QTimer(self)
        self._log_refresh_timer.setInterval(self.LOG_REFRESH_INTERVAL_MS)
        self._log_refresh_timer.timeout.connect(self._flush_pending_log_refresh)
        self._log_refresh_timer.start()

        self._load_profile_into_widgets()
        self.apply_mode_selection(self._state.selected_mode, emit_command=False)
        self.update_connection_state(False)
        self._render_logs()
        self._render_fault_log()
        self._render_identify_log()

    def _build_toolbar(self):
        toolbar = QToolBar("连接")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        toolbar.addWidget(QLabel("串口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        self.port_combo.currentTextChanged.connect(self._persist_profile_from_widgets)
        self.port_combo.currentTextChanged.connect(self._sync_connect_button)
        toolbar.addWidget(self.port_combo)

        toolbar.addSeparator()
        toolbar.addWidget(QLabel("波特率"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        self.baud_combo.currentTextChanged.connect(self._persist_profile_from_widgets)
        toolbar.addWidget(self.baud_combo)

        toolbar.addSeparator()
        self.refresh_ports_button = QPushButton("刷新串口")
        self.refresh_ports_button.clicked.connect(self.refresh_ports_requested.emit)
        toolbar.addWidget(self.refresh_ports_button)

        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self._request_connect)
        toolbar.addWidget(self.connect_button)

        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.clicked.connect(self.disconnect_requested.emit)
        toolbar.addWidget(self.disconnect_button)

        toolbar.addSeparator()
        self.connection_status_value = QLabel("未连接")
        toolbar.addWidget(self.connection_status_value)

    def _build_status_bar(self):
        self.notification_label = QLabel("就绪")
        self.notification_label.setStyleSheet("padding: 0 8px; color: #0f172a;")
        self.statusBar().addPermanentWidget(self.notification_label, 1)

    def _build_debug_panel(self) -> QWidget:
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(18)

        left_column = QWidget()
        left_layout = QVBoxLayout(left_column)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)
        left_layout.addWidget(self._build_actions_column())
        left_layout.addWidget(self._build_runtime_column())

        layout.addWidget(left_column, 1)
        layout.addWidget(self._build_fault_log_column(), 2)
        return panel

    def _build_actions_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        power_group = QGroupBox("功率级")
        power_layout = QVBoxLayout(power_group)
        self.unlock_button = QPushButton("解锁")
        self.lock_button = QPushButton("上锁")
        self.enable_button = QPushButton("使能")
        self.disable_button = QPushButton("禁用")
        self.clear_fault_button = QPushButton("清除故障")
        self.unlock_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.unlock_power(True)))
        self.lock_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.unlock_power(False)))
        self.enable_button.clicked.connect(self._request_enable_motor)
        self.disable_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.enable_motor(False)))
        self.clear_fault_button.clicked.connect(self._clear_fault_with_hint)
        for button in (
            self.unlock_button,
            self.lock_button,
            self.enable_button,
            self.disable_button,
            self.clear_fault_button,
        ):
            power_layout.addWidget(button)
        layout.addWidget(power_group)

        quick_group = QGroupBox("快捷操作")
        quick_layout = QVBoxLayout(quick_group)
        self.quick_arm_button = QPushButton("解锁并使能")
        self.quick_safe_stop_button = QPushButton("禁用并上锁")
        self.quick_clear_rearm_button = QPushButton("清故障并提示重使能")
        self.quick_arm_button.clicked.connect(self._request_quick_arm)
        self.quick_safe_stop_button.clicked.connect(
            lambda: self._dispatch_sequence([CommandBuilder.enable_motor(False), CommandBuilder.unlock_power(False)])
        )
        self.quick_clear_rearm_button.clicked.connect(self._clear_fault_with_hint)
        for button in (self.quick_arm_button, self.quick_safe_stop_button, self.quick_clear_rearm_button):
            quick_layout.addWidget(button)
        layout.addWidget(quick_group)

        mode_group = QGroupBox("模式")
        mode_layout = QVBoxLayout(mode_group)
        self.mode_button_group = QButtonGroup(self)
        self.torque_mode_button = QRadioButton("力矩")
        self.speed_mode_button = QRadioButton("速度")
        self.position_mode_button = QRadioButton("位置")
        self._register_mode_button(self.torque_mode_button, 0, self.mode_button_group)
        self._register_mode_button(self.speed_mode_button, 1, self.mode_button_group)
        self._register_mode_button(self.position_mode_button, 2, self.mode_button_group)
        self.mode_button_group.idClicked.connect(self.apply_mode_selection)
        mode_layout.addWidget(self.torque_mode_button)
        mode_layout.addWidget(self.speed_mode_button)
        mode_layout.addWidget(self.position_mode_button)
        self.target_label = QLabel(mode_target_label(0))
        self.target_label.setStyleSheet("font-weight: 600; color: #0f766e;")
        mode_layout.addWidget(QLabel("当前目标通道"))
        mode_layout.addWidget(self.target_label)
        hint = QLabel("力矩、速度和位置目标值请在“高级控制”页单独设置。")
        hint.setWordWrap(True)
        mode_layout.addWidget(hint)
        layout.addWidget(mode_group)

        return widget

    def _build_runtime_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        runtime_group = QGroupBox("运行状态")
        runtime_layout = QFormLayout(runtime_group)
        self.state_value = QLabel("--")
        self.angle_value = QLabel("--")
        self.speed_value = QLabel("--")
        self.currents_value = QLabel("--")
        self.refs_value = QLabel("--")
        self.voltages_value = QLabel("--")
        runtime_layout.addRow("FOC 状态", self.state_value)
        runtime_layout.addRow("角度", self.angle_value)
        runtime_layout.addRow("速度", self.speed_value)
        runtime_layout.addRow("电流", self.currents_value)
        runtime_layout.addRow("参考值", self.refs_value)
        runtime_layout.addRow("电压", self.voltages_value)
        layout.addWidget(runtime_group)

        session_group = QGroupBox("会话")
        session_layout = QFormLayout(session_group)
        self.packet_timestamp_value = QLabel("--")
        self.connection_state_detail_value = QLabel("未连接")
        self.data_status_value = QLabel("空闲")
        self.session_mode_value = QLabel(mode_name(self._state.selected_mode))
        session_layout.addRow("连接状态", self.connection_state_detail_value)
        session_layout.addRow("数据新鲜度", self.data_status_value)
        session_layout.addRow("当前模式", self.session_mode_value)
        session_layout.addRow("最近数据包", self.packet_timestamp_value)
        layout.addWidget(session_group)
        return widget

    def _build_chart_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        toggles_layout = QGridLayout()
        self.plot_channel_checks: dict[str, QCheckBox] = {}
        for index, channel in enumerate(PLOT_CHANNELS):
            checkbox = QCheckBox(PLOT_CHANNEL_LABELS.get(channel, channel))
            checkbox.setChecked(channel in {"angle", "speed_ref", "pos_ref_deg", "Vbus", "Ia", "Ib", "Ic", "Id", "Iq", "Iq_ref"})
            checkbox.toggled.connect(self._refresh_plot)
            self.plot_channel_checks[channel] = checkbox
            toggles_layout.addWidget(checkbox, index // 4, index % 4)
        layout.addLayout(toggles_layout)

        if pg is None:
            self.plot_widget = QLabel("当前环境未提供 pyqtgraph。")
            self.plot_widget.setWordWrap(True)
            layout.addWidget(self.plot_widget)
        else:
            self.plot_widget = pg.PlotWidget()
            self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
            self.plot_widget.addLegend(offset=(10, 10))
            self.plot_widget.setBackground("#f8fafc")
            self.plot_widget.setLabel("bottom", "时间 (ms)")
            self.plot_widget.setLabel("left", "数值")
            layout.addWidget(self.plot_widget, 1)
            palette = {
                "angle": "#2563eb",
                "speed": "#059669",
                "speed_ref": "#84cc16",
                "pos_ref_deg": "#9333ea",
                "Vbus": "#a16207",
                "Ia": "#dc2626",
                "Ib": "#2563eb",
                "Ic": "#16a34a",
                "Id": "#dc2626",
                "Iq": "#ea580c",
                "Id_ref": "#7c3aed",
                "Iq_ref": "#db2777",
                "Vd": "#0f766e",
                "Vq": "#475569",
            }
            for channel in PLOT_CHANNELS:
                pen = pg.mkPen(palette[channel], width=2)
                self._plot_curves[channel] = self.plot_widget.plot(
                    [], [], pen=pen, name=PLOT_CHANNEL_LABELS.get(channel, channel)
                )

        self.export_plot_button = QPushButton("导出曲线 CSV")
        self.export_plot_button.clicked.connect(self._export_plot_csv)
        layout.addWidget(self.export_plot_button)
        return widget

    def _build_fault_log_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        fault_group = QGroupBox("故障摘要")
        fault_layout = QFormLayout(fault_group)
        self.fault_state_value = QLabel("正常")
        self.fault_registers_value = QLabel("FAULT1 0x0000 | VGS2 0x0000")
        self.fault_timestamp_value = QLabel("--")
        fault_layout.addRow("状态", self.fault_state_value)
        fault_layout.addRow("寄存器", self.fault_registers_value)
        fault_layout.addRow("时间戳", self.fault_timestamp_value)
        layout.addWidget(fault_group)

        log_group = QGroupBox("串口日志")
        log_layout = QVBoxLayout(log_group)
        filter_layout = QHBoxLayout()
        self.log_filter_checks: dict[str, QCheckBox] = {}
        for level in LOG_LEVELS:
            checkbox = QCheckBox({
                "INFO": "信息",
                "TX": "发送",
                "RX": "接收",
                "ERROR": "错误",
            }[level])
            checkbox.setChecked(True)
            checkbox.toggled.connect(self._on_log_filter_changed)
            self.log_filter_checks[level] = checkbox
            filter_layout.addWidget(checkbox)
        log_layout.addLayout(filter_layout)

        button_layout = QHBoxLayout()
        self.copy_log_button = QPushButton("复制当前日志")
        self.clear_log_button = QPushButton("清空日志")
        self.copy_log_button.clicked.connect(self._copy_recent_log)
        self.clear_log_button.clicked.connect(self._clear_log)
        button_layout.addWidget(self.copy_log_button)
        button_layout.addWidget(self.clear_log_button)
        log_layout.addLayout(button_layout)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("连接事件、TX/RX 收发和错误信息会显示在这里。")
        log_layout.addWidget(self.log_view, 1)
        layout.addWidget(log_group, 1)

        fault_log_group = QGroupBox("故障日志")
        fault_log_layout = QVBoxLayout(fault_log_group)
        fault_button_layout = QHBoxLayout()
        self.copy_fault_log_button = QPushButton("复制故障日志")
        self.clear_fault_log_button = QPushButton("清空故障日志")
        self.copy_fault_log_button.clicked.connect(self._copy_fault_log)
        self.clear_fault_log_button.clicked.connect(self._clear_fault_log)
        fault_button_layout.addWidget(self.copy_fault_log_button)
        fault_button_layout.addWidget(self.clear_fault_log_button)
        fault_log_layout.addLayout(fault_button_layout)

        self.fault_log_view = QPlainTextEdit()
        self.fault_log_view.setReadOnly(True)
        self.fault_log_view.setPlaceholderText("故障摘要、详细故障文本和关键故障提示会保留在这里，直到手动清空。")
        fault_log_layout.addWidget(self.fault_log_view, 1)
        layout.addWidget(fault_log_group, 1)
        return widget

    def _build_identify_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        self.identify_param_group = QGroupBox("识别状态 / 电机参数")
        identify_param_layout = QVBoxLayout(self.identify_param_group)
        identify_param_layout.setContentsMargins(12, 12, 12, 12)
        identify_param_layout.setSpacing(8)

        self.identify_status_panel = QGroupBox("识别状态")
        state_layout = QFormLayout(self.identify_status_panel)
        self.identify_connection_value = QLabel("未连接")
        self.identify_power_value = QLabel("已上锁")
        self.identify_state_value = QLabel("--")
        self.identify_result_value = QLabel("未识别")
        self.identify_encoder_value = QLabel("未知")
        self.identify_stall_mode_value = QLabel("未授权")
        self.identify_stall_open_loop_value = QLabel("未激活")
        self.identify_fault_value = QLabel("未知")
        state_layout.addRow("连接状态", self.identify_connection_value)
        state_layout.addRow("功率级", self.identify_power_value)
        state_layout.addRow("FOC 状态", self.identify_state_value)
        state_layout.addRow("电机参数", self.identify_result_value)
        state_layout.addRow("编码器", self.identify_encoder_value)
        state_layout.addRow("堵转授权", self.identify_stall_mode_value)
        state_layout.addRow("开环试转", self.identify_stall_open_loop_value)
        state_layout.addRow("故障", self.identify_fault_value)
        identify_param_layout.addWidget(self.identify_status_panel)

        self.identify_params_panel = QGroupBox("电机参数")
        params_layout = QFormLayout(self.identify_params_panel)
        self.motor_param_rs_value = QLabel("--")
        self.motor_param_ld_value = QLabel("--")
        self.motor_param_lq_value = QLabel("--")
        self.motor_param_ke_value = QLabel("--")
        self.motor_param_pn_value = QLabel("--")
        self.motor_param_encoder_dir_value = QLabel("--")
        self.motor_param_theta_offset_value = QLabel("--")
        self.motor_param_theta_zero_value = QLabel("--")
        params_layout.addRow("Rs (Ohm)", self.motor_param_rs_value)
        params_layout.addRow("Ld (H)", self.motor_param_ld_value)
        params_layout.addRow("Lq (H)", self.motor_param_lq_value)
        params_layout.addRow("Ke", self.motor_param_ke_value)
        params_layout.addRow("Pn", self.motor_param_pn_value)
        params_layout.addRow("编码器方向", self.motor_param_encoder_dir_value)
        params_layout.addRow("theta_offset", self.motor_param_theta_offset_value)
        params_layout.addRow("theta_zero", self.motor_param_theta_zero_value)
        identify_param_layout.addWidget(self.identify_params_panel)

        self.identify_actions_panel = QGroupBox("识别操作")
        action_layout = QHBoxLayout(self.identify_actions_panel)
        self.identify_start_page_button = QPushButton("开始识别")
        self.identify_stop_page_button = QPushButton("停止识别")
        self.identify_clear_fault_button = QPushButton("清除故障")
        self.identify_start_page_button.clicked.connect(self._request_start_identify)
        self.identify_stop_page_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.stop_identify(), identify_note="已请求停止识别。")
        )
        self.identify_clear_fault_button.clicked.connect(self._clear_fault_with_hint)
        action_layout.addWidget(self.identify_start_page_button)
        action_layout.addWidget(self.identify_stop_page_button)
        action_layout.addWidget(self.identify_clear_fault_button)
        identify_param_layout.addWidget(self.identify_actions_panel)
        layout.addWidget(self.identify_param_group)

        self.identify_motor_base_group = QGroupBox("电机基础参数")
        motor_layout = QHBoxLayout(self.identify_motor_base_group)
        self.motor_pn_input = QLineEdit("11")
        self.motor_pn_input.setPlaceholderText("极对数 1-50")
        self.motor_pn_apply_button = QPushButton("应用极对数")
        self.motor_pn_apply_button.clicked.connect(self._apply_motor_pn)
        motor_layout.addWidget(QLabel("Pn"))
        motor_layout.addWidget(self.motor_pn_input)
        motor_layout.addWidget(self.motor_pn_apply_button)
        layout.addWidget(self.identify_motor_base_group)

        self.identify_encoder_dir_group = QGroupBox("编码器方向")
        encoder_dir_layout = QHBoxLayout(self.identify_encoder_dir_group)
        self.encoder_dir_combo = QComboBox()
        self.encoder_dir_combo.addItems(["-1 (负向)", "+1 (正向)"])
        self.encoder_dir_combo.setCurrentIndex(0)  # default -1
        self.encoder_dir_apply_button = QPushButton("应用编码器方向")
        self.encoder_dir_apply_button.clicked.connect(self._apply_encoder_dir)
        encoder_dir_layout.addWidget(QLabel("方向"))
        encoder_dir_layout.addWidget(self.encoder_dir_combo)
        encoder_dir_layout.addWidget(self.encoder_dir_apply_button)
        layout.addWidget(self.identify_encoder_dir_group)

        log_group = QGroupBox("识别进度 / 日志")
        log_layout = QVBoxLayout(log_group)
        self.identify_log_view = QPlainTextEdit()
        self.identify_log_view.setReadOnly(True)
        self.identify_log_view.setPlaceholderText(
            "识别请求、状态快照以及后续固件上报的识别进度会显示在这里。"
        )
        log_layout.addWidget(self.identify_log_view)
        layout.addWidget(log_group, 1)

        self.start_identify_button = self.identify_start_page_button
        self.stop_identify_button = self.identify_stop_page_button
        return widget

    def _build_advanced_control_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        summary_group = QGroupBox("控制模式")
        summary_layout = QVBoxLayout(summary_group)
        self.advanced_mode_value = QLabel(mode_name(self._state.selected_mode))
        self.advanced_mode_value.setStyleSheet("font-size: 15px; font-weight: 700; color: #0f766e;")
        summary_layout.addWidget(self.advanced_mode_value)

        advanced_button_group = QButtonGroup(self)
        advanced_mode_layout = QHBoxLayout()
        self.advanced_torque_button = QRadioButton("力矩")
        self.advanced_speed_button = QRadioButton("速度")
        self.advanced_position_button = QRadioButton("位置")
        self._register_mode_button(self.advanced_torque_button, 0, advanced_button_group)
        self._register_mode_button(self.advanced_speed_button, 1, advanced_button_group)
        self._register_mode_button(self.advanced_position_button, 2, advanced_button_group)
        advanced_button_group.idClicked.connect(self.apply_mode_selection)
        advanced_mode_layout.addWidget(self.advanced_torque_button)
        advanced_mode_layout.addWidget(self.advanced_speed_button)
        advanced_mode_layout.addWidget(self.advanced_position_button)
        summary_layout.addLayout(advanced_mode_layout)
        layout.addWidget(summary_group)

        target_preset_layout = QHBoxLayout()
        self.load_target_preset_button = QPushButton("加载本地预设")
        self.save_target_preset_button = QPushButton("保存本地预设")
        self.load_target_preset_button.clicked.connect(self._load_preset_fields)
        self.save_target_preset_button.clicked.connect(self._save_local_preset)
        target_preset_layout.addWidget(self.load_target_preset_button)
        target_preset_layout.addWidget(self.save_target_preset_button)
        layout.addLayout(target_preset_layout)

        self.current_group = QGroupBox("力矩模式")
        current_layout = QFormLayout(self.current_group)
        self.current_id_input = QLineEdit()
        self.current_iq_input = QLineEdit()
        self.current_apply_button = QPushButton("应用电流给定")
        self.current_apply_button.clicked.connect(self._apply_current_refs)
        current_layout.addRow("Id_ref (A)", self.current_id_input)
        current_layout.addRow("Iq_ref (A)", self.current_iq_input)
        current_layout.addRow(self.current_apply_button)
        layout.addWidget(self.current_group)

        self.speed_group = QGroupBox("速度模式")
        speed_layout = QFormLayout(self.speed_group)
        self.speed_ref_input = QLineEdit()
        self.speed_apply_button = QPushButton("应用速度给定")
        self.speed_apply_button.clicked.connect(self._apply_speed_ref)
        speed_layout.addRow("速度 (rad/s)", self.speed_ref_input)
        speed_layout.addRow(self.speed_apply_button)
        layout.addWidget(self.speed_group)

        self.position_group = QGroupBox("位置模式")
        position_layout = QFormLayout(self.position_group)
        self.position_ref_input = QLineEdit()
        self.position_apply_button = QPushButton("应用位置给定")
        self.position_apply_button.clicked.connect(self._apply_position_ref)
        position_layout.addRow("位置 (deg)", self.position_ref_input)
        position_layout.addRow(self.position_apply_button)
        layout.addWidget(self.position_group)

        self.protection_group = QGroupBox("保护阈值")
        protection_layout = QFormLayout(self.protection_group)
        self.vbus_uv_input = QLineEdit()
        self.vbus_ov_input = QLineEdit()
        self.vbus_limit_apply_button = QPushButton("应用电压阈值")
        self.vbus_limit_apply_button.clicked.connect(self._apply_vbus_limits)
        self.vbus_limit_actual_value = QLabel("固件当前：UV -- / OV --")
        self.vbus_limit_actual_value.setStyleSheet("color: #0f766e; font-weight: 600;")
        self.vbus_limit_hint = QLabel("仅在未运行、未识别时允许下发；建议 12V 台架先调低欠压阈值。")
        self.vbus_limit_hint.setWordWrap(True)
        self.vbus_limit_hint.setStyleSheet("color: #475569;")
        protection_layout.addRow("欠压阈值 (V)", self.vbus_uv_input)
        protection_layout.addRow("过压阈值 (V)", self.vbus_ov_input)
        protection_layout.addRow("当前阈值", self.vbus_limit_actual_value)
        protection_layout.addRow(self.vbus_limit_apply_button)
        protection_layout.addRow(self.vbus_limit_hint)

        self.adc_noise_samples_input = QLineEdit("4096")
        self.adc_noise_button = QPushButton("开始ADC噪声测试")
        self.adc_noise_button.clicked.connect(self._run_adc_noise_test)
        self.adc_noise_result_value = QLabel("ADC噪声：未测试")
        self.adc_noise_result_value.setWordWrap(True)
        self.adc_noise_result_value.setStyleSheet(
            "color: #334155; font-family: Consolas, 'Microsoft YaHei UI';"
        )
        self.adc_noise_hint = QLabel("上电但不使能电机时使用；只显示统计值，不刷原始4096点。")
        self.adc_noise_hint.setWordWrap(True)
        self.adc_noise_hint.setStyleSheet("color: #475569;")
        protection_layout.addRow("ADC样本数", self.adc_noise_samples_input)
        protection_layout.addRow(self.adc_noise_button)
        protection_layout.addRow("ADC噪声", self.adc_noise_result_value)
        protection_layout.addRow(self.adc_noise_hint)
        layout.addWidget(self.protection_group)

        self.home_group = QGroupBox("机械零点")
        home_layout = QFormLayout(self.home_group)
        self.home_offset_value = QLabel("-- rad")
        self.home_offset_value.setStyleSheet("color: #0f766e; font-weight: 600;")
        self.home_set_button = QPushButton("设置零点")
        self.home_set_button.clicked.connect(self._send_home)
        self.home_clear_button = QPushButton("清除零点")
        self.home_clear_button.clicked.connect(self._send_clear_home)
        self.home_hint = QLabel("手动将关节转到机械零位后，点击「设置零点」录当前角度为偏移量，断电保持。")
        self.home_hint.setWordWrap(True)
        self.home_hint.setStyleSheet("color: #475569;")
        home_btn_layout = QHBoxLayout()
        home_btn_layout.addWidget(self.home_set_button)
        home_btn_layout.addWidget(self.home_clear_button)
        home_layout.addRow("当前偏移", self.home_offset_value)
        home_layout.addRow(home_btn_layout)
        home_layout.addRow(self.home_hint)
        layout.addWidget(self.home_group)

        return widget

    def _build_pi_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        preset_layout = QHBoxLayout()
        self.load_pi_preset_button = QPushButton("加载本地默认值")
        self.save_pi_preset_button = QPushButton("保存本地预设")
        self.load_pi_preset_button.clicked.connect(self._load_preset_fields)
        self.save_pi_preset_button.clicked.connect(self._save_local_preset)
        preset_layout.addWidget(self.load_pi_preset_button)
        preset_layout.addWidget(self.save_pi_preset_button)
        layout.addLayout(preset_layout)

        self.current_pi_group = QGroupBox("电流环 PI")
        current_layout = QFormLayout(self.current_pi_group)
        self.current_pi_kp_input = QLineEdit()
        self.current_pi_ki_input = QLineEdit()
        self.current_pi_ki_input.setToolTip("Current-loop Ki is entered as continuous-time Ki; firmware divides it by the current-loop frequency.")
        self.current_pi_defaults_button = QPushButton("加载默认值")
        self.current_pi_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("current"))
        self.current_pi_apply_button = QPushButton("应用电流环 PI")
        self.current_pi_apply_button.clicked.connect(lambda: self._apply_loop_gains("current"))
        current_layout.addRow("Kp", self.current_pi_kp_input)
        current_layout.addRow("Ki", self.current_pi_ki_input)
        current_layout.addRow(self.current_pi_defaults_button, self.current_pi_apply_button)
        layout.addWidget(self.current_pi_group)

        self.speed_pi_group = QGroupBox("速度环 PI")
        speed_layout = QFormLayout(self.speed_pi_group)
        self.speed_pi_kp_input = QLineEdit()
        self.speed_pi_ki_input = QLineEdit()
        self.speed_pi_defaults_button = QPushButton("加载默认值")
        self.speed_pi_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("speed"))
        self.speed_pi_apply_button = QPushButton("应用速度环 PI")
        self.speed_pi_apply_button.clicked.connect(lambda: self._apply_loop_gains("speed"))
        speed_layout.addRow("Kp", self.speed_pi_kp_input)
        speed_layout.addRow("Ki", self.speed_pi_ki_input)
        speed_layout.addRow(self.speed_pi_defaults_button, self.speed_pi_apply_button)
        layout.addWidget(self.speed_pi_group)

        self.position_pd_group = QGroupBox("位置环 PD")
        position_layout = QFormLayout(self.position_pd_group)
        self.position_pd_kp_input = QLineEdit()
        self.position_pd_kd_input = QLineEdit()
        self.position_pd_defaults_button = QPushButton("加载默认值")
        self.position_pd_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("position"))
        self.position_pd_apply_button = QPushButton("应用位置环 PD")
        self.position_pd_apply_button.clicked.connect(lambda: self._apply_loop_gains("position"))
        position_layout.addRow("Kp", self.position_pd_kp_input)
        position_layout.addRow("Kd", self.position_pd_kd_input)
        position_layout.addRow(self.position_pd_defaults_button, self.position_pd_apply_button)
        layout.addWidget(self.position_pd_group)

        return widget

    def _register_mode_button(self, button: QRadioButton, mode: int, group: QButtonGroup):
        group.addButton(button, mode)
        self._mode_button_map[mode].append(button)

    def _load_profile_into_widgets(self):
        self._loading_profile = True
        self.baud_combo.setCurrentText(str(self._profile.baud_rate))
        for level, checkbox in self.log_filter_checks.items():
            checkbox.setChecked(level in self._profile.log_filters)
        self.current_id_input.setText(f"{self._profile.current_target[0]:.3f}")
        self.current_iq_input.setText(f"{self._profile.current_target[1]:.3f}")
        self.speed_ref_input.setText(f"{self._profile.speed_target:.3f}")
        self.position_ref_input.setText(f"{self._profile.position_target:.3f}")
        self.motor_pn_input.setText(str(self._profile.motor_pn))
        self.vbus_uv_input.setText(f"{self._profile.undervoltage_limit:.3f}")
        self.vbus_ov_input.setText(f"{self._profile.overvoltage_limit:.3f}")
        self._set_loop_inputs("current", self._profile.current_pi)
        self._set_loop_inputs("speed", self._profile.speed_pi)
        self._set_loop_inputs("position", self._profile.position_pd)
        self._loading_profile = False

    def _set_loop_inputs(self, loop_name: str, tuning: LoopTuning | PositionLoopTuning):
        widgets = {
            "current": (self.current_pi_kp_input, self.current_pi_ki_input),
            "speed": (self.speed_pi_kp_input, self.speed_pi_ki_input),
            "position": (self.position_pd_kp_input, self.position_pd_kd_input),
        }
        kp_widget, gain2_widget = widgets[loop_name]
        gain2_value = tuning.ki if hasattr(tuning, "ki") else tuning.kd
        kp_widget.setText(f"{tuning.kp:.6f}")
        gain2_widget.setText(f"{gain2_value:.6f}")

    def set_serial_worker(self, worker):
        self._serial_worker = worker
        self.refresh_ports_requested.connect(worker.refresh_ports)
        self.connect_requested.connect(worker.connect_port)
        self.disconnect_requested.connect(worker.disconnect_port)
        self.command_requested.connect(worker.send_command)
        worker.ports_updated.connect(self.update_ports)
        worker.connection_changed.connect(self.update_connection_state)
        worker.log_line.connect(self.handle_log_line)
        worker.packet_received.connect(self.apply_packet)

    def update_ports(self, ports: list[str]):
        self._state.available_ports = list(ports)
        current = self.port_combo.currentText().strip()
        preferred = current or self._profile.last_port
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if preferred and preferred in ports:
            self.port_combo.setCurrentText(preferred)
        self.port_combo.blockSignals(False)
        self._persist_profile_from_widgets()
        self._sync_connect_button()

    def update_connection_state(self, is_connected: bool):
        was_connected = bool(self._state.is_connected)
        self._state.is_connected = bool(is_connected)
        if not self._state.is_connected:
            self._state.power_unlocked = False
            self._state.motor_enabled = False
            self._state.identify_active = False
            self._state.motor_identified = False
            self._state.stall_mode_armed = False
            self._state.stall_open_loop_active = False
            self._state.foc_state = None
            self._state.fault_active = False
            self._state.last_packet = None
            self._state.last_packet_received_at_ms = None
            self._last_rx_activity_at_ms = None
            self._pending_vbus_limit_request = None
            self._pending_vbus_limit_requested_at_ms = None
            self._clear_runtime_snapshot()
            self._plot_buffer = RollingPlotBuffer()
            self._refresh_plot()
        elif self._state.last_packet is None:
            self.vbus_limit_actual_value.setText("固件当前：等待固件回传")
            if not was_connected:
                self._request_parameter_snapshot()

        self._apply_control_enable_state()
        self._refresh_identify_state_panel()
        self._refresh_mode_views()
        self._refresh_session_status()
        self._sync_connect_button()

    def _apply_control_enable_state(self):
        state = button_enable_state(self._state)
        self.connection_status_value.setText("已连接" if self._state.is_connected else "未连接")
        self.connection_state_detail_value.setText("已连接" if self._state.is_connected else "未连接")
        self.connection_status_value.setStyleSheet(
            "color: #0f766e; font-weight: 600;" if self._state.is_connected else "color: #b91c1c; font-weight: 600;"
        )
        self.disconnect_button.setEnabled(self._state.is_connected)
        self.unlock_button.setEnabled(state["can_unlock"])
        self.lock_button.setEnabled(state["can_lock"])
        self.enable_button.setEnabled(state["can_enable"])
        self.disable_button.setEnabled(state["can_disable"])
        self.clear_fault_button.setEnabled(state["can_clear_fault"])
        self.identify_start_page_button.setEnabled(state["can_identify_start"])
        self.identify_stop_page_button.setEnabled(state["can_identify_stop"])
        self.identify_clear_fault_button.setEnabled(state["can_clear_fault"])
        self.motor_pn_apply_button.setEnabled(can_edit_vbus_limits(self._state))
        self.encoder_dir_apply_button.setEnabled(can_edit_vbus_limits(self._state))

        target_enabled = state["can_send_target"]
        for button in (
            self.current_apply_button,
            self.speed_apply_button,
            self.position_apply_button,
            self.current_pi_apply_button,
            self.speed_pi_apply_button,
            self.position_pd_apply_button,
            self.quick_arm_button,
            self.quick_clear_rearm_button,
        ):
            button.setEnabled(target_enabled)
        self.vbus_limit_apply_button.setEnabled(can_edit_vbus_limits(self._state))
        self.adc_noise_button.setEnabled(can_edit_vbus_limits(self._state))
        self.quick_safe_stop_button.setEnabled(self._state.is_connected and (self._state.power_unlocked or self._state.motor_enabled))
        self.export_plot_button.setEnabled(self._plot_buffer.has_rows())

    def apply_mode_selection(self, mode: int, emit_command: bool = True):
        self._state.selected_mode = int(mode)
        self.target_label.setText(mode_target_label(self._state.selected_mode))
        self.session_mode_value.setText(mode_name(self._state.selected_mode))
        self.advanced_mode_value.setText(f"{mode_name(self._state.selected_mode)}模式已激活")
        self._sync_mode_buttons(self._state.selected_mode)
        self._refresh_mode_highlight()
        self._persist_profile_from_widgets()
        if emit_command and self._state.is_connected:
            self._dispatch_command(CommandBuilder.set_mode(self._state.selected_mode))

    def apply_packet(self, packet: FOCDataPacket):
        if getattr(packet, "phase_current_only", False):
            self._state.last_packet_received_at_ms = int(time.monotonic() * 1000)
            self._plot_buffer.append_packet(packet)
            self._request_plot_refresh()
            self.export_plot_button.setEnabled(self._plot_buffer.has_rows())
            self._refresh_session_status()
            return

        was_identify_active = bool(self._state.identify_active)
        apply_packet_effects(self._state, packet)
        self._state.last_packet_received_at_ms = int(time.monotonic() * 1000)
        encoder_offline = (self._state.encoder_detected is False)
        if encoder_offline != self._last_encoder_warning_state:
            self._last_encoder_warning_state = encoder_offline
            if encoder_offline:
                warning = "警告：未检测到 TLE5012 编码器；如需继续使能，请确认是否进入堵转模式。"
                self._record_identify_event(warning)
                self._show_notification("ERROR", warning)
            elif self._state.encoder_detected is True:
                self._record_identify_event("TLE5012 编码器已恢复在线。")
        snapshot = packet_snapshot(packet)
        fault = fault_summary_text(packet)
        self.packet_timestamp_value.setText(snapshot["timestamp"])
        self.state_value.setText(snapshot["state"])
        self.angle_value.setText(snapshot["angle"])
        self.speed_value.setText(snapshot["speed"])
        self.currents_value.setText(snapshot["currents"])
        self.refs_value.setText(snapshot["refs"])
        self.voltages_value.setText(snapshot["voltages"])
        self.fault_state_value.setText(fault["state"])
        self.fault_registers_value.setText(f'{fault["fault1"]} | {fault["vgs2"]}')
        self.fault_timestamp_value.setText(fault["timestamp"])
        fault_active = packet_has_active_fault(packet)
        warning_active = getattr(packet, "app_warning_flags", 0) != 0
        if fault_active:
            self.fault_state_value.setStyleSheet("color: #b91c1c; font-weight: 700;")
        elif warning_active:
            self.fault_state_value.setStyleSheet("color: #b45309; font-weight: 700;")
        else:
            self.fault_state_value.setStyleSheet("color: #166534; font-weight: 700;")
        if packet.undervoltage_limit is not None and packet.overvoltage_limit is not None:
            if not self._handle_pending_vbus_limit_confirmation(packet):
                self.vbus_limit_actual_value.setText(
                    f"UV {packet.undervoltage_limit:.3f} V / OV {packet.overvoltage_limit:.3f} V"
                )
        else:
            if not self._handle_pending_vbus_limit_confirmation(packet):
                self.vbus_limit_actual_value.setText("固件当前：未上报阈值（请确认已烧录最新固件）")
        fault_log_entry = fault_packet_log_entry(packet)
        if fault_log_entry is not None:
            self._record_fault_log_entry(*fault_log_entry)
        self._plot_buffer.append_packet(packet)
        self._request_plot_refresh()
        self._refresh_identify_state_panel()
        if self._state.identify_active or fault_active:
            self._record_identify_event(
                f"{packet.timestamp} ms | 状态={packet.foc_state} | 故障={'故障激活' if fault_active else '正常'}"
            )
        self.export_plot_button.setEnabled(self._plot_buffer.has_rows())
        self._apply_control_enable_state()
        self._refresh_identify_state_panel()
        if was_identify_active and (not self._state.identify_active) and self._state.motor_identified:
            self._request_parameter_snapshot()
        self._refresh_mode_views()
        self._refresh_session_status()

    def handle_log_line(self, level: str, message: str):
        if level == "RX":
            self._last_rx_activity_at_ms = int(time.monotonic() * 1000)
        self._log_entries.append((level, message))
        if len(self._log_entries) > self.LOG_LIMIT:
            self._log_entries = self._log_entries[-self.LOG_LIMIT :]

        fault_log_entry = fault_error_log_entry(level, message)
        if fault_log_entry is not None:
            self._record_fault_log_entry(*fault_log_entry)

        if level == "TX":
            apply_command_effects(self._state, message)
            if message.startswith("CMD:IDENTIFY"):
                self._record_identify_event(f"命令：{message}")
            elif message.startswith("CMD:CLEAR_FAULT"):
                self._record_identify_event("已发送清故障命令。")
            self._show_notification("INFO", f"已发送 {message}")
        elif level == "RX":
            adc_noise_result = parse_adc_noise_response(message)
            if adc_noise_result is not None:
                self.adc_noise_result_value.setText(adc_noise_result.display_text)
                if adc_noise_result.ok:
                    self.adc_noise_result_value.setStyleSheet(
                        "color: #0f766e; font-family: Consolas, 'Microsoft YaHei UI';"
                    )
                else:
                    self.adc_noise_result_value.setStyleSheet(
                        "color: #b45309; font-family: Consolas, 'Microsoft YaHei UI';"
                    )
                self._show_notification("INFO" if adc_noise_result.ok else "ERROR", adc_noise_result.display_text)
        elif level == "ERROR":
            self._record_identify_event(message)
            self._show_notification("ERROR", message)
        elif level == "INFO" and (("已连接" in message) or ("已断开" in message)):
            self._show_notification("INFO", message)

        self._request_log_refresh()
        self._refresh_identify_state_panel()
        self._refresh_session_status()
        self.update_connection_state(self._state.is_connected)

    def _request_connect(self):
        port = self.port_combo.currentText().strip()
        if not port:
            self._show_notification("ERROR", "未选择串口。")
            self.handle_log_line("ERROR", "未选择串口。")
            return
        self.connect_requested.emit(port, int(self.baud_combo.currentText()))

    def _persist_profile_from_widgets(self, *_, force: bool = False):
        if self._loading_profile and not force:
            return
        self._profile.last_port = self.port_combo.currentText().strip()
        try:
            self._profile.baud_rate = int(self.baud_combo.currentText())
        except ValueError:
            self._profile.baud_rate = 230400
        self._profile.selected_mode = self._state.selected_mode
        self._profile.log_filters = [
            level for level, checkbox in self.log_filter_checks.items() if checkbox.isChecked()
        ] or list(LOG_LEVELS)
        self._profile.undervoltage_limit = self._float_or_default(self.vbus_uv_input.text(), 18.0)
        self._profile.overvoltage_limit = self._float_or_default(self.vbus_ov_input.text(), 28.0)
        self._profile.motor_pn = int(self._float_or_default(self.motor_pn_input.text(), 11.0))
        self._profile.current_target = (
            self._float_or_default(self.current_id_input.text()),
            self._float_or_default(self.current_iq_input.text()),
        )
        self._profile.speed_target = self._float_or_default(self.speed_ref_input.text())
        self._profile.position_target = self._float_or_default(self.position_ref_input.text())
        self._profile.current_pi = LoopTuning(
            kp=self._float_or_default(self.current_pi_kp_input.text()),
            ki=self._float_or_default(self.current_pi_ki_input.text()),
        )
        self._profile.speed_pi = LoopTuning(
            kp=self._float_or_default(self.speed_pi_kp_input.text()),
            ki=self._float_or_default(self.speed_pi_ki_input.text()),
        )
        self._profile.position_pd = PositionLoopTuning(
            kp=self._float_or_default(self.position_pd_kp_input.text()),
            kd=self._float_or_default(self.position_pd_kd_input.text()),
        )
        save_gui_profile(self._profile_path, self._profile)

    def _float_or_default(self, raw_value: str, default: float = 0.0) -> float:
        try:
            return float(str(raw_value).strip())
        except ValueError:
            return default

    def _dispatch_command(self, command: str, identify_note: str | None = None):
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "当前未连接，命令已拦截。")
            return
        if identify_note:
            self._record_identify_event(identify_note)
        self.command_requested.emit(command)

    def _request_parameter_snapshot(self):
        if not self._state.is_connected:
            return
        self.command_requested.emit(CommandBuilder.fault_detail())

    def _ensure_motion_target_ready(self) -> bool:
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "当前未连接，命令已拦截。")
            return False
        if (not self._state.power_unlocked) or (not self._state.motor_enabled):
            self.handle_log_line("ERROR", "当前电机未解锁或未使能，请先解锁并使能后再下发目标值。")
            return False
        return True

    def _dispatch_sequence(self, commands: list[str]):
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "当前未连接，命令已拦截。")
            return
        for command in commands:
            self.command_requested.emit(command)

    def _request_enable_motor(self):
        commands = self._build_enable_sequence(include_unlock=False)
        if commands is not None:
            self._dispatch_sequence(commands)

    def _request_quick_arm(self):
        commands = self._build_enable_sequence(include_unlock=True)
        if commands is not None:
            self._dispatch_sequence(commands)

    def _build_enable_sequence(self, include_unlock: bool) -> list[str] | None:
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "当前未连接，命令已拦截。")
            return None

        commands: list[str] = []
        if include_unlock:
            commands.append(CommandBuilder.unlock_power(True))

        requires_stall_mode = (not self._state.motor_identified) or (self._state.encoder_detected is False)
        if requires_stall_mode:
            needs_confirm = should_confirm_stall_mode_enable(self._state)
            if include_unlock and not self._state.power_unlocked:
                needs_confirm = (
                    self._state.is_connected
                    and not self._state.motor_enabled
                    and not self._state.identify_active
                    and not self._state.fault_active
                    and requires_stall_mode
                    and not self._state.stall_mode_armed
                )

            if needs_confirm:
                if not self._confirm_stall_mode_enable():
                    return None
                commands.append(CommandBuilder.set_stall_mode(True))

            commands.append(CommandBuilder.set_mode(1))

        commands.append(CommandBuilder.enable_motor(True))
        return commands

    def _confirm_stall_mode_enable(self) -> bool:
        result = QMessageBox.question(
            self,
            "进入堵转模式",
            stall_mode_confirmation_text(self._state),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return result == QMessageBox.StandardButton.Yes

    def _apply_current_refs(self):
        try:
            command = build_current_ref_command(self.current_id_input.text(), self.current_iq_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return
        if not self._ensure_motion_target_ready():
            return
        self.current_id_input.setText(f"{self._float_or_default(self.current_id_input.text()):.3f}")
        self.current_iq_input.setText(f"{self._float_or_default(self.current_iq_input.text()):.3f}")
        if self._state.selected_mode != 0:
            self.apply_mode_selection(0)
        self._dispatch_command(command)

    def _apply_speed_ref(self):
        try:
            command = build_speed_ref_command(self.speed_ref_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return
        if not self._ensure_motion_target_ready():
            return
        self.speed_ref_input.setText(f"{self._float_or_default(self.speed_ref_input.text()):.3f}")
        if self._state.selected_mode != 1:
            self.apply_mode_selection(1)
        self._dispatch_command(command)

    def _apply_position_ref(self):
        try:
            command = build_position_ref_command(self.position_ref_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return
        if not self._ensure_motion_target_ready():
            return
        self.position_ref_input.setText(f"{self._float_or_default(self.position_ref_input.text()):.3f}")
        if self._state.selected_mode != 2:
            self.apply_mode_selection(2)
        self._dispatch_command(command)

    def _apply_vbus_limits(self):
        try:
            command = build_vbus_limit_command(self.vbus_uv_input.text(), self.vbus_ov_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return

        uv = self._float_or_default(self.vbus_uv_input.text())
        ov = self._float_or_default(self.vbus_ov_input.text())
        self.vbus_uv_input.setText(f"{uv:.3f}")
        self.vbus_ov_input.setText(f"{ov:.3f}")
        self._pending_vbus_limit_request = (uv, ov)
        self._pending_vbus_limit_requested_at_ms = int(time.monotonic() * 1000)
        self.vbus_limit_actual_value.setText(
            f"等待固件确认：目标 UV {uv:.3f} V / OV {ov:.3f} V"
        )
        self._dispatch_command(command)
        self._persist_profile_from_widgets(force=True)
        self._show_notification("INFO", f"已发送电压阈值更新，等待固件确认：UV={uv:.3f}V, OV={ov:.3f}V。")

    def _apply_motor_pn(self):
        try:
            command = build_motor_pn_command(self.motor_pn_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return

        pole_pairs = int(self._float_or_default(self.motor_pn_input.text(), 11.0))
        self.motor_pn_input.setText(str(pole_pairs))
        self._dispatch_command(
            command,
            identify_note=f"已设置电机极对数 Pn={pole_pairs}；参数识别会重新验证方向和零位。",
        )
        self._show_notification("INFO", f"已发送电机极对数：Pn={pole_pairs}。")

    def _apply_encoder_dir(self):
        idx = self.encoder_dir_combo.currentIndex()
        direction = -1 if idx == 0 else 1
        command = CommandBuilder.set_encoder_dir(direction)
        self._state.encoder_dir = direction
        self._dispatch_command(
            command,
            identify_note=f"已设置编码器方向：{direction}",
        )
        self._show_notification("INFO", f"已发送编码器方向：{direction}。")

    def _request_start_identify(self):
        """发送完整的识别序列: MOTOR_PN → ENCODER_DIR → UNLOCK → IDENTIFY"""
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "当前未连接，命令已拦截。")
            return

        # Step 1: Set motor pole pairs
        try:
            pn_cmd = build_motor_pn_command(self.motor_pn_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", f"极对数无效: {exc}")
            return
        self._record_identify_event("识别流程：设置极对数...")
        self.command_requested.emit(pn_cmd)

        # Step 2: Set encoder direction
        idx = self.encoder_dir_combo.currentIndex()
        direction = -1 if idx == 0 else 1
        self._state.encoder_dir = direction
        self._record_identify_event(f"识别流程：设置编码器方向={direction}...")
        self.command_requested.emit(CommandBuilder.set_encoder_dir(direction))

        # Step 3: Unlock power stage
        self._record_identify_event("识别流程：解锁功率级...")
        self.command_requested.emit(CommandBuilder.unlock_power(True))

        # Step 4: Start identify
        self._record_identify_event("识别流程：开始识别...")
        self.command_requested.emit(CommandBuilder.start_identify())

        self._show_notification("INFO", "已发送识别序列：Pn → 编码器方向 → 解锁 → 开始识别。")

    def _send_home(self):
        self._dispatch_command(CommandBuilder.set_home())
        self._show_notification("INFO", "已发送设置零点命令，等待固件确认...")

    def _send_clear_home(self):
        self._dispatch_command(CommandBuilder.clear_home())
        self.home_offset_value.setText("0.000 rad")
        self._show_notification("INFO", "已清除机械零点偏移。")

    def _run_adc_noise_test(self):
        try:
            command = build_adc_noise_command(self.adc_noise_samples_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return

        samples = int(self._float_or_default(self.adc_noise_samples_input.text(), 4096.0))
        self.adc_noise_samples_input.setText(str(samples))
        self.adc_noise_result_value.setText(f"ADC噪声：已请求 {samples} 点，等待固件返回...")
        self.adc_noise_result_value.setStyleSheet(
            "color: #475569; font-family: Consolas, 'Microsoft YaHei UI';"
        )
        self._dispatch_command(command)

    def _apply_loop_gains(self, loop_name: str):
        inputs = {
            "current": (self.current_pi_kp_input, self.current_pi_ki_input),
            "speed": (self.speed_pi_kp_input, self.speed_pi_ki_input),
            "position": (self.position_pd_kp_input, self.position_pd_kd_input),
        }
        kp_widget, gain2_widget = inputs[loop_name]
        try:
            command = build_loop_gain_command(loop_name, kp_widget.text(), gain2_widget.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return
        kp_widget.setText(f"{self._float_or_default(kp_widget.text()):.6f}")
        gain2_widget.setText(f"{self._float_or_default(gain2_widget.text()):.6f}")
        self._dispatch_command(command)

    def _apply_loop_defaults(self, loop_name: str):
        defaults = {
            "current": self._profile.current_pi,
            "speed": self._profile.speed_pi,
            "position": self._profile.position_pd,
        }
        self._set_loop_inputs(loop_name, defaults[loop_name])
        gain_kind = "PD" if loop_name == "position" else "PI"
        loop_text = {
            "current": "电流环",
            "speed": "速度环",
            "position": "位置环",
        }[loop_name]
        self._show_notification("INFO", f"已从本地预设加载{loop_text} {gain_kind} 默认值。")

    def _save_local_preset(self):
        self._persist_profile_from_widgets(force=True)
        self._show_notification("INFO", f"已保存本地预设到 {self._profile_path}。")

    def _load_preset_fields(self):
        self._profile = load_gui_profile(self._profile_path)
        self._load_profile_into_widgets()
        self.apply_mode_selection(self._profile.selected_mode, emit_command=False)
        self._show_notification("INFO", f"已从 {self._profile_path} 加载本地预设。")

    def _clear_fault_with_hint(self):
        self._dispatch_command(CommandBuilder.clear_fault(), identify_note="已请求清除故障。")
        if self._state.is_connected:
            self._show_notification("INFO", "已发送清故障命令。若硬件已恢复，可重新解锁并使能。")

    def _on_log_filter_changed(self):
        self._render_logs()
        self._persist_profile_from_widgets()

    def _request_log_refresh(self):
        self._log_refresh_pending = True

    def _flush_pending_log_refresh(self):
        if not self._log_refresh_pending:
            return
        self._log_refresh_pending = False
        self._render_logs()

    def _render_logs(self):
        enabled = {level for level, checkbox in self.log_filter_checks.items() if checkbox.isChecked()}
        visible = [log_line_text(level, message) for level, message in self._log_entries if level in enabled]
        self.log_view.setPlainText("\n".join(visible))

    def _record_fault_log_entry(self, signature: str, text: str):
        if not signature or not text or signature in self._fault_log_signatures:
            return
        self._fault_log_signatures.add(signature)
        self._fault_log_entries.append(text)
        self._render_fault_log()

    def _render_fault_log(self):
        self.fault_log_view.setPlainText("\n\n".join(self._fault_log_entries))

    def _record_identify_event(self, text: str):
        self._identify_entries.append(text)
        if len(self._identify_entries) > self.IDENTIFY_LOG_LIMIT:
            self._identify_entries = self._identify_entries[-self.IDENTIFY_LOG_LIMIT :]
        self._render_identify_log()

    def _render_identify_log(self):
        self.identify_log_view.setPlainText("\n".join(self._identify_entries))

    def _copy_recent_log(self):
        QApplication.clipboard().setText(self.log_view.toPlainText())
        self._show_notification("INFO", "已复制当前可见日志到剪贴板。")

    def _copy_fault_log(self):
        QApplication.clipboard().setText(self.fault_log_view.toPlainText())
        self._show_notification("INFO", "已复制故障日志到剪贴板。")

    def _clear_log(self):
        self._log_entries.clear()
        self._render_logs()
        self._show_notification("INFO", "已清空串口日志视图。")

    def _clear_fault_log(self):
        self._fault_log_entries.clear()
        self._fault_log_signatures.clear()
        self._render_fault_log()
        self._show_notification("INFO", "已清空故障日志视图。")

    def _sync_connect_button(self):
        self.connect_button.setEnabled((not self._state.is_connected) and self.port_combo.count() > 0)

    def _sync_mode_buttons(self, mode: int):
        for candidate_mode, buttons in self._mode_button_map.items():
            for button in buttons:
                button.blockSignals(True)
                button.setChecked(candidate_mode == mode)
                button.blockSignals(False)

    def _refresh_mode_views(self):
        self._sync_mode_buttons(self._state.selected_mode)
        self.session_mode_value.setText(mode_name(self._state.selected_mode))
        self.target_label.setText(mode_target_label(self._state.selected_mode))
        self.advanced_mode_value.setText(f"{mode_name(self._state.selected_mode)}模式已激活")
        self._refresh_mode_highlight()

    def _refresh_mode_highlight(self):
        groups = {
            0: (self.current_group, "力矩模式"),
            1: (self.speed_group, "速度模式"),
            2: (self.position_group, "位置模式"),
        }
        for mode, (group, title) in groups.items():
            active = mode == self._state.selected_mode
            group.setTitle(f"{title}（当前）" if active else title)
            group.setStyleSheet(
                "QGroupBox { border: 2px solid #0f766e; margin-top: 8px; }"
                if active
                else "QGroupBox { border: 1px solid #cbd5e1; margin-top: 8px; }"
            )

    def _refresh_identify_state_panel(self):
        packet = self._state.last_packet
        self.identify_connection_value.setText("已连接" if self._state.is_connected else "未连接")
        self.identify_power_value.setText("已解锁" if self._state.power_unlocked else "已上锁")
        self.identify_state_value.setText(str(packet.foc_state) if packet else "--")
        self.identify_result_value.setText("已识别" if self._state.motor_identified else "未识别")
        if self._state.encoder_detected is None:
            encoder_text = "未知"
        else:
            encoder_text = "在线" if self._state.encoder_detected else "未检测到"
        self.identify_encoder_value.setText(encoder_text)
        self.identify_encoder_value.setStyleSheet(
            "color: #166534; font-weight: 700;"
            if encoder_text == "在线"
            else ("color: #b91c1c; font-weight: 700;" if encoder_text == "未检测到" else "")
        )
        self.identify_stall_mode_value.setText("已授权" if self._state.stall_mode_armed else "未授权")
        self.identify_stall_open_loop_value.setText("激活中" if self._state.stall_open_loop_active else "未激活")
        self.identify_stall_open_loop_value.setStyleSheet(
            "color: #0f766e; font-weight: 700;"
            if self._state.stall_open_loop_active
            else "color: #64748b; font-weight: 600;"
        )
        if packet is None:
            fault_text = "未知"
        else:
            fault_text = "故障激活" if packet_has_active_fault(packet) else "正常"
        self.identify_fault_value.setText(fault_text)
        self.identify_fault_value.setStyleSheet(
            "color: #b91c1c; font-weight: 700;" if fault_text == "故障激活" else "color: #166534; font-weight: 700;"
        )
        rs_value = packet.motor_param_rs if packet and packet.motor_param_rs is not None else self.KNOWN_MOTOR_RS_OHM
        ld_value = packet.motor_param_ld if packet and packet.motor_param_ld is not None else self.KNOWN_MOTOR_LD_H
        lq_value = packet.motor_param_lq if packet and packet.motor_param_lq is not None else self.KNOWN_MOTOR_LQ_H
        ke_value = packet.motor_param_ke if packet and packet.motor_param_ke is not None else self.KNOWN_MOTOR_KE
        pn_value = packet.motor_param_pn if packet and packet.motor_param_pn is not None else self._profile.motor_pn
        encoder_dir_value = (
            packet.motor_param_encoder_dir
            if packet and packet.motor_param_encoder_dir is not None
            else self.KNOWN_MOTOR_ENCODER_DIR
        )

        self.motor_param_rs_value.setText(f"{rs_value:.3f}")
        self.motor_param_ld_value.setText(f"{ld_value:.6f}")
        self.motor_param_lq_value.setText(f"{lq_value:.6f}")
        self.motor_param_ke_value.setText(f"{ke_value:.6f}")
        self.motor_param_pn_value.setText(str(int(pn_value)))
        self.motor_param_encoder_dir_value.setText(str(int(encoder_dir_value)))

        theta_offset = (
            packet.motor_param_theta_offset
            if packet and packet.motor_param_theta_offset is not None
            else self.KNOWN_MOTOR_THETA_OFFSET
        )
        mech_zero = (
            packet.motor_param_mech_zero
            if packet and packet.motor_param_mech_zero is not None
            else self.KNOWN_MOTOR_THETA_ZERO
        )
        self.motor_param_theta_offset_value.setText(f"{theta_offset:.6f}")
        self.motor_param_theta_zero_value.setText(f"{mech_zero:.6f}")
        self.home_offset_value.setText(f"{mech_zero:.6f} rad")

    def _clear_runtime_snapshot(self):
        self.packet_timestamp_value.setText("--")
        self.state_value.setText("--")
        self.angle_value.setText("--")
        self.speed_value.setText("--")
        self.currents_value.setText("--")
        self.refs_value.setText("--")
        self.voltages_value.setText("--")
        self.fault_state_value.setText("正常")
        self.fault_registers_value.setText("FAULT1 0x0000 | VGS2 0x0000")
        self.fault_timestamp_value.setText("--")
        self.vbus_limit_actual_value.setText("固件当前：UV -- / OV --")
        self.fault_state_value.setStyleSheet("color: #166534; font-weight: 700;")

    def _handle_pending_vbus_limit_confirmation(self, packet: FOCDataPacket) -> bool:
        pending = self._pending_vbus_limit_request
        if pending is None:
            return False

        now_ms = int(time.monotonic() * 1000)
        wait_start_ms = self._pending_vbus_limit_requested_at_ms
        timed_out = (
            wait_start_ms is not None and
            (now_ms - wait_start_ms) >= self.VBUS_LIMIT_CONFIRM_TIMEOUT_MS
        )

        if packet.undervoltage_limit is None or packet.overvoltage_limit is None:
            if timed_out:
                self._pending_vbus_limit_request = None
                self._pending_vbus_limit_requested_at_ms = None
                self.vbus_limit_actual_value.setText("阈值未生效：固件未回传新的 UV / OV 阈值")
                self._show_notification("ERROR", "电压阈值未生效：固件未回传新的 UV / OV 阈值。")
                return True
            return False

        uv_target, ov_target = pending
        uv_match = abs(packet.undervoltage_limit - uv_target) <= self.VBUS_LIMIT_CONFIRM_EPSILON_V
        ov_match = abs(packet.overvoltage_limit - ov_target) <= self.VBUS_LIMIT_CONFIRM_EPSILON_V

        if uv_match and ov_match:
            self._pending_vbus_limit_request = None
            self._pending_vbus_limit_requested_at_ms = None
            self.vbus_limit_actual_value.setText(
                f"UV {packet.undervoltage_limit:.3f} V / OV {packet.overvoltage_limit:.3f} V"
            )
            self._show_notification(
                "INFO",
                f"固件确认电压阈值已更新：UV={packet.undervoltage_limit:.3f}V, OV={packet.overvoltage_limit:.3f}V。"
            )
            return True

        if timed_out:
            self._pending_vbus_limit_request = None
            self._pending_vbus_limit_requested_at_ms = None
            self.vbus_limit_actual_value.setText(
                f"阈值未生效：固件仍为 UV {packet.undervoltage_limit:.3f} V / OV {packet.overvoltage_limit:.3f} V"
            )
            self._show_notification(
                "ERROR",
                f"电压阈值未生效：固件仍回传 UV={packet.undervoltage_limit:.3f}V, OV={packet.overvoltage_limit:.3f}V。"
            )
            return True

        self.vbus_limit_actual_value.setText(
            f"等待固件确认：目标 UV {uv_target:.3f} V / OV {ov_target:.3f} V"
        )
        return True

    def _refresh_session_status(self):
        if not self._state.is_connected:
            self.data_status_value.setText("空闲")
            self.data_status_value.setStyleSheet("color: #64748b; font-weight: 600;")
            return
        now_ms = int(time.monotonic() * 1000)
        rx_recent = (
            self._last_rx_activity_at_ms is not None and
            not is_data_stale(self._last_rx_activity_at_ms, now_ms, self.STALE_THRESHOLD_MS)
        )
        if self._state.last_packet_received_at_ms is None:
            if rx_recent:
                self.data_status_value.setText("串口活跃，等待解析")
                self.data_status_value.setStyleSheet("color: #b45309; font-weight: 700;")
            else:
                self.data_status_value.setText("等待数据包")
                self.data_status_value.setStyleSheet("color: #b45309; font-weight: 600;")
            return
        stale = is_data_stale(self._state.last_packet_received_at_ms, now_ms, self.STALE_THRESHOLD_MS)
        if stale:
            if rx_recent:
                self.data_status_value.setText("串口活跃，数据包未刷新")
                self.data_status_value.setStyleSheet("color: #b45309; font-weight: 700;")
            else:
                self.data_status_value.setText("数据过期")
                self.data_status_value.setStyleSheet("color: #b91c1c; font-weight: 700;")
        else:
            self.data_status_value.setText("正常")
            self.data_status_value.setStyleSheet("color: #0f766e; font-weight: 700;")

    def _refresh_plot(self, *_):
        if pg is None:
            return
        for channel, curve in self._plot_curves.items():
            checkbox = self.plot_channel_checks[channel]
            if checkbox.isChecked():
                times, values = self._plot_buffer.series(channel)
                curve.setData(times, values)
            else:
                curve.setData([], [])

    def _request_plot_refresh(self):
        self._plot_refresh_pending = True

    def _flush_pending_plot_refresh(self):
        if not self._plot_refresh_pending:
            return
        self._plot_refresh_pending = False
        self._refresh_plot()

    def _on_tab_changed(self, index: int):
        if index == 1:
            self._plot_refresh_timer.start()
            self._flush_pending_plot_refresh()
        else:
            self._plot_refresh_timer.stop()

    def _export_plot_csv(self):
        rows = self._plot_buffer.export_rows(self._selected_plot_channels())
        if not rows:
            self._show_notification("ERROR", "当前没有可导出的曲线采样。")
            return
        save_path, _ = QFileDialog.getSaveFileName(
            self,
            "导出曲线 CSV",
            str(Path.home() / "foc_runtime_samples.csv"),
            "CSV 文件 (*.csv)",
        )
        if not save_path:
            return
        Path(save_path).write_text(format_plot_csv(rows), encoding="utf-8")
        self._show_notification("INFO", f"已导出曲线采样到 {save_path}。")

    def _selected_plot_channels(self) -> list[str]:
        selected = [channel for channel, checkbox in self.plot_channel_checks.items() if checkbox.isChecked()]
        return selected or list(PLOT_CHANNELS)

    def _show_notification(self, level: str, message: str):
        styles = {
            "INFO": "color: #0f766e; font-weight: 600;",
            "ERROR": "color: #b91c1c; font-weight: 700;",
        }
        prefix = "错误" if level == "ERROR" else "信息"
        text = f"{prefix}: {message}"
        self.notification_label.setText(text)
        self.notification_label.setStyleSheet(f"padding: 0 8px; {styles.get(level, styles['INFO'])}")
        self.statusBar().showMessage(text, 8000)
