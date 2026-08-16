from __future__ import annotations

import math
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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
    QScrollArea,
    QSlider,
    QSpinBox,
    QStackedWidget,
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
    from .data_parser import CommandBuilder, FOCDataPacket, AckResult, AckParser
    from .gui_logic import (
        APP_MODE_CN,
        APP_MODE_TOKENS,
        DEFAULT_PROFILE_PATH,
        LOG_LEVELS,
        PLOT_CHANNELS,
        ACK_TIMEOUT_MS,
        FOC_STATE_FAULT,
        GuiProfile,
        HostAppState,
        LoopTuning,
        PositionLoopTuning,
        RollingPlotBuffer,
                is_app_mode_synced,
                get_app_mode_prerequisites,
                build_app_enable_sequence,
                build_app_arm_sequence,
                build_app_target_sequence,
                build_app_config_sequence,
                start_command_sequence,
                advance_command_sequence,
                abort_command_sequence,
        is_app_mode_synced,
        get_app_mode_prerequisites,
        build_app_enable_sequence,
        build_app_arm_sequence,
        build_app_target_sequence,
        build_app_config_sequence,
        start_command_sequence,
        advance_command_sequence,
        abort_command_sequence,
        app_mode_cn,
        apply_command_effects,
        apply_ack_effects,
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
        can_dispatch_command,
        can_edit_vbus_limits,
        is_safe_fallback_command,
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
    from data_parser import CommandBuilder, FOCDataPacket, AckResult, AckParser
    from gui_logic import (
        APP_MODE_CN,
        APP_MODE_TOKENS,
        DEFAULT_PROFILE_PATH,
        LOG_LEVELS,
        PLOT_CHANNELS,
        ACK_TIMEOUT_MS,
        FOC_STATE_FAULT,
        GuiProfile,
        HostAppState,
        LoopTuning,
        PositionLoopTuning,
        RollingPlotBuffer,
        is_app_mode_synced,
        get_app_mode_prerequisites,
        build_app_enable_sequence,
        build_app_arm_sequence,
        build_app_target_sequence,
        build_app_config_sequence,
        start_command_sequence,
        advance_command_sequence,
        abort_command_sequence,
        app_mode_cn,
        apply_command_effects,
        apply_ack_effects,
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
        can_dispatch_command,
        can_edit_vbus_limits,
        is_safe_fallback_command,
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
    command_requested = Signal(str)
    refresh_ports_requested = Signal()
    connect_requested = Signal(str, int)
    disconnect_requested = Signal()
    bridge_wheel_enable_requested = Signal(bool)

    LOG_LIMIT = 400
    IDENTIFY_LOG_LIMIT = 200
    STALE_THRESHOLD_MS = 1500
    VBUS_LIMIT_CONFIRM_TIMEOUT_MS = 600
    VBUS_LIMIT_CONFIRM_EPSILON_V = 0.01
    PLOT_REFRESH_INTERVAL_MS = 50
    CURRENT_STREAM_PLOT_MAX_POINTS = 3000
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
        self._state = HostAppState(control_mode=self._profile.selected_mode)
        self._serial_worker = None
        self._ack_parser = AckParser()
        self._loading_profile = False
        self._log_entries: list[tuple[str, str]] = []
        self._fault_log_entries: list[str] = []
        self._fault_log_signatures: set[str] = set()
        self._identify_entries: list[str] = []
        self._plot_buffer = RollingPlotBuffer()
        self._plot_curves: dict[str, object] = {}
        self._plot_legend = None
        self._plot_legend_channels: frozenset[str] = frozenset()
        self._mode_button_map: dict[int, list[QRadioButton]] = {0: [], 1: [], 2: []}
        self._last_encoder_warning_state: bool | None = None
        self._last_rx_activity_at_ms: int | None = None
        self._last_cmd_sent: str | None = None   # V1.2: last TX command for diagnostics
        self._pending_vbus_limit_request: tuple[float, float] | None = None
        self._pending_vbus_limit_requested_at_ms: int | None = None
        self._plot_refresh_pending = False
        self._log_refresh_pending = False
        self._cur_stats_prev_at_s: float | None = None

        self.setWindowTitle("FOC 上位机调试工具")
        self.resize(1500, 920)

        # ── Scope session state ──
        self._scope_enabled: bool = False
        self._scope_paused: bool = False          # V1.2: freeze display without stopping acquisition
        self._scope_start_timestamp: float | None = None
        self._scope_start_seq: int | None = None
        self._scope_start_tick_ms: int | None = None
        self._scope_window_s: float = 5.0
        self._scope_follow_latest: bool = True
        self._scope_auto_y: bool = True
        self._scope_suppress_range_change: bool = False
        self._scope_ignore_manual_range_until_ms: float = 0.0

        self._build_toolbar()
        self._build_status_bar()

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._scrollable_tab(self._build_debug_panel()), "控制器参数")
        self.tabs.addTab(self._build_chart_tab(), "实时波形")
        self.tabs.addTab(self._scrollable_tab(self._build_identify_tab()), "参数识别")
        self.tabs.addTab(self._scrollable_tab(self._build_advanced_control_tab()), "高级控制")
        self.tabs.addTab(self._scrollable_tab(self._build_pi_tab()), "环路参数")
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

        # Current stream stats refresh (20Hz)
        self._cur_stream_stats_timer = QTimer(self)
        self._cur_stream_stats_timer.setInterval(50)
        self._cur_stream_stats_timer.timeout.connect(self._refresh_cur_stream_stats)
        # Not started until stream is active

        self._load_profile_into_widgets()
        self.apply_mode_selection(self._state.control_mode, emit_command=False)
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
        self.baud_combo.addItems(["1000000", "921600", "460800", "230400", "115200"])
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

    def _scrollable_tab(self, widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setWidget(widget)
        return scroll

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
        self.power_status_label = QLabel("未解锁 | 未使能")
        self.power_status_label.setStyleSheet("color: #64748b; font-weight: 600;")
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
        power_layout.addWidget(self.power_status_label)
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
        self.session_mode_value = QLabel(mode_name(self._state.control_mode))
        session_layout.addRow("连接状态", self.connection_state_detail_value)
        session_layout.addRow("数据新鲜度", self.data_status_value)
        session_layout.addRow("当前模式", self.session_mode_value)
        session_layout.addRow("最近数据包", self.packet_timestamp_value)
        layout.addWidget(session_group)

        # V1.2: Link diagnostics
        link_group = QGroupBox("链路诊断")
        link_layout = QFormLayout(link_group)
        self.last_cmd_sent_value = QLabel("--")
        self.last_ack_value = QLabel("--")
        self.last_nframe_value = QLabel("--")
        self.current_pending_value = QLabel("--")
        self.cur_link_diag_value = QLabel("--")
        link_layout.addRow("上次命令", self.last_cmd_sent_value)
        link_layout.addRow("上次ACK", self.last_ack_value)
        link_layout.addRow("上次N帧", self.last_nframe_value)
        link_layout.addRow("待确认", self.current_pending_value)
        link_layout.addRow("电流流状态", self.cur_link_diag_value)
        layout.addWidget(link_group)
        return widget

    def _build_chart_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(8)

        # ── Scope controls ──
        scope_bar = QHBoxLayout()
        self.scope_toggle_button = QPushButton("开始波形")
        self.scope_toggle_button.setCheckable(True)
        self.scope_toggle_button.setChecked(False)
        self.scope_toggle_button.toggled.connect(self._on_scope_toggled)
        scope_bar.addWidget(self.scope_toggle_button)

        scope_bar.addWidget(QLabel("  窗口"))
        self.scope_window_combo = QComboBox()
        self.scope_window_combo.addItems(["1s", "2s", "5s", "10s", "30s"])
        self.scope_window_combo.setCurrentText("5s")
        self.scope_window_combo.currentTextChanged.connect(self._on_scope_window_changed)
        scope_bar.addWidget(self.scope_window_combo)

        self.scope_back_to_latest_button = QPushButton("回到最新")
        self.scope_back_to_latest_button.clicked.connect(self._on_scope_wake)
        scope_bar.addWidget(self.scope_back_to_latest_button)

        self.scope_follow_check = QCheckBox("跟随最新")
        self.scope_follow_check.setChecked(True)
        self.scope_follow_check.toggled.connect(self._on_scope_follow_toggled)
        scope_bar.addWidget(self.scope_follow_check)

        self.scope_auto_y_check = QCheckBox("自动 Y")
        self.scope_auto_y_check.setChecked(True)
        self.scope_auto_y_check.toggled.connect(self._on_scope_auto_y_toggled)
        scope_bar.addWidget(self.scope_auto_y_check)

        # (V1.2 controls added below)

        # V1.2: plot pause — freeze display without stopping acquisition
        self.scope_pause_check = QCheckBox("绘图暂停")
        self.scope_pause_check.setChecked(False)
        self.scope_pause_check.toggled.connect(self._on_scope_pause_toggled)
        scope_bar.addWidget(self.scope_pause_check)

        scope_bar.addStretch()

        # V1.2: mixed display indicator
        self.scope_mixed_label = QLabel("")
        self.scope_mixed_label.setStyleSheet(
            "color: #b45309; font-weight: 700; font-size: 13px;"
        )
        scope_bar.addWidget(self.scope_mixed_label)
        layout.addLayout(scope_bar)

        # ── Channel toggles ──
        toggles_layout = QGridLayout()
        self.plot_channel_checks: dict[str, QCheckBox] = {}
        for index, channel in enumerate(PLOT_CHANNELS):
            checkbox = QCheckBox(PLOT_CHANNEL_LABELS.get(channel, channel))
            checkbox.setChecked(channel in {"Ia", "Ib", "Ic"})
            checkbox.toggled.connect(self._refresh_plot)
            self.plot_channel_checks[channel] = checkbox
            toggles_layout.addWidget(checkbox, index // 4, index % 4)
        layout.addLayout(toggles_layout)

        # ── Current Stream Controls ──
        cur_stream_group = QGroupBox("采集开关 (Current Stream)")
        cur_stream_group.setCheckable(True)
        cur_stream_group.setChecked(False)
        cur_stream_group.toggled.connect(self._on_cur_stream_toggled)
        self.cur_stream_group = cur_stream_group

        cur_layout = QVBoxLayout(cur_stream_group)
        cur_row = QHBoxLayout()
        cur_row.addWidget(QLabel("模式"))
        self.cur_mode_combo = QComboBox()
        self.cur_mode_combo.addItems(["OFF", "ASCII 200Hz", "BIN 1kHz (推荐)", "BIN 2kHz (实验)"])
        self.cur_mode_combo.currentIndexChanged.connect(self._on_cur_mode_changed)
        cur_row.addWidget(self.cur_mode_combo)

        cur_row.addWidget(QLabel("  "))
        self.cur_stats_label = QLabel("rx: -- fps | gap: -- | CRC: -- | 填充: -- | baud: 1M")
        cur_row.addWidget(self.cur_stats_label, 1)
        cur_layout.addLayout(cur_row)
        self.cur_diag_label = QLabel("diag: --")
        self.cur_diag_label.setStyleSheet("color: #334155; font-family: Consolas, 'Microsoft YaHei UI';")
        cur_layout.addWidget(self.cur_diag_label)
        layout.addWidget(cur_stream_group)

        if pg is None:
            self.plot_widget = QLabel("当前环境未提供 pyqtgraph。")
            self.plot_widget.setWordWrap(True)
            layout.addWidget(self.plot_widget)
        else:
            self.plot_widget = pg.PlotWidget()
            self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
            self._plot_legend = self.plot_widget.addLegend(offset=(10, 10))
            self.plot_widget.setBackground("#f8fafc")
            self.plot_widget.setLabel("bottom", "Time (s)")
            self.plot_widget.setLabel("left", "Value")
            try:
                self.plot_widget.getAxis("left").enableAutoSIPrefix(False)
            except Exception:
                pass
            # Track user zoom/pan to disable follow-latest
            self.plot_widget.getViewBox().sigRangeChangedManually.connect(self._on_user_zoom_or_pan)
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
                width = 1 if channel in {"Ia", "Ib", "Ic", "Id", "Iq"} else 2
                pen = pg.mkPen(palette[channel], width=width)
                curve = self.plot_widget.plot([], [], pen=pen)
                try:
                    curve.setClipToView(True)
                    curve.setDownsampling(auto=True, mode="peak")
                except Exception:
                    pass
                self._plot_curves[channel] = curve

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

        # ── APP_MODE Selector ──
        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel("产品模式:"))
        self.app_mode_combo = QComboBox()
        for token in APP_MODE_TOKENS:
            self.app_mode_combo.addItem(APP_MODE_CN[token], token)
        self.app_mode_set_button = QPushButton("设置模式")
        self.app_mode_set_button.clicked.connect(self._on_app_mode_set)
        selector_row.addWidget(self.app_mode_combo)
        selector_row.addWidget(self.app_mode_set_button)

        self.advanced_mode_value = QLabel(mode_name(self._state.control_mode))
        self.advanced_mode_value.setStyleSheet("font-size: 15px; font-weight: 700; color: #0f766e;")
        selector_row.addWidget(self.advanced_mode_value)
        selector_row.addStretch()
        layout.addLayout(selector_row)

        # ── Quick Power Controls (V1.2) ──
        power_row = QHBoxLayout()
        self.app_unlock_button = QPushButton("解锁")
        self.app_unlock_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.unlock_power(True))
        )
        self.app_lock_button = QPushButton("上锁")
        self.app_lock_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.unlock_power(False))
        )
        self.app_enable_button = QPushButton("使能")
        self.app_enable_button.clicked.connect(self._on_app_enable)
        self.app_disable_button = QPushButton("禁用")
        self.app_disable_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.enable_motor(False))
        )
        self.app_arm_button = QPushButton("解锁并使能")
        self.app_arm_button.clicked.connect(self._on_app_arm)
        self.app_power_status = QLabel("未解锁 | 未使能")
        self.app_power_status.setStyleSheet("color: #64748b; font-weight: 600;")

        power_row.addWidget(QLabel("功率:"))
        power_row.addWidget(self.app_unlock_button)
        power_row.addWidget(self.app_lock_button)
        power_row.addWidget(self.app_enable_button)
        power_row.addWidget(self.app_disable_button)
        power_row.addWidget(self.app_arm_button)
        power_row.addWidget(self.app_power_status)
        power_row.addStretch()
        layout.addLayout(power_row)

        # ── Mode Panel Stack ──
        self.mode_panel_stack = QStackedWidget()
        self.mode_panel_stack.addWidget(self._build_raw_mode_panel())       # 0: RAW
        self.mode_panel_stack.addWidget(self._build_joint_pos_panel())     # 1: JOINT_POS
        self.mode_panel_stack.addWidget(self._build_gimbal_speed_panel())  # 2: GIMBAL_SPEED
        self.mode_panel_stack.addWidget(self._build_hold_panel())          # 3: HOLD
        self.mode_panel_stack.addWidget(self._build_spring_damper_panel()) # 4: SPRING_DAMPER
        self.mode_panel_stack.addWidget(self._build_detent_panel())        # 5: DETENT
        self.mode_panel_stack.addWidget(self._build_scroll_wheel_panel())  # 6: SCROLL_WHEEL
        self.app_mode_combo.currentIndexChanged.connect(self._on_app_mode_combo_changed)
        layout.addWidget(self.mode_panel_stack, stretch=1)

        # ── STOP Button (always visible on this tab) ──
        self.app_stop_button = QPushButton("⏹ STOP — 紧急停止（禁用并上锁）")
        self.app_stop_button.setMinimumHeight(40)
        self.app_stop_button.setStyleSheet(
            "QPushButton { background-color: #b91c1c; color: white; font-weight: 700; font-size: 14px; }"
            "QPushButton:hover { background-color: #dc2626; }"
            "QPushButton:disabled { background-color: #9ca3af; }"
        )
        self.app_stop_button.clicked.connect(
            lambda: self._dispatch_sequence([CommandBuilder.enable_motor(False), CommandBuilder.unlock_power(False)])
        )
        layout.addWidget(self.app_stop_button)

        # ── Shared: Protection Group ──
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

        # ── Shared: Home Group ──
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

    # ── Product Mode Panel Builders (V1.2) ──────────────────────────────────

    def _build_raw_mode_panel(self) -> QWidget:
        """RAW mode: legacy torque/speed/position controls in a collapsible group."""
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("原始控制")
        group.setCheckable(True)
        group.setChecked(False)
        inner = QVBoxLayout(group)

        # Control mode radio buttons
        raw_button_group = QButtonGroup(self)
        raw_mode_row = QHBoxLayout()
        self.advanced_torque_button = QRadioButton("力矩")
        self.advanced_speed_button = QRadioButton("速度")
        self.advanced_position_button = QRadioButton("位置")
        self._register_mode_button(self.advanced_torque_button, 0, raw_button_group)
        self._register_mode_button(self.advanced_speed_button, 1, raw_button_group)
        self._register_mode_button(self.advanced_position_button, 2, raw_button_group)
        raw_button_group.idClicked.connect(self.apply_mode_selection)
        raw_mode_row.addWidget(self.advanced_torque_button)
        raw_mode_row.addWidget(self.advanced_speed_button)
        raw_mode_row.addWidget(self.advanced_position_button)
        inner.addLayout(raw_mode_row)

        # Preset buttons
        preset_row = QHBoxLayout()
        self.load_target_preset_button = QPushButton("加载本地预设")
        self.save_target_preset_button = QPushButton("保存本地预设")
        self.load_target_preset_button.clicked.connect(self._load_preset_fields)
        self.save_target_preset_button.clicked.connect(self._save_local_preset)
        preset_row.addWidget(self.load_target_preset_button)
        preset_row.addWidget(self.save_target_preset_button)
        inner.addLayout(preset_row)

        # Torque target
        self.current_group = QGroupBox("力矩模式")
        current_fl = QFormLayout(self.current_group)
        self.current_id_input = QLineEdit()
        self.current_iq_input = QLineEdit()
        self.current_apply_button = QPushButton("应用电流给定")
        self.current_apply_button.clicked.connect(self._apply_current_refs)
        current_fl.addRow("Id_ref (A)", self.current_id_input)
        current_fl.addRow("Iq_ref (A)", self.current_iq_input)
        current_fl.addRow(self.current_apply_button)
        inner.addWidget(self.current_group)

        # Speed target
        self.speed_group = QGroupBox("速度模式")
        speed_fl = QFormLayout(self.speed_group)
        self.speed_ref_input = QLineEdit()
        self.speed_apply_button = QPushButton("应用速度给定")
        self.speed_apply_button.clicked.connect(self._apply_speed_ref)
        speed_fl.addRow("速度 (rad/s)", self.speed_ref_input)
        speed_fl.addRow(self.speed_apply_button)
        inner.addWidget(self.speed_group)

        # Position target
        self.position_group = QGroupBox("位置模式")
        pos_fl = QFormLayout(self.position_group)
        self.position_ref_input = QLineEdit()
        self.position_apply_button = QPushButton("应用位置给定")
        self.position_apply_button.clicked.connect(self._apply_position_ref)
        pos_fl.addRow("位置 (deg)", self.position_ref_input)
        pos_fl.addRow(self.position_apply_button)
        inner.addWidget(self.position_group)

        panel_layout.addWidget(group)
        return panel

    def _build_joint_pos_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Soft Limits ──
        limits_group = QGroupBox("软限位")
        limits_fl = QFormLayout(limits_group)

        self.joint_limit_enable_cb = QCheckBox("启用软限位")
        self.joint_limit_enable_cb.toggled.connect(self._on_joint_limit_enable_toggled)

        self.joint_limit_min_spin = QDoubleSpinBox()
        self.joint_limit_min_spin.setRange(-180.0, 180.0)
        self.joint_limit_min_spin.setSuffix(" deg")
        self.joint_limit_min_spin.setDecimals(1)
        self.joint_limit_min_spin.setValue(-30.0)

        self.joint_limit_max_spin = QDoubleSpinBox()
        self.joint_limit_max_spin.setRange(-180.0, 180.0)
        self.joint_limit_max_spin.setSuffix(" deg")
        self.joint_limit_max_spin.setDecimals(1)
        self.joint_limit_max_spin.setValue(30.0)

        self.joint_limit_set_btn = QPushButton("设置限位")
        self.joint_limit_set_btn.clicked.connect(self._on_joint_limit_set)
        self.joint_limit_off_btn = QPushButton("关闭限位")
        self.joint_limit_off_btn.clicked.connect(self._on_joint_limit_off)

        limits_fl.addRow(self.joint_limit_enable_cb)
        limits_fl.addRow("最小角度:", self.joint_limit_min_spin)
        limits_fl.addRow("最大角度:", self.joint_limit_max_spin)
        btn_row = QHBoxLayout()
        btn_row.addWidget(self.joint_limit_set_btn)
        btn_row.addWidget(self.joint_limit_off_btn)
        limits_fl.addRow(btn_row)
        layout.addWidget(limits_group)

        # ── Position Target ──
        target_group = QGroupBox("位置目标")
        target_fl = QFormLayout(target_group)

        self.joint_pos_target_spin = QDoubleSpinBox()
        self.joint_pos_target_spin.setRange(-180.0, 180.0)
        self.joint_pos_target_spin.setSuffix(" deg")
        self.joint_pos_target_spin.setDecimals(1)
        self.joint_pos_target_spin.setValue(0.0)

        self.joint_pos_start_btn = QPushButton("发送位置目标")
        self.joint_pos_start_btn.clicked.connect(self._on_joint_pos_start)

        target_fl.addRow("目标角度:", self.joint_pos_target_spin)
        target_fl.addRow(self.joint_pos_start_btn)
        layout.addWidget(target_group)
        layout.addStretch()
        return panel

    def _build_gimbal_speed_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Speed Reference ──
        speed_group = QGroupBox("速度参考")
        speed_outer = QVBoxLayout(speed_group)

        slider_row = QHBoxLayout()
        self.gimbal_speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.gimbal_speed_slider.setRange(-100, 100)
        self.gimbal_speed_slider.setValue(0)
        self.gimbal_speed_value_label = QLabel("0.00 rad/s")
        self.gimbal_speed_slider.valueChanged.connect(self._on_gimbal_speed_slider_changed)
        slider_row.addWidget(self.gimbal_speed_slider)
        slider_row.addWidget(self.gimbal_speed_value_label)
        speed_outer.addLayout(slider_row)

        input_row = QHBoxLayout()
        self.gimbal_speed_spin = QDoubleSpinBox()
        self.gimbal_speed_spin.setRange(-8.0, 8.0)
        self.gimbal_speed_spin.setSingleStep(0.01)
        self.gimbal_speed_spin.setSuffix(" rad/s")
        self.gimbal_speed_spin.setDecimals(2)
        self.gimbal_speed_spin.setValue(0.0)
        self.gimbal_speed_set_btn = QPushButton("发送速度")
        self.gimbal_speed_set_btn.clicked.connect(self._on_gimbal_speed_set)
        input_row.addWidget(QLabel("速度:"))
        input_row.addWidget(self.gimbal_speed_spin)
        input_row.addWidget(self.gimbal_speed_set_btn)
        input_row.addStretch()
        speed_outer.addLayout(input_row)
        layout.addWidget(speed_group)

        # ── Ramp Acceleration ──
        ramp_group = QGroupBox("斜坡加速度")
        ramp_fl = QFormLayout(ramp_group)

        self.gimbal_ramp_spin = QDoubleSpinBox()
        self.gimbal_ramp_spin.setRange(0.1, 20.0)
        self.gimbal_ramp_spin.setValue(2.0)
        self.gimbal_ramp_spin.setSuffix(" rad/s²")
        self.gimbal_ramp_spin.setDecimals(1)
        self.gimbal_ramp_set_btn = QPushButton("设置加速度")
        self.gimbal_ramp_set_btn.clicked.connect(self._on_gimbal_ramp_set)

        ramp_fl.addRow("加速度:", self.gimbal_ramp_spin)
        ramp_fl.addRow(self.gimbal_ramp_set_btn)
        layout.addWidget(ramp_group)
        layout.addStretch()
        return panel

    def _build_hold_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Status ──
        status_group = QGroupBox("位置保持状态")
        status_fl = QFormLayout(status_group)
        self.hold_angle_label = QLabel("--")
        self.hold_angle_label.setStyleSheet("font-size: 14px; font-weight: 600;")
        self.hold_speed_label = QLabel("--")
        self.hold_speed_label.setStyleSheet("font-size: 14px; font-weight: 600;")
        status_fl.addRow("当前角度:", self.hold_angle_label)
        status_fl.addRow("当前速度:", self.hold_speed_label)
        layout.addWidget(status_group)

        # ── Lock Button ──
        self.hold_lock_btn = QPushButton("🔒 锁定当前位置")
        self.hold_lock_btn.setMinimumHeight(48)
        self.hold_lock_btn.setStyleSheet(
            "QPushButton { background-color: #0f766e; color: white; font-weight: 700; font-size: 14px; }"
            "QPushButton:hover { background-color: #0d9488; }"
            "QPushButton:disabled { background-color: #9ca3af; }"
        )
        self.hold_lock_btn.clicked.connect(self._on_hold_lock)
        layout.addWidget(self.hold_lock_btn)
        layout.addStretch()
        return panel

    def _build_spring_damper_panel(self) -> QWidget:
        try:
            from .gui_logic import SPRING_PRESETS
        except ImportError:
            from gui_logic import SPRING_PRESETS

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Presets ──
        preset_group = QGroupBox("预设")
        preset_row = QHBoxLayout(preset_group)
        self.spring_preset_soft_btn = QPushButton("柔和\nK=0.25 D=0.03")
        self.spring_preset_soft_btn.clicked.connect(lambda: self._on_spring_preset("soft"))
        self.spring_preset_std_btn = QPushButton("标准\nK=0.50 D=0.05")
        self.spring_preset_std_btn.clicked.connect(lambda: self._on_spring_preset("standard"))
        self.spring_preset_hard_btn = QPushButton("偏硬\nK=0.80 D=0.08")
        self.spring_preset_hard_btn.clicked.connect(lambda: self._on_spring_preset("hard"))
        for btn in (self.spring_preset_soft_btn, self.spring_preset_std_btn, self.spring_preset_hard_btn):
            btn.setMinimumHeight(44)
        preset_row.addWidget(self.spring_preset_soft_btn)
        preset_row.addWidget(self.spring_preset_std_btn)
        preset_row.addWidget(self.spring_preset_hard_btn)
        layout.addWidget(preset_group)

        # ── Manual Config ──
        cfg_group = QGroupBox("手动配置")
        cfg_fl = QFormLayout(cfg_group)

        self.spring_K_spin = QDoubleSpinBox()
        self.spring_K_spin.setRange(0.0, 5.0)
        self.spring_K_spin.setDecimals(3)
        self.spring_K_spin.setSingleStep(0.05)
        self.spring_K_spin.setValue(0.50)

        self.spring_D_spin = QDoubleSpinBox()
        self.spring_D_spin.setRange(0.0, 1.0)
        self.spring_D_spin.setDecimals(3)
        self.spring_D_spin.setSingleStep(0.01)
        self.spring_D_spin.setValue(0.05)

        self.spring_limit_spin = QDoubleSpinBox()
        self.spring_limit_spin.setRange(0.01, 2.0)
        self.spring_limit_spin.setDecimals(3)
        self.spring_limit_spin.setSingleStep(0.05)
        self.spring_limit_spin.setValue(0.60)

        self.spring_cfg_set_btn = QPushButton("应用参数")
        self.spring_cfg_set_btn.clicked.connect(self._on_spring_cfg_set)

        cfg_fl.addRow("K (刚度 A/rad):", self.spring_K_spin)
        cfg_fl.addRow("D (阻尼 A/(rad/s)):", self.spring_D_spin)
        cfg_fl.addRow("Limit (限幅 A):", self.spring_limit_spin)
        cfg_fl.addRow(self.spring_cfg_set_btn)
        layout.addWidget(cfg_group)
        layout.addStretch()
        return panel

    def _build_detent_panel(self) -> QWidget:
        try:
            from .gui_logic import DETENT_PRESETS
        except ImportError:
            from gui_logic import DETENT_PRESETS

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── Presets ──
        preset_group = QGroupBox("预设")
        preset_row = QHBoxLayout(preset_group)
        self.detent_preset_light_btn = QPushButton("轻卡点\n12格 强度3.00")
        self.detent_preset_light_btn.clicked.connect(lambda: self._on_detent_preset("light"))
        self.detent_preset_std_btn = QPushButton("标准\n12格 强度6.00")
        self.detent_preset_std_btn.clicked.connect(lambda: self._on_detent_preset("standard"))
        self.detent_preset_dense_btn = QPushButton("密集\n24格 强度7.00")
        self.detent_preset_dense_btn.clicked.connect(lambda: self._on_detent_preset("dense"))
        for btn in (self.detent_preset_light_btn, self.detent_preset_std_btn, self.detent_preset_dense_btn):
            btn.setMinimumHeight(44)
        preset_row.addWidget(self.detent_preset_light_btn)
        preset_row.addWidget(self.detent_preset_std_btn)
        preset_row.addWidget(self.detent_preset_dense_btn)
        layout.addWidget(preset_group)

        # ── Manual Config ──
        cfg_group = QGroupBox("手动配置")
        cfg_fl = QFormLayout(cfg_group)

        self.detent_count_spin = QSpinBox()
        self.detent_count_spin.setRange(1, 100)
        self.detent_count_spin.setValue(12)

        self.detent_strength_spin = QDoubleSpinBox()
        self.detent_strength_spin.setRange(0.0, 20.0)
        self.detent_strength_spin.setDecimals(3)
        self.detent_strength_spin.setSingleStep(0.5)
        self.detent_strength_spin.setValue(6.0)

        self.detent_width_spin = QDoubleSpinBox()
        self.detent_width_spin.setRange(0.01, 1.0)
        self.detent_width_spin.setDecimals(3)
        self.detent_width_spin.setSingleStep(0.01)
        self.detent_width_spin.setValue(0.24)

        self.detent_damping_spin = QDoubleSpinBox()
        self.detent_damping_spin.setRange(0.0, 0.5)
        self.detent_damping_spin.setDecimals(3)
        self.detent_damping_spin.setSingleStep(0.01)
        self.detent_damping_spin.setValue(0.10)

        self.detent_limit_spin = QDoubleSpinBox()
        self.detent_limit_spin.setRange(0.01, 2.0)
        self.detent_limit_spin.setDecimals(3)
        self.detent_limit_spin.setSingleStep(0.05)
        self.detent_limit_spin.setValue(0.60)

        self.detent_cfg_set_btn = QPushButton("应用参数")
        self.detent_cfg_set_btn.clicked.connect(self._on_detent_cfg_set)

        cfg_fl.addRow("Count (每圈卡点数):", self.detent_count_spin)
        cfg_fl.addRow("Strength (强度 A/rad):", self.detent_strength_spin)
        cfg_fl.addRow("Width (宽度 rad):", self.detent_width_spin)
        cfg_fl.addRow("Damping (阻尼 A/(rad/s)):", self.detent_damping_spin)
        cfg_fl.addRow("Limit (限幅 A):", self.detent_limit_spin)
        cfg_fl.addRow(self.detent_cfg_set_btn)
        layout.addWidget(cfg_group)
        layout.addStretch()
        return panel

    def _build_scroll_wheel_panel(self) -> QWidget:
        """SCROLL_WHEEL mode panel - managed by FOC_Device_Bridge tray application."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        info_group = QGroupBox("Scroll Wheel Mode / 滚轮鼠标")
        info_fl = QFormLayout(info_group)
        note = QLabel(
            "滚轮模式由 FOC Device Bridge 独占串口并注入系统滚轮事件。\n"
            "请先启动 Bridge，再在上位机顶部选择控制板串口并连接。\n"
            "会话超过 1 秒未收到保活时，固件会自动停止电机。"
        )
        note.setWordWrap(True)
        info_fl.addRow(note)
        layout.addWidget(info_group)

        status_group = QGroupBox("Wheel Status / 滚轮状态")
        status_fl = QFormLayout(status_group)
        self.wheel_session_label = QLabel("--")
        self.wheel_position_label = QLabel("--")
        self.wheel_delta_label = QLabel("--")
        status_fl.addRow("Session / 会话:", self.wheel_session_label)
        status_fl.addRow("Position (steps):", self.wheel_position_label)
        status_fl.addRow("Total Delta:", self.wheel_delta_label)
        layout.addWidget(status_group)

        control_row = QHBoxLayout()
        self.wheel_bridge_enable_btn = QPushButton("启用滚轮")
        self.wheel_bridge_disable_btn = QPushButton("停用滚轮")
        self.wheel_bridge_enable_btn.setEnabled(False)
        self.wheel_bridge_disable_btn.setEnabled(False)
        self.wheel_bridge_enable_btn.clicked.connect(
            lambda: self.bridge_wheel_enable_requested.emit(True)
        )
        self.wheel_bridge_disable_btn.clicked.connect(
            lambda: self.bridge_wheel_enable_requested.emit(False)
        )
        control_row.addWidget(self.wheel_bridge_enable_btn)
        control_row.addWidget(self.wheel_bridge_disable_btn)
        layout.addLayout(control_row)

        layout.addStretch()
        return panel

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
        self.bridge_wheel_enable_requested.connect(worker.request_bridge_wheel_enable)
        worker.ports_updated.connect(self.update_ports)
        worker.connection_changed.connect(self.update_connection_state)
        worker.log_line.connect(self.handle_log_line)
        worker.packet_received.connect(self.apply_packet)
        worker.current_samples_batch.connect(self._on_current_samples_batch)
        if hasattr(worker, "bridge_state_changed"):
            worker.bridge_state_changed.connect(self.update_bridge_state)
        if hasattr(worker, "wheel_status_changed"):
            worker.wheel_status_changed.connect(self.update_wheel_status)

    def update_bridge_state(self, state: dict):
        state_name = str(state.get("state", "DISCONNECTED"))
        port = str(state.get("port", ""))
        if state_name == "DISCONNECTED":
            self.wheel_session_label.setText("未连接")
            self.wheel_position_label.setText("--")
            self.wheel_delta_label.setText("--")
            self.wheel_bridge_enable_btn.setEnabled(False)
            self.wheel_bridge_disable_btn.setEnabled(False)
            return
        suffix = f" | {port}" if port else ""
        self.wheel_session_label.setText(f"{state_name}{suffix}")
        self.wheel_bridge_enable_btn.setEnabled(state_name == "CONNECTED_IDLE")
        self.wheel_bridge_disable_btn.setEnabled(
            state_name in ("WHEEL_ENABLING", "WHEEL_ACTIVE", "WHEEL_DISABLING")
        )

    def update_wheel_status(self, status: dict):
        self.wheel_position_label.setText(str(status.get("position_steps", "--")))
        self.wheel_delta_label.setText(str(status.get("total_delta", "--")))

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

        # V1.2: reset joint product mode state on disconnect
        if not self._state.is_connected:
            self._state.app_mode = None
            self._state.app_mode_ctrl = None
            self._state.joint_limit_enabled = False
            self._state.joint_limit_min = None
            self._state.joint_limit_max = None
            self._state.gimbal_ramp_accel = None
            self._state.spring_K = self._state.spring_D = self._state.spring_limit = None
            self._state.detent_count = None
            self._state.detent_strength = self._state.detent_width = self._state.detent_damping = self._state.detent_limit = None

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
            self.quick_clear_rearm_button,
        ):
            button.setEnabled(target_enabled)
        self.quick_arm_button.setEnabled(state["can_quick_arm"])
        self.vbus_limit_apply_button.setEnabled(can_edit_vbus_limits(self._state))
        self.adc_noise_button.setEnabled(can_edit_vbus_limits(self._state))
        self.quick_safe_stop_button.setEnabled(self._state.is_connected and (self._state.power_unlocked or self._state.motor_enabled))
        self.export_plot_button.setEnabled(self._plot_buffer.has_rows())

        # ── V1.2 App-tab power controls ──
        self.app_unlock_button.setEnabled(state["can_unlock"])
        self.app_lock_button.setEnabled(state["can_lock"])
        self.app_enable_button.setEnabled(state["app_enable"])
        self.app_disable_button.setEnabled(state["can_disable"])
        self.app_arm_button.setEnabled(state["app_arm"])
        if self._state.power_unlocked and self._state.motor_enabled:
            power_text = "已解锁 | 已使能"
            power_style = "color: #0f766e; font-weight: 600;"
        elif self._state.power_unlocked:
            power_text = "已解锁 | 未使能"
            power_style = "color: #b45309; font-weight: 600;"
        else:
            power_text = "未解锁 | 未使能"
            power_style = "color: #64748b; font-weight: 600;"
        for label in (self.power_status_label, self.app_power_status):
            label.setText(power_text)
            label.setStyleSheet(power_style)

        # ── V1.2 Joint Product Mode controls ──
        app_mode_ok = state["app_mode_selector"]
        joint_ok = state["joint_limit_config"]
        ramp_ok = state["gimbal_ramp_config"]
        sd_ok = state["spring_detent_config"]
        motion_ok = state["motion_target"]
        hold_ok = state["hold_button"]
        bridge_managed_wheel = (self._state.app_mode_selected or "RAW") == "SCROLL_WHEEL"

        self.app_mode_combo.setEnabled(app_mode_ok)
        self.app_mode_set_button.setEnabled(app_mode_ok and not bridge_managed_wheel)
        self.app_stop_button.setEnabled(self._state.is_connected)

        # JOINT_POS
        jl_checked = self.joint_limit_enable_cb.isChecked() if hasattr(self, 'joint_limit_enable_cb') else False
        self.joint_limit_enable_cb.setEnabled(joint_ok)
        self.joint_limit_min_spin.setEnabled(joint_ok and jl_checked)
        self.joint_limit_max_spin.setEnabled(joint_ok and jl_checked)
        self.joint_limit_set_btn.setEnabled(joint_ok and jl_checked)
        self.joint_limit_off_btn.setEnabled(joint_ok)
        self.joint_pos_target_spin.setEnabled(motion_ok)
        self.joint_pos_start_btn.setEnabled(motion_ok)

        # GIMBAL_SPEED
        self.gimbal_speed_slider.setEnabled(motion_ok)
        self.gimbal_speed_spin.setEnabled(motion_ok)
        self.gimbal_speed_set_btn.setEnabled(motion_ok)
        self.gimbal_ramp_spin.setEnabled(ramp_ok)
        self.gimbal_ramp_set_btn.setEnabled(ramp_ok)

        # HOLD
        self.hold_lock_btn.setEnabled(hold_ok)

        # SPRING_DAMPER
        self.spring_K_spin.setEnabled(sd_ok)
        self.spring_D_spin.setEnabled(sd_ok)
        self.spring_limit_spin.setEnabled(sd_ok)
        self.spring_cfg_set_btn.setEnabled(sd_ok)
        self.spring_preset_soft_btn.setEnabled(sd_ok)
        self.spring_preset_std_btn.setEnabled(sd_ok)
        self.spring_preset_hard_btn.setEnabled(sd_ok)

        # DETENT
        self.detent_count_spin.setEnabled(sd_ok)
        self.detent_strength_spin.setEnabled(sd_ok)
        self.detent_width_spin.setEnabled(sd_ok)
        self.detent_damping_spin.setEnabled(sd_ok)
        self.detent_limit_spin.setEnabled(sd_ok)
        self.detent_cfg_set_btn.setEnabled(sd_ok)
        self.detent_preset_light_btn.setEnabled(sd_ok)
        self.detent_preset_std_btn.setEnabled(sd_ok)
        self.detent_preset_dense_btn.setEnabled(sd_ok)

    def apply_mode_selection(self, mode: int, emit_command: bool = True):
        self._state.control_mode = int(mode)
        self.target_label.setText(mode_target_label(self._state.control_mode))
        self.session_mode_value.setText(mode_name(self._state.control_mode))
        self.advanced_mode_value.setText(self._app_control_status_text())
        self._sync_mode_buttons(self._state.control_mode)
        self._refresh_mode_highlight()
        self._persist_profile_from_widgets()
        if emit_command and self._state.is_connected:
            self._dispatch_command(CommandBuilder.set_mode(self._state.control_mode))

    def apply_packet(self, packet: FOCDataPacket):
        if getattr(packet, "phase_current_only", False):
            self._state.last_packet_received_at_ms = int(time.monotonic() * 1000)
            if self._scope_enabled and self._scope_start_timestamp is None and packet.timestamp > 0:
                self._scope_start_timestamp = float(packet.timestamp)
            self._plot_buffer.append_packet(packet)
            self._request_plot_refresh()
            self.export_plot_button.setEnabled(self._plot_buffer.has_rows())
            self._refresh_session_status()
            return

        if self._scope_enabled and self._scope_start_timestamp is None and packet.timestamp > 0:
            self._scope_start_timestamp = float(packet.timestamp)

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
        self._refresh_hold_status(packet)
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
            # V1.2: record timestamp for ACK timeout tracking
            now_ms = int(time.monotonic() * 1000)
            if self._state.pending_command:
                self._state.pending_command_sent_at_ms = now_ms
            self._last_cmd_sent = message.strip()
            if message.startswith("CMD:IDENTIFY"):
                self._record_identify_event(f"命令：{message}")
            elif message.startswith("CMD:CLEAR_FAULT"):
                self._record_identify_event("已发送清故障命令。")
            self._show_notification("INFO", f"已发送 {message}")
        elif level == "RX":
            # V1.2: Check for ACK responses first
            ack = self._ack_parser.parse_line(message)
            if ack is not None:
                now_ms = int(time.monotonic() * 1000)
                apply_ack_effects(self._state, ack, now_ms)
                # Advance command sequence if pending
                if ack.ok:
                    next_cmd = advance_command_sequence(self._state)
                    if next_cmd:
                        self.command_requested.emit(next_cmd)
                elif ack.command:
                    abort_command_sequence(self._state, "序列中止：" + ack.command + " 失败")
                if ack.ok:
                    self._show_notification("INFO", f"固件确认: {ack.raw}")
                else:
                    reason_text = ack.reason or "未知原因"
                    self._show_notification("ERROR", f"{ack.command} 失败: {reason_text}")
                    self._record_identify_event(f"命令失败: {ack.raw}")
            elif self._state.pending_sequence is not None:
                # Diagnostic: log RX lines that ACK parser missed while sequence is pending
                self.handle_log_line("INFO", f"[SEQ-DBG] ACK missed, seq_idx={self._state.pending_seq_index}: {message[:120]}")
            # Existing ADC_NOISE parsing
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
            # V1.2 — Joint Product Mode query responses
            self._handle_joint_product_rx(message)
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
        self._profile.selected_mode = self._state.control_mode
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
        allowed, reason = can_dispatch_command(self._state, command)
        if not allowed:
            self.handle_log_line("ERROR", reason)
            return
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
        has_advancing_command = any(not is_safe_fallback_command(command) for command in commands)
        if self._state.pending_command and has_advancing_command:
            self.handle_log_line("ERROR", f"正在等待 {self._state.pending_command} 确认，命令序列已拦截。")
            return
        fault_active = bool(self._state.fault_active or self._state.foc_state == FOC_STATE_FAULT)
        if fault_active and has_advancing_command:
            self.handle_log_line("ERROR", "当前处于故障状态，只允许停机、上锁、清故障或诊断命令。")
            return
        if len(commands) == 1:
            allowed, reason = can_dispatch_command(self._state, commands[0])
            if not allowed:
                self.handle_log_line("ERROR", reason)
                return
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

        requires_stall_mode = (not self._state.motor_identified) or (self._state.encoder_detected is not True)
        if not self._state.stall_mode_armed:
            commands.append(CommandBuilder.set_stall_mode(True))
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
        if self._state.control_mode != 0:
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
        if self._state.control_mode != 1:
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
        if self._state.control_mode != 2:
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

    # ── Joint Product Mode Slots (V1.2) ─────────────────────────────────────

    def _on_app_mode_combo_changed(self, idx: int):
        self.mode_panel_stack.setCurrentIndex(idx)
        token = self.app_mode_combo.itemData(idx)
        if token:
            self._state.app_mode_selected = token
        self._apply_control_enable_state()
        self._refresh_mode_views()

    def _on_app_enable(self):
        state = button_enable_state(self._state)
        if not state["app_enable"]:
            self._show_notification("ERROR", "\u65e0\u6cd5\u4f7f\u80fd\uff1a\u672a\u8fde\u63a5/\u672a\u89e3\u9501/\u5df2\u4f7f\u80fd/\u8bc6\u522b\u4e2d/\u6545\u969c")
            return
        selected = self._state.app_mode_selected or "RAW"
        ok, reason = get_app_mode_prerequisites(self._state, selected)
        if not ok:
            self._show_notification("ERROR", reason)
            return
        seq = build_app_enable_sequence(selected, self._state)
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_app_arm(self):
        state = button_enable_state(self._state)
        if not state["app_arm"]:
            self._show_notification("ERROR", "\u65e0\u6cd5\u89e3\u9501\u5e76\u4f7f\u80fd\uff1a\u672a\u8fde\u63a5/\u5df2\u4f7f\u80fd/\u8bc6\u522b\u4e2d/\u6545\u969c")
            return
        selected = self._state.app_mode_selected or "RAW"
        ok, reason = get_app_mode_prerequisites(self._state, selected)
        if not ok:
            self._show_notification("ERROR", reason)
            return
        seq = build_app_arm_sequence(selected, self._state)
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_app_mode_set(self):
        state = button_enable_state(self._state)
        if not state["app_mode_selector"]:
            self._show_notification("ERROR", "无法切换APP_MODE：未连接或识别中/故障")
            return
        mode = self.app_mode_combo.currentData()
        if mode == "SCROLL_WHEEL":
            self._show_notification("ERROR", "滚轮鼠标模式由 FOC Device Bridge 管理，请从系统托盘启用。")
            return
        try:
            cmd = CommandBuilder.app_mode_set(mode)
        except ValueError as e:
            self._show_notification("ERROR", str(e))
            return
        self._dispatch_command(cmd)

    def _on_joint_limit_set(self):
        state = button_enable_state(self._state)
        if not state["joint_limit_config"]:
            self._show_notification("ERROR", "无法设置软限位：未连接或识别中/故障")
            return
        min_deg = self.joint_limit_min_spin.value()
        max_deg = self.joint_limit_max_spin.value()
        if min_deg >= max_deg:
            self._show_notification("ERROR", "软限位错误：最小值必须小于最大值")
            return
        self._dispatch_command(CommandBuilder.joint_limit_set(min_deg, max_deg))

    def _on_joint_limit_off(self):
        state = button_enable_state(self._state)
        if not state["joint_limit_config"]:
            self._show_notification("ERROR", "无法关闭软限位：未连接或识别中/故障")
            return
        self._dispatch_command(CommandBuilder.joint_limit_off())

    def _on_joint_limit_enable_toggled(self, checked: bool):
        self.joint_limit_min_spin.setEnabled(checked)
        self.joint_limit_max_spin.setEnabled(checked)
        self.joint_limit_set_btn.setEnabled(checked)

    def _on_gimbal_speed_slider_changed(self, value: int):
        speed = value / 100.0
        self.gimbal_speed_value_label.setText(f"{speed:.2f} rad/s")
        self.gimbal_speed_spin.blockSignals(True)
        self.gimbal_speed_spin.setValue(speed)
        self.gimbal_speed_spin.blockSignals(False)

    def _on_gimbal_speed_set(self):
        state = button_enable_state(self._state)
        if not state["motion_target"]:
            self._show_notification("ERROR", "无法发送速度：未连接/未解锁/未使能/识别中/故障")
            return
        speed = self.gimbal_speed_spin.value()
        seq = build_app_target_sequence("GIMBAL_SPEED", CommandBuilder.set_speed_ref(speed))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)
        self.gimbal_speed_value_label.setText(f"{speed:.2f} rad/s")
        self.gimbal_speed_spin.blockSignals(True)
        self.gimbal_speed_spin.setValue(speed)
        self.gimbal_speed_spin.blockSignals(False)

    def _on_gimbal_ramp_set(self):
        state = button_enable_state(self._state)
        if not state["gimbal_ramp_config"]:
            self._show_notification("ERROR", "无法设置斜坡：未连接或识别中/故障")
            return
        accel = self.gimbal_ramp_spin.value()
        self._dispatch_command(CommandBuilder.gimbal_ramp_set(accel))

    def _on_hold_lock(self):
        """Lock current position via APP_MODE,HOLD (firmware auto-locks current angle)."""
        state = button_enable_state(self._state)
        if not state["hold_button"]:
            self._show_notification("ERROR", "无法锁定：未连接/未解锁/未使能/识别中/故障")
            return
        seq = [CommandBuilder.app_mode_set("HOLD")]
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_joint_pos_start(self):
        state = button_enable_state(self._state)
        if not state["motion_target"]:
            self._show_notification("ERROR", "无法发送位置目标：未连接/未解锁/未使能/识别中/故障")
            return
        deg = self.joint_pos_target_spin.value()
        rad = math.radians(deg)
        seq = build_app_target_sequence("JOINT_POS", CommandBuilder.set_position_ref(rad))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_spring_preset(self, preset: str):
        try:
            from .gui_logic import SPRING_PRESETS
        except ImportError:
            from gui_logic import SPRING_PRESETS
        state = button_enable_state(self._state)
        if not state["spring_detent_config"]:
            self._show_notification("ERROR", "无法设置弹簧参数：未连接或识别中/故障")
            return
        K, D, limit = SPRING_PRESETS[preset]
        self.spring_K_spin.setValue(K)
        self.spring_D_spin.setValue(D)
        self.spring_limit_spin.setValue(limit)
        seq = build_app_config_sequence("SPRING_DAMPER", CommandBuilder.spring_cfg_set(K, D, limit))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)
    def _on_spring_cfg_set(self):
        state = button_enable_state(self._state)
        if not state["spring_detent_config"]:
            self._show_notification("ERROR", "无法设置弹簧参数：未连接或识别中/故障")
            return
        K = self.spring_K_spin.value()
        D = self.spring_D_spin.value()
        limit = self.spring_limit_spin.value()
        seq = build_app_config_sequence("SPRING_DAMPER", CommandBuilder.spring_cfg_set(K, D, limit))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_detent_preset(self, preset: str):
        try:
            from .gui_logic import DETENT_PRESETS
        except ImportError:
            from gui_logic import DETENT_PRESETS
        state = button_enable_state(self._state)
        if not state["spring_detent_config"]:
            self._show_notification("ERROR", "无法设置卡点参数：未连接或识别中/故障")
            return
        count, strength, width, damping, limit = DETENT_PRESETS[preset]
        self.detent_count_spin.setValue(count)
        self.detent_strength_spin.setValue(strength)
        self.detent_width_spin.setValue(width)
        self.detent_damping_spin.setValue(damping)
        seq = build_app_config_sequence("DETENT", CommandBuilder.detent_cfg_set(count, strength, width, damping, limit))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)

    def _on_detent_cfg_set(self):
        state = button_enable_state(self._state)
        if not state["spring_detent_config"]:
            self._show_notification("ERROR", "无法设置卡点参数：未连接或识别中/故障")
            return
        count = self.detent_count_spin.value()
        strength = self.detent_strength_spin.value()
        width = self.detent_width_spin.value()
        damping = self.detent_damping_spin.value()
        limit = self.detent_limit_spin.value()
        seq = build_app_config_sequence("DETENT", CommandBuilder.detent_cfg_set(count, strength, width, damping, limit))
        first = start_command_sequence(self._state, seq)
        if first:
            self.command_requested.emit(first)
            self.handle_log_line("CMD", first)
    def _handle_joint_product_rx(self, line: str):
        """Parse Joint Product Mode query responses from RX and update UI."""
        try:
            from .gui_logic import (
                parse_app_mode_response, parse_joint_limit_response,
                parse_gimbal_ramp_response, parse_spring_cfg_response,
                parse_detent_cfg_response,
            )
        except ImportError:
            from gui_logic import (
                parse_app_mode_response, parse_joint_limit_response,
                parse_gimbal_ramp_response, parse_spring_cfg_response,
                parse_detent_cfg_response,
            )

        result = parse_app_mode_response(line)
        if result:
            self._state.app_mode = result["mode"]
            if result["ctrl"] is not None:
                self._state.app_mode_ctrl = result["ctrl"]
            else:
                inferred_ctrl = {
                    "JOINT_POS": 2,
                    "GIMBAL_SPEED": 1,
                    "HOLD": 2,
                    "SPRING_DAMPER": 2,
                    "DETENT": 2,
                    "SCROLL_WHEEL": 2,
                }.get(result["mode"])
                if inferred_ctrl is not None:
                    self._state.app_mode_ctrl = inferred_ctrl
            self._refresh_mode_views()
            return

        result = parse_joint_limit_response(line)
        if result:
            self._state.joint_limit_enabled = result["enabled"]
            self._state.joint_limit_min = result["min_deg"]
            self._state.joint_limit_max = result["max_deg"]
            self.joint_limit_enable_cb.setChecked(result["enabled"])
            if result["enabled"] and result["min_deg"] is not None:
                self.joint_limit_min_spin.setValue(result["min_deg"])
                self.joint_limit_max_spin.setValue(result["max_deg"])
            return

        result = parse_gimbal_ramp_response(line)
        if result:
            self._state.gimbal_ramp_accel = result["accel"]
            self.gimbal_ramp_spin.setValue(result["accel"])
            return

        result = parse_spring_cfg_response(line)
        if result:
            self._state.spring_K = result["K"]
            self._state.spring_D = result["D"]
            self._state.spring_limit = result["limit"]
            self.spring_K_spin.setValue(result["K"])
            self.spring_D_spin.setValue(result["D"])
            self.spring_limit_spin.setValue(result["limit"])
            return

        result = parse_detent_cfg_response(line)
        if result:
            self._state.detent_count = result["count"]
            self._state.detent_strength = result["strength"]
            self._state.detent_width = result["width"]
            self._state.detent_damping = result["damping"]
            self._state.detent_limit = result["limit"]
            self.detent_count_spin.setValue(result["count"])
            self.detent_strength_spin.setValue(result["strength"])
            self.detent_width_spin.setValue(result["width"])
            self.detent_damping_spin.setValue(result["damping"])
            self.detent_limit_spin.setValue(result["limit"])
            return

    def _query_joint_product_state(self):
        """Send batch of queries to sync Joint Product Mode state from firmware."""
        if not self._state.is_connected:
            return
        self.command_requested.emit(CommandBuilder.app_mode_query())
        self.command_requested.emit(CommandBuilder.joint_limit_query())
        self.command_requested.emit(CommandBuilder.gimbal_ramp_query())
        self.command_requested.emit(CommandBuilder.spring_cfg_query())
        self.command_requested.emit(CommandBuilder.detent_cfg_query())

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
        self._sync_mode_buttons(self._state.control_mode)
        self.session_mode_value.setText(mode_name(self._state.control_mode))
        self.target_label.setText(mode_target_label(self._state.control_mode))
        self.advanced_mode_value.setText(self._app_control_status_text())
        self._refresh_mode_highlight()

    def _app_control_status_text(self) -> str:
        app_token = self._state.app_mode or "RAW"
        app_label = app_mode_cn(app_token)
        selected = self._state.app_mode_selected or "RAW"
        if selected == app_token:
            sync_mark = ""
        else:
            sync_mark = f" [待同步：{app_mode_cn(selected)}]"
        if app_token == "RAW":
            ctrl_mode = self._state.app_mode_ctrl
            if ctrl_mode is None:
                ctrl_mode = self._state.control_mode
        else:
            ctrl_mode = self._inferred_app_control_mode(app_token)
        return f"产品：{app_label}{sync_mark} | 底层：{mode_name(ctrl_mode)}"

    @staticmethod
    def _inferred_app_control_mode(app_mode: str) -> int:
        if app_mode in {
            "JOINT_POS",
            "HOLD",
            "SPRING_DAMPER",
            "DETENT",
            "SCROLL_WHEEL",
        }:
            return 2
        if app_mode == "GIMBAL_SPEED":
            return 1
        return 0

    def _refresh_hold_status(self, packet: FOCDataPacket):
        self.hold_angle_label.setText(f"{packet.angle:.2f} deg")
        self.hold_speed_label.setText(f"{packet.speed:.2f} rad/s")

    def _refresh_mode_highlight(self):
        groups = {
            0: (self.current_group, "力矩模式"),
            1: (self.speed_group, "速度模式"),
            2: (self.position_group, "位置模式"),
        }
        for mode, (group, title) in groups.items():
            active = mode == self._state.control_mode
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

        # V1.2: Update link diagnostics labels
        self._update_link_diag_labels(now_ms)

        # V1.2: ACK timeout warning
        self._check_ack_timeout(now_ms)

    def _update_link_diag_labels(self, now_ms: int):
        """Refresh the link diagnostics group box."""
        state = self._state
        # Last command
        self.last_cmd_sent_value.setText(self._last_cmd_sent or "--")
        # Last ACK
        self.last_ack_value.setText(state.last_ack or "--")
        # Last N-frame
        if state.last_packet_received_at_ms:
            dt = now_ms - state.last_packet_received_at_ms
            self.last_nframe_value.setText(f"{dt}ms 前")
        else:
            self.last_nframe_value.setText("--")
        # Pending
        if state.pending_command:
            self.current_pending_value.setText(state.pending_command)
            self.current_pending_value.setStyleSheet("color: #b45309; font-weight: 600;")
        else:
            self.current_pending_value.setText("无")
            self.current_pending_value.setStyleSheet("")
        # Current stream diag is updated by _refresh_cur_stream_stats

    def _check_ack_timeout(self, now_ms: int):
        """Warn if a pending command hasn't received an ACK within timeout."""
        state = self._state
        if state.pending_command and state.pending_command_sent_at_ms:
            elapsed = now_ms - state.pending_command_sent_at_ms
            if elapsed > ACK_TIMEOUT_MS:
                cmd = state.pending_command
                self._show_notification(
                    "ERROR",
                    f"ACK 超时 ({cmd})，将等待 N 帧收敛。",
                )
                state.last_command_error = f"{cmd}: ACK 超时 >{ACK_TIMEOUT_MS}ms"

    def _refresh_plot(self, *_):
        if pg is None:
            return
        if not self._scope_enabled:
            for curve in self._plot_curves.values():
                curve.setData([], [])
            self.scope_mixed_label.setText("")
            return

        # V1.2: plot pause — freeze curves but keep data flowing
        if self._scope_paused:
            return

        channels = {
            channel
            for channel, checkbox in self.plot_channel_checks.items()
            if checkbox.isChecked()
        }
        self._sync_plot_legend(channels)
        cur_channels = {"Ia", "Ib", "Ic", "Id", "Iq"}
        telemetry_channels = channels - cur_channels
        cur_active = bool(channels & cur_channels) and self._has_current_stream_data()

        # V1.2: mixed display indicator
        if channels & cur_channels and channels & telemetry_channels:
            self.scope_mixed_label.setText(" [Mixed: 相电流 + 遥测] ")
        else:
            self.scope_mixed_label.setText("")

        # ── Current stream channels (decimated) ──
        cur_times: list[float] = []
        cur_series: dict[str, list[float]] = {ch: [] for ch in cur_channels}
        if cur_active:
            cur_times, cur_series = self._get_current_stream_plot_data(
                [ch for ch in cur_channels if ch in channels]
            )

        # ── Telemetry channels (from RollingPlotBuffer with relative time) ──
        telem_series: dict[str, tuple[list[float], list[float]]] = {}
        if telemetry_channels:
            raw = self._plot_buffer
            for channel in telemetry_channels:
                raw_t, raw_v = raw.series(channel)
                if not raw_t:
                    continue
                origin = self._scope_start_timestamp
                if origin is None or origin <= 0.0:
                    origin = raw_t[0]
                rel_t = [(t - origin) / 1000.0 for t in raw_t]
                telem_series[channel] = (rel_t, raw_v)

        # ── Set curve data ──
        for channel, curve in self._plot_curves.items():
            if channel not in channels:
                curve.setData([], [])
            elif channel in cur_series and cur_times:
                curve.setData(cur_times, cur_series[channel])
            elif channel in telem_series:
                curve.setData(*telem_series[channel])
            else:
                curve.setData([], [])

        # ── X range: follow-latest or user-controlled ──
        if self._scope_follow_latest:
            elapsed = self._scope_elapsed_s()
            if elapsed is not None:
                lo = max(0.0, elapsed - self._scope_window_s)
                hi = lo + self._scope_window_s
                self._set_plot_x_range(lo, hi)

        # ── Auto Y ──
        self._update_plot_axis_label(channels)
        if self._scope_auto_y:
            self._apply_auto_y_range(channels, cur_times, cur_series, telem_series)

    def _sync_plot_legend(self, channels: set[str]):
        legend = getattr(self, "_plot_legend", None)
        if legend is None:
            return
        legend_channels = frozenset(channels)
        if legend_channels == self._plot_legend_channels:
            return
        try:
            legend.clear()
            for channel in PLOT_CHANNELS:
                if channel in channels and channel in self._plot_curves:
                    legend.addItem(
                        self._plot_curves[channel],
                        PLOT_CHANNEL_LABELS.get(channel, channel),
                    )
            self._plot_legend_channels = legend_channels
        except Exception:
            return

    def _update_plot_axis_label(self, channels: set[str]):
        if pg is None:
            return
        current_channels = {"Ia", "Ib", "Ic", "Id", "Iq"}
        voltage_channels = {"Vbus", "Vd", "Vq"}
        speed_channels = {"speed", "speed_ref"}
        angle_channels = {"angle", "pos_ref_deg"}
        if channels and channels <= current_channels:
            self.plot_widget.setLabel("left", "Current", units="mA")
        elif channels and channels <= voltage_channels:
            self.plot_widget.setLabel("left", "Voltage", units="V")
        elif channels and channels <= speed_channels:
            self.plot_widget.setLabel("left", "Speed", units="rad/s")
        elif channels and channels <= angle_channels:
            self.plot_widget.setLabel("left", "Angle", units="deg")
        else:
            self.plot_widget.setLabel("left", "Mixed")

    def _scope_elapsed_s(self) -> float | None:
        """Seconds elapsed since scope session started."""
        if not self._scope_enabled:
            return None

        current_elapsed = self._current_stream_elapsed_s()
        if current_elapsed is not None:
            return current_elapsed

        origin = self._scope_start_timestamp
        if origin is not None and origin > 0.0 and self._state.last_packet is not None:
            return (self._state.last_packet.timestamp - origin) / 1000.0
        return None

    def _current_stream_elapsed_s(self) -> float | None:
        if self._scope_start_seq is None:
            return None
        worker = getattr(self, "_serial_worker", None)
        if worker is None:
            return None
        samples = worker.current_ring().get_recent(1)
        if not samples:
            return None
        return self._seq_delta(samples[0].seq, self._scope_start_seq) / self._current_stream_nominal_fps()

    @staticmethod
    def _seq_delta(seq: int, origin: int) -> int:
        return (int(seq) - int(origin)) & 0xFFFF

    def _has_current_stream_data(self) -> bool:
        worker = getattr(self, "_serial_worker", None)
        if worker is None:
            return False
        return worker.current_ring().stats()["total_received"] > 0

    def _get_current_stream_plot_data(self, channels: list[str]) -> tuple[list[float], dict[str, list[float]]]:
        """Decimate current stream samples for plotting (~3000 points max per channel)."""
        MAX_PLOT_POINTS = 3000
        worker = getattr(self, "_serial_worker", None)
        if worker is None:
            return [], {ch: [] for ch in channels}

        ring = worker.current_ring()
        samples = ring.get_all()
        if not samples:
            return [], {ch: [] for ch in channels}

        if self._scope_start_seq is None:
            self._scope_start_seq = samples[0].seq
        if self._scope_start_tick_ms is None:
            self._scope_start_tick_ms = samples[0].tick_ms

        fps = self._current_stream_nominal_fps()
        max_points = self.CURRENT_STREAM_PLOT_MAX_POINTS

        if self._scope_follow_latest:
            visible_count = int(max(1.0, self._scope_window_s) * max(1.0, fps)) + max_points
            samples = samples[-min(len(samples), visible_count):]

        if len(samples) > max_points:
            stride = max(1, math.ceil(len(samples) / max_points))
            decimated = samples[::stride]
            if decimated[-1] is not samples[-1]:
                decimated.append(samples[-1])
            samples = decimated

        times = [self._sample_time_s(s, fps) for s in samples]
        series = {
            ch: [self._sample_value(s, ch) for s in samples]
            for ch in channels
        }
        return times, series

        n = len(samples)
        if n <= MAX_PLOT_POINTS:
            picked = samples
        else:
            # V1.2: min/max envelope downsampling — preserve peak info
            bin_count = MAX_PLOT_POINTS // 2  # each bin emits 2 points (min, max)
            bin_size = max(1, n // bin_count)
            picked = []
            for i in range(0, n, bin_size):
                bin_s = samples[i:i + bin_size]
                if not bin_s:
                    continue
                # Collect all samples in this bin (retain both min and max per channel)
                picked.extend(bin_s)

        # ── Build envelope series ──
        # After picking, build per-channel min/max envelope by bucketing
        # picked into bins and emitting (time, min_val) then (time, max_val)
        raw_times: list[float] = []
        raw_series: dict[str, list[float]] = {ch: [] for ch in channels}
        for s in picked:
            raw_times.append(self._sample_time_s(s))
            for ch in channels:
                raw_series[ch].append(self._sample_value(s, ch))

        # Apply envelope if downsampled
        if n > MAX_PLOT_POINTS:
            env_times: list[float] = []
            env_series: dict[str, list[float]] = {ch: [] for ch in channels}
            bin_size_env = max(1, len(picked) // (MAX_PLOT_POINTS // 2))
            for i in range(0, len(picked), bin_size_env):
                idx_end = min(i + bin_size_env, len(picked))
                if i >= idx_end:
                    continue
                for ch in channels:
                    vals = raw_series[ch][i:idx_end]
                    if not vals:
                        continue
                    min_idx = min(range(len(vals)), key=lambda j: vals[j])
                    max_idx = max(range(len(vals)), key=lambda j: vals[j])
                    abs_min_i = i + min_idx
                    abs_max_i = i + max_idx
                    t_min = raw_times[abs_min_i]
                    t_max = raw_times[abs_max_i]
                    env_times.append(t_min)
                    env_times.append(t_max)
                    env_series[ch].append(vals[min_idx])
                    env_series[ch].append(vals[max_idx])
            # Sort by time to keep plot in order
            sort_idx = sorted(range(len(env_times)), key=lambda j: env_times[j])
            times = [env_times[j] for j in sort_idx]
            series = {ch: [env_series[ch][j] for j in sort_idx] for ch in channels}
        else:
            times = raw_times
            series = raw_series

        return times, series

    def _sample_time_s(self, s, fps: float | None = None) -> float:
        """Compute relative time (seconds) for a CurrentSample."""
        if self._scope_start_seq is not None:
            if fps is None:
                fps = self._current_stream_nominal_fps()
            return self._seq_delta(s.seq, self._scope_start_seq) / max(1.0, fps)
        if self._scope_start_tick_ms is not None:
            return (s.tick_ms - self._scope_start_tick_ms) / 1000.0
        return 0.0

    def _current_stream_nominal_fps(self) -> float:
        index = self.cur_mode_combo.currentIndex() if hasattr(self, "cur_mode_combo") else 0
        if index == 1:
            return 200.0
        if index == 3:
            return 2000.0
        return 1000.0

    def _estimate_current_stream_fps(self, samples: list) -> float:
        if len(samples) < 2:
            return 1000.0
        first = samples[0]
        latest = samples[-1]
        seq_span = self._seq_delta(latest.seq, first.seq)
        tick_span_s = (latest.tick_ms - first.tick_ms) / 1000.0
        if seq_span > 0 and tick_span_s > 0.0:
            return seq_span / tick_span_s
        return 1000.0

    def _measured_fps(self) -> float:
        """Estimate current stream frames per second from seq and tick deltas."""
        worker = getattr(self, "_serial_worker", None)
        if worker is None:
            return 1000.0
        samples = worker.current_ring().get_all()
        return self._estimate_current_stream_fps(samples)

    @staticmethod
    def _sample_value(s, ch: str) -> float:
        """Extract a channel value from a CurrentSample in display units."""
        if ch == "Ia":
            return s.ia * 1000.0
        elif ch == "Ib":
            return s.ib * 1000.0
        elif ch == "Ic":
            return s.ic * 1000.0
        elif ch == "Id":
            return s.id * 1000.0
        elif ch == "Iq":
            return s.iq * 1000.0
        return 0.0

    def _apply_auto_y_range(self, channels, cur_times, cur_series, telem_series):
        """Compute Y range from visible data and apply with 12% padding."""
        window_s = self._scope_window_s
        elapsed = self._scope_elapsed_s()
        if elapsed is None:
            return
        lo_x = max(0.0, elapsed - window_s)

        all_y: list[float] = []
        # From current stream
        if cur_times:
            for i, t in enumerate(cur_times):
                if lo_x <= t <= lo_x + window_s:
                    for ch in channels & {"Ia", "Ib", "Ic", "Id", "Iq"}:
                        if ch in cur_series and i < len(cur_series[ch]):
                            all_y.append(cur_series[ch][i])
        # From telemetry
        for ch, (t_arr, v_arr) in telem_series.items():
            if ch not in channels:
                continue
            for i, t in enumerate(t_arr):
                if lo_x <= t <= lo_x + window_s and i < len(v_arr):
                    all_y.append(v_arr[i])

        if not all_y:
            return

        y_min, y_max = min(all_y), max(all_y)
        if y_min == y_max:
            y_min -= 1.0
            y_max += 1.0
        pad = (y_max - y_min) * 0.12
        self._scope_suppress_range_change = True
        self.plot_widget.setYRange(y_min - pad, y_max + pad, padding=0.0)
        self._scope_suppress_range_change = False

    # ── Scope control callbacks ──

    def _on_scope_toggled(self, checked: bool):
        self._scope_enabled = bool(checked)
        self.scope_toggle_button.setText("停止波形" if checked else "开始波形")
        if checked:
            self._scope_paused = False
            self.scope_pause_check.setChecked(False)
            self._scope_start_timestamp = None
            self._scope_start_seq = None
            self._scope_start_tick_ms = None
            self._cur_stats_prev_total = 0
            self._cur_stats_prev_at_s = None
            self._scope_follow_latest = True
            self._scope_auto_y = True
            self.scope_follow_check.setChecked(True)
            self.scope_auto_y_check.setChecked(True)
            self._plot_buffer = RollingPlotBuffer()
            # Also clear current stream ring
            worker = getattr(self, "_serial_worker", None)
            if worker is not None:
                ring = worker.current_ring()
                ring.clear()
                parser = worker.current_parser()
                parser.reset_stats()
            if pg is not None:
                self._set_plot_x_range(0.0, self._scope_window_s)
            # Set origin from first arriving data
        else:
            for curve in self._plot_curves.values():
                curve.setData([], [])
            self._plot_buffer = RollingPlotBuffer()
            if pg is not None:
                self._set_plot_x_range(0.0, self._scope_window_s)

    def _on_scope_pause_toggled(self, checked: bool):
        """V1.2: freeze plot display without stopping firmware data flow."""
        self._scope_paused = bool(checked)

    def _set_plot_x_range(self, lo: float, hi: float):
        if pg is None:
            return
        self._scope_suppress_range_change = True
        self._scope_ignore_manual_range_until_ms = time.monotonic() * 1000.0 + 250.0
        self.plot_widget.setXRange(lo, hi, padding=0.0)
        self._scope_suppress_range_change = False

    def _on_scope_wake(self):
        """Back to latest: re-enable follow-latest and jump to current window."""
        if pg is None:
            return
        self._scope_follow_latest = True
        self.scope_follow_check.setChecked(True)
        elapsed = self._scope_elapsed_s()
        if elapsed is not None:
            lo = max(0.0, elapsed - self._scope_window_s)
            hi = lo + self._scope_window_s
            self._set_plot_x_range(lo, hi)

    def _on_scope_window_changed(self, text: str):
        try:
            self._scope_window_s = float(text.replace("s", ""))
        except ValueError:
            self._scope_window_s = 5.0
        self._refresh_plot()

    def _on_scope_follow_toggled(self, checked: bool):
        self._scope_follow_latest = bool(checked)

    def _on_scope_auto_y_toggled(self, checked: bool):
        self._scope_auto_y = bool(checked)

    def _on_user_zoom_or_pan(self, *_):
        """User manually zoomed/panned → disable follow-latest."""
        if self._scope_suppress_range_change:
            return
        if time.monotonic() * 1000.0 < self._scope_ignore_manual_range_until_ms:
            return
        if self._scope_follow_latest:
            self._scope_follow_latest = False
            self.scope_follow_check.blockSignals(True)
            self.scope_follow_check.setChecked(False)
            self.scope_follow_check.blockSignals(False)

    def _request_plot_refresh(self):
        self._plot_refresh_pending = True

    def _flush_pending_plot_refresh(self):
        if self._scope_enabled and getattr(self, "cur_stream_group", None) is not None and self.cur_stream_group.isChecked():
            self._plot_refresh_pending = False
            self._refresh_plot()
            return
        if not self._plot_refresh_pending:
            return
        self._plot_refresh_pending = False
        self._refresh_plot()

    # ── Current Stream Callbacks ──────────────────────────────────────────

    def _on_cur_stream_toggled(self, checked: bool):
        if checked:
            self._select_current_stream_plot_channels()
            self._reset_current_stream_scope_origin()
            self._cur_stream_stats_timer.start()
            if self.scope_window_combo.currentText() == "5s":
                self.scope_window_combo.setCurrentText("1s")
            if not self._scope_enabled:
                self.scope_toggle_button.setChecked(True)
            if self.cur_mode_combo.currentIndex() == 0:
                self.cur_mode_combo.setCurrentIndex(2)  # BIN 1kHz recommended
            else:
                self._on_cur_mode_changed(self.cur_mode_combo.currentIndex())
            self._show_notification("INFO", "Current stream BIN 1kHz enabled")
        else:
            self._cur_stream_stats_timer.stop()
            # Send OFF command via CommandBuilder
            self.command_requested.emit(CommandBuilder.telem_cur_off())
            self.cur_mode_combo.setCurrentIndex(0)

    def _select_current_stream_plot_channels(self):
        for channel, checkbox in self.plot_channel_checks.items():
            should_enable = channel in {"Ia", "Ib", "Ic"}
            if checkbox.isChecked() == should_enable:
                continue
            checkbox.blockSignals(True)
            checkbox.setChecked(should_enable)
            checkbox.blockSignals(False)

    def _reset_current_stream_scope_origin(self):
        self._scope_start_timestamp = None
        self._scope_start_seq = None
        self._scope_start_tick_ms = None
        self._cur_stats_prev_total = 0
        self._cur_stats_prev_at_s = None
        self._plot_buffer = RollingPlotBuffer()
        worker = getattr(self, "_serial_worker", None)
        if worker is not None:
            ring = worker.current_ring()
            ring.clear()
            parser = worker.current_parser()
            parser.reset_stats()
        if pg is not None:
            self._set_plot_x_range(0.0, self._scope_window_s)

    def _on_cur_mode_changed(self, index: int):
        if not self.cur_stream_group.isChecked():
            return
        if index > 0 and not self._scope_enabled:
            self.scope_toggle_button.setChecked(True)
        commands = [
            CommandBuilder.telem_cur_off(),          # 0: OFF
            CommandBuilder.telem_cur_ascii(200),     # 1: ASCII 200Hz
            CommandBuilder.telem_cur_bin(1000),      # 2: BIN 1kHz
            CommandBuilder.telem_cur_bin(2000),      # 3: BIN 2kHz
        ]
        if index < len(commands):
            self.command_requested.emit(commands[index])

    def _on_current_samples_batch(self, samples: list):
        """Receive batch of CurrentSample from serial worker.
        Set scope origin on first sample. Plot refresh is timer-driven."""
        if not self._scope_enabled or not samples:
            return
        # Set origin from first sample if not yet set
        if self._scope_start_seq is None:
            self._scope_start_seq = samples[0].seq
        if self._scope_start_tick_ms is None:
            self._scope_start_tick_ms = samples[0].tick_ms

    def _refresh_cur_stream_stats(self):
        """Update current stream stats label (called at 20Hz)."""
        worker = getattr(self, "_serial_worker", None)
        if worker is None:
            self.cur_stats_label.setText("rx: -- fps | gap: -- | CRC: -- | 填充: -- | baud: 1M")
            self.cur_diag_label.setText("diag: --")
            return

        ring = worker.current_ring()
        stats = ring.stats()
        total = stats["total_received"]
        fill = stats["ring_fill"]
        cap = stats["ring_capacity"]
        pct = (fill / cap * 100) if cap > 0 else 0

        # Approximate fps: delta since last tick
        now_s = time.monotonic()
        prev = getattr(self, "_cur_stats_prev_total", 0)
        prev_s = getattr(self, "_cur_stats_prev_at_s", None)
        self._cur_stats_prev_total = total
        self._cur_stats_prev_at_s = now_s
        fps = (total - prev) * 20  # 20Hz timer → multiply by 20 for fps

        if prev_s is None or now_s <= prev_s:
            fps = 0
        else:
            fps = int((total - prev) / (now_s - prev_s))

        self.cur_stats_label.setText(
            f"rx: {fps} fps | gap: {stats['seq_gaps']} | CRC: {stats['crc_errors']} | "
            f"填充: {fill}/{cap} ({pct:.0f}%) | baud: 1M"
        )
        self.cur_diag_label.setText(self._format_current_diagnostics(ring.get_recent(1000)))
        # V1.2: also update link diagnostics
        self.cur_link_diag_value.setText(
            f"{fps}fps | CRC:{stats['crc_errors']} | gap:{stats['seq_gaps']} | {fill}/{cap}"
        )

    @staticmethod
    def _format_current_diagnostics(samples: list) -> str:
        if not samples:
            return "diag: waiting for current samples"

        def mean(values: list[float]) -> float:
            return sum(values) / len(values) if values else 0.0

        def rms(values: list[float]) -> float:
            return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0

        def pkpk(values: list[float]) -> float:
            return (max(values) - min(values)) if values else 0.0

        def clean_zero(value: float) -> float:
            return 0.0 if abs(value) < 0.05 else value

        ia = [s.ia * 1000.0 for s in samples]
        ib = [s.ib * 1000.0 for s in samples]
        ic = [s.ic * 1000.0 for s in samples]
        id_vals = [s.id * 1000.0 for s in samples]
        iq_vals = [s.iq * 1000.0 for s in samples]
        sum_abc = [(s.ia + s.ib + s.ic) * 1000.0 for s in samples]

        return (
            f"diag n={len(samples)} | "
            f"sumABC mean={clean_zero(mean(sum_abc)):+.1f}mA rms={rms(sum_abc):.1f}mA | "
            f"Id mean={clean_zero(mean(id_vals)):+.1f}mA p-p={pkpk(id_vals):.1f}mA | "
            f"Iq mean={clean_zero(mean(iq_vals)):+.1f}mA p-p={pkpk(iq_vals):.1f}mA | "
            f"phase p-p={pkpk(ia):.0f}/{pkpk(ib):.0f}/{pkpk(ic):.0f}mA"
        )

    def _on_tab_changed(self, index: int):
        if index == 1:
            self._plot_refresh_timer.start()
            self._flush_pending_plot_refresh()
        else:
            self._plot_refresh_timer.stop()
        # V1.2: auto-query joint product mode state when switching to 高级控制 tab
        if index == 3:
            self._query_joint_product_state()

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
