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

try:
    from .data_parser import CommandBuilder, FOCDataPacket
    from .gui_logic import (
        DEFAULT_PROFILE_PATH,
        LOG_LEVELS,
        PLOT_CHANNELS,
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
        log_line_text,
        mode_name,
        mode_target_label,
        packet_snapshot,
        save_gui_profile,
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
        log_line_text,
        mode_name,
        mode_target_label,
        packet_snapshot,
        save_gui_profile,
    )


class HostMainWindow(QMainWindow):
    command_requested = pyqtSignal(str)
    refresh_ports_requested = pyqtSignal()
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    LOG_LIMIT = 400
    IDENTIFY_LOG_LIMIT = 200
    STALE_THRESHOLD_MS = 1500

    def __init__(self, profile_path: Path | str | None = None):
        super().__init__()
        self._profile_path = Path(profile_path) if profile_path else DEFAULT_PROFILE_PATH
        self._profile = load_gui_profile(self._profile_path)
        self._state = HostAppState(selected_mode=self._profile.selected_mode)
        self._serial_worker = None
        self._loading_profile = False
        self._log_entries: list[tuple[str, str]] = []
        self._identify_entries: list[str] = []
        self._plot_buffer = RollingPlotBuffer(max_samples=300)
        self._plot_curves: dict[str, object] = {}
        self._mode_button_map: dict[int, list[QRadioButton]] = {0: [], 1: [], 2: []}

        self.setWindowTitle("FOC Host Debug GUI")
        self.resize(1500, 920)

        self._build_toolbar()
        self._build_status_bar()

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._build_debug_panel(), "Debug Panel")
        self.tabs.addTab(self._build_identify_tab(), "Identify")
        self.tabs.addTab(self._build_advanced_control_tab(), "Advanced Control")
        self.tabs.addTab(self._build_pi_tab(), "PI Parameters")

        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.setInterval(250)
        self._heartbeat_timer.timeout.connect(self._refresh_session_status)
        self._heartbeat_timer.start()

        self._load_profile_into_widgets()
        self.apply_mode_selection(self._state.selected_mode, emit_command=False)
        self.update_connection_state(False)
        self._render_logs()
        self._render_identify_log()

    def _build_toolbar(self):
        toolbar = QToolBar("Connection")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        toolbar.addWidget(QLabel("Port"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        self.port_combo.currentTextChanged.connect(self._persist_profile_from_widgets)
        self.port_combo.currentTextChanged.connect(self._sync_connect_button)
        toolbar.addWidget(self.port_combo)

        toolbar.addSeparator()
        toolbar.addWidget(QLabel("Baud"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        self.baud_combo.currentTextChanged.connect(self._persist_profile_from_widgets)
        toolbar.addWidget(self.baud_combo)

        toolbar.addSeparator()
        self.refresh_ports_button = QPushButton("Refresh Ports")
        self.refresh_ports_button.clicked.connect(self.refresh_ports_requested.emit)
        toolbar.addWidget(self.refresh_ports_button)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._request_connect)
        toolbar.addWidget(self.connect_button)

        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_requested.emit)
        toolbar.addWidget(self.disconnect_button)

        toolbar.addSeparator()
        self.connection_status_value = QLabel("Disconnected")
        toolbar.addWidget(self.connection_status_value)

    def _build_status_bar(self):
        self.notification_label = QLabel("Ready")
        self.notification_label.setStyleSheet("padding: 0 8px; color: #0f172a;")
        self.statusBar().addPermanentWidget(self.notification_label, 1)

    def _build_debug_panel(self) -> QWidget:
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(18)
        layout.addWidget(self._build_actions_column(), 1)
        layout.addWidget(self._build_runtime_column(), 2)
        layout.addWidget(self._build_fault_log_column(), 2)
        return panel

    def _build_actions_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        power_group = QGroupBox("Power Stage")
        power_layout = QVBoxLayout(power_group)
        self.unlock_button = QPushButton("UNLOCK")
        self.lock_button = QPushButton("LOCK")
        self.enable_button = QPushButton("ENABLE")
        self.disable_button = QPushButton("DISABLE")
        self.clear_fault_button = QPushButton("CLEAR FAULT")
        self.unlock_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.unlock_power(True)))
        self.lock_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.unlock_power(False)))
        self.enable_button.clicked.connect(lambda: self._dispatch_command(CommandBuilder.enable_motor(True)))
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

        quick_group = QGroupBox("Quick Actions")
        quick_layout = QVBoxLayout(quick_group)
        self.quick_arm_button = QPushButton("Unlock + Enable")
        self.quick_safe_stop_button = QPushButton("Disable + Lock")
        self.quick_clear_rearm_button = QPushButton("Clear Fault + Re-arm Hint")
        self.quick_arm_button.clicked.connect(
            lambda: self._dispatch_sequence([CommandBuilder.unlock_power(True), CommandBuilder.enable_motor(True)])
        )
        self.quick_safe_stop_button.clicked.connect(
            lambda: self._dispatch_sequence([CommandBuilder.enable_motor(False), CommandBuilder.unlock_power(False)])
        )
        self.quick_clear_rearm_button.clicked.connect(self._clear_fault_with_hint)
        for button in (self.quick_arm_button, self.quick_safe_stop_button, self.quick_clear_rearm_button):
            quick_layout.addWidget(button)
        layout.addWidget(quick_group)

        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout(mode_group)
        self.mode_button_group = QButtonGroup(self)
        self.torque_mode_button = QRadioButton("Torque")
        self.speed_mode_button = QRadioButton("Speed")
        self.position_mode_button = QRadioButton("Position")
        self._register_mode_button(self.torque_mode_button, 0, self.mode_button_group)
        self._register_mode_button(self.speed_mode_button, 1, self.mode_button_group)
        self._register_mode_button(self.position_mode_button, 2, self.mode_button_group)
        self.mode_button_group.idClicked.connect(self.apply_mode_selection)
        mode_layout.addWidget(self.torque_mode_button)
        mode_layout.addWidget(self.speed_mode_button)
        mode_layout.addWidget(self.position_mode_button)
        self.target_label = QLabel(mode_target_label(0))
        self.target_label.setStyleSheet("font-weight: 600; color: #0f766e;")
        mode_layout.addWidget(QLabel("Active target path"))
        mode_layout.addWidget(self.target_label)
        hint = QLabel("Use the Advanced Control tab for dedicated torque, speed, and position entry.")
        hint.setWordWrap(True)
        mode_layout.addWidget(hint)
        layout.addWidget(mode_group)

        layout.addStretch(1)
        return widget

    def _build_runtime_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        runtime_group = QGroupBox("Runtime Status")
        runtime_layout = QFormLayout(runtime_group)
        self.state_value = QLabel("--")
        self.angle_value = QLabel("--")
        self.speed_value = QLabel("--")
        self.currents_value = QLabel("--")
        self.refs_value = QLabel("--")
        self.voltages_value = QLabel("--")
        runtime_layout.addRow("FOC State", self.state_value)
        runtime_layout.addRow("Angle", self.angle_value)
        runtime_layout.addRow("Speed", self.speed_value)
        runtime_layout.addRow("Currents", self.currents_value)
        runtime_layout.addRow("References", self.refs_value)
        runtime_layout.addRow("Voltages", self.voltages_value)
        layout.addWidget(runtime_group)

        session_group = QGroupBox("Session")
        session_layout = QFormLayout(session_group)
        self.packet_timestamp_value = QLabel("--")
        self.connection_state_detail_value = QLabel("Disconnected")
        self.data_status_value = QLabel("Idle")
        self.session_mode_value = QLabel(mode_name(self._state.selected_mode))
        session_layout.addRow("Connection", self.connection_state_detail_value)
        session_layout.addRow("Data Freshness", self.data_status_value)
        session_layout.addRow("Active Mode", self.session_mode_value)
        session_layout.addRow("Last Packet", self.packet_timestamp_value)
        layout.addWidget(session_group)

        self.plot_group = self._build_plot_group()
        layout.addWidget(self.plot_group, 1)
        return widget

    def _build_plot_group(self) -> QGroupBox:
        group = QGroupBox("Live Plot")
        group.setCheckable(True)
        group.setChecked(False)
        group.toggled.connect(self._refresh_plot)
        layout = QVBoxLayout(group)

        toggles_layout = QGridLayout()
        self.plot_channel_checks: dict[str, QCheckBox] = {}
        for index, channel in enumerate(PLOT_CHANNELS):
            checkbox = QCheckBox(channel)
            checkbox.setChecked(channel in {"speed", "Iq", "Iq_ref"})
            checkbox.toggled.connect(self._refresh_plot)
            self.plot_channel_checks[channel] = checkbox
            toggles_layout.addWidget(checkbox, index // 4, index % 4)
        layout.addLayout(toggles_layout)

        if pg is None:
            self.plot_widget = QLabel("pyqtgraph is unavailable in this environment.")
            self.plot_widget.setWordWrap(True)
            layout.addWidget(self.plot_widget)
        else:
            self.plot_widget = pg.PlotWidget()
            self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
            self.plot_widget.addLegend(offset=(10, 10))
            self.plot_widget.setBackground("#f8fafc")
            self.plot_widget.setLabel("bottom", "Time (ms)")
            self.plot_widget.setLabel("left", "Value")
            layout.addWidget(self.plot_widget, 1)
            palette = {
                "angle": "#2563eb",
                "speed": "#059669",
                "Id": "#dc2626",
                "Iq": "#ea580c",
                "Id_ref": "#7c3aed",
                "Iq_ref": "#db2777",
                "Vd": "#0f766e",
                "Vq": "#475569",
            }
            for channel in PLOT_CHANNELS:
                pen = pg.mkPen(palette[channel], width=2)
                self._plot_curves[channel] = self.plot_widget.plot([], [], pen=pen, name=channel)

        self.export_plot_button = QPushButton("Export Plot CSV")
        self.export_plot_button.clicked.connect(self._export_plot_csv)
        layout.addWidget(self.export_plot_button)
        return group

    def _build_fault_log_column(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)

        fault_group = QGroupBox("Fault Summary")
        fault_layout = QFormLayout(fault_group)
        self.fault_state_value = QLabel("NORMAL")
        self.fault_registers_value = QLabel("FAULT1 0x0000 | VGS2 0x0000")
        self.fault_timestamp_value = QLabel("--")
        fault_layout.addRow("State", self.fault_state_value)
        fault_layout.addRow("Registers", self.fault_registers_value)
        fault_layout.addRow("Timestamp", self.fault_timestamp_value)
        layout.addWidget(fault_group)

        log_group = QGroupBox("Serial Log")
        log_layout = QVBoxLayout(log_group)
        filter_layout = QHBoxLayout()
        self.log_filter_checks: dict[str, QCheckBox] = {}
        for level in LOG_LEVELS:
            checkbox = QCheckBox(level)
            checkbox.setChecked(True)
            checkbox.toggled.connect(self._on_log_filter_changed)
            self.log_filter_checks[level] = checkbox
            filter_layout.addWidget(checkbox)
        log_layout.addLayout(filter_layout)

        button_layout = QHBoxLayout()
        self.copy_log_button = QPushButton("Copy Recent Log")
        self.clear_log_button = QPushButton("Clear Log")
        self.copy_log_button.clicked.connect(self._copy_recent_log)
        self.clear_log_button.clicked.connect(self._clear_log)
        button_layout.addWidget(self.copy_log_button)
        button_layout.addWidget(self.clear_log_button)
        log_layout.addLayout(button_layout)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("Connection events, TX/RX lines, and errors appear here.")
        log_layout.addWidget(self.log_view, 1)
        layout.addWidget(log_group, 1)
        return widget

    def _build_identify_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        state_group = QGroupBox("Identify State")
        state_layout = QFormLayout(state_group)
        self.identify_connection_value = QLabel("Disconnected")
        self.identify_power_value = QLabel("Locked")
        self.identify_state_value = QLabel("--")
        self.identify_fault_value = QLabel("UNKNOWN")
        state_layout.addRow("Connection", self.identify_connection_value)
        state_layout.addRow("Power Stage", self.identify_power_value)
        state_layout.addRow("FOC State", self.identify_state_value)
        state_layout.addRow("Fault", self.identify_fault_value)
        layout.addWidget(state_group)

        action_group = QGroupBox("Identify Actions")
        action_layout = QHBoxLayout(action_group)
        self.identify_start_page_button = QPushButton("Start Identify")
        self.identify_stop_page_button = QPushButton("Stop Identify")
        self.identify_clear_fault_button = QPushButton("Clear Fault")
        self.identify_start_page_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.start_identify(), identify_note="Identify start requested.")
        )
        self.identify_stop_page_button.clicked.connect(
            lambda: self._dispatch_command(CommandBuilder.stop_identify(), identify_note="Identify stop requested.")
        )
        self.identify_clear_fault_button.clicked.connect(self._clear_fault_with_hint)
        action_layout.addWidget(self.identify_start_page_button)
        action_layout.addWidget(self.identify_stop_page_button)
        action_layout.addWidget(self.identify_clear_fault_button)
        layout.addWidget(action_group)

        log_group = QGroupBox("Identify Progress / Log")
        log_layout = QVBoxLayout(log_group)
        self.identify_log_view = QPlainTextEdit()
        self.identify_log_view.setReadOnly(True)
        self.identify_log_view.setPlaceholderText(
            "Identify requests, status snapshots, and future firmware progress events appear here."
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

        summary_group = QGroupBox("Control Mode")
        summary_layout = QVBoxLayout(summary_group)
        self.advanced_mode_value = QLabel(mode_name(self._state.selected_mode))
        self.advanced_mode_value.setStyleSheet("font-size: 15px; font-weight: 700; color: #0f766e;")
        summary_layout.addWidget(self.advanced_mode_value)

        advanced_button_group = QButtonGroup(self)
        advanced_mode_layout = QHBoxLayout()
        self.advanced_torque_button = QRadioButton("Torque")
        self.advanced_speed_button = QRadioButton("Speed")
        self.advanced_position_button = QRadioButton("Position")
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
        self.load_target_preset_button = QPushButton("Load Local Preset")
        self.save_target_preset_button = QPushButton("Save Local Preset")
        self.load_target_preset_button.clicked.connect(self._load_preset_fields)
        self.save_target_preset_button.clicked.connect(self._save_local_preset)
        target_preset_layout.addWidget(self.load_target_preset_button)
        target_preset_layout.addWidget(self.save_target_preset_button)
        layout.addLayout(target_preset_layout)

        self.current_group = QGroupBox("Torque Mode")
        current_layout = QFormLayout(self.current_group)
        self.current_id_input = QLineEdit()
        self.current_iq_input = QLineEdit()
        self.current_apply_button = QPushButton("Apply Current Refs")
        self.current_apply_button.clicked.connect(self._apply_current_refs)
        current_layout.addRow("Id_ref (A)", self.current_id_input)
        current_layout.addRow("Iq_ref (A)", self.current_iq_input)
        current_layout.addRow(self.current_apply_button)
        layout.addWidget(self.current_group)

        self.speed_group = QGroupBox("Speed Mode")
        speed_layout = QFormLayout(self.speed_group)
        self.speed_ref_input = QLineEdit()
        self.speed_apply_button = QPushButton("Apply Speed Ref")
        self.speed_apply_button.clicked.connect(self._apply_speed_ref)
        speed_layout.addRow("Speed (rad/s)", self.speed_ref_input)
        speed_layout.addRow(self.speed_apply_button)
        layout.addWidget(self.speed_group)

        self.position_group = QGroupBox("Position Mode")
        position_layout = QFormLayout(self.position_group)
        self.position_ref_input = QLineEdit()
        self.position_apply_button = QPushButton("Apply Position Ref")
        self.position_apply_button.clicked.connect(self._apply_position_ref)
        position_layout.addRow("Position (rad)", self.position_ref_input)
        position_layout.addRow(self.position_apply_button)
        layout.addWidget(self.position_group)

        layout.addStretch(1)
        return widget

    def _build_pi_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        preset_layout = QHBoxLayout()
        self.load_pi_preset_button = QPushButton("Load Local Defaults")
        self.save_pi_preset_button = QPushButton("Save Local Preset")
        self.load_pi_preset_button.clicked.connect(self._load_preset_fields)
        self.save_pi_preset_button.clicked.connect(self._save_local_preset)
        preset_layout.addWidget(self.load_pi_preset_button)
        preset_layout.addWidget(self.save_pi_preset_button)
        layout.addLayout(preset_layout)

        self.current_pi_group = QGroupBox("Current Loop PI")
        current_layout = QFormLayout(self.current_pi_group)
        self.current_pi_kp_input = QLineEdit()
        self.current_pi_ki_input = QLineEdit()
        self.current_pi_defaults_button = QPushButton("Load Defaults")
        self.current_pi_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("current"))
        self.current_pi_apply_button = QPushButton("Apply Current PI")
        self.current_pi_apply_button.clicked.connect(lambda: self._apply_pi("current"))
        current_layout.addRow("Kp", self.current_pi_kp_input)
        current_layout.addRow("Ki", self.current_pi_ki_input)
        current_layout.addRow(self.current_pi_defaults_button, self.current_pi_apply_button)
        layout.addWidget(self.current_pi_group)

        self.speed_pi_group = QGroupBox("Speed Loop PI")
        speed_layout = QFormLayout(self.speed_pi_group)
        self.speed_pi_kp_input = QLineEdit()
        self.speed_pi_ki_input = QLineEdit()
        self.speed_pi_defaults_button = QPushButton("Load Defaults")
        self.speed_pi_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("speed"))
        self.speed_pi_apply_button = QPushButton("Apply Speed PI")
        self.speed_pi_apply_button.clicked.connect(lambda: self._apply_pi("speed"))
        speed_layout.addRow("Kp", self.speed_pi_kp_input)
        speed_layout.addRow("Ki", self.speed_pi_ki_input)
        speed_layout.addRow(self.speed_pi_defaults_button, self.speed_pi_apply_button)
        layout.addWidget(self.speed_pi_group)

        self.position_pi_group = QGroupBox("Position Loop PI")
        position_layout = QFormLayout(self.position_pi_group)
        self.position_pi_kp_input = QLineEdit()
        self.position_pi_ki_input = QLineEdit()
        self.position_pi_defaults_button = QPushButton("Load Defaults")
        self.position_pi_defaults_button.clicked.connect(lambda: self._apply_loop_defaults("position"))
        self.position_pi_apply_button = QPushButton("Apply Position PI")
        self.position_pi_apply_button.clicked.connect(lambda: self._apply_pi("position"))
        position_layout.addRow("Kp", self.position_pi_kp_input)
        position_layout.addRow("Ki", self.position_pi_ki_input)
        position_layout.addRow(self.position_pi_defaults_button, self.position_pi_apply_button)
        layout.addWidget(self.position_pi_group)

        layout.addStretch(1)
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
        self._set_loop_inputs("current", self._profile.current_pi)
        self._set_loop_inputs("speed", self._profile.speed_pi)
        self._set_loop_inputs("position", self._profile.position_pi)
        self._loading_profile = False

    def _set_loop_inputs(self, loop_name: str, tuning: LoopTuning):
        widgets = {
            "current": (self.current_pi_kp_input, self.current_pi_ki_input),
            "speed": (self.speed_pi_kp_input, self.speed_pi_ki_input),
            "position": (self.position_pi_kp_input, self.position_pi_ki_input),
        }
        kp_widget, ki_widget = widgets[loop_name]
        kp_widget.setText(f"{tuning.kp:.6f}")
        ki_widget.setText(f"{tuning.ki:.6f}")

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
        self._state.is_connected = bool(is_connected)
        if not self._state.is_connected:
            self._state.power_unlocked = False
            self._state.motor_enabled = False
            self._state.identify_active = False

        state = button_enable_state(self._state)
        self.connection_status_value.setText("Connected" if self._state.is_connected else "Disconnected")
        self.connection_state_detail_value.setText("Connected" if self._state.is_connected else "Disconnected")
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

        target_enabled = state["can_send_target"]
        for button in (
            self.current_apply_button,
            self.speed_apply_button,
            self.position_apply_button,
            self.current_pi_apply_button,
            self.speed_pi_apply_button,
            self.position_pi_apply_button,
            self.quick_arm_button,
            self.quick_clear_rearm_button,
        ):
            button.setEnabled(target_enabled)
        self.quick_safe_stop_button.setEnabled(self._state.is_connected and (self._state.power_unlocked or self._state.motor_enabled))
        self.export_plot_button.setEnabled(bool(self._plot_buffer.export_rows()))

        self._refresh_identify_state_panel()
        self._refresh_mode_views()
        self._refresh_session_status()
        self._sync_connect_button()

    def apply_mode_selection(self, mode: int, emit_command: bool = True):
        self._state.selected_mode = int(mode)
        self.target_label.setText(mode_target_label(self._state.selected_mode))
        self.session_mode_value.setText(mode_name(self._state.selected_mode))
        self.advanced_mode_value.setText(f"{mode_name(self._state.selected_mode)} mode active")
        self._sync_mode_buttons(self._state.selected_mode)
        self._refresh_mode_highlight()
        self._persist_profile_from_widgets()
        if emit_command and self._state.is_connected:
            self._dispatch_command(CommandBuilder.set_mode(self._state.selected_mode))

    def apply_packet(self, packet: FOCDataPacket):
        self._state.last_packet = packet
        self._state.last_packet_received_at_ms = int(time.monotonic() * 1000)
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
        self.fault_state_value.setStyleSheet(
            "color: #b91c1c; font-weight: 700;" if packet.is_fault_active else "color: #166534; font-weight: 700;"
        )
        self._plot_buffer.append_packet(packet)
        self._refresh_plot()
        self._refresh_identify_state_panel()
        if self._state.identify_active or packet.is_fault_active:
            self._record_identify_event(
                f"{packet.timestamp} ms | state={packet.foc_state} | fault={'ACTIVE' if packet.is_fault_active else 'NORMAL'}"
            )
        self.export_plot_button.setEnabled(bool(self._plot_buffer.export_rows()))
        self._refresh_session_status()

    def handle_log_line(self, level: str, message: str):
        self._log_entries.append((level, message))
        if len(self._log_entries) > self.LOG_LIMIT:
            self._log_entries = self._log_entries[-self.LOG_LIMIT :]

        if level == "TX":
            apply_command_effects(self._state, message)
            if message.startswith("CMD:IDENTIFY"):
                self._record_identify_event(f"Command: {message}")
            elif message.startswith("CMD:CLEAR_FAULT"):
                self._record_identify_event("Fault clear command sent.")
            self._show_notification("INFO", f"Sent {message}")
        elif level == "ERROR":
            self._record_identify_event(message)
            self._show_notification("ERROR", message)
        elif level == "INFO" and ("Connected" in message or "disconnected" in message.lower()):
            self._show_notification("INFO", message)

        self._render_logs()
        self._refresh_identify_state_panel()
        self._refresh_session_status()
        self.update_connection_state(self._state.is_connected)

    def _request_connect(self):
        port = self.port_combo.currentText().strip()
        if not port:
            self._show_notification("ERROR", "No serial port selected.")
            self.handle_log_line("ERROR", "No serial port selected.")
            return
        self.connect_requested.emit(port, int(self.baud_combo.currentText()))

    def _persist_profile_from_widgets(self, *_, force: bool = False):
        if self._loading_profile and not force:
            return
        self._profile.last_port = self.port_combo.currentText().strip()
        try:
            self._profile.baud_rate = int(self.baud_combo.currentText())
        except ValueError:
            self._profile.baud_rate = 115200
        self._profile.selected_mode = self._state.selected_mode
        self._profile.log_filters = [
            level for level, checkbox in self.log_filter_checks.items() if checkbox.isChecked()
        ] or list(LOG_LEVELS)
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
        self._profile.position_pi = LoopTuning(
            kp=self._float_or_default(self.position_pi_kp_input.text()),
            ki=self._float_or_default(self.position_pi_ki_input.text()),
        )
        save_gui_profile(self._profile_path, self._profile)

    def _float_or_default(self, raw_value: str, default: float = 0.0) -> float:
        try:
            return float(str(raw_value).strip())
        except ValueError:
            return default

    def _dispatch_command(self, command: str, identify_note: str | None = None):
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "Command blocked while disconnected.")
            return
        if identify_note:
            self._record_identify_event(identify_note)
        self.command_requested.emit(command)

    def _dispatch_sequence(self, commands: list[str]):
        if not self._state.is_connected:
            self.handle_log_line("ERROR", "Command blocked while disconnected.")
            return
        for command in commands:
            self.command_requested.emit(command)

    def _apply_current_refs(self):
        try:
            command = build_current_ref_command(self.current_id_input.text(), self.current_iq_input.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
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
        self.position_ref_input.setText(f"{self._float_or_default(self.position_ref_input.text()):.3f}")
        if self._state.selected_mode != 2:
            self.apply_mode_selection(2)
        self._dispatch_command(command)

    def _apply_pi(self, loop_name: str):
        inputs = {
            "current": (self.current_pi_kp_input, self.current_pi_ki_input),
            "speed": (self.speed_pi_kp_input, self.speed_pi_ki_input),
            "position": (self.position_pi_kp_input, self.position_pi_ki_input),
        }
        kp_widget, ki_widget = inputs[loop_name]
        try:
            command = build_pi_command(loop_name, kp_widget.text(), ki_widget.text())
        except ValueError as exc:
            self._show_notification("ERROR", str(exc))
            return
        kp_widget.setText(f"{self._float_or_default(kp_widget.text()):.6f}")
        ki_widget.setText(f"{self._float_or_default(ki_widget.text()):.6f}")
        self._dispatch_command(command)

    def _apply_loop_defaults(self, loop_name: str):
        defaults = {
            "current": self._profile.current_pi,
            "speed": self._profile.speed_pi,
            "position": self._profile.position_pi,
        }
        self._set_loop_inputs(loop_name, defaults[loop_name])
        self._show_notification("INFO", f"Loaded {loop_name} PI defaults from local preset.")

    def _save_local_preset(self):
        self._persist_profile_from_widgets(force=True)
        self._show_notification("INFO", f"Saved local preset to {self._profile_path}.")

    def _load_preset_fields(self):
        self._profile = load_gui_profile(self._profile_path)
        self._load_profile_into_widgets()
        self.apply_mode_selection(self._profile.selected_mode, emit_command=False)
        self._show_notification("INFO", f"Loaded local preset from {self._profile_path}.")

    def _clear_fault_with_hint(self):
        self._dispatch_command(CommandBuilder.clear_fault(), identify_note="Fault clear requested.")
        if self._state.is_connected:
            self._show_notification("INFO", "Fault clear sent. Re-arm with Unlock + Enable if hardware is clean.")

    def _on_log_filter_changed(self):
        self._render_logs()
        self._persist_profile_from_widgets()

    def _render_logs(self):
        enabled = {level for level, checkbox in self.log_filter_checks.items() if checkbox.isChecked()}
        visible = [log_line_text(level, message) for level, message in self._log_entries if level in enabled]
        self.log_view.setPlainText("\n".join(visible))

    def _record_identify_event(self, text: str):
        self._identify_entries.append(text)
        if len(self._identify_entries) > self.IDENTIFY_LOG_LIMIT:
            self._identify_entries = self._identify_entries[-self.IDENTIFY_LOG_LIMIT :]
        self._render_identify_log()

    def _render_identify_log(self):
        self.identify_log_view.setPlainText("\n".join(self._identify_entries))

    def _copy_recent_log(self):
        QApplication.clipboard().setText(self.log_view.toPlainText())
        self._show_notification("INFO", "Copied visible log lines to clipboard.")

    def _clear_log(self):
        self._log_entries.clear()
        self._render_logs()
        self._show_notification("INFO", "Cleared serial log view.")

    def _sync_connect_button(self):
        self.connect_button.setEnabled((not self._state.is_connected) and self.port_combo.count() > 0)

    def _sync_mode_buttons(self, mode: int):
        for candidate_mode, buttons in self._mode_button_map.items():
            for button in buttons:
                button.blockSignals(True)
                button.setChecked(candidate_mode == mode)
                button.blockSignals(False)

    def _refresh_mode_views(self):
        self.session_mode_value.setText(mode_name(self._state.selected_mode))
        self.target_label.setText(mode_target_label(self._state.selected_mode))
        self.advanced_mode_value.setText(f"{mode_name(self._state.selected_mode)} mode active")
        self._refresh_mode_highlight()

    def _refresh_mode_highlight(self):
        groups = {
            0: (self.current_group, "Torque Mode"),
            1: (self.speed_group, "Speed Mode"),
            2: (self.position_group, "Position Mode"),
        }
        for mode, (group, title) in groups.items():
            active = mode == self._state.selected_mode
            group.setTitle(f"{title} (Active)" if active else title)
            group.setStyleSheet(
                "QGroupBox { border: 2px solid #0f766e; margin-top: 8px; }"
                if active
                else "QGroupBox { border: 1px solid #cbd5e1; margin-top: 8px; }"
            )

    def _refresh_identify_state_panel(self):
        packet = self._state.last_packet
        self.identify_connection_value.setText("Connected" if self._state.is_connected else "Disconnected")
        self.identify_power_value.setText("Unlocked" if self._state.power_unlocked else "Locked")
        self.identify_state_value.setText(str(packet.foc_state) if packet else "--")
        if packet is None:
            fault_text = "UNKNOWN"
        else:
            fault_text = "ACTIVE" if packet.is_fault_active else "NORMAL"
        self.identify_fault_value.setText(fault_text)
        self.identify_fault_value.setStyleSheet(
            "color: #b91c1c; font-weight: 700;" if fault_text == "ACTIVE" else "color: #166534; font-weight: 700;"
        )

    def _refresh_session_status(self):
        if not self._state.is_connected:
            self.data_status_value.setText("Idle")
            self.data_status_value.setStyleSheet("color: #64748b; font-weight: 600;")
            return
        if self._state.last_packet_received_at_ms is None:
            self.data_status_value.setText("Waiting for packets")
            self.data_status_value.setStyleSheet("color: #b45309; font-weight: 600;")
            return
        now_ms = int(time.monotonic() * 1000)
        stale = is_data_stale(self._state.last_packet_received_at_ms, now_ms, self.STALE_THRESHOLD_MS)
        if stale:
            self.data_status_value.setText("STALE")
            self.data_status_value.setStyleSheet("color: #b91c1c; font-weight: 700;")
        else:
            self.data_status_value.setText("Fresh")
            self.data_status_value.setStyleSheet("color: #0f766e; font-weight: 700;")

    def _refresh_plot(self, *_):
        if pg is None or not self.plot_group.isChecked():
            return
        for channel, curve in self._plot_curves.items():
            checkbox = self.plot_channel_checks[channel]
            if checkbox.isChecked():
                times, values = self._plot_buffer.series(channel)
                curve.setData(times, values)
            else:
                curve.setData([], [])

    def _export_plot_csv(self):
        rows = self._plot_buffer.export_rows(self._selected_plot_channels())
        if not rows:
            self._show_notification("ERROR", "No plot samples are available to export.")
            return
        save_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Plot CSV",
            str(Path.home() / "foc_runtime_samples.csv"),
            "CSV Files (*.csv)",
        )
        if not save_path:
            return
        Path(save_path).write_text(format_plot_csv(rows), encoding="utf-8")
        self._show_notification("INFO", f"Exported plot samples to {save_path}.")

    def _selected_plot_channels(self) -> list[str]:
        selected = [channel for channel, checkbox in self.plot_channel_checks.items() if checkbox.isChecked()]
        return selected or list(PLOT_CHANNELS)

    def _show_notification(self, level: str, message: str):
        styles = {
            "INFO": "color: #0f766e; font-weight: 600;",
            "ERROR": "color: #b91c1c; font-weight: 700;",
        }
        prefix = "ERROR" if level == "ERROR" else "INFO"
        text = f"{prefix}: {message}"
        self.notification_label.setText(text)
        self.notification_label.setStyleSheet(f"padding: 0 8px; {styles.get(level, styles['INFO'])}")
        self.statusBar().showMessage(text, 8000)
