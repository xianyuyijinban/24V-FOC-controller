from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QFormLayout,
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
    from .data_parser import CommandBuilder, FOCDataPacket
    from .gui_logic import (
        connection_command_state,
        fault_summary_text,
        log_line_text,
        mode_target_label,
        packet_snapshot,
    )
except ImportError:
    from data_parser import CommandBuilder, FOCDataPacket
    from gui_logic import (
        connection_command_state,
        fault_summary_text,
        log_line_text,
        mode_target_label,
        packet_snapshot,
    )


class HostMainWindow(QMainWindow):
    command_requested = pyqtSignal(str)
    refresh_ports_requested = pyqtSignal()
    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    LOG_LIMIT = 300

    def __init__(self):
        super().__init__()
        self._log_entries: list[str] = []
        self._is_connected = False
        self._selected_mode = 0
        self._serial_worker = None

        self.setWindowTitle("FOC Host Debug GUI")
        self.resize(1400, 840)

        self._build_toolbar()

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._build_debug_panel(), "Debug Panel")
        self.tabs.addTab(self._build_placeholder("Identify page reserved for future implementation."), "Identify")
        self.tabs.addTab(
            self._build_placeholder("Advanced Control page reserved for future implementation."),
            "Advanced Control",
        )
        self.tabs.addTab(
            self._build_placeholder("PI Parameters page reserved for future implementation."),
            "PI Parameters",
        )

        self.apply_mode_selection(0)
        self.update_connection_state(False)

    def _build_toolbar(self):
        toolbar = QToolBar("Connection")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        toolbar.addWidget(QLabel("Port"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(180)
        self.port_combo.currentIndexChanged.connect(self._sync_connect_button)
        toolbar.addWidget(self.port_combo)

        toolbar.addSeparator()
        toolbar.addWidget(QLabel("Baud"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
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
        self.unlock_button.clicked.connect(lambda: self._send_command(CommandBuilder.unlock_power(True)))
        self.lock_button.clicked.connect(lambda: self._send_command(CommandBuilder.unlock_power(False)))
        self.enable_button.clicked.connect(lambda: self._send_command(CommandBuilder.enable_motor(True)))
        self.disable_button.clicked.connect(lambda: self._send_command(CommandBuilder.enable_motor(False)))
        self.clear_fault_button.clicked.connect(lambda: self._send_command(CommandBuilder.clear_fault()))
        for button in (
            self.unlock_button,
            self.lock_button,
            self.enable_button,
            self.disable_button,
            self.clear_fault_button,
        ):
            power_layout.addWidget(button)
        layout.addWidget(power_group)

        identify_group = QGroupBox("Identify")
        identify_layout = QVBoxLayout(identify_group)
        self.start_identify_button = QPushButton("START IDENTIFY")
        self.stop_identify_button = QPushButton("STOP IDENTIFY")
        self.start_identify_button.clicked.connect(lambda: self._send_command(CommandBuilder.start_identify()))
        self.stop_identify_button.clicked.connect(lambda: self._send_command(CommandBuilder.stop_identify()))
        identify_layout.addWidget(self.start_identify_button)
        identify_layout.addWidget(self.stop_identify_button)
        layout.addWidget(identify_group)

        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout(mode_group)
        self.mode_button_group = QButtonGroup(self)
        self.torque_mode_button = QRadioButton("Torque")
        self.speed_mode_button = QRadioButton("Speed")
        self.position_mode_button = QRadioButton("Position")
        self.mode_button_group.addButton(self.torque_mode_button, 0)
        self.mode_button_group.addButton(self.speed_mode_button, 1)
        self.mode_button_group.addButton(self.position_mode_button, 2)
        self.mode_button_group.idClicked.connect(self.apply_mode_selection)
        self.torque_mode_button.setChecked(True)
        mode_layout.addWidget(self.torque_mode_button)
        mode_layout.addWidget(self.speed_mode_button)
        mode_layout.addWidget(self.position_mode_button)
        layout.addWidget(mode_group)

        target_group = QGroupBox("Target")
        target_layout = QFormLayout(target_group)
        self.target_label = QLabel(mode_target_label(0))
        self.target_input = QLineEdit()
        self.target_input.setPlaceholderText("Enter target value")
        self.send_target_button = QPushButton("Send Target")
        self.send_target_button.clicked.connect(self._send_target)
        target_layout.addRow("Mode Label", self.target_label)
        target_layout.addRow("Value", self.target_input)
        target_layout.addRow(self.send_target_button)
        layout.addWidget(target_group)

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

        connection_group = QGroupBox("Session")
        connection_layout = QFormLayout(connection_group)
        self.packet_timestamp_value = QLabel("--")
        self.connection_state_detail_value = QLabel("Disconnected")
        connection_layout.addRow("Connection", self.connection_state_detail_value)
        connection_layout.addRow("Last Packet", self.packet_timestamp_value)
        layout.addWidget(connection_group)

        layout.addStretch(1)
        return widget

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
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("Connection events, TX/RX lines, and errors appear here.")
        log_layout.addWidget(self.log_view)
        layout.addWidget(log_group, 1)

        return widget

    def _build_placeholder(self, text: str) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        label = QLabel(text)
        label.setWordWrap(True)
        layout.addWidget(label)
        layout.addStretch(1)
        return widget

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
        current = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current and current in ports:
            self.port_combo.setCurrentText(current)
        self.port_combo.blockSignals(False)
        self._sync_connect_button()

    def update_connection_state(self, is_connected: bool):
        self._is_connected = bool(is_connected)
        state = connection_command_state(self._is_connected)
        self.connection_status_value.setText("Connected" if self._is_connected else "Disconnected")
        self.connection_state_detail_value.setText("Connected" if self._is_connected else "Disconnected")
        self.connection_status_value.setStyleSheet(
            "color: #0f766e; font-weight: 600;" if self._is_connected else "color: #b91c1c; font-weight: 600;"
        )
        self.disconnect_button.setEnabled(self._is_connected)
        self.unlock_button.setEnabled(state["can_unlock"])
        self.lock_button.setEnabled(state["can_lock"])
        self.enable_button.setEnabled(state["can_enable"])
        self.disable_button.setEnabled(state["can_disable"])
        self.clear_fault_button.setEnabled(state["can_clear_fault"])
        self.start_identify_button.setEnabled(state["can_identify"])
        self.stop_identify_button.setEnabled(state["can_identify"])
        self.send_target_button.setEnabled(state["can_send_target"])
        self._sync_connect_button()

    def apply_mode_selection(self, mode: int):
        self._selected_mode = int(mode)
        self.target_label.setText(mode_target_label(self._selected_mode))
        if self._is_connected:
            self.command_requested.emit(CommandBuilder.set_mode(self._selected_mode))

    def apply_packet(self, packet: FOCDataPacket):
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

    def handle_log_line(self, level: str, message: str):
        self._log_entries.append(log_line_text(level, message))
        if len(self._log_entries) > self.LOG_LIMIT:
            self._log_entries = self._log_entries[-self.LOG_LIMIT :]
        self.log_view.setPlainText("\n".join(self._log_entries))
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _request_connect(self):
        port = self.port_combo.currentText().strip()
        if not port:
            self.handle_log_line("ERROR", "No serial port selected.")
            return
        self.connect_requested.emit(port, int(self.baud_combo.currentText()))

    def _send_command(self, command: str):
        if not self._is_connected:
            self.handle_log_line("ERROR", "Command blocked while disconnected.")
            return
        self.command_requested.emit(command)

    def _send_target(self):
        raw_value = self.target_input.text().strip()
        try:
            target = float(raw_value)
        except ValueError:
            self.handle_log_line("ERROR", f"Invalid target value: {raw_value or '<empty>'}")
            return

        if self._selected_mode == 0:
            command = CommandBuilder.set_current_ref(0.0, target)
        elif self._selected_mode == 1:
            command = CommandBuilder.set_speed_ref(target)
        else:
            command = CommandBuilder.set_position_ref(target)
        self._send_command(command)

    def _sync_connect_button(self):
        self.connect_button.setEnabled((not self._is_connected) and self.port_combo.count() > 0)
