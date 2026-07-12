from PySide6.QtCore import QObject, QTimer, Signal, Slot
import serial
from serial.tools import list_ports

try:
    from .data_parser import FOCDataParser, BinaryCurrentParser
    from .gui_logic import CurrentStreamRing
    from .serial_service import SerialService
    from .transport import Transport, DirectSerialTransport, BridgeTransport
except ImportError:
    from data_parser import FOCDataParser, BinaryCurrentParser
    from gui_logic import CurrentStreamRing
    from serial_service import SerialService
    from transport import Transport, DirectSerialTransport, BridgeTransport


class SerialWorker(QObject):
    connection_changed = Signal(bool)
    log_line = Signal(str, str)
    packet_received = Signal(object)
    ports_updated = Signal(list)
    # Current stream batch signal (emitted ~20Hz, not per-sample)
    current_samples_batch = Signal(list)
    bridge_state_changed = Signal(dict)
    wheel_status_changed = Signal(dict)

    def __init__(self, transport: Transport | None = None):
        super().__init__()
        self._poll_timer = None
        self._transport: Transport | None = transport
        self._parser = None
        self._service = None
        self._packet_count = 0
        self._unparsed_rx_notice_budget = 5

        # Binary current stream
        self._bin_parser: BinaryCurrentParser = BinaryCurrentParser()
        self._cur_ring: CurrentStreamRing = CurrentStreamRing()
        self._cur_stream_active: bool = False

    @Slot()
    def start(self):
        if self._poll_timer is None:
            # Poll at 10ms to keep up with 2kHz data inflow
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(10)
            self._poll_timer.timeout.connect(self._poll_serial)
            self._poll_timer.start()
        self.refresh_ports()

    @Slot()
    def stop(self):
        if self._poll_timer is not None:
            self._poll_timer.stop()
        self.disconnect_port()

    @Slot()
    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.ports_updated.emit(ports)
        if ports:
            self.log_line.emit("INFO", f"Detected serial ports: {', '.join(ports)}")
        else:
            self.log_line.emit("INFO", "No serial ports detected.")

    @Slot(str, int)
    def connect_port(self, port_name: str, baud_rate: int):
        self.disconnect_port()
        try:
            # Try BridgeTransport first — if bridge is running, use IPC
            device_connected = True
            bridge = BridgeTransport()
            if bridge.open():
                self._transport = bridge
                bridge.device_connection_changed.connect(self._on_bridge_device_connection)
                bridge.connection_state_changed.connect(self.bridge_state_changed)
                bridge.wheel_status_changed.connect(self.wheel_status_changed)
                bridge.bridge_error.connect(self._on_bridge_error)
                device_connected = bridge.device_connected()
                bridge_state = bridge.connection_state()
                self.bridge_state_changed.emit(bridge_state)
                if bridge.wheel_status():
                    self.wheel_status_changed.emit(bridge.wheel_status())
                if (not device_connected) or str(bridge_state.get("port", "")).upper() != port_name.upper():
                    bridge.request_device_connection(port_name, baud_rate)
                    device_connected = False
                    self.log_line.emit(
                        "INFO",
                        f"Requested Bridge connection to {port_name} @ {baud_rate}",
                    )
                if device_connected:
                    self.log_line.emit("INFO", "Connected via FOC_Device_Bridge")
                else:
                    self.log_line.emit(
                        "INFO",
                        "FOC_Device_Bridge is running but has no controller connection",
                    )
            else:
                # Fall back to direct serial
                self._transport = DirectSerialTransport(port_name, baud_rate)
                if not self._transport.open():
                    raise ConnectionError(f"Failed to open {port_name}")
                self.log_line.emit("INFO", f"Connected to {port_name} @ {baud_rate}")

            self._parser = FOCDataParser()
            self._parser.set_packet_callback(self._handle_packet)
            self._service = SerialService(
                serial_port=None,  # no direct serial access; use transport
                parser=self._parser,
                diagnostic_callback=self._handle_diagnostic_line,
            )
            self._packet_count = 0
            self._unparsed_rx_notice_budget = 5
            # Reset binary parser state
            self._bin_parser = BinaryCurrentParser()
            self._cur_ring = CurrentStreamRing()
            self._cur_stream_active = False
            self.connection_changed.emit(device_connected)
        except Exception as exc:
            self._transport = None
            self._parser = None
            self._service = None
            self.connection_changed.emit(False)
            self.log_line.emit("ERROR", f"Failed to open {port_name}: {exc}")

    @Slot()
    def disconnect_port(self):
        was_connected = bool(self._transport is not None and self._transport.is_open())
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception as exc:
                self.log_line.emit("ERROR", f"Error closing transport: {exc}")
        self._transport = None
        self._parser = None
        self._service = None
        self._packet_count = 0
        self._cur_stream_active = False
        self.connection_changed.emit(False)
        if was_connected:
            self.log_line.emit("INFO", "Serial port disconnected.")

    @Slot(str)
    def send_command(self, command: str):
        if self._transport is None or not self._transport.is_open():
            self.log_line.emit("ERROR", "Not connected, cannot send command.")
            return
        try:
            data = (command + "\n").encode("utf-8", errors="replace")
            self._transport.write(data)
            self.log_line.emit("TX", command.strip())
        except Exception as exc:
            self.log_line.emit("ERROR", f"Command send failed: {exc}")
            self.disconnect_port()

    def current_ring(self) -> CurrentStreamRing:
        return self._cur_ring

    def current_parser(self) -> BinaryCurrentParser:
        return self._bin_parser

    @Slot(bool)
    def request_bridge_wheel_enable(self, enable: bool):
        if not isinstance(self._transport, BridgeTransport):
            self.log_line.emit("ERROR", "滚轮鼠标需要先启动 FOC Device Bridge。")
            return
        if not self._transport.is_ipc_connected():
            self.log_line.emit("ERROR", "FOC Device Bridge IPC 未连接。")
            return
        if not self._transport.request_wheel_enable(enable):
            self.log_line.emit("ERROR", "无法向 Bridge 发送滚轮启停请求。")

    def _poll_serial(self):
        if self._transport is None or not self._transport.is_open():
            return

        try:
            waiting = self._transport.bytes_available()
            if waiting:
                payload = self._transport.read(waiting)
                if payload:
                    # Phase 1: Extract binary current frames
                    bin_samples, text_bytes = self._bin_parser.feed(payload)

                    if bin_samples:
                        self._cur_stream_active = True
                        self._cur_ring.extend(bin_samples)
                        # Update parser stats in ring
                        self._cur_ring.update_parser_stats(
                            self._bin_parser.crc_errors,
                            self._bin_parser.seq_gaps,
                        )
                        # Emit batch (caller throttles UI refresh, not us)
                        self.current_samples_batch.emit(bin_samples)

                    # Phase 2: Feed residual text bytes to ASCII parser
                    if text_bytes:
                        before_packets = self._packet_count
                        read_result = self._service.handle_bytes(text_bytes) if self._service else None
                        if (
                            read_result is not None
                            and self._packet_count == before_packets
                            and read_result.diagnostic_lines == 0
                            and self._unparsed_rx_notice_budget > 0
                        ):
                            preview = text_bytes[:32].hex(" ")
                            suffix = " ..." if len(text_bytes) > 32 else ""
                            self.log_line.emit("RX", f"Received {len(text_bytes)} bytes but no telemetry packet parsed: {preview}{suffix}")
                            self._unparsed_rx_notice_budget -= 1
        except Exception as exc:
            self.log_line.emit("ERROR", f"Serial read failed: {exc}")
            self.disconnect_port()

    def _handle_packet(self, packet):
        self._packet_count += 1
        self.packet_received.emit(packet)

    def _handle_diagnostic_line(self, line: str):
        self.log_line.emit("RX", line)

    def _on_bridge_device_connection(self, connected: bool):
        self.connection_changed.emit(bool(connected))
        if connected:
            self.log_line.emit("INFO", "Bridge connected to the motor controller")
        else:
            self.log_line.emit("ERROR", "Bridge lost the motor controller connection")

    def _on_bridge_error(self, message: str):
        self.log_line.emit("ERROR", f"Bridge: {message}")
