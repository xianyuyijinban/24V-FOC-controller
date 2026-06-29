from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot
import serial
from serial.tools import list_ports

try:
    from .data_parser import FOCDataParser, BinaryCurrentParser
    from .gui_logic import CurrentStreamRing
    from .serial_service import SerialService
except ImportError:
    from data_parser import FOCDataParser, BinaryCurrentParser
    from gui_logic import CurrentStreamRing
    from serial_service import SerialService


class SerialWorker(QObject):
    connection_changed = pyqtSignal(bool)
    log_line = pyqtSignal(str, str)
    packet_received = pyqtSignal(object)
    ports_updated = pyqtSignal(list)
    # Current stream batch signal (emitted ~20Hz, not per-sample)
    current_samples_batch = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self._poll_timer = None
        self._serial = None
        self._parser = None
        self._service = None
        self._packet_count = 0
        self._unparsed_rx_notice_budget = 5

        # Binary current stream
        self._bin_parser: BinaryCurrentParser = BinaryCurrentParser()
        self._cur_ring: CurrentStreamRing = CurrentStreamRing()
        self._cur_stream_active: bool = False

    @pyqtSlot()
    def start(self):
        if self._poll_timer is None:
            # Poll at 10ms to keep up with 2kHz data inflow
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(10)
            self._poll_timer.timeout.connect(self._poll_serial)
            self._poll_timer.start()
        self.refresh_ports()

    @pyqtSlot()
    def stop(self):
        if self._poll_timer is not None:
            self._poll_timer.stop()
        self.disconnect_port()

    @pyqtSlot()
    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.ports_updated.emit(ports)
        if ports:
            self.log_line.emit("INFO", f"Detected serial ports: {', '.join(ports)}")
        else:
            self.log_line.emit("INFO", "No serial ports detected.")

    @pyqtSlot(str, int)
    def connect_port(self, port_name: str, baud_rate: int):
        self.disconnect_port()
        try:
            self._serial = serial.Serial(port=port_name, baudrate=baud_rate, timeout=0.05)
            self._parser = FOCDataParser()
            self._parser.set_packet_callback(self._handle_packet)
            self._service = SerialService(
                serial_port=self._serial,
                parser=self._parser,
                diagnostic_callback=self._handle_diagnostic_line,
            )
            self._packet_count = 0
            self._unparsed_rx_notice_budget = 5
            # Reset binary parser state
            self._bin_parser = BinaryCurrentParser()
            self._cur_ring = CurrentStreamRing()
            self._cur_stream_active = False
            self.connection_changed.emit(True)
            self.log_line.emit("INFO", f"Connected to {port_name} @ {baud_rate}.")
        except Exception as exc:
            self._serial = None
            self._parser = None
            self._service = None
            self.connection_changed.emit(False)
            self.log_line.emit("ERROR", f"Failed to open {port_name}: {exc}")

    @pyqtSlot()
    def disconnect_port(self):
        was_connected = bool(self._serial and self._serial.is_open)
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception as exc:
                self.log_line.emit("ERROR", f"Error closing serial port: {exc}")
        self._serial = None
        self._parser = None
        self._service = None
        self._packet_count = 0
        self._cur_stream_active = False
        self.connection_changed.emit(False)
        if was_connected:
            self.log_line.emit("INFO", "Serial port disconnected.")

    @pyqtSlot(str)
    def send_command(self, command: str):
        if self._service is None:
            self.log_line.emit("ERROR", "Not connected, cannot send command.")
            return
        try:
            self._service.send_command(command)
            self.log_line.emit("TX", command.strip())
        except Exception as exc:
            self.log_line.emit("ERROR", f"Command send failed: {exc}")
            self.disconnect_port()

    def current_ring(self) -> CurrentStreamRing:
        return self._cur_ring

    def current_parser(self) -> BinaryCurrentParser:
        return self._bin_parser

    def _poll_serial(self):
        if self._service is None or self._serial is None or not self._serial.is_open:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting:
                payload = self._serial.read(waiting)
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
                        read_result = self._service.handle_bytes(text_bytes)
                        if (
                            self._packet_count == before_packets
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
