from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot
import serial
from serial.tools import list_ports

try:
    from .data_parser import FOCDataParser
    from .serial_service import SerialService
except ImportError:
    from data_parser import FOCDataParser
    from serial_service import SerialService


class SerialWorker(QObject):
    connection_changed = pyqtSignal(bool)
    log_line = pyqtSignal(str, str)
    packet_received = pyqtSignal(object)
    ports_updated = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self._poll_timer = None
        self._serial = None
        self._parser = None
        self._service = None

    @pyqtSlot()
    def start(self):
        if self._poll_timer is None:
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(50)
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
            self.log_line.emit("INFO", f"Detected ports: {', '.join(ports)}")
        else:
            self.log_line.emit("INFO", "No serial ports detected.")

    @pyqtSlot(str, int)
    def connect_port(self, port_name: str, baud_rate: int):
        self.disconnect_port()
        try:
            self._serial = serial.Serial(port=port_name, baudrate=baud_rate, timeout=0.05)
            self._parser = FOCDataParser()
            self._parser.set_packet_callback(self._handle_packet)
            self._service = SerialService(serial_port=self._serial, parser=self._parser)
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
                self.log_line.emit("ERROR", f"Failed to close port cleanly: {exc}")
        self._serial = None
        self._parser = None
        self._service = None
        self.connection_changed.emit(False)
        if was_connected:
            self.log_line.emit("INFO", "Serial port disconnected.")

    @pyqtSlot(str)
    def send_command(self, command: str):
        if self._service is None:
            self.log_line.emit("ERROR", "Cannot send command while disconnected.")
            return
        try:
            self._service.send_command(command)
            self.log_line.emit("TX", command.strip())
        except Exception as exc:
            self.log_line.emit("ERROR", f"Command send failed: {exc}")
            self.disconnect_port()

    def _poll_serial(self):
        if self._service is None or self._serial is None or not self._serial.is_open:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting:
                payload = self._serial.read(waiting)
                if payload:
                    self._service.handle_bytes(payload)
        except Exception as exc:
            self.log_line.emit("ERROR", f"Serial read failed: {exc}")
            self.disconnect_port()

    def _handle_packet(self, packet):
        self.packet_received.emit(packet)
        self.log_line.emit(
            "RX",
            f"{packet.timestamp} ms | state={packet.foc_state} | speed={packet.speed:.2f} rad/s",
        )
