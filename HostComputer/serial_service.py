from dataclasses import dataclass


@dataclass
class SerialReadResult:
    diagnostic_lines: int = 0


class SerialService:
    def __init__(self, serial_port, parser, diagnostic_callback=None):
        self.serial_port = serial_port
        self.parser = parser
        self._diagnostic_lines = 0
        if diagnostic_callback is not None and hasattr(self.parser, "set_diagnostic_callback"):
            self.parser.set_diagnostic_callback(self._wrap_diagnostic_callback(diagnostic_callback))

    def _wrap_diagnostic_callback(self, callback):
        def wrapped(line: str):
            self._diagnostic_lines += 1
            callback(line)

        return wrapped

    def send_command(self, command: str):
        self.serial_port.write(command.encode("utf-8"))

    def handle_bytes(self, payload: bytes):
        before = self._diagnostic_lines
        self.parser.feed_data(payload)
        return SerialReadResult(diagnostic_lines=self._diagnostic_lines - before)
