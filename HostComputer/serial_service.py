class SerialService:
    def __init__(self, serial_port, parser):
        self.serial_port = serial_port
        self.parser = parser

    def send_command(self, command: str):
        self.serial_port.write(command.encode("utf-8"))

    def handle_bytes(self, payload: bytes):
        self.parser.feed_data(payload)
