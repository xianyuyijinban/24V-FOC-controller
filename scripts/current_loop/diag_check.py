"""Quick check: board state + current loop diag after reflash."""
import serial
import time
import sys
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(HOST_DIR))
from HostComputer.data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"
BAUD = 1152000

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    parser = FOCDataParser()

    def drain(dur):
        deadline = time.time() + dur
        while time.time() < deadline:
            ser.timeout = max(0.01, min(deadline - time.time(), 0.1))
            data = ser.read(4096)
            if data:
                parser.feed_data(data)

    drain(0.5)

    # Send commands
    for label, cmd in [
        ("VBUS_LIMIT", "CMD:VBUS_LIMIT,8.000,30.000\n"),
        ("CLEAR_FAULT", "CMD:CLEAR_FAULT\n"),
        ("UNLOCK", "CMD:UNLOCK,1\n"),
        ("MODE=0", "CMD:MODE,0\n"),
        ("ENABLE=1", "CMD:ENABLE,1\n"),
        ("FAULT_DETAIL", "CMD:FAULT_DETAIL\n"),
    ]:
        print(f"\n--- {label} ---")
        ser.write(cmd.encode('utf-8'))
        drain(0.4)
        # Print received text
        data = ser.read(4096)
        if data:
            text = data.decode('utf-8', errors='ignore')
            # Only show CurrentLoopDiag line + context
            for line in text.splitlines():
                if 'CurrentLoopDiag' in line or 'Power:' in line or 'State:' in line or 'Vq' in line:
                    print(f"  {line.strip()}")

    ser.close()

if __name__ == "__main__":
    main()
