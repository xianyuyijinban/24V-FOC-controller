"""Quick diagnostic: check raw serial data and board state."""
import serial
import time
import sys
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"
BAUD = 1152000

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    parser = FOCDataParser()

    pkts = []
    parser.set_packet_callback(lambda p: pkts.append(p))

    # Drain any boot noise
    time.sleep(0.2)
    raw = ser.read(4096)
    print(f"Initial drain: {len(raw)} bytes")
    if raw:
        # Show first 500 chars
        text = raw.decode('utf-8', errors='ignore')
        print(f"Raw sample:\n{text[:800]}")

    # Feed to parser
    parser.feed_data(raw)
    print(f"Packets after drain: {len(pkts)}")

    # Now send commands to check state
    commands = [
        ("UNLOCK", "CMD:UNLOCK,1\n"),
        ("FAULT_DETAIL", "CMD:FAULT_DETAIL\n"),
    ]

    for label, cmd in commands:
        print(f"\n--- Sending {label} ---")
        ser.write(cmd.encode('utf-8'))
        time.sleep(0.3)
        data = ser.read(4096)
        parser.feed_data(data)
        text = data.decode('utf-8', errors='ignore')
        print(f"Response ({len(data)} bytes):\n{text[:600]}")

    # Check latest packet
    if pkts:
        last = pkts[-1]
        print(f"\nLast packet: state={last.foc_state}, vbus={last.vbus:.2f}, "
              f"control_mode={last.control_mode}, fault={last.app_fault_code}, "
              f"fault_flags=0x{last.fault_flags:08X}")
        print(f"  Iq_ref={last.Iq_ref:.4f}, Iq={last.Iq:.4f}, "
              f"Id_ref={last.Id_ref:.4f}, Id={last.Id:.4f}")

    ser.close()

if __name__ == "__main__":
    main()
