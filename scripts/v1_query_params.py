"""Query key FOC parameters — V1 baseline regression."""
import serial, time, sys

PORT = "COM9"
BAUD = 230400

def is_telemetry(line):
    """Only filter true telemetry frames: N,... and C,... (compact/normal packets)."""
    return line.startswith("N,") or line.startswith("C,")

def query(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n") if l.strip() and not is_telemetry(l.strip())]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    # Disable telemetry for clean responses
    print("TELEM:OFF =>", query(ser, "TELEM:OFF"))
    time.sleep(0.5)

    tests = [
        ("FW_INFO",    "CMD:FW_INFO?"),
        ("MOTION_CFG", "CMD:MOTION_CFG?"),
        ("COG",        "CMD:COG_CFG?"),
        ("BEMF",       "CMD:BEMF_CFG?"),
        ("RS_MODE",    "CMD:RS_FF_MODE?"),
        ("RS_ADAPTIVE","CMD:RS_FF_ADAPTIVE?"),
    ]

    all_pass = True
    for label, cmd in tests:
        lines = query(ser, cmd, wait=0.5)
        print(f"{label}: {cmd}")
        if lines:
            for l in lines:
                print(f"  -> {l}")
        else:
            print(f"  -> (no text response)")
            all_pass = False
        print()

    ser.write(b"TELEM:ON\n")
    time.sleep(0.1)
    ser.close()

    if all_pass:
        print("ALL QUERIES PASSED")
    else:
        print("SOME QUERIES FAILED (see above)")
    return 0 if all_pass else 1

if __name__ == "__main__":
    sys.exit(main())
