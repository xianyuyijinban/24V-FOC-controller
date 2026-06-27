"""Query COG with retries and longer delays. Query available params only."""
import serial, time, sys

PORT = "COM9"
BAUD = 1152000

def query(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n") if l.strip() and l[0] not in "CN"]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Turn off telemetry, wait for TX drain
    query(ser, "TELEM:OFF")
    time.sleep(1.0)

    # COG: try with various delays
    print("=== COG Query (retry loop) ===")
    for delay in [0.3, 0.5, 1.0, 2.0]:
        lines = query(ser, "CMD:COG_CFG?", wait=delay)
        print(f"  delay={delay}s: {lines if lines else '(no response)'}")
        if lines:
            break

    # RS_ADAPTIVE
    print("\n=== RS_ADAPTIVE ===")
    lines = query(ser, "CMD:RS_FF_ADAPTIVE?")
    print(f"  {lines}")

    # Try GAIN:PI_CURRENT? (new format)
    print("\n=== GAIN queries ===")
    for cmd in ["GAIN:PI_CURRENT?", "CMD:GAIN,PI_CURRENT?"]:
        lines = query(ser, cmd)
        print(f"  {cmd}: {lines if lines else '(no response)'}")

    # DIAG:PWM_DIAG summary
    print("\n=== PWM_DIAG ===")
    lines = query(ser, "DIAG:PWM_DIAG", wait=1.0)
    for l in lines[:5]:
        print(f"  {l[:200]}")

    ser.write(b"TELEM:ON\n")
    time.sleep(0.2)
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
