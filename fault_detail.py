"""Send CMD:FAULT_DETAIL and capture DirDiag line."""
import serial
import time
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.reset_input_buffer()

    # Drain stale telemetry for 200ms
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Send FAULT_DETAIL
    print("--- SENDING: CMD:FAULT_DETAIL ---", flush=True)
    ser.write(b"CMD:FAULT_DETAIL\r\n")
    ser.flush()

    # Collect output for 3 seconds
    collected = []
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected.append(text)
        time.sleep(0.05)

    ser.close()
    full = ''.join(collected)

    # Extract DirDiag line
    for line in full.split('\n'):
        if 'DirDiag' in line:
            print(f"\n{'='*70}")
            print(line.strip())
            print(f"{'='*70}")

    # Also print any other O-frame content
    print("\n--- Full O-frame output ---")
    in_o = False
    for line in full.split('\n'):
        if line.startswith('O,'):
            in_o = True
        if in_o:
            print(line.rstrip())
        if in_o and not line.strip():
            in_o = False

if __name__ == "__main__":
    main()
