"""Phase A: dump all raw UART data."""
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
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("--- SENDING: CMD:FAULT_DETAIL ---\n", flush=True)
    ser.write(b"CMD:FAULT_DETAIL\r\n")
    ser.flush()

    collected = []
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected.append(text)
            print(text, end='', flush=True)
        time.sleep(0.02)
    ser.close()

    full = ''.join(collected)
    print(f"\n\n--- Total bytes: {len(full)} ---")

    # Also check what we got
    for kw in ['Identified', 'valid_flag', 'Rs=', 'Ld=', 'Lq=', 'Ke=', 'Pn=', 'DirDiag', 'focState', 'FAULT', 'State:', 'zero']:
        count = full.count(kw)
        if count > 0:
            print(f"  '{kw}': {count} occurrences")

if __name__ == "__main__":
    main()
