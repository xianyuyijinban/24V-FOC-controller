"""Phase C Stage 1: Enable position closed-loop, monitor for 3s."""
import serial
import time
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

CMDS = [
    "CMD:CLEAR_FAULT",
    "CMD:UNLOCK,1",
    "CMD:MODE,2",
    "CMD:PREF,0.000",
    "CMD:ENABLE,1",
]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    time.sleep(0.2)
    ser.reset_input_buffer()

    for cmd in CMDS:
        print(f"--- SENDING: {cmd} ---", flush=True)
        ser.write((cmd + "\r\n").encode('utf-8'))
        ser.flush()
        time.sleep(0.3)

    print("\n=== ENABLE sent. Monitoring for 3 seconds ===\n", flush=True)

    t0 = time.time()
    fault_seen = False
    max_iq = 0.0
    max_ia = 0.0

    while time.time() - t0 < 3.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            for line in text.split('\n'):
                # Check N-frames for current and faults
                if line.startswith('N,'):
                    parts = line.split(',')
                    if len(parts) >= 22:
                        try:
                            iq = abs(float(parts[7]))   # iqText
                            ia = abs(float(parts[23]))  # iaText
                            ib = abs(float(parts[24]))  # ibText
                            ic = abs(float(parts[25]))  # icText
                            fault = parts[9]  # faultFlags
                            max_iq = max(max_iq, iq)
                            max_ia = max(max_ia, ia, ib, ic)
                            if fault != '0x00000000':
                                fault_seen = True
                                print(f"  ⚠️ FAULT: {line.strip()}", flush=True)
                        except (ValueError, IndexError):
                            pass
                # Show any fault/error output
                if any(kw in line for kw in ['FAULT', 'ERROR', 'OVER', 'fault']):
                    if 'FAULT_DETAIL' not in line and 'CMD:' not in line:
                        print(f"  {line.strip()}", flush=True)
            sys.stdout.flush()
        time.sleep(0.02)

    print(f"\n=== Stage 1 Results ===", flush=True)
    print(f"  Fault seen:  {'YES ⚠️' if fault_seen else 'NO ✅'}", flush=True)
    print(f"  Max |Iq|:    {max_iq:.3f} A", flush=True)
    print(f"  Max |Iabc|:  {max_ia:.3f} A", flush=True)

    if fault_seen or max_iq > 1.0:
        print(f"\n  ⛔ STOP: {'Overcurrent/fault detected' if fault_seen else 'Current too high'}. Do NOT proceed to PREF steps.", flush=True)
    else:
        print(f"\n  ✅ Stage 1 PASS. Ready for small PREF steps (0.087 rad).", flush=True)

    ser.close()

if __name__ == "__main__":
    main()
