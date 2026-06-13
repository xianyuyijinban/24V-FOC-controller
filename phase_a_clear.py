"""Phase A: Clear fault then FAULT_DETAIL to verify clean cold boot state."""
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
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Step 1: CLEAR_FAULT
    print("--- SENDING: CMD:CLEAR_FAULT ---", flush=True)
    ser.write(b"CMD:CLEAR_FAULT\r\n")
    ser.flush()
    time.sleep(0.5)

    # Step 2: FAULT_DETAIL
    print("--- SENDING: CMD:FAULT_DETAIL ---\n", flush=True)
    ser.write(b"CMD:FAULT_DETAIL\r\n")
    ser.flush()

    collected = []
    t0 = time.time()
    while time.time() - t0 < 4.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected.append(text)
        time.sleep(0.05)
    ser.close()

    full = ''.join(collected)

    # Print only the diagnostic text (between FAULT_DETAIL banner and N-frames)
    lines = full.split('\n')
    in_diag = False
    for line in lines:
        if 'FAULT DETECTED' in line or '==========' in line:
            in_diag = True
        if in_diag:
            # Stop at N-frame telemetry
            if line.startswith('N,') or line.startswith('C,'):
                break
            print(line.rstrip())

    # Key checks
    print(f"\n{'='*60}")
    print("Phase A Key Fields:")
    print(f"{'='*60}")
    checks = {
        'Identified:YES': '识别标记',
        'State:    READY': '就绪态 (READY)',
        'State:    IDLE': '就绪态 (IDLE)',
        'Pn=': '极对数',
        'enc_dir=': '编码器方向',
        'offset=': 'theta_offset',
        'Rs=': '定子电阻',
        'Ld=': 'd轴电感',
        'Lq=': 'q轴电感',
        'Ke=': '反电势',
        'DirDiag': '方向诊断',
        'zero=': '零位',
    }
    for pattern, label in checks.items():
        if pattern in full:
            print(f"  ✅ {label} ({pattern})")
        else:
            print(f"  ❌ MISSING: {label}")

    for pat in ['valid_flag']:
        if pat in full:
            print(f"  ✅ {pat}")

if __name__ == "__main__":
    main()
