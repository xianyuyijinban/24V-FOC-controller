"""Phase B verify: CLEAR_FAULT then FAULT_DETAIL, check zero persisted."""
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

    print("--- SENDING: CMD:CLEAR_FAULT ---", flush=True)
    ser.write(b"CMD:CLEAR_FAULT\r\n")
    ser.flush()
    time.sleep(0.5)

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

    # Extract diagnostic text
    lines = full.split('\n')
    in_diag = False
    for line in lines:
        if 'DIAGNOSTIC SNAPSHOT' in line or '==========' in line:
            in_diag = True
        if in_diag:
            if line.startswith('N,') or line.startswith('C,'):
                break
            print(line.rstrip())

    # Key verification
    print(f"\n{'='*60}")
    print("Phase B Cold-Boot Persistence:")
    print(f"{'='*60}")

    home_offset = "-2.934"
    checks = {
        'Identified:YES': '识别标记',
        'State:    READY': '就绪态 READY',
        'AppFault: 0': '无故障',
        'Pn=11': '极对数',
        'enc_dir=-1': '编码器方向',
        'offset=1.069 rad': 'theta_offset 不变',
        f'zero={home_offset} rad': 'zero 持久化 (=HOME offset)',
    }
    for pattern, label in checks.items():
        if pattern in full or pattern.replace('-2.934', '-2.935') in full:
            print(f"  ✅ {label}")
        else:
            print(f"  ❌ {label} — expected '{pattern}' not found")

    # Show actual ThetaDiag line
    for line in full.split('\n'):
        if 'ThetaDiag' in line:
            print(f"\n>>> {line.strip()}")

if __name__ == "__main__":
    main()
