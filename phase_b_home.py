"""Phase B: HOME then FAULT_DETAIL to capture zero offset."""
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

    # Step 1: HOME
    print("--- SENDING: CMD:HOME ---\n", flush=True)
    ser.write(b"CMD:HOME\r\n")
    ser.flush()

    # Collect HOME response + a bit
    collected = []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected.append(text)
            print(text, end='', flush=True)
        time.sleep(0.02)

    # Look for HOME response
    full_home = ''.join(collected)
    for line in full_home.split('\n'):
        if 'HOME' in line:
            print(f"\n>>> {line.strip()}")

    # Step 2: FAULT_DETAIL
    print("\n--- SENDING: CMD:FAULT_DETAIL ---\n", flush=True)
    ser.write(b"CMD:FAULT_DETAIL\r\n")
    ser.flush()

    collected2 = []
    t0 = time.time()
    while time.time() - t0 < 4.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected2.append(text)
        time.sleep(0.05)
    ser.close()

    full = ''.join(collected2)

    # Extract key lines
    print("=== Key Fields ===\n")
    for line in full.split('\n'):
        line_s = line.strip()
        if any(kw in line_s for kw in ['HOME,OK', 'ThetaDiag', 'State:', 'AppFault',
                                         'Identified', 'zero=', 'offset=',
                                         'enc_dir', 'Pn=', 'DirDiag']):
            print(f"  {line_s}")

    # Verification summary
    print(f"\n{'='*60}")
    print("Phase B HOME Verification:")
    print(f"{'='*60}")

    full_all = full_home + full
    checks = [
        ('HOME,OK', 'HOME 命令成功'),
        ('Identified:YES', '识别标记'),
        ('State:    READY', '就绪态 READY'),
        ('AppFault: 0', '无故障'),
        ('enc_dir=-1', '编码器方向'),
        ('zero=', '零位已设置'),
    ]
    for pattern, label in checks:
        if pattern in full_all:
            # Extract value if possible
            print(f"  ✅ {label} — found '{pattern}'")
        else:
            print(f"  ❌ {label} — MISSING")

if __name__ == "__main__":
    main()
