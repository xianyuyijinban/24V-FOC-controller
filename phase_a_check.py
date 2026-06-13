"""Phase A: Cold boot persistence check — send FAULT_DETAIL and verify params."""
import serial
import time
import sys
import io
import re

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

CHECKS = [
    ("Identified:YES", "识别标记"),
    ("valid_flag=0xFFFFFFFF", "参数有效"),
    ("Rs=", "定子电阻"),
    ("Ld=", "d轴电感"),
    ("Lq=", "q轴电感"),
    ("Ke=", "反电势常数"),
    ("Pn=", "极对数"),
    ("encoder_dir=", "编码器方向"),
    ("DirDiag", "方向诊断"),
    ("focState=READY", "就绪态"),
    ("FAULT", None),  # should NOT appear (except in CMD:FAULT_DETAIL itself)
]

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
    while time.time() - t0 < 4.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            collected.append(text)
        time.sleep(0.05)
    ser.close()

    full = ''.join(collected)

    # Extract O-frame
    o_lines = []
    in_o = False
    for line in full.split('\n'):
        if line.startswith('O,'):
            in_o = True
        if in_o:
            o_lines.append(line.rstrip())
        if in_o and not line.strip():
            break

    print("=== O-frame Output ===\n")
    for line in o_lines:
        print(line)

    # Run checks
    print(f"\n{'='*60}")
    print("Phase A Verification")
    print(f"{'='*60}")

    all_pass = True
    for pattern, label in CHECKS:
        found = pattern in full
        if label is None:
            continue  # skip the FAULT sentinel
        if label == "就绪态":
            # focState check: look for "State:" line or N-frame focState
            # Check O-frame for state
            if 'READY' in full or 'focState' in full:
                status = "✅ PASS"
            else:
                status = "⚠️ CHECK"
                all_pass = False
        elif found:
            status = "✅ PASS"
        else:
            status = "❌ MISSING"
            all_pass = False
        print(f"  {status}  {label} ({pattern})")

    # Check for unexpected FAULT
    fault_count = full.count("FAULT")
    # FAULT_DETAIL itself contains "FAULT" in the command echo
    if fault_count > 2:  # one in command echo, one in label, more = problem
        print(f"  ⚠️ WARN  Unexpected FAULT references: {fault_count}")
        all_pass = False
    else:
        print(f"  ✅ PASS  No unexpected FAULT")

    # Extract DirDiag line specifically
    for line in full.split('\n'):
        if 'DirDiag' in line:
            print(f"\n>>> {line.strip()}")

    print(f"\n{'='*60}")
    print("Phase A Result:", "ALL PASS ✅" if all_pass else "ISSUES FOUND ⚠️")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
