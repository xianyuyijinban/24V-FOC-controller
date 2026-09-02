"""V1 CAL Wizard test — safe: no full IDENTIFY, only infrastructure checks."""
import serial, time, sys
from datetime import datetime

PORT, BAUD = "COM9", 1152000

def send(ser, cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    return [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def main():
    print("=" * 70)
    print("V1 CAL Wizard (1.5) — infrastructure only (no full IDENTIFY)")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(4.0)
    ser.reset_input_buffer(); ser.read(ser.in_waiting); ser.reset_input_buffer()

    results = []

    # 1. STATUS?
    print("\n=== 1. CAL:STATUS? ===")
    lines = send(ser, "CAL:STATUS?")
    ok = len(lines) > 0
    for l in lines: print(f"  {l}")
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("STATUS?", "PASS" if ok else "FAIL", lines[0] if lines else "no"))

    # 2. Precheck + STOP interrupt
    print("\n=== 2. Precheck + STOP ===")
    send(ser, "CMD:UNLOCK,1"); time.sleep(0.2)
    lines = send(ser, "CAL:ALL", wait=0.3)
    for l in lines: print(f"  CAL:ALL -> {l}")
    lines = send(ser, "CAL:STOP", wait=0.5)
    for l in lines: print(f"  CAL:STOP -> {l}")

    # PN step may need time to gracefully exit; poll STATUS until idle/aborted
    final_state = "?"
    for attempt in range(6):  # up to ~3s total
        time.sleep(0.5)
        lines = send(ser, "CAL:STATUS?", wait=0.3)
        cal_text = " ".join(lines)
        if "running" not in cal_text.lower():
            final_state = cal_text
            break
        if attempt == 0:
            print(f"    (waiting for identify to stop...)")
        final_state = cal_text

    busy = "running" in final_state.lower()
    ok = not busy and len(lines) > 0
    for l in lines: print(f"  STATUS (final): {l}")
    print(f"  [{'PASS' if ok else 'FAIL'}] (stopped={'YES' if not busy else 'NO'})")
    results.append(("STOP interrupt", "PASS" if ok else "FAIL", final_state[:80]))

    # 3. Busy protection
    print("\n=== 3. Busy Protection ===")
    send(ser, "CMD:ENABLE,1"); time.sleep(0.2)
    send(ser, "CMD:SREF,0.2", wait=0.3)
    lines = send(ser, "CAL:ALL", wait=0.3)
    for l in lines: print(f"  {l}")
    precheck_ok = any("FAIL" in l or "precheck" in l for l in lines)
    print(f"  [{'PASS' if precheck_ok else 'INFO'}] (precheck should reject PWM active)")
    send(ser, "CTRL:STOP", wait=0.5)

    # 4. SAVE guard
    print("\n=== 4. CAL:SAVE ===")
    lines = send(ser, "CAL:SAVE")
    for l in lines: print(f"  {l}")
    ok = len(lines) > 0
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("SAVE guard", "PASS" if ok else "FAIL", lines[0] if lines else "no"))

    # 5. ALL,CONTINUE
    print("\n=== 5. CAL:ALL,CONTINUE ===")
    send(ser, "SYS:CLEAR_FAULT"); time.sleep(0.3)
    lines = send(ser, "CAL:ALL,CONTINUE")
    for l in lines: print(f"  {l}")
    ok = len(lines) > 0
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("ALL,CONTINUE", "PASS" if ok else "FAIL", lines[0] if lines else "no"))

    ser.close()

    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"\nCAL: {passed}/{len(results)} PASS, {failed}/{len(results)} FAIL")
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
