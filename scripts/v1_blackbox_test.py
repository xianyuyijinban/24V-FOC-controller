"""V1 BLACKBOX test (1.6)."""
import serial, time, sys

PORT, BAUD = "COM9", 1152000

def send(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    return [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(4.0)
    ser.reset_input_buffer(); ser.read(ser.in_waiting); ser.reset_input_buffer()

    results = []
    print("V1 BLACKBOX (1.6)")
    print("=" * 70)

    # 1. Status
    print("\n=== BLACKBOX? ===")
    lines = send(ser, "DIAG:BLACKBOX?")
    ok = len(lines) > 0
    for l in lines: print(f"  {l}")
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("BLACKBOX?", "PASS" if ok else "FAIL"))

    # Parse count
    count = 0
    for l in lines:
        if "count=" in l:
            try:
                count = int(l.split("count=")[1].split(",")[0])
            except: pass

    # 2. HEAD (last 5)
    print(f"\n=== BLACKBOX,HEAD ({count} total) ===")
    lines = send(ser, "DIAG:BLACKBOX,HEAD", wait=0.8)
    ok = len(lines) > 0
    csv_ok = any(l.startswith("BB,") for l in lines)  # at least one valid BB line
    for l in lines[:6]: print(f"  {l[:120]}")
    print(f"  [{'PASS' if ok and csv_ok else 'FAIL'}] (CSV format={'OK' if csv_ok else 'FAIL'})")
    results.append(("HEAD", "PASS" if ok and csv_ok else "FAIL"))

    # 3. DUMP (first 5 and last 2 of dump)
    print(f"\n=== BLACKBOX,DUMP ===")
    lines = send(ser, "DIAG:BLACKBOX,DUMP", wait=2.0)
    bb_lines = [l for l in lines if l.startswith("BB,")]
    dump_end = [l for l in lines if "DUMP" in l or "end" in l.lower()]
    ok = len(bb_lines) > 0
    print(f"  Samples: {len(bb_lines)}")
    for l in bb_lines[:3]: print(f"  {l[:120]}")
    if len(bb_lines) > 3: print(f"  ...")
    for l in bb_lines[-2:]: print(f"  {l[:120]}")
    for l in dump_end: print(f"  {l}")
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("DUMP", "PASS" if ok else "FAIL"))

    # 4. CLEAR
    print("\n=== BLACKBOX,CLEAR ===")
    lines = send(ser, "DIAG:BLACKBOX,CLEAR")
    for l in lines: print(f"  {l}")
    ok = any("OK" in l or "CLEAR" in l for l in lines)
    print(f"  [{'PASS' if ok else 'FAIL'}]")
    results.append(("CLEAR", "PASS" if ok else "FAIL"))

    # Verify cleared
    lines = send(ser, "DIAG:BLACKBOX?")
    for l in lines: print(f"  After CLEAR: {l}")

    ser.close()

    passed = sum(1 for _, s in results if s == "PASS")
    print(f"\nBLACKBOX: {passed}/{len(results)} PASS")
    return 0 if passed == len(results) else 1

if __name__ == "__main__":
    sys.exit(main())
