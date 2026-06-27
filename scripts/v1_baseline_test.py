"""
V1 Baseline Regression Test (1.1) — Robust version
Handles:
  - CLEAR_FAULT has no text response (silent operation)
  - COG/BEMF queries use DrvUart_StartSend (can be dropped if TX busy)
  - Response formats vary (some OK-prefixed, some CSV, some via telemetry path)
"""
import serial
import time
import sys
from datetime import datetime

PORT = "COM9"
BAUD = 230400

def flush_and_send(ser, cmd, wait=0.35):
    """Send command and collect response, filtering telemetry frames."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("utf-8"))
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    lines = [l.strip() for l in text.split("\n") if l.strip() and l[0] not in 'CN']
    return lines

def retry_send(ser, cmd, wait=0.35, max_retries=3):
    """Retry command if no text response (COG/BEMF can be dropped)."""
    for attempt in range(max_retries):
        lines = flush_and_send(ser, cmd, wait)
        if lines:
            return lines, attempt + 1
        time.sleep(0.10)
    return [], max_retries

def check_response(label, lines, must_have, must_not_have):
    """Check if response contains required keywords and lacks forbidden ones."""
    text = " ".join(lines)
    passed = True
    details = []

    for kw in must_have:
        if kw not in text:
            passed = False
            details.append(f"MISSING: '{kw}'")

    for kw in must_not_have:
        if kw in text:
            passed = False
            details.append(f"UNEXPECTED: '{kw}'")

    return passed, details

def main():
    print("=" * 70)
    print("V1 Baseline Regression Test (1.1) — 12V_STANDARD")
    print(f"Port: {PORT} @ {BAUD}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.5)  # let boot messages finish
    ser.reset_input_buffer()

    results = []

    # ── 1. FW_INFO ──
    print("\n── 1. SYS:FW_INFO? ──")
    lines, tries = retry_send(ser, "SYS:FW_INFO?")
    passed, details = check_response("FW_INFO", lines,
        ["FW_INFO", "baseline=12V_STANDARD", "version=1"],
        ["AppFault"])
    for l in lines: print(f"   {l}")
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}] tries={tries}")
    if details:
        for d in details: print(f"   !!! {d}")
    results.append(("SYS:FW_INFO?", status, lines, details))

    # ── 2. CLEAR_FAULT (no text response expected) ──
    print("\n── 2. SYS:CLEAR_FAULT ──")
    ser.reset_input_buffer()
    ser.write(b"SYS:CLEAR_FAULT\n")
    time.sleep(0.40)
    # After CLEAR_FAULT, check state via next telemetry line
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    # Look for N frames — state should NOT be 5 (FAULT) if clear succeeded
    n_lines = [l for l in text.split("\n") if l.startswith("N,")]
    state_ok = True
    for nl in n_lines[:3]:
        parts = nl.split(",")
        if len(parts) > 1:
            state_val = parts[1].strip()
            print(f"   State after CLEAR: {state_val} (5=FAULT)")
            if state_val == "5":
                state_ok = False

    # CLEAR_FAULT is designed to be silent — PASS if no new fault appears
    passed = state_ok  # state not stuck at 5
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}] (silent operation — checking state transition)")
    results.append(("SYS:CLEAR_FAULT", status, n_lines,
                    [] if passed else ["State stuck at FAULT(5)"]))

    # ── 3. MOTION_CFG ──
    print("\n── 3. MOTION:MOTION_CFG? ──")
    lines, tries = retry_send(ser, "MOTION:MOTION_CFG?")
    passed, details = check_response("MOTION_CFG", lines,
        ["MOTION_CFG", "speed=", "accel=", "cruise="],
        [])
    for l in lines: print(f"   {l}")
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}]")
    if details:
        for d in details: print(f"   !!! {d}")
    results.append(("MOTION:MOTION_CFG?", status, lines, details))

    # ── 4. COG ──
    print("\n── 4. FF:COG? ──")
    lines, tries = retry_send(ser, "FF:COG?", wait=0.40, max_retries=5)
    passed, details = check_response("FF:COG?", lines,
        ["COG_CFG", "gain=", "phase_deg="],
        [])
    for l in lines: print(f"   {l}")
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}] tries={tries}")
    if details:
        for d in details: print(f"   !!! {d}")
    # Parse gain and phase values
    for l in lines:
        if "gain=" in l:
            print(f"   >>> Parsed: {l}")
    results.append(("FF:COG?", status, lines, details))

    # ── 5. RS_MODE ──
    print("\n── 5. FF:RS_MODE? ──")
    lines, tries = retry_send(ser, "FF:RS_MODE?")
    passed, details = check_response("FF:RS_MODE", lines,
        ["RS_FF_MODE", "mode="],
        [])
    for l in lines: print(f"   {l}")
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}]")
    if details:
        for d in details: print(f"   !!! {d}")
    results.append(("FF:RS_MODE?", status, lines, details))

    # ── 6. BEMF ──
    print("\n── 6. FF:BEMF? ──")
    lines, tries = retry_send(ser, "FF:BEMF?", wait=0.40, max_retries=5)
    passed, details = check_response("FF:BEMF?", lines,
        ["BEMF_CFG", "user=", "hw="],
        [])
    for l in lines: print(f"   {l}")
    status = "PASS" if passed else "FAIL"
    print(f"   [{status}] tries={tries}")
    if details:
        for d in details: print(f"   !!! {d}")
    results.append(("FF:BEMF?", status, lines, details))

    ser.close()

    # ── Summary ──
    passed = sum(1 for r in results if r[1] == "PASS")
    failed = sum(1 for r in results if r[1] == "FAIL")

    print("\n" + "=" * 70)
    print(f"BASELINE REGRESSION: {passed}/{len(results)} PASS, {failed}/{len(results)} FAIL")
    print("=" * 70)

    if failed > 0:
        print("\nFAILED:")
        for r in results:
            if r[1] == "FAIL":
                print(f"  - {r[0]}: {', '.join(r[3])}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
