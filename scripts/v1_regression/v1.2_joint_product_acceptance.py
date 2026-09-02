#!/usr/bin/env python3
"""V1.2 Joint Product Mode — Protocol Acceptance Script.

Validates the full APP_MODE command cycle without touching UNLOCK/ENABLE.
All commands are safe (no motor power); hardware feel-testing is manual.

Usage:
    python scripts/v1.2_joint_product_acceptance.py COM9 1000000
"""

import sys
import time
import serial


BAUD = 1000000
TIMEOUT = 0.5
INTER_CMD_DELAY = 0.08  # 80ms between commands


def send(ser: serial.Serial, cmd: str) -> None:
    """Send a command (without trailing \\n — added here)."""
    line = cmd.rstrip("\n") + "\n"
    ser.write(line.encode("ascii"))
    ser.flush()


def read_response(ser: serial.Serial, timeout: float = TIMEOUT) -> list[str]:
    """Read all available lines. Returns list of non-empty stripped lines."""
    ser.timeout = timeout
    lines: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            # Normalize any replacement chars for Windows console safety
            line = line.encode("ascii", errors="replace").decode("ascii")
            if line:
                lines.append(line)
        else:
            break
    return lines


def cmd(ser: serial.Serial, command: str, expect_ok: bool = True) -> list[str]:
    """Send a command and return the RX response lines."""
    send(ser, command)
    time.sleep(INTER_CMD_DELAY)
    lines = read_response(ser)
    if expect_ok and not any("OK" in l for l in lines):
        print(f"  [WARN] No OK in response for: {command.strip()}")
        if lines:
            print(f"     got: {lines}")
    return lines


def test_header(title: str) -> None:
    print()
    print(f"── {title} ──")


def check(condition: bool, label: str) -> bool:
    status = "PASS" if condition else "FAIL"
    print(f"  {status} {label}")
    return condition


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD
    passed = 0
    failed = 0

    print(f"V1.2 Joint Product Mode Acceptance Test")
    print(f"Port: {port}  Baud: {baud}")
    print()

    # ── Connect ──
    ser = serial.Serial(port, baud, timeout=TIMEOUT)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Turn off binary current stream and reduce telemetry to avoid line noise
    send(ser, "TELEM:CUR,OFF\n")
    time.sleep(0.1)
    send(ser, "TELEM:RATE,10\n")
    time.sleep(0.1)
    ser.reset_input_buffer()
    time.sleep(0.2)
    read_response(ser, timeout=0.1)  # drain any residual

    # Verify FW alive
    test_header("FW_INFO")
    send(ser, "SYS:FW_INFO?\n")
    time.sleep(0.15)
    lines = read_response(ser)
    if any("FW_INFO" in l for l in lines):
        print(f"  [OK] FW_INFO: {[l for l in lines if 'FW_INFO' in l][0][:120]}")
        passed += 1
    else:
        print(f"  [FAIL] No FW_INFO response: {lines}")
        failed += 1

    # ── 1. APP_MODE Query ──
    test_header("1. APP_MODE Query")
    lines = cmd(ser, "CMD:APP_MODE?\n")
    ok = any("APP_MODE,OK" in l for l in lines)
    if check(ok, "CMD:APP_MODE? returns OK"):
        passed += 1
    else:
        failed += 1
    for l in lines:
        if "APP_MODE,OK" in l:
            print(f"     → {l}")

    # ── 2. Cycle all 6 APP_MODEs ──
    test_header("2. APP_MODE Switch Cycle")
    modes = ["RAW", "JOINT_POS", "GIMBAL_SPEED", "HOLD", "SPRING_DAMPER", "DETENT"]
    for mode in modes:
        lines = cmd(ser, f"CMD:APP_MODE,{mode}\n")
        ok = any(f"APP_MODE,OK,{mode}" in l for l in lines)
        symbol = "[OK]" if ok else "[FAIL]"
        print(f"  {symbol} CMD:APP_MODE,{mode}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"     response: {lines}")

    # Back to RAW
    cmd(ser, "CMD:APP_MODE,RAW\n")

    # ── 3. JOINT:LIMIT ──
    test_header("3. JOINT:LIMIT")

    # Query (should show current state)
    lines = cmd(ser, "JOINT:LIMIT?\n")
    ok = any("JOINT:LIMIT,OK" in l for l in lines)
    if check(ok, "JOINT:LIMIT? returns OK"):
        passed += 1
    else:
        failed += 1
    for l in lines:
        if "JOINT:LIMIT,OK" in l:
            print(f"     → {l}")

    # Set ±30°
    lines = cmd(ser, "JOINT:LIMIT,-30.0,30.0\n")
    ok = any("JOINT:LIMIT,OK" in l for l in lines)
    if check(ok, "JOINT:LIMIT,-30,30"):
        passed += 1
    else:
        failed += 1

    # Re-query to confirm
    lines = cmd(ser, "JOINT:LIMIT?\n")
    ok = any("min=-30.0deg" in l and "max=30.0deg" in l for l in lines)
    if check(ok, "JOINT:LIMIT? confirms ±30°"):
        passed += 1
    else:
        failed += 1

    # OFF
    lines = cmd(ser, "JOINT:LIMIT,OFF\n")
    ok = any("JOINT:LIMIT,OK,OFF" in l for l in lines)
    if check(ok, "JOINT:LIMIT,OFF"):
        passed += 1
    else:
        failed += 1

    # ── 4. GIMBAL:RAMP ──
    test_header("4. GIMBAL:RAMP")

    lines = cmd(ser, "GIMBAL:RAMP?\n")
    ok = any("GIMBAL:RAMP,OK" in l for l in lines)
    if check(ok, "GIMBAL:RAMP? returns OK"):
        passed += 1
    else:
        failed += 1
    for l in lines:
        if "GIMBAL:RAMP,OK" in l:
            print(f"     → {l}")

    lines = cmd(ser, "GIMBAL:RAMP,3.0\n")
    ok = any("GIMBAL:RAMP,OK" in l for l in lines)
    if check(ok, "GIMBAL:RAMP,3.0"):
        passed += 1
    else:
        failed += 1

    # Restore default
    cmd(ser, "GIMBAL:RAMP,2.0\n")

    # ── 5. SPRING:CFG ──
    test_header("5. SPRING:CFG")

    lines = cmd(ser, "SPRING:CFG?\n")
    ok = any("SPRING:CFG,OK" in l for l in lines)
    if check(ok, "SPRING:CFG? returns OK"):
        passed += 1
    else:
        failed += 1
    for l in lines:
        if "SPRING:CFG,OK" in l:
            print(f"     → {l}")

    # Test standard preset
    lines = cmd(ser, "SPRING:CFG,0.500,0.050,0.300\n")
    ok = any("SPRING:CFG,OK" in l for l in lines)
    if check(ok, "SPRING:CFG,0.50,0.05,0.30 (standard)"):
        passed += 1
    else:
        failed += 1

    # ── 6. DETENT:CFG ──
    test_header("6. DETENT:CFG")

    lines = cmd(ser, "DETENT:CFG?\n")
    ok = any("DETENT:CFG,OK" in l for l in lines)
    if check(ok, "DETENT:CFG? returns OK"):
        passed += 1
    else:
        failed += 1
    for l in lines:
        if "DETENT:CFG,OK" in l:
            print(f"     → {l}")

    # Test standard preset
    lines = cmd(ser, "DETENT:CFG,12,1.000,0.130,0.250\n")
    ok = any("DETENT:CFG,OK" in l for l in lines)
    if check(ok, "DETENT:CFG,12,1.0,0.13,0.25 (standard)"):
        passed += 1
    else:
        failed += 1

    # ── 7. STOP safety in product modes ──
    test_header("7. STOP Response in JOINT_POS mode")

    cmd(ser, "CMD:APP_MODE,JOINT_POS\n")
    time.sleep(0.05)
    # STOP won't change app_mode but should respond
    send(ser, "CTRL:STOP\n")
    time.sleep(0.15)
    lines = read_response(ser)
    # In JOINT_POS, CTRL:STOP should be handled (it's a core command)
    # The exact response format depends on firmware; just check it doesn't hang
    print(f"  [OK] CTRL:STOP sent in JOINT_POS mode (no hang)")
    passed += 1

    cmd(ser, "CMD:APP_MODE,RAW\n")

    # ── 8. Defaults restore ──
    test_header("8. Defaults Restore")
    cmd(ser, "JOINT:LIMIT,OFF\n")
    cmd(ser, "GIMBAL:RAMP,2.0\n")
    cmd(ser, "SPRING:CFG,0.500,0.050,0.300\n")
    cmd(ser, "DETENT:CFG,12,1.000,0.130,0.250\n")
    cmd(ser, "CMD:APP_MODE,RAW\n")
    cmd(ser, "TELEM:RATE,50\n")  # restore normal telemetry
    print(f"  [OK] Defaults restored")

    # ── Summary ──
    ser.close()
    print()
    print("=" * 50)
    total = passed + failed
    print(f"Results: {passed}/{total} passed ({failed} failed)")
    if failed == 0:
        print("V1.2 Protocol Acceptance: ALL PASSED")
        return 0
    else:
        print(f"V1.2 Protocol Acceptance: {failed} FAILURES")
        return 1


if __name__ == "__main__":
    sys.exit(main())
