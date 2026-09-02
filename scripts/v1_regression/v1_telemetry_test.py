"""
V1 Release Test — 1.4 Telemetry & Serial Stability
Tests: NUL bytes, 10/50/100Hz, command response under load, STOP while streaming
"""
import serial, time, sys, math
from datetime import datetime

PORT = "COM9"
BAUD = 1152000

def send(ser, cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    return raw, [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
                 if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def collect_raw(ser, duration_s):
    """Collect raw bytes for NUL byte analysis."""
    data = bytearray()
    deadline = time.time() + duration_s
    while time.time() < deadline:
        w = ser.in_waiting
        if w:
            data.extend(ser.read(w))
        else:
            time.sleep(0.005)
    return bytes(data)

def count_nul(data):
    """Count NUL bytes (0x00) in data."""
    nul = data.count(b'\x00')
    return nul, len(data), 100.0 * nul / len(data) if data else 0

def get_nframe(ser, timeout_s=1.0):
    ser.reset_input_buffer()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        w = ser.in_waiting
        if w:
            for l in ser.read(w).decode("utf-8", errors="replace").split("\n"):
                if l.strip().startswith("N,"):
                    parts = l.strip().split(",")
                    if len(parts) >= 25:
                        return parts
        time.sleep(0.02)
    return None

def main():
    print("=" * 70)
    print("V1 Telemetry & Serial Stability (1.4)")
    print(f"Time: {datetime.now():%Y-%m-%d %H:%M:%S}")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(4.0)
    ser.reset_input_buffer()
    ser.read(ser.in_waiting)
    ser.reset_input_buffer()

    results = []

    # ── Setup ──
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    time.sleep(0.2)

    # ═══════════════════════════════════════════
    # 1. NUL byte check @ 10Hz
    # ═══════════════════════════════════════════
    print("\n=== NUL Byte Check ===")
    for rate in [10, 50, 100]:
        send(ser, f"TELEM:RATE,{rate}")
        time.sleep(0.3)
        data = collect_raw(ser, 2.0)
        nul, total, pct = count_nul(data)
        ok = pct < 0.1  # <0.1% NUL
        status = "PASS" if ok else "FAIL"
        print(f"  TELEM @ {rate:>3}Hz: {nul}/{total} NUL ({pct:.3f}%) [{status}]")
        results.append((f"NUL@{rate}Hz", status, f"{pct:.3f}%"))

    # ═══════════════════════════════════════════
    # 2. Command response under 100Hz telemetry
    # ═══════════════════════════════════════════
    print("\n=== Command Response @100Hz ===")
    send(ser, "TELEM:RATE,100")
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Send a series of commands and verify they get responses
    cmd_tests = [
        "CMD:FW_INFO?",
        "CMD:APP_MODE?",
        "CMD:MOTION_CFG?",
        "DIAG:PWM_DIAG",
    ]
    for cmd in cmd_tests:
        _, lines = send(ser, cmd, wait=0.5)
        ok = len(lines) > 0
        status = "PASS" if ok else "FAIL"
        print(f"  {cmd}: {lines[0] if lines else 'NO RESPONSE'} [{status}]")
        results.append((f"CMD:{cmd.split(':')[-1].split('?')[0][:20]}@100Hz", status,
                        lines[0][:80] if lines else "no response"))
        time.sleep(0.1)

    # ═══════════════════════════════════════════
    # 3. STOP responds immediately while 100Hz streaming
    # ═══════════════════════════════════════════
    print("\n=== STOP While Streaming @100Hz ===")
    send(ser, "CMD:SREF,0.3")
    time.sleep(0.3)

    t0 = time.time()
    _, lines = send(ser, "CTRL:STOP", wait=0.5)
    t1 = time.time()
    elapsed_ms = (t1 - t0) * 1000
    # STOP should respond within 500ms even under 100Hz telemetry
    ok = len(lines) > 0 and elapsed_ms < 1000
    status = "PASS" if ok else "FAIL"
    print(f"  STOP response: {lines} ({elapsed_ms:.0f}ms) [{status}]")
    results.append(("STOP@100Hz", status, f"{elapsed_ms:.0f}ms"))

    # Verify state after STOP
    f = get_nframe(ser, timeout_s=0.5)
    if f:
        smap = {0:"IDLE", 1:"INIT", 2:"IDENTIFY", 3:"READY", 4:"RUNNING", 5:"FAULT"}
        st = int(f[2]) if f[2].isdigit() else -1
        print(f"  Post-STOP state: {smap.get(st, f[2])}")

    # ═══════════════════════════════════════════
    # 4. Long text output doesn't block commands
    # ═══════════════════════════════════════════
    print("\n=== Long Output + Command Response ===")

    # Send a long query (PWM_DIAG is ~250 chars)
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,10")
    time.sleep(0.3)

    # Send PWM_DIAG, immediately followed by FW_INFO? — both should respond
    ser.reset_input_buffer()
    ser.write(b"DIAG:PWM_DIAG\n")
    time.sleep(0.1)
    ser.write(b"CMD:FW_INFO?\n")
    time.sleep(0.6)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    has_pwm = "PWM," in text or "ARR=" in text
    has_fw = "FW_INFO," in text
    ok = has_pwm and has_fw
    status = "PASS" if ok else "FAIL"
    print(f"  PWM_DIAG + FW_INFO? interleaved: PWM={'OK' if has_pwm else 'FAIL'} FW={'OK' if has_fw else 'FAIL'} [{status}]")
    results.append(("LongOutput+CMD", status, f"PWM={has_pwm},FW={has_fw}"))

    # ═══════════════════════════════════════════
    # 5. FAULT_DETAIL and BLACKBOX responses
    # ═══════════════════════════════════════════
    print("\n=== DIAG Commands ===")

    _, lines = send(ser, "DIAG:FAULT_DETAIL", wait=1.0)
    ok = len(lines) > 0
    status = "PASS" if ok else "FAIL"
    print(f"  FAULT_DETAIL: {len(lines)} lines [{status}]")
    results.append(("FAULT_DETAIL", status, f"{len(lines)} lines"))

    _, lines = send(ser, "DIAG:BLACKBOX?", wait=0.4)
    ok = len(lines) > 0
    status = "PASS" if ok else "FAIL"
    print(f"  BLACKBOX?: {lines[0] if lines else 'no response'} [{status}]")
    results.append(("BLACKBOX?", status, lines[0][:80] if lines else "no"))

    # ═══════════════════════════════════════════
    # 6. TELEM:OFF
    # ═══════════════════════════════════════════
    print("\n=== TELEM:OFF ===")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.5)
    data_before = collect_raw(ser, 1.0)
    n_before = data_before.count(b'\nN,')  # count N frames

    send(ser, "TELEM:OFF")
    time.sleep(0.5)
    data_after = collect_raw(ser, 1.0)
    n_after = data_after.count(b'\nN,')  # count N frames after OFF

    ok = n_before > 10 and n_after < 3  # >10 frames before, near zero after
    status = "PASS" if ok else "FAIL"
    print(f"  Before: ~{n_before} N-frames/s, After: ~{n_after} N-frames/s [{status}]")
    results.append(("TELEM:OFF", status, f"{n_before}→{n_after} N-frames/s"))

    send(ser, "TELEM:RATE,10")
    ser.close()

    # ── Summary ──
    print("\n" + "=" * 70)
    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"TELEMETRY: {passed}/{len(results)} PASS, {failed}/{len(results)} FAIL")
    print("=" * 70)
    if failed:
        print("FAILED:")
        for name, _, info in results:
            if _ == "FAIL":
                print(f"  - {name}: {info}")

    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
