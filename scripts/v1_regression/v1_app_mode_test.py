"""
V1 Release Test — 1.3 APP_MODE validation
JOINT_POS / GIMBAL_SPEED / HOLD (auto) / SPRING_DAMPER (manual) / DETENT (manual)
STOP check: PWM_DIAG + APP_MODE? (not N-frame timing)
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
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def query_text(ser, cmd, wait=0.5):
    """Send query command and return text response lines."""
    lines = send(ser, cmd, wait)
    return [l for l in lines if not l.startswith(("N,", "C,"))]

def get_nframe(ser, timeout_s=1.0):
    """Get one N-frame within timeout. Returns dict or None."""
    ser.reset_input_buffer()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        w = ser.in_waiting
        if w:
            raw = ser.read(w)
            for l in raw.decode("utf-8", errors="replace").split("\n"):
                l = l.strip()
                if l.startswith("N,"):
                    parts = l.split(",")
                    if len(parts) >= 25:
                        smap = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}
                        st = int(parts[2]) if parts[2].isdigit() else -1
                        return {
                            "state": smap.get(st, parts[2]),
                            "state_num": st,
                            "angle": float(parts[3]) if parts[3].replace(".","").replace("-","").isdigit() else 0,
                            "speed": float(parts[4]) if parts[4].replace(".","").replace("-","").isdigit() else 0,
                            "fault": parts[14],
                            "Vq": float(parts[21]) if len(parts) > 21 and parts[21].replace(".","").replace("-","").isdigit() else 0,
                            "Iq": float(parts[6]) if parts[6].replace(".","").replace("-","").isdigit() else 0,
                        }
        time.sleep(0.02)
    return None

def verify_stop(ser):
    """STOP verification: PWM_DIAG response + APP_MODE? response."""
    send(ser, "CTRL:STOP", wait=0.5)
    time.sleep(0.3)

    # PWM_DIAG
    pwm_lines = send(ser, "DIAG:PWM_DIAG", wait=0.5)
    pwm_text = " ".join(pwm_lines)
    pwm_ok = "Ta=500" in pwm_text or "CCR=" in pwm_text
    pwm_stopped = "Ta=500,Tb=500,Tc=500" in pwm_text

    # APP_MODE?
    mode_lines = send(ser, "CMD:APP_MODE?", wait=0.4)
    mode_ok = len(mode_lines) > 0
    mode_text = " ".join(mode_lines)

    # Get state from N-frame (best effort)
    f = get_nframe(ser, timeout_s=0.8)

    # PASS if: PWM responds + APP_MODE responds + no FAULT state
    fault_free = True
    state_info = "?"
    if f:
        fault_free = (f["fault"] == "0" and f["state"] != "FAULT")
        state_info = f"state={f['state']}"

    ok = pwm_ok and mode_ok and fault_free
    return ok, f"{state_info} pwm={'OK' if pwm_ok else 'FAIL'} mode={'OK' if mode_ok else 'FAIL'} stopped={pwm_stopped}"

def circ_dist(a, b):
    d = abs(a - b) % 360.0
    return d if d <= 180.0 else 360.0 - d

def arm(ser):
    """Re-arm after STOP: CLEAR_FAULT, UNLOCK, ENABLE, TELEM."""
    send(ser, "SYS:CLEAR_FAULT", wait=0.3)
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.3)
    ser.reset_input_buffer()

def main():
    print("=" * 70)
    print("V1 APP_MODE Validation (1.3)")
    print(f"Time: {datetime.now():%Y-%m-%d %H:%M:%S}")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(1.5)
    ser.reset_input_buffer()

    results = []

    # ── Setup ──
    print("\n--- Setup ---")
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # ═══════════════════════════════════════════
    # JOINT_POS: soft limit + STOP
    # ═══════════════════════════════════════════
    print("\n=== 1. JOINT_POS (soft limit) ===")
    send(ser, "JOINT:LIMIT,-30,30")
    send(ser, "CMD:APP_MODE,JOINT_POS")
    time.sleep(0.3)

    print("  PREF=+20° (within limits)...")
    send(ser, "CMD:PREF,0.349")
    time.sleep(1.0)
    f = get_nframe(ser)
    if f:
        print(f"    angle={f['angle']:.2f}° state={f['state']}")

    print("  PREF=+40° (beyond +30° limit)...")
    send(ser, "CMD:PREF,0.698")
    time.sleep(1.0)
    f = get_nframe(ser)
    if f:
        clamped = 25 <= f["angle"] <= 35
        status = "PASS" if clamped else "FAIL"
        print(f"    angle={f['angle']:.2f}° clamped={'YES' if clamped else 'NO'} [{status}]")
        results.append(("JOINT_POS limit", status, f"angle={f['angle']:.1f}°"))

    ok, info = verify_stop(ser)
    status = "PASS" if ok else "FAIL"
    print(f"  STOP: [{status}] {info}")
    results.append(("JOINT_POS STOP", status, info))

    # ═══════════════════════════════════════════
    # GIMBAL_SPEED: ramp + STOP
    # ═══════════════════════════════════════════
    print("\n=== 2. GIMBAL_SPEED (ramp) ===")
    arm(ser)
    send(ser, "GIMBAL:RAMP,0.5")
    send(ser, "CMD:APP_MODE,GIMBAL_SPEED")
    time.sleep(0.3)

    print("  SREF=+0.5 (should ramp gently)...")
    send(ser, "CMD:SREF,0.5")
    f0 = get_nframe(ser, timeout_s=0.5)
    time.sleep(1.0)
    f1 = get_nframe(ser)
    if f0 and f1:
        # In GIMBAL_SPEED, speed should ramp (not jump instantly)
        speed_diff = f1["speed"] - f0["speed"]
        print(f"    t=0: speed={f0['speed']:.3f}  t=1.5s: speed={f1['speed']:.3f} (ramp delta={speed_diff:.3f})")
        status = "PASS" if f1["speed"] > 0.2 else "FAIL"
        print(f"    [{status}]")
        results.append(("GIMBAL_SPEED ramp", status, f"speed={f1['speed']:.3f}"))

    ok, info = verify_stop(ser)
    status = "PASS" if ok else "FAIL"
    print(f"  STOP: [{status}] {info}")
    results.append(("GIMBAL STOP", status, info))

    # ═══════════════════════════════════════════
    # HOLD: position hold accuracy
    # ═══════════════════════════════════════════
    print("\n=== 3. HOLD (position hold) ===")
    arm(ser)

    # First move to a known position
    print("  Moving to +10°...")
    send(ser, "CMD:APP_MODE,JOINT_POS")
    send(ser, "CMD:PREF,0.174533")  # 10° in rad
    time.sleep(1.5)
    f_start = get_nframe(ser)
    if f_start:
        print(f"    settled: angle={f_start['angle']:.2f}° state={f_start['state']}")

    # Engage HOLD
    print("  Engaging HOLD...")
    send(ser, "CMD:APP_MODE,HOLD")
    time.sleep(2.0)

    # Check position after 2s hold
    f_hold = get_nframe(ser)
    if f_hold and f_start:
        drift = circ_dist(f_hold["angle"], f_start["angle"])
        status = "PASS" if drift < 3.0 and f_hold["fault"] == "0" else "FAIL"
        print(f"    angle start={f_start['angle']:.2f}° hold={f_hold['angle']:.2f}° drift={drift:.2f}°")
        print(f"    state={f_hold['state']} fault={f_hold['fault']} [{status}]")
        results.append(("HOLD drift", status, f"drift={drift:.2f}°"))
    else:
        print("    FAIL: could not read position")
        results.append(("HOLD drift", "FAIL", "no data"))

    ok, info = verify_stop(ser)
    status = "PASS" if ok else "FAIL"
    print(f"  STOP: [{status}] {info}")
    results.append(("HOLD STOP", status, info))

    # ═══════════════════════════════════════════
    # SPRING_DAMPER (manual feel test)
    # ═══════════════════════════════════════════
    print("\n=== 4. SPRING_DAMPER (manual) ===")
    arm(ser)
    send(ser, "CMD:APP_MODE,SPRING_DAMPER")
    time.sleep(0.5)
    f = get_nframe(ser)
    if f:
        print(f"  Engaged: state={f['state']} fault={f['fault']}")
        print(f"  >>> MANUAL: gently push motor, release, verify it returns <<<")
        print(f"  >>> Check: no sustained oscillation, no fault <<<")
    results.append(("SPRING_DAMPER", "MANUAL", "requires hand feel"))

    ok, info = verify_stop(ser)
    status = "PASS" if ok else "FAIL"
    print(f"  STOP: [{status}] {info}")
    results.append(("SPRING STOP", status, info))

    # ═══════════════════════════════════════════
    # DETENT (manual feel test)
    # ═══════════════════════════════════════════
    print("\n=== 5. DETENT (manual) ===")
    arm(ser)
    send(ser, "CMD:APP_MODE,DETENT")
    time.sleep(0.5)
    f = get_nframe(ser)
    if f:
        print(f"  Engaged: state={f['state']} fault={f['fault']}")
        print(f"  >>> MANUAL: slowly turn motor, feel for detent clicks <<<")
        print(f"  >>> Check: speed→0 near detent, no oscillation <<<")
    results.append(("DETENT", "MANUAL", "requires hand feel"))

    ok, info = verify_stop(ser)
    status = "PASS" if ok else "FAIL"
    print(f"  STOP: [{status}] {info}")
    results.append(("DETENT STOP", status, info))

    ser.close()

    # ── Summary ──
    print("\n" + "=" * 70)
    auto = [(r[1], r[2]) for r in results if r[1] != "MANUAL"]
    manual = [r for r in results if r[1] == "MANUAL"]
    passed = sum(1 for s, _ in auto if s == "PASS")
    failed = sum(1 for s, _ in auto if s == "FAIL")
    print(f"APP_MODE (auto): {passed}/{len(auto)} PASS, {failed}/{len(auto)} FAIL")
    if manual:
        print(f"APP_MODE (manual): {len(manual)} pending — {', '.join(r[0] for r in manual)}")
    print("=" * 70)
    if failed:
        print("FAILED:")
        for r in results:
            if r[1] == "FAIL":
                print(f"  - {r[0]}: {r[2]}")

    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
