"""
V1 Release Test — 1.2 Control Mode Regression (RAW + SPEED / POSITION / STOP)
PREF uses RADIANS internally. SREF uses rad/s (mechanical).
"""
import serial, time, math, sys
from datetime import datetime

PORT = "COM9"
BAUD = 1152000

F_IDX = {
    "type": 0, "timestamp": 1, "state": 2, "angle": 3, "speed": 4,
    "Id": 5, "Iq": 6, "Vbus": 7, "faultFlags": 8,
    "appFault": 14, "ctrlMode": 15, "speed_ref": 17,
    "pos_ref": 18, "Iq_ref": 19, "Vq": 21,
}

def send_cmd(ser, cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def collect_telemetry(ser, duration_s=3.0):
    frames = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        waiting = ser.in_waiting
        if waiting:
            data = ser.read(waiting)
            for line in data.decode("utf-8", errors="replace").split("\n"):
                line = line.strip()
                if line.startswith("N,"):
                    parts = line.split(",")
                    if len(parts) >= 25:
                        frames.append(parts)
        else:
            time.sleep(0.01)
    return frames

def circ_dist_deg(a, b):
    d = abs(a - b) % 360.0
    return d if d <= 180.0 else 360.0 - d

def analyze_speed(frames, target):
    running = [f for f in frames if f[F_IDX["state"]] == "4"]
    if not running:
        return {"error": "Not RUNNING"}

    speeds, errors, vqs = [], [], []
    faults = 0
    for f in running:
        try:
            spd = float(f[F_IDX["speed"]])
            spd_ref = float(f[F_IDX["speed_ref"]])
            speeds.append(spd)
            errors.append(abs(spd - spd_ref))
            vqs.append(float(f[F_IDX["Vq"]]))
            if f[F_IDX["appFault"]] != "0":
                faults += 1
        except (ValueError, IndexError):
            continue

    if not speeds:
        return {"error": "No speed data"}

    avg_spd = sum(speeds) / len(speeds)
    avg_err = sum(errors) / len(errors)
    max_err = max(errors)
    settle_vq = (sum(vqs[-10:]) / len(vqs[-10:]) * 1000) if vqs else 0

    if abs(target) > 0.05:
        avg_ok = abs(avg_spd - target) <= abs(target) * 0.10
    else:
        avg_ok = abs(avg_spd) <= 0.05

    return {
        "target": target, "n": len(running),
        "avg_speed": avg_spd, "avg_err": avg_err, "max_err": max_err,
        "avg_ok": avg_ok, "faults": faults, "vq_mv": settle_vq,
    }

def analyze_position(frames, target_deg):
    running = [f for f in frames if f[F_IDX["state"]] == "4"]
    if not running:
        return {"error": "Not RUNNING"}

    circ_errs = []
    faults = 0
    for f in running[-20:]:
        try:
            angle = float(f[F_IDX["angle"]])
            circ_errs.append(circ_dist_deg(angle, target_deg))
            if f[F_IDX["appFault"]] != "0":
                faults += 1
        except (ValueError, IndexError):
            continue

    if not circ_errs:
        return {"error": "No pos data"}

    return {
        "target_deg": target_deg, "n": len(running),
        "avg_err_deg": sum(circ_errs) / len(circ_errs),
        "max_err_deg": max(circ_errs),
        "faults": faults,
    }

def check_stop(ser):
    send_cmd(ser, "CMD:STOP", wait=0.5)
    frames = collect_telemetry(ser, duration_s=1.0)
    if not frames:
        return {"error": "No telemetry after STOP"}
    states = [f[F_IDX["state"]] for f in frames]
    faults = sum(1 for f in frames if f[F_IDX["appFault"]] != "0")
    still_running = any(s == "4" for s in states)
    last = int(states[-1]) if states else -1
    smap = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}
    return {"still_running": still_running, "last_state": smap.get(last, str(last)), "faults": faults, "has_data": True}

def main():
    print("=" * 70)
    print("V1 Control Mode Regression (1.2) — RAW + SPEED / POSITION / STOP")
    print(f"PREF unit: RADIANS | SREF unit: rad/s | {datetime.now():%Y-%m-%d %H:%M:%S}")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    results = []

    # Setup
    print("\n--- Setup ---")
    send_cmd(ser, "CMD:UNLOCK,1")
    send_cmd(ser, "CMD:APP_MODE,RAW")
    send_cmd(ser, "CMD:ENABLE,1")
    send_cmd(ser, "TELEM:RATE,50")
    time.sleep(0.2)
    ser.reset_input_buffer()

    # ═══ A. RAW + SPEED ═══
    print("\n" + "=" * 70)
    print("A. RAW + SPEED MODE (SREF in rad/s)")
    print("=" * 70)

    send_cmd(ser, "CMD:MODE,1")

    for label, target in [("+0.5",0.5), ("-0.5",-0.5), ("+1.0",1.0), ("-1.0",-1.0), ("0",0.0)]:
        print(f"\n  [SREF={label}]")
        send_cmd(ser, f"CMD:SREF,{target:.3f}")
        time.sleep(0.5)
        a = analyze_speed(collect_telemetry(ser, 2.5), target)
        if a and "error" not in a:
            ok = a["avg_ok"] and a["faults"] == 0
            status = "PASS" if ok else "FAIL"
            print(f"    avg={a['avg_speed']:.4f} err(avg/max)={a['avg_err']:.4f}/{a['max_err']:.4f} Vq={a['vq_mv']:.0f}mV faults={a['faults']}")
            print(f"    [{status}]")
            results.append((f"SREF={label}", status, a))
        else:
            print(f"    FAIL: {a.get('error','?') if a else '?'}")
            results.append((f"SREF={label}", "FAIL", a or {}))

    # RTZ Vq
    send_cmd(ser, "CMD:SREF,0.000")
    time.sleep(1.0)
    a0 = analyze_speed(collect_telemetry(ser, 1.0), 0.0)
    if a0 and "error" not in a0:
        print(f"\n  RTZ Vq: {a0['vq_mv']:.0f} mV (target <30mV)")
        ok = abs(a0["vq_mv"]) < 30
        print(f"  [{'PASS' if ok else 'FAIL'}]")
        results.append(("RTZ Vq", "PASS" if ok else "FAIL", a0))

    # ═══ B. RAW + POSITION ═══
    print("\n" + "=" * 70)
    print("B. RAW + POSITION MODE (PREF in RADIANS)")
    print("=" * 70)

    send_cmd(ser, "CMD:MODE,2")

    # PREF targets: (label, rad, deg)
    pos_targets = [
        ("0°",    0.0,       0),
        ("+5°",   0.087266,  5),
        ("-5°",  -0.087266,  355),  # normalized
        ("+20°",  0.349066,  20),
        ("-20°", -0.349066,  340),  # normalized
        ("0°",    0.0,       0),
    ]

    for label, rad_val, deg_target in pos_targets:
        print(f"\n  [PREF={label} ({rad_val:.6f} rad)]")
        send_cmd(ser, f"CMD:PREF,{rad_val:.6f}")
        time.sleep(1.0)
        a = analyze_position(collect_telemetry(ser, 2.0), deg_target)
        if a and "error" not in a:
            ok = a["max_err_deg"] < 3.0 and a["faults"] == 0
            status = "PASS" if ok else "FAIL"
            print(f"    pos err: avg={a['avg_err_deg']:.2f}° max={a['max_err_deg']:.2f}° faults={a['faults']}")
            print(f"    [{status}]")
            results.append((f"PREF={label}", status, a))
        else:
            print(f"    FAIL: {a.get('error','?') if a else '?'}")
            results.append((f"PREF={label}", "FAIL", a or {}))

    # ═══ C. STOP ═══
    print("\n" + "=" * 70)
    print("C. STOP VERIFICATION")
    print("=" * 70)

    # STOP from position
    print("\n  [STOP from POSITION]")
    send_cmd(ser, "CMD:PREF,0.174533")  # 10° in rad
    time.sleep(0.5)
    r = check_stop(ser)
    ok = not r.get("still_running", True) and r.get("faults", 99) == 0
    print(f"    Running={r.get('still_running')} State={r.get('last_state')} Faults={r.get('faults')}")
    print(f"    [{'PASS' if ok else 'FAIL'}]")
    results.append(("STOP (pos)", "PASS" if ok else "FAIL", r))

    # STOP from speed
    send_cmd(ser, "CMD:ENABLE,1")
    time.sleep(0.3)
    send_cmd(ser, "CMD:MODE,1")
    send_cmd(ser, "CMD:SREF,0.5")
    time.sleep(0.5)

    print("\n  [STOP from SPEED]")
    r = check_stop(ser)
    ok = not r.get("still_running", True) and r.get("faults", 99) == 0
    print(f"    Running={r.get('still_running')} State={r.get('last_state')} Faults={r.get('faults')}")
    print(f"    [{'PASS' if ok else 'FAIL'}]")
    results.append(("STOP (spd)", "PASS" if ok else "FAIL", r))

    # Teardown
    send_cmd(ser, "CMD:UNLOCK,0")
    send_cmd(ser, "TELEM:RATE,10")
    ser.close()

    # Summary
    print("\n" + "=" * 70)
    passed = sum(1 for r in results if r[1] == "PASS")
    failed = sum(1 for r in results if r[1] == "FAIL")
    print(f"CONTROL REGRESSION: {passed}/{len(results)} PASS, {failed}/{len(results)} FAIL")
    print("=" * 70)
    if failed:
        print("FAILED:")
        for r in results:
            if r[1] == "FAIL":
                print(f"  - {r[0]}")
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
