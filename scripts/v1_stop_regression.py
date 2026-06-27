"""
Unified STOP regression — all 6 APP_MODEs.
Each: enter → run briefly → STOP → query APP_MODE? + PWM_DIAG + FW_INFO?
Must all respond, state=READY, PWM stopped, fault=0.
"""
import serial, time, sys, math

PORT = "COM9"
BAUD = 230400

def send(ser, cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    return [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def verify_stop(ser):
    """STOP + 3 queries. Returns (ok, info_dict)."""
    send(ser, "CTRL:STOP", wait=0.5)
    time.sleep(0.5)

    # PWM_DIAG
    pwm_lines = send(ser, "DIAG:PWM_DIAG", wait=0.8)
    pwm_text = " ".join(pwm_lines)
    pwm_ok = "Ta=500,Tb=500,Tc=500" in pwm_text

    # APP_MODE?
    mode_lines = send(ser, "CMD:APP_MODE?", wait=0.5)
    mode_ok = len(mode_lines) > 0 and "APP_MODE" in " ".join(mode_lines)

    # FW_INFO? — check state via N-frame in response
    fw_lines = send(ser, "CMD:FW_INFO?", wait=0.5)
    fw_ok = len(fw_lines) > 0

    # Best-effort N frame for state
    ser.reset_input_buffer()
    time.sleep(0.5)
    raw = ser.read(ser.in_waiting)
    state = "?"
    fault = "?"
    for l in raw.decode("utf-8", errors="replace").split("\n"):
        if l.strip().startswith("N,"):
            parts = l.strip().split(",")
            if len(parts) >= 25:
                smap = {0:"IDLE", 1:"INIT", 2:"IDENTIFY", 3:"READY", 4:"RUNNING", 5:"FAULT"}
                st = int(parts[2]) if parts[2].isdigit() else -1
                state = smap.get(st, parts[2])
                fault = parts[14]
            break

    # Decisive: all 3 queries respond + state is READY/IDLE + fault=0
    all_ok = pwm_ok and mode_ok and fw_ok and state in ("READY", "IDLE") and fault == "0"
    info = f"state={state} pwm={'OK' if pwm_ok else 'FAIL'} mode={'OK' if mode_ok else 'FAIL'} fw={'OK' if fw_ok else 'FAIL'} fault={fault}"

    return all_ok, info

def arm(ser):
    """Re-arm after STOP: full unlock+enable sequence."""
    send(ser, "SYS:CLEAR_FAULT", wait=0.3)
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.3)
    ser.reset_input_buffer()

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(4.0)  # wait for boot
    ser.reset_input_buffer()
    ser.read(ser.in_waiting)
    ser.reset_input_buffer()

    results = []
    # Initial unlock
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.2)
    ser.reset_input_buffer()

    modes = [
        # (name, setup_cmds, app_mode_cmd, run_cmd)
        ("RAW",            [], "CMD:APP_MODE,RAW",            "CMD:SREF,0.3"),
        ("JOINT_POS",      [], "CMD:APP_MODE,JOINT_POS",      "CMD:PREF,0.174533"),
        ("GIMBAL_SPEED",   [], "CMD:APP_MODE,GIMBAL_SPEED",   "CMD:SREF,0.3"),
        ("HOLD",           ["CMD:APP_MODE,JOINT_POS", "CMD:PREF,0.174533"],
                               "CMD:APP_MODE,HOLD",            None),
        ("SPRING_DAMPER",  ["CMD:APP_MODE,RAW", "CMD:MODE,1", "CMD:SREF,0.3"],
                               "CMD:APP_MODE,SPRING_DAMPER",   None),
        ("DETENT",         ["CMD:APP_MODE,RAW", "CMD:MODE,1", "CMD:SREF,0.3"],
                               "CMD:APP_MODE,DETENT",          None),
    ]

    print("Unified STOP Regression (all 6 modes)")
    print("=" * 70)

    for name, setup_cmds, mode_cmd, run_cmd in modes:
        arm(ser)
        # Optional setup (e.g. enter RUNNING before switching to product mode)
        for sc in setup_cmds:
            send(ser, sc)
            time.sleep(0.3)
        send(ser, mode_cmd)
        time.sleep(0.3)
        # Brief run
        if run_cmd:
            send(ser, run_cmd)
            time.sleep(0.3)
        time.sleep(0.5)  # let mode engage

        ok, info = verify_stop(ser)
        status = "PASS" if ok else "FAIL"
        print(f"  {name:<16} STOP: [{status}] {info}")
        results.append((name, status, info))

    ser.close()

    print("=" * 70)
    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"STOP REGRESSION: {passed}/{len(results)} PASS, {failed}/{len(results)} FAIL")
    if failed:
        print("FAILED:")
        for name, _, info in results:
            if _ == "FAIL":
                print(f"  - {name}: {info}")

    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
