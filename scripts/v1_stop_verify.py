"""V1 STOP verification — uses DIAG:PWM_DIAG + next N-frame state."""
import serial, time, sys

PORT = "COM9"
BAUD = 230400

def query(ser, cmd, wait=0.5):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def get_status(ser):
    """Get state + PWM status after STOP."""
    # Try PWM_DIAG first
    lines = query(ser, "DIAG:PWM_DIAG", wait=0.7)
    pwm_info = " ".join(lines[:3]) if lines else "(no response)"

    # Get state from any available N frame
    ser.reset_input_buffer()
    time.sleep(0.3)
    raw = ser.read(ser.in_waiting)
    state, sref, iqref, fault = "?", "?", "?", "?"
    for l in raw.decode("utf-8", errors="replace").split("\n"):
        if l.strip().startswith("N,"):
            parts = l.strip().split(",")
            if len(parts) >= 25:
                smap = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}
                st = int(parts[2]) if parts[2].isdigit() else -1
                state = smap.get(st, parts[2])
                sref = parts[17]
                iqref = parts[19]
                fault = parts[14]
            break

    return state, sref, iqref, fault, pwm_info

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(1.5)  # wait for boot
    ser.reset_input_buffer()

    # Setup
    print("Setup...")
    query(ser, "CMD:UNLOCK,1")
    query(ser, "CMD:APP_MODE,RAW")
    query(ser, "CMD:ENABLE,1")
    time.sleep(0.3)

    all_pass = True

    # ── STOP from SPEED mode ──
    print("\n=== STOP from SPEED mode ===")
    query(ser, "CMD:MODE,1")
    query(ser, "CMD:SREF,0.5")
    time.sleep(0.6)

    query(ser, "CTRL:STOP", wait=0.5)
    time.sleep(0.5)

    state, sref, iqref, fault, pwm = get_status(ser)
    print(f"  State={state}  speed_ref={sref}  Iq_ref={iqref}  fault={fault}")
    print(f"  PWM: {pwm[:250]}")

    # STOP indicator: Ta=Tb=Tc=500 (50% duty = PWM disabled output)
    stopped = "Ta=500,Tb=500,Tc=500" in pwm
    ok = (state in ("READY", "IDLE")) and (fault == "0") and stopped
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] (need: READY/IDLE + Ta=Tb=Tc=500 + fault=0)")
    if not ok:
        all_pass = False

    # ── STOP from POSITION mode ──
    print("\n=== STOP from POSITION mode ===")
    query(ser, "CMD:ENABLE,1")
    time.sleep(0.3)
    query(ser, "CMD:MODE,2")
    query(ser, "CMD:PREF,0.174533")  # 10° in rad
    time.sleep(0.6)

    query(ser, "CTRL:STOP", wait=0.5)
    time.sleep(0.5)

    state, sref, iqref, fault, pwm = get_status(ser)
    print(f"  State={state}  speed_ref={sref}  Iq_ref={iqref}  fault={fault}")
    print(f"  PWM: {pwm[:250]}")

    stopped = "Ta=500,Tb=500,Tc=500" in pwm
    ok = (state in ("READY", "IDLE")) and (fault == "0") and stopped
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}]")
    if not ok:
        all_pass = False

    # Don't lock — leave READY for potential follow-up tests
    ser.close()

    print(f"\nSTOP VERIFICATION: {'PASS' if all_pass else 'FAIL'}")
    return 0 if all_pass else 1

if __name__ == "__main__":
    sys.exit(main())
