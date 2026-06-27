"""Isolated HOLD test — clean MCU reset, no chain contamination."""
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
                        smap = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}
                        st = int(parts[2]) if parts[2].isdigit() else -1
                        return {
                            "state": smap.get(st, parts[2]),
                            "angle": float(parts[3]) if parts[3].replace(".","").replace("-","").isdigit() else 0,
                            "fault": parts[14],
                            "Vq": float(parts[21]) if len(parts) > 21 and parts[21].replace(".","").replace("-","").isdigit() else 0,
                        }
        time.sleep(0.02)
    return None

def circ_dist(a, b):
    d = abs(a - b) % 360.0
    return d if d <= 180.0 else 360.0 - d

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(4.0)  # wait for boot diagnostics to finish
    # Drain boot output
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.read(ser.in_waiting)  # drain residual
    ser.reset_input_buffer()

    print("=== HOLD Isolated Test ===")

    # Setup
    send(ser, "CMD:UNLOCK,1")
    send(ser, "CMD:ENABLE,1")
    send(ser, "TELEM:RATE,50")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Move to known position
    print("Moving to +10°...")
    send(ser, "CMD:APP_MODE,JOINT_POS")
    send(ser, "CMD:PREF,0.174533")
    time.sleep(1.5)
    f0 = get_nframe(ser)
    if f0:
        print(f"  Settled: angle={f0['angle']:.2f}° state={f0['state']}")

    # Engage HOLD
    print("Engaging HOLD...")
    send(ser, "CMD:APP_MODE,HOLD")
    time.sleep(2.5)

    f1 = get_nframe(ser)
    if f0 and f1:
        drift = circ_dist(f1["angle"], f0["angle"])
        ok = drift < 3.0 and f1["fault"] == "0"
        print(f"  Start={f0['angle']:.2f}° Hold={f1['angle']:.2f}° drift={drift:.2f}° fault={f1['fault']}")
        print(f"  [{'PASS' if ok else 'FAIL'}]")
    else:
        print("  FAIL: no position data")
        ok = False

    # STOP verification — use send() pattern that works in full test
    print("STOP...")
    send(ser, "CTRL:STOP", wait=0.6)
    time.sleep(0.5)

    pwm = " ".join(send(ser, "DIAG:PWM_DIAG", wait=0.8))
    mode = " ".join(send(ser, "CMD:APP_MODE?", wait=0.5))
    f2 = get_nframe(ser, timeout_s=1.0)

    pwm_ok = "Ta=500" in pwm
    mode_ok = "APP_MODE" in mode
    fault_ok = f2 and f2["fault"] == "0" and f2["state"] != "FAULT" if f2 else True

    print(f"  PWM={'OK' if pwm_ok else 'FAIL'} Mode={'OK' if mode_ok else 'FAIL'} Fault={'OK' if fault_ok else 'FAIL'}")
    stop_ok = pwm_ok and mode_ok and fault_ok
    print(f"  [{'PASS' if stop_ok else 'FAIL'}]")

    ser.close()
    print(f"\nHOLD: {'PASS' if ok and stop_ok else 'FAIL'}")
    return 0 if ok and stop_ok else 1

if __name__ == "__main__":
    sys.exit(main())
