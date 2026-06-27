"""PREF unit verification — radians vs degrees."""
import serial, time, math, sys

PORT = "COM9"
BAUD = 230400

def query(ser, cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    raw = ser.read(ser.in_waiting)
    text = raw.decode("utf-8", errors="replace")
    return [l.strip() for l in text.split("\n")
            if l.strip() and not (l.startswith("N,") or l.startswith("C,"))]

def get_state(ser):
    ser.reset_input_buffer()
    ser.write(b"SYS:FW_INFO?\n")
    time.sleep(0.3)
    raw = ser.read(ser.in_waiting)
    for l in raw.decode("utf-8", errors="replace").split("\n"):
        if l.strip().startswith("N,"):
            parts = l.strip().split(",")
            if len(parts) >= 25:
                state_map = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}
                st = parts[2]
                return {
                    "state": f"{st} ({state_map.get(int(st) if st.isdigit() else -1, '?')})",
                    "angle": parts[3],
                    "pos_ref": parts[18],
                    "speed": parts[4],
                    "appFault": parts[14],
                    "Vq": parts[21] if len(parts) > 21 else "?",
                }
    return None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    # Setup
    query(ser, "CMD:UNLOCK,1")
    query(ser, "CMD:APP_MODE,RAW")
    query(ser, "CMD:ENABLE,1")
    query(ser, "TELEM:RATE,10")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Position mode
    query(ser, "CMD:MODE,2")
    time.sleep(0.2)

    # Test: send PREF in RADIANS (5° = 0.087266 rad, 20° = 0.349066 rad)
    tests = [
        ("PREF=0 rad (0°)",     0.0),
        ("PREF=0.087266 (+5°)",  0.087266),
        ("PREF=-0.087266 (-5°)", -0.087266),
        ("PREF=0.349066 (+20°)", 0.349066),
        ("PREF=-0.349066 (-20°)",-0.349066),
        ("PREF=0 (return)",      0.0),
    ]

    print("PREF Unit Verification")
    print("=" * 70)
    print(f"{'Test':<25} {'State':<15} {'Angle':>8} {'PosRef':>8} {'Speed':>8} {'Vq':>8} {'Fault'}")
    print("-" * 70)

    for label, target_rad in tests:
        cmd = f"CMD:PREF,{target_rad:.6f}"
        query(ser, cmd)
        time.sleep(1.5)  # let position settle

        # Collect a few frames and average
        ser.reset_input_buffer()
        time.sleep(0.4)
        raw = ser.read(ser.in_waiting)
        angles = []
        for l in raw.decode("utf-8", errors="replace").split("\n"):
            if l.strip().startswith("N,"):
                parts = l.strip().split(",")
                if len(parts) >= 25:
                    try:
                        angles.append(float(parts[3]))
                    except ValueError:
                        pass

        st = get_state(ser)
        if st:
            expected_deg = target_rad * 180.0 / math.pi
            angle = float(st["angle"]) if st["angle"].replace(".","").replace("-","").isdigit() else 0
            print(f"{label:<25} {st['state']:<15} {angle:>8.2f} {st['pos_ref']:>8} {st['speed']:>8} {st['Vq']:>8} {st['appFault']}")

        # Also check FAULT_DETAIL for PrefDiag info
        if target_rad != 0:
            lines = query(ser, "DIAG:FAULT_DETAIL", wait=0.5)
            for l in lines:
                if "PrefDiag" in l or "PositionLoop" in l or "Theta" in l:
                    print(f"  DIAG: {l[:200]}")

    # Teardown
    query(ser, "CMD:STOP")
    query(ser, "CMD:UNLOCK,0")
    ser.close()

if __name__ == "__main__":
    main()
