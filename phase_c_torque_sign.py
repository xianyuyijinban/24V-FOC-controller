"""Torque sign test: use CMD:IREF in torque mode, observe encoder direction."""
import serial
import time
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

def send_cmd(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

def read_frames(ser, duration):
    snapshots = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            for line in text.split('\n'):
                if line.startswith('N,'):
                    parts = line.split(',')
                    if len(parts) >= 25:
                        try:
                            snap = {
                                'tick': int(parts[1]),
                                'focState': int(parts[2]),
                                'angle': float(parts[3]),
                                'speed': float(parts[4]),
                                'Id': float(parts[5]),
                                'Iq': float(parts[6]),
                                'Vbus': float(parts[7]),
                                'faultFlags': parts[8],
                                'controlMode': int(parts[15]) if parts[15] else 0,
                                'IqRef': float(parts[19]) if parts[19] else 0,
                                'Ia': float(parts[22]) if parts[22] else 0,
                                'Ib': float(parts[23]) if parts[23] else 0,
                                'Ic': float(parts[24]) if parts[24] else 0,
                            }
                            snapshots.append(snap)
                        except (ValueError, IndexError):
                            pass
        time.sleep(0.01)
    return snapshots

def summarize(snaps, label):
    if not snaps:
        print(f"  {label}: NO DATA", flush=True)
        return 0
    angles = [s['angle'] for s in snaps]
    iqs = [s['Iq'] for s in snaps]
    iq_refs = [s['IqRef'] for s in snaps]
    speeds = [s['speed'] for s in snaps]
    faults = set(s['faultFlags'] for s in snaps)
    delta = angles[-1] - angles[0]
    if delta > 180: delta -= 360
    if delta < -180: delta += 360
    print(f"  {label}:", flush=True)
    print(f"    angle: {angles[0]:.1f}° → {angles[-1]:.1f}° (Δ={delta:+.1f}°)", flush=True)
    print(f"    Iq: {min(iqs):.3f}~{max(iqs):.3f} A | IqRef: {min(iq_refs):.3f}~{max(iq_refs):.3f} A", flush=True)
    print(f"    speed: {min(speeds):.1f}~{max(speeds):.1f} rad/s", flush=True)
    print(f"    fault: {faults}", flush=True)
    # Check for runaway
    if max(abs(s) for s in speeds) > 5.0:
        print(f"    ⚠️ RUNAWAY DETECTED (speed > 5 rad/s)!", flush=True)
        return -1
    return delta

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("=== Setup: Torque mode ===", flush=True)
    send_cmd(ser, "CMD:CLEAR_FAULT")
    send_cmd(ser, "CMD:UNLOCK,1")
    send_cmd(ser, "CMD:MODE,0")           # Torque mode
    send_cmd(ser, "CMD:IREF,0.0,0.0")     # Zero torque
    send_cmd(ser, "CMD:ENABLE,1")
    time.sleep(0.5)

    # Baseline
    print("\n=== Baseline (300ms) ===", flush=True)
    bl = read_frames(ser, 0.3)
    summarize(bl, "Baseline")

    # Test +Iq
    print("\n=== Test +Iq: IREF,0.0,0.15 (300ms) ===", flush=True)
    send_cmd(ser, "CMD:IREF,0.0,0.15", wait=0.05)
    pos_test = read_frames(ser, 0.6)
    pos_delta = summarize(pos_test, "+IqRef=0.15")

    # Zero
    send_cmd(ser, "CMD:IREF,0.0,0.0")
    time.sleep(0.5)

    # Test -Iq
    if pos_delta != -1:  # no runaway
        print("\n=== Test -Iq: IREF,0.0,-0.15 (300ms) ===", flush=True)
        send_cmd(ser, "CMD:IREF,0.0,-0.15", wait=0.05)
        neg_test = read_frames(ser, 0.6)
        neg_delta = summarize(neg_test, "-IqRef=0.15")

        # Zero and disable
        send_cmd(ser, "CMD:IREF,0.0,0.0")
        send_cmd(ser, "CMD:ENABLE,0")
    else:
        send_cmd(ser, "CMD:ENABLE,0")
        neg_delta = 0
        print("\n  ⛔ Skipped -Iq test due to runaway", flush=True)

    # Analysis
    print(f"\n{'='*60}", flush=True)
    print("Torque Sign Analysis:", flush=True)
    print(f"{'='*60}", flush=True)

    if pos_delta != -1:
        p_sign = 1 if pos_delta > 1.0 else -1 if pos_delta < -1.0 else 0
        n_sign = 1 if neg_delta > 1.0 else -1 if neg_delta < -1.0 else 0
        print(f"  +IqRef=0.15 → Δangle = {pos_delta:+.1f}° ({'POS' if p_sign>0 else 'NEG' if p_sign<0 else 'NONE'})", flush=True)
        print(f"  -IqRef=0.15 → Δangle = {neg_delta:+.1f}° ({'POS' if n_sign>0 else 'NEG' if n_sign<0 else 'NONE'})", flush=True)

        if p_sign != 0 and n_sign != 0:
            if p_sign == -n_sign:
                print(f"  ✅ Signs opposite — torque polarity is consistent", flush=True)
            else:
                print(f"  ⚠️ Both same sign — check mechanical load", flush=True)

        # enc_dir=-1: positive electrical → negative mechanical
        # So +Iq should → negative mechanical angle
        if p_sign < 0:
            print(f"  ✅ +Iq → negative mechanical — MATCHES enc_dir=-1 expectation", flush=True)
        elif p_sign > 0:
            print(f"  ⚠️ +Iq → positive mechanical — INVERTED vs enc_dir=-1 expectation", flush=True)
            print(f"  → Current/torque sign chain is INVERTED", flush=True)

    ser.close()

if __name__ == "__main__":
    main()
