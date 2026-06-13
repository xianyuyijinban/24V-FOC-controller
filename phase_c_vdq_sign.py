"""VDQ torque sign test: inject pure Q-axis voltage, observe encoder direction."""
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
    """Read frames for duration seconds, return list of N-frame snapshots."""
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
        return
    angles = [s['angle'] for s in snaps]
    iqs = [s['Iq'] for s in snaps]
    speeds = [s['speed'] for s in snaps]
    faults = set(s['faultFlags'] for s in snaps)
    print(f"  {label}:", flush=True)
    print(f"    angle: {angles[0]:.1f}° → {angles[-1]:.1f}° (Δ={angles[-1]-angles[0]:.1f}°)", flush=True)
    print(f"    Iq:    {min(iqs):.3f} ~ {max(iqs):.3f} A", flush=True)
    print(f"    speed: {min(speeds):.1f} ~ {max(speeds):.1f} rad/s", flush=True)
    print(f"    fault: {faults}", flush=True)

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Setup
    print("=== Setup ===", flush=True)
    send_cmd(ser, "CMD:CLEAR_FAULT")
    send_cmd(ser, "CMD:UNLOCK,1")

    # Baseline: read initial angle
    print("\n=== Baseline (500ms) ===", flush=True)
    baseline = read_frames(ser, 0.5)
    summarize(baseline, "Baseline")
    if baseline:
        start_angle = baseline[0]['angle']
    else:
        print("ERROR: No telemetry!", flush=True)
        ser.close()
        return

    # Test 1: Positive Q-axis voltage
    print("\n=== Test +Vq: VDQ_TEST,0.0,1.5,500 ===", flush=True)
    send_cmd(ser, "CMD:VDQ_TEST,0.0,1.5,500", wait=0.05)
    pos_test = read_frames(ser, 1.5)  # capture during + after pulse
    summarize(pos_test, "+Vq (1.5V, 500ms)")

    # Wait for motor to settle
    time.sleep(1.0)

    # Test 2: Negative Q-axis voltage
    print("\n=== Test -Vq: VDQ_TEST,0.0,-1.5,500 ===", flush=True)
    send_cmd(ser, "CMD:VDQ_TEST,0.0,-1.5,500", wait=0.05)
    neg_test = read_frames(ser, 1.5)
    summarize(neg_test, "-Vq (-1.5V, 500ms)")

    # Analysis
    print(f"\n{'='*60}", flush=True)
    print("Torque Sign Analysis:", flush=True)
    print(f"{'='*60}", flush=True)

    if pos_test and neg_test:
        pos_delta = pos_test[-1]['angle'] - pos_test[0]['angle']
        neg_delta = neg_test[-1]['angle'] - neg_test[0]['angle']

        # Handle angle wrap-around
        if pos_delta > 180: pos_delta -= 360
        if pos_delta < -180: pos_delta += 360
        if neg_delta > 180: neg_delta -= 360
        if neg_delta < -180: neg_delta += 360

        print(f"  +Vq → Δangle = {pos_delta:+.1f}°", flush=True)
        print(f"  -Vq → Δangle = {neg_delta:+.1f}°", flush=True)

        if abs(pos_delta) < 0.5:
            print(f"  ⚠️ +Vq barely moved — motor may be cogged or voltage too low", flush=True)
        if abs(neg_delta) < 0.5:
            print(f"  ⚠️ -Vq barely moved", flush=True)

        # Expected: +Vq should produce positive electrical torque
        # With enc_dir=-1, positive electrical = negative mechanical
        # So +Vq → negative mechanical angle change
        pos_sign = 1 if pos_delta > 0 else -1 if pos_delta < 0 else 0
        neg_sign = 1 if neg_delta > 0 else -1 if neg_delta < 0 else 0

        if pos_sign != 0 and neg_sign != 0:
            if pos_sign == -neg_sign:
                print(f"  ✅ Signs are opposite — torque polarity is consistent", flush=True)
                print(f"  📐 +Vq → {'POS' if pos_sign>0 else 'NEG'} mechanical direction", flush=True)
                print(f"  📐 With enc_dir=-1, this is {'EXPECTED' if pos_sign<0 else 'INVERTED ⚠️'} (pos elec → neg mech)", flush=True)
            else:
                print(f"  ⚠️ Both directions moved same way — check mechanical load", flush=True)
    else:
        print(f"  ❌ Insufficient data for analysis", flush=True)

    ser.close()

if __name__ == "__main__":
    main()
