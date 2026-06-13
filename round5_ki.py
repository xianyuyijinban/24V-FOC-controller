"""Torque test with Ki: short pulses, check Iq tracking + Vq margin."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
PORT, BAUD = "COM9", 230400

def send(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

def get_diag(ser):
    """Send CURRENT_SNAP + FAULT_DETAIL, return parsed diag lines."""
    send(ser, "CMD:CURRENT_SNAP", 0.2)
    time.sleep(0.3)
    send(ser, "CMD:FAULT_DETAIL", 3.0)
    t0 = time.time()
    buf = []
    while time.time() - t0 < 2.0:
        if ser.in_waiting:
            buf.append(ser.read(ser.in_waiting).decode('utf-8', errors='replace'))
        time.sleep(0.05)
    return ''.join(buf)

def collect_nframes(ser, duration):
    snaps = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            for line in data.decode('utf-8', errors='replace').split('\n'):
                if line.startswith('N,'):
                    parts = line.split(',')
                    if len(parts) >= 25:
                        try:
                            snaps.append({
                                'angle': float(parts[3]), 'speed': float(parts[4]),
                                'Iq': float(parts[6]), 'IqRef': float(parts[19]) if parts[19] else 0,
                            })
                        except (ValueError, IndexError): pass
        time.sleep(0.005)
    return snaps

def run_pulse(ser, iq_ref, duration=0.5):
    print(f"\n--- IqRef={iq_ref:.2f}A ({duration}s pulse) ---")
    send(ser, f"CMD:IREF,0,{iq_ref:.2f}", 0.02)
    snaps = collect_nframes(ser, duration)
    diag = get_diag(ser)
    send(ser, "CMD:IREF,0,0", 0.2)

    # Extract key fields
    for line in diag.split('\n'):
        s = line.strip()
        if any(kw in s for kw in ['CurrentDQ', 'AdcDQ', 'LoopABC']):
            print(f"  {s}")

    if snaps:
        a0, a1 = snaps[0]['angle'], snaps[-1]['angle']
        delta = a1 - a0
        if delta > 180: delta -= 360
        if delta < -180: delta += 360
        iqs = [s['Iq'] for s in snaps]
        iqrefs = [s['IqRef'] for s in snaps]
        speeds = [abs(s['speed']) for s in snaps]
        print(f"  Δangle={delta:+.1f}° | Iq={min(iqs):.3f}~{max(iqs):.3f}A "
              f"| IqRef={min(iqrefs):.3f}~{max(iqrefs):.3f}A | max|ω|={max(speeds):.1f}")
        # Verdict
        iq_avg = sum(iqs)/len(iqs)
        iq_err = abs(iq_avg - iq_ref)
        iq_range = max(iqs) - min(iqs)
        if iq_range < 0.15 and iq_err < 0.03:
            print(f"  ✅ Iq tracks (avg={iq_avg:.3f}A, err={iq_err:.3f}A, ripple={iq_range:.3f}A)")
        else:
            print(f"  ⚠️ Iq tracking: avg={iq_avg:.3f}A err={iq_err:.3f}A ripple={iq_range:.3f}A")
        if delta < -1.0:
            print(f"  ✅ Direction: NEG (correct)")
        elif delta > 1.0:
            print(f"  ✅ Direction: NEG? (Δ={delta:+.1f}°, moving)")
        else:
            print(f"  ⚠️ Minimal movement (Δ={delta:+.1f}°)")

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.2)
ser.reset_input_buffer()

# Setup
print("=== Setup: Kp=0.03 Ki=0.5 ===")
send(ser, "CMD:ENABLE,0", 0.5)
send(ser, "CMD:PI_CURRENT,0.03,0.5", 0.3)
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:MODE,0", 0.3)
send(ser, "CMD:UNLOCK,1", 0.3)
send(ser, "CMD:ENABLE,1", 0.5)
# Verify initial state is stable
time.sleep(0.3)
bl = collect_nframes(ser, 0.2)
if bl and max(abs(s['speed']) for s in bl) < 0.5:
    print("  Baseline stable ✅")
else:
    print("  ⚠️ Baseline unstable!")

# Test pulses
for iq in [0.05, 0.10]:
    run_pulse(ser, iq, 0.5)
    time.sleep(0.5)  # settle

send(ser, "CMD:ENABLE,0", 0.3)
ser.close()
