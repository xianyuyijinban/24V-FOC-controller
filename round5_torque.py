"""Round 5 torque sign test: P-only Kp=0.03, +IqRef=0.05A"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
PORT, BAUD = "COM9", 230400

def send(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

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
                                'Id': float(parts[5]), 'Iq': float(parts[6]),
                                'IqRef': float(parts[19]) if parts[19] else 0,
                                'faultFlags': parts[8],
                            })
                        except (ValueError, IndexError): pass
        time.sleep(0.01)
    return snaps

def show_diag(ser):
    send(ser, "CMD:FAULT_DETAIL", 3.0)
    t0 = time.time()
    buf = []
    while time.time() - t0 < 1.5:
        if ser.in_waiting:
            buf.append(ser.read(ser.in_waiting).decode('utf-8', errors='replace'))
        time.sleep(0.05)
    for line in ''.join(buf).split('\n'):
        s = line.strip()
        if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag',
                                    'CurrentDQ', 'AdcDQ', 'LoopABC', 'DirDiag']):
            print(f"  {s}")

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.2)
ser.reset_input_buffer()

# Setup
print("=== Setup ===")
send(ser, "CMD:ENABLE,0", 0.5)
send(ser, "CMD:CLEAR_FAULT", 0.3)
print("  State before torque test:")
show_diag(ser)

# Torque mode, zero current
send(ser, "CMD:MODE,0", 0.3)
send(ser, "CMD:UNLOCK,1", 0.3)
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:ENABLE,1", 0.3)
time.sleep(0.3)

# Baseline
print("\n=== Baseline (300ms) ===")
bl = collect_nframes(ser, 0.3)
if bl:
    print(f"  angle={bl[0]['angle']:.1f}° Iq={bl[0]['Iq']:.3f}A speed={bl[0]['speed']:.1f}rad/s")

# +IqRef=0.05A
print("\n=== +IqRef=0.05A (1s) ===")
send(ser, "CMD:IREF,0,0.05", 0.05)
t1 = collect_nframes(ser, 1.0)
if t1:
    a0, a1 = t1[0]['angle'], t1[-1]['angle']
    delta = a1 - a0
    if delta > 180: delta -= 360
    if delta < -180: delta += 360
    iqs = [s['Iq'] for s in t1]
    iqrefs = [s['IqRef'] for s in t1]
    speeds = [abs(s['speed']) for s in t1]
    print(f"  angle: {a0:.1f}° → {a1:.1f}° (Δ={delta:+.1f}°)")
    print(f"  Iq: {min(iqs):.3f}~{max(iqs):.3f}A  IqRef: {min(iqrefs):.3f}~{max(iqrefs):.3f}A")
    print(f"  max|speed|: {max(speeds):.1f} rad/s")

    # Diagnostics during torque
    print("\n=== CURRENT_SNAP + FAULT_DETAIL ===")
    send(ser, "CMD:CURRENT_SNAP", 0.3)
    time.sleep(0.3)
    show_diag(ser)

    # Verdict
    print(f"\n{'='*50}")
    if delta < -1.0:
        print(f"✅ +IqRef → NEG mech (Δ={delta:+.1f}°) — TORQUE SIGN CORRECT")
    elif delta > 1.0:
        print(f"❌ +IqRef → POS mech (Δ={delta:+.1f}°) — STILL INVERTED")
    else:
        print(f"⚠️  Negligible movement (Δ={delta:+.1f}°)")
    if max(speeds) < 2.0:
        print(f"✅ No runaway (max|speed|={max(speeds):.1f})")
    else:
        print(f"❌ Runaway (max|speed|={max(speeds):.1f})")
    print(f"{'='*50}")

# Zero and disable
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:ENABLE,0", 0.3)
ser.close()
