"""Round 5: Cold boot → IDENTIFY → Torque sign test (ADC polarity fix)."""
import serial, time, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

def send(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

def snarf(ser, duration):
    """Collect all data for duration seconds."""
    buf = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buf.append(data.decode('utf-8', errors='replace'))
        time.sleep(0.01)
    return ''.join(buf)

def parse_nframes(text):
    snaps = []
    for line in text.split('\n'):
        if line.startswith('N,'):
            parts = line.split(',')
            if len(parts) >= 27:
                try:
                    snaps.append({
                        'tick': int(parts[1]), 'focState': int(parts[2]),
                        'angle': float(parts[3]), 'speed': float(parts[4]),
                        'Id': float(parts[5]), 'Iq': float(parts[6]),
                        'IqRef': float(parts[19]) if parts[19] else 0,
                        'idState': int(parts[26]) if parts[26].strip() else -1,
                        'idError': int(parts[27]) if parts[27].strip() else -1,
                        'faultFlags': parts[8],
                    })
                except (ValueError, IndexError): pass
    return snaps

ser = serial.Serial(PORT, BAUD, timeout=0.05)
ser.reset_input_buffer()
time.sleep(0.3)
ser.reset_input_buffer()

# ====== Phase 1: Cold boot check ======
print("=" * 60)
print("Phase 1: Cold Boot Check")
print("=" * 60)
send(ser, "CMD:CLEAR_FAULT", 0.5)
send(ser, "CMD:FAULT_DETAIL", 3.0)
raw = snarf(ser, 1.0)
for line in raw.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag']):
        print(f"  {s}")
identified = 'Identified: NO' in raw or 'Identified:NO' in raw
print(f"  → Identified: {'NO ✅' if identified else 'UNEXPECTED ⚠️'}")

if not identified:
    print("  ⛔ Aborting — expected Identified:NO after PARAM_VERSION bump")
    ser.close()
    sys.exit(1)

# ====== Phase 2: IDENTIFY ======
print(f"\n{'='*60}")
print("Phase 2: Full IDENTIFY")
print("=" * 60)
for cmd in ["CMD:MOTOR_PN,11", "CMD:ENCODER_DIR,-1", "CMD:UNLOCK,1"]:
    print(f"  --- {cmd} ---")
    send(ser, cmd, 0.3)

print(f"  --- CMD:IDENTIFY,1 (waiting up to 120s) ---")
send(ser, "CMD:IDENTIFY,1", 0.1)

id_complete = False
t0 = time.time()
while not id_complete and time.time() - t0 < 130:
    raw = snarf(ser, 0.2)
    snaps = parse_nframes(raw)
    for s in snaps:
        if s['idState'] == 8:
            id_complete = True
    time.sleep(0.05)

if id_complete:
    print(f"  IDENTIFY complete ✅ ({time.time()-t0:.1f}s)")
else:
    print(f"  ⚠️ IDENTIFY timeout")
    ser.close()
    sys.exit(1)

# FAULT_DETAIL after IDENTIFY
time.sleep(0.5)
send(ser, "CMD:FAULT_DETAIL", 3.0)
raw = snarf(ser, 1.0)
for line in raw.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag',
                                'DirDiag', 'ParamDiag']):
        print(f"  {s}")

# ====== Phase 3: Torque sign test ======
print(f"\n{'='*60}")
print("Phase 3: Torque Sign Test (P-only Kp=0.03, IqRef=0.05A)")
print("=" * 60)

# Disable first (IDENTIFY left it in a post-completion state)
send(ser, "CMD:ENABLE,0", 0.5)
send(ser, "CMD:CLEAR_FAULT", 0.3)
send(ser, "CMD:MODE,0", 0.3)
send(ser, "CMD:UNLOCK,1", 0.3)
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:ENABLE,1", 0.3)

# Baseline
time.sleep(0.3)
print("  Baseline (300ms):")
raw = snarf(ser, 0.3)
bl = parse_nframes(raw)
if bl:
    iqs = [s['Iq'] for s in bl]
    spd = [abs(s['speed']) for s in bl]
    print(f"    Iq={min(iqs):.3f}~{max(iqs):.3f}A, max|speed|={max(spd):.1f}rad/s")

# Apply +IqRef=0.05A
print("  +IqRef=0.05A (1s):")
send(ser, "CMD:IREF,0,0.05", 0.05)
raw = snarf(ser, 1.0)
t1 = parse_nframes(raw)
if t1:
    angles = [s['angle'] for s in t1]
    iqs = [s['Iq'] for s in t1]
    iqrefs = [s['IqRef'] for s in t1]
    speeds = [s['speed'] for s in t1]
    delta = angles[-1] - angles[0]
    if delta > 180: delta -= 360
    if delta < -180: delta += 360
    max_spd = max(abs(s) for s in speeds)
    print(f"    angle: {angles[0]:.1f}° → {angles[-1]:.1f}° (Δ={delta:+.1f}°)")
    print(f"    Iq: {min(iqs):.3f}~{max(iqs):.3f}A | IqRef: {min(iqrefs):.3f}~{max(iqrefs):.3f}A")
    print(f"    max|speed|: {max_spd:.1f} rad/s")

# CURRENT_SNAP
send(ser, "CMD:CURRENT_SNAP", 0.3)
time.sleep(0.3)
send(ser, "CMD:FAULT_DETAIL", 3.0)
raw = snarf(ser, 1.0)
for line in raw.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['CurrentDQ', 'AdcDQ', 'LoopABC', 'CURRENT_SNAP']):
        print(f"  {s}")

# Zero and disable
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:ENABLE,0", 0.3)

# ====== Verdict ======
print(f"\n{'='*60}")
print("VERDICT")
print("=" * 60)
if t1:
    delta = angles[-1] - angles[0]
    if delta > 180: delta -= 360
    if delta < -180: delta += 360

    passed = True
    if delta < -1.0:
        print(f"  ✅ Direction: +IqRef → NEG mech (Δ={delta:+.1f}°) — matches enc_dir=-1")
    elif delta > 1.0:
        print(f"  ❌ Direction: +IqRef → POS mech (Δ={delta:+.1f}°) — INVERTED vs enc_dir=-1")
        passed = False
    else:
        print(f"  ⚠️  Direction: negligible movement (Δ={delta:+.1f}°)")
        passed = False

    if max_spd < 2.0:
        print(f"  ✅ No runaway (max|speed|={max_spd:.1f} rad/s)")
    else:
        print(f"  ❌ Runaway (max|speed|={max_spd:.1f} rad/s)")
        passed = False

    if max(iqs) < 0.2 and min(iqs) > -0.2:
        print(f"  ✅ Iq stable ({min(iqs):.3f}~{max(iqs):.3f}A)")
    else:
        print(f"  ⚠️  Iq oscillating ({min(iqs):.3f}~{max(iqs):.3f}A)")
        passed = False

    print(f"\n  Overall: {'✅ TORQUE SIGN CORRECT' if passed else '❌ STILL INVERTED'}")
else:
    print("  ❌ No telemetry data")

ser.close()
