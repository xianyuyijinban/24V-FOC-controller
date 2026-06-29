"""
Phase C: HOME then PREF sequence.
Verifies: CMD:HOME sets mech_zero_offset, position mapping is self-consistent.
"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=1.5)
time.sleep(0.5)
ser.reset_input_buffer()

def cmd(s, wait=0.4):
    ser.write((s+'\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def read_diag():
    """Read FAULT_DETAIL and return key fields"""
    info = {}
    cmd("CMD:FAULT_DETAIL", 2.0)
    time.sleep(2.5)
    text = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
    for line in text.split('\n'):
        s = line.strip()
        if 'ThetaDiag:' in s:
            # mech=X rad | elec=Y rad | offset=Z rad | zero=W rad | Pn=N | enc_dir=D
            import re
            m = re.search(r'mech=([-\d.]+).*?zero=([-\d.]+).*?enc_dir=([-\d]+)', s)
            if m:
                info['mech'] = float(m.group(1))
                info['zero'] = float(m.group(2))
                info['enc_dir'] = int(m.group(3))
        if 'State:' in s and 'AppFault' not in s:
            info['state'] = s.split('State:')[1].strip().split()[0]
    for line in text.split('\n'):
        if line.startswith('N,'):
            p = line.split(',')
            if len(p) >= 4:
                info['angle_deg'] = float(p[3])
            break
    return info

def grab_pos(d=3.0):
    """Monitor position loop frames"""
    frames = []
    t0 = time.time()
    while time.time() - t0 < d:
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode('utf-8',errors='replace').split('\n'):
                if l.startswith('N,'):
                    p = l.split(',')
                    if len(p) >= 22:
                        frames.append({
                            'st': int(p[2]), 'ang': float(p[3]),
                            'spd': float(p[4]), 'Iq': float(p[6]),
                            'Iq_ref': float(p[19]), 'pos_ref': float(p[18]),
                        })
        time.sleep(0.02)
    return frames

# ===== Step 1: Check current state =====
print("=" * 60)
print("Step 1: Current State (before HOME)")
print("=" * 60)
drain()
info = read_diag()
print(f"  state={info.get('state','?')} mech={info.get('mech','?'):.3f} rad zero={info.get('zero','?'):.3f} rad enc_dir={info.get('enc_dir','?')}")
print(f"  angle_sensor={info.get('angle_deg','?'):.1f} deg")

# ===== Step 2: HOME =====
print("\n" + "=" * 60)
print("Step 2: CMD:HOME")
print("=" * 60)
cmd("CMD:HOME", 0.8)
time.sleep(0.5)
raw = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
for line in raw.split('\n'):
    if 'HOME' in line.upper():
        print(f"  RESPONSE: {line.strip()[:200]}")

# Verify zero changed
print("\n  After HOME:")
info2 = read_diag()
print(f"  mech={info2.get('mech','?'):.3f} rad zero={info2.get('zero','?'):.3f} rad")
if abs(info2.get('zero', 0)) > 0.001:
    print("  mech_zero_offset SET")
else:
    print("  WARNING: mech_zero_offset still 0")

# ===== Step 3: Enable position mode =====
print("\n" + "=" * 60)
print("Step 3: Position Mode Enable")
print("=" * 60)
cmd("CMD:CLEAR_FAULT", 0.5); drain()
cmd("CMD:UNLOCK,1", 0.4)
cmd("CMD:MODE,2", 0.3)
cmd("CMD:ENABLE,1", 0.5); drain()

# ===== Step 4: PREF sequence =====
print("\n" + "=" * 60)
print("Step 4: PREF Sequence")
print("=" * 60)

# Record home position (should be near 0 after HOME)
f0 = grab_pos(1.0)
run0 = [f for f in f0 if f['st'] == 4]
if run0:
    home_ang = run0[-1]['ang']
    print(f"  Position at enable: {home_ang:.1f} deg")

for target_deg in [0, 5, -5, 20, -20, 0]:
    target_rad = target_deg * 3.14159 / 180.0
    print(f"\n--- PREF={target_deg} deg ({target_rad:.3f} rad) ---")
    cmd(f"CMD:PREF,{target_rad}", 0.5)
    frames = grab_pos(3.0)
    run = [f for f in frames if f['st'] == 4]
    if run:
        a_end = run[-1]['ang']
        err = abs(a_end - target_deg)
        # Also compute shortest angular distance
        err_short = min(err, 360 - err)
        status = "PASS" if err_short < 5 else ("WEAK" if err_short < 15 else "FAIL")
        print(f"  final_angle={a_end:.1f} deg  target={target_deg} deg  error={err_short:.1f} deg  [{status}]")
        print(f"  last_Iq={run[-1]['Iq']:.3f}A  pos_ref_raw={run[-1]['pos_ref']:.3f} rad")
    else:
        states = set(f['st'] for f in frames)
        print(f"  NO RUNNING FRAMES (states={states})")

# ===== Step 5: Disable =====
cmd("CMD:ENABLE,0", 0.5)
print("\nDone.")
ser.close()
