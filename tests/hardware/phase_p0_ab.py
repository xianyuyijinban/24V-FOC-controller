"""
Phase P0 A/B: Cogging LUT validation
P0 already active (cog_valid=1). Compare with Phase E baseline.
"""
import serial, time, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=2)
time.sleep(0.5); ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s+'\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def get_ffdiag():
    cmd("CMD:FAULT_DETAIL", 2.0)
    time.sleep(2.5)
    t = ser.read(ser.in_waiting).decode('utf-8','replace')
    for l in t.split('\n'):
        if 'FFDiag:' in l.strip():
            m = re.search(r'total=([-\d.]+).*bemf.*Vq=([-\d.]+).*en=(\d).*inertia.*Iq=([-\d.]+).*blk=(\d).*friction.*Iq=([-\d.]+).*en=(\d).*cogging.*Iq=([-\d.]+).*en=(\d)', l)
            if m:
                return {
                    'total': float(m.group(1)), 'bemf_vq': float(m.group(2)),
                    'bemf_en': int(m.group(3)), 'inertia_iq': float(m.group(4)),
                    'inertia_blk': int(m.group(5)), 'friction_iq': float(m.group(6)),
                    'friction_en': int(m.group(7)), 'cogging_iq': float(m.group(8)),
                    'cogging_en': int(m.group(9))
                }
    return {}

def grab(d=3.0):
    frames = []
    t0 = time.time()
    while time.time()-t0 < d:
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode('utf-8','replace').split('\n'):
                if l.startswith('N,') and len(p:=l.split(','))>=22:
                    frames.append({
                        'st': int(p[2]), 'ang': float(p[3]),
                        'Iq': float(p[6]), 'Iq_ref': float(p[19]),
                    })
        time.sleep(0.01)
    return frames

print("=" * 60)
print("Phase P0 A/B: Cogging LUT Validation")
print("=" * 60)

drain()
cmd("CMD:CLEAR_FAULT", 0.5); drain()
cmd("CMD:UNLOCK,1", 0.4)
cmd("CMD:HOME", 0.5); drain()
cmd("CMD:MODE,2", 0.3)
cmd("CMD:ENABLE,1", 0.5); drain()
time.sleep(0.5)

# Slow sweep: PREF 0→10→20→30→-10→-20→-30→0
# Capture FFDiag at each hold to see cogging_iq pattern
print("\n--- Slow PREF sweep (observe cogging_iq) ---")
for target in [0, 5, 10, 20, -5, -10, -20, 0]:
    trad = target * 3.14159 / 180.0
    cmd(f"CMD:PREF,{trad}", 0.4)
    time.sleep(1.5)
    ff = get_ffdiag()
    cog = ff.get('cogging_iq', 0)
    print(f"  PREF={target:+3d} deg  cogging_iq={cog:+.4f}A  cog_en={ff.get('cogging_en','?')}")

# Now standard PREF steps with settling metrics
print("\n--- PREF step response ---")
for target in [0, 10, -10, 20, -20, 0]:
    trad = target * 3.14159 / 180.0
    print(f"\n  PREF={target:+3d} deg:")
    cmd(f"CMD:PREF,{trad}", 0.4)
    frames = grab(4.0)
    run = [f for f in frames if f['st'] == 4]
    if run:
        a_end = run[-1]['ang']
        err = min(abs(a_end-target), 360-abs(a_end-target))
        iq_peak = max(abs(f['Iq']) for f in run)
        iq_settle = sum(abs(f['Iq']) for f in run[-20:])/min(20, len(run[-20:]))
        status = "PASS" if err < (5 if abs(target)>=20 else 3) else "CHECK"
        print(f"    angle={a_end:.1f} deg  err={err:.1f} deg  Iq_peak={iq_peak:.3f}A  Iq_settle={iq_settle:.4f}A  [{status}]")

# FFDiag during motion
print("\n--- FFDiag during motion ---")
cmd("CMD:PREF,0.349", 0.4)  # 20 deg
time.sleep(1)
ff = get_ffdiag()
print(f"  total={ff.get('total',0):.3f}A  inertia={ff.get('inertia_iq',0):.3f}A  friction={ff.get('friction_iq',0):.3f}A  cogging={ff.get('cogging_iq',0):.3f}A")
print(f"  bemf_en={ff.get('bemf_en',0)}  inertia_blk={ff.get('inertia_blk',0)}  friction_en={ff.get('friction_en',0)}  cogging_en={ff.get('cogging_en',0)}")

cmd("CMD:ENABLE,0", 0.5)
print("\nDone.")
ser.close()
