"""
Phase E: Load Acceptance Test
P1+P2+P3 on, P0/P4 off. PREF steps with 5s dwell + 0<->20 cycling.
Records: err, iq_cmd, FFDiag, settle_time.
"""
import serial, time, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=2)
time.sleep(0.5); ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s+'\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def grab_all(duration=5.0):
    """Capture all N-frames + any diagnostic lines for duration"""
    frames = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode('utf-8',errors='replace').split('\n'):
                if l.startswith('N,') and len(p:=l.split(','))>=22:
                    frames.append({
                        't_ms': int(p[1]), 'st': int(p[2]),
                        'ang': float(p[3]), 'spd': float(p[4]),
                        'Iq': float(p[6]), 'Iq_ref': float(p[19]),
                        'Vq': float(p[21]), 'mode': int(p[15]),
                    })
        time.sleep(0.01)
    return frames

def get_ffdiag():
    """Get current FFDiag values"""
    cmd("CMD:FAULT_DETAIL", 2.0)
    time.sleep(2.5)
    t = ser.read(ser.in_waiting).decode('utf-8','replace')
    result = {}
    for l in t.split('\n'):
        s = l.strip()
        if 'FFDiag:' in s:
            m = re.search(r'total=([-\d.]+).*bemf.*Vq=([-\d.]+).*en=(\d).*inertia.*Iq=([-\d.]+).*blk=(\d).*friction.*Iq=([-\d.]+).*en=(\d).*enc_dir_blk=(\d)', s)
            if m:
                result = {k:float(m.group(i)) for i,k in enumerate(
                    ['total','bemf_vq','bemf_en','inertia_iq','inertia_blk',
                     'friction_iq','friction_en','enc_blk'],1)}
    return result

def get_jdiag():
    """Get J/B values"""
    cmd("CMD:JDIAG", 0.5)
    time.sleep(0.5)
    t = ser.read(ser.in_waiting).decode('utf-8','replace')
    for l in t.split('\n'):
        if l.startswith('JDIAG,'):
            return l.strip()
    return None

RESULTS = []

# ===== Setup =====
print("=" * 60)
print("Phase E: Load Acceptance Test")
print("=" * 60)

drain()
jdiag = get_jdiag()
print(f"  Params: {jdiag}")

ff = get_ffdiag()
print(f"  FF idle: total={ff.get('total',0):.3f}A bemf_en={ff.get('bemf_en',0):.0f} inertia_blk={ff.get('inertia_blk',0):.0f} friction_en={ff.get('friction_en',0):.0f}")

# HOME + Position mode
cmd("CMD:CLEAR_FAULT", 0.5); drain()
cmd("CMD:UNLOCK,1", 0.4)
cmd("CMD:HOME", 0.5); drain()
cmd("CMD:MODE,2", 0.3)
cmd("CMD:ENABLE,1", 0.5); drain()
time.sleep(0.5)

# ===== PREF Steps =====
print("\n" + "=" * 60)
print("PREF Step Response (5s dwell each)")
print("=" * 60)

for target_deg in [0, 10, -10, 20, -20, 0]:
    target_rad = target_deg * 3.14159 / 180.0
    label = f"PREF={target_deg:+d}"
    print(f"\n--- {label} deg ---")
    cmd(f"CMD:PREF,{target_rad}", 0.4)
    frames = grab_all(5.0)

    run = [f for f in frames if f['st'] == 4]
    if not run:
        print("  NO RUNNING FRAMES")
        continue

    # Settle: first 1s is transient, last 4s is settled
    N = len(run)
    if N < 10:
        print(f"  Too few frames ({N})")
        continue

    settle_cut = min(N//5, 20)  # first ~1s
    transient = run[:settle_cut]
    settled = run[settle_cut:]

    a_start = run[0]['ang']
    a_end = settled[-1]['ang'] if settled else run[-1]['ang']
    err = min(abs(a_end - target_deg), 360 - abs(a_end - target_deg))

    iq_transient = max(abs(f['Iq']) for f in transient) if transient else 0
    iq_settled = sum(abs(f['Iq']) for f in settled) / max(1, len(settled))
    iqr_settled = sum(abs(f['Iq_ref']) for f in settled) / max(1, len(settled))

    # Settle time: when |error| < 3 deg and stays
    settle_time = 0.0
    for i, f in enumerate(run):
        e = min(abs(f['ang'] - target_deg), 360 - abs(f['ang'] - target_deg))
        if e < 3.0:
            # Check next 5 frames
            if all(min(abs(run[j]['ang'] - target_deg), 360 - abs(run[j]['ang'] - target_deg)) < 3.0
                   for j in range(i, min(i+5, len(run)))):
                settle_time = (f['t_ms'] - run[0]['t_ms']) / 1000.0
                break

    status = "PASS" if err < (5 if abs(target_deg) >= 20 else 3) else "CHECK"
    print(f"  angle: {a_start:.1f} -> {a_end:.1f}  err={err:.1f} deg  [{status}]")
    print(f"  Iq: transient_max={iq_transient:.3f}A  settled_avg={iq_settled:.3f}A  ref_avg={iqr_settled:.3f}A")
    print(f"  settle_time={settle_time:.1f}s")

    RESULTS.append({
        'target': target_deg, 'err': err, 'status': status,
        'Iq_transient': iq_transient, 'Iq_settled': iq_settled,
        'settle_time': settle_time
    })

# FFDiag snapshot during hold
ff_end = get_ffdiag()
print(f"\n  FF hold: total={ff_end.get('total',0):.3f}A  inertia_iq={ff_end.get('inertia_iq',0):.3f}A  friction_iq={ff_end.get('friction_iq',0):.3f}A")

# ===== 0<->20 cycling =====
print("\n" + "=" * 60)
print("0 <-> 20 deg cycling (10 cycles)")
print("=" * 60)

faults = 0
for cycle in range(10):
    for target in [20, 0]:
        cmd(f"CMD:PREF,{target * 3.14159 / 180.0}", 0.2)
        time.sleep(0.5)  # 0.5s per step = 1s per cycle

    # Quick N-frame check every 2 cycles
    if cycle % 2 == 0:
        f = grab_all(1.0)
        run = [x for x in f if x['st'] == 4]
        if run:
            a = run[-1]['ang']
            iq = run[-1]['Iq']
            states = set(x['st'] for x in f)
            if 5 in states: faults += 1
        print(f"  cycle {cycle}: angle={a:.1f}  Iq={iq:.3f}A  faults={faults}")

print(f"\n  Cycling done: {10} cycles, faults={faults}")

# ===== Summary =====
print("\n" + "=" * 60)
print("Phase E SUMMARY")
print("=" * 60)
all_pass = all(r['status'] == 'PASS' for r in RESULTS)
print(f"  PREF steps: {'ALL PASS' if all_pass else 'SOME CHECK'}")
for r in RESULTS:
    print(f"    PREF={r['target']:+3d}: err={r['err']:.1f}  settle={r['settle_time']:.1f}s  Iq_t={r['Iq_transient']:.3f}A  [{r['status']}]")
print(f"  Cycling: faults={faults}")
print(f"  FF active: bemf_en={ff_end.get('bemf_en',0):.0f}  inertia_blk={ff_end.get('inertia_blk',0):.0f}  friction_en={ff_end.get('friction_en',0):.0f}")

cmd("CMD:ENABLE,0", 0.5)
ser.close()
print("\nDone.")
