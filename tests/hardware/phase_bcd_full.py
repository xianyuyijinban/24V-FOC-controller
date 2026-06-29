"""
Post-brake-release test sequence: Phase B→C→D
Run after: brake physically released, shaft rotates freely by hand
"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=1.5)
time.sleep(0.5)
ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s + '\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def grab(duration=1.5):
    frames = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
                if line.startswith('N,'):
                    p = line.split(',')
                    if len(p) >= 21:
                        frames.append((int(p[1]), int(p[2]), float(p[3]),
                                       float(p[5]), float(p[6]), float(p[16]), float(p[19])))
        time.sleep(0.02)
    return frames

def diag():
    cmd("CMD:FAULT_DETAIL", 2.5)
    time.sleep(1.5)
    for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
        s = line.strip()
        if any(kw in s for kw in ['State:','AppFault','Identified','ThetaDiag','DirDiag',
                                   'FFDiag','DRV8350S Communication','Comm:','FAULT1:']):
            print(f"  {s[:250]}")

drain()
print("=" * 60)
print("B0: Pre-flight check")
print("=" * 60)
diag()

# ============================================================
# B: Low-current torque ramp (0.03→0.05→0.10A)
# ============================================================
print("\n" + "=" * 60)
print("B: Low-current torque test")
print("=" * 60)
drain()
cmd("CMD:UNLOCK,1", 0.4)
drain()
cmd("CMD:MODE,0", 0.3)
cmd("CMD:ENABLE,1", 0.5)
drain()

for iq_target in [0.03, 0.05, 0.10]:
    print(f"\n--- Iq={iq_target}A ---")
    cmd(f"CMD:IREF,0.0,{iq_target}", 0.4)
    frames = grab(2.0)
    if frames:
        states = set(f[1] for f in frames)
        a0, a1 = frames[0][2], frames[-1][2]
        iq_avg = sum(f[4] for f in frames) / len(frames)
        iq_ref_avg = sum(f[6] for f in frames) / len(frames)
        iq_err_pct = abs(iq_avg - iq_target) / iq_target * 100 if iq_target > 0 else 0
        print(f"  states={states} angle: {a0:.2f}°→{a1:.2f}° Δ={a1-a0:.3f}°")
        print(f"  Iq_actual={iq_avg:.3f}A, Iq_ref={iq_ref_avg:.3f}A, err={iq_err_pct:.0f}%")
        status = "OK" if abs(a1-a0) > 0.5 and iq_err_pct < 30 else "CHECK"
        print(f"  VERDICT: {status}")
    else:
        print("  No frames!")

cmd("CMD:ENABLE,0", 0.4)
drain()

# ============================================================
# C: Position closed-loop (if B passed)
# ============================================================
# Only run if motor can rotate freely
print("\n" + "=" * 60)
print("C: Position loop test")
print("=" * 60)
print("  Checking if position mode is viable...")
cmd("CMD:MODE,2", 0.3)
cmd("CMD:ENABLE,1", 0.5)
drain()

# PREF=0 (go to home)
cmd("CMD:PREF,0", 0.5)
frames = grab(3.0)
if frames:
    a_end = frames[-1][2]
    err_deg = abs(a_end)
    print(f"  PREF=0: final angle={a_end:.2f}° error={err_deg:.2f}°")
    if err_deg < 5:
        print("  VERDICT OK")
    else:
        print(f"  VERDICT: need re-identify (error {err_deg:.1f}° > 5°)")

# PREF ±5°
for target_deg in [5, -5, 0]:
    target_rad = target_deg * 3.14159 / 180.0
    print(f"\n--- PREF={target_deg}° ({target_rad:.3f} rad) ---")
    cmd(f"CMD:PREF,{target_rad}", 0.5)
    frames = grab(3.0)
    if frames:
        a_end = frames[-1][2]
        # Convert sensor-frame angle back to control frame
        err = abs(a_end - target_deg)
        print(f"  angle={a_end:.2f}° target={target_deg}° error={err:.2f}°")

cmd("CMD:ENABLE,0", 0.4)
print("\nDone.")
ser.close()
