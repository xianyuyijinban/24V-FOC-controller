"""
P0 Phase Sweep: scan cog_phase_offset, compare PREF errors vs P0 off baseline.
Tests phases: 0, +/-15, +/-30, +/-45, +/-60, +/-90 degrees, plus P0 off.
"""
import serial, time, sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=2)
time.sleep(0.5); ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s+'\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def grab(d=2.5):
    frames = []
    t0 = time.time()
    while time.time()-t0 < d:
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode('utf-8','replace').split('\n'):
                if l.startswith('N,') and len(p:=l.split(','))>=22:
                    frames.append({'st':int(p[2]),'ang':float(p[3]),'Iq':float(p[6])})
        time.sleep(0.01)
    return frames

def test_phase(phase_deg, label):
    """Run PREF sequence with given phase offset. Returns avg error."""
    cmd("CMD:COG_PHASE,{}".format(phase_deg * math.pi / 180.0), 0.3)
    drain()
    cmd("CMD:CLEAR_FAULT", 0.5); drain()
    cmd("CMD:UNLOCK,1", 0.4)
    cmd("CMD:HOME", 0.5); drain()
    cmd("CMD:MODE,2", 0.3)
    cmd("CMD:ENABLE,1", 0.5); drain()
    time.sleep(0.3)

    errors = []
    for target in [0, 10, -10, 20, -20, 0]:
        cmd("CMD:PREF,{}".format(target * math.pi / 180.0), 0.4)
        f = grab(2.5)
        run = [x for x in f if x['st'] == 4]
        if run:
            a_end = run[-1]['ang']
            err = min(abs(a_end-target), 360-abs(a_end-target))
            errors.append(abs(err))
    cmd("CMD:ENABLE,0", 0.5)
    avg_err = sum(errors)/len(errors) if errors else 99
    print("  {:>12s}  avg_err={:5.1f} deg  errors={}".format(label, avg_err,
          ",".join("{:.0f}".format(e) for e in errors)))
    return avg_err

print("=" * 60)
print("P0 Phase Sweep (gain=0.25x)")
print("=" * 60)

# Baseline: P0 off
print("\n--- P0 OFF baseline ---")
cmd("CMD:COG_PHASE,0", 0.3)
# We can't turn P0 off at runtime, so skip baseline for now
# Just sweep phases

results = []
for phase_deg in [0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90]:
    label = "phase={:+d} deg".format(phase_deg)
    print("\n--- {} ---".format(label))
    avg = test_phase(phase_deg, label)
    results.append((phase_deg, avg))

print("\n" + "=" * 60)
print("Phase Sweep Summary (lower avg_err = better)")
print("=" * 60)
results.sort(key=lambda x: x[1])
for phase, err in results:
    bar = "*" * int(err/2)
    print("  {:+4d} deg  avg_err={:5.1f} deg  {}".format(phase, err, bar))

best_phase = results[0][0]
print("\nBest phase: {} deg (avg_err={:.1f})".format(best_phase, results[0][1]))

ser.close()
print("\nDone.")
