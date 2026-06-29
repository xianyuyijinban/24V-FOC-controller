"""
Speed mode test: try to spin motor at 1, 3, 5 rad/s both directions.
Observe if motor rotates, direction, current draw.
"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s + '\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def grab(duration=3.0):
    frames = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
                if line.startswith('N,'):
                    p = line.split(',')
                    if len(p) >= 25:
                        try:
                            frames.append({
                                't': int(p[1]), 'st': int(p[2]),
                                'ang': float(p[3]), 'spd': float(p[4]),
                                'Iq': float(p[6]), 'Iq_ref': float(p[19]),
                                'Vq': float(p[21]),
                            })
                        except: pass
        time.sleep(0.02)
    return frames

# Setup
drain()
cmd("CMD:CLEAR_FAULT", 0.5)
drain()
cmd("CMD:UNLOCK,1", 0.4)
drain()

# Speed mode
cmd("CMD:MODE,1", 0.3)  # Speed mode
cmd("CMD:ENABLE,1", 0.5)
drain()

# Test speeds
for spd in [1.0, -1.0, 3.0, -3.0, 5.0, -5.0]:
    print(f"\n{'='*60}")
    print(f"Speed ref = {spd} rad/s")
    print(f"{'='*60}")
    cmd(f"CMD:SREF,{spd}", 0.4)
    frames = grab(3.0)

    if not frames:
        print("  NO DATA")
        continue

    states = set(f['st'] for f in frames)
    a0, a1 = frames[0]['ang'], frames[-1]['ang']
    spd_vals = [f['spd'] for f in frames if f['st'] == 4]
    iq_vals = [f['Iq'] for f in frames if f['st'] == 4]
    iq_refs = [f['Iq_ref'] for f in frames if f['st'] == 4]
    vq_vals = [f['Vq'] for f in frames if f['st'] == 4]

    spd_avg = sum(spd_vals)/len(spd_vals) if spd_vals else 0
    iq_avg = sum(iq_vals)/len(iq_vals) if iq_vals else 0
    iqr_avg = sum(iq_refs)/len(iq_refs) if iq_refs else 0

    print(f"  states={states} | Δangle={a1-a0:.2f}°")
    print(f"  speed_avg={spd_avg:.3f} rad/s | Iq_avg={iq_avg:.3f}A | Iq_ref_avg={iqr_avg:.3f}A")
    if abs(a1 - a0) > 5:
        print(f"  ✅ MOTOR ROTATING (Δ={a1-a0:.1f}°)")
    elif abs(spd_avg) > 0.1:
        print(f"  ✅ Motor has speed ({spd_avg:.2f} rad/s)")
    else:
        print(f"  ❌ Motor NOT rotating")

    # Show last 3 frames
    for f in frames[-3:]:
        print(f"    t={f['t']} st={f['st']} ang={f['ang']:.2f}° spd={f['spd']:.2f} Iq={f['Iq']:.3f}/{f['Iq_ref']:.3f}A Vq={f['Vq']:.2f}V")

    # Stop if fault
    if 5 in states:
        print("  ⚠️ FAULT detected, stopping")
        break

cmd("CMD:ENABLE,0", 0.5)
ser.close()
print("\nDone.")
