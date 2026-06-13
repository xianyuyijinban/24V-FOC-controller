"""Current loop P-only test: Kp=0.03, IqRef=0.05A, check stability."""
import serial, time, sys, io, re
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

def send(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

def read_all(ser, duration, verbose=True):
    """Read and print telemetry for duration. Return N-frame snapshots."""
    snaps = []
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
                                'tick': int(parts[1]), 'focState': int(parts[2]),
                                'angle': float(parts[3]), 'speed': float(parts[4]),
                                'Id': float(parts[5]), 'Iq': float(parts[6]),
                                'IqRef': float(parts[19]) if parts[19] else 0,
                                'faultFlags': parts[8],
                            }
                            snaps.append(snap)
                        except (ValueError, IndexError): pass
                s = line.strip()
                if any(kw in s for kw in ['FAULT', 'fault', 'CURRENT_SNAP', 'Diag']):
                    if len(s) > 10:
                        print(f"  {s}", flush=True)
        time.sleep(0.01)
    return snaps

def stats(snaps, label):
    if not snaps: return
    angles = [s['angle'] for s in snaps]
    iqs = [s['Iq'] for s in snaps]
    iq_refs = [s['IqRef'] for s in snaps]
    speeds = [s['speed'] for s in snaps]
    delta = angles[-1] - angles[0]
    if delta > 180: delta -= 360
    if delta < -180: delta += 360
    max_speed = max(abs(s) for s in speeds)
    print(f"  {label}: angle Δ={delta:+.1f}° | Iq={min(iqs):.3f}~{max(iqs):.3f}A "
          f"| IqRef={min(iq_refs):.3f}~{max(iq_refs):.3f}A | max|speed|={max_speed:.1f} rad/s",
          flush=True)
    if max_speed > 2.0:
        print(f"  ⚠️ RUNAWAY", flush=True)
    elif max(iqs) > 0.5 or min(iqs) < -0.5:
        print(f"  ⚠️ Iq oscillation > ±0.5A", flush=True)
    else:
        print(f"  ✅ Stable", flush=True)

ser = serial.Serial(PORT, BAUD, timeout=0.05)
ser.reset_input_buffer()
time.sleep(0.2)
ser.reset_input_buffer()

# === Step 1: Disable, set P-only low gain ===
print("=== Setup: disable, P-only Kp=0.03 ===", flush=True)
send(ser, "CMD:ENABLE,0")
send(ser, "CMD:PI_CURRENT,0.03,0")
time.sleep(0.2)

# Verify PI setting took effect via FAULT_DETAIL
send(ser, "CMD:FAULT_DETAIL", wait=2.0)
# Drain response (we'll get another one later)
ser.reset_input_buffer()

# === Step 2: Enable torque mode with zero current ===
print("\n=== Enable torque mode, IqRef=0 ===", flush=True)
send(ser, "CMD:IREF,0,0")
send(ser, "CMD:MODE,0")
send(ser, "CMD:ENABLE,1")
time.sleep(0.3)

print("Baseline (300ms):", flush=True)
bl = read_all(ser, 0.3, verbose=False)
stats(bl, "Baseline")

if bl and max(abs(s['speed']) for s in bl) > 2:
    print("⛔ Already unstable at IqRef=0. Aborting.", flush=True)
    send(ser, "CMD:ENABLE,0")
    ser.close()
    sys.exit(1)

# === Step 3: Apply IqRef=0.05A, monitor ===
print("\n=== IqRef=0.05A (600ms) ===", flush=True)
send(ser, "CMD:IREF,0,0.05", wait=0.05)
t1 = read_all(ser, 0.6)
stats(t1, "+IqRef=0.05A")

# === Step 4: Diagnostic snapshots ===
print("\n--- CURRENT_SNAP ---", flush=True)
send(ser, "CMD:CURRENT_SNAP")
time.sleep(0.5)
print("\n--- FAULT_DETAIL ---", flush=True)
send(ser, "CMD:FAULT_DETAIL")
time.sleep(2.0)

# === Step 5: Zero and disable ===
print("\n=== Zero current, disable ===", flush=True)
send(ser, "CMD:IREF,0,0")
time.sleep(0.3)
send(ser, "CMD:ENABLE,0")

# Print collected diagnostics
collected = []
t0 = time.time()
while time.time() - t0 < 2.0:
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        text = data.decode('utf-8', errors='replace')
        collected.append(text)
    time.sleep(0.05)

full = ''.join(collected)
for line in full.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['CURRENT_SNAP', 'CurrentDQ', 'LoopABC', 'AdcDQ',
                                'State:', 'AppFault', 'ThetaDiag']):
        print(f"  {s}", flush=True)

# Final assessment
print(f"\n{'='*50}", flush=True)
if t1:
    iqs = [s['Iq'] for s in t1]
    speeds = [s['speed'] for s in t1]
    if max(abs(s) for s in speeds) < 2.0 and max(iqs) < 0.5 and min(iqs) > -0.5:
        print("✅ P-only Kp=0.03, IqRef=0.05 — STABLE", flush=True)
        print("   Ready to step up: Kp=0.05, IqRef=0.05→0.10→0.15", flush=True)
    else:
        print("⚠️  Still unstable at Kp=0.03 — need deeper investigation", flush=True)
        print("   → Check current sampling phase / low-side reconstruction / Park feedback sign", flush=True)
else:
    print("❌ No telemetry data", flush=True)
print(f"{'='*50}", flush=True)

ser.close()
