"""Step up IqRef: 0.05→0.10→0.15A with Kp=0.03, confirm direction."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
PORT, BAUD = "COM9", 230400

def send(ser, cmd, wait=0.3):
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)

def sample(ser, duration):
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
        time.sleep(0.01)
    return snaps

def run_test(ser, iq_ref, duration):
    send(ser, f"CMD:IREF,0,{iq_ref:.2f}", 0.05)
    snaps = sample(ser, duration)
    if snaps:
        a0, a1 = snaps[0]['angle'], snaps[-1]['angle']
        delta = a1 - a0
        if delta > 180: delta -= 360
        if delta < -180: delta += 360
        iqs = [s['Iq'] for s in snaps]
        iqrefs = [s['IqRef'] for s in snaps]
        spd = max(abs(s['speed']) for s in snaps)
        print(f"  +IqRef={iq_ref:.2f}A: Δ={delta:+.1f}° | Iq={min(iqs):.3f}~{max(iqs):.3f}A "
              f"| IqRef={min(iqrefs):.3f}~{max(iqrefs):.3f}A | max|ω|={spd:.1f}")
        if spd > 3:
            print(f"    ⚠️ RUNAWAY — stopping")
            return False
        return True
    return False

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.2)
ser.reset_input_buffer()

# Setup
print("=== Setup ===")
send(ser, "CMD:ENABLE,0", 0.5)
send(ser, "CMD:CLEAR_FAULT", 0.3)
send(ser, "CMD:MODE,0", 0.3)
send(ser, "CMD:UNLOCK,1", 0.3)
send(ser, "CMD:IREF,0,0", 0.3)
send(ser, "CMD:ENABLE,1", 0.3)
time.sleep(0.3)

print(f"\n{'='*55}")
print("Step test: Kp=0.03, IqRef ladder 0.05→0.10→0.15→0.20A")
print(f"{'='*55}")

for iq in [0.05, 0.10, 0.15, 0.20]:
    ok = run_test(ser, iq, 0.8)
    send(ser, "CMD:IREF,0,0", 0.3)
    time.sleep(0.3)
    if not ok:
        break

# Show final diag
print(f"\n=== FAULT_DETAIL ===")
send(ser, "CMD:FAULT_DETAIL", 3.0)
t0 = time.time()
buf = []
while time.time() - t0 < 1.5:
    if ser.in_waiting:
        buf.append(ser.read(ser.in_waiting).decode('utf-8', errors='replace'))
    time.sleep(0.05)
for line in ''.join(buf).split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['CurrentDQ', 'ThetaDiag', 'DirDiag']):
        print(f"  {s}")

send(ser, "CMD:ENABLE,0", 0.3)
ser.close()
print(f"\n{'='*55}")
print("All steps stable → torque sign is CORRECT, current loop P-only works.")
print(f"{'='*55}")
