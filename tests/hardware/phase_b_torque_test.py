"""Clean test: CLEAR_FAULT first, then torque ramp with full diagnostics"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s + '\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

def grab_raw(duration=2.0):
    lines = []
    t0 = time.time()
    while time.time() - t0 < duration:
        if ser.in_waiting:
            for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
                if line.startswith('N,'):
                    lines.append(line.strip())
        time.sleep(0.02)
    return lines

# ====== Step 1: CLEAR FAULT ======
print("=" * 60)
print("Step 1: CLEAR_FAULT")
print("=" * 60)
drain()
cmd("CMD:CLEAR_FAULT", 0.5)
drain()
# Verify state
cmd("CMD:SAFETY_STAT", 0.4)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    if line.startswith('N,'):
        p = line.split(',')
        if len(p) >= 3:
            print(f"  state={p[2]} (expect 3=READY) appFault={p[14] if len(p)>14 else '?'}")

# ====== Step 2: FAULT_DETAIL baseline ======
print("\n" + "=" * 60)
print("Step 2: Pre-test FAULT_DETAIL")
print("=" * 60)
cmd("CMD:FAULT_DETAIL", 2.0)
time.sleep(2.5)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['State:','AppFault','Identified','ThetaDiag','Power:',
                               'DRV8350S Communication','Comm:','FAULT1:']):
        print(f"  {s[:250]}")

# ====== Step 3: UNLOCK + ENABLE ======
print("\n" + "=" * 60)
print("Step 3: UNLOCK + TORQUE MODE + ENABLE")
print("=" * 60)
drain()
cmd("CMD:UNLOCK,1", 0.4)
cmd("CMD:MODE,0", 0.3)
cmd("CMD:ENABLE,1", 0.5)
drain()

# Verify RUNNING
cmd("CMD:SAFETY_STAT", 0.4)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    if line.startswith('N,'):
        p = line.split(',')
        if len(p) >= 3:
            st = int(p[2])
            print(f"  state={st} {'✅ RUNNING' if st==4 else '❌ NOT RUNNING (expect 4)'}")

# ====== Step 4: Torque ramp ======
for iq_cmd in [0.05, 0.10, 0.20, 0.50, 1.00]:
    print(f"\n{'='*60}")
    print(f"Iq_ref = {iq_cmd}A")
    print(f"{'='*60}")
    cmd(f"CMD:IREF,0.0,{iq_cmd}", 0.4)
    lines = grab_raw(2.0)

    if not lines:
        print("  NO DATA")
        continue

    # Parse last 5 frames for steady state
    angles = []
    iqs = []
    vqs = []
    ias = []
    states = set()
    for line in lines[-10:]:
        p = line.split(',')
        if len(p) >= 25:
            try:
                st = int(p[2])
                states.add(st)
                if st == 4:  # Only RUNNING frames
                    angles.append(float(p[3]))
                    iqs.append(float(p[6]))
                    vqs.append(float(p[21]))
                    ias.append(abs(float(p[22])) + abs(float(p[23])) + abs(float(p[24])))
            except (ValueError, IndexError):
                pass

    if angles:
        iq_avg = sum(iqs)/len(iqs)
        vq_avg = sum(vqs)/len(vqs)
        ia_sum_avg = sum(ias)/len(ias)
        a0, a1 = lines[0].split(',')[3], lines[-1].split(',')[3]
        da = float(a1) - float(a0)
        print(f"  states={states} | Δangle={da:.4f}°")
        print(f"  Iq_avg={iq_avg:.4f}A | Vq_avg={vq_avg:.3f}V | Σ|Iabc|_avg={ia_sum_avg:.4f}A")
        if abs(da) > 0.5:
            print(f"  ✅ MOTOR MOVING (Δ={da:.2f}°)")
        else:
            print(f"  ⚠️  Minimal movement (Δ={da:.4f}°)")
        if iq_avg > 0.001:
            print(f"  ✅ Current flowing (Iq={iq_avg:.3f}A)")
        else:
            print(f"  ❌ No current!")
    else:
        print(f"  states={states} (no RUNNING frames)")

# ====== Step 5: Disable ======
print("\n" + "=" * 60)
print("Step 5: DISABLE + FINAL CHECK")
print("=" * 60)
cmd("CMD:ENABLE,0", 0.5)
cmd("CMD:FAULT_DETAIL", 2.0)
time.sleep(2.5)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['State:','AppFault','ThetaDiag','FFDiag','SpeedLoopDiag']):
        print(f"  {s[:250]}")

ser.close()
print("\nDone.")
