"""Round 4: Full IDENTIFY with theta_offset +PI fix."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

CMDS = [
    ("CMD:CLEAR_FAULT", 0.5),
    ("CMD:MOTOR_PN,11", 0.3),
    ("CMD:ENCODER_DIR,-1", 0.3),
    ("CMD:UNLOCK,1", 0.3),
    ("CMD:IDENTIFY,1", 120.0),
]

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()
time.sleep(0.2)
ser.reset_input_buffer()

cmd_idx = 0
cmd_timer = time.time()
overall_start = time.time()
id_complete = False

try:
    while True:
        now = time.time()
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            text = data.decode('utf-8', errors='replace')
            # Track identify state from N-frames
            for line in text.split('\n'):
                if line.startswith('N,'):
                    parts = line.split(',')
                    if len(parts) >= 27:
                        try:
                            id_state = int(parts[26]) if parts[26].strip() else -1
                            id_error = int(parts[27]) if parts[27].strip() else -1
                            if id_state == 8:
                                id_complete = True
                        except ValueError:
                            pass
                # Print O-frame / diagnostic lines
                s = line.strip()
                if any(kw in s for kw in ['HOME,OK', 'ENCODER_DIR,OK', 'MotorPn',
                                            'State:', 'error', 'COMPLETE', 'FAIL']):
                    print(s, flush=True)
            sys.stdout.flush()

        if cmd_idx < len(CMDS):
            cmd, delay = CMDS[cmd_idx]
            if now - cmd_timer >= delay:
                print(f"\n--- SENDING: {cmd} ---", flush=True)
                ser.write((cmd + "\r\n").encode('utf-8'))
                ser.flush()
                cmd_timer = time.time()
                cmd_idx += 1
        elif id_complete:
            print("\n=== IDENTIFY COMPLETE ===", flush=True)
            break
        elif now - cmd_timer > 150:
            print("\n--- Timeout waiting for IDENTIFY completion ---", flush=True)
            break

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nInterrupted.", flush=True)
finally:
    ser.close()

# Now send FAULT_DETAIL
print("\n--- Fetching FAULT_DETAIL ---\n", flush=True)
ser2 = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(0.3)
ser2.write(b"CMD:FAULT_DETAIL\r\n")
ser2.flush()

collected = []
t0 = time.time()
while time.time() - t0 < 4.0:
    if ser2.in_waiting:
        data = ser2.read(ser2.in_waiting)
        text = data.decode('utf-8', errors='replace')
        collected.append(text)
    time.sleep(0.05)
ser2.close()

full = ''.join(collected)
for line in full.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag',
                                'DirDiag', 'ParamDiag', 'HOME,OK']):
        print(s)

# Verify new offset
print(f"\n{'='*50}")
for line in full.split('\n'):
    if 'ThetaDiag' in line:
        print(line.strip())
        # Extract offset
        import re
        m = re.search(r'offset=([\d.-]+) rad', line)
        if m:
            offset = float(m.group(1))
            expected = 1.069 + 3.14159  # old offset + pi
            print(f"  Old offset: 1.069 rad")
            print(f"  Expected:   ~{expected:.3f} rad (= 1.069 + π)")
            print(f"  Actual:     {offset:.3f} rad")
            diff = abs((offset % 6.28318) - (expected % 6.28318))
            if diff < 0.5:
                print(f"  ✅ Offset matches expectation (diff={diff:.3f} rad)")
            else:
                print(f"  ⚠️  Offset differs from expectation (diff={diff:.3f} rad)")
print(f"{'='*50}")
