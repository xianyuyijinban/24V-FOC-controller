"""Cold boot check: verify Identified:NO after PARAM_VERSION bump."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400

ser = serial.Serial(PORT, BAUD, timeout=0.2)
ser.reset_input_buffer()
time.sleep(0.3)
ser.reset_input_buffer()

print("--- SENDING: CMD:CLEAR_FAULT ---", flush=True)
ser.write(b"CMD:CLEAR_FAULT\r\n")
ser.flush()
time.sleep(0.5)

print("--- SENDING: CMD:FAULT_DETAIL ---\n", flush=True)
ser.write(b"CMD:FAULT_DETAIL\r\n")
ser.flush()

collected = []
t0 = time.time()
while time.time() - t0 < 4.0:
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        text = data.decode('utf-8', errors='replace')
        collected.append(text)
    time.sleep(0.05)
ser.close()

full = ''.join(collected)

# Extract key lines
for line in full.split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag',
                                'valid_flag', 'PARAM', 'ParamDiag', 'invalid=']):
        print(s)

# Check
print(f"\n{'='*50}")
if 'Identified: NO' in full or 'Identified:NO' in full:
    print("✅ Identified:NO — PARAM_VERSION bump works")
elif 'Identified: YES' in full:
    print("⚠️  Identified:YES — old params still valid?")
else:
    # check N-frame motorIdentified field
    print("Check N-frame motorIdentified...")
    for line in full.split('\n'):
        if line.startswith('N,'):
            parts = line.split(',')
            if len(parts) >= 11:
                print(f"  motorIdentified={parts[10]} focState={parts[2]}")
            break

if 'State:    READY' in full:
    print("✅ State: READY (no fault)")
elif 'State:    FAULT' in full:
    print("⚠️  State: FAULT")
print(f"{'='*50}")
