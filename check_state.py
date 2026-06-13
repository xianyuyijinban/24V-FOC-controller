"""Quick state check: FAULT_DETAIL + N-frame idState."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
PORT, BAUD = "COM9", 230400

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

# Get latest N-frames first
print("=== Latest N-frames ===")
raw = b''
t0 = time.time()
while time.time() - t0 < 1.0:
    if ser.in_waiting:
        raw += ser.read(ser.in_waiting)
    time.sleep(0.05)

for line in raw.decode('utf-8', errors='replace').split('\n'):
    if line.startswith('N,'):
        parts = line.split(',')
        if len(parts) >= 27:
            print(f"  tick={parts[1]} focState={parts[2]} idState={parts[26]} idError={parts[27]}")

# Then FAULT_DETAIL
print("\n=== FAULT_DETAIL ===")
ser.write(b"CMD:FAULT_DETAIL\r\n")
ser.flush()
time.sleep(3.0)

raw = b''
while time.time() - t0 < 4.0:
    if ser.in_waiting:
        raw += ser.read(ser.in_waiting)
    time.sleep(0.05)

for line in raw.decode('utf-8', errors='replace').split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['Identified', 'State:', 'AppFault', 'ThetaDiag',
                                'DirDiag', 'ParamDiag', 'COMPLETE', 'Error',
                                'State:']):
        print(f"  {s}")

ser.close()
