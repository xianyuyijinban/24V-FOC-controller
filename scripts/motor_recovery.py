"""Quick motor parameter recovery: clear fault, run IDENTIFY, verify"""
import serial, time, sys

s = serial.Serial('COM9', 1152000, timeout=0.3)
time.sleep(1.0)
s.reset_input_buffer()

# Clear fault, set params, unlock
for cmd in ['CMD:CLEAR_FAULT', 'CMD:VBUS_LIMIT,8,15', 'CMD:UNLOCK,1']:
    s.write((cmd + '\r\n').encode())
    time.sleep(0.3)
    s.read(s.in_waiting or 65536)

time.sleep(0.5)

# Check encoder
s.write(b'CMD:TLE_STATUS\r\n')
time.sleep(0.5)
data = s.read(s.in_waiting or 65536).decode('ascii', errors='replace')
print('=== TLE_STATUS ===')
for l in data.split('\n')[:8]:
    l = l.strip()
    if l:
        print(l[:250])

# Start IDENTIFY
print('\n=== Starting IDENTIFY (waiting up to 60s) ===')
s.write(b'CMD:IDENTIFY,1\r\n')

identified = False
for i in range(60):
    time.sleep(1.0)
    data = s.read(s.in_waiting or 65536).decode('ascii', errors='replace')
    for line in data.split('\n'):
        line = line.strip()
        if not line.startswith('N,'):
            continue
        parts = line.split(',')
        if len(parts) < 26:
            continue
        state = parts[2]
        id_state = parts[25]
        motor_id = parts[10]
        if state == '0' and motor_id == '1':
            print(f'[{i}s] SUCCESS! State=READY, motorIdentified=YES, idState={id_state}')
            identified = True
            break
        elif state == '3':
            print(f'[{i}s] FAULT! idState={id_state}')
            break
        elif state == '2' and i % 5 == 0:
            print(f'[{i}s] IDENTIFY running, subState={id_state}')
    if identified or (state == '3'):
        break

if not identified:
    print('FAILED: Motor identification did not complete')
    s.write(b'CMD:FAULT_DETAIL\r\n')
    time.sleep(0.5)
    data = s.read(s.in_waiting or 65536).decode('ascii', errors='replace')
    print('\n=== FAULT_DETAIL ===')
    for l in data.split('\n')[:15]:
        print(l.strip()[:250])

s.close()
