import serial, time, re

ser = serial.Serial('COM9', 230400, timeout=0.3)
time.sleep(0.3)

def send(c):
    ser.write((c + '\n').encode())

def capture():
    ser.reset_input_buffer()
    time.sleep(0.2)
    send('CMD:FAULT_DETAIL')
    time.sleep(2.5)
    buf = b''
    s = time.time()
    while time.time() - s < 4.0:
        x = ser.read(8192)
        if x:
            buf += x
    return buf.decode('utf-8', errors='replace')

def parse(text):
    r = {}
    for l in text.split('\n'):
        if 'PositionLoopDiag' in l:
            for k in ['err', 'iq_cmd']:
                m = re.search(rf'{k}=([\d.\-]+)', l)
                if m:
                    r[k] = float(m.group(1))
        if 'TrajDiag' in l:
            m = re.search(r'active=(\d)', l)
            if m:
                r['traj'] = int(m.group(1))
        if 'AppFault' in l:
            m = re.search(r'AppFault:\s*(\d+)', l)
            if m:
                r['fault'] = int(m.group(1))
    return r

send('CMD:CLEAR_FAULT'); time.sleep(0.5)
send('CMD:PI_CURRENT,0.030000,0.500000'); time.sleep(0.1)
send('CMD:PI_SPEED,0.300000,0.000000'); time.sleep(0.1)
send('CMD:PD_POS,2.000000,0.080000'); time.sleep(0.1)
send('CMD:MODE,2'); time.sleep(0.1)
send('CMD:PREF,0.000'); time.sleep(0.1)
send('CMD:UNLOCK,1'); time.sleep(0.3)
send('CMD:ENABLE,1'); time.sleep(2.0)
print('V4 Endurance: 0<->80 deg x 50, 3s dwell')
t0 = time.time()
errors = []
traj_hits = 0
faults = 0

for i in range(50):
    send('CMD:PREF,1.396')
    time.sleep(1.5)
    send('CMD:PREF,0.000')
    time.sleep(1.5)
    if i % 10 == 9:
        d = parse(capture())
        e = abs(d.get('err', 99)) * 57.3
        errors.append(e)
        if d.get('traj', 0) == 1:
            traj_hits += 1
        if d.get('fault', 0) != 0:
            faults += 1
        print(f'  cycle {i+1:2d}/50  err={e:.1f} deg  traj={d.get("traj","?")}  iq={d.get("iq_cmd",0):+.4f}A  fault={d.get("fault","?")}  t={time.time()-t0:.0f}s')

time.sleep(2.0)
d = parse(capture())
fe = abs(d.get('err', 99)) * 57.3
print(f'\nFinal: err={fe:.1f} deg  faults={faults}  traj_hits={traj_hits}  elapsed={time.time()-t0:.0f}s')
if errors:
    print(f'Max err: {max(errors):.1f} deg  Avg err: {sum(errors)/len(errors):.1f} deg')

send('CMD:ENABLE,0')
ser.close()

all_ok = (max(errors) if errors else 0) < 3.0 and faults == 0
print(f'V4 Endurance: {"PASS" if all_ok else "CHECK"}')
