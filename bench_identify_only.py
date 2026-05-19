"""
Focused test: Section 7 + 8 only
Save FULL FAULT_DETAIL text, fix CSV mapping
"""
import sys, io, time, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
os.makedirs('test_data', exist_ok=True)

import serial
s = serial.Serial('COM15', 230400, timeout=0.1)
time.sleep(1)
s.read(5000)

def drain():
    data = b''
    while s.in_waiting:
        data += s.read(s.in_waiting)
        time.sleep(0.02)
    return data

def read_raw(duration_ms):
    data = b''
    deadline = time.time() + duration_ms / 1000
    while time.time() < deadline:
        if s.in_waiting:
            data += s.read(s.in_waiting)
        else:
            time.sleep(0.01)
    return data

# Log raw N packets to CSV with correct field mapping
def log_csv(filename, raw_data, max_lines=300):
    lines = raw_data.decode('utf-8', errors='replace').strip().split('\n')
    n_lines = []
    for l in lines:
        if l.startswith('N,') or l.startswith('F,'):
            n_lines.append(l)
    
    path = f'test_data/{filename}'
    with open(path, 'w') as f:
        f.write('timestamp_ms,angle,speed,speed_ref,pos_ref_deg,Vbus,Ia,Ib,Ic,Id,Iq,Id_ref,Iq_ref,Vd,Vq,foc_state,fault_flags\n')
        count = 0
        for l in n_lines[:max_lines]:
            parts = l.split(',')
            if len(parts) < 12:
                continue
            pfx = parts[0]
            ts = parts[1]
            st = parts[2]
            ang = parts[3]
            spd = parts[4]
            idq = parts[5]
            iqq = parts[6]
            vb = parts[7]
            ff = parts[8]
            # Extended fields if available
            if pfx == 'N' and len(parts) >= 22:
                # N,timestamp,foc_state,angle,speed,Id,Iq,vbus,fault_flags,enc_detected,ident,stall_armed,stall_active,app_warn,app_fault,ctrl_mode,Id_ref,speed_ref,pos_ref,Iq_ref,Vd,Vq,...
                ia = parts[22] if len(parts) > 22 else ''
                ib = parts[23] if len(parts) > 23 else ''
                ic = parts[24] if len(parts) > 24 else ''
                id_ref = parts[16] if len(parts) > 16 else ''
                iq_ref = parts[19] if len(parts) > 19 else ''
                spd_ref = parts[17] if len(parts) > 17 else ''
                pos_ref = parts[18] if len(parts) > 18 else ''
                vd = parts[20] if len(parts) > 20 else ''
                vq = parts[21] if len(parts) > 21 else ''
            else:
                ia = ib = ic = id_ref = iq_ref = spd_ref = pos_ref = vd = vq = ''
            f.write(f'{ts},{ang},{spd},{spd_ref},{pos_ref},{vb},{ia},{ib},{ic},{idq},{iqq},{id_ref},{iq_ref},{vd},{vq},{st},{ff}\n')
            count += 1
    print(f'  [CSV] {path} ({count} lines)')
    return path


# ===== Section 7: Config =====
print('='*60)
print('Section 7: Parameter Configuration')
print('='*60)

drain()
s.write(b'CMD:VBUS_LIMIT,9.000,16.000\n')
time.sleep(0.2)
s.write(b'CMD:MOTOR_PN,11\n')
time.sleep(0.2)
s.write(b'CMD:CLEAR_FAULT\n')
time.sleep(0.5)
raw = drain()
text = raw.decode('utf-8', errors='replace')

print('After config + CLEAR:')
for l in text.strip().split('\n')[-5:]:
    l = l.strip()
    if l.startswith('N,'):
        parts = l.split(',')
        print(f'  foc={parts[2]} Vbus={parts[7]}V ident={parts[10]} stall={parts[11]} app_fault={parts[14]} uv={parts[-2]} ov={parts[-1]}')

log_csv('s7_config.csv', raw)

# ===== Section 8: Identification =====
print()
print('='*60)
print('Section 8: Motor Identification')
print('='*60)

drain()
s.write(b'CMD:UNLOCK,1\n')
time.sleep(0.3)

ident_raw = b''
ident_start = time.time()

s.write(b'CMD:IDENTIFY,1\n')

# Wait up to 120s for identify to complete or fault
ident_done = False
for i in range(120):
    time.sleep(1)
    chunk = read_raw(100)
    ident_raw += chunk
    
    text = chunk.decode('utf-8', errors='replace')
    
    # Check for completion
    if 'FAULT DETECTED' in text or 'MI_ERR' in text:
        print(f'  [FAULT] at t={i+1}s')
        ident_done = True
        break
    
    # Check raw N packets for ident status
    for line in text.split('\n'):
        if line.startswith('N,'):
            parts = line.split(',')
            if len(parts) > 10 and parts[10] == '1':
                print(f'  [IDENTIFY SUCCESS] at t={i+1}s')
                ident_done = True
                break
    
    if ident_done:
        break
    
    if i % 10 == 0 and i > 0:
        print(f'  ...waiting ({i+1}s)')

# Collect remaining data
time.sleep(0.5)
ident_raw += read_raw(1000)

# Save raw telemetry CSV
log_csv('s8_identify.csv', ident_raw)

# ===== CRITICAL: Save FULL FAULT_DETAIL =====
print()
print('='*60)
print('FAULT_DETAIL capture')
print('='*60)

drain()
s.write(b'CMD:FAULT_DETAIL\n')
time.sleep(1.5)
fault_raw = read_raw(2000)

# Save COMPLETE raw text to file
fault_path = 'test_data/s8_fault_detail_complete.txt'
with open(fault_path, 'wb') as f:
    f.write(fault_raw)
print(f'  [RAW] {fault_path} ({len(fault_raw)} bytes)')

# Print the full text for verification
fault_text = fault_raw.decode('utf-8', errors='replace')
print()
print('=== FULL FAULT_DETAIL TEXT ===')
print(fault_text)
print('=== END ===')

# ===== VERIFY RsDiag contains I_mag_avg =====
print()
print('='*60)
print('RsDiag Verification')
print('='*60)

if 'RsDiag:' in fault_text:
    rsdiag_line = ''
    for l in fault_text.split('\n'):
        if 'RsDiag:' in l:
            rsdiag_line = l.strip()
            break
    
    has_imag_avg = 'I_mag_avg' in fault_text
    print(f'  RsDiag line: {rsdiag_line}')
    print(f'  Contains I_mag_avg: {has_imag_avg}')
    
    if has_imag_avg:
        print('  [VALID] Firmware has I_mag_avg field')
    else:
        print('  [INVALID] Firmware lacks I_mag_avg field — not latest version')
else:
    print('  [WARN] No RsDiag found in fault detail')

# Also extract PnDiag and DirDiag
for line in fault_text.split('\n'):
    if 'PnDiag:' in line:
        print(f'  {line.strip()}')
    if 'DirDiag:' in line:
        print(f'  {line.strip()}')

s.close()
print()
print('Done.')
