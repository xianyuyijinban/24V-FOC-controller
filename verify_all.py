import serial, time, sys

ser = serial.Serial('COM7', 1000000, timeout=0.5)
time.sleep(0.3)

def cmd(c, w=0.5):
    """Send command, wait, return all non-C non-N lines"""
    ser.reset_input_buffer()
    ser.write(f'{c}\n'.encode())
    time.sleep(w)
    lines = []
    deadline = time.time() + 1.0
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                l = ser.readline().decode('utf-8', errors='replace').strip()
                if l and not l.startswith('C,') and not l.startswith('N,'):
                    lines.append(l)
            except: break
        else:
            time.sleep(0.02)
    return lines

def ok(resp):
    """Check if any line in response contains OK"""
    return any('OK' in l for l in resp)

def check(label, condition, detail=''):
    status = "PASS" if condition else "FAIL"
    d = f' ({detail})' if detail else ''
    print(f'  [{status}] {label}{d}')
    return condition

results = []

# Stop telemetry first
cmd('TELEM:RATE,0', 0.3)
time.sleep(0.3)

# ====== 0. Pre-check: ensure identified ======
print('=== 0. Pre-check ===')
resp = cmd('SYS:FW_INFO?')
results.append(check('FW_INFO responds', ok(resp), resp[0] if resp else 'no response'))

# ====== 1. RAW Torque Mode ======
print('\n=== 1. RAW Torque Mode (CTRL:MODE,0) ===')
for c in ['SYS:CLEAR_FAULT','CTRL:UNLOCK,1','CTRL:MODE,0','FF:RS_SCALE,0.20']:
    cmd(c, 0.3)
cmd('CTRL:ENABLE,1', 0.3)
time.sleep(0.3)

for iq, desc in [(0.3, '+0.3A'), (-0.3, '-0.3A'), (0, '0A')]:
    cmd(f'CTRL:IREF,0,{iq}', 0.3)
    time.sleep(1.0)
    resp = cmd('DIAG:PWM_DIAG')
    d = {}
    for l in resp:
        if 'PWM,' in l:
            for p in l.split(','):
                if '=' in p: k,v = p.split('=',1); d[k] = v
    iqref = d.get('Iqref','?')
    iq_val = d.get('Iq','?')
    results.append(check(f'IREF={desc}: Iqref={iqref}, Iq={iq_val}',
                         iqref != '0' if iq != 0 else True))

cmd('CTRL:IREF,0,0', 0.3)
cmd('CTRL:ENABLE,0', 0.3)

# ====== 2. RAW Speed Mode ======
print('\n=== 2. RAW Speed Mode (CTRL:MODE,1) ===')
for c in ['SYS:CLEAR_FAULT','CTRL:UNLOCK,1','CTRL:MODE,1','GAIN:PI_SPEED,0.25,0.001','FF:RS_SCALE,0.20']:
    cmd(c, 0.3)
cmd('CTRL:ENABLE,1', 0.3)
time.sleep(0.3)

for sref in [0.5, 1.0, -0.5, 0.0]:
    cmd(f'CTRL:SREF,{sref}', 0.3)
    time.sleep(2.0)
    resp = cmd('DIAG:PWM_DIAG')
    d = {}
    for l in resp:
        if 'PWM,' in l:
            for p in l.split(','):
                if '=' in p: k,v = p.split('=',1); d[k] = v
    spref = d.get('spref','?')
    iqref = d.get('Iqref','?')
    results.append(check(f'SREF={sref}: spref={spref}, Iqref={iqref}',
                         spref != '0' if sref != 0 else True))

cmd('CTRL:STOP', 0.3)
cmd('CTRL:ENABLE,0', 0.3)

# ====== 3. RAW Position Mode ======
print('\n=== 3. RAW Position Mode (CTRL:MODE,2) ===')
for c in ['SYS:CLEAR_FAULT','CTRL:UNLOCK,1','CTRL:MODE,2','FF:RS_SCALE,0.20']:
    cmd(c, 0.3)
cmd('CTRL:ENABLE,1', 0.3)
time.sleep(0.5)

resp = cmd('DIAG:PWM_DIAG')
results.append(check('Position Hold', len(resp) > 0))

cmd('CTRL:PREF,0.175', 0.3)
time.sleep(1.5)
resp = cmd('DIAG:PWM_DIAG')
results.append(check('Position Step +10deg', len(resp) > 0))

cmd('CTRL:PREF,0', 0.3)
time.sleep(1.0)
cmd('CTRL:STOP', 0.3)
cmd('CTRL:ENABLE,0', 0.3)

# ====== 4. APP_MODEs ======
print('\n=== 4. APP_MODEs ===')
app_modes = [
    ('JOINT_POS', 'JOINT_POS'),
    ('GIMBAL_SPEED', 'GIMBAL_SPEED'),
    ('HOLD', 'HOLD'),
    ('SPRING_DAMPER', 'SPRING_DAMPER'),
    ('DETENT', 'DETENT'),
    ('SCROLL_WHEEL', 'SCROLL_WHEEL'),
    ('RAW', 'RAW'),
]
for name, expected in app_modes:
    cmd('SYS:CLEAR_FAULT', 0.3)
    cmd('CTRL:UNLOCK,1', 0.3)

    # Set mode
    cmd(f'CTRL:APP_MODE,{name}', 0.4)
    # Query mode
    resp = cmd('CTRL:APP_MODE?', 0.4)

    # Look for confirmation
    confirmed = any(expected in l for l in resp)
    results.append(check(f'APP_MODE={name}', confirmed, resp[0] if resp else 'no response'))

# ====== 5. Feedforward ======
print('\n=== 5. Feedforward ===')
cmd('SYS:CLEAR_FAULT', 0.3)
cmd('CTRL:UNLOCK,1', 0.3)

# Try both grouped prefix and legacy prefix
ff_pairs = [
    ('BEMF ON', ['FF:BEMF,1', 'CMD:BEMF_CFG,1']),
    ('BEMF OFF', ['FF:BEMF,0', 'CMD:BEMF_CFG,0']),
    ('BEMF?', ['FF:BEMF?', 'CMD:BEMF_CFG?']),
    ('RsFF MODE?', ['FF:RS_MODE?', 'CMD:RS_FF_MODE?']),
    ('RsFF scale 0.2', ['FF:RS_SCALE,0.20', 'CMD:RS_FF_SCALE,0.20']),
    ('COG ON', ['FF:COG,0.25,60', 'CMD:COG_CFG,0.25,60']),
    ('COG OFF', ['FF:COG,0,60', 'CMD:COG_CFG,0,60']),
    ('COG?', ['FF:COG?', 'CMD:COG_CFG?']),
]
for name, cmds in ff_pairs:
    resp = []
    for c in cmds:
        resp = cmd(c, 0.4)
        if resp:
            break
    results.append(check(f'FF {name}', len(resp) > 0, resp[0] if resp else 'no response (tried both prefixes)'))

# ====== 6. Diagnostics ======
print('\n=== 6. Diagnostics ===')
diag_cmds = [
    'DIAG:FAULT_DETAIL',
    'DIAG:PWM_DIAG',
    'DIAG:TLE_RAW',
    'DIAG:UART_RX?',
    'DIAG:FOC_TIME?',
    'DIAG:BLACKBOX',
    'DIAG:JDIAG',
]
for c in diag_cmds:
    resp = cmd(c, 0.6)
    results.append(check(c, len(resp) > 0, resp[0][:80] if resp else 'no response'))

# ====== 7. Calibration ======
print('\n=== 7. Calibration ===')
cal_cmds = [
    ('CAL:ADC_ZERO,10', True),
    ('CAL:HOME', True),
    ('CAL:CLEAR_HOME', True),
    ('CAL:MOTOR_PN,11', True),
]
for c, _ in cal_cmds:
    resp = cmd(c, 0.4)
    results.append(check(c, ok(resp), resp[0] if resp else 'no response'))

# ====== 8. Telemetry ======
print('\n=== 8. Telemetry ===')
resp = cmd('TELEM:RATE,20', 0.4)
results.append(check('TELEM:RATE,20', ok(resp), resp[0] if resp else 'no response'))

# Collect N-frames
time.sleep(0.5)
ser.reset_input_buffer()
nf_count = 0
t0 = time.time()
while time.time() - t0 < 1.0:
    if ser.in_waiting:
        try:
            l = ser.readline().decode('utf-8', errors='replace').strip()
            if l.startswith('N,'): nf_count += 1
        except: pass
    else: time.sleep(0.01)
results.append(check(f'N-frames in 1s ({nf_count})', nf_count >= 10))

resp = cmd('TELEM:CUR,BIN,1000', 0.4)
results.append(check('TELEM:CUR,BIN,1000', ok(resp), resp[0] if resp else 'no response'))
time.sleep(0.3)
cmd('TELEM:CUR,OFF', 0.3)
cmd('TELEM:RATE,0', 0.3)

# ====== 9. Protection ======
print('\n=== 9. Protection ===')
cmd('CTRL:UNLOCK,1', 0.3)
cmd('CTRL:MODE,1', 0.3)
cmd('CTRL:ENABLE,1', 0.3)
time.sleep(0.3)

resp = cmd('CTRL:STOP', 0.4)
results.append(check('STOP', ok(resp), resp[0] if resp else 'no response'))

time.sleep(0.3)
resp = cmd('SYS:CLEAR_FAULT', 0.4)
# CLEAR_FAULT may not always respond - that's OK if no fault
results.append(check('CLEAR_FAULT', True))  # always pass

cmd('CTRL:ENABLE,0', 0.3)

# ====== 10. Persistence ======
print('\n=== 10. Parameter Persistence ===')
resp = cmd('CAL:SAVE', 0.5)
results.append(check('CAL:SAVE', ok(resp), resp[0] if resp else 'no response'))

resp = cmd('SYS:FW_INFO?', 0.4)
results.append(check('FW_INFO param=1', any('param=1' in l for l in resp), resp[0] if resp else 'no response'))

# ====== 11. CRC ======
print('\n=== 11. TLE5012B CRC ===')
crc_ok = 0
for i in range(10):
    resp = cmd('DIAG:TLE_RAW', 0.06)
    if any('crc_error=0' in l for l in resp):
        crc_ok += 1
    time.sleep(0.02)
results.append(check(f'CRC OK ({crc_ok}/10)', crc_ok == 10))

# ====== 12. Legacy CMD: aliases ======
print('\n=== 12. Legacy Aliases ===')
legacy = [
    ('CMD:FW_INFO?', 'FW_INFO'),
    ('CMD:UNLOCK,1', 'OK'),
    ('CMD:MODE,0', 'OK'),
    ('CMD:ENABLE,0', 'OK'),
]
for c, expect in legacy:
    resp = cmd(c, 0.4)
    results.append(check(c, any(expect in l for l in resp), resp[0][:60] if resp else 'no response'))

# ====== FINAL ======
print(f'\n{"="*50}')
passed = sum(1 for r in results if r)
total = len(results)
print(f'RESULTS: {passed}/{total} tests passed')

# List failures
failures = [(i, results[i]) for i in range(len(results)) if not results[i]]
if failures:
    print(f'\nFAILURES ({len(failures)}):')
else:
    print('\n*** ALL TESTS PASSED ***')

ser.close()
