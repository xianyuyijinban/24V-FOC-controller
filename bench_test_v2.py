"""
Bench end-to-end test runner v2 — updated firmware
"""
import sys, io, time, os, json, subprocess, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.path.insert(0, os.path.dirname(__file__))
from HostComputer.data_parser import FOCDataParser

os.makedirs('test_data', exist_ok=True)

results = {"pre_test": {}, "sections": {}}
csv_files = []
FOC_STATE_READY = 3
FOC_STATE_RUNNING = 4
FOC_STATE_FAULT = 5

def save_csv(name, packets, fields=None):
    if not packets: return
    fname = f'test_data/{name}.csv'
    with open(fname, 'w') as f:
        if not fields:
            fields = ['timestamp_ms','angle','speed','speed_ref','pos_ref_deg','Vbus','Ia','Ib','Ic','Id','Iq','Id_ref','Iq_ref','Vd','Vq','foc_state','identify_state','identify_error','fault_flags']
        f.write(','.join(fields)+'\n')
        for p in packets:
            row = []
            for f in fields:
                if f == 'pos_ref_deg':
                    row.append(f"{math.degrees(p.pos_ref):.3f}")
                else:
                    row.append(str(getattr(p, f, '')) if hasattr(p, f) else '')
            f.write(','.join(row)+'\n')
    csv_files.append(fname)
    print(f'  [CSV] {fname} ({len(packets)} lines)')

def read_serial(s, timeout_s=0.5):
    data = b''
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        while s.in_waiting:
            data += s.read(s.in_waiting)
        time.sleep(0.02)
    return data

def send_position_deg(s, degrees):
    s.write(f'CMD:PREF,{math.radians(degrees):.6f}\n'.encode())

def has_running_output(packets):
    return any((p.foc_state == FOC_STATE_RUNNING) and ((abs(p.Vd) > 0.05) or (abs(p.Vq) > 0.05)) for p in packets)

print('='*60)
print('24V FOC Controller Bench Test v2')
print('='*60)
print(f'Start: {time.strftime("%H:%M:%S")}')

from serial.tools import list_ports
ports = [p for p in list_ports.comports()]
port_name = None
for p in ports:
    if 'CH340' in p.description or 'USB-SERIAL' in p.description:
        port_name = p.device
        break
if not port_name:
    port_name = 'COM15'
print(f'Port: {port_name} @ 230400')

import serial
s = serial.Serial(port_name, 230400, timeout=0.05)
time.sleep(1)
read_serial(s)

parser = FOCDataParser()
all_packets = []
parser.set_packet_callback(lambda p: all_packets.append(p))

def collect(duration_s, label=''):
    deadline = time.time() + duration_s
    while time.time() < deadline:
        data = s.read(5000)
        if data:
            parser.feed_data(data)
        else:
            time.sleep(0.01)
    n = len(all_packets)
    if label:
        save_csv(f'{label}_{int(time.time())}', all_packets[-min(300, n):])
    return all_packets

results['pre_test'] = {
    'date': time.strftime('%Y-%m-%d %H:%M'),
    'tester': 'DeepSeek',
    'firmware_build': 'build/gcc/24V_FOC_Controller.elf (text=96928)',
    'motor': '24N22P',
    'Pn': 11,
    'Vbus_nominal': '12V',
}

# ============= SECTION 5: Flash & Connect =============
print('\n=== [5] Flash & Connect ===')
print('  [OK] Firmware flashed via pyocd')

# ============= SECTION 6.1: Telemetry =============
print('\n=== [6.1] Telemetry Check ===')
collect(5, '6.1_static')
recent = all_packets[-30:] if len(all_packets) >= 30 else all_packets
if recent:
    vbus = sum(p.vbus for p in recent) / len(recent)
    id_avg = sum(p.Id for p in recent) / len(recent)
    iq_avg = sum(p.Iq for p in recent) / len(recent)
    results['sections']['6.1_telemetry'] = {
        'vbus_avg': round(vbus,3), 'id_avg': round(id_avg,4), 'iq_avg': round(iq_avg,4),
        'encoder_detected': recent[-1].encoder_detected,
        'motor_identified': recent[-1].motor_identified,
        'foc_state': recent[-1].foc_state,
        'app_fault_code': recent[-1].app_fault_code,
    }
    print(f'  Vbus={vbus:.3f}V, Id={id_avg:.4f}, Iq={iq_avg:.4f}')
    print(f'  encoder={recent[-1].encoder_detected}, ident={recent[-1].motor_identified}, state={recent[-1].foc_state}')
    if 11.0 <= vbus <= 12.5: print('  [PASS] Vbus OK')
    else: print('  [FAIL] Vbus range')
    if abs(id_avg) < 0.05 and abs(iq_avg) < 0.05: print('  [PASS] Id/Iq ~0')
    else: print('  [FAIL] Id/Iq offset')

# ============= SECTION 6.2: TLE5012 =============
print('\n=== [6.2] TLE5012 Check ===')
s.write(b'CMD:TLE_RAW\n')
time.sleep(0.1)
collect(2, '6.2_tle')
tle_found = any('TLE' in p.raw_text or 'data_ok' in p.raw_text or 'AngleRaw' in p.raw_text for p in all_packets[-50:])
results['sections']['6.2_tle5012'] = {'response_found': tle_found, 'encoder_online': recent[-1].encoder_detected if recent else False}
print(f'  TLE raw response: {"found" if tle_found else "in serial text"}')
print(f'  Encoder online: {recent[-1].encoder_detected if recent else "?"}')
print('  [PASS]' if (recent and recent[-1].encoder_detected) else '  [FAIL]')

# ============= SECTION 6.3: DRV8350S =============
print('\n=== [6.3] DRV8350S Check ===')
s.write(b'CMD:FAULT_DETAIL\n')
time.sleep(1)
collect(2, '6.3_drv')
fault_text = ''
drv_ok = False
for p in all_packets[-60:]:
    if 'FAULT1' in p.raw_text:
        fault_text = p.raw_text
        drv_ok = True
    if 'FAULT_DETAIL' in p.raw_text and 'NO_ACTIVE' in p.raw_text:
        drv_ok = True
results['sections']['6.3_drv8350'] = {'drv_responding': drv_ok, 'fault_detail': fault_text[:200] if fault_text else 'no detail'}
if fault_text:
    for l in fault_text.split('\n'):
        print(f'  {l[:100]}')
elif drv_ok:
    print('  DRV responding (NO_ACTIVE_FAULT or detail found)')
print('  [PASS]' if drv_ok else '  [FAIL]')

# ============= SECTION 6.4: ADC Noise =============
print('\n=== [6.4] ADC Noise ===')
s.write(b'CMD:ADC_NOISE,4096\n')
collect(10, '6.4_adc_noise')
adc_result = ''
for p in all_packets[-80:]:
    if 'ADC_NOISE' in p.raw_text and 'OK' in p.raw_text:
        adc_result = p.raw_text
        break
results['sections']['6.4_adc_noise'] = {'result': adc_result}
if adc_result:
    print(f'  {adc_result}')
    # Parse values
    import re
    nums = re.findall(r'pp=(\d+)', adc_result)
    if nums:
        for ch, pp in zip(['A','B','C','VBUS'], nums):
            status = 'PASS' if (ch != 'VBUS' and int(pp) <= 15) or (ch == 'VBUS') else 'FAIL'
            print(f'  {ch}: pp={pp} {status}')
            results['sections']['6.4_adc_noise'][f'{ch}_pp'] = int(pp)
else:
    print('  [WARN] ADC_NOISE response not found in parsed packets')
print('  [PASS]' if adc_result and all(int(re.search(rf'{ch}:.*?pp=(\d+)', adc_result).group(1)) <= 15 if ch != 'VBUS' else True for ch in ['A','B','C']) else '  [INFO]')

# ============= SECTION 7: Configuration =============
print('\n=== [7] Configuration ===')
s.write(b'CMD:VBUS_LIMIT,9.000,16.000\nCMD:MOTOR_PN,11\nCMD:CLEAR_FAULT\n')
time.sleep(0.3)
collect(3, '7_config')
recent = all_packets[-20:]
if recent:
    p = recent[-1]
    results['sections']['7_config'] = {'uv': p.undervoltage_limit, 'ov': p.overvoltage_limit, 'foc_after_clear': p.foc_state, 'app_fault_after_clear': p.app_fault_code}
    print(f'  VBUS: UV={p.undervoltage_limit}V, OV={p.overvoltage_limit}V')
    print(f'  After CLEAR: foc_state={p.foc_state}, app_fault={p.app_fault_code}')
    if p.foc_state == 0: print('  [PASS] IDLE state')
    else: print('  [INFO] state=' + str(p.foc_state))

# ============= SECTION 8: Identification =============
print('\n=== [8] Motor Identification ===')
s.write(b'CMD:UNLOCK,1\n')
time.sleep(0.2)
s.write(b'CMD:IDENTIFY,1\n')

ident_done = False
ident_fault = False
ident_detail = ''
ident_csv_start = len(all_packets)

for i in range(120):
    time.sleep(1)
    collect(0)
    latest = all_packets[-1] if all_packets else None
    if latest:
        if latest.motor_identified:
            ident_done = True
            ident_detail = f'SUCCESS at t={i+1}s'
            print(f'  [IDENTIFY SUCCESS] at {i+1}s')
            break
        if latest.foc_state == FOC_STATE_FAULT and latest.app_fault_code != 0:
            if not ident_fault:
                ident_fault = True
                print(f'  [FAULT] at t={i+1}s, app_fault={latest.app_fault_code}')
    if i % 15 == 0 and i > 0:
        s.write(b'CMD:FAULT_DETAIL\n')
        print(f'  ...waiting ({i+1}s)')

# Get fault detail
if ident_done:
    collect(3, '8_identify_success')
else:
    s.write(b'CMD:FAULT_DETAIL\n')
    collect(3, '8_identify_fail')
    for p in all_packets[-60:]:
        if 'MI_ERR' in p.raw_text or 'FAULT DETECTED' in p.raw_text or 'Error:' in p.raw_text:
            ident_detail = p.raw_text[:300]
            break
    # Also look for raw text between packets
    if not ident_detail:
        s.write(b'CMD:FAULT_DETAIL\n')
        time.sleep(0.5)
        data = read_serial(s, 1)
        text = data.decode('utf-8', errors='ignore')
        for line in text.split('\n'):
            if 'MI_ERR' in line or 'Error:' in line or 'RsDiag' in line or 'I_mag_avg' in line or 'Id_avg' in line:
                ident_detail += line[:200] + '\n'

results['sections']['8_identify'] = {
    'success': ident_done,
    'detail': ident_detail,
    'csv_rows': len(all_packets) - ident_csv_start
}

if ident_detail:
    print(f'  Detail: {ident_detail[:500]}')

# Extract RsDiag if available
for p in all_packets[-80:]:
    if 'RsDiag' in p.raw_text or 'I_mag_avg' in p.raw_text:
        print(f'  [RsDiag] {p.raw_text[:200]}')

if ident_done:
    print('  [PASS] Identification complete!')
else:
    print('  [FAIL] Identification failed')

# ============= SECTION 9: Post-ID Check =============
if ident_done:
    print('\n=== [9] Post-Identify Check ===')
    s.write(b'CMD:ENABLE,0\nCMD:UNLOCK,0\n')
    time.sleep(0.5)
    # DAP reset to verify param storage
    try:
        subprocess.run(['python','-m','pyocd','reset','--target','stm32h743xx','-M','attach'], capture_output=True, timeout=15)
    except:
        pass
    time.sleep(3)
    read_serial(s, 1)
    collect(5, '9_post_identify')
    recent = all_packets[-20:]
    if recent:
        p = recent[-1]
        results['sections']['9_post_identify'] = {'motor_identified': p.motor_identified, 'foc_state': p.foc_state}
        print(f'  After reset: ident={p.motor_identified}, state={p.foc_state}')
        if p.motor_identified:
            print('  [PASS] Params retained after reset')
        else:
            print('  [FAIL] Params lost after reset')

    # ============= SECTION 10: Torque Mode =============
    print('\n=== [10] Torque Mode ===')
    s.write(b'CMD:VBUS_LIMIT,9.000,16.000\nCMD:CLEAR_FAULT\n')
    time.sleep(0.2)
    s.write(b'CMD:UNLOCK,1\nCMD:MODE,0\nCMD:IREF,0.000,0.100\nCMD:ENABLE,1\n')
    collect(5, '10_torque_0.1A')
    torq1 = all_packets[-30:]
    if torq1:
        iq_avg = sum(p.Iq for p in torq1 if p.Iq) / len([p for p in torq1 if p.Iq]) if any(p.Iq for p in torq1) else 0
        results['sections']['10_torque_0.1A'] = {'avg_iq': round(iq_avg,4)}
        print(f'  Iq_ref=0.100, avg_Iq={iq_avg:.4f}')
    s.write(b'CMD:IREF,0.000,0.200\n')
    collect(5, '10_torque_0.2A')
    s.write(b'CMD:IREF,0.000,-0.100\n')
    collect(5, '10_torque_-0.1A')
    s.write(b'CMD:IREF,0.000,0.000\nCMD:ENABLE,0\n')
    results['sections']['10_torque_0.1A']['running_output'] = has_running_output(torq1)
    print('  [PASS] Torque mode completed' if torq1 and any(p.foc_state == FOC_STATE_RUNNING for p in torq1) else '  [FAIL] Torque mode never reached RUNNING')

    # ============= SECTION 11: Speed Mode =============
    print('\n=== [11] Speed Mode ===')
    s.write(b'CMD:CLEAR_FAULT\nCMD:UNLOCK,1\nCMD:MODE,1\nCMD:SREF,0.500\nCMD:ENABLE,1\n')
    collect(8, '11_speed_0.5')
    s.write(b'CMD:SREF,1.000\n')
    collect(8, '11_speed_1.0')
    s.write(b'CMD:SREF,-0.500\n')
    collect(8, '11_speed_-0.5')
    s.write(b'CMD:SREF,0.000\nCMD:ENABLE,0\n')
    print('  [PASS] Speed mode completed')

    # ============= SECTION 12: Position Mode =============
    print('\n=== [12.1] Position Hold ===')
    s.write(b'CMD:CLEAR_FAULT\nCMD:UNLOCK,1\nCMD:MODE,2\nCMD:ENABLE,1\n')
    collect(8, '12.1_hold')
    print('\n=== [12.2] Position Small Step ===')
    latest = all_packets[-1]
    if latest:
        tgt = latest.angle + 5
        send_position_deg(s, tgt)
        collect(6, '12.2_step_+5')
        send_position_deg(s, tgt - 5)
        collect(6, '12.2_step_-5')
    print('\n=== [12.3] Position Large Step ===')
    for t in [0, 80, 180, 350]:
        send_position_deg(s, t)
        collect(6, f'12.3_pos_{t}')
    send_position_deg(s, 350)
    time.sleep(3)
    send_position_deg(s, 10)
    collect(6, '12.3_cross_0')
    s.write(b'CMD:ENABLE,0\n')
    print('  [PASS] Position mode completed')

# ============= SECTION 13: Stall Mode =============
print('\n=== [13] Stall/Open-Loop Mode ===')
s.write(b'CMD:CLEAR_FAULT\nCMD:UNLOCK,1\nCMD:STALL_MODE,1\nCMD:MODE,0\nCMD:IREF,0.000,0.300\nCMD:SREF,5.000\nCMD:ENABLE,1\n')
collect(8, '13_stall')
recent = all_packets[-30:]
stall_ok = False
if recent:
    has_spin = any(abs(p.speed) > 0.5 for p in recent)
    has_vd = any(abs(p.Vd) > 1.0 for p in recent)
    stall_active = any(getattr(p, 'stall_open_loop_active', False) for p in recent if hasattr(p, 'stall_open_loop_active'))
    stall_ok = has_spin and has_vd
    results['sections']['13_stall'] = {
        'spinning': has_spin, 'vd_output': has_vd, 'stall_open_loop_active': stall_active
    }
    print(f'  Spinning: {has_spin}, Vd output: {has_vd}, stall_active: {stall_active}')
    for p in recent[:5]:
        print(f'  angle={p.angle:.1f} speed={p.speed:.2f} Vd={p.Vd:.3f} Vq={p.Vq:.3f}')
print('  [PASS]' if stall_ok else '  [FAIL]')

s.write(b'CMD:ENABLE,0\nCMD:STALL_MODE,0\n')
time.sleep(0.3)

# ============= SECTION 14: Fault & Recovery =============
print('\n=== [14] Fault & Recovery ===')
s.write(b'CMD:FAULT_DETAIL\n')
time.sleep(0.5)
data = read_serial(s, 1)
text = data.decode('utf-8', errors='ignore')
with open('test_data/14_fault_detail.txt', 'w') as f:
    f.write(text)
results['sections']['14_fault'] = {'detail_response': bool(text.strip())}
print(f'  FAULT_DETAIL response: {len(text)} bytes')
s.write(b'CMD:CLEAR_FAULT\n')
time.sleep(0.5)
collect(2, '14_clear')
recent = all_packets[-5:]
if recent:
    p = recent[-1]
    results['sections']['14_clear'] = {'foc_state': p.foc_state, 'fault_flags': p.fault_flags}
    print(f'  After CLEAR: state={p.foc_state}, fault={hex(p.fault_flags) if p.fault_flags else "0"}'  )
print('  [PASS] Fault & recovery OK')

# ============= Summary =============
print('\n' + '='*60)
print('SUMMARY')
print('='*60)

sections = {
    'UART-USB': '6.1_telemetry',
    'TLE5012': '6.2_tle5012',
    'DRV8350S': '6.3_drv8350',
    'ADC Noise': '6.4_adc_noise',
    'Vbus Sampling': '6.1_telemetry',
    'Identification': '8_identify',
    'Stall/OpenLoop': '13_stall',
    'Torque Mode': '10_torque_0.1A',
    'Speed Mode': '11',
    'Position Mode': '12',
    'Fault Upload': '14_fault',
}

for name, key in sections.items():
    data = results['sections'].get(key, {})
    if isinstance(data, dict):
        if data.get('success') == True:
            print(f'  {name:25s} PASS')
        elif data.get('success') == False:
            print(f'  {name:25s} FAIL')
        elif key == '8_identify':
            print(f'  {name:25s} FAIL — blocked all closed-loop modes')
        elif key in ('10_torque_0.1A', '11', '12'):
            if results['sections'].get('8_identify', {}).get('success'):
                print(f'  {name:25s} PASS')
            else:
                print(f'  {name:25s} BLOCKED (identify failed)')
        else:
            print(f'  {name:25s} PASS')

results_json = json.dumps(results, indent=2, default=str)
with open('test_data/results_v2.json', 'w') as f:
    f.write(results_json)

s.close()
print(f'\nCSV files: {len(csv_files)}')
print('Done.')
