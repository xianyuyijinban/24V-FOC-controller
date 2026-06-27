"""
12V Load Margin — clean protocol
Each point: empty 3s settle → capture 1s → LOAD prompt → load 3s → capture 1s → release → zero return
"""
import serial, time, sys, statistics, json
from datetime import datetime

PORT='COM9'; BAUD = 1152000
RESULTS = []

s = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
s.read(s.in_waiting or 65536)

def cmd(c, w=0.3):
    s.reset_input_buffer()
    s.write((c+'\r\n').encode())
    time.sleep(w)
    return s.read(s.in_waiting or 65536).decode('ascii', errors='replace')

def capture(dur=1.0):
    s.reset_input_buffer()
    time.sleep(dur)
    data = s.read(s.in_waiting or 65536).decode('ascii', errors='replace')
    frames=[]
    for line in data.split('\n'):
        line=line.strip()
        if line.startswith('N,'):
            parts=line.split(',')
            if len(parts)>=25:
                try:
                    frames.append({
                        'ts':int(parts[1]),'state':int(parts[2]),
                        'speed':float(parts[4]),'Iq':float(parts[6]),
                        'fault':parts[8].strip(),'Vq':float(parts[20]),
                        'Vbus':float(parts[7]),'Vd':float(parts[19]),
                        'ctrl_mode':int(parts[15]),
                    })
                except: pass
    return frames

def stats(frames, label=""):
    if not frames: return {}
    sp=[f['speed'] for f in frames]; iq=[f['Iq'] for f in frames]
    vq=[f['Vq'] for f in frames]; vb=[f['Vbus'] for f in frames]
    return {
        'n':len(frames),
        'w_mean':statistics.mean(sp), 'w_std':statistics.stdev(sp) if len(sp)>1 else 0,
        'Iq_mean':statistics.mean(iq), 'Iq_peak':max(abs(v) for v in iq),
        'Vq_mean':statistics.mean(vq), 'Vq_pk':max(abs(v) for v in vq),
        'Vbus_mean':statistics.mean(vb), 'Vbus_min':min(vb),
    }

def check_fault(frames):
    for f in frames:
        if f['fault']!='0x00000000': return f['fault']
    return None

# ── Setup ──
print("=== Setup ===")
for c in ['CMD:CLEAR_FAULT','CMD:VBUS_LIMIT,10,16',
          'CMD:PI_CURRENT,0.50,0','CMD:PI_SPEED,0.25,0.001',
          'CMD:RS_FF_MODE,1','CMD:RS_FF_SCALE,0.20','CMD:RS_FF_ADAPTIVE,0',
          'CMD:COG_CFG,0.25,60','CMD:BEMF_CFG,0',
          'CMD:UNLOCK,1','CMD:ENABLE,1']:
    cmd(c, 0.15)
time.sleep(0.5)
fs = capture(0.5)
print(f"  State={fs[-1]['state'] if fs else '?'}  Vbus={fs[-1]['Vbus'] if fs else 0:.1f}V  Fault={check_fault(fs) or 'none'}")

# ── Load test points ──
POINTS = [+0.5, -0.5, +0.7, -0.7]

for sref in POINTS:
    print(f"\n{'='*60}")
    print(f"  SREF = {sref:+.1f} rad/s")
    print(f"{'='*60}")

    # 1. SREF=0, wait 1s
    cmd("CMD:SREF,0", 0.3)
    time.sleep(1.0)

    # 2-3. Set SREF, empty run 3s, then capture 1s
    cmd(f"CMD:SREF,{sref:.3f}", 0.2)
    print(f"  空载运行 3s...")
    time.sleep(3.0)
    fs_empty = capture(1.0)
    if not fs_empty:
        print(f"  EMPTY: NO DATA")
        continue
    se = stats(fs_empty)
    fe = check_fault(fs_empty)
    print(f"  空载: ω={se['w_mean']:+.3f}±{se['w_std']:.3f}  Iq={se['Iq_mean']:+.3f}A  Iq_pk={se['Iq_peak']:+.3f}A  Vq_pk={se['Vq_pk']*1000:.0f}mV  Vbus_min={se['Vbus_min']:.1f}V  fault={fe or 'none'}")

    if fe:
        print(f"  ** 空载已故障，跳过加载 **")
        cmd("CMD:CLEAR_FAULT",0.3); cmd("CMD:UNLOCK,1",0.2); cmd("CMD:ENABLE,1",0.5)
        RESULTS.append({'sref':sref,'empty':se,'empty_fault':fe,'loaded':None,'zero':None,'verdict':'EMPTY_FAULT'})
        continue

    # 4-6. LOAD NOW prompt, load 3s, capture 1s
    print(f"  >>> LOAD NOW — 用手捏电机轴施加负载 (3s) <<<")
    time.sleep(3.0)
    fs_load = capture(1.0)
    if not fs_load:
        print(f"  LOAD: NO DATA")
        RESULTS.append({'sref':sref,'empty':se,'empty_fault':fe,'loaded':None,'zero':None,'verdict':'LOAD_NO_DATA'})
        cmd("CMD:SREF,0",0.3); continue

    sl = stats(fs_load)
    fl = check_fault(fs_load)
    dw = abs(se['w_mean'] - sl['w_mean']) if se and sl else 0
    diq = sl['Iq_mean'] - se['Iq_mean']
    speed_drop_pct = dw / abs(sref) * 100 if abs(sref) > 0.001 else 0
    print(f"  负载: ω={sl['w_mean']:+.3f}±{sl['w_std']:.3f}  Iq={sl['Iq_mean']:+.3f}A  Iq_pk={sl['Iq_peak']:+.3f}A  Vq_pk={sl['Vq_pk']*1000:.0f}mV  Vbus_min={sl['Vbus_min']:.1f}V  fault={fl or 'none'}")
    print(f"  Δ:    dω={dw:+.3f} ({speed_drop_pct:.0f}% drop)  dIq={diq:+.3f}A")

    # 7-9. RELEASE, SREF=0, wait 2s, capture zero return
    print(f"  >>> RELEASE — 松开电机 <<<")
    cmd("CMD:SREF,0", 0.3)
    time.sleep(2.0)
    fs_zero = capture(1.0)
    if fs_zero:
        sz = stats(fs_zero)
        fz = check_fault(fs_zero)
        print(f"  回零: ω={sz['w_mean']:+.3f}  Vq_pk={sz['Vq_pk']*1000:.0f}mV  Vbus_min={sz['Vbus_min']:.1f}V  fault={fz or 'none'}")
    else:
        sz = {}; fz = 'NO_DATA'

    # ── Verdict ──
    checks = []
    if speed_drop_pct < 20: checks.append("speed_drop_OK")
    else: checks.append(f"speed_drop_{speed_drop_pct:.0f}%")
    if diq > 0.005: checks.append("Iq_increase_OK")
    else: checks.append("Iq_no_response")
    if sl['Iq_peak'] < 0.35: checks.append("Iq_peak_OK")
    else: checks.append(f"Iq_peak_HIGH({sl['Iq_peak']:.2f}A)")
    if not fl: checks.append("no_fault")
    else: checks.append(f"FAULT_{fl}")
    if fz != 'NO_DATA' and not fz and (sz.get('Vq_pk',0) < 0.100):
        checks.append("zero_clean")
    else: checks.append("zero_dirty")

    verdict = "PASS" if all(c.endswith('OK') or c in ('no_fault','zero_clean') for c in checks) else "CHECK"
    print(f"  Verdict: {verdict}  [{', '.join(checks)}]")

    RESULTS.append({
        'sref': sref,
        'empty': se, 'empty_fault': fe,
        'loaded': sl, 'loaded_fault': fl,
        'zero': sz, 'zero_fault': fz,
        'checks': checks, 'verdict': verdict,
    })

    if fl:
        cmd("CMD:CLEAR_FAULT",0.3); cmd("CMD:UNLOCK,1",0.2); cmd("CMD:ENABLE,1",0.5)
        time.sleep(0.5)

# ── Cleanup ──
cmd("CMD:SREF,0",0.3); cmd("CMD:ENABLE,0",0.3); cmd("CMD:UNLOCK,0",0.3)
s.close()

# ── Summary ──
print(f"\n{'='*60}")
print("LOAD TEST SUMMARY")
print(f"{'='*60}")
print(f"  {'SREF':>6s}  {'empty w':>8s}  {'load w':>8s}  {'dIq':>6s}  {'Iq_pk':>6s}  {'Vbus_min':>8s}  {'Verdict':>8s}")
for r in RESULTS:
    se = r.get('empty',{}) or {}; sl = r.get('loaded',{}) or {}
    diq = sl.get('Iq_mean',0) - se.get('Iq_mean',0)
    print(f"  {r['sref']:+6.1f}  {se.get('w_mean',0):+8.3f}  {sl.get('w_mean',0):+8.3f}  {diq:+6.3f}  {sl.get('Iq_peak',0):+6.3f}  {sl.get('Vbus_min',0) or se.get('Vbus_min',0):7.1f}V  {r['verdict']:>8s}")

# Save
fn = f"scripts/12v_load_clean_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
with open(fn,'w') as f:
    # Convert stats dicts to serializable form
    json.dump(RESULTS, f, indent=2, default=str)
print(f"\nResults: {fn}")
print("Done.")
