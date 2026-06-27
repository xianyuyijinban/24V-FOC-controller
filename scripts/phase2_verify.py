"""
Phase 2 Verification: Speed Loop Integral Gating
Tests Ki=0.001 with gated integration.
Sweep 0->+-0.05->0->+-0.10->0->+-0.20->0->+-0.30->0->+-1.0->0, then durability.
"""
import serial, time, json
from datetime import datetime

PORT, BAUD = 'COM9', 230400
HOLD, ZERO = 3.0, 1.5

class FOCBoard:
    def __init__(self):
        self.s = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(0.5); self.drain()
    def drain(self):
        time.sleep(0.3); self.s.read(self.s.in_waiting or 65536)
    def cmd(self, c, wait=0.5):
        self.s.reset_input_buffer(); self.s.write((c+'\r\n').encode())
        if wait<=0: return ""
        time.sleep(wait)
        return self.s.read(self.s.in_waiting or 65536).decode('ascii', errors='replace')
    def capture(self, dur=2.0):
        self.s.reset_input_buffer(); time.sleep(dur)
        data = self.s.read(self.s.in_waiting or 65536)
        text = data.decode('ascii', errors='replace')
        frames = []
        for line in text.split('\n'):
            line = line.strip()
            if line.startswith('N,'):
                p = line.split(',')
                if len(p)>=33:
                    try:
                        frames.append({'ts':int(p[1]),'speed':float(p[4]),'Id':float(p[5]),
                            'Iq':float(p[6]),'fault':p[8].strip(),'Id_ref':float(p[16]),
                            'speed_ref':float(p[17]),'Iq_ref':float(p[19]),'Vd':float(p[20]),
                            'Vq':float(p[21])})
                    except: pass
        return frames
    def close(self): self.s.close()

def analyze(frames, sref):
    if not frames or len(frames)<20: return None
    n = len(frames)
    sf = frames[-min(50,n):]
    spd = [f['speed'] for f in sf]
    iqs = [f['Iq'] for f in sf]; vqs = [f['Vq'] for f in sf]
    m = sum(spd)/len(spd)
    return {'speed':m,'err':m-sref,'osc':(sum((s-m)**2 for s in spd)/len(spd))**0.5,
            'Iq':sum(iqs)/len(iqs),'Vq':sum(vqs)/len(vqs),'n':n}

def analyze_zero(frames):
    """Measure settled Vq: use only last 0.5s (frames beyond 300ms mark).
    Frames arrive at ~50Hz, ZERO_DUR=1.5s => ~75 frames. Last 25 frames = ~0.5s."""
    if not frames: return None
    n = len(frames)
    settled = frames[max(0, n-25):]  # last 0.5s, well past 300ms settle
    z = [f for f in settled if abs(f['speed'])<0.03]
    if z: return max(abs(f['Vq']) for f in z), max(abs(f['speed']) for f in z)
    if settled: return max(abs(f['Vq']) for f in settled), max(abs(f['speed']) for f in settled)
    return None, None

# ─── Main ───
print("="*70)
print("PHASE 2 VERIFICATION: Integral Gating (Ki=0.001)")
print(datetime.now().isoformat())
board = FOCBoard()

try:
    # Setup
    for c in ['CMD:VBUS_LIMIT,8,15','CMD:CLEAR_FAULT','CMD:PI_CURRENT,0.50,0',
              'CMD:PI_SPEED,0.25,0.001','CMD:RS_FF_MODE,1','CMD:RS_FF_SCALE,0.20',
              'CMD:RS_FF_ADAPTIVE,0','CMD:COG_CFG,0.25,60','CMD:BEMF_CFG,0',
              'CMD:UNLOCK,1','CMD:ENABLE,1','CMD:MODE,1']:
        board.cmd(c, wait=0.15)
    board.drain()

    # Sweep
    results = {}
    for sref in [0.05, -0.05, 0.10, -0.10, 0.20, -0.20, 0.30, -0.30, 1.0, -1.0]:
        label = f"SREF={sref:+.2f}"
        print(f"\n{label}")
        reps = []
        for r in range(3):
            board.cmd("CMD:SREF,0", wait=0.3); time.sleep(0.3)
            board.drain()
            board.cmd(f"CMD:SREF,{sref:.2f}", wait=0.2); time.sleep(0.1)
            fh = board.capture(HOLD)
            board.cmd("CMD:SREF,0", wait=0.2); time.sleep(0.1)
            fz = board.capture(ZERO)
            h = analyze(fh, sref)
            zvq, zspd = analyze_zero(fz) if fz else (None,None)
            flt = [f for f in fh+fz if f['fault']!='0x00000000']
            reps.append({'hold':h,'zeroVq':zvq,'zeroSpd':zspd,'faults':flt})

        hs = [x['hold'] for x in reps if x['hold']]
        zvqs = [x['zeroVq'] for x in reps if x['zeroVq'] is not None]
        if hs:
            avg_spd = sum(x['speed'] for x in hs)/len(hs)
            avg_err = sum(x['err'] for x in hs)/len(hs)
            avg_iq  = sum(x['Iq'] for x in hs)/len(hs)
            avg_vq  = sum(x['Vq'] for x in hs)/len(hs)
            avg_osc = sum(x['osc'] for x in hs)/len(hs)
        else: avg_spd=avg_err=avg_iq=avg_vq=avg_osc=None
        avg_zvq = sum(zvqs)/len(zvqs) if zvqs else None

        print(f"  spd={avg_spd:.3f} err={avg_err*1000:.0f}mrad/s osc={avg_osc*1000:.0f}m "
              f"Iq={avg_iq*1000:.0f}mA Vq={avg_vq*1000:.0f}mV zeroVq={avg_zvq*1000:.0f}mV")
        results[label] = {'sref':sref,'speed':avg_spd,'err':avg_err,'osc':avg_osc,
                          'Iq':avg_iq,'Vq':avg_vq,'zeroVq':avg_zvq}

    # ─── Durability: 20 cycles 0->+1.0->0->-1.0->0 ───
    print(f"\n{'='*70}\nDURABILITY: 20 cycles +-1.0\n{'='*70}")
    dur_vqs = []
    for i in range(20):
        board.cmd("CMD:SREF,1.0", wait=0.2); time.sleep(0.15)
        board.cmd("CMD:SREF,0", wait=0.2); time.sleep(0.15)
        board.cmd("CMD:SREF,-1.0", wait=0.2); time.sleep(0.15)
        board.cmd("CMD:SREF,0", wait=0.2); time.sleep(0.5)
        f = board.capture(0.5)
        zvq,_ = analyze_zero(f)
        dur_vqs.append(zvq*1000 if zvq else 0)
        flt = [x for x in f if x['fault']!='0x00000000']
        if flt: print(f"  CYCLE {i+1} FAULT: {flt[0]['fault']}")
        if (i+1)%5==0: print(f"  Cycle {i+1}/20: Vq_pk={dur_vqs[-1]:.0f}mV")

    print(f"\n  Durability Vq_pks: {[f'{v:.0f}' for v in dur_vqs[:10]]}...")
    print(f"  Max Vq_pk={max(dur_vqs):.0f}mV, faults={sum(1 for v in dur_vqs if v>100)}")

    # ─── Acceptance ───
    print(f"\n{'='*70}\nACCEPTANCE\n{'='*70}")
    s010p = results.get('SREF=+0.10',{})
    s010n = results.get('SREF=-0.10',{})
    s020p = results.get('SREF=+0.20',{})
    s030p = results.get('SREF=+0.30',{})

    p1 = abs(s010p.get('speed',0))>=0.08
    p2 = abs(s010n.get('speed',0))>=0.08
    p3 = abs(s020p.get('err',1))<0.03  # <15% of 0.20
    p4 = abs(s030p.get('err',1))<0.045 # <15% of 0.30
    all_zvq = [r.get('zeroVq',0) or 0 for r in results.values()]
    p5 = max(all_zvq)<0.05 if all_zvq else False
    p6 = max(dur_vqs)<100  # durability Vq <100mV

    print(f"  SREF=+0.10 speed>0.08: {s010p.get('speed',0):.3f} {'PASS' if p1 else 'FAIL'}")
    print(f"  SREF=-0.10 speed>0.08: {s010n.get('speed',0):.3f} {'PASS' if p2 else 'FAIL'}")
    print(f"  SREF=+0.20 err<15%:    err={s020p.get('err',0)*1000:.0f}m {'PASS' if p3 else 'FAIL'}")
    print(f"  SREF=+0.30 err<15%:    err={s030p.get('err',0)*1000:.0f}m {'PASS' if p4 else 'FAIL'}")
    print(f"  Max zero Vq<50mV:      {max(all_zvq)*1000:.0f}mV {'PASS' if p5 else 'FAIL'}")
    print(f"  Durability Vq<100mV:   {max(dur_vqs):.0f}mV {'PASS' if p6 else 'FAIL'}")

    all_pass = p1 and p2 and p3 and p4 and p5 and p6
    print(f"\n  VERDICT: {'ALL PASS - Write Ki=0.001 to baseline' if all_pass else 'FAIL - needs tuning'}")

    # Disable
    board.cmd("CMD:MODE,0"); board.cmd("CMD:SREF,0"); board.cmd("CMD:ENABLE,0")

    # Save
    out = f"E:/24V_FOC_Controller_sync_20260519/scripts/phase2_verify_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    json.dump({'results':{k:{kk:vv for kk,vv in v.items() if kk!='faults'} for k,v in results.items()},
               'durability':{'vqs_mV':dur_vqs,'max':max(dur_vqs)},
               'verdict':'PASS' if all_pass else 'FAIL'}, open(out,'w'), indent=2, default=str)
    print(f"Saved: {out}")

finally:
    board.cmd("CMD:MODE,0"); board.cmd("CMD:SREF,0"); board.cmd("CMD:ENABLE,0")
    board.close()
