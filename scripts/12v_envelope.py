"""
12V Operating Envelope Characterization
=======================================
Finds stable speed ceiling, low-speed floor, position limits, and voltage margin.
No pass/fail — this is measurement, not verification.

Tests:
  1. Speed sweep: ±0.3 → ±1.0 in 0.1 steps, find saturation knee
  2. Low-speed dwell: ±0.05/±0.10/±0.20 with 5s settle
  3. Position envelope: sweep speed_limit & accel
  4. Load margin: hand-grab test (prompted)
"""
import serial, time, sys, json, statistics
from datetime import datetime

PORT='COM9'; BAUD=230400

class FOCBoard:
    def __init__(self):
        self.s = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(0.5); self.drain()
    def drain(self):
        time.sleep(0.3); self.s.read(self.s.in_waiting or 65536)
    def cmd(self, c, w=0.3):
        self.s.reset_input_buffer(); self.s.write((c+'\r\n').encode())
        time.sleep(w); return self.s.read(self.s.in_waiting or 65536)
    def capture(self, dur=1.0):
        self.s.reset_input_buffer(); time.sleep(dur)
        data = self.s.read(self.s.in_waiting or 65536)
        text = data.decode('ascii', errors='replace')
        frames=[]
        for line in text.split('\n'):
            line=line.strip()
            if line.startswith('N,'):
                parts=line.split(',')
                if len(parts)>=25:
                    try:
                        frames.append({
                            'speed':float(parts[4]),'Iq':float(parts[6]),
                            'fault':parts[8].strip(),'Vq':float(parts[20]),
                            'state':int(parts[2]),'Vbus':float(parts[7]),
                            'ctrl_mode':int(parts[15]),'Iq_ref':float(parts[19]),
                        })
                    except: pass
        return frames
    def close(self):
        self.s.close()

def fault(frames):
    for f in frames:
        if f['fault']!='0x00000000': return f['fault']
    return None

def stats(frames):
    if not frames: return {}
    sp=[f['speed'] for f in frames]; vq=[f['Vq'] for f in frames]
    iq=[f['Iq'] for f in frames]; vb=[f['Vbus'] for f in frames]
    return {
        'n':len(frames),
        'w_mean':statistics.mean(sp), 'w_std':statistics.stdev(sp) if len(sp)>1 else 0,
        'w_pk':max(abs(s) for s in sp),
        'Vq_mean':statistics.mean(vq), 'Vq_pk':max(abs(v) for v in vq),
        'Iq_mean':statistics.mean(iq), 'Iq_pk':max(abs(v) for v in iq),
        'Vbus_mean':statistics.mean(vb), 'Vbus_min':min(vb),
    }

def setup(board):
    print("=== Setup (12V baseline) ===")
    for c in [
        "CMD:CLEAR_FAULT","CMD:VBUS_LIMIT,10,16",
        "CMD:PI_CURRENT,0.50,0","CMD:PI_SPEED,0.25,0.001",
        "CMD:RS_FF_MODE,1","CMD:RS_FF_SCALE,0.20","CMD:RS_FF_ADAPTIVE,0",
        "CMD:COG_CFG,0.25,60","CMD:BEMF_CFG,0",
        "CMD:UNLOCK,1","CMD:ENABLE,1",
    ]:
        board.cmd(c, 0.15)
    time.sleep(0.3)
    fs = board.capture(0.3)
    st = fs[-1]['state'] if fs else -1
    fl = fault(fs)
    vb = fs[-1]['Vbus'] if fs else 0
    print(f"  State={st}  Vbus={vb:.1f}V  Fault={fl or 'none'}")

# ── Test 1: Speed Envelope ──
def test_speed_envelope(board):
    print("\n"+"="*60)
    print("TEST 1: Speed Envelope — find saturation knee")
    print("="*60)
    print(f"  {'SREF':>8s}  {'w_avg':>8s}  {'track%':>6s}  {'w_std':>6s}  {'Vq_pk':>6s}  {'Iq_avg':>6s}  {'Vbus':>6s}  Note")
    print(f"  {'-'*8}  {'-'*8}  {'-'*6}  {'-'*6}  {'-'*6}  {'-'*6}  {'-'*6}  ----")

    results = []
    for sref in [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        for sign in [1, -1]:
            val = sref * sign
            board.cmd(f"CMD:SREF,{val:.3f}", 0.2)
            time.sleep(2.0)  # long settle
            fs = board.capture(2.0)
            if not fs:
                print(f"  {val:+8.2f}  {'NO DATA':>8s}")
                results.append({'sref':val,'error':'NO DATA'})
                continue

            s = stats(fs); fl = fault(fs)
            track_pct = abs(s['w_mean'])/sref*100 if sref>0.001 else 0

            # Classify
            if fl:
                note = f"FAULT {fl}"
            elif track_pct > 110:
                note = "overshoot"
            elif track_pct >= 85:
                note = "stable"
            elif track_pct >= 60:
                note = "degraded"
            else:
                note = "SATURATED"

            print(f"  {val:+8.2f}  {s['w_mean']:+8.3f}  {track_pct:5.0f}%  {s['w_std']:5.3f}  {s['Vq_pk']*1000:5.0f}mV  {s['Iq_mean']:+6.3f}  {s['Vbus_min']:5.1f}V  {note}")

            results.append({**s, 'sref':val, 'track_pct':track_pct, 'note':note, 'fault':fl})

            if fl:
                board.cmd("CMD:CLEAR_FAULT", 0.3)
                board.cmd("CMD:UNLOCK,1", 0.2)
                board.cmd("CMD:ENABLE,1", 0.5)
                time.sleep(0.3)

    # Summary
    stable = [r for r in results if r.get('note')=='stable']
    degraded = [r for r in results if r.get('note')=='degraded']
    sat = [r for r in results if r.get('note')=='SATURATED']
    print(f"\n  Summary: {len(stable)} stable, {len(degraded)} degraded, {len(sat)} saturated")
    if stable:
        max_stable = max(abs(r['sref']) for r in stable)
        print(f"  Stable ceiling: ±{max_stable:.1f} rad/s")
    if sat:
        sat_speed = min(abs(r['sref']) for r in sat)
        print(f"  Saturation onset: ±{sat_speed:.1f} rad/s")
    return results

# ── Test 2: Low-Speed Dwell ──
def test_low_speed_dwell(board):
    print("\n"+"="*60)
    print("TEST 2: Low-Speed Dwell — find control floor")
    print("="*60)
    print(f"  {'SREF':>8s}  {'w_avg':>8s}  {'track%':>6s}  {'w_std':>6s}  {'Vq_pk':>6s}  {'Iq_avg':>6s}  Note")

    results = []
    for sref in [0.05, 0.10, 0.15, 0.20]:
        for sign in [1, -1]:
            val = sref * sign
            board.cmd(f"CMD:SREF,{val:.3f}", 0.2)
            time.sleep(3.0)  # extra long settle for low speed
            fs = board.capture(3.0)
            if not fs:
                print(f"  {val:+8.2f}  {'NO DATA':>8s}")
                continue
            s = stats(fs); fl = fault(fs)
            track_pct = abs(s['w_mean'])/sref*100 if sref>0.001 else 0

            if fl: note=f"FAULT {fl}"
            elif s['w_pk'] < sref * 0.3: note="STUCK"
            elif track_pct >= 70: note="tracking"
            elif track_pct >= 40: note="slipping"
            else: note="stiction"

            print(f"  {val:+8.2f}  {s['w_mean']:+8.3f}  {track_pct:5.0f}%  {s['w_std']:5.3f}  {s['Vq_pk']*1000:5.0f}mV  {s['Iq_mean']:+6.3f}  {note}")
            results.append({**s, 'sref':val, 'track_pct':track_pct, 'note':note})

    track_ok = [r for r in results if r.get('note')=='tracking']
    slip = [r for r in results if r.get('note')=='slipping']
    stuck = [r for r in results if r.get('note') in ('stiction','STUCK')]
    print(f"\n  Summary: {len(track_ok)} tracking, {len(slip)} slipping, {len(stuck)} stuck/stiction")
    if track_ok:
        min_track = min(abs(r['sref']) for r in track_ok)
        print(f"  Control floor: ±{min_track:.2f} rad/s")
    return results

# ── Test 3: Position Envelope ──
def test_position_envelope(board):
    print("\n"+"="*60)
    print("TEST 3: Position Mode Envelope")
    print("="*60)

    # Switch to position mode
    board.cmd("CMD:MODE,2", 0.5)

    # Test with conservative defaults first
    print("\n  --- Position hold stability ---")
    for pref in [0, 5, -5, 20, -20, 0]:
        board.cmd(f"CMD:PREF,{pref:.1f}", 0.2)
        time.sleep(2.0)
        fs = board.capture(1.5)
        if fs:
            s = stats(fs); fl = fault(fs)
            drift = abs(s['w_mean'])
            ok = "OK" if drift<0.03 and not fl else ("FLT" if fl else f"drift={drift:.3f}")
            print(f"    PREF={pref:+5.0f}  w={s['w_mean']:+.4f}  Vq_pk={s['Vq_pk']*1000:4.0f}mV  Iq={s['Iq_mean']:+.4f}A  {ok}")
        else:
            print(f"    PREF={pref:+5.0f}  NO DATA")

    # Test speed limit sweep
    print("\n  --- Speed limit characterization ---")
    board.cmd("CMD:PREF,0", 0.3)
    time.sleep(1.0)

    for speed_lim in [0.5, 1.0, 2.0, 5.0]:
        board.cmd(f"CMD:MOTION_CFG,{speed_lim:.1f},2.0,0.5", 0.3)
        time.sleep(0.3)
        # Try a step that would use the speed limit
        board.cmd("CMD:PREF,10", 0.3)
        time.sleep(1.5)
        fs = board.capture(1.0)
        if fs:
            s = stats(fs); fl = fault(fs)
            max_sp = max(abs(f['speed']) for f in fs)
            ok = "OK" if not fl else f"FLT {fl}"
            print(f"    speed_lim={speed_lim:.1f}  w_max={max_sp:.3f}  w_mean={s['w_mean']:.3f}  {ok}")
        else:
            print(f"    speed_lim={speed_lim:.1f}  NO DATA")
        board.cmd("CMD:PREF,0", 0.3)
        time.sleep(1.0)

    # Back to speed mode
    board.cmd("CMD:MODE,1", 0.3)
    print("  Mode: SPEED")

# ── Test 4: Load Margin (manual) ──
def test_load_margin(board):
    print("\n"+"="*60)
    print("TEST 4: Load Margin — hand-grab check")
    print("="*60)

    speeds = [0.3, 0.5, 0.7]
    results = []

    for sref in speeds:
        print(f"\n  >>> SREF=+{sref:.1f} rad/s — apply hand load for 3s <<<")
        board.cmd(f"CMD:SREF,{sref:.3f}", 0.2)
        time.sleep(1.0)

        # Baseline (no load)
        fs0 = board.capture(1.0)
        s0 = stats(fs0) if fs0 else {}
        fl0 = fault(fs0)

        # Load phase (user applies load)
        time.sleep(0.5)
        fs_load = board.capture(2.0)
        s_load = stats(fs_load) if fs_load else {}
        fl_load = fault(fs_load)

        # Recovery
        time.sleep(1.0)
        fs_rec = board.capture(1.0)
        s_rec = stats(fs_rec) if fs_rec else {}
        fl_rec = fault(fs_rec)

        iq_delta = (s_load.get('Iq_mean',0) - s0.get('Iq_mean',0)) if s0 and s_load else 0
        sp_drop = (s0.get('w_mean',0) - s_load.get('w_mean',0)) if s0 and s_load else 0

        print(f"    Baseline:  w={s0.get('w_mean',0):+.3f}  Iq={s0.get('Iq_mean',0):+.3f}A  Vq={s0.get('Vq_pk',0)*1000:.0f}mV")
        print(f"    Loaded:    w={s_load.get('w_mean',0):+.3f}  Iq={s_load.get('Iq_mean',0):+.3f}A  Vq={s_load.get('Vq_pk',0)*1000:.0f}mV")
        print(f"    Recovery:  w={s_rec.get('w_mean',0):+.3f}  Iq={s_rec.get('Iq_mean',0):+.3f}A  Vq={s_rec.get('Vq_pk',0)*1000:.0f}mV")
        print(f"    Delta:     dw={sp_drop:+.3f}  dIq={iq_delta:+.3f}A  faults={fl0 or 'none'}/{fl_load or 'none'}/{fl_rec or 'none'}")

        results.append({
            'sref': sref,
            'baseline': s0, 'loaded': s_load, 'recovery': s_rec,
            'iq_delta': iq_delta, 'speed_drop': sp_drop,
            'faults': [fl0, fl_load, fl_rec],
        })

        board.cmd("CMD:SREF,0", 0.3)
        time.sleep(1.0)
        if fl_load or fl_rec:
            board.cmd("CMD:CLEAR_FAULT", 0.3)
            board.cmd("CMD:UNLOCK,1", 0.2)
            board.cmd("CMD:ENABLE,1", 0.5)
            time.sleep(0.3)

    # Negative direction
    print(f"\n  >>> SREF=-0.5 rad/s — apply hand load for 3s <<<")
    board.cmd("CMD:SREF,-0.500", 0.2)
    time.sleep(1.0)
    fs0 = board.capture(1.0)
    s0 = stats(fs0) if fs0 else {}
    time.sleep(0.5)
    fs_load = board.capture(2.0)
    s_load = stats(fs_load) if fs_load else {}
    time.sleep(1.0)
    fs_rec = board.capture(1.0)
    s_rec = stats(fs_rec) if fs_rec else {}
    fl_load = fault(fs_load) if fs_load else None
    print(f"    Baseline: w={s0.get('w_mean',0):+.3f} Iq={s0.get('Iq_mean',0):+.3f}A")
    print(f"    Loaded:   w={s_load.get('w_mean',0):+.3f} Iq={s_load.get('Iq_mean',0):+.3f}A")
    print(f"    Recovery: w={s_rec.get('w_mean',0):+.3f} Iq={s_rec.get('Iq_mean',0):+.3f}A")
    print(f"    Faults: {fault(fs0) or 'none'}/{fl_load or 'none'}/{fault(fs_rec) if fs_rec else 'none'}")

    board.cmd("CMD:SREF,0", 0.3)
    return results


# ── Main ──
def main():
    print("12V Operating Envelope Characterization")
    print(f"Time: {datetime.now().isoformat()}")
    board = FOCBoard()
    setup(board)

    all_results = {'timestamp': datetime.now().isoformat(), 'vbus_nominal': '12V'}

    # Test 1: speed envelope
    all_results['speed_envelope'] = test_speed_envelope(board)

    # Test 2: low-speed dwell
    all_results['low_speed_dwell'] = test_low_speed_dwell(board)

    # Test 3: position envelope
    all_results['position_envelope'] = test_position_envelope(board)

    # Test 4: load margin
    all_results['load_margin'] = test_load_margin(board)

    # Cleanup
    board.cmd("CMD:SREF,0", 0.3)
    board.cmd("CMD:ENABLE,0", 0.3)
    board.cmd("CMD:UNLOCK,0", 0.3)
    board.close()

    # Save
    fn = f"scripts/12v_envelope_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(fn, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"\nResults: {fn}")

    # ── Final synthesis ──
    print("\n"+"="*60)
    print("12V ENVELOPE SUMMARY")
    print("="*60)

    se = all_results['speed_envelope']
    stable = [r for r in se if r.get('note')=='stable']
    degraded = [r for r in se if r.get('note')=='degraded']
    if stable:
        max_stable = max(abs(r['sref']) for r in stable)
        print(f"  Rated speed:    ±{max_stable:.1f} rad/s")
    if degraded:
        max_deg = max(abs(r['sref']) for r in degraded)
        print(f"  Limit speed:    ±{max_deg:.1f} rad/s (degraded tracking)")

    ls = all_results['low_speed_dwell']
    track_ok = [r for r in ls if r.get('note')=='tracking']
    if track_ok:
        min_track = min(abs(r['sref']) for r in track_ok)
        print(f"  Control floor:  ±{min_track:.2f} rad/s")

    print(f"\n  Proposed defaults for 12V baseline:")
    rated_sp = 0.5
    limit_sp = 0.7
    print(f"    MOTION_CFG speed_limit:  {rated_sp:.1f} rad/s")
    print(f"    MOTION_CFG cruise:       {rated_sp*0.6:.1f} rad/s")
    print(f"    MOTION_CFG accel:        2.0 rad/s²")
    print(f"    Max SREF command:        ±{limit_sp:.1f} rad/s")

if __name__=='__main__':
    main()
