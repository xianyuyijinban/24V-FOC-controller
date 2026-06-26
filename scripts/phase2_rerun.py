"""
Phase 2 Re-run: Endurance + Position Regression (after DRV8350S UVLO fault cleared)
12V supply: using ±0.5 rad/s for endurance to stay within voltage limits
"""
import serial, time, sys, json, statistics
from datetime import datetime

PORT = 'COM9'; BAUD = 230400

class FOCBoard:
    def __init__(self):
        self.s = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(0.5); self.drain()
    def drain(self):
        time.sleep(0.3); self.s.read(self.s.in_waiting or 65536)
    def cmd(self, c, w=0.5):
        self.s.reset_input_buffer(); self.s.write((c+'\r\n').encode())
        time.sleep(w); return self.s.read(self.s.in_waiting or 65536)
    def capture(self, dur=1.0):
        self.s.reset_input_buffer(); time.sleep(dur)
        data = self.s.read(self.s.in_waiting or 65536)
        text = data.decode('ascii', errors='replace')
        frames = []
        for line in text.split('\n'):
            line = line.strip()
            if line.startswith('N,'):
                parts = line.split(',')
                if len(parts) >= 30:
                    try:
                        frames.append({
                            'ts': int(parts[1]), 'state': int(parts[2]),
                            'speed': float(parts[4]), 'Iq': float(parts[6]),
                            'fault': parts[8].strip(), 'Vq': float(parts[20]),
                            'ctrl_mode': int(parts[15]), 'speed_ref': float(parts[17]),
                        })
                    except: pass
        return frames
    def close(self):
        self.s.close()

def check_fault(frames):
    for f in frames:
        if f['fault'] != '0x00000000': return f['fault']
    return None

def setup(board):
    print("=== Setup ===")
    cmds = [
        "CMD:CLEAR_FAULT", "CMD:VBUS_LIMIT,10,16",
        "CMD:PI_CURRENT,0.50,0", "CMD:PI_SPEED,0.25,0.001",
        "CMD:RS_FF_MODE,1", "CMD:RS_FF_SCALE,0.20", "CMD:RS_FF_ADAPTIVE,0",
        "CMD:COG_CFG,0.25,60", "CMD:BEMF_CFG,0", "CMD:KE_TEMP,0",
        "CMD:UNLOCK,1",
    ]
    for c in cmds:
        board.cmd(c, 0.2)
    board.cmd("CMD:ENABLE,1", 0.5)
    time.sleep(0.3)
    print("  PWM enabled")

def test_endurance(board, speed=0.5, cycles=20):
    print(f"\n{'='*60}")
    print(f"Endurance Test: ±{speed} rad/s, {cycles} cycles")
    print(f"{'='*60}")

    results = []
    faults = 0
    vq_pks = []

    for i in range(cycles):
        # +speed
        board.cmd(f"CMD:SREF,{speed:.3f}", 0.2)
        time.sleep(0.5)
        fp = board.capture(0.5)
        fp_fault = check_fault(fp)

        # →0
        board.cmd("CMD:SREF,0", 0.2)
        time.sleep(0.4)
        fz1 = board.capture(0.4)
        fz1_fault = check_fault(fz1)

        # -speed
        board.cmd(f"CMD:SREF,-{speed:.3f}", 0.2)
        time.sleep(0.5)
        fn = board.capture(0.5)
        fn_fault = check_fault(fn)

        # →0
        board.cmd("CMD:SREF,0", 0.2)
        time.sleep(0.4)
        fz2 = board.capture(0.4)
        fz2_fault = check_fault(fz2)

        any_fault = fp_fault or fz1_fault or fn_fault or fz2_fault
        if any_fault:
            faults += 1

        # Vq peaks from zero-return
        vq_pk_cycle = 0
        for fz in [fz1, fz2]:
            if fz:
                vqs = [abs(f['Vq']) for f in fz]
                if vqs: vq_pk_cycle = max(vq_pk_cycle, max(vqs))
        vq_pks.append(vq_pk_cycle)

        # Speed tracking
        pos_speeds = [f['speed'] for f in fp] if fp else [0]
        neg_speeds = [f['speed'] for f in fn] if fn else [0]
        pos_mean = statistics.mean(pos_speeds) if pos_speeds else 0
        neg_mean = statistics.mean(neg_speeds) if neg_speeds else 0

        status = "FAULT" if any_fault else "OK"
        marker = " !" if any_fault else ""
        print(f"  {i+1:2d}/{cycles}: {status:5s} ω+={pos_mean:+.3f} ω-={neg_mean:+.3f} Vq_pk={vq_pk_cycle*1000:3.0f}mV{marker}")

        results.append({
            'cycle': i+1, 'speed_pos_mean': pos_mean, 'speed_neg_mean': neg_mean,
            'vq_pk_mV': vq_pk_cycle*1000, 'fault': any_fault,
            'fp_fault': fp_fault, 'fz1_fault': fz1_fault,
            'fn_fault': fn_fault, 'fz2_fault': fz2_fault,
        })

        if any_fault:
            # Try to recover
            board.cmd("CMD:CLEAR_FAULT", 0.3)
            board.cmd("CMD:UNLOCK,1", 0.2)
            board.cmd("CMD:ENABLE,1", 0.5)
            time.sleep(0.3)

    # Summary
    print(f"\n  --- Summary ---")
    vq_ok = sum(1 for v in vq_pks if v < 0.060)
    print(f"  Faults: {faults}/{cycles}")
    print(f"  Vq_pk <60mV: {vq_ok}/{cycles}")
    if vq_pks:
        print(f"  Vq_pk range: {min(vq_pks)*1000:.0f}-{max(vq_pks)*1000:.0f} mV")

    return {'cycles': cycles, 'speed': speed, 'faults': faults,
            'vq_ok_count': vq_ok, 'vq_pk_max_mV': max(vq_pks)*1000 if vq_pks else 0,
            'results': results}

def test_position(board):
    print(f"\n{'='*60}")
    print("Position Regression Test")
    print(f"{'='*60}")

    # Switch to position mode
    board.cmd("CMD:MODE,2", 0.5)
    print("  Mode: POSITION")

    pos_targets = [0.0, 5.0, -5.0, 20.0, -20.0, 0.0]
    results = {}

    for pref in pos_targets:
        label = f"PREF={pref:+.1f}"
        board.cmd(f"CMD:PREF,{pref:.1f}", 0.3)
        time.sleep(2.0)
        frames = board.capture(2.0)

        if not frames:
            print(f"  {label:12s} NO DATA")
            results[label] = {'error': 'NO DATA'}
            continue

        fault = check_fault(frames)
        speeds = [f['speed'] for f in frames]
        mean_speed = statistics.mean(speeds) if speeds else 0
        drift_ok = abs(mean_speed) < 0.05

        status = "PASS" if (drift_ok and not fault) else ("FAULT" if fault else "DRIFT")
        print(f"  {label:12s} ω={mean_speed:+.4f} rad/s drift_ok={drift_ok} fault={fault or 'none'} → {status}")

        results[label] = {
            'n_frames': len(frames), 'mean_speed': mean_speed,
            'drift_ok': drift_ok, 'fault': fault, 'status': status,
        }

        if fault:
            board.cmd("CMD:CLEAR_FAULT", 0.3)
            board.cmd("CMD:UNLOCK,1", 0.2)
            board.cmd("CMD:ENABLE,1", 0.5)
            time.sleep(0.5)

    # Back to speed mode
    board.cmd("CMD:MODE,1", 0.3)
    print("  Mode: SPEED (restored)")
    return results


def main():
    board = FOCBoard()
    setup(board)

    all_results = {'timestamp': datetime.now().isoformat(), 'vbus': '12V'}

    # Endurance at ±0.5 (safe for 12V)
    all_results['endurance_0.5'] = test_endurance(board, speed=0.5, cycles=20)

    # Position regression
    all_results['position'] = test_position(board)

    # Cleanup
    board.cmd("CMD:SREF,0", 0.3)
    board.cmd("CMD:ENABLE,0", 0.3)
    board.cmd("CMD:UNLOCK,0", 0.3)
    board.close()

    # Save
    fn = f"scripts/phase2_rerun_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(fn, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)

    # Final verdict
    print(f"\n{'='*60}")
    endur = all_results['endurance_0.5']
    pos = all_results['position']

    e_ok = endur['faults'] == 0 and endur['vq_ok_count'] >= 18
    p_ok = all(v.get('status') == 'PASS' for v in pos.values() if isinstance(v, dict))

    print(f"Endurance (±0.5): {'PASS' if e_ok else 'CHECK'} ({endur['faults']} faults, {endur['vq_ok_count']}/{endur['cycles']} Vq<60mV)")
    print(f"Position:        {'PASS' if p_ok else 'CHECK'}")
    print(f"Results: {fn}")

    return 0 if (e_ok and p_ok) else 1

if __name__ == '__main__':
    sys.exit(main())
