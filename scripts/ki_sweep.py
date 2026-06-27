"""
Speed Loop Ki Sweep — Phase 1 of low-speed maintenance fix.
Baseline: Kp=0.25, Ki=0, RS_FF=DQ/0.20, COG=0.25/60, BEMF=OFF
Sweeps Ki=0.0005, 0.001, 0.002 across SREF +-0.05..+-1.0
"""
import serial
import time
import json
from datetime import datetime

PORT = 'COM9'
BAUD = 1152000

KI_VALUES = [0.0005, 0.001, 0.002]
SREF_VALUES = [0.05, 0.10, 0.20, 0.30, 1.0]
HOLD_DUR = 3.0
ZERO_DUR = 1.5
REPEATS = 3


class FOCBoard:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.5)
        self.drain()

    def drain(self):
        time.sleep(0.3)
        self.s.read(self.s.in_waiting or 65536)

    def cmd(self, cmd_str, wait=0.5):
        self.s.reset_input_buffer()
        self.s.write((cmd_str + '\r\n').encode())
        if wait <= 0:
            return ""
        time.sleep(wait)
        data = self.s.read(self.s.in_waiting or 65536)
        return data.decode('ascii', errors='replace')

    def capture(self, duration=2.0):
        self.s.reset_input_buffer()
        time.sleep(duration)
        data = self.s.read(self.s.in_waiting or 65536)
        text = data.decode('ascii', errors='replace')
        frames = []
        for line in text.split('\n'):
            line = line.strip()
            if line.startswith('N,'):
                parts = line.split(',')
                if len(parts) >= 33:
                    try:
                        frames.append({
                            'ts': int(parts[1]),
                            'speed': float(parts[4]),
                            'Id': float(parts[5]),
                            'Iq': float(parts[6]),
                            'fault': parts[8].strip(),
                            'Id_ref': float(parts[16]),
                            'speed_ref': float(parts[17]),
                            'Iq_ref': float(parts[19]),
                            'Vd': float(parts[20]),
                            'Vq': float(parts[21]),
                        })
                    except (ValueError, IndexError):
                        pass
        return frames

    def close(self):
        self.s.close()


def analyze_hold(frames, sref_target):
    if not frames or len(frames) < 20:
        return None
    n = len(frames)
    # Last 50 frames (~1s) of hold
    sf = frames[-min(50, n):]
    speeds = [f['speed'] for f in sf]
    iqs = [f['Iq'] for f in sf]
    iq_refs = [f['Iq_ref'] for f in sf]
    vqs = [f['Vq'] for f in sf]

    avg_spd = sum(speeds)/len(speeds)
    return {
        'speed_mean': avg_spd,
        'speed_err': avg_spd - sref_target,
        'speed_std': (sum((s-avg_spd)**2 for s in speeds)/len(speeds))**0.5,
        'Iq_mean': sum(iqs)/len(iqs),
        'Iq_ref_mean': sum(iq_refs)/len(iq_refs),
        'Vq_mean': sum(vqs)/len(vqs),
        'n': n,
    }


def analyze_zero(frames):
    if not frames:
        return None
    zf = [f for f in frames[-40:] if abs(f['speed']) < 0.03]
    if zf:
        vq_pk = max(abs(f['Vq']) for f in zf)
        spd_pk = max(abs(f['speed']) for f in zf)
    else:
        vq_pk = max(abs(f['Vq']) for f in frames[-10:])
        spd_pk = max(abs(f['speed']) for f in frames[-10:])
    return {'Vq_pk': vq_pk, 'speed_pk': spd_pk}


def setup_baseline(board):
    print("=== Baseline ===")
    for cmd, desc in [
        ("CMD:VBUS_LIMIT,8,15", "VBUS"),
        ("CMD:CLEAR_FAULT", "ClearFault"),
        ("CMD:PI_CURRENT,0.50,0", "PI_CUR=0.50/0"),
        ("CMD:PI_SPEED,0.25,0", "PI_SPD=0.25/0"),
        ("CMD:RS_FF_MODE,1", "RS_FF=DQ"),
        ("CMD:RS_FF_SCALE,0.20", "RS_FF=0.20"),
        ("CMD:RS_FF_ADAPTIVE,0", "RS_FF_ADAPT=OFF"),
        ("CMD:COG_CFG,0.25,60", "COG=0.25/60"),
        ("CMD:BEMF_CFG,0", "BEMF=OFF"),
        ("CMD:UNLOCK,1", "UNLOCK"),
    ]:
        board.cmd(cmd, wait=0.15)
    board.cmd("CMD:ENABLE,1", wait=0.3)
    board.cmd("CMD:MODE,1", wait=0.3)
    board.drain()


def run_ki_sweep(board):
    all_results = {}

    for ki in KI_VALUES:
        print(f"\n{'='*70}")
        print(f"Ki = {ki:.4f}")
        print(f"{'='*70}")
        board.cmd(f"CMD:PI_SPEED,0.25,{ki:.4f}", wait=0.2)
        ki_results = {}

        for sref_mag in SREF_VALUES:
            for direction, sign in [("POS", 1.0), ("NEG", -1.0)]:
                sref = sref_mag * sign
                label = f"SREF={sref:+.2f}"
                rep_data = []

                for rep in range(REPEATS):
                    board.cmd("CMD:SREF,0", wait=0.3)
                    time.sleep(0.3)
                    board.drain()
                    board.cmd(f"CMD:SREF,{sref:.3f}", wait=0.2)
                    time.sleep(0.1)
                    frames_hold = board.capture(duration=HOLD_DUR)
                    board.cmd("CMD:SREF,0", wait=0.2)
                    time.sleep(0.1)
                    frames_zero = board.capture(duration=ZERO_DUR)

                    hold = analyze_hold(frames_hold, sref)
                    zero = analyze_zero(frames_zero)
                    faults = [f['fault'] for f in frames_hold + frames_zero if f['fault'] != '0x00000000']

                    rep_data.append({'hold': hold, 'zero': zero, 'faults': faults})

                # Average across reps
                holds = [r['hold'] for r in rep_data if r['hold']]
                zeros = [r['zero'] for r in rep_data if r['zero']]
                faults_all = [f for r in rep_data for f in r['faults']]

                if holds:
                    avg_spd = sum(h['speed_mean'] for h in holds)/len(holds)
                    avg_err = sum(h['speed_err'] for h in holds)/len(holds)
                    avg_iq = sum(h['Iq_mean'] for h in holds)/len(holds)
                    avg_iqr = sum(h['Iq_ref_mean'] for h in holds)/len(holds)
                    avg_vq = sum(h['Vq_mean'] for h in holds)/len(holds)
                    avg_osc = sum(h['speed_std'] for h in holds)/len(holds)
                else:
                    avg_spd = avg_err = avg_iq = avg_iqr = avg_vq = avg_osc = None

                if zeros:
                    avg_zvq = sum(z['Vq_pk'] for z in zeros)/len(zeros)
                    avg_zspd = sum(z['speed_pk'] for z in zeros)/len(zeros)
                else:
                    avg_zvq = avg_zspd = None

                # Print
                spd_s = f"{avg_spd:.3f}" if avg_spd is not None else "N/A"
                err_s = f"{avg_err*1000:.0f}m" if avg_err is not None else "N/A"
                iq_s = f"{avg_iq*1000:.0f}mA" if avg_iq is not None else "N/A"
                iqr_s = f"{avg_iqr*1000:.0f}mA" if avg_iqr is not None else "N/A"
                zvq_s = f"{avg_zvq*1000:.0f}mV" if avg_zvq is not None else "N/A"
                osc_s = f"osc={avg_osc*1000:.0f}m" if avg_osc is not None else ""
                flt_s = f"FLT:{len(faults_all)}" if faults_all else "OK"

                print(f"  {label:>8s}  spd={spd_s:>8s}  err={err_s:>6s}  "
                      f"Iq={iq_s:>6s}  Iqref={iqr_s:>6s}  Vq={avg_vq*1000:.0f}mV  "
                      f"zeroVq={zvq_s:>5s}  {osc_s}  {flt_s}")

                ki_results[label] = {
                    'sref': sref, 'direction': direction,
                    'speed_mean': avg_spd, 'speed_err': avg_err,
                    'speed_osc': avg_osc,
                    'Iq_mean': avg_iq, 'Iq_ref_mean': avg_iqr, 'Vq_mean': avg_vq,
                    'zero_Vq_pk': avg_zvq, 'zero_speed_pk': avg_zspd,
                    'faults': len(faults_all),
                }

        all_results[f"Ki={ki:.4f}"] = ki_results

    return all_results


def print_summary(all_results):
    print(f"\n{'='*90}")
    print("KI SWEEP SUMMARY")
    print(f"{'='*90}")

    for ki_label, ki_data in all_results.items():
        print(f"\n--- {ki_label} ---")
        print(f"{'SREF':>8s} {'Dir':>4s} {'Speed':>8s} {'Err':>7s} {'Osc':>7s} "
              f"{'Iq':>7s} {'Iqref':>7s} {'Vq':>7s} {'ZeroVq':>7s} {'Flt':>3s}")
        for label, d in ki_data.items():
            if d['speed_mean'] is None:
                continue
            spd = f"{d['speed_mean']:.3f}"
            err = f"{d['speed_err']*1000:.0f}m"
            osc = f"{d['speed_osc']*1000:.0f}m"
            iq = f"{d['Iq_mean']*1000:.0f}mA"
            iqr = f"{d['Iq_ref_mean']*1000:.0f}mA"
            vq = f"{d['Vq_mean']*1000:.0f}mV"
            zvq = f"{d['zero_Vq_pk']*1000:.0f}mV"
            flt = d['faults']
            print(f"{d['sref']:+8.2f} {d['direction']:>4s} {spd:>8s} {err:>7s} {osc:>7s} "
                  f"{iq:>7s} {iqr:>7s} {vq:>7s} {zvq:>7s} {flt:>3d}")

    # Decision summary
    print(f"\n{'='*90}")
    print("ACCEPTANCE CHECK (target Ki=0.001)")
    print(f"{'='*90}")
    ki001 = all_results.get("Ki=0.0010", {})
    if ki001:
        sref_010 = ki001.get("SREF=+0.10", {})
        sref_m010 = ki001.get("SREF=-0.10", {})
        sref_100 = ki001.get("SREF=+1.00", {})

        pos_010_ok = sref_010.get('speed_mean', 0) and abs(sref_010.get('speed_mean', 0)) >= 0.08
        neg_010_ok = sref_m010.get('speed_mean', 0) and abs(sref_m010.get('speed_mean', 0)) >= 0.08

        print(f"  SREF=+0.10 speed={sref_010.get('speed_mean',0):.3f} (>0.08? {'PASS' if pos_010_ok else 'FAIL'})")
        print(f"  SREF=-0.10 speed={sref_m010.get('speed_mean',0):.3f} (>0.08? {'PASS' if neg_010_ok else 'FAIL'})")

        # Check zero return
        all_zvq = [d.get('zero_Vq_pk', 0) or 0 for d in ki001.values()]
        max_zvq = max(all_zvq) if all_zvq else 0
        print(f"  Max zero Vq={max_zvq*1000:.0f}mV (<50mV? {'PASS' if max_zvq<0.05 else 'FAIL'})")

        # Symmetry
        pos_spds = [d['speed_mean'] for d in ki001.values() if d['direction'] == 'POS' and d['speed_mean']]
        neg_spds = [abs(d['speed_mean']) for d in ki001.values() if d['direction'] == 'NEG' and d['speed_mean']]
        if pos_spds and neg_spds:
            asym = abs(sum(pos_spds)/len(pos_spds) - sum(neg_spds)/len(neg_spds))
            print(f"  POS/NEG asymmetry={asym*1000:.0f}mrad/s (<50m? {'PASS' if asym<0.05 else 'FAIL'})")


def main():
    print("=" * 90)
    print("SPEED LOOP Ki SWEEP")
    print(f"Started: {datetime.now().isoformat()}")
    print("=" * 90)

    board = FOCBoard()
    try:
        setup_baseline(board)
        results = run_ki_sweep(board)
        print_summary(results)

        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.cmd("CMD:ENABLE,0")

        out = f"E:/24V_FOC_Controller_sync_20260519/scripts/ki_sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(out, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        print(f"\nSaved: {out}")

    finally:
        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.cmd("CMD:ENABLE,0")
        board.close()


if __name__ == '__main__':
    main()
