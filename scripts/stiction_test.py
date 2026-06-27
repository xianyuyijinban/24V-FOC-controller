"""
Stiction / Low-Speed Verification Test
Baseline: PI_CURRENT=0.50/0, PI_SPEED=0.25/0, RS_FF=DQ/0.20, COG=0.25/60°, BEMF=OFF
Sweeps SREF from ±0.03 to ±0.30 rad/s, 5 repeats each.
Measures start delay, steady-state error, Iq, Vq, return-to-zero residual.
"""
import serial
import time
import sys
import json
from datetime import datetime

PORT = 'COM9'
BAUD = 1152000

# Test parameters
SREF_VALUES = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30]
REPEATS = 5
HOLD_DURATION = 2.0    # seconds to hold each SREF
ZERO_DURATION = 1.0    # seconds to settle at zero after each step


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
        """Capture N-frames for duration seconds. Returns list of parsed frames."""
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
                            'state': int(parts[2]),
                            'angle': float(parts[3]),
                            'speed': float(parts[4]),
                            'Id': float(parts[5]),
                            'Iq': float(parts[6]),
                            'Vbus': float(parts[7]),
                            'fault': parts[8].strip(),
                            'ctrl_mode': int(parts[15]),
                            'Id_ref': float(parts[16]),
                            'speed_ref': float(parts[17]),
                            'Iq_ref': float(parts[19]),
                            'Vd': float(parts[20]),
                            'Vq': float(parts[21]),
                            'Ia': float(parts[22]),
                            'Ib': float(parts[23]),
                            'Ic': float(parts[24]),
                        })
                    except (ValueError, IndexError):
                        pass
        return frames

    def pwm_diag(self):
        resp = self.cmd("CMD:PWM_DIAG", wait=0.5)
        for line in resp.split('\n'):
            if 'PWM,' in line:
                return line.strip()
        return ""

    def read_query(self, cmd, wait=0.3):
        resp = self.cmd(cmd, wait=wait)
        for line in resp.split('\n'):
            line = line.strip()
            if line and not line.startswith('>'):
                return line[:200]
        return resp[:200].strip() if resp else ""

    def close(self):
        self.s.close()


def pwm_diag_parse(diag_line):
    d = {}
    if not diag_line:
        return d
    for part in diag_line.split(','):
        if '=' in part:
            k, v = part.split('=', 1)
            try:
                d[k.strip()] = int(v.strip())
            except ValueError:
                d[k.strip()] = v.strip()
    return d


def check_fault(frames):
    for f in frames:
        if f['fault'] != '0x00000000':
            return f['fault']
    return None


def analyze_hold(frames, sref_target):
    """Analyze hold-period frames only."""
    if not frames:
        return {'start_delay_ms': None, 'steady_speed': None, 'speed_error': None,
                'steady_Iq': None, 'steady_Iq_ref': None, 'steady_Vq': None,
                'steady_Id': None, 'n_frames': 0, 'fault': None}

    fault = check_fault(frames)
    speeds = [f['speed'] for f in frames]
    n = len(frames)

    # Start delay: find first frame where |speed| > 0.02 rad/s
    start_idx = None
    for i, s in enumerate(speeds):
        if abs(s) > 0.02:
            start_idx = i
            break
    start_delay_ms = start_idx * 20 if start_idx is not None else None

    # Steady state: last 50 frames of hold
    steady_n = min(50, n)
    if steady_n > 10:
        sf = frames[-steady_n:]
        steady_speed = sum(f['speed'] for f in sf) / steady_n
        speed_error = steady_speed - sref_target
        steady_Iq = sum(f['Iq'] for f in sf) / steady_n
        steady_Iq_ref = sum(f['Iq_ref'] for f in sf) / steady_n
        steady_Vq = sum(f['Vq'] for f in sf) / steady_n
        steady_Id = sum(f['Id'] for f in sf) / steady_n
    else:
        steady_speed = speed_error = steady_Iq = steady_Iq_ref = steady_Vq = steady_Id = None

    return {
        'start_delay_ms': start_delay_ms, 'steady_speed': steady_speed,
        'speed_error': speed_error, 'steady_Iq': steady_Iq,
        'steady_Iq_ref': steady_Iq_ref, 'steady_Vq': steady_Vq,
        'steady_Id': steady_Id, 'n_frames': n, 'fault': fault,
    }


def analyze_zero(frames):
    """Analyze zero-return settle frames. Returns peak |Vq| (V)."""
    if not frames:
        return None
    zf = [f for f in frames if abs(f['speed']) < 0.03]
    if zf:
        return max(abs(f['Vq']) for f in zf)
    return max(abs(f['Vq']) for f in frames[-10:]) if len(frames) >= 10 else None


def setup_baseline(board):
    """Set baseline parameters."""
    print("=== Setting Baseline Parameters ===")
    cmds = [
        ("CMD:VBUS_LIMIT,8,15", "VBUS_LIMIT: 8V/15V"),
        ("CMD:CLEAR_FAULT", "Clear fault"),
        ("CMD:PI_CURRENT,0.50,0", "PI_CURRENT: 0.50/0"),
        ("CMD:PI_SPEED,0.25,0", "PI_SPEED: 0.25/0"),
        ("CMD:RS_FF_MODE,1", "RS_FF: DQ mode"),
        ("CMD:RS_FF_SCALE,0.20", "RS_FF scale: 0.20"),
        ("CMD:RS_FF_ADAPTIVE,0", "RS_FF adaptive: OFF"),
        ("CMD:COG_CFG,0.25,60", "COG: 0.25/60°"),
        ("CMD:BEMF_CFG,0", "BEMF: OFF"),
        ("CMD:KE_TEMP,0", "KE_TEMP: 0"),
        ("CMD:UNLOCK,1", "UNLOCK power stage"),
    ]
    for cmd, desc in cmds:
        resp = board.cmd(cmd, wait=0.2)
        first = resp.split('\n')[0].strip() if resp else ""
        print(f"  {desc:30s} -> {first[:80]}")

    # Verify
    print("\n  Baseline Verification:")
    queries = [
        "CMD:BEMF_CFG?", "CMD:RS_FF_MODE?", "CMD:RS_FF_ADAPTIVE?",
        "CMD:COG_CFG?", "CMD:FAULT_DETAIL"
    ]
    for q in queries:
        resp = board.read_query(q)
        print(f"    {q:25s} -> {resp[:120]}")


def run_stiction_test(board):
    """Run the full stiction test across all SREF values and both directions."""
    all_results = {}

    # Enable in SPEED mode
    board.cmd("CMD:ENABLE,1", wait=0.3)
    board.cmd("CMD:MODE,1", wait=0.3)  # SPEED mode
    board.drain()

    for sref_mag in SREF_VALUES:
        for direction, sign in [("POS", 1.0), ("NEG", -1.0)]:
            sref_val = sref_mag * sign
            label = f"SREF={sref_val:+.2f}"
            print(f"\n{'─'*60}")
            print(f"Testing {label}  ({REPEATS} repeats)")
            print(f"{'─'*60}")

            rep_results = []
            for rep in range(REPEATS):
                # Ensure we're at zero first
                board.cmd("CMD:SREF,0", wait=0.3)
                time.sleep(0.3)

                # Drain and then set SREF
                board.drain()
                board.cmd(f"CMD:SREF,{sref_val:.3f}", wait=0.2)
                time.sleep(0.1)

                # Capture during hold (2s)
                frames_hold = board.capture(duration=HOLD_DURATION)

                # Return to zero and capture settle
                board.cmd("CMD:SREF,0", wait=0.2)
                time.sleep(0.1)
                frames_zero = board.capture(duration=ZERO_DURATION)

                # Analyze hold and zero separately
                result = analyze_hold(frames_hold, sref_val)
                result['zero_Vq_pk'] = analyze_zero(frames_zero)
                result['rep'] = rep + 1

                delay_str = f"{result['start_delay_ms']:.0f}ms" if result['start_delay_ms'] is not None else "N/A"
                spd_str = f"{result['steady_speed']:.3f}" if result['steady_speed'] is not None else "N/A"
                err_str = f"{result['speed_error']*1000:.0f}mrad/s" if result['speed_error'] is not None else "N/A"
                iq_str = f"{result['steady_Iq']*1000:.0f}mA" if result['steady_Iq'] is not None else "N/A"
                vq_str = f"{result['steady_Vq']*1000:.0f}mV" if result['steady_Vq'] is not None else "N/A"
                zvq_str = f"{result['zero_Vq_pk']*1000:.0f}mV" if result['zero_Vq_pk'] is not None else "N/A"
                flt = result['fault'] if result['fault'] else "OK"

                status = ""
                if result['start_delay_ms'] is not None and result['start_delay_ms'] > 300:
                    status += " SLOW_START"
                if result['zero_Vq_pk'] is not None and result['zero_Vq_pk'] > 0.05:
                    status += " VQ_HIGH"

                print(f"  Rep{rep+1}: delay={delay_str:>6s} | speed={spd_str:>7s} err={err_str:>8s} | "
                      f"Iq={iq_str:>6s} | Vq={vq_str:>6s} | zeroVq={zvq_str:>6s} | fault={flt}{status}")

                rep_results.append(result)

                # Check for faults between reps
                if result['fault']:
                    print(f"    ** Fault detected, clearing **")
                    board.cmd("CMD:CLEAR_FAULT", wait=0.3)

            # Get PWM_DIAG snapshot at this SREF for friction breakdown
            board.cmd(f"CMD:SREF,{sref_val:.3f}", wait=0.3)
            time.sleep(1.0)
            diag_raw = board.pwm_diag()
            diag = pwm_diag_parse(diag_raw)
            board.cmd("CMD:SREF,0", wait=0.3)

            print(f"  PWM_DIAG @ {label}: Vqpi={diag.get('Vqpi','?')}mV, "
                  f"Vqb={diag.get('Vqb','?')}mV, "
                  f"Iqref={diag.get('Iqref','?')}mA, "
                  f"Iq={diag.get('Iq','?')}mA")

            # Summarize this (sref, dir)
            delays = [r['start_delay_ms'] for r in rep_results if r['start_delay_ms'] is not None]
            errors = [r['speed_error'] for r in rep_results if r['speed_error'] is not None]
            zero_vqs = [r['zero_Vq_pk'] for r in rep_results if r['zero_Vq_pk'] is not None]
            faults = [r['fault'] for r in rep_results if r['fault']]

            avg_delay = sum(delays) / len(delays) if delays else None
            avg_error = sum(errors) / len(errors) if errors else None
            avg_zero_vq = sum(zero_vqs) / len(zero_vqs) if zero_vqs else None

            all_results[label] = {
                'sref': sref_val,
                'direction': direction,
                'reps': rep_results,
                'avg_delay_ms': avg_delay,
                'avg_speed_error': avg_error,
                'avg_zero_Vq_pk': avg_zero_vq,
                'faults': faults,
                'pwm_diag_snap': diag,
            }

            print(f"  Summary: avg_delay={avg_delay:.0f}ms, avg_err={avg_error*1000:.0f}mrad/s, "
                  f"avg_zeroVq={avg_zero_vq*1000:.0f}mV, faults={len(faults)}")

    return all_results


def print_summary_table(all_results):
    """Print a clean summary table and evaluate decision criteria."""
    print(f"\n{'='*80}")
    print("STICTION TEST SUMMARY")
    print(f"{'='*80}")

    # Header
    print(f"\n{'SREF':>8s} {'Dir':>4s} {'Delay':>7s} {'SpeedErr':>9s} {'Iq':>7s} "
          f"{'Vq':>7s} {'ZeroVq':>7s} {'Faults':>6s}")

    decisions = []

    for label, r in all_results.items():
        sref = r['sref']
        direction = r['direction']
        avg_delay = r['avg_delay_ms']
        avg_error = r['avg_speed_error']
        avg_zero_vq = r['avg_zero_Vq_pk']
        faults = len(r['faults'])

        # Get average Iq and Vq from reps
        iqs = [rep['steady_Iq'] for rep in r['reps'] if rep['steady_Iq'] is not None]
        vqs = [rep['steady_Vq'] for rep in r['reps'] if rep['steady_Vq'] is not None]
        avg_iq = sum(iqs)/len(iqs)*1000 if iqs else 0  # mA
        avg_vq = sum(vqs)/len(vqs)*1000 if vqs else 0  # mV

        delay_s = f"{avg_delay:.0f}ms" if avg_delay is not None else "N/A"
        err_s = f"{avg_error*1000:.0f}m" if avg_error is not None else "N/A"
        zvq_s = f"{avg_zero_vq*1000:.0f}mV" if avg_zero_vq is not None else "N/A"

        # Decision flags
        flags = []
        if avg_delay is not None and avg_delay > 300 and abs(sref) >= 0.10:
            flags.append("SLOW")
        if avg_error is not None and abs(avg_error) > 0.05 and abs(sref) >= 0.10:
            flags.append("ERR")
        if avg_zero_vq is not None and avg_zero_vq > 0.05:
            flags.append("VQ>50mV")

        print(f"{sref:+.2f}  {direction:>4s} {delay_s:>7s} {err_s:>9s} "
              f"{avg_iq:>5.0f}mA {avg_vq:>5.0f}mV {zvq_s:>7s} {faults:>6d} "
              f"{'WARN '+','.join(flags) if flags else 'OK'}")

        if flags:
            decisions.append((abs(sref), direction, flags))

    # Decision criteria evaluation
    print(f"\n{'─'*80}")
    print("DECISION CRITERIA EVALUATION")
    print(f"{'─'*80}")

    # 1. Minimum sustainable speed
    min_sustained_pos = None
    min_sustained_neg = None
    for label, r in all_results.items():
        sref = r['sref']
        if r['avg_delay_ms'] is not None and r['avg_delay_ms'] < 300:
            delays_ok = all(rep['start_delay_ms'] is not None and rep['start_delay_ms'] < 300
                          for rep in r['reps'] if rep['start_delay_ms'] is not None)
            if delays_ok and len(r['faults']) == 0:
                if sref > 0 and (min_sustained_pos is None or sref < min_sustained_pos):
                    min_sustained_pos = sref
                if sref < 0 and (min_sustained_neg is None or abs(sref) < abs(min_sustained_neg)):
                    min_sustained_neg = sref

    print(f"  Min sustained POS: {min_sustained_pos:.2f} rad/s" if min_sustained_pos else "  Min sustained POS: N/A")
    print(f"  Min sustained NEG: {min_sustained_neg:.2f} rad/s" if min_sustained_neg else "  Min sustained NEG: N/A")

    # 2. Symmetry check at 0.10 rad/s
    sref_010_pos = all_results.get('SREF=+0.10', {})
    sref_010_neg = all_results.get('SREF=-0.10', {})
    if sref_010_pos and sref_010_neg:
        pos_err = abs(sref_010_pos.get('avg_speed_error', 0) or 0)
        neg_err = abs(sref_010_neg.get('avg_speed_error', 0) or 0)
        asym = abs(pos_err - neg_err)
        print(f"  SREF=+-0.10 asymmetry: {asym*1000:.0f} mrad/s {'PASS' if asym < 0.05 else 'WARN >50mrad/s'}")

    # 3. Return-to-zero Vq
    all_zero_vq = [r['avg_zero_Vq_pk'] for r in all_results.values() if r['avg_zero_Vq_pk'] is not None]
    if all_zero_vq:
        max_zvq = max(all_zero_vq)
        print(f"  Max return-to-zero Vq: {max_zvq*1000:.0f}mV {'PASS' if max_zvq < 0.05 else 'WARN >50mV'}")

    # 4. Overall verdict
    print(f"\n{'─'*80}")
    critical_failures = [d for d in decisions if d[0] >= 0.10]
    if not critical_failures and (min_sustained_pos is not None and min_sustained_pos <= 0.10
                                   and min_sustained_neg is not None and abs(min_sustained_neg) <= 0.10):
        print("  VERDICT: PASS — Existing friction compensation is sufficient.")
        print("  Action: No code changes. Proceed to load inertia / performance testing.")
    else:
        print("  VERDICT: FAIL — Adaptive breakaway may be needed.")
        print(f"  Critical failures at: {[(d[0], d[1]) for d in critical_failures]}")
        print("  Next: implement adaptive breakaway (see test plan for spec).")

    return critical_failures


def main():
    print("=" * 80)
    print("STICTION / LOW-SPEED VERIFICATION TEST")
    print(f"Started: {datetime.now().isoformat()}")
    print("=" * 80)

    board = FOCBoard()

    try:
        # Baseline setup
        print("\n### BASELINE SETUP ###")
        setup_baseline(board)

        # Check initial fault
        resp = board.cmd("CMD:FAULT?", wait=0.3)
        print(f"\n  Initial fault: {resp.split(chr(10))[0].strip() if resp else 'N/A'}")

        # Run test
        all_results = run_stiction_test(board)

        # Print summary
        critical_failures = print_summary_table(all_results)

        # Final: disable
        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.cmd("CMD:ENABLE,0")

        # Save results
        output_file = f"E:/24V_FOC_Controller_sync_20260519/scripts/stiction_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        save_data = {
            'test': 'stiction_verification',
            'baseline': 'PI_CURRENT=0.50/0, PI_SPEED=0.25/0, RS_FF=DQ/0.20, COG=0.25/60, BEMF=OFF',
            'results': {},
            'verdict': 'PASS' if not critical_failures else 'FAIL',
        }
        for label, r in all_results.items():
            save_data['results'][label] = {
                'sref': r['sref'],
                'direction': r['direction'],
                'avg_delay_ms': r['avg_delay_ms'],
                'avg_speed_error': r['avg_speed_error'],
                'avg_zero_Vq_pk': r['avg_zero_Vq_pk'],
                'faults': r['faults'],
                'reps': [{k: v for k, v in rep.items() if k != 'fault'}
                         for rep in r['reps']],
                'pwm_diag_snap': r['pwm_diag_snap'],
            }
        with open(output_file, 'w') as f:
            json.dump(save_data, f, indent=2, default=str)
        print(f"\nResults saved to: {output_file}")

    finally:
        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.cmd("CMD:ENABLE,0")
        board.close()


if __name__ == '__main__':
    main()
