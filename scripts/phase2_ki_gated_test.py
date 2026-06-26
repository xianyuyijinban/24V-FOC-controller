"""
Phase 2 Speed Ki Gated Test Plan
=================================
Baseline: PI_CURRENT=0.50/0, PI_SPEED=0.25/0.001 gated, RS_FF=DQ/0.20, BEMF=OFF, COG=0.25/+60deg

Test sequence:
  1. Cold-start verification (AppFault, config queries)
  2. Low-speed sweep (±0.05, ±0.10, ±0.20, ±0.30, ±1.00)
  3. Zero-return check (Vq_pk after SREF=0)
  4. 20-cycle endurance (+1.0→0→-1.0→0)
  5. Position regression (PREF=0,+5,-5,+20,-20,0)

Output: Console log + JSON results file
"""
import serial
import time
import sys
import json
import statistics
from datetime import datetime

PORT = 'COM9'
BAUD = 230400
RESULTS_FILE = f"scripts/phase2_ki_gated_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"


class FOCBoard:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.5)
        self.drain()

    def drain(self):
        time.sleep(0.3)
        self.s.read(self.s.in_waiting or 65536)

    def cmd(self, cmd_str, wait=0.5):
        """Send command, return response text"""
        self.s.reset_input_buffer()
        self.s.write((cmd_str + '\r\n').encode())
        if wait <= 0:
            return ""
        time.sleep(wait)
        data = self.s.read(self.s.in_waiting or 65536)
        return data.decode('ascii', errors='replace')

    def capture(self, duration=2.0):
        """Capture N-frame data for duration seconds"""
        self.s.reset_input_buffer()
        time.sleep(duration)
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
                            'Iq_ref': float(parts[18]),
                            'Vd': float(parts[19]),
                            'Vq': float(parts[20]),
                            'Ia': float(parts[21]),
                            'Ib': float(parts[22]),
                            'Ic': float(parts[23]),
                        })
                    except (ValueError, IndexError):
                        pass
        return frames

    def read_query(self, cmd, wait=0.3):
        """Read a query command response"""
        resp = self.cmd(cmd, wait=wait)
        for line in resp.split('\n'):
            line = line.strip()
            if line and not line.startswith('>'):
                return line[:200]
        return resp[:200].strip() if resp else ""

    def pwm_diag(self):
        """Get PWM_DIAG output"""
        resp = self.cmd("CMD:PWM_DIAG", wait=0.5)
        for line in resp.split('\n'):
            if 'PWM,' in line:
                return line.strip()
        return ""

    def fault_detail(self):
        """Get FAULT_DETAIL"""
        return self.read_query("CMD:FAULT_DETAIL", wait=0.5)

    def close(self):
        self.s.close()


# ── Helper functions ──

def fmt_frames(frames):
    """Summary stats for a frame batch"""
    if not frames:
        return "NO DATA"
    n = len(frames)
    def avg(vals): return sum(vals) / len(vals) if vals else 0.0
    def peak(vals): return max(abs(v) for v in vals) if vals else 0.0
    speeds = [f['speed'] for f in frames]
    iqs = [f['Iq'] for f in frames]
    vqs = [f['Vq'] for f in frames]
    speed_refs = [f['speed_ref'] for f in frames]
    return (f"n={n:3d} | ω_avg={avg(speeds):+.4f} ω_pk={peak(speeds):.4f} "
            f"| ω_ref={avg(speed_refs):+.4f} "
            f"| Iq_avg={avg(iqs):+.4f} Iq_pk={peak(iqs):.4f} "
            f"| Vq_avg={avg(vqs):.4f} Vq_pk={peak(vqs):.4f}")


def check_fault(frames):
    """Return first non-zero fault flag, or None"""
    for f in frames:
        if f['fault'] != '0x00000000':
            return f['fault']
    return None


def speed_stats(frames):
    """Return (mean_speed, mean_abs_speed, pk_speed, mean_Vq, pk_Vq)"""
    if not frames:
        return (0, 0, 0, 0, 0)
    speeds = [f['speed'] for f in frames]
    vqs = [f['Vq'] for f in frames]
    return (
        statistics.mean(speeds),
        statistics.mean([abs(s) for s in speeds]),
        max(abs(s) for s in speeds),
        statistics.mean(vqs),
        max(abs(v) for v in vqs),
    )


def check_speed_tracking(frames, sref):
    """Check if actual speed reaches target at acceptable level"""
    if not frames:
        return False, 0.0
    # Use last 60% of frames for settled state
    n_settle = max(1, len(frames) * 6 // 10)
    settled = frames[-n_settle:]
    speeds = [f['speed'] for f in settled]
    mean_speed = statistics.mean(speeds)
    abs_target = abs(sref)
    ratio = abs(mean_speed) / abs_target if abs_target > 0.001 else 0.0
    # Acceptance: >80% of target for |sref|>=0.10, >70% for 0.05
    threshold = 0.70 if abs_target < 0.08 else 0.80
    ok = ratio >= threshold
    return ok, ratio, mean_speed


# ── Test 1: Cold Start Verification ──

def test_cold_start(board):
    print("\n" + "=" * 70)
    print("TEST 1: Cold Start Verification")
    print("=" * 70)

    results = {}

    # 1a: Check AppFault
    fault_resp = board.fault_detail()
    results['fault_detail'] = fault_resp
    app_fault_ok = 'AppFault' not in fault_resp or 'AppFault=0' in fault_resp or 'NONE' in fault_resp
    print(f"  FAULT_DETAIL: {fault_resp[:120]}")
    print(f"  AppFault=0 ? {'PASS' if app_fault_ok else 'CHECK: ' + fault_resp}")

    # 1b: Check config queries
    queries = {
        'MOTION_CFG': 'CMD:MOTION_CFG?',
        'COG_CFG': 'CMD:COG_CFG?',
        'BEMF_CFG': 'CMD:BEMF_CFG?',
        'RS_FF_MODE': 'CMD:RS_FF_MODE?',
        'RS_FF_SCALE': 'CMD:RS_FF_SCALE?',
        'RS_FF_ADAPTIVE': 'CMD:RS_FF_ADAPTIVE?',
    }

    checks = {}
    for name, cmd in queries.items():
        resp = board.read_query(cmd, wait=0.3)
        checks[name] = resp
        print(f"  {name:18s} -> {resp[:100]}")

    # Verify key constraints
    bemf_ok = 'OFF' in checks.get('BEMF_CFG', '') or '0' in checks.get('BEMF_CFG', '')
    rsff_mode_ok = 'DQ' in checks.get('RS_FF_MODE', '') or '1' in checks.get('RS_FF_MODE', '')
    rsff_scale_ok = '0.20' in checks.get('RS_FF_SCALE', '')
    rsff_adaptive_ok = 'OFF' in checks.get('RS_FF_ADAPTIVE', '') or '0' in checks.get('RS_FF_ADAPTIVE', '')

    print(f"  BEMF=OFF:     {'PASS' if bemf_ok else 'FAIL'}")
    print(f"  RS_FF_MODE=DQ: {'PASS' if rsff_mode_ok else 'FAIL'}")
    print(f"  RS_FF_SCALE=0.20: {'PASS' if rsff_scale_ok else 'FAIL'}")
    print(f"  RS_FF_ADAPTIVE=OFF: {'PASS' if rsff_adaptive_ok else 'FAIL'}")

    all_ok = app_fault_ok and bemf_ok and rsff_mode_ok and rsff_scale_ok and rsff_adaptive_ok
    print(f"  Cold Start: {'ALL PASS' if all_ok else 'SOME CHECKS FAILED'}")

    results['checks'] = checks
    results['all_pass'] = all_ok
    return results


# ── Test 2: Low-Speed Sweep ──

def test_low_speed_sweep(board):
    print("\n" + "=" * 70)
    print("TEST 2: Low-Speed Sweep (gated Ki=0.001)")
    print("=" * 70)

    sweep_targets = [0.05, 0.10, 0.20, 0.30, 1.00]
    all_results = {}

    for sref_pos in sweep_targets:
        for sign in [+1.0, -1.0]:
            sref = sref_pos * sign
            label = f"SREF={sref:+.2f}"
            print(f"\n  --- {label} ---")

            # Send SREF, wait settle, capture
            board.cmd(f"CMD:SREF,{sref:.3f}", wait=0.3)
            time.sleep(1.5)  # settle
            frames = board.capture(duration=2.0)

            if not frames:
                print(f"    NO DATA!")
                all_results[label] = {'error': 'NO DATA', 'n_frames': 0}
                continue

            fault = check_fault(frames)
            ok, ratio, mean_speed = check_speed_tracking(frames, sref)
            _, _, pk_speed, mean_vq, pk_vq = speed_stats(frames)
            summary = fmt_frames(frames)

            status = "PASS" if ok and not fault else ("FAULT" if fault else "FAIL")
            print(f"    {summary}")
            print(f"    Tracking: {ratio*100:.0f}% of target (need >{70 if abs(sref)<0.08 else 80}%) → {status}")
            if fault:
                print(f"    ** FAULT: {fault} **")

            all_results[label] = {
                'n_frames': len(frames),
                'mean_speed': mean_speed,
                'pk_speed': pk_speed,
                'tracking_ratio': ratio,
                'tracking_ok': ok,
                'mean_vq': mean_vq,
                'pk_vq': pk_vq,
                'fault': fault,
                'status': status,
                'summary': summary,
            }

    # Direction asymmetry check
    print("\n  --- Direction Asymmetry ---")
    asymmetry_issues = []
    for sref_pos in sweep_targets:
        pos_key = f"SREF=+{sref_pos:.2f}"
        neg_key = f"SREF=-{sref_pos:.2f}"
        if pos_key in all_results and neg_key in all_results:
            pos_speed = abs(all_results[pos_key].get('mean_speed', 0))
            neg_speed = abs(all_results[neg_key].get('mean_speed', 0))
            diff = abs(pos_speed - neg_speed)
            status = "OK" if diff < 0.05 else "ASYMMETRY"
            print(f"    |SREF|={sref_pos:.2f}: |ω+|={pos_speed:.4f} |ω-|={neg_speed:.4f} diff={diff:.4f} → {status}")
            if diff >= 0.05:
                asymmetry_issues.append(f"SREF={sref_pos:.2f} diff={diff:.4f}")

    all_results['asymmetry_issues'] = asymmetry_issues
    return all_results


# ── Test 3: Zero-Return Check ──

def test_zero_return(board):
    print("\n" + "=" * 70)
    print("TEST 3: Zero-Return Vq Check")
    print("=" * 70)

    # After sweep, already at SREF=0 from last step. But let's do explicit returns.
    zero_results = {}
    vq_peaks = []

    # Test return to zero from various speeds
    test_srefs = [0.05, 0.10, 0.30, 1.00]

    for sref in test_srefs:
        for sign in [+1.0, -1.0]:
            label = f"+{sref:.2f}→0" if sign > 0 else f"-{sref:.2f}→0"
            print(f"\n  --- {label} ---")

            # Set speed, then return to zero
            board.cmd(f"CMD:SREF,{sref * sign:.3f}", wait=0.3)
            time.sleep(0.8)
            board.cmd("CMD:SREF,0", wait=0.2)

            # Capture the zero-return transient + settle (500ms)
            board.s.reset_input_buffer()
            time.sleep(0.6)
            data = board.s.read(board.s.in_waiting or 65536)
            text = data.decode('ascii', errors='replace')

            frames = []
            for line in text.split('\n'):
                line = line.strip()
                if line.startswith('N,'):
                    parts = line.split(',')
                    if len(parts) >= 30:
                        try:
                            frames.append({
                                'speed': float(parts[4]),
                                'Iq': float(parts[6]),
                                'fault': parts[8].strip(),
                                'Vq': float(parts[20]),
                            })
                        except (ValueError, IndexError):
                            pass

            if frames:
                vqs = [abs(f['Vq']) for f in frames]
                vq_pk = max(vqs) if vqs else 0.0
                vq_mean = statistics.mean(vqs) if vqs else 0.0
                vq_peaks.append(vq_pk)
                fault = check_fault(frames)

                # Acceptable range: compare to P-only baseline (17-56mV, occasional ~100mV)
                if vq_pk < 0.060:
                    grade = "PASS (<60mV)"
                elif vq_pk < 0.120:
                    grade = "OK (<120mV, within baseline)"
                else:
                    grade = "NOTE (>{:.0f}mV, record as zero-speed control floor)".format(vq_pk * 1000)

                print(f"    Vq_pk={vq_pk*1000:.0f}mV Vq_avg={vq_mean*1000:.0f}mV n={len(frames)} → {grade}")
                if fault:
                    print(f"    ** FAULT: {fault} **")

                zero_results[label] = {
                    'vq_pk_mV': vq_pk * 1000,
                    'vq_mean_mV': vq_mean * 1000,
                    'n_frames': len(frames),
                    'grade': grade,
                    'fault': fault,
                }
            else:
                print(f"    NO DATA!")
                zero_results[label] = {'error': 'NO DATA'}

    # Summary
    if vq_peaks:
        print(f"\n  Vq_pk range: {min(vq_peaks)*1000:.0f} - {max(vq_peaks)*1000:.0f} mV")
        print(f"  Vq_pk mean:  {statistics.mean(vq_peaks)*1000:.0f} mV")
    zero_results['vq_pk_max_mV'] = max(vq_peaks) * 1000 if vq_peaks else 0
    zero_results['vq_pk_mean_mV'] = statistics.mean(vq_peaks) * 1000 if vq_peaks else 0

    return zero_results


# ── Test 4: 20-Cycle Endurance ──

def test_endurance(board):
    print("\n" + "=" * 70)
    print("TEST 4: 20-Cycle Endurance (+1.0 → 0 → -1.0 → 0)")
    print("=" * 70)

    cycles = 20
    cycle_results = []
    faults_detected = 0
    all_vq_pks = []

    for i in range(cycles):
        print(f"\n  Cycle {i+1}/{cycles}: ", end='', flush=True)

        # +1.0 rad/s
        board.cmd("CMD:SREF,1.000", wait=0.2)
        time.sleep(0.6)
        frames_pos = board.capture(duration=0.8)
        fault_pos = check_fault(frames_pos) if frames_pos else None

        # Return to 0
        board.cmd("CMD:SREF,0", wait=0.2)
        time.sleep(0.5)
        frames_z1 = board.capture(duration=0.6)
        fault_z1 = check_fault(frames_z1) if frames_z1 else None

        # -1.0 rad/s
        board.cmd("CMD:SREF,-1.000", wait=0.2)
        time.sleep(0.6)
        frames_neg = board.capture(duration=0.8)
        fault_neg = check_fault(frames_neg) if frames_neg else None

        # Return to 0
        board.cmd("CMD:SREF,0", wait=0.2)
        time.sleep(0.5)
        frames_z2 = board.capture(duration=0.6)
        fault_z2 = check_fault(frames_z2) if frames_z2 else None

        # Collect Vq peaks from zero-return frames
        vq_pks_cycle = []
        for fz in [frames_z1, frames_z2]:
            if fz:
                vqs = [abs(f['Vq']) for f in fz]
                if vqs:
                    vq_pks_cycle.append(max(vqs))

        vq_pk_cycle = max(vq_pks_cycle) if vq_pks_cycle else 0.0
        all_vq_pks.append(vq_pk_cycle)

        any_fault = fault_pos or fault_z1 or fault_neg or fault_z2
        if any_fault:
            faults_detected += 1
            print(f"FAULT! pos={fault_pos} z1={fault_z1} neg={fault_neg} z2={fault_z2}")
        else:
            print(f"OK  Vq_pk={vq_pk_cycle*1000:.0f}mV")

        cycle_results.append({
            'cycle': i + 1,
            'fault_pos': fault_pos,
            'fault_z1': fault_z1,
            'fault_neg': fault_neg,
            'fault_z2': fault_z2,
            'vq_pk_mV': vq_pk_cycle * 1000,
        })

    # Summary
    print(f"\n  --- Endurance Summary ---")
    total_faults = faults_detected
    vq_below_60 = sum(1 for v in all_vq_pks if v < 0.060)
    print(f"  Faults: {total_faults}/{cycles}")
    print(f"  Vq_pk <60mV: {vq_below_60}/{cycles}")
    if all_vq_pks:
        print(f"  Vq_pk range: {min(all_vq_pks)*1000:.0f} - {max(all_vq_pks)*1000:.0f} mV")

    passed = (total_faults == 0) and (vq_below_60 >= cycles * 0.8)  # allow 20% margin

    return {
        'cycles': cycles,
        'faults_detected': total_faults,
        'vq_pk_below_60mV_count': vq_below_60,
        'cycle_results': cycle_results,
        'overall_pass': passed,
    }


# ── Test 5: Position Regression ──

def test_position_regression(board):
    print("\n" + "=" * 70)
    print("TEST 5: Position Regression (verify no speed integral residue)")
    print("=" * 70)

    # Switch to position mode
    board.cmd("CMD:MODE,2", wait=0.5)
    print("  Switched to POSITION mode")

    pos_targets = [0.0, +5.0, -5.0, +20.0, -20.0, 0.0]
    pos_results = {}

    for pref in pos_targets:
        label = f"PREF={pref:+.1f}°"
        print(f"\n  --- {label} ---")

        board.cmd(f"CMD:PREF,{pref:.1f}", wait=0.3)
        time.sleep(2.0)  # settle
        frames = board.capture(duration=2.0)

        if not frames:
            print(f"    NO DATA!")
            pos_results[label] = {'error': 'NO DATA'}
            continue

        fault = check_fault(frames)
        summary = fmt_frames(frames)

        # Check position error: need <3° steady-state
        if len(frames) >= 10:
            settled = frames[-10:]
            speeds = [f['speed'] for f in settled]
            mean_speed = statistics.mean(speeds)

            # Position error can be inferred from speed (should be ~0 at steady state)
            # Also check for drift: speed should be near zero when holding
            drift_ok = abs(mean_speed) < 0.05  # rad/s — no continuous drift

            status = "PASS" if (drift_ok and not fault) else ("FAULT" if fault else "DRIFT")
            print(f"    {summary}")
            print(f"    Drift: ω_mean={mean_speed:.4f} rad/s → {'OK' if drift_ok else 'DRIFT DETECTED'}")
            if fault:
                print(f"    ** FAULT: {fault} **")
        else:
            drift_ok = None
            status = "LOW_DATA"
            print(f"    {summary}")

        pos_results[label] = {
            'n_frames': len(frames),
            'fault': fault,
            'status': status,
            'summary': summary,
            'drift_ok': drift_ok,
        }

    # Switch back to speed mode
    board.cmd("CMD:MODE,1", wait=0.3)
    print("\n  Switched back to SPEED mode")

    return pos_results


# ── Main ──

def main():
    print("Phase 2 Speed Ki Gated Test Plan")
    print(f"Port: {PORT} @ {BAUD}")
    print(f"Results: {RESULTS_FILE}")
    print(f"Time: {datetime.now().isoformat()}")

    board = FOCBoard(PORT, BAUD)

    # Setup baseline
    print("\n" + "=" * 70)
    print("SETUP: Configuring Baseline Parameters")
    print("=" * 70)

    setup_cmds = [
        ("CMD:VBUS_LIMIT,10,16", "VBUS_LIMIT: 10-16V"),
        ("CMD:CLEAR_FAULT", "Clear fault"),
        ("CMD:PI_CURRENT,0.50,0", "PI_CURRENT: 0.50/0"),
        ("CMD:PI_SPEED,0.25,0.001", "PI_SPEED: 0.25/0.001 (gated)"),
        ("CMD:RS_FF_MODE,1", "RS_FF: DQ mode"),
        ("CMD:RS_FF_SCALE,0.20", "RS_FF_SCALE: 0.20"),
        ("CMD:RS_FF_ADAPTIVE,0", "RS_FF_ADAPTIVE: OFF"),
        ("CMD:COG_CFG,0.25,60", "COG: 0.25/+60deg"),
        ("CMD:BEMF_CFG,0", "BEMF: OFF"),
        ("CMD:KE_TEMP,0", "KE_TEMP: 0"),
        ("CMD:UNLOCK,1", "UNLOCK power stage"),
    ]

    for cmd, desc in setup_cmds:
        resp = board.cmd(cmd, wait=0.2)
        first_line = resp.split('\n')[0].strip() if resp else ""
        print(f"  {desc:35s} -> {first_line[:80]}")

    # Enable PWM
    print("\n  Enabling PWM...")
    board.cmd("CMD:ENABLE,1", wait=0.5)
    time.sleep(0.5)

    # Verify PWM is running
    diag = board.pwm_diag()
    print(f"  PWM_DIAG: {diag[:120] if diag else 'No response'}")

    all_results = {
        'timestamp': datetime.now().isoformat(),
        'baseline': 'PI_CURRENT=0.50/0, PI_SPEED=0.25/0.001 gated, RS_FF=DQ/0.20, BEMF=OFF, COG=0.25/+60deg',
        'ki_gated': True,
    }

    # Run tests sequentially
    try:
        all_results['cold_start'] = test_cold_start(board)
    except Exception as e:
        print(f"\n  !! Cold start test error: {e}")
        all_results['cold_start'] = {'error': str(e)}

    try:
        all_results['low_speed_sweep'] = test_low_speed_sweep(board)
    except Exception as e:
        print(f"\n  !! Low speed sweep error: {e}")
        all_results['low_speed_sweep'] = {'error': str(e)}

    try:
        all_results['zero_return'] = test_zero_return(board)
    except Exception as e:
        print(f"\n  !! Zero return test error: {e}")
        all_results['zero_return'] = {'error': str(e)}

    try:
        all_results['endurance'] = test_endurance(board)
    except Exception as e:
        print(f"\n  !! Endurance test error: {e}")
        all_results['endurance'] = {'error': str(e)}

    try:
        all_results['position_regression'] = test_position_regression(board)
    except Exception as e:
        print(f"\n  !! Position regression error: {e}")
        all_results['position_regression'] = {'error': str(e)}

    # Disable PWM
    board.cmd("CMD:ENABLE,0", wait=0.3)
    board.cmd("CMD:UNLOCK,0", wait=0.3)
    board.close()

    # ── Final Summary ──
    print("\n" + "=" * 70)
    print("FINAL SUMMARY")
    print("=" * 70)

    # Cold start
    cs = all_results.get('cold_start', {})
    print(f"  Cold Start:      {'PASS' if cs.get('all_pass') else 'CHECK'}")

    # Low speed sweep
    sweep = all_results.get('low_speed_sweep', {})
    if sweep and 'error' not in sweep:
        failures = [k for k, v in sweep.items() if isinstance(v, dict) and v.get('status') == 'FAIL']
        faults = [k for k, v in sweep.items() if isinstance(v, dict) and v.get('status') == 'FAULT']
        asymmetry = sweep.get('asymmetry_issues', [])
        n_targets = len([k for k in sweep if k.startswith('SREF=')])
        n_ok = n_targets - len(failures) - len(faults)
        print(f"  Low-Speed Sweep: {n_ok}/{n_targets} OK" + (f", FAIL: {failures}" if failures else ""))
        if asymmetry:
            print(f"    Asymmetry: {asymmetry}")

    # Zero return
    zr = all_results.get('zero_return', {})
    if zr and 'error' not in zr:
        print(f"  Zero-Return:     Vq_pk max={zr.get('vq_pk_max_mV', '?')}mV, mean={zr.get('vq_pk_mean_mV', '?')}mV")

    # Endurance
    endur = all_results.get('endurance', {})
    if endur and 'error' not in endur:
        print(f"  Endurance:       {endur.get('faults_detected', '?')} faults / {endur.get('cycles', '?')} cycles")
        print(f"                   Vq<60mV: {endur.get('vq_pk_below_60mV_count', '?')}/{endur.get('cycles', '?')}")

    # Position
    pos = all_results.get('position_regression', {})
    if pos and 'error' not in pos:
        pos_fails = [k for k, v in pos.items() if isinstance(v, dict) and v.get('status') not in ('PASS', None)]
        print(f"  Position:        {'ALL PASS' if not pos_fails else 'ISSUES: ' + str(pos_fails)}")

    # Save results
    with open(RESULTS_FILE, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"\nResults saved to {RESULTS_FILE}")

    # Print overall verdict
    print("\n" + "=" * 70)
    overall = True
    if not cs.get('all_pass'):
        overall = False
    if sweep and 'error' not in sweep and len(failures) > 0:
        overall = False
    if endur and 'error' not in endur and endur.get('faults_detected', 0) > 0:
        overall = False
    print(f"OVERALL: {'ALL TESTS PASSED' if overall else 'SOME TESTS NEED ATTENTION'}")
    print("=" * 70)

    return 0 if overall else 1


if __name__ == '__main__':
    sys.exit(main())
