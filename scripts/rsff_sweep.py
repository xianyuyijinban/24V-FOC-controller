"""
RsFF Scale Sweep Test
Baseline: PI_CURRENT=0.50/0, PI_SPEED=0.25/0, BEMF=OFF, COG=0.25/60°
Tests RS_FF_MODE=1 (DQ) with scales 0 (baseline), 0.10, 0.20, 0.30, 0.40
"""
import serial
import time
import sys
import json
from datetime import datetime

PORT = 'COM9'
BAUD = 230400

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
        """Capture telemetry frames for duration seconds"""
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
        """Get PWM_DIAG for RsFF/PI breakdown"""
        resp = self.cmd("CMD:PWM_DIAG", wait=0.5)
        for line in resp.split('\n'):
            if 'PWM,' in line:
                return line.strip()
        return ""

    def close(self):
        self.s.close()


def fmt_frames(frames):
    """Summarize a frame list"""
    if not frames:
        return "NO DATA"
    n = len(frames)
    def avg(vals): return sum(vals)/len(vals) if vals else 0
    def peak(vals): return max(abs(v) for v in vals) if vals else 0

    speeds = [f['speed'] for f in frames]
    iqs = [f['Iq'] for f in frames]
    ids = [f['Id'] for f in frames]
    vds = [f['Vd'] for f in frames]
    vqs = [f['Vq'] for f in frames]
    iq_refs = [f['Iq_ref'] for f in frames]

    return (f"n={n} | "
            f"speed: {avg(speeds):.3f} pk={peak(speeds):.3f} | "
            f"Iq: {avg(iqs):.4f} pk={peak(iqs):.4f} | "
            f"Id: {avg(ids):.4f} pk={peak(ids):.4f} | "
            f"Vd: {avg(vds):.4f} pk={peak(vds):.4f} | "
            f"Vq: {avg(vqs):.4f} pk={peak(vqs):.4f} | "
            f"Iq_ref: {avg(iq_refs):.4f}")

def check_fault(frames):
    for f in frames:
        if f['fault'] != '0x00000000':
            return f['fault']
    return None


def setup_baseline(board, rsff_scale=None):
    """Set baseline parameters. If rsff_scale is None, RS_FF=OFF"""
    print("=== Setting Baseline Parameters ===")
    cmds = [
        "CMD:CLEAR_FAULT",
        "CMD:VBUS_LIMIT,8,15",
        "CMD:BEMF_CFG,0",
        "CMD:PI_CURRENT,0.50,0",
        "CMD:PI_SPEED,0.25,0",
        "CMD:COG_CFG,0.25,60",
        "CMD:RS_FF_ADAPTIVE,0",
        "CMD:UNLOCK,1",
    ]
    if rsff_scale is None:
        cmds.append("CMD:RS_FF_MODE,0")
        print("  RS_FF: OFF")
    else:
        cmds.append("CMD:RS_FF_MODE,1")
        cmds.append(f"CMD:RS_FF_SCALE,{rsff_scale:.2f}")
        print(f"  RS_FF: DQ mode, scale={rsff_scale:.2f}")

    for cmd in cmds:
        resp = board.cmd(cmd, wait=0.2)
        # Print just first line of response
        first_line = resp.split('\n')[0].strip() if resp else ""
        print(f"  {cmd} -> {first_line[:80]}")

    # Verify RS_FF settings
    print("\n  Query RS_FF state:")
    for q in ["CMD:RS_FF_MODE?", "CMD:RS_FF_ADAPTIVE?"]:
        resp = board.cmd(q, wait=0.2)
        first_line = resp.split('\n')[0].strip() if resp else ""
        print(f"    {q} -> {first_line[:80]}")


def run_sref_sweep(board, label):
    """SREF sweep: 0→+0.5→0→-0.5→0→+1.0→0→-1.0→0"""
    print(f"\n{'='*60}")
    print(f"SREF Sweep: {label}")
    print(f"{'='*60}")

    results = {}
    steps = [
        ("SREF=0 (idle)", "CMD:SREF,0", 2.0),
        ("SREF=+0.5", "CMD:SREF,0.5", 2.0),
        ("SREF=0", "CMD:SREF,0", 2.0),
        ("SREF=-0.5", "CMD:SREF,-0.5", 2.0),
        ("SREF=0", "CMD:SREF,0", 2.0),
        ("SREF=+1.0", "CMD:SREF,1.0", 2.0),
        ("SREF=0", "CMD:SREF,0", 2.0),
        ("SREF=-1.0", "CMD:SREF,-1.0", 2.0),
        ("SREF=0", "CMD:SREF,0", 3.0),  # longer settle at end
    ]

    for name, cmd, dur in steps:
        print(f"\n--- {name} ---")
        board.cmd(cmd, wait=0.3)
        time.sleep(0.5)  # Let system settle
        frames = board.capture(duration=dur)
        fault = check_fault(frames)
        summary = fmt_frames(frames)
        print(f"  {summary}")
        if fault:
            print(f"  ** FAULT: {fault} **")
        results[name] = {
            'cmd': cmd,
            'n_frames': len(frames),
            'fault': fault,
            'summary': summary,
        }

    return results


def run_pref_sweep(board, label):
    """PREF sweep: 0→+5°→-5°→+20°→-20°→0"""
    print(f"\n{'='*60}")
    print(f"PREF Sweep: {label}")
    print(f"{'='*60}")

    results = {}
    steps = [
        ("PREF=0 (hold)", "CMD:PREF,0", 2.0),
        ("PREF=+5°", "CMD:PREF,5", 2.0),
        ("PREF=-5°", "CMD:PREF,-5", 2.0),
        ("PREF=+20°", "CMD:PREF,20", 2.0),
        ("PREF=-20°", "CMD:PREF,-20", 2.0),
        ("PREF=0", "CMD:PREF,0", 3.0),
    ]

    for name, cmd, dur in steps:
        print(f"\n--- {name} ---")
        board.cmd(cmd, wait=0.3)
        time.sleep(0.5)
        frames = board.capture(duration=dur)
        fault = check_fault(frames)
        summary = fmt_frames(frames)
        print(f"  {summary}")
        if fault:
            print(f"  ** FAULT: {fault} **")
        results[name] = {
            'cmd': cmd,
            'n_frames': len(frames),
            'fault': fault,
            'summary': summary,
        }

    return results


def run_single_test(board, rsff_scale, label):
    """Run full test (SREF + PREF) for one RS_FF_SCALE"""
    print(f"\n{'#'*60}")
    print(f"# TEST: {label}")
    print(f"{'#'*60}")

    # Setup
    setup_baseline(board, rsff_scale)

    # Enable motor
    board.cmd("CMD:ENABLE,1", wait=0.3)
    board.drain()

    # Get PWM_DIAG at idle
    diag = board.pwm_diag()
    print(f"\n  PWM_DIAG (idle): {diag[:120]}")

    # SREF sweep in speed mode
    board.cmd("CMD:MODE,2")
    board.drain()
    sref_results = run_sref_sweep(board, label)

    # PREF sweep in position mode
    board.cmd("CMD:MODE,3")
    board.drain()
    pref_results = run_pref_sweep(board, label)

    # Get PWM_DIAG after sweeps
    diag2 = board.pwm_diag()
    print(f"\n  PWM_DIAG (post-sweep): {diag2[:120]}")

    # Disable motor
    board.cmd("CMD:SREF,0")
    board.cmd("CMD:MODE,0")
    board.cmd("CMD:ENABLE,0")
    board.drain()

    return {
        'label': label,
        'rsff_scale': rsff_scale,
        'pwm_diag_idle': diag,
        'pwm_diag_post': diag2,
        'sref': sref_results,
        'pref': pref_results,
    }


def main():
    print("="*60)
    print("RsFF Scale Sweep Test")
    print(f"Started: {datetime.now().isoformat()}")
    print("="*60)

    board = FOCBoard()

    try:
        # Check initial state
        print("\n=== Initial State ===")
        board.cmd("CMD:CLEAR_FAULT")
        resp = board.cmd("CMD:FAULT?", wait=0.5)
        for line in resp.split('\n')[:20]:  # First 20 lines of detail
            line = line.strip()
            if line:
                print(f"  {line[:120]}")

        all_results = {}

        # Test 1: RS_FF=OFF baseline
        all_results['baseline'] = run_single_test(board, None, "RS_FF=OFF (Baseline)")

        # Tests 2-5: Scale sweep
        scales = [0.10, 0.20, 0.30, 0.40]
        for scale in scales:
            label = f"RS_FF_SCALE={scale:.2f}"
            all_results[label] = run_single_test(board, scale, label)

        # Print final summary
        print(f"\n{'='*60}")
        print("FINAL SUMMARY")
        print(f"{'='*60}")

        for key, test in all_results.items():
            print(f"\n--- {key} ---")
            print(f"  PWM_DIAG idle: {test['pwm_diag_idle'][:120]}")
            # Summarize SREF zero returns
            sref_zeros = [k for k in test['sref'] if 'SREF=0' in k and 'idle' not in k]
            for zk in sref_zeros:
                s = test['sref'][zk]['summary']
                print(f"  {zk}: {s}")
            print(f"  PWM_DIAG post: {test['pwm_diag_post'][:120]}")

            # Check for any faults across all steps
            faults_found = []
            for phase_results in [test['sref'], test['pref']]:
                for step_name, step_data in phase_results.items():
                    if step_data.get('fault') and step_data['fault'] != '0x00000000':
                        faults_found.append(f"{step_name}: {step_data['fault']}")
            if faults_found:
                print(f"  ** FAULTS: {', '.join(faults_found)}")
            else:
                print(f"  ** No faults")

        # Save results
        output_file = f"E:/24V_FOC_Controller_sync_20260519/scripts/rsff_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(output_file, 'w') as f:
            json.dump(all_results, f, indent=2, default=str)
        print(f"\nResults saved to: {output_file}")

    finally:
        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.close()


if __name__ == '__main__':
    main()
