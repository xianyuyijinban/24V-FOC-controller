"""
BEMF Regression Test — Phase 1/2/3
Baseline: PI_CURRENT=0.50/0, PI_SPEED=0.25/0, RS_FF=DQ/0.20, COG=0.25/60°
Tests BEMF ON with KE_TEMP sweep (0.003, 0.006, 0 default)
"""
import serial
import time
import sys
import json
from datetime import datetime

PORT = 'COM9'
BAUD = 1152000

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
        resp = self.cmd("CMD:PWM_DIAG", wait=0.5)
        for line in resp.split('\n'):
            if 'PWM,' in line:
                return line.strip()
        return ""

    def read_query(self, cmd, wait=0.3):
        """Read a query command response (just first meaningful line)"""
        resp = self.cmd(cmd, wait=wait)
        for line in resp.split('\n'):
            line = line.strip()
            if line and not line.startswith('>'):
                return line[:200]
        return resp[:200].strip() if resp else ""

    def close(self):
        self.s.close()


def fmt_frames(frames):
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
    return (f"n={n} | speed: {avg(speeds):.3f} pk={peak(speeds):.3f} | "
            f"Iq: {avg(iqs):.4f} pk={peak(iqs):.4f} | Id: {avg(ids):.4f} pk={peak(ids):.4f} | "
            f"Vq: {avg(vqs):.4f} pk={peak(vqs):.4f} | Iq_ref: {avg(iq_refs):.4f}")

def check_fault(frames):
    for f in frames:
        if f['fault'] != '0x00000000':
            return f['fault']
    return None

def pwm_diag_parse(diag_line):
    """Parse PWM_DIAG into dict. Key fields: Vqpi, BEMF blocked, sat_ratio"""
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


def setup_baseline(board):
    """Set baseline parameters: PI, RS_FF, COG, BEMF=OFF"""
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
        ("CMD:KE_TEMP,0", "KE_TEMP: 0 (default)"),
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


def run_sref_sweep(board, label, capture_dur=2.0):
    """SREF sweep: 0→+0.5→0→-0.5→0→+1.0→0→-1.0→0"""
    print(f"\n--- SREF Sweep: {label} ---")
    results = {}
    steps = [
        ("SREF=0 idle", "CMD:SREF,0", capture_dur),
        ("SREF=+0.5", "CMD:SREF,0.5", capture_dur),
        ("SREF=0", "CMD:SREF,0", capture_dur),
        ("SREF=-0.5", "CMD:SREF,-0.5", capture_dur),
        ("SREF=0", "CMD:SREF,0", capture_dur),
        ("SREF=+1.0", "CMD:SREF,1.0", capture_dur),
        ("SREF=0", "CMD:SREF,0", capture_dur),
        ("SREF=-1.0", "CMD:SREF,-1.0", capture_dur),
        ("SREF=0 settle", "CMD:SREF,0", 3.0),
    ]
    for name, cmd, dur in steps:
        board.cmd(cmd, wait=0.3)
        time.sleep(0.5)
        frames = board.capture(duration=dur)
        fault = check_fault(frames)
        summary = fmt_frames(frames)
        print(f"  {name:20s} | {summary}")
        if fault:
            print(f"  ** FAULT: {fault} **")

        # Get PWM_DIAG for key steps
        diag = ""
        if "SREF=" in name and "idle" not in name:
            diag_raw = board.pwm_diag()
            diag = pwm_diag_parse(diag_raw)
            print(f"    PWM_DIAG: Vqpi={diag.get('Vqpi','?')}mV, Vqb={diag.get('Vqb','?')}mV, bemb={diag.get('bemb','?')}, ome={diag.get('ome','?')}, Ke={diag.get('Ke','?')}")

        results[name] = {
            'cmd': cmd, 'n_frames': len(frames),
            'fault': fault, 'summary': summary, 'pwm_diag': diag,
        }
    return results


def run_smoke_cycles(board, cycles=10):
    """10 cycles of 0→+1.0→0→-1.0→0"""
    print(f"\n--- Smoke Test ({cycles} cycles) ---")
    faults_seen = []
    zero_vq_peaks = []
    for i in range(cycles):
        board.cmd("CMD:SREF,1.0", wait=0.3)
        time.sleep(0.3)
        board.cmd("CMD:SREF,0", wait=0.3)
        time.sleep(0.3)
        board.cmd("CMD:SREF,-1.0", wait=0.3)
        time.sleep(0.3)
        board.cmd("CMD:SREF,0", wait=0.3)
        time.sleep(0.3)

        # Capture zero-return settle
        frames = board.capture(duration=1.0)
        fault = check_fault(frames)
        if fault:
            faults_seen.append(f"{i}:{fault}")

        vqs = [abs(f['Vq']) for f in frames if f['speed'] < 0.05]
        vq_pk = max(vqs) if vqs else 0
        zero_vq_peaks.append(vq_pk)

        # Quick fault check: capture frames and check AppFault field
        if i % 2 == 1:
            check_frames = board.capture(duration=0.3)
            flt = check_fault(check_frames)
            if flt:
                faults_seen.append(f"cycle{i}:{flt}")

        if (i + 1) % 5 == 0:
            print(f"  Cycle {i+1}/{cycles}: Vq_pk={vq_pk*1000:.0f}mV, faults={len(faults_seen)}")

    # Final settle check
    frames = board.capture(duration=1.0)
    final_vq_pk = max([abs(f['Vq']) for f in frames]) * 1000 if frames else 0
    print(f"  Final Vq_pk={final_vq_pk:.0f}mV")

    return {'zero_vq_peaks_mV': [p*1000 for p in zero_vq_peaks],
            'final_vq_pk_mV': final_vq_pk, 'faults': faults_seen}


def main():
    print("="*60)
    print("BEMF Regression Test")
    print(f"Started: {datetime.now().isoformat()}")
    print("="*60)

    board = FOCBoard()

    try:
        # ─── Baseline Setup ───
        print("\n### BASELINE SETUP ###")
        setup_baseline(board)

        # Check initial fault state
        resp = board.cmd("CMD:FAULT?", wait=0.3)
        print(f"\n  Initial fault: {resp.split(chr(10))[0].strip() if resp else 'N/A'}")

        # Enable motor in SPEED mode (SREF sweeps need speed loop active)
        board.cmd("CMD:ENABLE,1", wait=0.3)
        board.cmd("CMD:MODE,1", wait=0.3)  # SPEED mode for SREF sweeps
        board.drain()

        # ─── Phase 1: BEMF ON Static Check ───
        print(f"\n{'='*60}")
        print("PHASE 1: BEMF ON Static Check")
        print(f"{'='*60}")

        board.cmd("CMD:BEMF_CFG,1", wait=0.3)
        resp = board.read_query("CMD:BEMF_CFG?")
        print(f"  BEMF_CFG? -> {resp}")

        board.cmd("CMD:SREF,0", wait=0.3)
        time.sleep(0.5)
        frames = board.capture(duration=1.5)
        fault = check_fault(frames)
        diag_raw = board.pwm_diag()
        diag = pwm_diag_parse(diag_raw)

        vqs = [abs(f['Vq']) for f in frames]
        vq_pk_static = max(vqs) * 1000 if vqs else 0
        print(f"  Static Vq_pk={vq_pk_static:.0f}mV, Fault={fault}")
        print(f"  PWM_DIAG: Vq={diag.get('Vq','?')}mV, Vqpi={diag.get('Vqpi','?')}mV, Vqb={diag.get('Vqb','?')}mV, ome={diag.get('ome','?')}, bemb={diag.get('bemb','?')}, Ke={diag.get('Ke','?')}")

        if vq_pk_static > 30:
            print("  ** WARNING: Static Vq > 30mV — check BEMF interaction **")

        # Convert FAULT to full format for check
        resp = board.read_query("CMD:FAULT_DETAIL")
        print(f"  FAULT_DETAIL: {resp[:200]}")

        phase1_pass = (fault is None or fault == '0x00000000') and vq_pk_static < 30
        print(f"  Phase 1 {'PASS' if phase1_pass else 'WARNING'}")

        if vq_pk_static > 80:
            print("  ** Static Vq too high, aborting! **")
            board.cmd("CMD:BEMF_CFG,0")
            board.cmd("CMD:ENABLE,0")
            board.close()
            return

        # ─── Phase 2: Ke Sweep ───
        print(f"\n{'='*60}")
        print("PHASE 2: Ke Sweep (SREF at low speed)")
        print(f"{'='*60}")

        ke_values = [0.003, 0.006, 0.0]  # 0.0 = default from identification
        ke_results = {}

        for ke in ke_values:
            label = f"Ke_default" if ke == 0.0 else f"Ke={ke:.3f}"
            print(f"\n### Testing {label} ###")

            board.cmd(f"CMD:KE_TEMP,{ke:.3f}", wait=0.3)
            resp = board.read_query("CMD:KE_TEMP?")
            print(f"  KE_TEMP? -> {resp}")

            # SREF sweep
            sref = run_sref_sweep(board, f"BEMF=ON, {label}")

            # Quick fault check after sweep
            time.sleep(0.3)
            flt = board.read_query("CMD:FAULT?")
            fault_after = flt[:60] if flt else "N/A"
            print(f"  Post-sweep fault: {fault_after}")

            ke_results[label] = {
                'ke_value': ke,
                'sref': sref,
                'fault_after': fault_after,
            }

        # ─── Select best Ke from Phase 2 settle data ───
        best_ke = None
        best_settle_vq_mv = 999.0
        print(f"\n### Phase 2 Settle Vq Summary ###")
        for label, kr in ke_results.items():
            sref = kr['sref']
            settle = sref.get('SREF=0 settle', {})
            summary = settle.get('summary', '')
            vq_pk_mv = 999.0
            if 'Vq:' in summary:
                try:
                    # summary format: "Vq: 0.0239 pk=0.0520 | ..."
                    vq_pk_str = summary.split('Vq:')[1].split('pk=')[1].split('|')[0].strip()
                    vq_pk_mv = float(vq_pk_str) * 1000.0
                except (ValueError, IndexError):
                    pass
            print(f"  {label:20s} settle Vq_pk={vq_pk_mv:.0f}mV")
            if vq_pk_mv < best_settle_vq_mv:
                best_settle_vq_mv = vq_pk_mv
                best_ke = kr['ke_value']

        if best_ke is None:
            best_ke = 0.003  # fallback

        print(f"\n  => Best Ke={best_ke:.3f} (settle Vq_pk={best_settle_vq_mv:.0f}mV)")

        # ─── Phase 3: 10-cycle Smoke on best Ke ───
        print(f"\n{'='*60}")
        print(f"PHASE 3: 10-Cycle Smoke Test (auto-selected Ke={best_ke:.3f})")
        print(f"{'='*60}")

        board.cmd(f"CMD:KE_TEMP,{best_ke:.3f}", wait=0.3)
        smoke = run_smoke_cycles(board, cycles=10)

        print(f"\n  Smoke results: Vq_pks={[f'{v:.0f}' for v in smoke['zero_vq_peaks_mV'][:5]]}... mV")
        print(f"  Faults: {smoke['faults'] if smoke['faults'] else 'None'}")

        # ─── Phase 3B: 5-cycle on runner-up Ke (if different) ───
        runner_up = [(kr['ke_value'], kr['sref'].get('SREF=0 settle', {}).get('summary',''))
                     for kr in ke_results.values()
                     if kr['ke_value'] != best_ke]
        if runner_up:
            runner_up.sort(key=lambda x: 999 if 'Vq:' not in x[1] else float(x[1].split('Vq:')[1].split('pk=')[1].split('|')[0]))
            runner_ke = runner_up[0][0]
            print(f"\n  Phase 3B: 5-cycle on runner-up Ke={runner_ke:.3f}")
            board.cmd(f"CMD:KE_TEMP,{runner_ke:.3f}", wait=0.3)
            smoke_b = run_smoke_cycles(board, cycles=5)
            print(f"  Smoke B results: Vq_pks={[f'{v:.0f}' for v in smoke_b['zero_vq_peaks_mV'][:3]]}... mV")
            smoke = {'phase3a': smoke, 'phase3b': smoke_b, 'best_ke': best_ke, 'runner_ke': runner_ke}
        else:
            smoke = {'phase3a': smoke, 'best_ke': best_ke}

        # ─── Final Status ───
        print(f"\n{'='*60}")
        print("FINAL STATUS")
        print(f"{'='*60}")

        board.cmd("CMD:SREF,0", wait=0.3)
        board.cmd("CMD:ENABLE,0", wait=0.3)

        diag_final = board.pwm_diag()
        print(f"  PWM_DIAG final: {diag_final[:150]}")

        # Save results
        output_file = f"E:/24V_FOC_Controller_sync_20260519/scripts/bemf_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        all_data = {
            'phase1': {'vq_pk_static_mV': vq_pk_static, 'fault': fault, 'pwm_diag': diag, 'pass': phase1_pass},
            'phase2': ke_results,
            'phase3': smoke,
            'diag_final': diag_final,
        }
        with open(output_file, 'w') as f:
            json.dump(all_data, f, indent=2, default=str)
        print(f"\nResults saved to: {output_file}")

    finally:
        board.cmd("CMD:MODE,0")
        board.cmd("CMD:SREF,0")
        board.cmd("CMD:ENABLE,0")
        board.close()


if __name__ == '__main__':
    main()
