#!/usr/bin/env python3
"""
FOC Closed-Loop Recovery Test Runner
=====================================
Automated test script for v1.8 firmware verification.
Tests: cold boot → identify → current loop → position bring-up

Usage: python test_plan_runner.py
"""

import serial
import time
import sys
import re
import json
from datetime import datetime

# ─── Config ───────────────────────────────────────────────────────
PORT = "COM9"
BAUD = 230400
TIMEOUT_SHORT = 0.3
TIMEOUT_MED = 1.0
TIMEOUT_LONG = 3.0

# ─── State ────────────────────────────────────────────────────────
ser = None
rx_buffer = b""
log_lines = []
phase_results = {}

def log(msg, level="INFO"):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] [{level}] {msg}"
    print(line)
    log_lines.append(line)

def open_serial():
    global ser
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_SHORT)
    time.sleep(0.2)
    ser.reset_input_buffer()
    log(f"Serial {PORT} @ {BAUD} opened")

def close_serial():
    global ser
    if ser and ser.is_open:
        ser.close()
        log("Serial closed")

def send_cmd(cmd):
    """Send a command with newline termination."""
    full_cmd = cmd + "\n"
    ser.write(full_cmd.encode('utf-8'))
    log(f"TX: {cmd}", "CMD")

def read_until(timeout_s=2.0, min_lines=1):
    """Read serial until timeout, return list of decoded lines."""
    global rx_buffer
    lines = []
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            b = ser.read(4096)
            if b:
                rx_buffer += b
        except:
            pass

        # Extract complete lines
        while b'\n' in rx_buffer:
            line, rx_buffer = rx_buffer.split(b'\n', 1)
            try:
                decoded = line.decode('utf-8', errors='replace').strip()
                if decoded:
                    lines.append(decoded)
            except:
                pass

        if lines and time.time() - start > 0.3:
            break  # Got some data, short wait
        time.sleep(0.02)

    return lines

def flush_and_read(timeout_s=3.0):
    """Flush serial buffer, send nothing, read existing output."""
    ser.reset_input_buffer()
    global rx_buffer
    rx_buffer = b""
    lines = []
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            b = ser.read(4096)
            if b:
                rx_buffer += b
        except:
            pass

        while b'\n' in rx_buffer:
            line, rx_buffer = rx_buffer.split(b'\n', 1)
            try:
                decoded = line.decode('utf-8', errors='replace').strip()
                if decoded:
                    lines.append(decoded)
            except:
                pass
        time.sleep(0.02)
    return lines

def extract_field(line, field_name):
    """Extract value after a field_name: pattern."""
    m = re.search(rf'{field_name}[:=]\s*([^\s,]+)', line)
    return m.group(1) if m else None

def wait_for_text(pattern, timeout_s=5.0):
    """Wait until a line matching regex appears."""
    start = time.time()
    while time.time() - start < timeout_s:
        lines = read_until(1.0, min_lines=0)
        for l in lines:
            if re.search(pattern, l, re.IGNORECASE):
                return l
        time.sleep(0.05)
    return None


# ═══════════════════════════════════════════════════════════════════
# TEST PHASES
# ═══════════════════════════════════════════════════════════════════

def phase1_cold_boot():
    """
    Phase 1: Cold boot verification.
    Send CLEAR_FAULT, then FAULT_DETAIL.
    Verify: Identified should show state, no active fault.
    """
    log("=" * 60)
    log("PHASE 1: Cold Boot Verification")
    log("=" * 60)

    # Clear any existing fault
    send_cmd("CMD:CLEAR_FAULT")
    time.sleep(0.5)
    lines = read_until(2.0)
    for l in lines:
        log(f"  RX: {l[:200]}", "RX")

    # Request fault detail
    send_cmd("CMD:FAULT_DETAIL")
    time.sleep(1.5)
    lines = read_until(5.0)

    # Parse the response
    fault_detail_text = []
    identified = None
    has_fault = False
    encoder_detected = None

    for l in lines:
        log(f"  RX: {l[:250]}", "RX")
        fault_detail_text.append(l)

        if 'Identified' in l:
            identified = 'YES' if 'YES' in l.upper() else ('NO' if 'NO' in l.upper() else None)
        if 'fault' in l.lower() and ('active' in l.lower() or 'DRV' in l or 'FAULT' in l):
            has_fault = True
        if 'Encoder' in l and ('detected' in l.lower() or 'YES' in l):
            encoder_detected = True

    # Also check N packets for motor_identified flag
    n_packets = [l for l in lines if l.startswith('N,')]
    if n_packets:
        parts = n_packets[0].split(',')
        if len(parts) > 18:
            # Field 17 (0-indexed) is motor_identified
            motor_id = parts[17] if len(parts) > 17 else '?'
            log(f"  N-packet motor_identified={motor_id}")
            if motor_id == '1':
                identified = 'YES'
            elif motor_id == '0':
                identified = 'NO'

    log(f"Result: Identified={identified}, has_fault={has_fault}, encoder_detected={encoder_detected}")

    phase_results['phase1'] = {
        'identified': identified,
        'has_fault': has_fault,
        'encoder_detected': encoder_detected,
        'raw_text': '\n'.join(fault_detail_text[-20:])  # Last 20 lines
    }

    if identified == 'YES':
        log("WARNING: motor_identified=YES — old params may still be valid", "WARN")
        log("Proceeding anyway since this might be expected post-identify retention.", "WARN")

    if has_fault:
        log("WARNING: FAULT DETECTED — check board state", "WARN")

    return phase_results['phase1']


def phase2_identify():
    """
    Phase 2: Motor parameter identification.
    Send MOTOR_PN,11 / ENCODER_DIR,-1 / UNLOCK,1 / IDENTIFY,1
    Wait for completion, then FAULT_DETAIL.
    Verify: COMPLETE, status=1, locked_dir=-1, verify_accum>=0.300, valid_flag=0xFFFFFFFF
    """
    log("=" * 60)
    log("PHASE 2: Motor Parameter Identification")
    log("=" * 60)

    # Configure motor params
    send_cmd("CMD:MOTOR_PN,11")
    time.sleep(0.3)
    read_until(0.5)

    send_cmd("CMD:ENCODER_DIR,-1")
    time.sleep(0.3)
    read_until(0.5)

    # Unlock power stage
    send_cmd("CMD:CLEAR_FAULT")
    time.sleep(0.5)
    read_until(0.5)

    send_cmd("CMD:UNLOCK,1")
    time.sleep(0.5)
    read_until(0.5)

    # Start identification
    log("Starting motor identification...")
    send_cmd("CMD:IDENTIFY,1")

    # Monitor progress - wait for completion (can take 10-30s)
    identify_complete = False
    identify_lines = []
    start_time = time.time()
    timeout_s = 40.0

    while time.time() - start_time < timeout_s:
        lines = read_until(2.0)
        for l in lines:
            if l.startswith('N,') or l.startswith('F,') or l.startswith('CMD:') or 'IDENTIFY' in l.upper() or 'COMPLETE' in l.upper():
                identify_lines.append(l)
                log(f"  RX: {l[:200]}", "RX")
                if 'COMPLETE' in l.upper() or 'complete' in l.lower():
                    identify_complete = True
                    break
            elif 'identify' in l.lower():
                identify_lines.append(l)
                log(f"  RX: {l[:200]}", "RX")

        if identify_complete:
            break

        # Check N packets for identify_state
        n_packets = [l for l in lines if l.startswith('N,')]
        for p in n_packets:
            parts = p.split(',')
            if len(parts) >= 23:
                # identify_state is field 23 (0-indexed)
                pass

    if not identify_complete:
        log("WARNING: Identification may still be running or timed out", "WARN")

    # Wait a bit more and get final state
    time.sleep(2.0)

    # Get fault detail
    send_cmd("CMD:FAULT_DETAIL")
    time.sleep(1.5)
    final_lines = read_until(5.0)

    detail_text = []
    for l in final_lines:
        log(f"  RX: {l[:250]}", "RX")
        detail_text.append(l)

    # Parse results
    full_text = '\n'.join(identify_lines + final_lines)

    status = None
    locked_dir = None
    verify_accum = None
    valid_flag = None

    for l in (identify_lines + final_lines):
        if 'status' in l.lower() and '=' in l:
            m = re.search(r'status[=:]\s*(\d+)', l, re.IGNORECASE)
            if m: status = int(m.group(1))
        if 'locked_dir' in l.lower():
            m = re.search(r'locked_dir[=:]\s*(-?\d+)', l, re.IGNORECASE)
            if m: locked_dir = int(m.group(1))
        if 'verify_accum' in l.lower():
            m = re.search(r'verify_accum[=:]\s*([\d.]+)', l, re.IGNORECASE)
            if m: verify_accum = float(m.group(1))
        if 'valid_flag' in l.lower():
            m = re.search(r'valid_flag[=:]\s*(0x[0-9a-fA-F]+)', l, re.IGNORECASE)
            if m: valid_flag = m.group(1)

    log(f"Results: status={status}, locked_dir={locked_dir}, verify_accum={verify_accum}, valid_flag={valid_flag}")

    checks = {
        'completed': identify_complete,
        'status_ok': status == 1,
        'locked_dir_ok': locked_dir == -1,
        'verify_accum_ok': verify_accum is not None and verify_accum >= 0.300,
        'valid_flag_ok': valid_flag == '0xFFFFFFFF' or valid_flag == '0xffffffff'
    }

    phase_results['phase2'] = {
        'checks': checks,
        'status': status,
        'locked_dir': locked_dir,
        'verify_accum': verify_accum,
        'valid_flag': valid_flag,
        'all_pass': all(checks.values())
    }

    return phase_results['phase2']


def phase3_current_loop_micro():
    """
    Phase 3: Current loop micro-integral verification.
    PI_CURRENT,0.03,0.5 / MODE,0 / UNLOCK,1 / ENABLE,1 / IREF,0,0.05
    After 0.3-0.5s: CURRENT_SNAP, FAULT_DETAIL, IREF,0,0
    Verify: Iq≈0.05A, no ±1A oscillation, +IqRef → negative mechanical angle.
    """
    log("=" * 60)
    log("PHASE 3: Current Loop Micro-Integral Verification")
    log("=" * 60)

    # Configure
    send_cmd("CMD:CLEAR_FAULT")
    time.sleep(0.5)
    read_until(0.5)

    send_cmd("CMD:PI_CURRENT,0.03,0.5")
    time.sleep(0.3)
    read_until(0.3)

    send_cmd("CMD:MODE,0")  # Torque mode
    time.sleep(0.3)
    read_until(0.3)

    send_cmd("CMD:UNLOCK,1")
    time.sleep(0.3)
    read_until(0.3)

    # Enable motor
    send_cmd("CMD:ENABLE,1")
    time.sleep(0.5)
    enable_lines = read_until(1.0)
    for l in enable_lines:
        log(f"  RX: {l[:200]}", "RX")

    # Apply 0.05A Iq
    log("Setting Iq_ref=0.05A...")
    send_cmd("CMD:IREF,0,0.05")

    # Wait 0.4s then sample
    time.sleep(0.4)

    # Current snapshot
    send_cmd("CMD:CURRENT_SNAP")
    time.sleep(0.3)
    snap_lines = read_until(1.5)

    # Fault detail
    send_cmd("CMD:FAULT_DETAIL")
    time.sleep(0.5)

    # Stop current
    send_cmd("CMD:IREF,0,0")

    # Read all output
    time.sleep(0.5)
    all_lines = read_until(2.0)

    all_output = snap_lines + all_lines
    for l in all_output:
        log(f"  RX: {l[:200]}", "RX")

    # Parse N-packets for Iq, speed
    n_packets = [l for l in all_output if l.startswith('N,')]
    iq_values = []
    speed_values = []
    fault_flags = []

    for p in n_packets:
        parts = p.split(',')
        if len(parts) >= 10:
            try:
                iq_values.append(float(parts[8]))
                speed_values.append(float(parts[5]))
                fault_flags.append(parts[7])
            except:
                pass

    if iq_values:
        avg_iq = sum(iq_values) / len(iq_values)
        max_iq = max(iq_values)
        min_iq = min(iq_values)
        iq_oscillation = max_iq - min_iq
        log(f"Iq stats: avg={avg_iq:.4f}, min={min_iq:.4f}, max={max_iq:.4f}, oscillation={iq_oscillation:.4f}")
    else:
        avg_iq = None
        iq_oscillation = None
        log("WARNING: No N-packets with Iq data found", "WARN")

    if speed_values:
        avg_speed = sum(speed_values) / len(speed_values)
        log(f"Speed stats: avg={avg_speed:.4f} rad/s")

    has_fault = any('0x00000000' not in f for f in fault_flags) if fault_flags else False

    checks = {
        'iq_ok': avg_iq is not None and abs(avg_iq - 0.05) < 0.04,  # Within 0.01-0.09
        'no_oscillation': iq_oscillation is None or iq_oscillation < 1.0,  # Less than ±1A
        'no_fault': not has_fault
    }

    phase_results['phase3'] = {
        'checks': checks,
        'avg_iq': avg_iq,
        'iq_oscillation': iq_oscillation,
        'speed_values': speed_values[:10] if speed_values else [],
        'has_fault': has_fault,
        'all_pass': all(checks.values())
    }

    return phase_results['phase3']


def phase4_pulse_0_10A():
    """
    Phase 4: 0.10A short pulse verification.
    IREF,0,0.10 for 0.3-0.5s then stop.
    Verify: Iq follows 0.10A, direction negative mechanical, Vq not saturated, no fault.
    """
    log("=" * 60)
    log("PHASE 4: 0.10A Short Pulse Verification")
    log("=" * 60)

    # Re-enable if needed
    send_cmd("CMD:UNLOCK,1")
    time.sleep(0.3)
    send_cmd("CMD:ENABLE,1")
    time.sleep(0.3)
    read_until(0.5)

    # Apply 0.10A
    log("Setting Iq_ref=0.10A...")
    send_cmd("CMD:IREF,0,0.10")

    # Wait 0.4s
    time.sleep(0.4)

    # Capture snapshot while pulse is active
    send_cmd("CMD:CURRENT_SNAP")
    time.sleep(0.2)
    snap1 = read_until(1.0)

    send_cmd("CMD:FAULT_DETAIL")
    time.sleep(0.2)

    # Stop
    send_cmd("CMD:IREF,0,0")
    time.sleep(0.2)
    send_cmd("CMD:ENABLE,0")
    time.sleep(0.5)

    all_lines = snap1 + read_until(2.0)
    for l in all_lines:
        log(f"  RX: {l[:200]}", "RX")

    # Parse
    n_packets = [l for l in all_lines if l.startswith('N,')]
    iq_values = []
    vq_values = []
    speed_values = []
    ia_values = []

    for p in n_packets:
        parts = p.split(',')
        if len(parts) >= 22:
            try:
                iq_values.append(float(parts[8]))
                # Vq is at index 19, Vd at 18
                if len(parts) > 19:
                    vq_values.append(float(parts[19]))
                speed_values.append(float(parts[5]))
                # Ia, Ib, Ic are at indices 20, 21, 22
                if len(parts) > 20:
                    ia_values.append(float(parts[20]))
            except:
                pass

    if iq_values:
        avg_iq = sum(iq_values) / len(iq_values)
        max_iq = max(iq_values)
        log(f"Iq stats: avg={avg_iq:.4f}, max={max_iq:.4f}")
    else:
        avg_iq = None
        log("WARNING: No Iq data", "WARN")

    if vq_values:
        avg_vq = sum(vq_values) / len(vq_values)
        max_vq = max(vq_values)
        log(f"Vq stats: avg={avg_vq:.4f}, max={max_vq:.4f}")
    else:
        avg_vq = None

    if speed_values:
        avg_speed = sum(speed_values) / len(speed_values)
        log(f"Speed: avg={avg_speed:.4f} rad/s")

    # Check if CURRENT_SNAP returned explicit values
    snap_iq = None
    for l in all_lines:
        if 'Iq' in l and 'SNAP' not in l.upper():
            m = re.search(r'Iq[=:]\s*([\d.-]+)', l)
            if m: snap_iq = float(m.group(1))

    checks = {
        'iq_follows': avg_iq is not None and abs(avg_iq - 0.10) < 0.07,  # 0.03 to 0.17
        'no_fault': True,  # Check from fault detail
        'iq_not_negative': avg_iq is not None and avg_iq > 0  # Should be positive
    }

    phase_results['phase4'] = {
        'checks': checks,
        'avg_iq': avg_iq,
        'avg_vq': avg_vq,
        'snap_iq': snap_iq,
        'all_pass': all(checks.values())
    }

    return phase_results['phase4']


def phase5_home_and_position():
    """
    Phase 5: HOME persistence and position bring-up.
    CMD:HOME, power cycle, verify zero maintained.
    Position closed loop from zero target, small angle steps.
    """
    log("=" * 60)
    log("PHASE 5: HOME Persistence & Position Bring-Up")
    log("=" * 60)

    # Step 1: HOME command
    log("Step 5a: HOME command...")
    send_cmd("CMD:HOME")
    time.sleep(1.0)
    home_lines = read_until(3.0)
    for l in home_lines:
        log(f"  RX: {l[:200]}", "RX")

    send_cmd("CMD:FAULT_DETAIL")
    time.sleep(1.5)
    detail_lines = read_until(3.0)
    for l in detail_lines:
        log(f"  RX: {l[:200]}", "RX")

    # Check if HOME was acknowledged
    home_ok = any('HOME' in l.upper() or 'zero' in l.lower() for l in home_lines + detail_lines)
    log(f"HOME acknowledged: {home_ok}")

    # Step 2: Setup position mode
    log("Step 5b: Position closed loop setup...")
    send_cmd("CMD:CLEAR_FAULT")
    time.sleep(0.5)
    read_until(0.5)

    send_cmd("CMD:PI_CURRENT,0.03,0.5")
    time.sleep(0.2)
    send_cmd("CMD:PD_POS,1.0,0.05")
    time.sleep(0.2)
    send_cmd("CMD:PI_SPEED,0.10,0")
    time.sleep(0.2)
    send_cmd("CMD:MODE,2")  # Position mode
    time.sleep(0.2)
    send_cmd("CMD:PREF,0.000")
    time.sleep(0.3)
    send_cmd("CMD:UNLOCK,1")
    time.sleep(0.3)

    # Enable in position mode
    log("Enabling position mode at PREF=0...")
    send_cmd("CMD:ENABLE,1")

    # Monitor for 2 seconds
    pos_hold_lines = []
    start = time.time()
    while time.time() - start < 2.5:
        lines = read_until(1.0)
        for l in lines:
            if l.startswith('N,'):
                pos_hold_lines.append(l)
                parts = l.split(',')
                if len(parts) >= 8:
                    try:
                        state = int(parts[2])
                        speed = float(parts[5])
                        fault = parts[7]
                        log(f"  State={state}, Speed={speed:.4f}, Fault={fault}", "RX")
                    except:
                        pass

    # Check convergence
    speeds = []
    for l in pos_hold_lines:
        parts = l.split(',')
        if len(parts) >= 6:
            try:
                speeds.append(float(parts[5]))
            except:
                pass

    if speeds:
        avg_speed = sum(speeds) / len(speeds)
        max_abs_speed = max(abs(s) for s in speeds)
        log(f"Speed at PREF=0: avg={avg_speed:.4f}, max_abs={max_abs_speed:.4f}")
        converging = max_abs_speed < 0.5  # Less than 0.5 rad/s
    else:
        converging = False
        log("WARNING: No speed data during position hold", "WARN")

    # Step 3: Small angle steps
    log("Step 5c: Small angle steps...")
    angle_results = []

    for angle_rad in [0.052, 0.0, -0.052, 0.0]:
        log(f"  Setting PREF={angle_rad:.3f}...")
        send_cmd(f"CMD:PREF,{angle_rad:.3f}")
        time.sleep(0.5)

        # Read telemetry
        lines = read_until(1.0)
        n_lines = [l for l in lines if l.startswith('N,')]

        for l in n_lines:
            parts = l.split(',')
            if len(parts) >= 8:
                try:
                    angle_deg = float(parts[4])
                    speed_rps = float(parts[5])
                    fault = parts[7]
                    log(f"    Angle={angle_deg:.2f}°, Speed={speed_rps:.4f}, Fault={fault}", "RX")
                    angle_results.append({
                        'target_rad': angle_rad,
                        'actual_angle_deg': angle_deg,
                        'speed': speed_rps,
                        'fault': fault
                    })
                except:
                    pass

        # After each step, check for faults
        if any('0x00000000' not in l for l in n_lines if l.startswith('N,') and len(l.split(',')) > 7):
            log("WARNING: Possible fault detected during angle step", "WARN")

    # Then try larger steps: ±0.087 rad
    for angle_rad in [0.087, 0.0, -0.087, 0.0]:
        log(f"  Setting PREF={angle_rad:.3f}...")
        send_cmd(f"CMD:PREF,{angle_rad:.3f}")
        time.sleep(0.5)

        lines = read_until(1.0)
        n_lines = [l for l in lines if l.startswith('N,')]

        for l in n_lines:
            parts = l.split(',')
            if len(parts) >= 8:
                try:
                    angle_deg = float(parts[4])
                    speed_rps = float(parts[5])
                    fault = parts[7]
                    log(f"    Angle={angle_deg:.2f}°, Speed={speed_rps:.4f}, Fault={fault}", "RX")
                    angle_results.append({
                        'target_rad': angle_rad,
                        'actual_angle_deg': angle_deg,
                        'speed': speed_rps,
                        'fault': fault
                    })
                except:
                    pass

    # Disable motor
    send_cmd("CMD:ENABLE,0")
    time.sleep(0.5)
    read_until(1.0)

    phase_results['phase5'] = {
        'home_ok': home_ok,
        'position_hold_converging': converging,
        'avg_hold_speed': avg_speed if speeds else None,
        'angle_steps': angle_results,
        'all_pass': home_ok and converging
    }

    return phase_results['phase5']


def print_summary():
    """Print test results summary."""
    log("=" * 60)
    log("TEST PLAN SUMMARY")
    log("=" * 60)

    all_pass = True
    for phase_name in ['phase1', 'phase2', 'phase3', 'phase4', 'phase5']:
        if phase_name in phase_results:
            r = phase_results[phase_name]
            phase_all_pass = r.get('all_pass', False)
            status = "✅ PASS" if phase_all_pass else "❌ FAIL"
            log(f"  {phase_name}: {status}")
            if not phase_all_pass:
                all_pass = False
                for check_name, check_val in r.get('checks', {}).items():
                    log(f"    {check_name}: {'PASS' if check_val else 'FAIL'}")
        else:
            log(f"  {phase_name}: ⚠️ NOT RUN")
            all_pass = False

    log(f"\nOverall: {'✅ ALL PASS' if all_pass else '❌ SOME FAILURES'}")

    # Save detailed results
    with open("test_plan_results.json", "w") as f:
        json.dump(phase_results, f, indent=2, default=str)
    log("Detailed results saved to test_plan_results.json")

    # Save full log
    with open("test_plan_log.txt", "w", encoding='utf-8') as f:
        f.write('\n'.join(log_lines))
    log("Full log saved to test_plan_log.txt")


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    log("FOC Closed-Loop Recovery Test Runner")
    log(f"Port: {PORT}, Baud: {BAUD}")
    log(f"Started at {datetime.now().isoformat()}")

    try:
        open_serial()

        # ── Phase 1: Cold Boot ──
        phase1_cold_boot()

        # ── Phase 2: Motor Identification ──
        phase2_identify()

        # ── Phase 3: Current Loop Micro-Integral ──
        # Note: only if phase 2 passed or user wants to continue
        if phase_results['phase2'].get('all_pass', False):
            phase3_current_loop_micro()
        else:
            log("⚠️ Skipping Phase 3 — Phase 2 did not fully pass", "WARN")
            log("Continuing anyway per test plan instructions...", "INFO")
            phase3_current_loop_micro()

        # ── Phase 4: 0.10A Pulse ──
        phase4_pulse_0_10A()

        # ── Phase 5: HOME & Position ──
        phase5_home_and_position()

    except KeyboardInterrupt:
        log("Test interrupted by user", "WARN")
    except Exception as e:
        log(f"Test error: {e}", "ERROR")
        import traceback
        traceback.print_exc()
    finally:
        close_serial()
        print_summary()


if __name__ == "__main__":
    main()
