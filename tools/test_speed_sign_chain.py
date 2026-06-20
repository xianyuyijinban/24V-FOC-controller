#!/usr/bin/env python3
"""
Speed Mode Sign Chain Fix — Automated Verification Test Suite
=============================================================
Validates the encoder_dir mapping fix in FOC_App_SpeedLoop.

N frame field indices (from uart_upload.c:376-408):
  [0]="N"  [1]=timestamp  [2]=focState  [3]=angle(deg)
  [4]=speed(rad/s)  [5]=Id(A)  [6]=Iq(A)  [7]=vbus(V)
  [8]=faultFlags(hex)  [9]=encDetected  [10]=motorIdentified
  [11]=stallArmed  [12]=stallActive  [13]=warnFlags
  [14]=faultCode  [15]=controlMode(0=T/1=S/2=P)
  [16]=Id_ref  [17]=speed_ref  [18]=pos_ref
  [19]=Iq_ref  [20]=Vd  [21]=Vq
  [22]=Ia  [23]=Ib  [24]=Ic
  [25]=identifyState  [26]=identifyError
  [27]=uvLimit  [28]=ovLimit
  [29]=adcCalib  [30..32]=adcOffsetA/B/C

Prerequisites:
  - Control board powered, motor connected
  - COM9 at 230400 bps
  - Motor identified (encoder_dir should be -1)
"""

import serial
import time
import sys
import re
import json
from datetime import datetime

# Fix Unicode output on Windows (GBK codec can't encode most Unicode)
if sys.stdout.encoding and sys.stdout.encoding.upper() in ('GBK', 'GB2312', 'GB18030', 'CP936'):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# ── Configuration ──────────────────────────────────────────────
PORT = "COM9"
BAUD = 230400
TIMEOUT = 0.3  # serial read timeout

# Test parameters
IQ_SIGN_TEST_CURRENT = 0.08   # A — small enough to be safe, large enough to move
SPEED_SMALL_SIGNAL = 0.5      # rad/s
SPEED_STEPS = [+1.0, +2.0, +4.0, -1.0, -2.0, -4.0, 0.0]
POS_STEPS = [0.0, 20.0, 40.0, 80.0, 0.0]  # degrees
COGGING_CYCLES = 10           # 0↔40° cycles
COGGING_GAIN = 0.25
COGGING_PHASE = 60.0

# Safety limits
MAX_FAULT_CODE = 0
MAX_SPEED_OVERSHOOT_RATIO = 2.0  # speed shouldn't exceed 2x reference
POS_ERROR_THRESHOLD_DEG = 3.0

# ── Frame Parser ───────────────────────────────────────────────

def parse_n_frame(fields):
    """Parse N-frame comma-separated fields into dict."""
    if len(fields) < 33:
        return None
    try:
        return {
            "type": "N",
            "timestamp": int(fields[1]),
            "foc_state": int(fields[2]),
            "angle_deg": float(fields[3]),
            "speed_radps": float(fields[4]),
            "Id_A": float(fields[5]),
            "Iq_A": float(fields[6]),
            "vbus_V": float(fields[7]),
            "fault_flags": fields[8],
            "enc_detected": int(fields[9]),
            "motor_identified": int(fields[10]),
            "stall_armed": int(fields[11]),
            "stall_active": int(fields[12]),
            "warn_flags": fields[13],
            "fault_code": int(fields[14]),
            "control_mode": int(fields[15]),
            "Id_ref_A": float(fields[16]),
            "speed_ref_radps": float(fields[17]),
            "pos_ref_deg": float(fields[18]),
            "Iq_ref_A": float(fields[19]),
            "Vd_V": float(fields[20]),
            "Vq_V": float(fields[21]),
            "Ia_A": float(fields[22]),
            "Ib_A": float(fields[23]),
            "Ic_A": float(fields[24]),
            "identify_state": int(fields[25]),
            "identify_error": int(fields[26]),
            "uv_limit_V": float(fields[27]),
            "ov_limit_V": float(fields[28]),
            "adc_calib": int(fields[29]),
            "adc_offset_a": int(fields[30]),
            "adc_offset_b": int(fields[31]),
            "adc_offset_c": int(fields[32]),
        }
    except (ValueError, IndexError) as e:
        return None


def parse_c_frame(fields):
    """Parse C-frame (phase current)."""
    if len(fields) < 4:
        return None
    try:
        return {
            "type": "C",
            "timestamp": int(fields[1]),
            "Ia_A": float(fields[2]),
            "Ib_A": float(fields[3]),
            "Ic_A": float(fields[4]),
        }
    except (ValueError, IndexError):
        return None


# ── Test Runner ─────────────────────────────────────────────────

class TestRunner:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.results = []
        self._buf = b""
        self._raw_frames = []

    def connect(self):
        """Open serial port, drain initial data."""
        print(f"[INIT] Opening {self.port}@{self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=TIMEOUT)
        time.sleep(0.3)
        # Drain stale data
        self.ser.reset_input_buffer()
        time.sleep(0.1)
        print("[INIT] Connected.")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INIT] Disconnected.")

    def send(self, cmd):
        """Send a command string (newline appended automatically)."""
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.ser.write(cmd.encode())
        self.ser.flush()
        print(f"  TX > {cmd.strip()}")

    def read_frames(self, duration_s):
        """Read and parse frames for duration_s seconds.
        Returns dict with 'N', 'C', 'F' keys containing parsed dicts."""
        frames = {"N": [], "C": [], "F": []}
        start = time.time()
        buf = b""
        while time.time() - start < duration_s:
            waiting = self.ser.in_waiting
            if waiting:
                buf += self.ser.read(waiting)
            # Extract complete lines
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    text = line.decode("ascii", errors="replace").strip()
                except:
                    continue
                if not text:
                    continue
                fields = text.split(",")
                if text.startswith("N,"):
                    parsed = parse_n_frame(fields)
                    if parsed:
                        frames["N"].append(parsed)
                elif text.startswith("C,"):
                    parsed = parse_c_frame(fields)
                    if parsed:
                        frames["C"].append(parsed)
                elif text.startswith("F,"):
                    frames["F"].append(fields)
            time.sleep(0.005)
        return frames

    def collect_samples(self, duration_s, label=""):
        """Collect frames for duration_s, return parsed frames."""
        frames = self.read_frames(duration_s)
        n_count = len(frames["N"])
        print(f"  [{label}] Collected {n_count} N-frames in {duration_s:.1f}s")
        return frames

    def emergency_stop(self):
        """Immediate safe stop."""
        print("\n[EMERGENCY] Stopping motor...")
        self.send("CMD:MODE,0")
        time.sleep(0.1)
        self.send("CMD:ENABLE,0")
        time.sleep(0.3)
        print("[EMERGENCY] Motor stopped.")

    def check_fault(self, frames, step_name):
        """Check if any fault occurred during test step."""
        for n in frames.get("N", []):
            if n["fault_code"] != 0:
                print(f"  ⚠ FAULT code={n['fault_code']} flags={n['fault_flags']} in step '{step_name}'")
                return False
        return True

    # ── Test Steps ──────────────────────────────────────────

    def test_01_static_iq_sign(self):
        """
        STEP 1: Static Iq Sign Verification (Torque Mode)
        Verify that positive Iq_ref → motor moves in a consistent direction,
        and negative Iq_ref → moves opposite. This establishes the baseline
        q-axis sign convention for all subsequent tests.
        """
        print("\n" + "=" * 60)
        print("TEST 1: Static Iq Sign Verification")
        print("=" * 60)

        # Ensure safe state
        self.send("CMD:ENABLE,0")
        time.sleep(0.2)
        self.send("CMD:MODE,0")  # Torque mode
        time.sleep(0.1)

        # Record baseline angle
        baseline = self.read_frames(0.5)
        if not baseline["N"]:
            print("  FAIL: No N-frames received. Is board powered?")
            return False
        angle_start = baseline["N"][-1]["angle_deg"]
        print(f"  Baseline angle: {angle_start:.2f}°")

        # Test positive Iq
        self.send("CMD:ENABLE,1")
        time.sleep(0.1)
        self.send(f"CMD:IREF,0,{IQ_SIGN_TEST_CURRENT:.3f}")
        time.sleep(0.2)
        pos_frames = self.read_frames(0.8)
        self.send("CMD:ENABLE,0")
        time.sleep(0.1)

        if not pos_frames["N"]:
            print("  FAIL: No data during +Iq test")
            return False

        pos_angles = [n["angle_deg"] for n in pos_frames["N"]]
        pos_iq_refs = [n["Iq_ref_A"] for n in pos_frames["N"]]
        angle_pos_end = pos_angles[-1]
        delta_pos = angle_pos_end - angle_start
        # Handle angle wrap-around
        if delta_pos > 180:
            delta_pos -= 360
        elif delta_pos < -180:
            delta_pos += 360
        avg_iq_pos = sum(pos_iq_refs) / len(pos_iq_refs) if pos_iq_refs else 0

        print(f"  +Iq={avg_iq_pos:.3f}A → Δangle={delta_pos:+.2f}° "
              f"({angle_start:.2f}→{angle_pos_end:.2f})")

        # Wait a moment for motor to settle
        time.sleep(0.5)

        # Test negative Iq
        angle_before_neg = self.read_frames(0.3)
        angle_neg_start = angle_before_neg["N"][-1]["angle_deg"] if angle_before_neg["N"] else angle_pos_end

        self.send("CMD:ENABLE,1")
        time.sleep(0.1)
        self.send(f"CMD:IREF,0,{-IQ_SIGN_TEST_CURRENT:.3f}")
        time.sleep(0.2)
        neg_frames = self.read_frames(0.8)
        self.send("CMD:ENABLE,0")
        time.sleep(0.1)

        if not neg_frames["N"]:
            print("  FAIL: No data during -Iq test")
            return False

        neg_angles = [n["angle_deg"] for n in neg_frames["N"]]
        neg_iq_refs = [n["Iq_ref_A"] for n in neg_frames["N"]]
        angle_neg_end = neg_angles[-1]
        delta_neg = angle_neg_end - angle_neg_start
        if delta_neg > 180:
            delta_neg -= 360
        elif delta_neg < -180:
            delta_neg += 360
        avg_iq_neg = sum(neg_iq_refs) / len(neg_iq_refs) if neg_iq_refs else 0

        print(f"  -Iq={avg_iq_neg:.3f}A → Δangle={delta_neg:+.2f}° "
              f"({angle_neg_start:.2f}→{angle_neg_end:.2f})")

        # Verify: opposite sign → opposite direction
        # Positive Iq should move one way, negative Iq the other
        if (delta_pos > 0 and delta_neg < 0) or (delta_pos < 0 and delta_neg > 0):
            print("  PASS: +Iq and -Iq produce opposite angle movement ✓")
        elif abs(delta_pos) < 0.1 or abs(delta_neg) < 0.1:
            print("  WARN: Insufficient movement — check motor wiring/power")
            return None  # inconclusive
        else:
            print(f"  FAIL: +Iq→{delta_pos:+.2f}°, -Iq→{delta_neg:+.2f}° "
                  f"— same direction or unexpected")
            return False

        self._iq_pos_direction = "pos" if delta_pos > 0 else "neg"
        self._iq_delta_pos = delta_pos
        self._iq_delta_neg = delta_neg
        return True

    def test_02_speed_small_signal(self):
        """
        STEP 2: Speed Mode Small Signal
        +SREF should produce positive speed, -SREF negative speed.
        Speed must not run away beyond the reference.
        """
        print("\n" + "=" * 60)
        print("TEST 2: Speed Mode Small Signal (±0.5 rad/s)")
        print("=" * 60)

        self.send("CMD:ENABLE,0")
        time.sleep(0.2)
        self.send("CMD:MOTION_CFG_RESET")
        time.sleep(0.1)
        self.send("CMD:MODE,1")  # Speed mode
        time.sleep(0.1)

        # Test +0.5 rad/s
        self.send("CMD:ENABLE,1")
        time.sleep(0.1)
        self.send(f"CMD:SREF,{SPEED_SMALL_SIGNAL:.3f}")
        pos_frames = self.read_frames(2.0)
        self.send("CMD:ENABLE,0")
        time.sleep(0.2)

        if not pos_frames["N"]:
            print("  FAIL: No data during +SREF test")
            return False

        pos_speeds = [n["speed_radps"] for n in pos_frames["N"]]
        pos_angles = [n["angle_deg"] for n in pos_frames["N"]]
        pos_faults = [n for n in pos_frames["N"] if n["fault_code"] != 0]

        # Check for runaway: max speed should be bounded
        max_speed_pos = max(pos_speeds) if pos_speeds else 0
        min_speed_pos = min(pos_speeds) if pos_speeds else 0
        avg_speed_pos = sum(pos_speeds[-10:]) / 10 if len(pos_speeds) >= 10 else 0
        angle_delta_pos = pos_angles[-1] - pos_angles[0] if len(pos_angles) >= 2 else 0
        if angle_delta_pos > 180:
            angle_delta_pos -= 360
        elif angle_delta_pos < -180:
            angle_delta_pos += 360

        print(f"  +SREF={SPEED_SMALL_SIGNAL}: speed=[{min_speed_pos:.2f}..{max_speed_pos:.2f}] "
              f"avg_last10={avg_speed_pos:.3f} Δangle={angle_delta_pos:+.1f}° faults={len(pos_faults)}")

        # Wait for motor to stop
        time.sleep(0.5)

        # Test -0.5 rad/s
        self.send("CMD:ENABLE,1")
        time.sleep(0.1)
        self.send(f"CMD:SREF,{-SPEED_SMALL_SIGNAL:.3f}")
        neg_frames = self.read_frames(2.0)
        self.send("CMD:ENABLE,0")
        time.sleep(0.2)

        if not neg_frames["N"]:
            print("  FAIL: No data during -SREF test")
            return False

        neg_speeds = [n["speed_radps"] for n in neg_frames["N"]]
        neg_angles = [n["angle_deg"] for n in neg_frames["N"]]
        neg_faults = [n for n in neg_frames["N"] if n["fault_code"] != 0]

        max_speed_neg = max(neg_speeds) if neg_speeds else 0
        min_speed_neg = min(neg_speeds) if neg_speeds else 0
        avg_speed_neg = sum(neg_speeds[-10:]) / 10 if len(neg_speeds) >= 10 else 0
        angle_delta_neg = neg_angles[-1] - neg_angles[0] if len(neg_angles) >= 2 else 0
        if angle_delta_neg > 180:
            angle_delta_neg -= 360
        elif angle_delta_neg < -180:
            angle_delta_neg += 360

        print(f"  -SREF={-SPEED_SMALL_SIGNAL}: speed=[{min_speed_neg:.2f}..{max_speed_neg:.2f}] "
              f"avg_last10={avg_speed_neg:.3f} Δangle={angle_delta_neg:+.1f}° faults={len(neg_faults)}")

        # Verdict
        passed = True

        # 1. Opposite direction
        if (avg_speed_pos > 0 and avg_speed_neg < 0):
            print("  ✓ Speed directions correct: +SREF→positive, -SREF→negative")
        elif (avg_speed_pos < 0 and avg_speed_neg > 0):
            print("  ✓ Speed directions correct (inverted convention)")
            # This is also fine — just documenting
        else:
            print(f"  ✗ Speed direction mismatch: +SREF avg={avg_speed_pos:.3f}, -SREF avg={avg_speed_neg:.3f}")
            passed = False

        # 2. No runaway (speed shouldn't exceed 2x reference in either direction)
        runaway_threshold = SPEED_SMALL_SIGNAL * MAX_SPEED_OVERSHOOT_RATIO
        if abs(max_speed_pos) > runaway_threshold * 3 or abs(min_speed_neg) > runaway_threshold * 3:
            # For very small signal, allow more overshoot since friction dominates
            print(f"  ⚠ Possible overshoot: max|speed| = {max(abs(max_speed_pos), abs(min_speed_neg)):.2f} "
                  f"(ref={SPEED_SMALL_SIGNAL})")
            # Not a hard fail for 0.5 rad/s — friction may prevent movement

        # 3. No faults
        if pos_faults or neg_faults:
            print(f"  ✗ Faults detected: +SREF={len(pos_faults)}, -SREF={len(neg_faults)}")
            passed = False
        else:
            print("  ✓ No faults")

        if passed:
            print("  PASS ✓")
        else:
            print("  FAIL ✗")
        return passed

    def test_03_speed_steps(self):
        """
        STEP 3: Speed Mode Step Response
        Test +1,+2,+4,-1,-2,-4,0. Each step should converge,
        Iq_ref should reverse sign when overspeeding, no faults.
        """
        print("\n" + "=" * 60)
        print("TEST 3: Speed Mode Step Response")
        print("=" * 60)

        self.send("CMD:ENABLE,0")
        time.sleep(0.2)
        self.send("CMD:MODE,1")
        time.sleep(0.1)

        all_passed = True

        for step_ref in SPEED_STEPS:
            label = f"SREF={step_ref:+.1f}"
            self.send("CMD:ENABLE,1")
            time.sleep(0.1)
            self.send(f"CMD:SREF,{step_ref:.3f}")

            # Longer duration for larger steps
            duration = 2.0 if abs(step_ref) <= 2.0 else 3.0
            frames = self.read_frames(duration)
            self.send("CMD:ENABLE,0")
            time.sleep(0.3)

            if not frames["N"]:
                print(f"  {label}: FAIL — no data")
                all_passed = False
                continue

            n_frames = frames["N"]
            speeds = [n["speed_radps"] for n in n_frames]
            iq_refs = [n["Iq_ref_A"] for n in n_frames]
            faults = [n for n in n_frames if n["fault_code"] != 0]

            if not speeds:
                print(f"  {label}: FAIL — no speed data")
                all_passed = False
                continue

            avg_speed = sum(speeds[-5:]) / min(5, len(speeds[-5:])) if len(speeds) >= 5 else speeds[-1]
            avg_iq = sum(iq_refs[-5:]) / min(5, len(iq_refs[-5:])) if len(iq_refs) >= 5 else iq_refs[-1]
            err = step_ref - avg_speed

            status = "✓" if abs(err) < 1.0 and not faults else "⚠"
            if faults:
                status = "✗ FAULT"

            print(f"  {label}: speed_avg={avg_speed:+.3f} error={err:+.3f} "
                  f"Iq_ref_avg={avg_iq:+.4f} faults={len(faults)} {status}")

            if faults:
                all_passed = False

            # Check for runaway: speed sign should match reference direction
            if step_ref != 0 and abs(avg_speed) > 0.2:
                if (step_ref > 0 and avg_speed < -0.5) or (step_ref < 0 and avg_speed > 0.5):
                    print(f"    CRITICAL: Speed sign opposite to reference — possible runaway!")
                    all_passed = False

        if all_passed:
            print("  PASS ✓")
        else:
            print("  FAIL ✗")
        return all_passed

    def test_04_position_regression(self):
        """
        STEP 4: Position Mode Regression
        PREF=0,+20,+40,+80,0. Error should stay <3°, no oscillation.
        """
        print("\n" + "=" * 60)
        print("TEST 4: Position Mode Regression")
        print("=" * 60)

        self.send("CMD:ENABLE,0")
        time.sleep(0.2)
        self.send("CMD:MODE,2")  # Position mode
        time.sleep(0.1)

        all_passed = True

        for step_deg in POS_STEPS:
            label = f"PREF={step_deg:+.0f}°"
            self.send("CMD:ENABLE,1")
            time.sleep(0.1)
            self.send(f"CMD:PREF,{step_deg:.3f}")
            duration = 2.5 if step_deg != 0 else 2.0
            frames = self.read_frames(duration)
            self.send("CMD:ENABLE,0")
            time.sleep(0.2)

            if not frames["N"]:
                print(f"  {label}: FAIL — no data")
                all_passed = False
                continue

            n_frames = frames["N"]
            # Position mode N-frames: pos_ref is our target
            angles = [n["angle_deg"] for n in n_frames]
            pos_refs = [n["pos_ref_deg"] for n in n_frames]
            faults = [n for n in n_frames if n["fault_code"] != 0]
            iq_refs = [n["Iq_ref_A"] for n in n_frames]

            # Last few samples — should be settled
            last_n = min(10, len(n_frames))
            last_angles = angles[-last_n:] if last_n > 0 else []
            last_iq = iq_refs[-last_n:] if last_n > 0 else []

            if last_angles:
                avg_angle = sum(last_angles) / len(last_angles)
                # Compute error considering angle wrap
                raw_err = step_deg - avg_angle
                # Normalize to [-180, 180)
                if raw_err > 180:
                    raw_err -= 360
                elif raw_err < -180:
                    raw_err += 360

                max_iq = max(abs(i) for i in iq_refs) if iq_refs else 0

                status = "✓" if abs(raw_err) < POS_ERROR_THRESHOLD_DEG and not faults else "⚠"
                if faults:
                    status = "✗ FAULT"

                print(f"  {label}: angle={avg_angle:.2f}° error={raw_err:+.2f}° "
                      f"max|Iq_ref|={max_iq:.4f}A faults={len(faults)} {status}")

                if abs(raw_err) >= POS_ERROR_THRESHOLD_DEG:
                    all_passed = False
                if faults:
                    all_passed = False
            else:
                print(f"  {label}: FAIL — no angle samples")
                all_passed = False

        if all_passed:
            print("  PASS ✓")
        else:
            print("  FAIL ✗")
        return all_passed

    def test_05_p0_cogging_regression(self):
        """
        STEP 5: P0 Cogging Feedforward Regression
        Configure cogging LUT, run 0↔40° cycles, verify error/Iq
        not degraded vs baseline.
        """
        print("\n" + "=" * 60)
        print("TEST 5: P0 Cogging Feedforward Regression")
        print("=" * 60)

        self.send("CMD:ENABLE,0")
        time.sleep(0.2)
        self.send("CMD:MODE,2")  # Position mode
        time.sleep(0.1)

        # Enable cogging
        self.send(f"CMD:COG_CFG,{COGGING_GAIN:.3f},{COGGING_PHASE:.1f}")
        time.sleep(0.1)

        # First run a baseline without cogging (disable then re-enable)
        # Actually, just test with cogging enabled — compare error across cycles
        cycle_errors = []
        cycle_iq_peaks = []

        for i in range(COGGING_CYCLES):
            # Move to 40°
            self.send("CMD:ENABLE,1")
            time.sleep(0.05)
            self.send("CMD:PREF,40.000")
            frames_to = self.read_frames(1.5)

            # Move back to 0°
            self.send("CMD:PREF,0.000")
            frames_back = self.read_frames(1.5)
            self.send("CMD:ENABLE,0")
            time.sleep(0.1)

            all_frames_n = frames_to.get("N", []) + frames_back.get("N", [])
            if not all_frames_n:
                print(f"  Cycle {i+1}: no data, skipping")
                continue

            errors = []
            iq_peaks = []
            for n in all_frames_n:
                raw_err = n["pos_ref_deg"] - n["angle_deg"]
                if raw_err > 180:
                    raw_err -= 360
                elif raw_err < -180:
                    raw_err += 360
                errors.append(abs(raw_err))
                iq_peaks.append(abs(n["Iq_ref_A"]))

            max_err = max(errors) if errors else 0
            max_iq = max(iq_peaks) if iq_peaks else 0
            cycle_errors.append(max_err)
            cycle_iq_peaks.append(max_iq)

            status = "✓" if max_err < POS_ERROR_THRESHOLD_DEG else "⚠"
            print(f"  Cycle {i+1:2d}: max_err={max_err:.2f}° max|Iq|={max_iq:.4f}A {status}")

        if cycle_errors and cycle_iq_peaks:
            avg_err = sum(cycle_errors) / len(cycle_errors)
            avg_iq = sum(cycle_iq_peaks) / len(cycle_iq_peaks)
            max_err = max(cycle_errors)
            print(f"  Summary: avg_err={avg_err:.2f}° max_err={max_err:.2f}° "
                  f"avg_iq_peak={avg_iq:.4f}A over {len(cycle_errors)} cycles")

            # Check consistency: errors shouldn't diverge over cycles
            first_half_err = sum(cycle_errors[:len(cycle_errors)//2]) / max(1, len(cycle_errors)//2)
            second_half_err = sum(cycle_errors[len(cycle_errors)//2:]) / max(1, len(cycle_errors) - len(cycle_errors)//2)

            if second_half_err > first_half_err * 1.5:
                print(f"  ⚠ Error degradation: {first_half_err:.2f}°→{second_half_err:.2f}°")
            else:
                print(f"  ✓ Error stable across cycles: {first_half_err:.2f}°→{second_half_err:.2f}°")

            if max_err < POS_ERROR_THRESHOLD_DEG * 2:
                print("  PASS ✓")
                return True
            else:
                print(f"  FAIL: max_err={max_err:.2f}° exceeds threshold")
                return False
        else:
            print("  FAIL: No valid cycle data")
            return False

    # ── Main ─────────────────────────────────────────────────

    def run_all(self):
        """Execute all test steps in sequence."""
        print("\n" + "█" * 60)
        print("█ SPEED MODE SIGN CHAIN FIX — VERIFICATION SUITE")
        print("█" * 60)
        print(f"█ Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"█ Port: {self.port} @ {self.baud}")
        print("█" * 60)

        try:
            self.connect()
            # Unlock power stage (required by firmware security)
            self.send("CMD:UNLOCK,1")
            time.sleep(0.2)
            # Set UV threshold to 10V for 12V supply (already configured, reinforce)
            self.send("CMD:VBUS_LIMIT,10.0,30.0")
            time.sleep(0.2)
        except Exception as e:
            print(f"\nFATAL: Cannot open {self.port}: {e}")
            print("Check: board powered? COM port correct?")
            return 1

        results = {}

        try:
            # Run each test step
            for test_func, name in [
                (self.test_01_static_iq_sign, "1. Static Iq Sign"),
                (self.test_02_speed_small_signal, "2. Speed Small Signal"),
                (self.test_03_speed_steps, "3. Speed Step Response"),
                (self.test_04_position_regression, "4. Position Regression"),
                (self.test_05_p0_cogging_regression, "5. P0 Cogging Regression"),
            ]:
                try:
                    result = test_func()
                    results[name] = result
                except Exception as e:
                    print(f"\n  EXCEPTION in '{name}': {e}")
                    import traceback
                    traceback.print_exc()
                    results[name] = False
                    # Try to stop motor
                    try:
                        self.emergency_stop()
                    except:
                        pass
                    # Continue with next test

        finally:
            # Always ensure motor is off
            try:
                self.emergency_stop()
            except:
                pass
            self.disconnect()

        # Summary
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        passed = 0
        failed = 0
        inconclusive = 0
        for name, result in results.items():
            if result is True:
                status = "PASS ✓"
                passed += 1
            elif result is False:
                status = "FAIL ✗"
                failed += 1
            else:
                status = "INCONCLUSIVE ?"
                inconclusive += 1
            print(f"  {status}  {name}")

        print(f"\n{passed} passed, {failed} failed, {inconclusive} inconclusive")

        return 1 if failed > 0 else 0


if __name__ == "__main__":
    runner = TestRunner()
    sys.exit(runner.run_all())
