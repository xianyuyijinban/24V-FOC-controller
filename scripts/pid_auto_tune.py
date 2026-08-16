"""
FOC PID Auto-Tuning Engine
===========================
Autonomous step-response-based PID tuning for 24V FOC controller.

Protocol: UART text commands (COM9, 230400 baud), terminated with \\n or \\r\\n.
Uses HostComputer/data_parser.py for N-frame telemetry parsing.

Tuning strategy (innermost-loop-first):
  Current loop → analytical from R/L, then step-response verify
  Speed loop   → step-response heuristic tune
  Position loop → step-response heuristic tune

Usage:
  python pid_auto_tune.py current  [--port COM9] [--baud 230400]
  python pid_auto_tune.py speed    [--port COM9] [--baud 230400]
  python pid_auto_tune.py position [--port COM9] [--baud 230400]
  python pid_auto_tune.py all      [--port COM9] [--baud 230400]  # all 3 loops

Author: Claude Code PID Auto-Tune Skill
Project: E:\\24V_FOC_Controller_sync_20260519
"""

import serial
import time
import csv
import json
import math
import sys
import os
import argparse
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List, Dict, Tuple, Callable
from collections import deque

# ── Path setup ────────────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
HOST_DIR = PROJECT_ROOT / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class StepMetrics:
    """Step response quality metrics for a single step test."""
    rise_time_ms: float = 0.0         # 10%→90% rise time
    peak_time_ms: float = 0.0         # time to first peak
    overshoot_pct: float = 0.0        # % overshoot
    settling_time_ms: float = 0.0     # time to stay within ±2% band
    steady_state_error_pct: float = 0.0  # final error as % of step
    steady_state_std: float = 0.0     # std dev in steady state
    rms_error: float = 0.0            # RMS of error over entire test
    oscillation_count: int = 0        # # of zero-crossings after peak
    is_stable: bool = True            # did it converge?
    fault: bool = False               # fault during test?
    raw_data: List[dict] = field(default_factory=list)

    def score(self, overshoot_weight: float = 3.0, speed_weight: float = 1.0,
              error_weight: float = 2.0, osc_weight: float = 2.0) -> float:
        """Composite score — lower is better."""
        if self.fault or not self.is_stable:
            return float('inf')
        return (
            overshoot_weight * self.overshoot_pct / 100.0 +
            speed_weight * self.settling_time_ms / 1000.0 +
            error_weight * abs(self.steady_state_error_pct) / 100.0 +
            osc_weight * self.oscillation_count
        )


@dataclass
class TuneResult:
    """Result of tuning one loop."""
    loop: str                      # "current", "speed", "position"
    kp_final: float
    ki_final: float
    kd_final: float = 0.0
    iterations: int = 0
    final_metrics: Optional[StepMetrics] = None
    history: List[dict] = field(default_factory=list)
    success: bool = False
    notes: str = ""


# ── FOC Board Interface ───────────────────────────────────────────────────────

class FOCBoard:
    """High-level interface to the FOC controller over UART."""

    DEFAULT_PORT = "COM9"
    DEFAULT_BAUD = 230400

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.parser = FOCDataParser()
        self.packets: List = []
        self.ack_lines: List[str] = []
        self.diag_lines: List[str] = []
        self._latest_n: Optional[object] = None

        self.parser.set_packet_callback(self._on_packet)
        self.parser.set_diagnostic_callback(self._on_diag)

    def _on_packet(self, pkt):
        self.packets.append(pkt)
        if pkt.raw_text.startswith("N,") and pkt.foc_state == 4:
            self._latest_n = pkt

    def _on_diag(self, line: str):
        self.diag_lines.append(line)
        # Also track ACK lines
        for prefix in ("UNLOCK,", "ENABLE,", "MODE,", "IREF,", "SREF,",
                        "PREF,", "PI_CURRENT,", "PI_SPEED,", "PD_POS,",
                        "CLEAR_FAULT,", "MOTOR_PN,", "ENCODER_DIR,",
                        "VBUS_LIMIT,", "IDENTIFY,", "HOME,", "CLEAR_HOME,"):
            if line.startswith(prefix):
                self.ack_lines.append(line.strip())
                break

    @property
    def latest_n(self):
        return self._latest_n

    def connect(self) -> bool:
        """Open serial port and establish connection."""
        print(f"[CONNECT] Opening {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.3)
        return True

    def disconnect(self):
        """Safely shut down and close serial port."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[DISCONNECT] Serial closed.")

    def drain(self, duration_s: float = 0.3):
        """Read and parse all buffered data for `duration_s` seconds."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            self.ser.timeout = max(0.01, min(remaining, 0.1))
            try:
                data = self.ser.read(4096)
                if data:
                    self.parser.feed_data(data)
            except Exception:
                break

    def send(self, cmd_str: str, wait: float = 0.15):
        """Send a raw command string (MUST include \\n terminator)."""
        self.packets.clear()
        self.ack_lines.clear()
        self.diag_lines.clear()
        self.ser.reset_input_buffer()
        self.ser.write(cmd_str.encode('utf-8'))
        if wait > 0:
            self.drain(wait)

    def cmd(self, cmd_str: str, wait: float = 0.15, check_ack: bool = True) -> bool:
        """Send a command (without \\n — added automatically). Returns True if ACK OK."""
        self.send(cmd_str + '\n', wait=wait)
        if check_ack:
            for ack in self.ack_lines:
                if "OK" in ack:
                    return True
                if "FAIL" in ack:
                    print(f"  [WARN] ACK FAIL: {ack}")
                    return False
        return True

    def get_foc_state(self) -> int:
        """Get current FOC state (0=IDLE..4=RUNNING,5=FAULT)."""
        if self.latest_n:
            return self.latest_n.foc_state
        return -1

    def is_running(self) -> bool:
        return self.get_foc_state() == 4

    def emergency_stop(self):
        """Immediately disable motor and lock power stage."""
        self.send("CMD:ENABLE,0\n", wait=0.1)
        self.send("CMD:UNLOCK,0\n", wait=0.1)
        print("[ESTOP] Motor disabled, power locked.")


# ── Step Test Engine ──────────────────────────────────────────────────────────

class StepTester:
    """Runs step-response tests on a specific control loop and computes metrics."""

    def __init__(self, board: FOCBoard):
        self.board = board

    def current_step(self, iq_step: float = 0.10, hold_s: float = 1.0) -> StepMetrics:
        """Run a current (Iq) step test. Motor must be in torque mode (MODE=0) and enabled."""
        return self._run_step(
            setpoint_cmd=lambda v: CommandBuilder.set_current_ref(0.0, v),
            zero_cmd=lambda: CommandBuilder.set_current_ref(0.0, 0.0),
            step_amplitude=iq_step,
            hold_s=hold_s,
            measurement_field='Iq',
            ref_field='Iq_ref'
        )

    def speed_step(self, speed_step: float = 1.0, hold_s: float = 2.0) -> StepMetrics:
        """Run a speed step test. Motor must be in speed mode (MODE=1) and enabled."""
        return self._run_step(
            setpoint_cmd=lambda v: CommandBuilder.set_speed_ref(v),
            zero_cmd=lambda: CommandBuilder.set_speed_ref(0.0),
            step_amplitude=speed_step,
            hold_s=hold_s,
            measurement_field='speed',
            ref_field='speed_ref'
        )

    def position_step(self, pos_step: float = 0.5, hold_s: float = 2.0) -> StepMetrics:
        """Run a position step test. Motor must be in position mode (MODE=2) and enabled."""
        return self._run_step(
            setpoint_cmd=lambda v: CommandBuilder.set_position_ref(v),
            zero_cmd=lambda: CommandBuilder.set_position_ref(0.0),
            step_amplitude=pos_step,
            hold_s=hold_s,
            measurement_field='angle',
            ref_field='pos_ref'
        )

    def _run_step(self, setpoint_cmd: Callable, zero_cmd: Callable,
                  step_amplitude: float, hold_s: float,
                  measurement_field: str, ref_field: str) -> StepMetrics:
        """Generic step test: zero → step → zero, collecting N-frames throughout."""
        metrics = StepMetrics()
        raw = []

        # Clear buffer and start recording
        self.board.packets.clear()
        self.board.drain(0.3)

        # Apply step
        t0 = time.time()
        self.board.send(setpoint_cmd(step_amplitude), wait=0.02)

        # Record during hold
        while time.time() - t0 < hold_s:
            self.board.drain(0.05)

        # Return to zero
        self.board.send(zero_cmd(), wait=0.02)
        t_end = time.time()

        # Collect one more drain
        self.board.drain(0.3)

        # Extract N-frame data
        raw = [
            {
                'ts': pkt.timestamp,
                'field': getattr(pkt, measurement_field, 0.0),
                'ref': getattr(pkt, ref_field, 0.0),
                'fault': pkt.fault_flags,
                'app_fault': pkt.app_fault_code,
            }
            for pkt in self.board.packets
            if pkt.raw_text.startswith("N,") and pkt.foc_state == 4
        ]

        if not raw or len(raw) < 5:
            metrics.is_stable = False
            metrics.raw_data = raw
            return metrics

        # Check for faults
        faults = [r for r in raw if r['fault'] or r['app_fault']]
        if faults:
            metrics.fault = True
            metrics.is_stable = False
            metrics.raw_data = raw
            return metrics

        # Use timestamps relative to first packet
        t_start = raw[0]['ts']
        times = [(r['ts'] - t_start) / 1000.0 for r in raw]  # seconds
        values = [r['field'] for r in raw]
        refs = [r['ref'] for r in raw]

        # Find step onset (when ref changes from ~0)
        step_onset_idx = 0
        for i, r in enumerate(refs):
            if abs(r) > 0.01 * abs(step_amplitude):
                step_onset_idx = i
                break

        if step_onset_idx >= len(values) - 1:
            metrics.is_stable = False
            metrics.raw_data = raw
            return metrics

        # Compute metrics from step onset onward
        t0 = times[step_onset_idx]
        times = [t - t0 for t in times[step_onset_idx:]]
        values = values[step_onset_idx:]
        refs = refs[step_onset_idx:]

        target = abs(step_amplitude)
        errors = [v - r for v, r in zip(values, refs)]
        abs_errors = [abs(e) for e in errors]

        # Rise time: 10%→90% of target
        rise_start = None
        rise_end = None
        for t, v in zip(times, values):
            av = abs(v)
            if rise_start is None and av >= 0.10 * target:
                rise_start = t
            if rise_start is not None and rise_end is None and av >= 0.90 * target:
                rise_end = t
                break
        metrics.rise_time_ms = ((rise_end - rise_start) * 1000.0) if (rise_start and rise_end) else 999.0

        # Peak and overshoot
        peak_idx = max(range(len(values)), key=lambda i: abs(values[i]))
        peak_val = abs(values[peak_idx])
        metrics.peak_time_ms = times[peak_idx] * 1000.0
        metrics.overshoot_pct = max(0.0, (peak_val - target) / target * 100.0) if target > 0.001 else 0.0

        # Settling time: last time error exceeds ±2% band
        band = 0.02 * target
        settle_idx = len(times) - 1
        for i in range(len(times) - 1, -1, -1):
            if abs_errors[i] > band:
                settle_idx = i
                break
        else:
            settle_idx = 0
        metrics.settling_time_ms = times[settle_idx] * 1000.0

        # Steady-state error (last 20% of data)
        ss_start = max(0, int(0.8 * len(values)))
        ss_errors = errors[ss_start:]
        if ss_errors:
            metrics.steady_state_error_pct = (sum(ss_errors) / len(ss_errors)) / target * 100.0
            metrics.steady_state_std = (sum((e - metrics.steady_state_error_pct * target / 100.0)**2
                                            for e in ss_errors) / len(ss_errors)) ** 0.5

        # RMS error
        metrics.rms_error = (sum(e**2 for e in errors) / len(errors)) ** 0.5

        # Oscillation count: zero-crossings of the derivative after peak
        deriv = [values[i+1] - values[i] for i in range(len(values)-1)]
        if peak_idx < len(deriv):
            post_peak_deriv = deriv[peak_idx:]
            metrics.oscillation_count = sum(
                1 for i in range(1, len(post_peak_deriv))
                if post_peak_deriv[i-1] * post_peak_deriv[i] < 0
            )

        # Stability check: did values converge?
        if ss_errors:
            metrics.is_stable = (
                metrics.overshoot_pct < 80.0 and
                abs(metrics.steady_state_error_pct) < 50.0 and
                metrics.oscillation_count < 20
            )

        metrics.raw_data = raw
        return metrics


# ── PID Tuners ────────────────────────────────────────────────────────────────

class CurrentLoopTuner:
    """Tune current loop PI using analytical formulas + step verify."""

    def __init__(self, board: FOCBoard, tester: StepTester):
        self.board = board
        self.tester = tester

    def tune(self, R: Optional[float] = None, L: Optional[float] = None,
             bw_hz: float = 500.0, verify: bool = True) -> TuneResult:
        """Tune current loop. If R/L are provided, compute analytically.
        Otherwise, query from motor params and use heuristic sweep."""

        result = TuneResult(loop="current")

        # Try to get motor parameters from FAULT_DETAIL
        if R is None or L is None:
            self.board.cmd("CMD:FAULT_DETAIL", wait=0.3)
            self.board.drain(0.3)
            # Try to extract from latest N frame's extra fields
            # Fallback: use heuristic
            if R is None:
                R = 0.5   # Default Ω — typical small BLDC
            if L is None:
                L = 0.0005  # Default H

        # Analytical computation
        kp = 2.0 * math.pi * bw_hz * L
        ki = 2.0 * math.pi * bw_hz * R

        result.kp_final = kp
        result.ki_final = ki

        print(f"\n[CURRENT LOOP] Analytical: Kp={kp:.6f}, Ki={ki:.6f}")
        print(f"  (R={R:.4f}Ω, L={L*1000:.3f}mH, BW={bw_hz}Hz)")

        # Apply
        self.board.cmd(CommandBuilder.set_current_pi(kp, ki).strip(), wait=0.15)

        if verify:
            print("[CURRENT LOOP] Verifying with step test...")
            metrics = self.tester.current_step(iq_step=0.10, hold_s=1.0)
            result.final_metrics = metrics
            result.iterations = 1
            self._print_metrics(metrics)

            if metrics.fault:
                result.success = False
                result.notes = "Fault during verification"
                return result

            # Simple adjustment: reduce if overshoot > 10%
            if metrics.overshoot_pct > 10.0:
                kp *= 0.7
                ki *= 0.7
                self.board.cmd(CommandBuilder.set_current_pi(kp, ki).strip(), wait=0.15)
                result.kp_final = kp
                result.ki_final = ki
                result.notes = "Reduced gains due to overshoot"
                print(f"  → Adjusted: Kp={kp:.6f}, Ki={ki:.6f}")
                result.iterations = 2
            elif metrics.steady_state_error_pct > 5.0:
                ki *= 1.5
                self.board.cmd(CommandBuilder.set_current_pi(kp, ki).strip(), wait=0.15)
                result.kp_final = kp
                result.ki_final = ki
                result.notes = "Increased Ki to reduce steady-state error"
                print(f"  → Adjusted: Kp={kp:.6f}, Ki={ki:.6f}")
                result.iterations = 2

            result.success = metrics.is_stable and not metrics.fault

        return result

    @staticmethod
    def _print_metrics(m: StepMetrics):
        print(f"  Rise: {m.rise_time_ms:.0f}ms | Peak: {m.peak_time_ms:.0f}ms")
        print(f"  Overshoot: {m.overshoot_pct:.1f}% | Settling: {m.settling_time_ms:.0f}ms")
        print(f"  SSE: {m.steady_state_error_pct:.1f}% | OscCount: {m.oscillation_count}")
        print(f"  RMS err: {m.rms_error:.4f} | Stable: {m.is_stable} | Fault: {m.fault}")


class SpeedLoopTuner:
    """Tune speed loop PI using iterative step-response heuristic."""

    MAX_ITERATIONS = 15
    CONVERGENCE_TOL = 0.005  # score improvement threshold

    def __init__(self, board: FOCBoard, tester: StepTester):
        self.board = board
        self.tester = tester

    def tune(self, kp_init: float = 0.05, ki_init: float = 0.0,
             step_rad_s: float = 1.0, hold_s: float = 2.0) -> TuneResult:
        """Iteratively tune speed loop PI."""

        result = TuneResult(loop="speed")
        kp, ki = kp_init, ki_init

        self.board.cmd(CommandBuilder.set_speed_pi(kp, ki).strip(), wait=0.15)
        print(f"\n[SPEED LOOP] Initial: Kp={kp:.6f}, Ki={ki:.6f}")

        best_score = float('inf')
        best_kp, best_ki = kp, ki
        best_metrics = None

        for iteration in range(self.MAX_ITERATIONS):
            print(f"\n--- Iteration {iteration + 1}/{self.MAX_ITERATIONS} ---")
            print(f"  Kp={kp:.6f}, Ki={ki:.6f}")

            # Run step test
            metrics = self.tester.speed_step(speed_step=step_rad_s, hold_s=hold_s)
            self._print_metrics(metrics)

            if metrics.fault:
                print("  [ABORT] Fault detected!")
                result.notes = "Fault during tuning"
                break

            score = metrics.score()
            print(f"  Score: {score:.4f} (best: {best_score:.4f})")

            result.history.append({
                'iteration': iteration,
                'kp': kp, 'ki': ki,
                'metrics': {
                    'rise_time_ms': metrics.rise_time_ms,
                    'overshoot_pct': metrics.overshoot_pct,
                    'settling_time_ms': metrics.settling_time_ms,
                    'sse_pct': metrics.steady_state_error_pct,
                    'osc_count': metrics.oscillation_count,
                    'rms_error': metrics.rms_error,
                    'score': score,
                }
            })

            # Track best
            if score < best_score:
                best_score = score
                best_kp, best_ki = kp, ki
                best_metrics = metrics

            # Convergence check
            if iteration > 2:
                recent_scores = [h['metrics']['score'] for h in result.history[-3:]]
                if max(recent_scores) - min(recent_scores) < self.CONVERGENCE_TOL:
                    print("  [CONVERGED] Score plateau detected.")
                    break

            # Heuristic adjustment rules
            if metrics.overshoot_pct > 20.0:
                # Too much overshoot → reduce Kp, increase Ki slightly for damping
                kp *= 0.6
                ki = max(0.0, ki * 0.7)
                print("  → Rule: High overshoot → reduce Kp, reduce Ki")
            elif metrics.overshoot_pct > 5.0 and metrics.oscillation_count > 3:
                # Oscillating → reduce both
                kp *= 0.75
                ki *= 0.5
                print("  → Rule: Oscillating → reduce Kp, reduce Ki")
            elif metrics.rise_time_ms > 500:
                # Too slow → increase Kp
                kp *= 1.8
                print("  → Rule: Slow rise → increase Kp")
            elif abs(metrics.steady_state_error_pct) > 5.0:
                # Steady-state error → increase Ki
                ki = max(0.001, ki + abs(metrics.steady_state_error_pct) * kp * 0.02)
                print("  → Rule: SSE → increase Ki")
            elif metrics.rise_time_ms > 200:
                # Could be faster → slightly increase Kp
                kp *= 1.3
                print("  → Rule: Could be faster → slight Kp increase")
            elif metrics.oscillation_count > 2:
                # Small oscillations → slight Ki reduction
                ki *= 0.7
                print("  → Rule: Small oscillations → reduce Ki")
            else:
                # Good response — try pushing a bit more
                if metrics.overshoot_pct < 2.0 and metrics.rise_time_ms > 100:
                    kp *= 1.2
                    print("  → Rule: Good, but try faster → small Kp bump")
                else:
                    print("  → Good enough, converging...")
                    break

            # Clamp
            kp = max(0.001, min(kp, 5.0))
            ki = max(0.0, min(ki, 5.0))

            self.board.cmd(CommandBuilder.set_speed_pi(kp, ki).strip(), wait=0.15)

        # Restore best
        print(f"\n[SPEED LOOP] Best: Kp={best_kp:.6f}, Ki={best_ki:.6f}, score={best_score:.4f}")
        self.board.cmd(CommandBuilder.set_speed_pi(best_kp, best_ki).strip(), wait=0.15)

        result.kp_final = best_kp
        result.ki_final = best_ki
        result.final_metrics = best_metrics
        result.iterations = len(result.history)
        result.success = best_metrics is not None and best_metrics.is_stable and not best_metrics.fault

        if not result.success:
            result.notes = (result.notes + "; " if result.notes else "") + "Did not converge to stable tune"

        return result

    @staticmethod
    def _print_metrics(m: StepMetrics):
        print(f"  Rise: {m.rise_time_ms:.0f}ms | Peak: {m.peak_time_ms:.0f}ms")
        print(f"  Overshoot: {m.overshoot_pct:.1f}% | Settling: {m.settling_time_ms:.0f}ms")
        print(f"  SSE: {m.steady_state_error_pct:.1f}% | σ: {m.steady_state_std:.4f}")
        print(f"  OscCount: {m.oscillation_count} | RMS err: {m.rms_error:.4f}")
        print(f"  Stable: {m.is_stable} | Fault: {m.fault}")


class PositionLoopTuner:
    """Tune position loop PD using iterative step-response heuristic."""

    MAX_ITERATIONS = 12

    def __init__(self, board: FOCBoard, tester: StepTester):
        self.board = board
        self.tester = tester

    def tune(self, kp_init: float = 2.0, kd_init: float = 0.05,
             step_rad: float = 0.5, hold_s: float = 2.0) -> TuneResult:
        """Iteratively tune position loop PD."""

        result = TuneResult(loop="position")
        kp, kd = kp_init, kd_init

        self.board.cmd(CommandBuilder.set_position_pd(kp, kd).strip(), wait=0.15)
        print(f"\n[POSITION LOOP] Initial: Kp={kp:.6f}, Kd={kd:.6f}")

        best_score = float('inf')
        best_kp, best_kd = kp, kd
        best_metrics = None

        for iteration in range(self.MAX_ITERATIONS):
            print(f"\n--- Iteration {iteration + 1}/{self.MAX_ITERATIONS} ---")
            print(f"  Kp={kp:.6f}, Kd={kd:.6f}")

            # First go to zero
            self.board.cmd(CommandBuilder.set_position_ref(0.0), wait=0.5)
            self.board.drain(0.3)

            metrics = self.tester.position_step(pos_step=step_rad, hold_s=hold_s)
            self._print_metrics(metrics)

            if metrics.fault:
                print("  [ABORT] Fault detected!")
                result.notes = "Fault during tuning"
                break

            score = metrics.score(overshoot_weight=5.0, osc_weight=3.0)
            print(f"  Score: {score:.4f} (best: {best_score:.4f})")

            result.history.append({
                'iteration': iteration,
                'kp': kp, 'kd': kd,
                'metrics': {
                    'rise_time_ms': metrics.rise_time_ms,
                    'overshoot_pct': metrics.overshoot_pct,
                    'settling_time_ms': metrics.settling_time_ms,
                    'sse_pct': metrics.steady_state_error_pct,
                    'osc_count': metrics.oscillation_count,
                    'score': score,
                }
            })

            if score < best_score:
                best_score = score
                best_kp, best_kd = kp, kd
                best_metrics = metrics

            # Position loop heuristic rules
            if metrics.overshoot_pct > 15.0 and metrics.oscillation_count > 2:
                # Oscillating → reduce Kp, increase Kd
                kp *= 0.7
                kd *= 1.5
                print("  → Rule: Oscillation → reduce Kp, increase Kd")
            elif metrics.overshoot_pct > 10.0:
                kp *= 0.7
                kd *= 1.3
                print("  → Rule: Overshoot → reduce Kp, increase Kd")
            elif metrics.rise_time_ms > 500:
                kp *= 1.5
                print("  → Rule: Slow rise → increase Kp")
            elif abs(metrics.steady_state_error_pct) > 3.0:
                kp *= 1.3
                print("  → Rule: SSE → increase Kp (P-only position loop)")
            elif metrics.oscillation_count > 3:
                kd *= 1.5
                print("  → Rule: Ringing → increase Kd for damping")
            else:
                # Try a bit faster
                if metrics.overshoot_pct < 3.0 and metrics.rise_time_ms > 150:
                    kp *= 1.15
                    print("  → Rule: Good, try faster → small Kp bump")
                else:
                    print("  → Good enough, converging...")
                    break

            # Clamp
            kp = max(0.1, min(kp, 20.0))
            kd = max(0.0, min(kd, 1.0))

            self.board.cmd(CommandBuilder.set_position_pd(kp, kd).strip(), wait=0.15)

        print(f"\n[POSITION LOOP] Best: Kp={best_kp:.6f}, Kd={best_kd:.6f}, score={best_score:.4f}")
        self.board.cmd(CommandBuilder.set_position_pd(best_kp, best_kd).strip(), wait=0.15)

        result.kp_final = best_kp
        result.kd_final = best_kd
        result.final_metrics = best_metrics
        result.iterations = len(result.history)
        result.success = best_metrics is not None and best_metrics.is_stable and not best_metrics.fault

        if not result.success:
            result.notes = (result.notes + "; " if result.notes else "") + "Did not converge to stable tune"

        return result

    @staticmethod
    def _print_metrics(m: StepMetrics):
        print(f"  Rise: {m.rise_time_ms:.0f}ms | Peak: {m.peak_time_ms:.0f}ms")
        print(f"  Overshoot: {m.overshoot_pct:.1f}% | Settling: {m.settling_time_ms:.0f}ms")
        print(f"  SSE: {m.steady_state_error_pct:.1f}% | OscCount: {m.oscillation_count}")
        print(f"  Stable: {m.is_stable} | Fault: {m.fault}")


# ── Orchestrator ──────────────────────────────────────────────────────────────

class PIDAutoTuner:
    """Top-level orchestrator: bring up board, tune loops in correct order."""

    def __init__(self, port: str = "COM9", baud: int = 230400):
        self.board = FOCBoard(port, baud)
        self.tester = StepTester(self.board)
        self.results: Dict[str, TuneResult] = {}
        self.log_dir: Optional[Path] = None

    def setup(self, baseline_current_pi: Tuple[float, float] = (0.03, 0.50),
              baseline_speed_pi: Tuple[float, float] = (0.30, 0.0),
              baseline_pos_pd: Tuple[float, float] = (4.0, 0.12)) -> bool:
        """Pre-flight: connect, clear faults, unlock, enable, apply baseline params."""

        if not self.board.connect():
            return False

        print("\n=== Pre-flight ===")

        # 1. Vbus limits
        print("[1/7] Setting Vbus limits...")
        self.board.cmd("CMD:VBUS_LIMIT,8.000,30.000", wait=0.2)

        # 2. Clear fault
        print("[2/7] Clearing fault...")
        self.board.cmd("CMD:CLEAR_FAULT", wait=0.2)
        self.board.drain(0.3)

        # 3. Apply baseline current PI
        print(f"[3/7] Baseline current PI: {baseline_current_pi}")
        self.board.cmd(f"CMD:PI_CURRENT,{baseline_current_pi[0]:.6f},{baseline_current_pi[1]:.6f}", wait=0.15)

        # 4. Apply baseline speed PI
        print(f"[4/7] Baseline speed PI: {baseline_speed_pi}")
        self.board.cmd(f"CMD:PI_SPEED,{baseline_speed_pi[0]:.6f},{baseline_speed_pi[1]:.6f}", wait=0.15)

        # 5. Apply baseline position PD
        print(f"[5/7] Baseline position PD: {baseline_pos_pd}")
        self.board.cmd(f"CMD:PD_POS,{baseline_pos_pd[0]:.6f},{baseline_pos_pd[1]:.6f}", wait=0.15)

        # 6. Unlock
        print("[6/7] Unlocking power stage...")
        self.board.cmd("CMD:UNLOCK,1", wait=0.3)

        # 7. Check state
        print("[7/7] Checking FOC state...")
        self.board.cmd("CMD:FAULT_DETAIL", wait=0.3)
        self.board.drain(0.3)

        state = self.board.get_foc_state()
        print(f"  FOC state: {state}")

        if state < 0:
            print("  [WARN] No telemetry received — continuing anyway")
        elif state == 5:
            print("  [ERROR] Controller in FAULT state!")
            return False

        print("=== Pre-flight complete ===\n")
        return True

    def tune_current_loop(self, R: float = None, L: float = None,
                          bw_hz: float = 500.0, verify: bool = True) -> TuneResult:
        """Tune current loop in torque mode."""
        print("\n" + "=" * 60)
        print("  CURRENT LOOP TUNING")
        print("=" * 60)

        # Set torque mode and enable
        print("[SETUP] Torque mode + enable...")
        self.board.cmd("CMD:MODE,0", wait=0.2)
        self.board.cmd("CMD:ENABLE,1", wait=0.3)

        if not self.board.is_running():
            print("[WARN] Motor not RUNNING — attempting to proceed anyway")

        tuner = CurrentLoopTuner(self.board, self.tester)
        result = tuner.tune(R=R, L=L, bw_hz=bw_hz, verify=verify)
        self.results['current'] = result
        return result

    def tune_speed_loop(self, kp_init: float = 0.05, ki_init: float = 0.0,
                        step_rad_s: float = 1.0) -> TuneResult:
        """Tune speed loop in speed mode. Requires current loop already tuned."""
        print("\n" + "=" * 60)
        print("  SPEED LOOP TUNING")
        print("=" * 60)

        # Switch to speed mode
        print("[SETUP] Speed mode + enable...")
        self.board.cmd("CMD:MODE,1", wait=0.2)
        self.board.cmd("CMD:ENABLE,1", wait=0.3)

        if not self.board.is_running():
            # Try re-enable
            self.board.cmd("CMD:ENABLE,0", wait=0.15)
            self.board.cmd("CMD:UNLOCK,1", wait=0.2)
            self.board.cmd("CMD:MODE,1", wait=0.15)
            self.board.cmd("CMD:ENABLE,1", wait=0.3)

        tuner = SpeedLoopTuner(self.board, self.tester)
        result = tuner.tune(kp_init=kp_init, ki_init=ki_init,
                           step_rad_s=step_rad_s)
        self.results['speed'] = result
        return result

    def tune_position_loop(self, kp_init: float = 2.0, kd_init: float = 0.05,
                           step_rad: float = 0.5) -> TuneResult:
        """Tune position loop in position mode."""
        print("\n" + "=" * 60)
        print("  POSITION LOOP TUNING")
        print("=" * 60)

        # Switch to position mode
        print("[SETUP] Position mode + enable...")
        self.board.cmd("CMD:MODE,2", wait=0.2)
        self.board.cmd("CMD:MOTION_CFG_RESET", wait=0.15)
        self.board.cmd("CMD:ENABLE,1", wait=0.3)

        if not self.board.is_running():
            self.board.cmd("CMD:ENABLE,0", wait=0.15)
            self.board.cmd("CMD:UNLOCK,1", wait=0.2)
            self.board.cmd("CMD:MODE,2", wait=0.15)
            self.board.cmd("CMD:ENABLE,1", wait=0.3)

        tuner = PositionLoopTuner(self.board, self.tester)
        result = tuner.tune(kp_init=kp_init, kd_init=kd_init,
                           step_rad=step_rad)
        self.results['position'] = result
        return result

    def shutdown(self):
        """Safe shutdown: disable motor, lock power, disconnect."""
        print("\n=== Shutdown ===")
        try:
            self.board.cmd("CMD:SREF,0", wait=0.1)
            self.board.cmd("CMD:IREF,0.000,0.000", wait=0.1)
            self.board.cmd("CMD:ENABLE,0", wait=0.15)
            self.board.cmd("CMD:UNLOCK,0", wait=0.15)
        except Exception:
            pass
        self.board.disconnect()

    def save_report(self, log_dir: Optional[Path] = None):
        """Save tuning results to JSON report and CSV log."""
        if log_dir is None:
            log_dir = Path(__file__).resolve().parent / "pid_tune_logs"
        os.makedirs(log_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = log_dir / f"tune_report_{ts}.json"

        report = {}
        for loop, r in self.results.items():
            report[loop] = {
                'success': r.success,
                'iterations': r.iterations,
                'kp_final': r.kp_final,
                'ki_final': r.ki_final,
                'kd_final': r.kd_final,
                'notes': r.notes,
                'final_metrics': None,
                'history': r.history,
            }
            if r.final_metrics:
                report[loop]['final_metrics'] = {
                    'rise_time_ms': r.final_metrics.rise_time_ms,
                    'overshoot_pct': r.final_metrics.overshoot_pct,
                    'settling_time_ms': r.final_metrics.settling_time_ms,
                    'sse_pct': r.final_metrics.steady_state_error_pct,
                    'osc_count': r.final_metrics.oscillation_count,
                    'rms_error': r.final_metrics.rms_error,
                    'is_stable': r.final_metrics.is_stable,
                }

        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        print(f"\n[REPORT] Saved to: {report_path}")

        # Print summary
        print("\n" + "=" * 70)
        print("  TUNING SUMMARY")
        print("=" * 70)
        for loop, r in self.results.items():
            status = "✓ OK" if r.success else "✗ FAIL"
            print(f"  {loop:12s}  {status:6s}  "
                  f"Kp={r.kp_final:.6f}  Ki={r.ki_final:.6f}  Kd={r.kd_final:.6f}  "
                  f"iters={r.iterations}")
            if r.notes:
                print(f"             Notes: {r.notes}")

        # Print commands to apply these results
        print("\n--- Commands to apply these gains ---")
        for loop, r in self.results.items():
            if loop == 'current':
                print(f"  CMD:PI_CURRENT,{r.kp_final:.6f},{r.ki_final:.6f}")
            elif loop == 'speed':
                print(f"  CMD:PI_SPEED,{r.kp_final:.6f},{r.ki_final:.6f}")
            elif loop == 'position':
                print(f"  CMD:PD_POS,{r.kp_final:.6f},{r.kd_final:.6f}")
        print("=" * 70)

        return report_path


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="FOC PID Auto-Tuning Engine",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pid_auto_tune.py current
  python pid_auto_tune.py speed   --kp 0.05 --ki 0.0
  python pid_auto_tune.py position --kp 2.0 --kd 0.05
  python pid_auto_tune.py all     --port COM9 --baud 230400
        """
    )
    parser.add_argument("loop", choices=["current", "speed", "position", "all"],
                       help="Which loop to tune")
    parser.add_argument("--port", default="COM9", help="Serial port (default: COM9)")
    parser.add_argument("--baud", type=int, default=230400,
                       help="Baud rate (default: 230400)")
    parser.add_argument("--kp", type=float, default=None,
                       help="Initial Kp (for speed/position loops)")
    parser.add_argument("--ki", type=float, default=None,
                       help="Initial Ki (for speed loop)")
    parser.add_argument("--kd", type=float, default=None,
                       help="Initial Kd (for position loop)")
    parser.add_argument("--R", type=float, default=None,
                       help="Motor phase resistance (Ω, for current loop)")
    parser.add_argument("--L", type=float, default=None,
                       help="Motor phase inductance (H, for current loop)")
    parser.add_argument("--bw", type=float, default=500.0,
                       help="Current loop bandwidth Hz (default: 500)")
    parser.add_argument("--step", type=float, default=None,
                       help="Step amplitude (speed: rad/s, position: rad)")
    parser.add_argument("--hold", type=float, default=2.0,
                       help="Hold duration per step test (seconds)")
    parser.add_argument("--dry-run", action="store_true",
                       help="Report what would be done without connecting")
    args = parser.parse_args()

    if args.dry_run:
        print("=== DRY RUN ===")
        print(f"Would tune: {args.loop}")
        print(f"Port: {args.port} @ {args.baud}")
        return

    tuner = PIDAutoTuner(port=args.port, baud=args.baud)

    try:
        if not tuner.setup():
            print("[FATAL] Pre-flight failed. Aborting.")
            tuner.shutdown()
            sys.exit(1)

        if args.loop in ("current", "all"):
            result = tuner.tune_current_loop(
                R=args.R, L=args.L, bw_hz=args.bw
            )
            if not result.success:
                print(f"[WARN] Current loop tuning incomplete: {result.notes}")

        if args.loop in ("speed", "all"):
            kp_init = args.kp if args.kp is not None else 0.05
            ki_init = args.ki if args.ki is not None else 0.0
            step = args.step if args.step is not None else 1.0
            result = tuner.tune_speed_loop(
                kp_init=kp_init, ki_init=ki_init, step_rad_s=step
            )
            if not result.success:
                print(f"[WARN] Speed loop tuning incomplete: {result.notes}")

        if args.loop in ("position", "all"):
            kp_init = args.kp if args.kp is not None else 2.0
            kd_init = args.kd if args.kd is not None else 0.05
            step = args.step if args.step is not None else 0.5
            result = tuner.tune_position_loop(
                kp_init=kp_init, kd_init=kd_init, step_rad=step
            )
            if not result.success:
                print(f"[WARN] Position loop tuning incomplete: {result.notes}")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] User aborted.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        tuner.shutdown()

    # Save report
    if tuner.results:
        tuner.save_report()


if __name__ == "__main__":
    main()
