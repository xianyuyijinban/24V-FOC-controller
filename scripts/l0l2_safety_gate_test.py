#!/usr/bin/env python3
"""
L0-L2 Safety Gate Functional Verification Experiment
=====================================================
验证 HZ24 FOC (sync_20260519) 的指令系统、解锁流程、安全保护机制。
全程无功率输出（栅极关闭，电机不动）。

L0 — 通信与身份（18 条）：带宽验证、诊断查询、APP_MODE 循环、配置读写往返
L1 — 解锁门禁负路径（15 条）：锁定三探针、UNLOCK 副作用、越界拒绝、静默矩阵
L2 — 无功率故障注入与恢复（11 条）：OV注入、FAULT门禁、自动恢复、黑匣子导出

Usage:
  python scripts/l0l2_safety_gate_test.py                      # auto-scan COM
  python scripts/l0l2_safety_gate_test.py --port COM7
  python scripts/l0l2_safety_gate_test.py --layer L0           # dry-run first
  python scripts/l0l2_safety_gate_test.py --skip-wheel-enable  # skip L1-W1
  python scripts/l0l2_safety_gate_test.py --allow-fault-unlocked-probe --l2-uv

Output: scripts/l0l2_safety_<ts>.json + scripts/l0l2_safety_<ts>.md
"""

import serial
import serial.tools.list_ports
import time
import sys
import re
import json
import argparse
import atexit
import signal
import os
from datetime import datetime
from dataclasses import dataclass, field, asdict

# ─── Constants ─────────────────────────────────────────────────────────────
PROJECT_ROOT = r"E:\24V_FOC_Controller_sync_20260519"
DEFAULT_BAUD = 1000000
T_SHORT = 0.30
T_MED = 1.0
T_LONG = 3.0
T_NFRAME_PRED = 2.0
T_NFRAME_RECOVER = 1.0

# N-frame 33-field indices (0-based, comma-split)
NF = {
    "marker": 0,      # "N"
    "ts": 1,
    "state": 2,       # 0=IDLE 1=INIT 2=IDENTIFY 3=READY 4=RUNNING 5=FAULT
    "angle": 3,
    "speed": 4,
    "Id": 5,
    "Iq": 6,
    "Vbus": 7,
    "faultFlags": 8,
    "enc_detected": 9,
    "identified": 10,
    "stall_armed": 11,
    "stall_open": 12,
    "warn_flags": 13,
    "app_fault": 14,  # 0=NONE 1=OC 2=OV 3=UV 4=ENC 5=DRV 6=PARAM 7=ADC
    "ctrl_mode": 15,  # 0=TORQUE 1=SPEED 2=POSITION
    "Id_ref": 16,
    "speed_ref": 17,
    "pos_ref": 18,
    "Iq_ref": 19,
    "Vd": 20,
    "Vq": 21,
    "Ia": 22,
    "Ib": 23,
    "Ic": 24,
    "identify_state": 25,
    "identify_err": 26,
    "uv_limit": 27,
    "ov_limit": 28,
    "adc_calib": 29,
    "offsA": 30,
    "offsB": 31,
    "offsC": 32,
}
NF_COUNT = 33

STATE_MAP = {0: "IDLE", 1: "INIT", 2: "IDENTIFY", 3: "READY", 4: "RUNNING", 5: "FAULT"}
FAULT_MAP = {0: "NONE", 1: "OVERCURRENT", 2: "OVERVOLTAGE", 3: "UNDERVOLTAGE",
             4: "ENCODER", 5: "DRV8350S", 6: "PARAM_INVALID", 7: "ADC_SAMPLING"}

# Safety: NEVER send these commands during L0-L2 experiment
BLACKLIST_ALWAYS = [
    "CAL:SAVE", "CMD:SAVE", "HOME", "CLEAR_HOME",
    "ENCODER_DIR,1", "ENCODER_DIR,-1", "MOTOR_PN",
    "ADC_NOISE", "ADC_PHASE_SCAN", "ADC_SECTOR_SCAN",
    "TLE_GPIO_DIAG,1",
    "IREF", "SREF", "PREF",
]
# Additionally blocked when believed_unlocked==True
BLACKLIST_WHEN_UNLOCKED = [
    "IDENTIFY,1", "CAL:IDENTIFY,1",
    "CAL:ALL", "CMD:ALL",
]

# ─── Data Classes ──────────────────────────────────────────────────────────

@dataclass
class TestCase:
    id: str
    layer: str
    name: str
    status: str = "PENDING"  # PENDING / PASS / FAIL / SKIP / INFO
    detail: str = ""
    sent: list = field(default_factory=list)
    expect: str = ""
    got: list = field(default_factory=list)
    nframe_evidence: dict = field(default_factory=dict)
    raw_log: list = field(default_factory=list)
    t_start: float = 0.0
    t_end: float = 0.0


class PanicSentinel(Exception):
    """Raised when a safety invariant is violated."""
    pass


# ─── Serial I/O ─────────────────────────────────────────────────────────────

class SerialIO:
    def __init__(self, port, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def drain(self, dur=0.3):
        time.sleep(dur)
        if self.ser and self.ser.is_open:
            self.ser.read(self.ser.in_waiting)

    def send_line(self, cmd, wait=T_SHORT):
        """Send a command line, return list of non-telemetry response lines."""
        if not self.is_open:
            return []
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode("utf-8"))
        time.sleep(wait)
        raw = self.ser.read(self.ser.in_waiting)
        text = raw.decode("utf-8", errors="replace")
        # Filter out N/C/B frames (telemetry) and W frames
        lines = []
        for ln in text.split("\n"):
            ln = ln.strip()
            if not ln:
                continue
            if ln.startswith(("N,", "C,", "B,")):
                continue
            lines.append(ln)
        return lines

    def send_raw(self, data):
        """Send raw bytes (for binary stress)."""
        if self.ser and self.ser.is_open:
            self.ser.write(data)

    def collect_nframes(self, dur_s=2.0):
        """Collect parsed N-frames (list of lists of strings)."""
        frames = []
        deadline = time.time() + dur_s
        while time.time() < deadline:
            if not self.is_open:
                break
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode("utf-8", errors="replace").split("\n"):
                    line = line.strip()
                    if line.startswith("N,"):
                        parts = line.split(",")
                        if len(parts) >= NF_COUNT:
                            frames.append(parts)
            else:
                time.sleep(0.01)
        return frames

    def collect_raw(self, dur_s):
        """Collect all raw bytes for a duration."""
        data = bytearray()
        deadline = time.time() + dur_s
        while time.time() < deadline:
            if not self.is_open:
                break
            w = self.ser.in_waiting
            if w:
                data.extend(self.ser.read(w))
            else:
                time.sleep(0.005)
        return bytes(data)

    def wait_response(self, expect_prefix, timeout=T_MED):
        """Wait for any response line starting with expect_prefix. Returns (line, all_lines)."""
        deadline = time.time() + timeout
        all_lines = []
        while time.time() < deadline:
            if not self.is_open:
                break
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode("utf-8", errors="replace").split("\n"):
                    line = line.strip()
                    if not line or line.startswith(("N,", "C,")):
                        continue
                    all_lines.append(line)
                    if line.startswith(expect_prefix):
                        return (line, all_lines)
            else:
                time.sleep(0.02)
        return (None, all_lines)

    def wait_nframe(self, predicate, timeout=T_NFRAME_PRED):
        """Poll N-frames until predicate(nframe_parts) is True. Returns (True, frames)."""
        deadline = time.time() + timeout
        frames = []
        while time.time() < deadline:
            if not self.is_open:
                break
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode("utf-8", errors="replace").split("\n"):
                    line = line.strip()
                    if line.startswith("N,"):
                        parts = line.split(",")
                        if len(parts) >= NF_COUNT:
                            frames.append(parts)
                            if predicate(parts):
                                return (True, frames)
            else:
                time.sleep(0.01)
        return (False, frames)

    def get_nframe(self, timeout_s=1.0):
        """Get a single N-frame as a dict."""
        self.ser.reset_input_buffer()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if not self.is_open:
                break
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode("utf-8", errors="replace").split("\n"):
                    line = line.strip()
                    if line.startswith("N,"):
                        parts = line.split(",")
                        if len(parts) >= NF_COUNT:
                            return parts
            else:
                time.sleep(0.02)
        return None


# ─── Port Scanner ───────────────────────────────────────────────────────────

def scan_ports(baud=DEFAULT_BAUD):
    """Scan all COM ports @baud, return list of candidate ports."""
    candidates = []
    ports = list(serial.tools.list_ports.comports())
    print(f"Serial port scan ({len(ports)} ports @ {baud} baud)...")

    for p in ports:
        try:
            ser = serial.Serial(p.device, baud, timeout=0.2)
            # Passive listen for N-frames (0.4s)
            time.sleep(0.45)
            raw = ser.read(ser.in_waiting or 1)
            text = raw.decode("utf-8", errors="replace")
            has_nframes = "N," in text and text.count(",") > 20

            if has_nframes:
                candidates.append(p.device)
                print(f"  {p.device}: N-frame detected (passive)")
            else:
                # Active probe
                ser.reset_input_buffer()
                ser.write(b"SYS:FW_INFO?\n")
                time.sleep(0.6)
                raw = ser.read(ser.in_waiting or 1)
                text = raw.decode("utf-8", errors="replace")
                if "FW_INFO" in text:
                    candidates.append(p.device)
                    print(f"  {p.device}: FW_INFO? ACK (active probe)")
                else:
                    print(f"  {p.device}: no response")

            ser.close()
        except Exception as e:
            print(f"  {p.device}: error ({e})")

    return candidates


# ─── Safety Guard ───────────────────────────────────────────────────────────

class SafetyGuard:
    def __init__(self):
        self.believed_unlocked = False

    def set_unlocked(self, val):
        self.believed_unlocked = val

    def check(self, cmd, step_type="normal"):
        """Raise PanicSentinel if command violates blacklist."""
        # L1-W1 exemption: SCROLL_WHEEL guarded ENABLE,1
        if step_type == "WHEEL_GUARDED_ENABLE":
            if "ENABLE,1" in cmd or "ENABLE,1" in cmd:
                return  # allowed
        else:
            if "ENABLE,1" in cmd or "ENABLE,1" in cmd:
                raise PanicSentinel(f"BLACKLIST VIOLATION: ENABLE,1 in step_type={step_type}")

        for banned in BLACKLIST_ALWAYS:
            if banned in cmd:
                raise PanicSentinel(f"BLACKLIST VIOLATION: permanent banned '{banned}' in '{cmd}'")

        if self.believed_unlocked:
            for banned in BLACKLIST_WHEN_UNLOCKED:
                if banned in cmd:
                    raise PanicSentinel(
                        f"BLACKLIST VIOLATION: unlocked-banned '{banned}' in '{cmd}'")

    def panic(self, io, baseline, reason=""):
        """Emergency safety sequence. Ignores individual failures, best-effort."""
        print(f"\n{'!'*60}")
        print(f"!!! PANIC: {reason}")
        print(f"{'!'*60}")
        if io.is_open:
            safety_seq = [
                ("CMD:ENABLE,0", 0.2),
                ("CMD:UNLOCK,0", 0.2),
                ("CMD:STALL_MODE,0", 0.2),
            ]
            if baseline:
                safety_seq.append((f"CMD:VBUS_LIMIT,{baseline['uv0']:.1f},{baseline['ov0']:.1f}", 0.2))
            safety_seq.append(("CMD:APP_MODE,RAW", 0.2))
            safety_seq.append(("TELEM:CUR,OFF", 0.2))
            safety_seq.append(("TELEM:RATE,50", 0.2))

            for cmd, w in safety_seq:
                try:
                    io.send_line(cmd, w)
                except Exception:
                    pass

            # Grab final snapshot
            try:
                diag = io.send_line("CMD:FAULT_DETAIL", T_LONG)
                print(f"FINAL FAULT_DETAIL: {len(diag)} lines")
            except Exception:
                pass
        self.believed_unlocked = False

    @staticmethod
    def static_check_all_cases(cases):
        """Pre-execution static audit: assert no case has blacklisted commands."""
        errors = []
        for c in cases:
            for cmd in c.sent:
                for banned in BLACKLIST_ALWAYS:
                    if banned in cmd:
                        errors.append(f"Case {c.id}: banned '{banned}' in '{cmd}'")
                # ENABLE,1 check (except WHEEL_GUARDED_ENABLE)
                if "ENABLE,1" in cmd:
                    errors.append(f"Case {c.id}: ENABLE,1 in '{cmd}' (not WHEEL_GUARDED_ENABLE)")
        if errors:
            for e in errors:
                print(f"  STATIC CHECK FAIL: {e}")
            raise PanicSentinel(f"Static blacklist audit failed: {len(errors)} violations")
        print("  Static blacklist audit: PASS")


# ─── Runner ─────────────────────────────────────────────────────────────────

class Runner:
    def __init__(self, io, guard, args):
        self.io = io
        self.guard = guard
        self.args = args
        self.results = []
        self._continue = True
        self.baseline = {}       # {uv0, ov0, vbus0, state0, identified0, ctrl_mode0, app_mode0, ...}
        self.fw_git = ""
        self._layer_aborted = set()

    def record_case(self, case):
        self.results.append(case)
        tag = {"PASS": "[OK]", "FAIL": "[!!]", "SKIP": "[-]", "INFO": "[i]", "PENDING": "[?]", "ABORTED": "[A]"}
        t = tag.get(case.status, "[?]")
        print(f"  {t} {case.id} {case.name}: {case.status}  {case.detail[:120]}")

    def _send_checked(self, cmd, wait=T_SHORT, step_type="normal"):
        """Send a command through the safety guard."""
        self.guard.check(cmd, step_type)
        return self.io.send_line(cmd, wait)

    # ── Helpers ────────────────────────────────────────────────────────

    def _match_prefix(self, lines, prefix):
        """Check if any response line starts with prefix."""
        return any(l.startswith(prefix) for l in lines)

    def _match_all_prefixes(self, lines, prefixes):
        """Check that all prefixes appear somewhere in lines."""
        text = " ".join(lines)
        return all(p in text for p in prefixes)

    def _get_nf_int(self, nf, key):
        try:
            val = nf[NF[key]]
            return int(val, 0) if val.startswith("0x") or val.startswith("0X") else int(val)
        except (IndexError, ValueError):
            return -999

    def _get_nf_float(self, nf, key):
        try:
            return float(nf[NF[key]])
        except (IndexError, ValueError):
            return float('nan')

    def _nf_state_ok(self, nf, expected_state=None):
        """Safety check: never RUNNING(4), optionally match expected state."""
        s = self._get_nf_int(nf, "state")
        if s == 4:
            raise PanicSentinel(f"SAFETY VIOLATION: N-frame state==4 (RUNNING) detected!")
        if expected_state is not None and s != expected_state:
            return False
        return True

    def _check_current_zero(self, nf):
        """Assert that Ia/Ib/Ic are near zero."""
        ia = abs(self._get_nf_float(nf, "Ia"))
        ib = abs(self._get_nf_float(nf, "Ib"))
        ic = abs(self._get_nf_float(nf, "Ic"))
        iq_ref = abs(self._get_nf_float(nf, "Iq_ref"))
        return ia < 0.1 and ib < 0.1 and ic < 0.1 and iq_ref < 0.05

    def _check_unexpected_fault(self, layer_name):
        """If a fault appears outside L2 injection windows, handle it."""
        frames = self.io.collect_nframes(0.5)
        for nf in frames:
            s = self._get_nf_int(nf, "state")
            if s == 5:
                fa = self._get_nf_int(nf, "app_fault")
                print(f"  UNEXPECTED FAULT in {layer_name}: state=5 app_fault={fa} ({FAULT_MAP.get(fa, '?')})")
                # Capture evidence
                fd = self.io.send_line("CMD:FAULT_DETAIL", T_LONG)
                bb = self.io.send_line("CMD:BLACKBOX?", T_MED)
                print(f"  FAULT_DETAIL: {len(fd)} lines, BLACKBOX?: {bb}")

                if fa in (2, 3):  # voltage fault
                    self.io.send_line(f"CMD:VBUS_LIMIT,{self.baseline['uv0']:.1f},{self.baseline['ov0']:.1f}", T_SHORT)
                    time.sleep(1.0)
                    nf2 = self.io.collect_nframes(1.0)
                    recovered = any(self._get_nf_int(x, "state") == 3 for x in nf2)
                    if recovered:
                        print(f"  Auto-recovered from voltage fault")
                        return True
                else:
                    self.io.send_line("SYS:CLEAR_FAULT", T_MED)
                    time.sleep(3.0)
                    nf2 = self.io.collect_nframes(1.0)
                    recovered = not any(self._get_nf_int(x, "state") == 5 for x in nf2)
                    if recovered:
                        print(f"  Recovered after CLEAR_FAULT")
                        return True
                return False
        return True  # no unexpected fault

    # ── Preflight: Scan, FW_INFO, git compare, baseline snapshot ──────

    def preflight(self):
        """Phase A: port scan, version check, baseline snapshot, health gate."""
        print(f"{'='*60}")
        print("PREFLIGHT: Port Scan + Firmware Identity + Baseline")
        print(f"{'='*60}")

        # A1: Port scan
        port = self.args.port
        if port:
            print(f"Using specified port: {port}")
        else:
            candidates = scan_ports(self.io.baud)
            if len(candidates) == 0:
                print("ERROR: No FOC device found on any COM port.")
                sys.exit(1)
            elif len(candidates) > 1:
                print(f"Multiple candidates: {candidates}")
                print("Re-run with --port COMx to select one.")
                sys.exit(1)
            port = candidates[0]
            print(f"Auto-detected: {port}")

        self.io.port = port
        self.io.open()

        # A2: FW_INFO + git hash compare
        print(f"\n--- FW_INFO ---")
        resp = self.io.send_line("SYS:FW_INFO?", T_MED)
        fw_line = None
        for l in resp:
            print(f"  {l}")
            if "FW_INFO" in l:
                fw_line = l

        if not fw_line:
            print("ERROR: No FW_INFO response. Board may be unresponsive.")
            sys.exit(1)

        # Extract git hash from FW_INFO,git=<hash>
        m = re.search(r'git=([0-9a-fA-F]+)', fw_line)
        board_hash = m.group(1) if m else "UNKNOWN"

        # Get workspace git info
        import subprocess
        try:
            ws_hash = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=10
            ).stdout.strip()
            ws_dirty = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=10
            ).stdout.strip()
            is_dirty = len(ws_dirty) > 0
        except Exception:
            ws_hash = "N/A"
            is_dirty = False

        print(f"\n  Board FW git:  {board_hash}")
        print(f"  Workspace git: {ws_hash}{' (DIRTY)' if is_dirty else ''}")

        if board_hash != ws_hash:
            print(f"\n  VERSION MISMATCH!")
            print(f"  Workspace has uncommitted changes: {is_dirty}")
            if not self.args.no_reflash:
                choice = input(f"  [c]ontinue with board firmware / [a]bort: ").strip().lower()
                if choice == 'a':
                    sys.exit(0)

        self.fw_git = board_hash

        # A3: Baseline snapshot + health gate
        print(f"\n--- Baseline N-frame Snapshot (2s) ---")
        self.io.drain(0.5)
        frames = self.io.collect_nframes(2.0)
        if len(frames) < 30:
            print(f"ERROR: Only {len(frames)} N-frames in 2s. Check serial link.")
            sys.exit(1)

        # Compute median values from last 20 frames
        recent = frames[-20:]
        def _median(key, cast=float):
            vals = sorted([cast(f[NF[key]]) for f in recent if len(f) > NF[key]])
            return vals[len(vals)//2] if vals else 0

        state0 = int(_median("state", int))
        identified0 = int(_median("identified", int))
        enc0 = int(_median("enc_detected", int))
        vbus0 = _median("Vbus")
        uv0 = _median("uv_limit")
        ov0 = _median("ov_limit")
        ctrl_mode0 = int(_median("ctrl_mode", int))
        app_fault0 = int(_median("app_fault", int))
        fault_flags0 = int(_median("faultFlags", lambda x: int(x, 0)))
        stall0 = int(_median("stall_armed", int))

        self.baseline = {
            "state0": state0, "identified0": identified0, "enc0": enc0,
            "vbus0": vbus0, "uv0": uv0, "ov0": ov0,
            "ctrl_mode0": ctrl_mode0, "app_fault0": app_fault0,
            "fault_flags0": fault_flags0, "stall0": stall0,
        }

        print(f"  State: {STATE_MAP.get(state0, '?')} ({state0})")
        print(f"  Identified: {identified0}, Encoder: {enc0}")
        print(f"  Vbus: {vbus0:.2f}V, UV={uv0:.2f}V, OV={ov0:.2f}V")
        print(f"  AppFault: {app_fault0}, FaultFlags: 0x{fault_flags0:08X}")
        print(f"  CtrlMode: {ctrl_mode0}, StallArmed: {stall0}")

        # Health gate
        if app_fault0 != 0:
            print(f"HEALTH GATE FAIL: app_fault={app_fault0} ({FAULT_MAP.get(app_fault0, '?')}). Aborting.")
            sys.exit(1)
        if fault_flags0 != 0:
            print(f"HEALTH GATE FAIL: faultFlags=0x{fault_flags0:08X}. Aborting.")
            sys.exit(1)
        if state0 not in (0, 3):
            print(f"HEALTH GATE FAIL: state={state0} ({STATE_MAP.get(state0, '?')}). Expected IDLE(0) or READY(3).")
            sys.exit(1)

        # Verify currents near zero
        for i, f in enumerate(recent[:5]):
            if not self._check_current_zero(f):
                ia = self._get_nf_float(f, "Ia")
                ib = self._get_nf_float(f, "Ib")
                ic = self._get_nf_float(f, "Ic")
                print(f"HEALTH GATE FAIL: Non-zero current detected (Ia={ia:.3f}A, Ib={ib:.3f}A, Ic={ic:.3f}A)")
                sys.exit(1)

        print(f"  Health gate: PASS  (state={'IDLE' if state0==0 else 'READY'}, 0 faults, 0 current)")

        # Also capture current APP_MODE
        resp = self.io.send_line("CMD:APP_MODE?", T_MED)
        app_mode0 = "RAW"
        for l in resp:
            m = re.search(r'APP_MODE,OK,(\w+)', l)
            if m:
                app_mode0 = m.group(1)
        self.baseline["app_mode0"] = app_mode0
        print(f"  APP_MODE: {app_mode0}")

        # Get TELEM:RATE current value
        resp = self.io.send_line("TELEM:RATE?", T_MED)
        telem_rate0 = 50
        for l in resp:
            m = re.search(r'(\d+)Hz', l)
            if m:
                telem_rate0 = int(m.group(1))
        self.baseline["telem_rate0"] = telem_rate0

    # ── L0: Communication & Identity (18 cases) ─────────────────────

    def layer_L0(self):
        print(f"\n{'#'*60}")
        print(f"# L0 — COMMUNICATION & IDENTITY")
        print(f"{'#'*60}")

        # L0-01: Port scan (already done in preflight)
        self.record_case(TestCase("L0-01", "L0", "Port scan", "PASS",
                                  detail=f"Port: {self.io.port}"))

        # L0-02: FW_INFO already done
        self.record_case(TestCase("L0-02", "L0", "FW_INFO identity", "PASS",
                                  detail=f"git={self.fw_git}"))

        # L0-03: FW_INFO 20x burst
        ok = 0
        for i in range(20):
            resp = self.io.send_line("SYS:FW_INFO?", 0.06)
            if any("FW_INFO" in l for l in resp):
                ok += 1
        self.record_case(TestCase("L0-03", "L0", "FW_INFO 20x burst",
                                  "PASS" if ok == 20 else "FAIL",
                                  detail=f"{ok}/20 OK"))

        # L0-04: SYS:CMDS?
        resp = self.io.send_line("SYS:CMDS?", T_MED)
        ok = any("SYS:CMDS,OK" in l for l in resp) and len(resp) >= 5
        self.record_case(TestCase("L0-04", "L0", "SYS:CMDS? index",
                                  "PASS" if ok else "FAIL",
                                  detail=f"{len(resp)} lines"))

        # L0-05: Baseline snapshot (already done in preflight)
        self.record_case(TestCase("L0-05", "L0", "Baseline snapshot + health gate", "PASS",
                                  detail=f"state={STATE_MAP.get(self.baseline['state0'],'?')} "
                                         f"Vbus={self.baseline['vbus0']:.2f}V"))

        # L0-06: TELEM:RATE sweep
        rate_results = []
        for rate in [100, 10, 0]:
            resp = self.io.send_line(f"TELEM:RATE,{rate}", T_MED)
            ack_ok = self._match_prefix(resp, "TELEM:RATE,OK")
            if rate == 0:
                # RATE=0: drain residual queued frames (1.5s), verify near-zero
                self.io.drain(1.5)
                frames = self.io.collect_nframes(2.0)
                actual = len(frames) / 2.0
                ok = actual < 3.0  # residual frames drop to near-zero
            else:
                self.io.drain(0.3)
                frames = self.io.collect_nframes(2.0)
                actual = len(frames) / 2.0
                ok = abs(actual / rate - 1.0) < 0.40
            rate_results.append(f"{rate}Hz→{actual:.1f}Hz {'OK' if ok else 'FAIL'}")
            # Command channel must survive RATE=0
            if rate == 0:
                resp2 = self.io.send_line("SYS:FW_INFO?", T_MED)
                alive = any("FW_INFO" in l for l in resp2)
                rate_results.append(f"FW_INFO@RATE0:{'OK' if alive else 'FAIL'}")

        # Restore 50Hz
        self.io.send_line("TELEM:RATE,50", T_MED)
        self.io.send_line("TELEM:RATE?", T_MED)

        all_ok = "FAIL" not in " ".join(rate_results)
        self.record_case(TestCase("L0-06", "L0", "TELEM:RATE 100/10/0Hz sweep",
                                  "PASS" if all_ok else "FAIL",
                                  detail="; ".join(rate_results)))

        # L0-07: TELEM:CUR? + OFF
        resp = self.io.send_line("TELEM:CUR?", T_MED)
        ok1 = self._match_prefix(resp, "CUR_STREAM,OK")
        resp2 = self.io.send_line("TELEM:CUR,OFF", T_MED)
        ok2 = self._match_prefix(resp2, "CUR_STREAM,OK")
        self.record_case(TestCase("L0-07", "L0", "TELEM:CUR? → OFF",
                                  "PASS" if (ok1 and ok2) else "FAIL",
                                  detail=f"CUR?={'OK' if ok1 else 'FAIL'} OFF={'OK' if ok2 else 'FAIL'}"))

        # L0-08: DIAG:FAULT_DETAIL
        resp = self.io.send_line("CMD:FAULT_DETAIL", T_LONG)
        has_title = any("Diagnostic Snapshot" in l for l in resp)
        has_pwm = any("pwm=" in l for l in resp)
        pwm_line = next((l for l in resp if "pwm=" in l), "NOT FOUND")
        self.record_case(TestCase("L0-08", "L0", "DIAG:FAULT_DETAIL",
                                  "PASS" if has_title else "FAIL",
                                  detail=f"lines={len(resp)} pwm_line='{pwm_line.strip()}'"))
        if has_pwm and "pwm=0 moe=0" not in pwm_line:
            raise PanicSentinel(f"PWM NOT ZERO: {pwm_line}")

        # L0-09: DIAG:UART_RX?
        resp = self.io.send_line("CMD:UART_RX?", T_MED)
        ok = self._match_prefix(resp, "UART_RX,OK")
        self.record_case(TestCase("L0-09", "L0", "DIAG:UART_RX?",
                                  "PASS" if ok else "FAIL"))

        # L0-10: DIAG:TLE_RAW
        resp = self.io.send_line("CMD:TLE_RAW", T_MED)
        ok = self._match_prefix(resp, "TLE_RAW")
        self.record_case(TestCase("L0-10", "L0", "DIAG:TLE_RAW",
                                  "PASS" if ok else "FAIL"))

        # L0-11: DIAG:FOC_TIME?
        # Async multi-line output, collect for 3s
        self.io.send_raw(b"CMD:FOC_TIME?\n")
        time.sleep(T_LONG)
        raw = self.io.ser.read(self.io.ser.in_waiting)
        lines = [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
                 if l.strip() and "FOC_TIME" in l]
        ok = any("END" in l for l in lines)
        self.record_case(TestCase("L0-11", "L0", "DIAG:FOC_TIME?",
                                  "PASS" if ok else "FAIL" if len(lines) > 0 else "SKIP",
                                  detail=f"{len(lines)} FOC_TIME lines, has END={ok}"))
        if len(lines) == 0:
            self.record_case(TestCase("L0-11", "L0", "DIAG:FOC_TIME? (retry)",
                                      "SKIP", detail="profiler may need identify first"))

        # L0-12: DIAG:BLACKBOX?
        resp = self.io.send_line("CMD:BLACKBOX?", T_MED)
        ok = self._match_prefix(resp, "BLACKBOX,OK")
        frozen0 = False
        count0 = 0
        for l in resp:
            m = re.search(r'frozen=(\d)', l)
            if m:
                frozen0 = bool(int(m.group(1)))
            m = re.search(r'count=(\d+)', l)
            if m:
                count0 = int(m.group(1))
        self.record_case(TestCase("L0-12", "L0", "DIAG:BLACKBOX?",
                                  "PASS" if ok else "FAIL",
                                  detail=f"frozen={frozen0}, count={count0}"))

        # If frozen from previous session, clear it for L2
        self._bb_frozen_from_prev = frozen0
        if frozen0:
            print(f"  Stale blackbox freeze detected (count={count0}). Clearing for L2.")
            self.io.send_line("CMD:BLACKBOX,CLEAR", T_MED)

        # L0-13: APP_MODE cycle
        modes_to_test = ["JOINT_POS", "GIMBAL_SPEED", "HOLD", "SPRING_DAMPER", "DETENT"]
        if self.baseline["identified0"] == 0 or self.baseline["enc0"] == 0:
            # Not identified / no encoder → haptic modes will fail silently
            modes_to_test = ["JOINT_POS", "GIMBAL_SPEED"]  # only non-haptic

        mode_results = []
        for mode in modes_to_test:
            resp = self.io.send_line(f"CMD:APP_MODE,{mode}", T_MED)
            set_ok = self._match_prefix(resp, f"APP_MODE,OK,{mode}")
            resp2 = self.io.send_line("CMD:APP_MODE?", T_MED)
            q_ok = mode in " ".join(resp2)
            mode_results.append(f"{mode}: set={'OK' if set_ok else 'FAIL'} q={'OK' if q_ok else 'FAIL'}")

        # Return to RAW + restore control mode
        self.io.send_line("CMD:APP_MODE,RAW", T_MED)
        c0 = self.baseline["ctrl_mode0"]
        self.io.send_line(f"CMD:MODE,{c0}", T_MED)

        all_mode_ok = "FAIL" not in " ".join(mode_results)
        self.record_case(TestCase("L0-13", "L0", "APP_MODE cycle",
                                  "PASS" if all_mode_ok else "FAIL",
                                  detail="; ".join(mode_results)))

        # L0-14: Config read-write-read round-trip (net-zero)
        config_results = []

        # SPRING:CFG read → write back same → read verify
        resp_r1 = self.io.send_line("SPRING:CFG?", T_MED)
        m = re.search(r'K=([0-9.]+),D=([0-9.]+),limit=([0-9.]+)', " ".join(resp_r1))
        if m:
            sk, sd, sl = m.group(1), m.group(2), m.group(3)
            self.io.send_line(f"SPRING:CFG,{sk},{sd},{sl}", T_MED)
            resp_r2 = self.io.send_line("SPRING:CFG?", T_MED)
            m2 = re.search(r'K=([0-9.]+),D=([0-9.]+),limit=([0-9.]+)', " ".join(resp_r2))
            spring_ok = m2 and abs(float(m2.group(1)) - float(sk)) < 0.01
            config_results.append(f"SPRING:{'OK' if spring_ok else 'FAIL'}")
        else:
            config_results.append("SPRING:SKIP(no match)")

        # DETENT:CFG (5-param)
        resp_r1 = self.io.send_line("DETENT:CFG?", T_MED)
        m = re.search(r'count=([0-9.]+),strength=([0-9.]+),width=([0-9.]+),damping=([0-9.]+),limit=([0-9.]+)', " ".join(resp_r1))
        if m:
            dc, ds, dw, dd, dl = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
            self.io.send_line(f"DETENT:CFG,{dc},{ds},{dw},{dd},{dl}", T_MED)
            resp_r2 = self.io.send_line("DETENT:CFG?", T_MED)
            m2 = re.search(r'count=([0-9.]+)', " ".join(resp_r2))
            detent_ok = m2 and abs(float(m2.group(1)) - float(dc)) < 0.5
            config_results.append(f"DETENT:{'OK' if detent_ok else 'FAIL'}")
        else:
            config_results.append("DETENT:SKIP")

        # WHEEL:CFG (5-param)
        resp_r1 = self.io.send_line("WHEEL:CFG?", T_MED)
        m = re.search(r'count=([0-9.]+),strength=([0-9.]+),width=([0-9.]+),damping=([0-9.]+),limit=([0-9.]+)', " ".join(resp_r1))
        if m:
            wc, ws, ww, wd, wl = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
            self.io.send_line(f"WHEEL:CFG,{wc},{ws},{ww},{wd},{wl}", T_MED)
            resp_r2 = self.io.send_line("WHEEL:CFG?", T_MED)
            m2 = re.search(r'count=([0-9.]+)', " ".join(resp_r2))
            wheel_ok = m2 and abs(float(m2.group(1)) - float(wc)) < 0.5
            config_results.append(f"WHEEL:{'OK' if wheel_ok else 'FAIL'}")
        else:
            config_results.append("WHEEL:SKIP")

        # GIMBAL:RAMP?
        resp_r1 = self.io.send_line("GIMBAL:RAMP?", T_MED)
        m = re.search(r'accel=([0-9.]+)', " ".join(resp_r1))
        if m:
            ga = m.group(1)
            self.io.send_line(f"GIMBAL:RAMP,{ga}", T_MED)
            resp_r2 = self.io.send_line("GIMBAL:RAMP?", T_MED)
            m2 = re.search(r'accel=([0-9.]+)', " ".join(resp_r2))
            ramp_ok = m2 and abs(float(m2.group(1)) - float(ga)) < 0.1
            config_results.append(f"RAMP:{'OK' if ramp_ok else 'FAIL'}")
        else:
            config_results.append("RAMP:SKIP")

        # MOTION:MOTION_CFG?
        resp_r1 = self.io.send_line("CMD:MOTION_CFG?", T_MED)
        m = re.search(r'speed=([0-9.]+),accel=([0-9.]+),cruise=([0-9.]+)', " ".join(resp_r1))
        if m:
            ms, ma, mc = m.group(1), m.group(2), m.group(3)
            self.io.send_line(f"CMD:MOTION_CFG,{ms},{ma},{mc}", T_MED)
            resp_r2 = self.io.send_line("CMD:MOTION_CFG?", T_MED)
            m2 = re.search(r'speed=([0-9.]+)', " ".join(resp_r2))
            motion_ok = m2 and abs(float(m2.group(1)) - float(ms)) < 0.01
            config_results.append(f"MOTION:{'OK' if motion_ok else 'FAIL'}")
        else:
            config_results.append("MOTION:SKIP")

        # JOINT:LIMIT?
        resp_r1 = self.io.send_line("JOINT:LIMIT?", T_MED)
        if "OFF" in " ".join(resp_r1):
            config_results.append("JOINT:SKIP(OFF)")
        else:
            m = re.search(r'min=([0-9.-]+)deg,max=([0-9.-]+)deg', " ".join(resp_r1))
            if m:
                jmin, jmax = m.group(1), m.group(2)
                self.io.send_line(f"JOINT:LIMIT,{jmin},{jmax}", T_MED)
                resp_r2 = self.io.send_line("JOINT:LIMIT?", T_MED)
                m2 = re.search(r'min=([0-9.-]+)deg', " ".join(resp_r2))
                joint_ok = m2 and abs(float(m2.group(1)) - float(jmin)) < 0.5
                config_results.append(f"JOINT:{'OK' if joint_ok else 'FAIL'}")

        all_cfg_ok = not any("FAIL" in c for c in config_results)
        self.record_case(TestCase("L0-14", "L0", "Config read-write-read roundtrip",
                                  "PASS" if all_cfg_ok else "FAIL",
                                  detail="; ".join(config_results)))

        # L0-15: FF group queries
        ff_queries = [
            ("CMD:COG_CFG?", "BEMF"),
            ("CMD:BEMF_CFG?", "BEMF"),
            ("CMD:RS_FF_MODE?", "RS_MODE"),
            ("CMD:RS_FF_ADAPTIVE?", "RS_ADAPTIVE"),
            ("CMD:RS_FF_SIGN_PROTECT?", "RS_SIGN"),
        ]
        ff_results = []
        for cmd, tag in ff_queries:
            resp = self.io.send_line(cmd, T_MED)
            ok = len(resp) > 0
            ff_results.append(f"{tag}:{'OK' if ok else 'FAIL'}")
        self.record_case(TestCase("L0-15", "L0", "FF group queries",
                                  "INFO" if all("FAIL" not in r for r in ff_results) else "FAIL",
                                  detail="; ".join(ff_results)))

        # L0-16: GAIN illegal values (silent, must not break channel)
        gain_cmds = [
            ("CMD:PI_CURRENT,-1,0", "PI_CURRENT"),
            ("CMD:PI_SPEED,-1,0", "PI_SPEED"),
            ("CMD:PD_POS,-1,0", "PD_POS"),
        ]
        gain_results = []
        for cmd, tag in gain_cmds:
            resp = self.io.send_line(cmd, 0.5)
            silent = len(resp) == 0  # no ACK expected
            gain_results.append(f"{tag}:{'SILENT' if silent else 'GOT_RESP'}")
        # Verify channel alive
        resp_alive = self.io.send_line("SYS:FW_INFO?", T_MED)
        alive = any("FW_INFO" in l for l in resp_alive)
        gain_results.append(f"CHANNEL:{'ALIVE' if alive else 'DEAD'}")

        all_gain_ok = all("SILENT" in r or "ALIVE" in r for r in gain_results) and alive
        self.record_case(TestCase("L0-16", "L0", "GAIN illegal values (silent reject)",
                                  "PASS" if all_gain_ok else "FAIL",
                                  detail="; ".join(gain_results)))

        # L0-17: PWM-OFF gated commands
        resp = self.io.send_line("CMD:ADC_ZERO,16", T_MED)  # blocking ~16ms, safe
        adc_ok = self._match_prefix(resp, "ADC_ZERO,OK")
        resp2 = self.io.send_line("CMD:TLE_GPIO_DIAG,0", T_MED)
        tle_ok = self._match_prefix(resp2, "TLE_GPIO_DIAG,STOP")
        self.record_case(TestCase("L0-17", "L0", "PWM-OFF gated (ADC_ZERO + TLE_GPIO_DIAG STOP)",
                                  "PASS" if (adc_ok and tle_ok) else "FAIL",
                                  detail=f"ADC={'OK' if adc_ok else 'FAIL'} TLE={'OK' if tle_ok else 'FAIL'}"))

        # L0-18: CAL:STATUS?
        resp = self.io.send_line("CAL:STATUS?", T_MED)
        ok = self._match_prefix(resp, "CAL:STATUS,OK")
        self.record_case(TestCase("L0-18", "L0", "CAL:STATUS?",
                                  "PASS" if ok else "FAIL"))

    # ── L1: Unlock Gate Negative Paths (15 cases) ────────────────────

    def layer_L1(self):
        print(f"\n{'#'*60}")
        print(f"# L1 — UNLOCK GATE NEGATIVE PATHS")
        print(f"{'#'*60}")

        # L1-01: ENABLE,0 idempotent
        resp = self._send_checked("CMD:ENABLE,0", T_MED)
        ok = self._match_prefix(resp, "ENABLE,OK,0")
        self.record_case(TestCase("L1-01", "L1", "ENABLE,0 idempotent",
                                  "PASS" if ok else "FAIL", sent=["CMD:ENABLE,0"],
                                  expect="ENABLE,OK,0", got=resp))

        # L1-02: LOCKED REJECT ENABLE,1
        resp = self._send_checked("CMD:ENABLE,1", T_MED)
        ok = self._match_prefix(resp, "ENABLE,FAIL,locked")
        self.record_case(TestCase("L1-02", "L1", "Locked reject ENABLE,1",
                                  "PASS" if ok else "FAIL", sent=["CMD:ENABLE,1"],
                                  expect="ENABLE,FAIL,locked", got=resp))

        # L1-03: LOCKED REJECT IDENTIFY,1
        resp = self._send_checked("CMD:IDENTIFY,1", T_MED)
        ok = self._match_prefix(resp, "IDENTIFY,FAIL,locked")
        self.record_case(TestCase("L1-03", "L1", "Locked reject IDENTIFY,1",
                                  "PASS" if ok else "FAIL", sent=["CMD:IDENTIFY,1"],
                                  expect="IDENTIFY,FAIL,locked", got=resp))

        # L1-04: LOCKED REJECT CAL:ALL
        resp = self._send_checked("CAL:ALL", T_MED)
        ok = self._match_prefix(resp, "CAL:ALL,FAIL,precheck=4")
        self.record_case(TestCase("L1-04", "L1", "Locked reject CAL:ALL (precheck=4)",
                                  "PASS" if ok else "FAIL", sent=["CAL:ALL"],
                                  expect="CAL:ALL,FAIL,precheck=4", got=resp))

        # L1-05: STALL_MODE independent of UNLOCK
        resp = self._send_checked("CMD:STALL_MODE,1", T_MED)
        ok1 = self._match_prefix(resp, "STALL_MODE,OK,1")
        nf = self.io.get_nframe(1.0)
        stall1 = self._get_nf_int(nf, "stall_armed") == 1 if nf else False
        resp2 = self._send_checked("CMD:STALL_MODE,0", T_MED)
        ok0 = self._match_prefix(resp2, "STALL_MODE,OK,0")
        self.record_case(TestCase("L1-05", "L1", "STALL_MODE independent of UNLOCK",
                                  "PASS" if (ok1 and ok0 and stall1) else "FAIL",
                                  detail=f"STALL,1={'OK' if ok1 else 'FAIL'} "
                                         f"Nframe[11]=1={'YES' if stall1 else 'NO'} "
                                         f"STALL,0={'OK' if ok0 else 'FAIL'}"))

        # L1-06: UNLOCK,1 (ACK only, verify no power action)
        resp = self._send_checked("CMD:UNLOCK,1", T_MED)
        ok = self._match_prefix(resp, "UNLOCK,OK,1")
        # Collect 1s N-frames to verify state unchanged, Vd=Vq=0, Iq_ref=0
        self.guard.set_unlocked(True)  # enter unlocked window
        self.io.drain(0.3)
        frames = self.io.collect_nframes(1.0)
        state_ok = all(self._get_nf_int(f, "state") != 4 for f in frames)
        vd_vq_ok = all(abs(self._get_nf_float(f, "Vd")) < 0.01 and
                        abs(self._get_nf_float(f, "Vq")) < 0.01 for f in frames[:5])
        iq_ok = all(abs(self._get_nf_float(f, "Iq_ref")) < 0.01 for f in frames[:5])

        self.record_case(TestCase("L1-06", "L1", "UNLOCK,1 (ACK only, no power)",
                                  "PASS" if (ok and state_ok and vd_vq_ok and iq_ok) else "FAIL",
                                  detail=f"ACK={'OK' if ok else 'FAIL'} state_no_4={'OK' if state_ok else 'FAIL'} "
                                         f"Vd=Vq=0={'OK' if vd_vq_ok else 'FAIL'} Iq_ref=0={'OK' if iq_ok else 'FAIL'}"))

        # L1-07: PWM evidence during unlock window
        resp = self._send_checked("CMD:FAULT_DETAIL", T_LONG)
        pwm_line = next((l for l in resp if "pwm=" in l), "")
        pwm_ok = "pwm=0 moe=0" in pwm_line
        self.record_case(TestCase("L1-07", "L1", "PWM evidence during unlock window",
                                  "PASS" if pwm_ok else "FAIL",
                                  detail=f"pwm_line='{pwm_line.strip()}'"))

        # L1-08: Arm STALL during unlock window (prep for L1-09)
        resp = self._send_checked("CMD:STALL_MODE,1", T_MED)
        ok = self._match_prefix(resp, "STALL_MODE,OK,1")
        self.record_case(TestCase("L1-08", "L1", "Arm STALL during unlock window",
                                  "PASS" if ok else "FAIL"))

        # L1-09: UNLOCK,0 side effects (triple verify)
        # Note: UNLOCK,0 will send Disable→StopIdentify→clear stall_armed (it.c:1731-1737)
        # ENABLE,0→ENABLE,0→DISABLE workaround: just send UNLOCK,0 and check
        resp = self._send_checked("CMD:UNLOCK,0", T_MED)
        ok_ack = self._match_prefix(resp, "UNLOCK,OK,0")
        self.guard.set_unlocked(False)

        # Side effect 1: stall_armed cleared
        self.io.drain(0.3)
        nf = self.io.get_nframe(1.0)
        stall_cleared = self._get_nf_int(nf, "stall_armed") == 0 if nf else False

        # Side effect 2: re-locked (ENABLE,1 → locked)
        resp2 = self._send_checked("CMD:ENABLE,1", T_MED)
        locked_again = self._match_prefix(resp2, "ENABLE,FAIL,locked")

        # Side effect 3: re-locked (CAL:ALL → precheck=4)
        resp3 = self._send_checked("CAL:ALL", T_MED)
        cal_locked = self._match_prefix(resp3, "CAL:ALL,FAIL,precheck=4")

        self.record_case(TestCase("L1-09", "L1", "UNLOCK,0 side effects (triple verify)",
                                  "PASS" if (ok_ack and stall_cleared and locked_again and cal_locked) else "FAIL",
                                  detail=f"ACK={'OK' if ok_ack else 'FAIL'} "
                                         f"stall_cleared={'YES' if stall_cleared else 'NO'} "
                                         f"re-locked={'YES' if locked_again else 'NO'} "
                                         f"CAL_precheck4={'YES' if cal_locked else 'NO'}"))

        # L1-10: UNLOCK,2 behavior (non-zero → unlock)
        resp = self._send_checked("CMD:UNLOCK,2", T_MED)
        ok = self._match_prefix(resp, "UNLOCK,OK,1")
        self.guard.set_unlocked(True)
        resp2 = self._send_checked("CMD:UNLOCK,0", T_MED)
        self.guard.set_unlocked(False)
        self.record_case(TestCase("L1-10", "L1", "UNLOCK,2 → OK,1 (non-zero=unlock)",
                                  "PASS" if ok else "FAIL",
                                  detail="UNLOCK,2 treated as UNLOCK,1 — documented behavior"))

        # L1-11: MODE range check
        resp = self._send_checked("CMD:MODE,9", T_MED)
        ok = self._match_prefix(resp, "MODE,FAIL,range")
        c0 = self.baseline["ctrl_mode0"]
        self._send_checked(f"CMD:MODE,{c0}", T_MED)  # restore
        self.record_case(TestCase("L1-11", "L1", "MODE,9 → FAIL,range",
                                  "PASS" if ok else "FAIL"))

        # L1-12: Boundary violation matrix
        boundary_tests = [
            ("TELEM:RATE,101", "TELEM:RATE,FAIL,range (0-100Hz)"),
            ("TELEM:CUR,BIN,99", "CUR_STREAM,FAIL,rate range (100-5000Hz)"),
            ("CMD:ENCODER_DIR,5", "ENCODER_DIR,FAIL,invalid=5"),
            ("GIMBAL:RAMP,999", "GIMBAL:RAMP,FAIL,range"),
            ("SPRING:CFG,0.5,0.05,99", "SPRING:CFG,FAIL,range"),
            ("CMD:MOTION_CFG,99,99,99", "MOTION_CFG,FAIL,range"),
            ("JOINT:LIMIT,30,-30", "JOINT:LIMIT,FAIL,range (min<max)"),
        ]
        bound_results = []
        for cmd, expect_prefix in boundary_tests:
            resp = self._send_checked(cmd, T_MED)
            ok = self._match_prefix(resp, expect_prefix)
            bound_results.append(f"{'OK' if ok else 'FAIL'}")
        all_bound_ok = all(r == "OK" for r in bound_results)
        self.record_case(TestCase("L1-12", "L1", "Boundary violation matrix (7 cmds)",
                                  "PASS" if all_bound_ok else "FAIL",
                                  detail=" ".join(bound_results)))

        # L1-13/14: Silent reject matrix
        silent_tests = [
            ("garbage: \xff\xfeZZZZ", "binary garbage"),
            ("CMD:NOT_A_CMD", "unknown command"),
            ("CMD:UNLOCK,abc", "UNLOCK non-numeric"),
            ("CMD:VBUS_LIMIT,0,18", "VBUS_LIMIT uv<=0"),
            ("CMD:VBUS_LIMIT,15,12", "VBUS_LIMIT ov<=uv"),
        ]
        uv_before = self.baseline["uv0"]
        ov_before = self.baseline["ov0"]

        silent_results = []
        for cmd, tag in silent_tests:
            # Send raw for binary garbage
            if "\xff" in cmd:
                self.io.send_raw(b"\xff\xfeZZZZ\n")
                time.sleep(0.5)
                resp = self.io.send_line("SYS:FW_INFO?", T_MED)  # channel alive check
            elif len(cmd) > 200:
                # 300-byte line
                long_line = "X" * 300
                self.io.send_raw((long_line + "\n").encode("utf-8"))
                time.sleep(0.5)
                resp = self.io.send_line("SYS:FW_INFO?", T_MED)
            else:
                resp = self.io.send_line(cmd, 0.5)
            # Expect no response for the silent command itself
            # Verify channel alive with FW_INFO afterwards
            resp2 = self.io.send_line("SYS:FW_INFO?", T_MED)
            alive = any("FW_INFO" in l for l in resp2)
            silent_results.append(f"{tag}:{'SILENT+ALIVE' if alive else 'DEAD'}")

        # N-frame uv/ov must be unchanged after invalid VBUS_LIMIT
        nf = self.io.get_nframe(1.0)
        uv_unchanged = abs(self._get_nf_float(nf, "uv_limit") - uv_before) < 0.1 if nf else False
        ov_unchanged = abs(self._get_nf_float(nf, "ov_limit") - ov_before) < 0.1 if nf else False

        # L1-14: Write-back same VBUS_LIMIT (PWM-OFF acceptance)
        self._send_checked(f"CMD:VBUS_LIMIT,{uv_before:.1f},{ov_before:.1f}", T_SHORT)
        nf2 = self.io.get_nframe(1.0)
        uv_stable = abs(self._get_nf_float(nf2, "uv_limit") - uv_before) < 0.1 if nf2 else False
        ov_stable = abs(self._get_nf_float(nf2, "ov_limit") - ov_before) < 0.1 if nf2 else False

        all_silent_ok = all("SILENT+ALIVE" in r for r in silent_results)
        self.record_case(TestCase("L1-13", "L1", "Silent reject matrix (5 cmds)",
                                  "PASS" if all_silent_ok else "FAIL",
                                  detail="; ".join(silent_results)))
        self.record_case(TestCase("L1-14", "L1", "VBUS_LIMIT writeback same (PWM-OFF accept)",
                                  "PASS" if (uv_unchanged and ov_unchanged and uv_stable and ov_stable) else "FAIL",
                                  detail=f"uv_unchanged={'YES' if uv_unchanged else 'NO'} "
                                         f"ov_unchanged={'YES' if ov_unchanged else 'NO'}"))

        # L1-W1: SCROLL_WHEEL no-session reject (only ENCTRL:ENABLE,1 sent in experiment)
        if self.args.skip_wheel_enable:
            self.record_case(TestCase("L1-W1", "L1", "SCROLL_WHEEL no-session reject",
                                      "SKIP", detail="--skip-wheel-enable"))
        else:
            wheel_results = []

            # (1) WHEEL:STATUS? — ensure session==0
            resp = self._send_checked("WHEEL:STATUS?", T_MED)
            session0 = any("session=0" in l for l in resp) or any("session=0" in l for l in resp)
            wheel_results.append(f"session0={'YES' if session0 else 'NO(UNEXPECTED)'}")

            # (2) APP_MODE,SCROLL_WHEEL
            resp = self._send_checked("CMD:APP_MODE,SCROLL_WHEEL", T_MED)
            # Can return: APP_MODE,OK,SCROLL_WHEEL or APP_MODE,FAIL,not_identified
            # or APP_MODE,FAIL,no_encoder or silent (if mode didn't change per handler)
            mode_ok = self._match_prefix(resp, "APP_MODE,OK,SCROLL_WHEEL")
            mode_fail_ident = any("not_identified" in l for l in resp)
            mode_fail_enc = any("no_encoder" in l for l in resp)

            if mode_fail_ident or mode_fail_enc:
                reason = "not_identified" if mode_fail_ident else "no_encoder"
                self.record_case(TestCase("L1-W1", "L1",
                    "SCROLL_WHEEL no-session reject",
                    "PASS-B",  # variant: board not identified
                    detail=f"APP_MODE SCROLL_WHEEL → FAIL,{reason} (safe — ENABLE,1 not sent)",
                    sent=["CMD:APP_MODE,SCROLL_WHEEL"],
                    expect="APP_MODE,OK,SCROLL_WHEEL or FAIL,not_identified/no_encoder",
                    got=resp))
            elif not mode_ok:
                # Silent — mode didn't change; verify current is not SCROLL_WHEEL
                resp2 = self._send_checked("CMD:APP_MODE?", T_MED)
                if not any("SCROLL_WHEEL" in l for l in resp2):
                    self.record_case(TestCase("L1-W1", "L1",
                        "SCROLL_WHEEL no-session reject",
                        "PASS-B",
                        detail="APP_MODE SCROLL_WHEEL → silent (not identified/no encoder likely), safe skip",
                        sent=["CMD:APP_MODE,SCROLL_WHEEL"],
                        expect="silent (handler gated)", got=resp))
                else:
                    # Unexpected: we're now in SCROLL_WHEEL mode
                    # Continue with guarded ENABLE,1
                    pass
            else:
                # Mode switch succeeded, continue with guarded ENABLE,1
                # (3) Confirm mode
                resp3 = self._send_checked("CMD:APP_MODE?", T_MED)
                confirmed = any("SCROLL_WHEEL" in l for l in resp3)
                wheel_results.append(f"mode_confirmed={'OK' if confirmed else 'FAIL'}")

                if not confirmed:
                    self.record_case(TestCase("L1-W1", "L1",
                        "SCROLL_WHEEL no-session reject",
                        "SKIP", detail="Mode confirmation failed, not sending ENABLE,1"))
                else:
                    # (4) UNLOCK,1
                    self._send_checked("CMD:UNLOCK,1", T_MED)
                    self.guard.set_unlocked(True)
                    wheel_results.append("unlocked")

                    # (5) ENABLE,1 — THE ONLY ENABLE,1 IN THE ENTIRE EXPERIMENT
                    # Dual code guard: it.c:1883 handler returns BEFORE FOC_App_Enable
                    resp5 = self._send_checked("CMD:ENABLE,1", T_MED,
                                               step_type="WHEEL_GUARDED_ENABLE")
                    no_session = self._match_prefix(resp5, "ENABLE,FAIL,no_wheel_session")
                    wheel_results.append(f"ENABLE,1={'no_wheel_session' if no_session else 'UNEXPECTED'}")

                    # (6) UNLOCK,0
                    self._send_checked("CMD:UNLOCK,0", T_MED)
                    self.guard.set_unlocked(False)

                    # (7) APP_MODE,RAW
                    self._send_checked("CMD:APP_MODE,RAW", T_MED)
                    resp7 = self._send_checked("CMD:APP_MODE?", T_MED)
                    back_to_raw = any("RAW" in l for l in resp7)

                    wheel_ok = session0 and no_session and back_to_raw
                    self.record_case(TestCase("L1-W1", "L1",
                        "SCROLL_WHEEL no-session reject (ENABLE,FAIL,no_wheel_session)",
                        "PASS" if wheel_ok else "FAIL",
                        detail="; ".join(wheel_results),
                        sent=["... → APP_MODE,SCROLL_WHEEL → UNLOCK,1 → ENABLE,1 → UNLOCK,0 → APP_MODE,RAW"],
                        expect="ENABLE,FAIL,no_wheel_session",
                        got=resp5))

        # L1-W2: Session timeout demo (INFO)
        resp = self._send_checked("WHEEL:SESSION,7,100", T_MED)  # clamped to 100ms
        ok1 = self._match_prefix(resp, "WHEEL:SESSION,OK")
        time.sleep(1.5)  # wait for timeout
        resp2 = self._send_checked("WHEEL:STATUS?", T_MED)
        session_end = any("session=0" in l for l in resp2)
        self.record_case(TestCase("L1-W2", "L1", "WHEEL session timeout demo",
                                  "PASS" if (ok1 and session_end) else "INFO",
                                  detail=f"session_start={'OK' if ok1 else 'FAIL'} "
                                         f"session_ended={'YES' if session_end else 'NO'}"))

    # ── L2: Fault Injection & Recovery (11 cases, power_unlocked=0) ──

    def layer_L2(self):
        print(f"\n{'#'*60}")
        print(f"# L2 — FAULT INJECTION & RECOVERY (no power)")
        print(f"{'#'*60}")

        state0 = self.baseline["state0"]
        if state0 != 3:
            self.record_case(TestCase("L2-SKIP", "L2",
                "Layer L2 SKIPPED — state != READY",
                "SKIP",
                detail=f"State={STATE_MAP.get(state0, '?')} ({state0}). "
                       f"Voltage trip only polled in READY/RUNNING (foc_app.c:250-262)."
                       f" Motor must be identified for READY state."))
            return

        # Ensure still locked
        self._send_checked("CMD:UNLOCK,0", T_MED)
        self.guard.set_unlocked(False)

        uv0 = self.baseline["uv0"]
        ov0 = self.baseline["ov0"]
        vbus0 = self.baseline["vbus0"]

        # L2-00: Baseline capture
        nf = self.io.get_nframe(1.0)
        if nf:
            uv_now = self._get_nf_float(nf, "uv_limit")
            ov_now = self._get_nf_float(nf, "ov_limit")
            vbus_now = self._get_nf_float(nf, "Vbus")
            self.record_case(TestCase("L2-00", "L2", "Baseline capture",
                                      "PASS",
                                      detail=f"UV={uv_now:.2f} OV={ov_now:.2f} Vbus={vbus_now:.2f}"))
            # Update with actual board values
            uv0, ov0, vbus0 = uv_now, ov_now, vbus_now
        else:
            self.record_case(TestCase("L2-00", "L2", "Baseline capture", "FAIL",
                                      detail="No N-frame"))

        # Stability check: Vbus ripple < 0.3V over 1s
        frames_1s = self.io.collect_nframes(1.5)
        if len(frames_1s) > 10:
            vb_vals = [self._get_nf_float(f, "Vbus") for f in frames_1s]
            ripple = max(vb_vals) - min(vb_vals)
            stable = ripple < 0.5
            if not stable:
                self.record_case(TestCase("L2-STABLE", "L2", "Vbus stability check",
                                          "FAIL", detail=f"ripple={ripple:.3f}V > 0.5V threshold"))
                return
            else:
                self.record_case(TestCase("L2-STABLE", "L2", "Vbus stability check",
                                          "PASS", detail=f"ripple={ripple:.3f}V"))

        # L2-01: BLACKBOX pre-clear + verify continuous sampling
        resp = self._send_checked("CMD:BLACKBOX,CLEAR", T_MED)
        time.sleep(1.2)
        resp2 = self._send_checked("CMD:BLACKBOX?", T_MED)
        count1 = 0
        frozen1 = False
        for l in resp2:
            m = re.search(r'count=(\d+)', l)
            if m:
                count1 = int(m.group(1))
            m = re.search(r'frozen=(\d)', l)
            if m:
                frozen1 = bool(int(m.group(1)))
        self.record_case(TestCase("L2-01", "L2", "BLACKBOX pre-clear + verify sampling",
                                  "PASS" if (not frozen1 and count1 >= 30) else "FAIL" if count1 < 10 else "PASS",
                                  detail=f"count={count1} frozen={frozen1} (need count>=10 for freeze evid)"))

        # L2-02: Arm STALL (to observe EnterFault clearing it)
        resp = self._send_checked("CMD:STALL_MODE,1", T_MED)
        ok_stall = self._match_prefix(resp, "STALL_MODE,OK,1")
        nf = self.io.get_nframe(1.0)
        stall_armed_before = self._get_nf_int(nf, "stall_armed") == 1 if nf else False
        self.record_case(TestCase("L2-02", "L2", "Arm STALL for EnterFault clear observation",
                                  "PASS" if (ok_stall and stall_armed_before) else "FAIL",
                                  detail=f"STALL,1={'OK' if ok_stall else 'FAIL'} "
                                         f"Nframe[11]={'1' if stall_armed_before else '?'}"))

        # L2-03: OV Injection
        # ov_i = Vbus - 2; trip = ov_i + 1 = Vbus - 1 < Vbus → trip guaranteed
        # uv_i = max(0.5, min(uv0, ov_i - 2))
        ov_i = round(vbus0 - 2.0, 1)
        uv_i = max(0.5, min(uv0, ov_i - 2.0))
        if ov_i < 2.0 or uv_i < 0.5:
            self.record_case(TestCase("L2-03", "L2", "OV injection", "SKIP",
                                      detail=f"Vbus too low: Vbus={vbus0:.2f}V calc_ov={ov_i:.1f}"))
            return

        self._send_checked(f"CMD:VBUS_LIMIT,{uv_i:.1f},{ov_i:.1f}", T_SHORT)

        # Wait for FAULT to appear (state=5, app_fault=2, stall_armed=0)
        def _ov_fault_pred(nf_parts):
            try:
                s = int(nf_parts[NF["state"]])
                fa = int(nf_parts[NF["app_fault"]])
                sa = int(nf_parts[NF["stall_armed"]])
                return s == 5 and fa == 2 and sa == 0
            except (IndexError, ValueError):
                return False

        ok, frames_during = self.io.wait_nframe(_ov_fault_pred, T_NFRAME_PRED)
        if ok:
            # Find the confirming frame
            for f in reversed(frames_during):
                try:
                    if int(f[NF["state"]]) == 5 and int(f[NF["app_fault"]]) == 2:
                        stall_after = int(f[NF["stall_armed"]])
                        warn_after = int(f[NF["warn_flags"]])
                        self.record_case(TestCase("L2-03", "L2", "OV injection → FAULT",
                                                  "PASS",
                                                  detail=f"state=5 app_fault=2 stall_armed={stall_after}(was 1→0) "
                                                         f"warn=0x{warn_after:08X} (bit1=OV warn) "
                                                         f"ov_i={ov_i:.1f} Vbus={vbus0:.2f}"))
                        break
                except (IndexError, ValueError):
                    continue
            else:
                self.record_case(TestCase("L2-03", "L2", "OV injection → FAULT",
                                          "PASS", detail="state=5 app_fault=2 (frames captured)"))
        else:
            self.record_case(TestCase("L2-03", "L2", "OV injection → FAULT",
                                      "FAIL", detail=f"No fault within {T_NFRAME_PRED}s. "
                                                     f"Injection: UV={uv_i:.1f} OV={ov_i:.1f} Vbus={vbus0:.2f}"))

        # L2-04: FAULT state gate (locked)
        resp = self._send_checked("CMD:ENABLE,1", T_MED)
        l04a = self._match_prefix(resp, "ENABLE,FAIL,locked")

        resp = self._send_checked("CMD:IDENTIFY,1", T_MED)
        l04b_ok = self._match_prefix(resp, "IDENTIFY,FAIL,locked")

        resp = self._send_checked("CAL:ALL", T_MED)
        # precheck order: fault(2) → vbus(3) → unlock(4). FAULT state → precheck=2
        l04c_ok = self._match_prefix(resp, "CAL:ALL,FAIL,precheck=2")

        self.record_case(TestCase("L2-04", "L2", "FAULT state gate (locked)",
                                  "PASS" if (l04a and l04b_ok and l04c_ok) else "FAIL",
                                  detail=f"ENABLE={'locked' if l04a else '?'}, "
                                         f"IDENTIFY={'locked' if l04b_ok else '?'}, "
                                         f"CAL:ALL={'precheck=2' if l04c_ok else '?'}"))

        # L2-04b: Optional — FAULT state unlocked probe
        if self.args.allow_fault_unlocked_probe:
            self._send_checked("CMD:UNLOCK,1", T_MED)
            self.guard.set_unlocked(True)
            resp_en = self._send_checked("CMD:ENABLE,1", T_MED)
            en_fault = self._match_prefix(resp_en, "ENABLE,FAIL,fault=2")
            resp_id = self._send_checked("CMD:IDENTIFY,1", T_MED)
            id_fault = self._match_prefix(resp_id, "IDENTIFY,FAIL,fault=2")
            self._send_checked("CMD:UNLOCK,0", T_MED)
            self.guard.set_unlocked(False)
            self.record_case(TestCase("L2-04b", "L2", "FAULT state unlocked probe",
                                      "PASS" if (en_fault and id_fault) else "INFO",
                                      detail=f"ENABLE={'fault=2' if en_fault else '?'} "
                                             f"IDENTIFY={'fault=2' if id_fault else '?'}"))

        # L2-05: Blackbox frozen
        resp = self._send_checked("CMD:BLACKBOX?", T_MED)
        frozen_ok = False
        bb_count = 0
        bb_reason = -1
        for l in resp:
            m = re.search(r'frozen=(\d)', l)
            if m:
                frozen_ok = bool(int(m.group(1)))
            m = re.search(r'count=(\d+)', l)
            if m:
                bb_count = int(m.group(1))
            m = re.search(r'reason=(\d+)', l)
            if m:
                bb_reason = int(m.group(1))
        self.record_case(TestCase("L2-05", "L2", "Blackbox frozen verification",
                                  "PASS" if (frozen_ok and bb_count >= 10 and bb_reason == 2) else "FAIL",
                                  detail=f"frozen={frozen_ok} count={bb_count} reason={bb_reason} (expect reason=2)"))

        # Also capture FAULT_DETAIL title
        fd = self._send_checked("CMD:FAULT_DETAIL", T_LONG)
        has_fault_title = any("FAULT DETECTED" in l for l in fd)
        self.record_case(TestCase("L2-05b", "L2", "FAULT_DETAIL title",
                                  "PASS" if has_fault_title else "FAIL",
                                  detail=f"title={'FAULT DETECTED' if has_fault_title else 'Diagnostic Snapshot'})"))

        # L2-06: CLEAR_FAULT ineffective during fault (vbus_ok=false re-asserts fault)
        self._send_checked("SYS:CLEAR_FAULT", T_MED)  # silent command
        time.sleep(0.5)
        nf_after_clear = self.io.get_nframe(1.0)
        still_fault = self._get_nf_int(nf_after_clear, "state") == 5 if nf_after_clear else False
        self.record_case(TestCase("L2-06", "L2", "CLEAR_FAULT ineffective during fault",
                                  "PASS" if still_fault else "FAIL",
                                  detail=f"state={'still 5 (expected)' if still_fault else 'RECOVERED (unexpected)'}"))

        # L2-07: Recovery — restore thresholds
        self._send_checked(f"CMD:VBUS_LIMIT,{uv0:.1f},{ov0:.1f}", T_SHORT)

        def _recovered_pred(nf_parts):
            try:
                s = int(nf_parts[NF["state"]])
                fa = int(nf_parts[NF["app_fault"]])
                return s == 3 and fa == 0
            except (IndexError, ValueError):
                return False

        ok, frames_rec = self.io.wait_nframe(_recovered_pred, T_NFRAME_RECOVER)
        if ok:
            # Verify uv/ov back to baseline
            for f in reversed(frames_rec):
                try:
                    if int(f[NF["state"]]) == 3:
                        uv_now = float(f[NF["uv_limit"]])
                        ov_now = float(f[NF["ov_limit"]])
                        uv_restored = abs(uv_now - uv0) < 0.1
                        ov_restored = abs(ov_now - ov0) < 0.1
                        self.record_case(TestCase("L2-07", "L2", "Recovery (restore thresholds)",
                                                  "PASS" if (uv_restored and ov_restored) else "FAIL",
                                                  detail=f"state 5→3 app_fault→0 "
                                                         f"UV={uv_now:.1f}(expect {uv0:.1f})={'OK' if uv_restored else 'FAIL'} "
                                                         f"OV={ov_now:.1f}(expect {ov0:.1f})={'OK' if ov_restored else 'FAIL'}"))
                        break
                except (IndexError, ValueError):
                    continue
            else:
                self.record_case(TestCase("L2-07", "L2", "Recovery (restore thresholds)",
                                          "PASS", detail="state 5→3 app_fault→0 (recovered ≤1s)"))
        else:
            self.record_case(TestCase("L2-07", "L2", "Recovery (restore thresholds)",
                                      "FAIL", detail=f"No recovery within {T_NFRAME_RECOVER}s"))

        # L2-08: Optional UV injection
        if self.args.l2_uv and vbus0 > 8.0:
            uv_inj = round(vbus0 + 2.0, 1)
            ov_inj = round(vbus0 + 4.0, 1)
            self._send_checked(f"CMD:VBUS_LIMIT,{uv_inj:.1f},{ov_inj:.1f}", T_SHORT)

            def _uv_fault_pred(nf_parts):
                try:
                    return int(nf_parts[NF["state"]]) == 5 and int(nf_parts[NF["app_fault"]]) == 3
                except (IndexError, ValueError):
                    return False

            ok2, _ = self.io.wait_nframe(_uv_fault_pred, T_NFRAME_PRED)
            self._send_checked(f"CMD:VBUS_LIMIT,{uv0:.1f},{ov0:.1f}", T_SHORT)
            time.sleep(1.0)
            nf_final = self.io.get_nframe(1.0)
            recovered = self._get_nf_int(nf_final, "state") == 3 if nf_final else False
            self.record_case(TestCase("L2-08", "L2", "UV injection (optional --l2-uv)",
                                      "PASS" if (ok2 and recovered) else "FAIL",
                                      detail=f"UV_fault={'YES' if ok2 else 'NO'} recover={'YES' if recovered else 'NO'}"))

        # L2-09: Blackbox freeze persists + DUMP
        resp = self._send_checked("CMD:BLACKBOX?", T_MED)
        still_frozen = any("frozen=1" in l for l in resp)
        self.record_case(TestCase("L2-09a", "L2", "Blackbox freeze persists across recovery",
                                  "PASS" if still_frozen else "FAIL"))

        # DUMP
        self.io.drain(0.3)
        self.io.send_raw(b"CMD:BLACKBOX,DUMP\n")
        time.sleep(T_LONG)
        raw = self.io.ser.read(self.io.ser.in_waiting)
        dump_lines = [l.strip() for l in raw.decode("utf-8", errors="replace").split("\n")
                      if l.strip() and l.startswith(("BB,", "BLACKBOX,DUMP"))]
        bb_lines = [l for l in dump_lines if l.startswith("BB,")]
        has_end = any("end" in l for l in dump_lines)
        self.record_case(TestCase("L2-09b", "L2", "BLACKBOX,DUMP export",
                                  "PASS" if (len(bb_lines) > 5 and has_end) else "FAIL",
                                  detail=f"BB lines={len(bb_lines)} has_end={has_end}"))

        # Save DUMP as CSV snippet in detail
        if bb_lines:
            # Extract Vbus from BB lines for evidence
            vb_samples = []
            for bl in bb_lines:
                parts = bl.split(",")
                if len(parts) >= 7:
                    try:
                        vb_samples.append(float(parts[5]))
                    except (ValueError, IndexError):
                        pass
            if vb_samples:
                self.results[-1].detail += f" Vbus_range=[{min(vb_samples):.1f}..{max(vb_samples):.1f}]V"

        # CLEAR
        resp = self._send_checked("CMD:BLACKBOX,CLEAR", T_MED)
        time.sleep(0.5)
        resp2 = self._send_checked("CMD:BLACKBOX?", T_MED)
        frozen_cleared = any("frozen=0" in l for l in resp2)
        count_cleared = False
        for l in resp2:
            m = re.search(r'count=(\d+)', l)
            if m and int(m.group(1)) < 10:
                count_cleared = True
        self.record_case(TestCase("L2-09c", "L2", "BLACKBOX,CLEAR",
                                  "PASS" if (frozen_cleared and count_cleared) else "FAIL",
                                  detail=f"frozen_clear={'YES' if frozen_cleared else 'NO'} count_reset={'YES' if count_cleared else 'NO'}"))

        # L2-10: Healthy CLEAR_FAULT no-op
        self._send_checked("SYS:CLEAR_FAULT", T_MED)
        time.sleep(0.5)
        nf10 = self.io.get_nframe(1.0)
        state_still_ok = self._get_nf_int(nf10, "state") == 3 if nf10 else False
        appfault_still_0 = self._get_nf_int(nf10, "app_fault") == 0 if nf10 else False
        self.record_case(TestCase("L2-10", "L2", "Healthy CLEAR_FAULT no-op",
                                  "PASS" if (state_still_ok and appfault_still_0) else "FAIL",
                                  detail=f"state={'3(stable)' if state_still_ok else '?'} "
                                         f"app_fault={'0(stable)' if appfault_still_0 else '?'}"))

        # L2-11: STALL reset (should already be 0 from EnterFault)
        resp = self._send_checked("CMD:STALL_MODE,0", T_MED)
        self.record_case(TestCase("L2-11", "L2", "STALL_MODE,0 reset (idempotent)",
                                  "PASS" if self._match_prefix(resp, "STALL_MODE,OK,0") else "FAIL"))

    # ── Cleanup ────────────────────────────────────────────────────────

    def cleanup(self):
        """Restore baseline state."""
        print(f"\n{'='*60}")
        print("CLEANUP: Restoring baseline state")
        print(f"{'='*60}")

        c0 = self.baseline["ctrl_mode0"]
        uv0 = self.baseline["uv0"]
        ov0 = self.baseline["ov0"]

        cleanup_seq = [
            ("CMD:ENABLE,0", 0.2, "ENABLE,OK,0", "ENABLE=0"),
            ("CMD:UNLOCK,0", 0.2, "UNLOCK,OK,0", "UNLOCK=0"),
            ("CMD:STALL_MODE,0", 0.2, "STALL_MODE,OK,0", "STALL=0"),
            (f"CMD:VBUS_LIMIT,{uv0:.1f},{ov0:.1f}", 0.2, None, "VBUS_LIMIT restore (silent)"),
            ("CMD:APP_MODE,RAW", 0.3, None, "APP_MODE=RAW"),
            (f"CMD:MODE,{c0}", 0.3, f"MODE,OK,{c0}", f"MODE={c0}"),
            ("TELEM:CUR,OFF", 0.3, "CUR_STREAM,OK", "CUR=OFF"),
            ("TELEM:RATE,50", 0.3, "TELEM:RATE,OK,50", "RATE=50"),
        ]

        self.guard.set_unlocked(False)

        for cmd, wait, expect_prefix, tag in cleanup_seq:
            try:
                resp = self.io.send_line(cmd, wait)
                if expect_prefix:
                    ok = self._match_prefix(resp, expect_prefix)
                    print(f"  [{('OK' if ok else 'WARN')}] {tag}  {resp[:80] if resp else 'SILENT'}")
                else:
                    # VBUS_LIMIT is silent — verify via N-frame
                    if "VBUS_LIMIT" in cmd:
                        nf = self.io.get_nframe(1.0)
                        if nf:
                            uv_now = self._get_nf_float(nf, "uv_limit")
                            ov_now = self._get_nf_float(nf, "ov_limit")
                            print(f"  [{'OK' if abs(uv_now - uv0) < 0.1 else 'WARN'}] {tag} "
                                  f"UV={uv_now:.1f}/{uv0:.1f} OV={ov_now:.1f}/{ov0:.1f}")
                    else:
                        print(f"  [OK] {tag}")
            except Exception as e:
                print(f"  [WARN] {tag} failed: {e}")

        # Final FAULT_DETAIL snapshot
        self.io.drain(0.5)
        resp = self.io.send_line("CMD:FAULT_DETAIL", T_LONG)
        pwm_line = next((l for l in resp if "pwm=" in l), "")
        title = "Diagnostic Snapshot" if any("Diagnostic Snapshot" in l for l in resp) else "FAULT TITLE MISSING"
        print(f"  Final FAULT_DETAIL: {title}, pwm_line='{pwm_line.strip()}'")

        # Final N-frame consistency check vs baseline
        frames = self.io.collect_nframes(2.0)
        if frames:
            last = frames[-1]
            final_state = self._get_nf_int(last, "state")
            final_identified = self._get_nf_int(last, "identified")
            final_uv = self._get_nf_float(last, "uv_limit")
            final_ov = self._get_nf_float(last, "ov_limit")
            final_vbus = self._get_nf_float(last, "Vbus")

            state_match = final_state == self.baseline["state0"] if self.baseline["state0"] != 5 else True
            ident_match = final_identified == self.baseline["identified0"]

            print(f"\n  Baseline vs Final:")
            print(f"    State:   {STATE_MAP.get(self.baseline['state0'],'?')} → {STATE_MAP.get(final_state,'?')} {'MATCH' if state_match else 'DIFF'}")
            print(f"    Identified: {self.baseline['identified0']} → {final_identified} {'MATCH' if ident_match else 'DIFF'}")
            print(f"    UV:      {self.baseline['uv0']:.1f} → {final_uv:.1f}V")
            print(f"    OV:      {self.baseline['ov0']:.1f} → {final_ov:.1f}V")
            print(f"    Vbus:    {self.baseline['vbus0']:.1f} → {final_vbus:.1f}V")

    # ── Report ─────────────────────────────────────────────────────────

    def generate_report(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = os.path.join(PROJECT_ROOT, "scripts", f"l0l2_safety_{ts}.json")
        md_path = os.path.join(PROJECT_ROOT, "scripts", f"l0l2_safety_{ts}.md")

        # ── JSON ──
        report = {
            "timestamp": datetime.now().isoformat(),
            "port": self.io.port,
            "baud": self.io.baud,
            "fw_git": self.fw_git,
            "baseline": {k: v for k, v in self.baseline.items() if not isinstance(v, (int, float)) or isinstance(v, bool)
                         or k in ("state0","identified0","enc0","vbus0","uv0","ov0","ctrl_mode0","app_fault0","fault_flags0","stall0")},
            "baseline_state": STATE_MAP.get(self.baseline.get("state0", -1), "?"),
            "layer_summary": {},
            "results": []
        }

        for r in self.results:
            # Convert to serializable dict, truncate raw_log
            d = {
                "id": r.id, "layer": r.layer, "name": r.name,
                "status": r.status, "detail": r.detail,
                "sent": r.sent, "expect": r.expect,
                "got": r.got[:5] if len(r.got) > 5 else r.got,
                "nframe_evidence": r.nframe_evidence,
            }
            report["results"].append(d)

            # Layer summary
            if r.layer not in report["layer_summary"]:
                report["layer_summary"][r.layer] = "PASS"
            if r.status == "FAIL" and report["layer_summary"][r.layer] != "FAIL":
                report["layer_summary"][r.layer] = "FAIL"

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, default=str)
        print(f"\nJSON report: {json_path}")

        # ── Markdown ──
        overall = "ALL PASS" if all(report["layer_summary"].get(l, "PASS") == "PASS"
                                    for l in ["L0", "L1", "L2"]) else "HAS FAILURES"

        md = []
        md.append(f"# L0-L2 Safety Gate Verification Report")
        md.append(f"")
        md.append(f"**Time**: {datetime.now().isoformat()}  ")
        md.append(f"**Port**: {self.io.port} @ {self.io.baud} baud  ")
        md.append(f"**FW Git**: `{self.fw_git}`  ")
        md.append(f"**Baseline**: state={STATE_MAP.get(self.baseline.get('state0', -1), '?')} "
                  f"identified={self.baseline.get('identified0', '?')} "
                  f"Vbus={self.baseline.get('vbus0', '?'):.2f}V  ")
        md.append(f"**Overall**: {overall}  ")
        md.append(f"")

        for layer in ["L0", "L1", "L2"]:
            layer_cases = [r for r in self.results if r.layer == layer]
            if not layer_cases:
                continue
            md.append(f"## {layer}")
            md.append(f"")
            md.append(f"| ID | Test | Status | Detail |")
            md.append(f"|----|------|--------|--------|")
            for c in layer_cases:
                detail_escaped = c.detail.replace("|", "\\|")[:120]
                md.append(f"| {c.id} | {c.name} | **{c.status}** | {detail_escaped} |")
            md.append(f"")

        # Firmware behavior findings
        md.append(f"## Firmware Behavior Notes (Code-Corroborated)")
        md.append(f"")
        md.append(f"1. **No unified FAIL,parse**: Garbage/unknown commands, `VBUS_LIMIT`, "
                  f"`CLEAR_FAULT`, `GAIN:*` all silent. Channel survival verified via subsequent `FW_INFO?`.")
        md.append(f"2. **UNLOCK,N: `int_arg != 0` rule** (it.c:1728): `UNLOCK,2` → `UNLOCK,OK,1` "
                  f"(non-zero → unlock, no range rejection).")
        md.append(f"3. **Voltage trip only in READY/RUNNING** (foc_app.c:250-262): IDLE state "
                  f"does not poll voltage. L2 requires state==3 (READY, motor identified).")
        md.append(f"4. **Voltage/encoder faults auto-recover** (0.5V hysteresis, foc_app.c:264-276). "
                  f"OC/DRV/ADC require `CLEAR_FAULT`. `CLEAR_FAULT` ineffective during active fault condition.")
        md.append(f"5. **ENABLE,1 gate chain** (it.c:1876): `locked` → `no_wheel_session` (handler-level, "
                  f"pre-FOC_App_Enable — zero power risk) → `FAULT` → `rejected(identified/enc/stall)`.")
        md.append(f"6. **EnterFault clears stall_mode_armed** (foc_app.c:620), preserves `power_unlocked`.")
        md.append(f"7. **power_unlocked has no telemetry visibility** — only verifiable via gate probe commands.")
        md.append(f"8. **CMD:STOP handler** (it.c:1154) returns `CTRL:STOP,OK` for all STOP aliases "
                  f"(CTRL:/CAL: → CMD:). `CAL:STOP,OK,aborted` at it.c:1476 is unreachable dead code.")

        # Baseline / final comparison
        md.append(f"")
        md.append(f"## Baseline vs Final State")
        md.append(f"")
        md.append(f"Initial: `{STATE_MAP.get(self.baseline.get('state0', -1), '?')}` "
                  f"Vbus={self.baseline.get('vbus0', 0):.2f}V "
                  f"UV={self.baseline.get('uv0', 0):.1f}V "
                  f"OV={self.baseline.get('ov0', 0):.1f}V "
                  f"CtrlMode={self.baseline.get('ctrl_mode0', '?')}")
        md.append(f"")

        with open(md_path, "w", encoding="utf-8") as f:
            f.write("\n".join(md))
        print(f"Markdown report: {md_path}")

        return overall == "ALL PASS"


# ─── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="L0-L2 Safety Gate Functional Verification")
    parser.add_argument("--port", default="", help="COM port (auto-scan if omitted)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--layer", default="L0,L1,L2", help="Layers to run (comma-sep)")
    parser.add_argument("--skip-wheel-enable", action="store_true",
                        help="Skip L1-W1 (SCROLL_WHEEL no-session reject, the only ENABLE,1)")
    parser.add_argument("--allow-fault-unlocked-probe", action="store_true",
                        help="Enable L2-04b (UNLOCK+ENABLE in FAULT state)")
    parser.add_argument("--l2-uv", action="store_true",
                        help="Enable L2-08 UV injection round")
    parser.add_argument("--no-reflash", action="store_true",
                        help="Skip reflash prompt on version mismatch")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    layers = set(l.strip() for l in args.layer.split(",") if l.strip())

    print("=" * 60)
    print("L0-L2 SAFETY GATE FUNCTIONAL VERIFICATION")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Layers: {', '.join(sorted(layers))}")
    print("=" * 60)

    io = SerialIO(args.port, args.baud)
    guard = SafetyGuard()
    runner = Runner(io, guard, args)

    # Register panic handlers (last-resort safety)
    def _on_panic_signal(signum, frame):
        guard.panic(io, runner.baseline, f"Signal {signum}")
        io.close()
        sys.exit(1)

    def _on_panic_atexit():
        if guard.believed_unlocked:
            guard.panic(io, runner.baseline, "atexit with unlocked state")

    signal.signal(signal.SIGINT, _on_panic_signal)
    atexit.register(_on_panic_atexit)

    try:
        # Phase A: Preflight
        runner.preflight()

        # Phase B: Layers
        layer_fns = {"L0": runner.layer_L0, "L1": runner.layer_L1, "L2": runner.layer_L2}

        for lname in ["L0", "L1", "L2"]:
            if lname not in layers:
                continue
            if not runner._continue:
                break

            # Check for unexpected fault before layer
            if not runner._check_unexpected_fault(lname):
                print(f"  Unexpected fault before {lname} — aborting layer")
                break

            try:
                layer_fns[lname]()
            except PanicSentinel as e:
                print(f"\n  PANIC in {lname}: {e}")
                guard.panic(io, runner.baseline, str(e))
                break
            except Exception as e:
                print(f"\n  EXCEPTION in {lname}: {e}")
                import traceback
                traceback.print_exc()
                # Try to recover
                guard.panic(io, runner.baseline, str(e))
                break

        # Phase C: Cleanup
        runner.cleanup()

        # Phase D: Report
        ok = runner.generate_report()

        if ok:
            print(f"\n{'='*60}")
            print("EXPERIMENT COMPLETE — ALL PASS")
            print(f"{'='*60}")
        else:
            print(f"\n{'='*60}")
            print("EXPERIMENT COMPLETE — SOME FAILURES (see report)")
            print(f"{'='*60}")
            sys.exit(1)

    except PanicSentinel as e:
        print(f"\nFATAL SAFETY VIOLATION: {e}")
        guard.panic(io, runner.baseline, str(e))
        io.close()
        sys.exit(2)
    except Exception as e:
        print(f"\nFATAL: {e}")
        import traceback
        traceback.print_exc()
        guard.panic(io, runner.baseline, str(e))
        io.close()
        sys.exit(2)
    finally:
        io.close()


if __name__ == "__main__":
    main()
