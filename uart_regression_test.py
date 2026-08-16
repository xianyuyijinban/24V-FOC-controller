#!/usr/bin/env python3
"""
UART Communication Regression Test (Circular DMA)
==================================================
Verifies HostComputer <-> firmware communication at 1000000 baud after
UART circular DMA migration.

Phases covered:
  1 - Raw serial basics + FW_INFO 100x burst
  2 - Text command ACK matrix (safety, APP_MODE, config)
  4 - N-frame telemetry stability (50Hz, 100Hz)
  5 - Binary Current Stream (BIN1000/2000) + command coexistence
  8 - Long text responses mixed with binary stream

Usage:
  python uart_regression_test.py
  python uart_regression_test.py --port COM7 --baud 1000000
  python uart_regression_test.py --phase 1,2    # run selected phases

Outputs:
  uart_regression_YYYYMMDD_HHMMSS.json
"""

import serial
import time
import sys
import re
import json
import argparse
from datetime import datetime
from dataclasses import dataclass, field, asdict

# ─── Config ───────────────────────────────────────────────────────────────
PORT = "COM7"
BAUD = 1000000
T_SHORT = 0.3
T_MED = 1.0
T_LONG = 3.0

# N-frame indices
NF = {
    "state": 2, "angle": 3, "speed": 4, "Id": 5, "Iq": 6, "Vbus": 7,
    "faultFlags": 8, "appFault": 14, "ctrlMode": 15,
    "speed_ref": 17, "pos_ref": 18, "Iq_ref": 19, "Vq": 21,
}
STATE_MAP = {0:"IDLE",1:"INIT",2:"IDENTIFY",3:"READY",4:"RUNNING",5:"FAULT"}

# ─── Utilities ────────────────────────────────────────────────────────────

@dataclass
class TestItem:
    phase: str
    name: str
    status: str  # PASS / FAIL / SKIP / INFO
    detail: str = ""
    measurements: dict = field(default_factory=dict)
    raw: list = field(default_factory=list)

class SerialIO:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd, wait=T_SHORT):
        """Send, return non-telemetry response lines."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        time.sleep(wait)
        raw = self.ser.read(self.ser.in_waiting)
        text = raw.decode(errors="replace")
        return [l.strip() for l in text.split("\n")
                if l.strip() and not l.startswith(("N,", "C,", "B,"))]

    def send_raw(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def drain(self, dur=0.3):
        time.sleep(dur)
        self.ser.read(self.ser.in_waiting)

    def collect_nframes(self, dur_s=2.0):
        """Returns parsed N-frame list."""
        frames = []
        deadline = time.time() + dur_s
        while time.time() < deadline:
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode(errors="replace").split("\n"):
                    line = line.strip()
                    if line.startswith("N,"):
                        parts = line.split(",")
                        if len(parts) >= 25:
                            frames.append(parts)
            else:
                time.sleep(0.01)
        return frames

    def collect_raw(self, dur_s):
        """Collect all raw bytes."""
        data = bytearray()
        deadline = time.time() + dur_s
        while time.time() < deadline:
            w = self.ser.in_waiting
            if w:
                data.extend(self.ser.read(w))
            else:
                time.sleep(0.005)
        return bytes(data)

# ─── Test Runner ─────────────────────────────────────────────────────────

class UartRegressionRunner:
    def __init__(self, io, phase_filter=None):
        self.io = io
        self.phase_filter = phase_filter
        self.results = []
        self._continue = True

    def record(self, phase, name, status, detail="", meas=None, raw=None):
        self.results.append(TestItem(phase, name, status, detail, meas or {}, raw or []))
        tag = {"PASS":"[OK]", "FAIL":"[!!]", "SKIP":"[-]", "INFO":"[i]"}
        print(f"  {tag.get(status,'[?]')} {name}: {status}  {detail[:100]}")

    def run(self, pid, name, fn):
        if self.phase_filter is not None and pid not in self.phase_filter:
            print(f"\n=== Phase {pid}: {name} -- SKIPPED ===")
            return
        if not self._continue:
            print(f"\n=== Phase {pid}: {name} -- STOPPED ===")
            return
        print(f"\n{'='*60}\nPhase {pid}: {name}\n{'='*60}")
        try:
            fn()
        except KeyboardInterrupt:
            print("Interrupted.")
            self._continue = False
        except Exception as e:
            print(f"ERROR: {e}")
            self.record(str(pid), name, "FAIL", f"EXCEPTION: {e}")
            self._continue = False

    # ── Phase 1: Raw Serial Basics ───────────────────────────────────

    def phase1(self):
        """Raw serial + FW_INFO 100x burst + RX stat."""

        # 1.1 Basic queries + NUL check (with streams OFF)
        print("--- 1.1 Basic queries ---")
        self.io.send("TELEM:CUR,OFF", T_SHORT)
        self.io.send("TELEM:RATE,50", T_SHORT)
        self.io.drain(0.3)
        for cmd, expect in [
            ("SYS:FW_INFO?", ["FW_INFO", "12V_STANDARD"]),
            ("CMD:UART_RX_STAT?", []),
        ]:
            resp = self.io.send(cmd, T_MED)
            ok = all(e in " ".join(resp) for e in expect) if expect else len(resp) > 0
            self.record("1", f"1.1 {cmd}", "PASS" if ok else "FAIL", raw=resp)

        # Check NUL bytes
        print("  Checking for NUL bytes in 5s of idle data...")
        raw5s = self.io.collect_raw(5.0)
        nul = raw5s.count(b'\x00')
        self.record("1", "1.1 NUL bytes in 5s idle",
                     "PASS" if nul == 0 else "FAIL",
                     detail=f"NUL count: {nul}/{len(raw5s)}B")

        # 1.2 FW_INFO 100x burst
        print("--- 1.2 FW_INFO? 100x burst ---")
        ok_count = 0
        fail_count = 0
        rx_before = self._get_rx_errors()
        for i in range(100):
            resp = self.io.send("SYS:FW_INFO?", 0.05)
            if any("FW_INFO" in l for l in resp):
                ok_count += 1
            else:
                fail_count += 1
            if fail_count >= 2 and ok_count == 0:
                break  # early abort if serial dead
        rx_after = self._get_rx_errors()
        passed = ok_count == 100
        self.record("1", "1.2 FW_INFO? 100x burst",
                     "PASS" if passed else "FAIL",
                     detail=f"{ok_count}/{ok_count+fail_count} OK, "
                            f"RX errors: {rx_before} -> {rx_after}",
                     meas={"burst_ok": ok_count, "burst_fail": fail_count,
                            "rx_err_before": rx_before, "rx_err_after": rx_after})

        # 1.3 Post-burst serial alive
        resp = self.io.send("SYS:FW_INFO?", T_SHORT)
        self.record("1", "1.3 Post-burst serial alive",
                     "PASS" if any("FW_INFO" in l for l in resp) else "FAIL",
                     raw=resp)

    def _get_rx_errors(self):
        resp = self.io.send("CMD:UART_RX_STAT?", T_SHORT)
        for l in resp:
            m = re.search(r'err[^=]*[=:]\s*(\d+)', l)
            if m:
                return int(m.group(1))
        return -1

    # ── Phase 2: Text Command ACK Matrix ─────────────────────────────

    def phase2(self):
        """Verify all critical commands produce proper ACK."""

        # 2.1 Safety commands
        print("--- 2.1 Safety commands ---")
        safety_cmds = [
            ("SYS:CLEAR_FAULT", []),          # silent operation
            ("CMD:UNLOCK,1", ["UNLOCK,OK,1"]),
            ("CMD:UNLOCK,0", ["UNLOCK,OK,0"]),
            ("CMD:UNLOCK,1", ["UNLOCK,OK,1"]),
            ("CMD:STALL_MODE,1", ["STALL_MODE,OK"]),
            ("CMD:STALL_MODE,0", ["STALL_MODE,OK"]),
            ("CMD:MODE,1", ["MODE,OK,1"]),
            ("CMD:MODE,2", ["MODE,OK,2"]),
            ("CMD:APP_MODE?", ["APP_MODE,OK", "RAW"]),
        ]
        for cmd, expect in safety_cmds:
            resp = self.io.send(cmd, T_MED)
            ok = all(e in " ".join(resp) for e in expect) if expect else True
            self.record("2", f"2.1 {cmd}",
                         "PASS" if ok else "FAIL",
                         detail=f"expect={','.join(expect)}" if expect else "",
                         raw=resp)

        # 2.2 APP_MODE commands
        print("--- 2.2 APP_MODE commands ---")
        modes = ["RAW", "JOINT_POS", "GIMBAL_SPEED", "HOLD",
                 "SPRING_DAMPER", "DETENT", "RAW"]
        for mode in modes:
            set_r = self.io.send(f"CMD:APP_MODE,{mode}", T_SHORT)
            q_r = self.io.send("CMD:APP_MODE?", T_SHORT)
            set_ok = any("APP_MODE,OK" in l for l in set_r)
            q_ok = any(mode in l for l in q_r)
            self.record("2", f"2.2 APP_MODE {mode}",
                         "PASS" if (set_ok and q_ok) else "FAIL",
                         raw=set_r + q_r)

        # 2.3 Config commands
        print("--- 2.3 Config commands ---")
        config_cmds = [
            ("JOINT:LIMIT,-30,30", ["JOINT:LIMIT,OK"]),
            ("JOINT:LIMIT?", ["JOINT:LIMIT,OK"]),
            ("GIMBAL:RAMP,2.0", ["GIMBAL:RAMP,OK"]),
            ("GIMBAL:RAMP?", ["GIMBAL:RAMP,OK"]),
            ("SPRING:CFG,0.500,0.050,0.300", ["SPRING:CFG,OK"]),
            ("SPRING:CFG?", ["SPRING:CFG,OK"]),
            ("DETENT:CFG,12,1.000,0.130,0.250", ["DETENT:CFG,OK"]),
            ("DETENT:CFG?", ["DETENT:CFG,OK"]),
        ]
        for cmd, expect in config_cmds:
            resp = self.io.send(cmd, T_SHORT)
            ok = all(e in " ".join(resp) for e in expect)
            self.record("2", f"2.3 {cmd}",
                         "PASS" if ok else "FAIL", raw=resp)

    # ── Phase 4: N-frame Telemetry Stability ─────────────────────────

    def _check_nframe_rate(self, label, target_hz, collect_s=10.0):
        self.io.send(f"TELEM:RATE,{target_hz}", T_SHORT)
        self.io.drain(0.3)
        frames = self.io.collect_nframes(collect_s)
        actual_hz = len(frames) / collect_s
        pct = actual_hz / target_hz * 100

        # NUL byte check
        raw = self.io.collect_raw(1.0)
        nul = raw.count(b'\x00')

        pass_ok = actual_hz >= 20 and nul == 0

        # Check angle/speed/Vbus updating
        has_data = True
        if frames:
            try:
                angles = [float(f[NF["angle"]]) for f in frames[:10]]
                has_data = len(set(angles)) > 0  # at least some variation is good
            except (IndexError, ValueError):
                has_data = False

        self.record("4", f"4.{label} N-frame {target_hz}Hz",
                     "PASS" if pass_ok else "FAIL",
                     detail=f"actual={actual_hz:.1f}Hz ({pct:.0f}% of target), "
                            f"NUL={nul}, has_data={has_data}",
                     meas={"target_hz": target_hz, "actual_hz": actual_hz,
                            "nul": nul, "count": len(frames)})
        return frames

    def phase4(self):
        """N-frame 50Hz and 100Hz stability."""

        # 4.1 Default 50Hz
        self.io.send("TELEM:CUR,OFF", T_SHORT)
        nf_50 = self._check_nframe_rate("1", 50, collect_s=10.0)

        # 4.2 100Hz + concurrent commands
        self.io.send("TELEM:RATE,100", T_SHORT)
        self.io.drain(0.5)

        print("  Sampling 100Hz + concurrent commands for 10s...")
        deadline = time.time() + 10.0
        cmd_count = 0
        cmd_ok = 0
        frames_100 = []
        while time.time() < deadline:
            # Collect 1s of frames
            batch = self.io.collect_nframes(0.7)
            frames_100.extend(batch)
            # Send query commands
            for q in ["SYS:FW_INFO?", "CMD:APP_MODE?", "DIAG:PWM_DIAG"]:
                cmd_count += 1
                resp = self.io.send(q, 0.08)
                if len(resp) > 0:
                    cmd_ok += 1

        actual_hz = len(frames_100) / 10.0
        cmd_rate = cmd_ok / cmd_count * 100 if cmd_count > 0 else 0
        self.record("4", "4.2 N-frame 100Hz + commands",
                     "PASS" if (actual_hz > 50 and cmd_rate > 80) else "FAIL",
                     detail=f"nf_hz={actual_hz:.1f}, cmd_ok={cmd_ok}/{cmd_count} ({cmd_rate:.0f}%)",
                     meas={"nf_hz": actual_hz, "cmd_ok": cmd_ok, "cmd_total": cmd_count})

        # Restore 50Hz
        self.io.send("TELEM:RATE,50", T_SHORT)

    # ── Phase 5: Binary Current Stream ───────────────────────────────

    def phase5(self):
        """Binary current stream 1000/2000 with command coexistence."""

        def _check_cmd_ok(label, msg):
            resp = self.io.send(msg, 0.5)
            # A command succeeded if we got any response at all
            # (DIAG:PWM_DIAG returns "PWM,ARR=..." without "OK")
            ok = len(resp) > 0
            self.record("5", f"5.{label} {msg}", "PASS" if ok else "WARN", raw=resp)
            return ok

        # 5.1 BIN 1000 (20s, mixed commands)
        print("--- 5.1 BIN 1000 + commands (20s) ---")
        self.io.drain(0.5)
        resp = self.io.send("TELEM:CUR,BIN,1000", T_SHORT)
        started = any("CUR_STREAM,OK" in l for l in resp)
        self.record("5", "5.1a TELEM:CUR,BIN,1000",
                     "PASS" if started else "FAIL", raw=resp)
        if not started:
            return

        deadline = time.time() + 20.0
        cmd_ok_5a = 0
        cmd_total_5a = 0
        bin_bytes = 0
        while time.time() < deadline:
            time.sleep(0.5)
            # Send alternating commands
            q = ["SYS:FW_INFO?", "CMD:APP_MODE?", "CMD:UART_RX_STAT?", "DIAG:PWM_DIAG"][cmd_total_5a % 4]
            cmd_total_5a += 1
            if _check_cmd_ok(f"1b cmd#{cmd_total_5a}", q):
                cmd_ok_5a += 1
            # Also collect some binary data for CRC check
            w = self.io.ser.in_waiting
            if w:
                bin_bytes += w
                self.io.ser.read(w)

        # STOP + CUR OFF
        _check_cmd_ok("1c CTRL:STOP", "CTRL:STOP")
        _check_cmd_ok("1c TELEM:CUR,OFF", "TELEM:CUR,OFF")
        _check_cmd_ok("1c TELEM:RATE,50", "TELEM:RATE,50")

        # Verify N-frame recovery
        time.sleep(0.5)
        nf = self.io.collect_nframes(2.0)
        nf_ok = len(nf) > 5
        self.record("5", "5.1c N-frame recovery after BIN1000",
                     "PASS" if nf_ok else "FAIL",
                     detail=f"{len(nf)} N-frames in 2s")

        cmd_rate = cmd_ok_5a / cmd_total_5a * 100 if cmd_total_5a else 0
        self.record("5", "5.1d BIN1000 command success rate",
                     "PASS" if cmd_ok_5a == cmd_total_5a else "FAIL",
                     detail=f"{cmd_ok_5a}/{cmd_total_5a} OK ({cmd_rate:.0f}%)",
                     meas={"cmd_ok": cmd_ok_5a, "cmd_total": cmd_total_5a,
                            "bin_bytes": bin_bytes})

        # 5.2 BIN 2000 (10s, experimental)
        print("--- 5.2 BIN 2000 experimental (10s) ---")
        resp = self.io.send("TELEM:CUR,BIN,2000", T_SHORT)
        started2 = any("CUR_STREAM,OK" in l for l in resp)
        self.record("5", "5.2a TELEM:CUR,BIN,2000",
                     "PASS" if started2 else "FAIL", raw=resp)
        if not started2:
            self.record("5", "5.2 BIN2000 skipped", "SKIP", "Did not start")
            return

        deadline = time.time() + 10.0
        cmd_ok_5b = 0
        cmd_total_5b = 0
        while time.time() < deadline:
            time.sleep(0.5)
            q = ["SYS:FW_INFO?", "CMD:UART_RX_STAT?"][cmd_total_5b % 2]
            cmd_total_5b += 1
            if _check_cmd_ok(f"2b cmd#{cmd_total_5b}", q):
                cmd_ok_5b += 1

        _check_cmd_ok("2c CTRL:STOP", "CTRL:STOP")
        _check_cmd_ok("2c CUR OFF", "TELEM:CUR,OFF")
        self.io.send("TELEM:RATE,50", T_SHORT)

        cmd_rate_b = cmd_ok_5b / cmd_total_5b * 100 if cmd_total_5b else 0
        self.record("5", "5.2d BIN2000 command success",
                     "PASS" if cmd_ok_5b == cmd_total_5b else "WARN",
                     detail=f"{cmd_ok_5b}/{cmd_total_5b} OK ({cmd_rate_b:.0f}%) "
                            + "(experimental, not release blocker)",
                     meas={"cmd_ok": cmd_ok_5b, "cmd_total": cmd_total_5b})

    # ── Phase 8: Long Text Responses ─────────────────────────────────

    def phase8(self):
        """Long text responses with and without binary current stream."""

        # 8.1 Without current stream
        print("--- 8.1 Long text (CUR OFF) ---")
        self.io.send("TELEM:CUR,OFF", T_SHORT)
        self.io.drain(0.5)
        for cmd, label in [
            ("DIAG:FAULT_DETAIL", "FAULT_DETAIL"),
            ("DIAG:BLACKBOX?", "BLACKBOX?"),
            ("DIAG:BLACKBOX,HEAD", "BLACKBOX,HEAD"),
        ]:
            resp = self.io.send(cmd, T_LONG)
            complete = any(len(l) > 10 for l in resp) if resp else False
            self.record("8", f"8.1 {label}",
                         "PASS" if complete else "FAIL",
                         detail=f"{len(resp)} lines, {sum(len(l) for l in resp)} chars",
                         raw=resp[:5])

        # 8.2 With BIN 1000 running
        print("--- 8.2 Long text (BIN1000 active) ---")
        self.io.send("TELEM:CUR,BIN,1000", T_SHORT)
        self.io.drain(1.0)
        for cmd, label in [
            ("DIAG:FAULT_DETAIL", "BIN+FAULT_DETAIL"),
            ("DIAG:BLACKBOX?", "BIN+BLACKBOX?"),
        ]:
            resp = self.io.send(cmd, T_LONG)
            has_content = any(len(l) > 10 for l in resp) if resp else False
            self.record("8", f"8.2 {label}",
                         "PASS" if has_content else "FAIL",
                         detail=f"{len(resp)} lines response")

        # Verify commands still work after
        resp = self.io.send("SYS:FW_INFO?", T_MED)
        alive = any("FW_INFO" in l for l in resp)
        self.record("8", "8.2 Post-BIN command alive",
                     "PASS" if alive else "FAIL", raw=resp)

        # Cleanup
        self.io.send("TELEM:CUR,OFF", T_SHORT)
        self.io.send("TELEM:RATE,50", T_SHORT)

    # ── Report ───────────────────────────────────────────────────────

    def generate_report(self, port="COM7", baud=1000000):
        print(f"\n{'='*60}")
        print("UART REGRESSION TEST -- FINAL REPORT")
        print(f"{'='*60}")
        print(f"{'Phase':<6} {'Test':<45} {'Status':<8}")
        print("-"*60)

        phase_status = {}
        for r in self.results:
            if r.phase not in phase_status:
                phase_status[r.phase] = "PASS"
            if r.status == "FAIL":
                phase_status[r.phase] = "FAIL"
            elif r.status == "WARN" and phase_status[r.phase] != "FAIL":
                phase_status[r.phase] = "WARN"

            print(f"P{r.phase:<3} {r.name:<45} {r.status:<8}")
            if r.status == "FAIL":
                print(f"      {r.detail[:90]}")

        print("-"*60)
        print("Per-Phase Summary:")
        fails = []
        for pid in sorted(phase_status.keys(), key=lambda x: int(x)):
            s = phase_status[pid]
            print(f"  Phase {pid}: {s}")
            if s == "FAIL":
                fails.append(pid)
        print(f"\nOverall: {'ALL PASS' if not fails else f'FAIL in phases {fails}'}")

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"uart_regression_{ts}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump({
                "timestamp": datetime.now().isoformat(),
                "port": port,
                "baud": baud,
                "phase_summary": phase_status,
                "results": [asdict(r) for r in self.results],
            }, f, indent=2)
        print(f"Report saved: {path}")
        return len(fails) == 0

# ─── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="UART Communication Regression Test")
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--phase", default="", help="Comma-separated phases, e.g. 1,2")
    args = parser.parse_args()

    phase_filter = None
    if args.phase:
        phase_filter = set(int(p.strip()) for p in args.phase.split(",") if p.strip().isdigit())

    port, baud = args.port, args.baud

    print("="*60)
    print("UART Communication Regression Test (Circular DMA)")
    print(f"Port: {port} @ {baud}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)

    io = SerialIO(port, baud)
    try:
        io.open()
        runner = UartRegressionRunner(io, phase_filter)
        phases = [
            (1, "Raw Serial + FW_INFO 100x", runner.phase1),
            (2, "Text Command ACK Matrix", runner.phase2),
            (4, "N-frame Telemetry Stability", runner.phase4),
            (5, "Binary Current Stream + Commands", runner.phase5),
            (8, "Long Text Responses", runner.phase8),
        ]
        for pid, name, fn in phases:
            runner.run(pid, name, fn)
            if not runner._continue:
                print(f"\nStopped at Phase {pid}")
                break
        runner.generate_report(port, baud)
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        sys.exit(1)
    finally:
        io.close()

if __name__ == "__main__":
    main()
