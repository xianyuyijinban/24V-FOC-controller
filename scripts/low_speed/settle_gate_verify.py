#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""settle_gate_verify.py — 稳定判据门版 G1/G4 各 8 轮 (2026-09-03 Kimi 抽签框架收官)

判据 (Kimi):
- settle 改稳定判据门: 回位后每 150ms 读角度, 连续 2s 滑动窗 pp<0.1° 才进正式测量窗
- 超时上限 15s, 超时记 TIMEOUT 并把尾巴轨迹存下来
- 可证伪预测 1: 过了门的测量窗零漂移
- 可证伪预测 2: 超时/额外等待率 ≈12% (与漂移率同源)
- G4 轮1 尾巴形态: 单调爬(收敛尾巴) vs 振荡(另一回事)

用法: python scripts/low_speed/settle_gate_verify.py COM10 --power-ok
"""
import argparse
import json
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--gate-pp", type=float, default=0.1, help="稳定门阈值 deg")
    ap.add_argument("--gate-window", type=float, default=2.0, help="稳定门滑动窗 s")
    ap.add_argument("--gate-timeout", type=float, default=15.0, help="稳定门超时 s")
    ap.add_argument("--meas-win", type=float, default=2.0, help="正式测量窗 s")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:VOLT_OFF\n"):
        ser.write(c)
        time.sleep(0.2)
    ser.reset_input_buffer()

    class LineBuffer:
        def __init__(self):
            self.buf = ""

        def _drain(self):
            n = ser.in_waiting
            if not n:
                return []
            self.buf += ser.read(n).decode(errors="replace")
            if "\n" not in self.buf:
                return []
            lines = self.buf.split("\n")
            self.buf = lines.pop()
            return lines

    lb = LineBuffer()

    def expect(cmd, prefix, timeout=3.0):
        ser.reset_input_buffer()
        lb.buf = ""
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            for l in lb._drain():
                if l.strip().startswith(prefix):
                    return True
            time.sleep(0.01)
        return False

    def send(cmd, wait=0.3):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)

    # JDIAG
    ser.reset_input_buffer()
    ser.write(b"CMD:JDIAG\n")
    dl = time.time() + 5.0
    jbuf = ""
    while time.time() < dl:
        if ser.in_waiting:
            jbuf += ser.read(ser.in_waiting).decode(errors="replace")
            if "JDIAG," in jbuf:
                break
        else:
            time.sleep(0.02)
    jline = next((l.strip() for l in jbuf.replace("\r", "").split("\n") if l.startswith("JDIAG,")), None)
    print("JDIAG:", jline[:110] if jline else "无响应")

    # PDBBIN + N帧 单线程 reader (避免双线程读串口 segfault)
    pdb_rows = []
    nframe_q = []
    stop = [False]

    def on_pdb(s):
        pdb_rows.append((s.host_rx_time, s.theta_user_rad, s.pos_err_rad, s.iq_cmd))

    def on_line(line):
        if line.strip().startswith("N,"):
            nframe_q.append(line.strip())

    parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb)

    def pdb_reader():
        while not stop[0]:
            try:
                n = ser.in_waiting
                if n:
                    parser.feed(ser.read(n))
                else:
                    time.sleep(0.001)
            except Exception:
                break

    # 配置 (电流模式定版)
    send("CMD:UNLOCK,1", 0.15)
    ser.reset_input_buffer()
    if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
        print("POS_DIRECT fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK"):
        print("GAIN fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK"):
        print("KI fail"); ser.close(); return 1
    send("CMD:COG_CFG,0.0,60.0", 0.15)
    if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
        print("FRIC fail"); ser.close(); return 1
    if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
        print("AW fail"); ser.close(); return 1
    if not expect("CMD:MODE,2", "MODE,OK"):
        print("MODE fail"); ser.close(); return 1
    time.sleep(0.3)
    en_ok = False
    for attempt in range(3):
        if expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
            en_ok = True
            break
        send("CMD:CLEAR_FAULT", 0.8)
    if not en_ok:
        print("ENABLE fail"); ser.close(); return 1
    ser.write(b"CMD:ON\n")
    time.sleep(0.4)
    ser.reset_input_buffer()
    time.sleep(0.5)

    # NB: reader 线程 (pdb_reader) 在 PDBBIN 使能后才启动; 主线程读角统一走 nframe_q
    # (避免双线程同时读串口 segfault — 2026-09-03 家族)
    def read_angle():
        dl = time.time() + 0.8
        while time.time() < dl:
            while nframe_q:
                l = nframe_q.pop(0)
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        return float(p[3])
                    except ValueError:
                        pass
            time.sleep(0.01)
        return None

    # 先钉锚 (reader 未启动, 用 lb 安全)
    lb._drain()
    ser.reset_input_buffer()
    time.sleep(0.2)
    # 用 lb 读当前角 (reader 未启动)
    a0 = None
    dl = time.time() + 1.0
    while time.time() < dl:
        for l in lb._drain():
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        a0 = float(p[3])
                    except ValueError:
                        pass
        if a0 is not None:
            break
        time.sleep(0.02)
    if a0 is None:
        print("no angle"); ser.close(); return 1
    print("锚定位: %.2f°" % a0, flush=True)
    # 钉位 (用 a0 直接发, 不再运动中重读)
    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
    time.sleep(2.0)
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    import threading
    threading.Thread(target=pdb_reader, daemon=True).start()

    def wait_stable(target_deg, timeout):
        """稳定判据门: 每 150ms 读角度 (nframe_q), 连续 gate_window 滑动窗 pp < gate_pp 才通过。
        返回 (ok, waited_s, traj_start_idx)"""
        t0 = time.time()
        recent = []
        rb = len(pdb_rows)
        while time.time() - t0 < timeout:
            while nframe_q:
                l = nframe_q.pop(0)
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        a = float(p[3])
                    except ValueError:
                        continue
                    # 角度相对目标 (环绕)
                    d = a - target_deg
                    if d > 180.0: d -= 360.0
                    elif d < -180.0: d += 360.0
                    recent.append((time.time(), d))
                    cutoff = time.time() - args.gate_window
                    recent = [x for x in recent if x[0] >= cutoff]
                    if len(recent) >= 5:
                        pp = max(x[1] for x in recent) - min(x[1] for x in recent)
                        if pp < args.gate_pp:
                            return True, time.time() - t0, rb
            time.sleep(0.15)
        return False, time.time() - t0, rb

    def measure_win(target_deg, rb):
        """正式测量窗 (过了门的 2s N帧 pp, nframe_q 供给)"""
        t0 = time.time()
        angs_meas = []
        dl = t0 + args.meas_win
        while time.time() < dl:
            while nframe_q:
                l = nframe_q.pop(0)
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        a = float(p[3])
                        d = a - target_deg
                        if d > 180.0: d -= 360.0
                        elif d < -180.0: d += 360.0
                        angs_meas.append(d)
                    except ValueError:
                        pass
            time.sleep(0.002)
        pp = (max(angs_meas) - min(angs_meas)) if angs_meas else -1
        traj = pdb_rows[rb:]
        return pp, traj

    results = []
    groups = [("G1_static", "none"), ("G4_rampstep", "rampstep")]
    try:
        for gname, seq_type in groups:
            print("\n===== %s (%s) =====" % (gname, seq_type), flush=True)
            for i in range(8):
                # PDBBIN 轨迹断点
                rb = len(pdb_rows)
                if seq_type == "none":
                    pass
                elif seq_type == "rampstep":
                    # 斜坡 6° (12s @0.5°/s) + 阶跃 +6° + 回位
                    dur = 12.0
                    t1 = time.time()
                    last = 0.0
                    while time.time() - t1 < dur:
                        t = time.time() - t1
                        if t - last >= 0.2:
                            frac = min(t / dur, 1.0)
                            cur = a0 * DEG2RAD + (6 * DEG2RAD) * frac
                            ser.write(b"CMD:PREF,%.6f\n" % cur)
                            last = t
                        nframe_q.clear()   # reader 已启动, 主线程只清队列不碰串口 (segfault 修复)
                    ser.write(b"CMD:PREF,%.6f\n" % ((a0 + 6) * DEG2RAD))
                    time.sleep(2.0)
                    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
                    time.sleep(1.0)
                # 稳定门
                ok, waited, rb2 = wait_stable(a0, args.gate_timeout)
                if not ok:
                    traj = pdb_rows[rb:]
                    ts = time.strftime("%H:%M:%S")
                    print("  %s 轮%d: TIMEOUT(%.1fs) %s — 尾巴存%s" % (gname, i, waited, ts, len(traj)), flush=True)
                    results.append({"group": gname, "round": i, "timeout": True,
                                    "waited_s": round(waited, 2), "traj_len": len(traj)})
                    continue
                pp, traj = measure_win(a0, rb2)
                cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
                ts = time.strftime("%H:%M:%S")
                print("  %s 轮%d: gate_wait=%.1fs pp=%.3f %s (%d traj)" %
                      (gname, i, waited, pp, cls, len(traj)), flush=True)
                results.append({"group": gname, "round": i, "timeout": False,
                                "waited_s": round(waited, 2), "pp": round(pp, 4),
                                "cls": cls, "traj_len": len(traj)})
                time.sleep(1.0)
    finally:
        stop[0] = True
        time.sleep(0.3)
        ser.write(b"CMD:VOLT_OFF")
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "settle_gate_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    to = [r for r in results if r["timeout"]]
    drift = [r for r in results if "cls" in r and r["cls"] == "DRIFT"]
    print("超时/总轮: %d/%d (%.0f%%)" % (len(to), len(results), len(to) / len(results) * 100))
    print("过门后漂移: %d" % len(drift))
    return 0


if __name__ == "__main__":
    sys.exit(main())
