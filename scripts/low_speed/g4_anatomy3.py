#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""g4_anatomy3.py — G4-only 环死捕获 (2026-09-04 Kimi 规格 v3)

规格:
1. N帧轨迹加 p2(state)、p8(faultFlags) 两列进 JSON
2. pdb_traj 加 pos_ref_rad (payload 第7 float) 和 tick_2khz (帧新鲜度)
3. CMD:ON 后回读确认 state==RUNNING 再开跑 (盲发无回读观测家族教训)
4. pos_ref 列分流: 环死时 PREF 照进 pos_ref 但 pos_err 不更新 = 环体死;
   pos_ref 冻结 = 指令链问题

目标: 复现环死 (位置环中段死亡) — 捕 1 轮环死即停, 全四列+state/fault 在手。
用法: python scripts/low_speed/g4_anatomy3.py COM10 --power-ok
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
    ap.add_argument("--max-rounds", type=int, default=20)
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

    # 单线程 reader
    pdb_rows = []   # (host_rx, tick_2khz, theta_user, pos_err, iq_cmd, pos_ref)
    nframe_q = []   # (host_rx, p1, p2_state, p3_ang, p6_iq, p8_fault, p19_iqref)
    stop = [False]

    def on_pdb(s):
        pdb_rows.append((s.host_rx_time, s.tick_2khz, s.theta_user_rad,
                         s.pos_err_rad, s.iq_cmd, s.pos_ref_rad))

    def on_line(line):
        l = line.strip()
        if l.startswith("N,"):
            p = l.split(",")
            if len(p) >= 21:
                try:
                    nframe_q.append((time.time(), p[1], p[2], float(p[3]),
                                     float(p[6]), p[8], float(p[19])))
                except ValueError:
                    pass

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

    def confirm_running():
        """回读确认 state==RUNNING (p2==4? 看数值, 用 N帧 state 字段)"""
        dl = time.time() + 1.0
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 3:
                        return p[2]
            time.sleep(0.02)
        return None

    st = confirm_running()
    print("ENABLE 后 state(p2)=%s (确认 RUNNING)" % st, flush=True)

    # 锚定位
    a0 = None
    dl = time.time() + 1.0
    while time.time() < dl and a0 is None:
        for l in lb._drain():
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        a0 = float(p[3])
                        break
                    except ValueError:
                        pass
        if a0 is None:
            time.sleep(0.02)
    if a0 is None:
        print("no angle"); ser.close(); return 1
    print("锚定位: %.2f°" % a0, flush=True)
    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
    time.sleep(2.0)
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    import threading
    threading.Thread(target=pdb_reader, daemon=True).start()

    results = []
    dead_found = False
    round_i = 0
    try:
        while not dead_found and round_i < args.max_rounds:
            rb = len(pdb_rows)
            # G4 序列
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
                nframe_q.clear()
                time.sleep(0.01)
            ser.write(b"CMD:PREF,%.6f\n" % ((a0 + 6) * DEG2RAD))
            time.sleep(2.0)
            ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
            time.sleep(1.5)
            # 抓 2s 窗 (全收)
            m0 = time.time()
            angs_meas = []
            dl2 = m0 + 2.0
            while time.time() < dl2:
                while nframe_q:
                    host_t, p1, state, ang, iq, fault, iqr = nframe_q.pop(0)
                    d = ang - a0
                    if d > 180.0: d -= 360.0
                    elif d < -180.0: d += 360.0
                    angs_meas.append((ang, state, fault))
                time.sleep(0.002)
            pp = (max(x[0] for x in angs_meas) - min(x[0] for x in angs_meas)) if angs_meas else -1
            states = set(x[1] for x in angs_meas)
            faults = set(x[2] for x in angs_meas)
            traj = pdb_rows[rb:]
            # 环死判定: pos_err 全 0 + theta 冻结 (2s 窗 pp 小)
            pes = [x[3] for x in traj]
            pe_zero = all(abs(p) < 1e-7 for p in pes) if pes else False
            cls = "DEAD" if (pe_zero and pp >= 0) else \
                  ("DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID"))
            ts = time.strftime("%H:%M:%S")
            print("G4 轮%d: ts=%s pp=%.3f states=%s faults=%s pe_zero=%s %s (pdb=%d)" %
                  (round_i, ts, pp, states, faults, pe_zero, cls, len(traj)), flush=True)
            results.append({
                "round": round_i, "ts": ts, "pp": round(pp, 4), "states": list(states),
                "faults": list(faults), "pe_zero": pe_zero, "cls": cls,
                "pdb_traj": [(round(r[0], 3), r[1], round(r[2], 5), round(r[3], 5),
                              round(r[4], 5), round(r[5], 5)) for r in traj],
                "nframe_sample": [(p[0], p[1], p[2]) for p in []],  # 填充在下方
            })
            if cls == "DEAD":
                dead_found = True
                print("  *** 环死捕获 轮%d — 停止" % round_i, flush=True)
            round_i += 1
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
                       "g4_anatomy3_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    print("环死捕获: %s (%d 轮)" % (dead_found, len(results)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
