#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""g4_anatomy2.py — G4-only 轨迹全量捕获 (2026-09-03 Kimi 第三轮规格)

规格 (Kimi):
1. 轨迹全量落盘: 每轮全程(斜坡+阶跃+回位+门+测量窗) PDBBIN 行 + N帧(p1时间戳/p3角/p6 Iq/p19 Iq_ref) 存 JSON
2. 门启动时先排空并记录积压帧数 (每轮一个数) — 解释 gate_wait 方差
3. N帧 p1 时间戳进轨迹 — 帧新鲜度自证
4. 捕 2 轮漂移 + 2 轮干净即停

判别矩阵 (PDBBIN payload: theta_user/pos_err/iq_cmd 交叉):
  theta_user 走6° + pos_err镜像 + 连续slew(≥几十帧) → 电机真动 → stick-slip: 看静止期iq_cmd静默爬升
  theta_user 走6° + pos_err镜像 + 1-2帧跳变 → 观测毛刺
  theta_user 走6° + pos_err不动 → 用户帧转换bug
  漂移轮12s斜坡期theta没ramp到a0+6 → 那轮从斜坡期粘死/异常

用法: python scripts/low_speed/g4_anatomy2.py COM10 --power-ok
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
    ap.add_argument("--gate-pp", type=float, default=0.1)
    ap.add_argument("--gate-window", type=float, default=2.0)
    ap.add_argument("--gate-timeout", type=float, default=15.0)
    ap.add_argument("--meas-win", type=float, default=2.0)
    ap.add_argument("--drift-target", type=int, default=2)
    ap.add_argument("--clean-target", type=int, default=2)
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

    # 单线程 reader (PDBBIN + N帧 统一走 parser; 主线程不碰串口读)
    pdb_rows = []       # (host_rx_time, theta_user_rad, pos_err_rad, iq_cmd, flags)
    nframe_q = []       # (host_rx_time, p1_ts, p3_ang, p6_iq, p19_iqref)
    stop = [False]

    def on_pdb(s):
        pdb_rows.append((s.host_rx_time, s.theta_user_rad, s.pos_err_rad, s.iq_cmd, s.flags))

    def on_line(line):
        l = line.strip()
        if l.startswith("N,"):
            p = l.split(",")
            if len(p) >= 25:
                try:
                    nframe_q.append((time.time(), p[1], float(p[3]),
                                     float(p[6]), float(p[19])))
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

    # 锚定位 (reader 未启动 — lb 安全)
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
    drift_n = 0
    clean_n = 0
    try:
        round_i = 0
        while (drift_n < args.drift_target or clean_n < args.clean_target) and round_i < 20:
            rb = len(pdb_rows)
            rb_n = len(nframe_q)
            # G4 序列: 斜坡6°(12s) + 阶跃6° + 回位
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
            time.sleep(1.0)
            # 稳定门: 排空并记录积压帧数 (发现2 验证)
            backlog_n = len(nframe_q)
            nframe_q.clear()
            t0 = time.time()
            recent = []
            ok = False
            while time.time() - t0 < args.gate_timeout:
                while nframe_q:
                    host_t, p1, ang, iq, iqr = nframe_q.pop(0)
                    d = ang - a0
                    if d > 180.0: d -= 360.0
                    elif d < -180.0: d += 360.0
                    recent.append((host_t, d))
                    cutoff = time.time() - args.gate_window
                    recent = [x for x in recent if x[0] >= cutoff]
                    if len(recent) >= 5:
                        pp = max(x[1] for x in recent) - min(x[1] for x in recent)
                        if pp < args.gate_pp:
                            ok = True
                            break
                if ok:
                    break
                time.sleep(0.02)
            waited = time.time() - t0
            # 测量窗 (全量收 PDBBIN + N帧)
            meas_start = len(pdb_rows)
            m0 = time.time()
            angs_meas = []
            dl2 = m0 + args.meas_win
            while time.time() < dl2:
                while nframe_q:
                    host_t, p1, ang, iq, iqr = nframe_q.pop(0)
                    d = ang - a0
                    if d > 180.0: d -= 360.0
                    elif d < -180.0: d += 360.0
                    angs_meas.append(d)
                time.sleep(0.002)
            pp = (max(angs_meas) - min(angs_meas)) if angs_meas else -1
            cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
            # 全量轨迹切片
            traj = pdb_rows[rb:]
            ntraj = nframe_q  # 空 (已清)
            ts = time.strftime("%H:%M:%S")
            print("G4 轮%d: ts=%s backlog=%d gate_wait=%.1fs(ok=%s) pp=%.3f %s (pdb_traj=%d)" %
                  (round_i, ts, backlog_n, waited, ok, pp, cls, len(traj)), flush=True)
            results.append({
                "round": round_i, "ts": ts, "backlog_n": backlog_n,
                "gate_ok": ok, "gate_wait_s": round(waited, 2),
                "pp": round(pp, 4), "cls": cls,
                "pdb_traj": [(round(r[0], 3), round(r[1], 5), round(r[2], 5),
                              round(r[3], 5), r[4]) for r in traj],
                "nframe_count": len(traj),
            })
            if cls == "DRIFT":
                drift_n += 1
            elif cls == "CLEAN":
                clean_n += 1
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
                       "g4_anatomy_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    print("捕 %d 漂移 %d 干净 (目标 %d/%d)" % (drift_n, clean_n, args.drift_target, args.clean_target))
    return 0


if __name__ == "__main__":
    sys.exit(main())
