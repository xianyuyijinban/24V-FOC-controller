#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""g1_trajectory.py — 漂移轮轨迹捕获 (2026-09-03 Kimi 归因终审)

目的: 连跑 G1 (最便宜序列), PDBBIN 200Hz 全程落盘 (钉位前 2s 起), 捕 >=2 轮漂移。
归因三分法 (修正版):
  iq_ref/iq_cmd 对位置"漂移"全程无反应 → 观测链假象实锤 (电机没动, 读数在漂) → 查磁体紧固+pp计算链
  iq_ref 有反应但压不住 → 真实物理漂移 → 再按振荡/慢爬/死区三分

判据: 漂移轮看 pos_err 轨迹 + iq_cmd 轨迹是否同步跟随:
  - pos_err 大但 iq_cmd 不变 → 观测假象
  - pos_err 大且 iq_cmd 跟随 (时变) → 真实

用法: python scripts/low_speed/g1_trajectory.py COM10 --power-ok [--rounds 16]
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
    ap.add_argument("--rounds", type=int, default=16)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--win", type=float, default=2.0)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--stop-at", type=int, default=2, help="捕到 N 轮漂移即停")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
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

    def read_angle():
        dl = time.time() + 0.8
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        return float(p[3])
            time.sleep(0.01)
        return None

    # PDBBIN 采集器 (200Hz, 轨迹) — 单线程 reader 统一读, parser 分发
    # 主线程不再直接读串口 (双线程同时 read 会 segfault — 2026-09-03 实测)
    pdb_rows = []
    nframe_q = []   # N 帧行队列 (line_cb 写入, 主线程轮询)
    stop = [False]

    def on_pdb(s):
        pdb_rows.append((s.host_rx_time, s.theta_user_rad,
                         s.pos_err_rad, s.iq_cmd, s.iq_act, s.pos_ref_rad))

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

    # NOTE: reader 线程必须在 PDBBIN 使能后才启动 — 提前启动会抢占配置命令响应
    # (gating bug 家族: UNLOCK/POS_DIRECT OK 被 reader 吞掉 → expect 超时)

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
    a0 = read_angle()
    if a0 is None:
        print("no angle"); ser.close(); return 1
    # 钉到当前位再等 (G1 纯静止)
    ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
    time.sleep(1.0)
    a0 = read_angle()
    print("锚定位: %.2f" % a0, flush=True)

    # 开 PDBBIN (之后才启动 reader 线程 — gating bug 修复)
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    import threading
    threading.Thread(target=pdb_reader, daemon=True).start()

    results = []
    drift_count = 0
    try:
        for i in range(args.rounds):
            # 轮开始: 清轨迹索引 (从钉位前开始)
            row_base = len(pdb_rows)
            time.sleep(1.0)   # 钉位后稳定 1s
            # 稳态窗 2s (同时收 PDBBIN 轨迹 + N 帧 iq_ref 时间序列) — 单线程 reader 供给
            t0 = time.time()
            iq_refs_t = []
            angs = []
            dl = t0 + args.win
            while time.time() < dl:
                # 从 nframe_q 取 N 帧 (reader 线程已解析)
                while nframe_q:
                    l = nframe_q.pop(0)
                    p = l.split(",")
                    if len(p) >= 25:
                        try:
                            angs.append(float(p[3]))
                        except ValueError:
                            pass
                        try:
                            iq_refs_t.append((time.time(), float(p[19])))
                        except ValueError:
                            pass
                time.sleep(0.002)
            pp = (max(angs) - min(angs)) if angs else -1
            cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
            # 轨迹切片 (PDBBIN 该轮范围)
            traj = pdb_rows[row_base:]
            # 分析判据: pos_err 范围 + iq_cmd 范围 (跟随关系)
            pos_errs = [r[2] for r in traj]
            iq_cmds = [r[3] for r in traj]
            pe_pp = (max(pos_errs) - min(pos_errs)) if pos_errs else 0
            iq_pp = (max(iq_cmds) - min(iq_cmds)) if iq_cmds else 0
            # iq_ref N帧
            iqr_pk = (max(v for _, v in iq_refs_t) - min(v for _, v in iq_refs_t)) if iq_refs_t else 0
            ts = time.strftime("%H:%M:%S")
            print("轮 %d: ts=%s pp=%.3f pos_err_pp=%.3frad iq_cmd_pp=%.4fA iq_ref_pp=%.4fA %s" %
                  (i, ts, pp, pe_pp, iq_pp, iqr_pk, cls), flush=True)
            results.append({"round": i, "ts": ts, "pp": round(pp, 4), "cls": cls,
                            "pos_err_pp": round(pe_pp, 5), "iq_cmd_pp": round(iq_pp, 5),
                            "iq_ref_pp": round(iqr_pk, 5),
                            "traj": [(round(r[0], 3), round(r[1], 5), round(r[2], 5),
                                      round(r[3], 5), round(r[4], 5), round(r[5], 5))
                                     for r in traj]})
            if cls == "DRIFT":
                drift_count += 1
                print("  *** DRIFT 捕获 %d/%d — 轨迹已存, 可停" % (drift_count, args.stop_at), flush=True)
                if drift_count >= args.stop_at:
                    break
            time.sleep(1.5)
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
                       "g1_traj_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    print("捕获 %d 漂移轮 (共 %d 轮)" % (drift_count, len(results)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
