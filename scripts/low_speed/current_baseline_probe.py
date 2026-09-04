#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""current_baseline_probe.py — A 侧基线复查: 电流模式连跑 N 轮 + 热状态记录 (2026-09-03 裁决)

目的: 判断电流模式双态漂移 2/2 高频是热相关还是新回归 (C1 检查)
- 电流模式 (MODE,2 + POS_DIRECT,1) 定版配置连跑 N 轮 (默认 4)
- 每轮: 钉位 → 稳态 2s 窗 pp (双态判据: >=3° 漂移态)
- 每轮记录: 时间戳 + Vbus + DIR? 状态字 (integral/fric) 作为热代理
- 交错 ABBA 协议前先单侧基线

用法: python scripts/low_speed/current_baseline_probe.py COM10 --power-ok [--rounds 4]
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import serial

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--win", type=float, default=2.0)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
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

    def expect(cmd, prefix, timeout=1.5):
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

    def read_vbus():
        """N 帧 p7 = vbus V"""
        dl = time.time() + 0.5
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        try:
                            return float(p[7])
                        except ValueError:
                            return None
            time.sleep(0.01)
        return None

    def read_dir_state():
        """DIR? 状态字 (停流后抓, 避免 P0 被 P1 抢)"""
        send("CMD:PDBBIN,0", 0.3)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"CMD:DIR?\n")
        time.sleep(0.4)
        s = b""
        t0 = time.time()
        while time.time() - t0 < 1.5:
            if ser.in_waiting:
                s += ser.read(ser.in_waiting)
            else:
                time.sleep(0.05)
        send("CMD:PDBBIN,1", 0.3)
        return next((l.strip() for l in s.decode(errors="replace").split("\n")
                     if l.startswith("DIR,OK")), None)

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
    print("start angle: %.2f" % a0, flush=True)
    ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
    time.sleep(0.6)
    a0 = read_angle()

    results = []
    try:
        for i in range(args.rounds):
            # 钉位 + 稳定后测 2s 窗 (同 verify 稳态)
            ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
            time.sleep(args.settle)
            t0 = time.time()
            angs = []
            vbus = None
            dl = t0 + args.win
            while time.time() < dl:
                for l in lb._drain():
                    if l.startswith("N,"):
                        p = l.split(",")
                        if len(p) >= 25:
                            if int(p[8], 16) != 0:
                                raise RuntimeError("fault: %s" % p[8])
                            angs.append(float(p[3]))
                            if vbus is None:
                                try:
                                    vbus = float(p[7])
                                except ValueError:
                                    pass
                time.sleep(0.002)
            pp = (max(angs) - min(angs)) if angs else -1
            cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
            dir_state = read_dir_state()
            ts = time.strftime("%H:%M:%S")
            print("轮 %d: ts=%s pp=%.3f Vbus=%.2f %s | %s" %
                  (i, ts, pp, vbus if vbus else -1, cls, (dir_state or "")[:60]), flush=True)
            results.append({"round": i, "ts": ts, "pp": round(pp, 4), "cls": cls,
                            "vbus": round(vbus, 3) if vbus else None, "dir_state": dir_state})
            # 稍等让热状态接近稳态 (下一轮开始前)
            time.sleep(2.0)
    finally:
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "current_baseline_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    drift = sum(1 for r in results if r["cls"] == "DRIFT")
    print("基线: %d 轮 %d 漂移态" % (len(results), drift))
    return 0


if __name__ == "__main__":
    sys.exit(main())
