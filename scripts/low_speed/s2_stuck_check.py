#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s2_stuck_check.py — stuck-clean 判定: 电压模式稳态窗平均位置误差 (2026-09-03 Kimi)

判据: 电压模式 (MODE,3+POS_DIRECT) 钉位后稳态 2s:
  |err| 几度且 pp≈0 → stuck-clean (欠力推不动, 信号作废)
  err≈0 且 pp≈0 → 真干净

抓: 钉位后 2s 窗的 avg|err| (N帧 p[3] vs PREF 目标) + pp + iq_ref 均值。
用法: python scripts/low_speed/s2_stuck_check.py COM10 --power-ok
"""
import argparse
import json
import math
import os
import sys
import time

import serial

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--kp", type=float, default=0.245)   # 半增益 (S2 保守)
    ap.add_argument("--kd", type=float, default=0.0035)
    ap.add_argument("--ki", type=float, default=0.185)
    ap.add_argument("--win", type=float, default=2.0)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:VOLT_OFF\n", b"CMD:MODE,0\n"):
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

    # 配置 (电压模式 S2)
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
    if not expect("CMD:MODE,3", "MODE,OK"):
        print("MODE,3 fail"); ser.close(); return 1
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

    a0 = read_angle()
    if a0 is None:
        print("no angle"); ser.close(); return 1
    print("当前位置: %.2f°" % a0, flush=True)
    # 钉到当前位置
    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
    time.sleep(6.0)   # 让它收敛 (欠力状态尾巴可能长)
    # 稳态窗 2s: 读 pos_err ≈ a_meas - a_target (p[3] vs 目标)
    t0 = time.time()
    angs = []
    iq_refs = []
    dl = t0 + args.win
    while time.time() < dl:
        for l in lb._drain():
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 25:
                    try:
                        angs.append(float(p[3]))
                    except ValueError:
                        pass
                    try:
                        iq_refs.append(float(p[19]))
                    except ValueError:
                        pass
        time.sleep(0.002)
    if not angs:
        print("无 N 帧"); ser.close(); return 1
    mean_a = sum(angs) / len(angs)
    pp = max(angs) - min(angs)
    mean_iqr = sum(iq_refs) / len(iq_refs) if iq_refs else 0
    err_deg = mean_a - a0   # 平均位置误差 (相对钉位目标)
    print("\n稳态 2s: 平均角=%.2f° 目标=%.2f° err=%+.2f° pp=%.3f° iq_ref_mean=%+.4fA" %
          (mean_a, a0, err_deg, pp, mean_iqr), flush=True)
    verdict = "stuck-clean" if (abs(err_deg) > 1.0 and pp < 0.5) else \
              ("真实干净" if (abs(err_deg) < 1.0 and pp < 0.5) else "中间态")
    print("判据: %s (err>1° 且 pp<0.5° = stuck-clean)" % verdict)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "s2_stuck_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "err_deg": round(err_deg, 3), "pp": round(pp, 4),
                   "iq_ref_mean": round(mean_iqr, 5), "verdict": verdict}, f,
                  ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    ser.write(b"CMD:VOLT_OFF")
    send("CMD:OFF")
    send("CMD:MODE,0")
    send("CMD:STOP")
    send("CMD:CLEAR_FAULT")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
