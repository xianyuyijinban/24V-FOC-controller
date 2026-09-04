#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""seq_anatomy.py — 序列解剖矩阵 G1-G4 (2026-09-03 Kimi 序列解剖)

目的: 定位双态漂移触发源到段 (电流环积分残留头号嫌疑)
组:
  G1 纯静止 (钉位+5s+测2s) — 锚点 (基线已 4 净)
  G2 阶跃-only +6° 后测稳态 (verify 原序列)
  G3 斜坡-only 12s 0.5°/s (无阶跃) 后测稳态 — 头号嫌疑
  G4 斜坡+阶跃 (A/B 原序列) — 复现 A/B
每轮必抓: 稳态窗 pp + Iq_ref(p19)/Iq_meas(p6) 残留 + DIR? 状态字

判据 (Kimi): 稳态窗 Iq_ref (或 Iq_meas) 残留非零 → 电流环积分残留实锤;
干净轮 Iq_ref≈0, 漂移轮 Iq_ref 恒非零且衰减。

用法: python scripts/low_speed/seq_anatomy.py COM10 --power-ok [--rounds 4]
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

    def read_dir_state():
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

    def steady_window(a0):
        """钉 a0 + 测 win 窗。返回 (pp, iq_ref_mean, iq_meas_mean, ...)"""
        ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
        time.sleep(args.settle)
        t0 = time.time()
        angs = []
        iq_refs = []
        iq_meases = []
        dl = t0 + args.win
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        if int(p[8], 16) != 0:
                            raise RuntimeError("fault: %s" % p[8])
                        angs.append(float(p[3]))
                        try:
                            iq_refs.append(float(p[19]))   # p[19] = Iq_ref?
                            iq_meases.append(float(p[6]))  # p[6] = Iq 测量
                        except ValueError:
                            pass
            time.sleep(0.002)
        pp = (max(angs) - min(angs)) if angs else -1
        return pp, (sum(iq_refs) / len(iq_refs) if iq_refs else 0), \
               (sum(iq_meases) / len(iq_meases) if iq_meases else 0), len(angs)

    # 核对 p19 到底是啥 (Id_ref=16, Iq_ref=17? 还是 19?)
    import re
    # 从 uart_upload.c 410 行 APPEND_FMT 数: N,%lu %u %s %s %s %s %s 0x%X %u %u %u %u 0x%X %u %u %s %s %s %s %s %s %s %s %s %u %u %s %s %u %d %d %d
    # p0=N p1=ts p2=state p3=angle p4=speed p5=Id p6=Iq p7=vbus p8=fault p9=enc p10=id p11=stall p12=ol p13=warn p14=faultcode p15=mode p16=Id_ref p17=speed_ref p18=pos_ref p19=Iq_ref p20=Vd p21=Vq ...
    # 所以 p19 = Iq_ref ✓ (p16=Id_ref)
    print("字段: p6=Iq_meas p19=Iq_ref", flush=True)

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
    # 钉到 240.7° (解剖固定位)
    ser.write(b"CMD:PREF,%.5f\n" % (240.7 * DEG2RAD))
    time.sleep(1.5)
    a0 = read_angle()
    print("锚定位: %.2f (目标 240.7)" % a0, flush=True)

    results = []
    groups = [("G1_static", "none"), ("G2_step", "step"), ("G3_ramp", "ramp"), ("G4_rampstep", "rampstep")]
    try:
        for gname, seq_type in groups:
            print("\n===== %s (%s) =====" % (gname, seq_type), flush=True)
            for i in range(args.rounds):
                # 序列执行
                if seq_type == "none":
                    pass
                elif seq_type == "step":
                    ser.write(b"CMD:PREF,%.6f\n" % ((a0 + 6) * DEG2RAD))
                    time.sleep(3.0)
                    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
                    time.sleep(1.0)
                elif seq_type == "ramp":
                    # 12s 斜坡 0.5°/s (0.2s 一步)
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
                        # drain N 帧 (避免缓冲堆积)
                        lb._drain()
                elif seq_type == "rampstep":
                    # 斜坡 12s 后阶跃 6°
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
                        lb._drain()
                    ser.write(b"CMD:PREF,%.6f\n" % ((a0 + 6) * DEG2RAD))
                    time.sleep(2.0)
                    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
                    time.sleep(1.0)
                # 稳态窗
                pp, iq_ref, iq_meas, n = steady_window(a0)
                cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
                dir_state = read_dir_state()
                ts = time.strftime("%H:%M:%S")
                print("  %s 轮%d: ts=%s pp=%.3f Iq_ref=%+6.4f Iq_meas=%+6.4f %s | %s" %
                      (gname, i, ts, pp, iq_ref, iq_meas, cls, (dir_state or "")[:50]), flush=True)
                results.append({"group": gname, "round": i, "ts": ts, "pp": round(pp, 4),
                                "iq_ref": round(iq_ref, 5), "iq_meas": round(iq_meas, 5),
                                "cls": cls, "dir_state": dir_state})
                time.sleep(1.0)
    finally:
        ser.write(b"CMD:VOLT_OFF")
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "seq_anatomy_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    for g in groups:
        gres = [r for r in results if r["group"] == g[0]]
        drift = sum(1 for r in gres if r["cls"] == "DRIFT")
        print("%s: %d 轮 %d 漂移" % (g[0], len(gres), drift))
    return 0


if __name__ == "__main__":
    sys.exit(main())
