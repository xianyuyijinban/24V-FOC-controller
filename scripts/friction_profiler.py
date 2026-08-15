#!/usr/bin/env python3
"""
摩擦特性测量 — 力矩模式递增 IREF 找启动电流(克服静摩擦+齿槽所需力矩)

方法: TORQUE 模式(MODE,0)下, IREF 从 0 递增到 I_max(±), 每档保持短时
观察电机是否转动。启动电流 = 电机开始连续转动的 IREF 阈值, 正反向各测。
同时采样实际 Iq/Vq 交叉验证。

风险控制: 电流限幅在 ±I_max(默认0.3A), 每档观察 <0.8s, 转动即记录并回零。

用法: python scripts/friction_profiler.py --port COM10 --power-ok [--imax 0.3]
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pos_direct_ab_test import N_IDX


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--imax", type=float, default=0.30, help="最大扫描电流 A")
    ap.add_argument("--istep", type=float, default=0.02, help="电流步进 A")
    ap.add_argument("--hold", type=float, default=0.8, help="每档观察时长 s")
    ap.add_argument("--move-thresh", type=float, default=1.0,
                    help="判定转动角度阈值 deg(档内累计)")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok (电机将转动)")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def send(cmd, wait=0.5):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)

    def expect(cmd, prefix, timeout=1.5):
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    l = l.strip()
                    if l.startswith(prefix):
                        return True
            else:
                time.sleep(0.01)
        return False

    def read_n():
        dl = time.time() + 0.6
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.startswith("N,"):
                        p = l.split(",")
                        if len(p) >= 25:
                            return {"angle": float(p[3]), "iq_ref": float(p[19]),
                                    "iq_act": float(p[6]), "vq": float(p[21]),
                                    "fault": int(p[14])}
            else:
                time.sleep(0.01)
        return None

    send("CMD:UNLOCK,1")
    expect("CMD:POS_DIRECT,0", "POS_DIRECT,OK")
    send("CMD:COG_CFG,0.00,0.0")
    expect("CMD:MODE,0", "MODE,OK")  # TORQUE
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    time.sleep(0.4)

    def probe(direction, imax, istep):
        """从 0 递增到 ±imax, 返回 (启动电流, 各档数据)。direction=+1/-1"""
        rows = []
        start_i = None
        a_ref = None
        for i_abs in [round(x, 4) for x in
                      [k * istep for k in range(0, int(imax / istep) + 1)]]:
            iq = direction * i_abs
            ser.reset_input_buffer()
            ser.write(b"CMD:IREF,0.000,%.4f\n" % iq)  # Id=0, Iq=目标
            time.sleep(args.hold)
            a0 = read_n()
            # 记录转动累计
            if a_ref is None:
                a_ref = a0["angle"] if a0 else None
            else:
                if a0:
                    d = a0["angle"] - a_ref
                    if d > 180.0:
                        d -= 360.0
                    elif d < -180.0:
                        d += 360.0
                    a_ref += d
            moved = 0.0
            if a0:
                # 与档首角度比较(读两次)
                ser.reset_input_buffer()
                ser.write(b"CMD:IREF,0.000,%.4f\n" % iq)
                time.sleep(0.3)
                a1 = read_n()
                if a1:
                    dm = a1["angle"] - a0["angle"]
                    if dm > 180.0:
                        dm -= 360.0
                    elif dm < -180.0:
                        dm += 360.0
                    moved = abs(dm)
            row = {"i": iq, "moved_deg": round(moved, 2),
                   "iq_act": a0["iq_act"] if a0 else None,
                   "vq": a0["vq"] if a0 else None,
                   "fault": a0["fault"] if a0 else 0}
            rows.append(row)
            if moved >= args.move_thresh and start_i is None:
                start_i = iq
            ser.write(b"CMD:IREF,0.000,0.000\n")  # 回零停住
            time.sleep(0.2)
        return start_i, rows

    print("\n=== 正向摩擦测量 (IREF 0→+%.2fA) ===" % args.imax, flush=True)
    pos_i, pos_rows = probe(+1, args.imax, args.istep)
    print("  启动电流(正向): %s A" % ("未达" if pos_i is None else "+%.3f" % pos_i))
    for r in pos_rows:
        mark = " <-启动" if r["i"] == pos_i else ""
        print("  I=+%.3f moved=%.2f° iq_act=%s vq=%s%s" %
              (r["i"], r["moved_deg"], r["iq_act"], r["vq"], mark), flush=True)

    print("\n=== 反向摩擦测量 (IREF 0→-%.2fA) ===" % args.imax, flush=True)
    neg_i, neg_rows = probe(-1, args.imax, args.istep)
    print("  启动电流(反向): %s A" % ("未达" if neg_i is None else "-%.3f" % abs(neg_i)))
    for r in neg_rows:
        mark = " <-启动" if r["i"] == neg_i else ""
        print("  I=%+.3f moved=%.2f° iq_act=%s vq=%s%s" %
              (r["i"], r["moved_deg"], r["iq_act"], r["vq"], mark), flush=True)

    # 汇总
    print("\n================ 摩擦特性 ================")
    if pos_i is not None:
        print("  正向前进电流: %+.3f A  反向: %+.3f A" %
              (pos_i, neg_i if neg_i is not None else 0.0))
        print("  平均启动电流: %.3f A (库仑摩擦≈%.0f mA×Kt)" %
              (statistics.mean([abs(pos_i), abs(neg_i or 0)]), 0))
    else:
        print("  扫描范围内未启动 (imax=%.2fA), 需加大 imax" % args.imax)

    ser.write(b"CMD:STOP")
    ser.write(b"CMD:CLEAR_FAULT")
    ser.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "friction_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "pos": pos_rows, "neg": neg_rows,
                   "pos_start": pos_i, "neg_start": neg_i},
                  f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
