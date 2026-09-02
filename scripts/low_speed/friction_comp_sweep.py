#!/usr/bin/env python3
"""
静摩擦补偿力矩扫描 — 找最优补偿电流 (DIRECT kp=0.5/kd=0.03 + COG OFF)

对每个补偿电流 fric ∈ {0,0.05,0.07,0.09,0.11,0.13}A:
  ±6° 步进, 采样到位过程: 0.5s/1s/2s 到位度, 稳态抖动, 过冲。
判据: 到位快 + 无过冲 + 稳态稳 = 最优。观察正反向对称。

用法: python scripts/friction_comp_sweep.py --port COM10 --power-ok
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
from common.pos_direct_ab_test import N_IDX

DEG2RAD = 3.14159265358979 / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--comps", default="0,0.05,0.07,0.09,0.11,0.13",
                    help="补偿电流扫描列表 A")
    ap.add_argument("--step", type=float, default=6.0, help="步进幅度 deg")
    ap.add_argument("--settle", type=float, default=3.0, help="每步采样时长 s")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok (电机将转动)")
        return 0
    comps = [float(x) for x in args.comps.split(",")]
    step_rad = args.step * DEG2RAD

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def send(cmd, wait=0.4):
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

    def read_angle():
        dl = time.time() + 0.8
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.startswith("N,"):
                        p = l.split(",")
                        if len(p) >= 25:
                            return float(p[3])
            else:
                time.sleep(0.01)
        return None

    def step_and_track(a0, target_rad):
        """发 PREF 到 target, 采样 settle 秒, 返回 (t, err_deg) 序列"""
        ser.reset_input_buffer()
        ser.write(b"CMD:PREF,%.5f\n" % target_rad)
        samples = []
        t0 = time.time()
        dl = t0 + args.settle
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.startswith("N,"):
                        p = l.split(",")
                        if len(p) >= 25:
                            a = float(p[3])
                            d = a - a0
                            if d > 180.0:
                                d -= 360.0
                            elif d < -180.0:
                                d += 360.0
                            samples.append((time.time() - t0, d))  # 相对起点的位移 deg
            else:
                time.sleep(0.005)
        return samples

    def summarize(samples, target_deg):
        """从轨迹算到位度。target_deg=目标相对位移(如+6)。返回 dict"""
        if len(samples) < 5:
            return None
        target = abs(target_deg)
        # 到位度: 各时间点位移/目标
        err_at = {}
        for tm in (0.5, 1.0, 2.0):
            # 找 tm 时刻最近采样
            best = min(samples, key=lambda s: abs(s[0] - tm))
            frac = abs(best[1]) / target
            err_at[tm] = round((1.0 - min(frac, 1.0)) * 100.0, 1)  # 未到位百分比
        # 稳态窗(最后 1s): 相对目标误差
        stab = [abs(abs(s[1]) - target) for s in samples if s[0] >= args.settle - 1.0]
        # 过冲: 位移超过目标的最大量
        max_overshoot = max((abs(abs(s[1]) - target) for s in samples), default=0.0)
        return {
            "err_at_0p5s": err_at[0.5],
            "err_at_1s": err_at[1.0],
            "err_at_2s": err_at[2.0],
            "steady_pp_deg": max(stab) - min(stab) if stab else 0.0,
            "overshoot_deg": max(0.0, max_overshoot),
        }

    send("CMD:UNLOCK,1")
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,0.5000,0.0300", "POS_DIRECT_GAIN,OK")
    send("CMD:COG_CFG,0.00,0.0")
    expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    time.sleep(0.5)
    # 挪到安全区
    a0 = read_angle()
    if a0 is not None and (a0 < 25 or a0 > 335):
        ser.write(b"CMD:PREF,%.5f\n" % (100.0 * DEG2RAD))
        time.sleep(1.2)
        a0 = read_angle()

    rows = []
    try:
        for comp in comps:
            print("\n=== 补偿电流 %.2fA ===" % comp, flush=True)
            expect("CMD:FRIC_COMP,%.3f,%.3f" % (comp, comp), "FRIC_COMP,OK")
            a0 = read_angle()
            if a0 is None:
                print("  读角失败, 跳过")
                continue
            # 正向
            pos = step_and_track(a0, (a0 + args.step) * DEG2RAD)
            r_pos = summarize(pos, args.step)
            # 回起点
            ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
            time.sleep(0.6)
            # 反向
            neg = step_and_track(a0, (a0 - args.step) * DEG2RAD)
            r_neg = summarize(neg, -args.step)
            ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
            time.sleep(0.6)
            if r_pos is None or r_neg is None:
                print("  采样不足, 跳过")
                continue
            row = {"comp": comp, "pos": r_pos, "neg": r_neg}
            rows.append(row)
            print("  正向: 0.5s剩%.0f%% 1s剩%.0f%% 2s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                  (r_pos["err_at_0p5s"], r_pos["err_at_1s"], r_pos["err_at_2s"],
                   r_pos["steady_pp_deg"], r_pos["overshoot_deg"]), flush=True)
            print("  反向: 0.5s剩%.0f%% 1s剩%.0f%% 2s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                  (r_neg["err_at_0p5s"], r_neg["err_at_1s"], r_neg["err_at_2s"],
                   r_neg["steady_pp_deg"], r_neg["overshoot_deg"]), flush=True)
    finally:
        ser.write(b"CMD:STOP")
        ser.write(b"CMD:CLEAR_FAULT")
        ser.close()

    # 汇总: 用 1s 到位度 + 稳态 + 过冲 综合排序
    print("\n================ 摩擦补偿扫描汇总 ================")
    print("%-8s %-14s %-14s %-12s %-10s" % ("compA", "pos 1s剩%", "neg 1s剩%", "稳态pp均值", "过冲均值"))
    for r in rows:
        pp_avg = (r["pos"]["steady_pp_deg"] + r["neg"]["steady_pp_deg"]) / 2
        os_avg = (r["pos"]["overshoot_deg"] + r["neg"]["overshoot_deg"]) / 2
        print("%-8.2f %-14.1f %-14.1f %-12.3f %-10.3f" %
              (r["comp"], r["pos"]["err_at_1s"], r["neg"]["err_at_1s"], pp_avg, os_avg))
    # 综合评分: 到位度越高越好(剩%低), 稳态pp低, 过冲低
    def score(r):
        s = 0.0
        s += (r["pos"]["err_at_1s"] + r["neg"]["err_at_1s"]) * 0.5  # 到位(低分好)
        s += (r["pos"]["steady_pp_deg"] + r["neg"]["steady_pp_deg"]) * 2.0
        s += (r["pos"]["overshoot_deg"] + r["neg"]["overshoot_deg"]) * 2.0
        return s
    best = min(rows, key=score)
    print("\n  综合最优: 补偿 %.2fA (得分 %.1f)" % (best["comp"], score(best)))

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "friction_comp_sweep_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "rows": rows}, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
