#!/usr/bin/env python3
"""
阶跃过冲 KD 扫描 + 斜坡命令验证 — 解决 DIRECT 阶跃阻尼不足问题

背景: 摩擦补偿扫描发现 6° 阶跃普遍过冲 3~5°(60~90% 超调), 0A 补偿也有 5.4°,
  非补偿造成, 是 DIRECT kp=0.5/kd=0.03 阶跃阻尼不足。低速稳态优化(kd 小)
  与大步进阻尼需求矛盾。

实验 A (默认): kp 固定 0.5, kd 扫描 {0.03,0.06,0.09,0.12,0.15,0.20}
  每档 6° 阶跃(±), 测过冲/到位度/稳态pp。
  回答: 加大 KD 能否压过冲? 稳态代价多少?

实验 B (--mode=ramp): PC 端斜坡 PREF 代替阶跃, kd 固定 0.03
  --ramp-deg-s 控制斜坡速度(默认 2.0°/s)。
  回答: 斜坡能否消除过冲(云台真实场景是斜坡而非阶跃)?

摩擦补偿统一固定 0.12A(最优区间), COG OFF。

用法:
  python scripts/overshoot_kd_sweep.py --port COM10 --power-ok
  python scripts/overshoot_kd_sweep.py --mode=ramp --port COM10 --power-ok
  python scripts/overshoot_kd_sweep.py --mode=ramp --ramp-deg-s 1.0 --port COM10 --power-ok
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pos_direct_ab_test import N_IDX

DEG2RAD = 3.14159265358979 / 180.0
FRIC_FIX = 0.12  # 摩擦补偿固定值 A(最优区间)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--mode", default="kd", choices=["kd", "ramp"],
                    help="kd=KD扫描(实验A) / ramp=斜坡命令(实验B)")
    ap.add_argument("--kds", default="0.03,0.06,0.09,0.12,0.15,0.20",
                    help="实验A: KD 扫描列表")
    ap.add_argument("--kp", type=float, default=0.5, help="实验A: 固定 KP")
    ap.add_argument("--ramp-deg-s", type=float, default=2.0, help="实验B: 斜坡速度 deg/s")
    ap.add_argument("--step", type=float, default=6.0, help="步进幅度 deg")
    ap.add_argument("--settle", type=float, default=3.0, help="每步采样时长 s")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok (电机将转动)")
        return 0

    kds = [float(x) for x in args.kds.split(",")]
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

    def ramp_and_track(a0, target_rad, ramp_deg_s):
        """单线程交替斜坡: 每0.2s发一步PREF(低密度写, 避免同句柄并发饿死读) + 读N帧。
        返回 (t, err_deg) 序列, err_deg 为相对起点的位移。"""
        ramp_deg_s = max(ramp_deg_s, 0.1)
        dur = abs(target_rad - a0 * DEG2RAD) / (ramp_deg_s * DEG2RAD)
        t0 = time.time()
        last_sent = 0.0
        samples = []
        dl = t0 + dur + args.settle  # 斜坡 + 到位后 settle
        while time.time() < dl:
            if time.time() - last_sent >= 0.2:
                frac = min((time.time() - t0) / dur, 1.0) if dur > 0 else 1.0
                cur = a0 * DEG2RAD + (target_rad - a0 * DEG2RAD) * frac
                ser.write(b"CMD:PREF,%.5f\n" % cur)
                last_sent = time.time()
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
                            samples.append((time.time() - t0, d))
            else:
                time.sleep(0.005)
        return samples

    def summarize(samples, target_deg, t_end_off=1.0):
        """从轨迹算到位度。target_deg=目标相对位移(如+6)。稳态窗取轨迹末尾 t_end_off 秒。"""
        if len(samples) < 5:
            return None
        target = abs(target_deg)
        err_at = {}
        for tm in (0.5, 1.0, 2.0):
            best = min(samples, key=lambda s: abs(s[0] - tm))
            frac = abs(best[1]) / target
            err_at[tm] = round((1.0 - min(frac, 1.0)) * 100.0, 1)  # 未到位百分比
        t_end = samples[-1][0]
        stab = [abs(abs(s[1]) - target) for s in samples if s[0] >= t_end - t_end_off]
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
    if args.mode == "ramp":
        expect("CMD:POS_DIRECT_GAIN,%.4f,0.0300" % args.kp, "POS_DIRECT_GAIN,OK")
    send("CMD:COG_CFG,0.00,0.0")
    expect("CMD:FRIC_COMP,%.3f,%.3f" % (FRIC_FIX, FRIC_FIX), "FRIC_COMP,OK")
    expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    time.sleep(0.5)
    a0 = read_angle()
    if a0 is not None and (a0 < 25 or a0 > 335):
        ser.write(b"CMD:PREF,%.5f\n" % (100.0 * DEG2RAD))
        time.sleep(1.2)
        a0 = read_angle()

    rows = []
    try:
        if args.mode == "kd":
            for kd in kds:
                print("\n=== KD=%.2f (kp=%.2f) ===" % (kd, args.kp), flush=True)
                if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, kd),
                              "POS_DIRECT_GAIN,OK"):
                    print("  设置增益失败, 跳过")
                    continue
                a0 = read_angle()
                if a0 is None:
                    print("  读角失败, 跳过")
                    continue
                pos = step_and_track(a0, (a0 + args.step) * DEG2RAD)
                r_pos = summarize(pos, args.step)
                ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
                time.sleep(0.8)
                neg = step_and_track(a0, (a0 - args.step) * DEG2RAD)
                r_neg = summarize(neg, -args.step)
                ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
                time.sleep(0.8)
                if r_pos is None or r_neg is None:
                    print("  采样不足, 跳过")
                    continue
                row = {"kd": kd, "kp": args.kp, "pos": r_pos, "neg": r_neg}
                rows.append(row)
                print("  正向: 0.5s剩%.0f%% 1s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                      (r_pos["err_at_0p5s"], r_pos["err_at_1s"],
                       r_pos["steady_pp_deg"], r_pos["overshoot_deg"]), flush=True)
                print("  反向: 0.5s剩%.0f%% 1s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                      (r_neg["err_at_0p5s"], r_neg["err_at_1s"],
                       r_neg["steady_pp_deg"], r_neg["overshoot_deg"]), flush=True)
        else:  # ramp
            ramp_deg_s = args.ramp_deg_s
            a0 = read_angle()
            if a0 is None:
                print("读角失败")
                return 1
            print("\n=== 斜坡 %.2f°/s (kp=%.2f/kd=0.03) ===" % (ramp_deg_s, args.kp), flush=True)
            pos = ramp_and_track(a0, (a0 + args.step) * DEG2RAD, ramp_deg_s)
            r_pos = summarize(pos, args.step)
            a0_after = a0 + args.step
            neg = ramp_and_track(a0_after, a0 * DEG2RAD, ramp_deg_s)
            r_neg = summarize(neg, -args.step)
            if r_pos is not None and r_neg is not None:
                # 稀疏轨迹(每 0.25s 取一点)便于诊断粘滑
                def sparse(samples):
                    out = []
                    for i in range(int(samples[-1][0] * 4) + 1):
                        tgt = i * 0.25
                        best = min(samples, key=lambda s: abs(s[0] - tgt))
                        out.append([round(best[0], 2), round(best[1], 2)])
                    return out
                rows.append({"kd": 0.03, "kp": args.kp, "ramp_deg_s": ramp_deg_s,
                             "pos": r_pos, "neg": r_neg,
                             "pos_traj": sparse(pos), "neg_traj": sparse(neg)})
                print("  正向: 0.5s剩%.0f%% 1s剩%.0f%% 2s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                      (r_pos["err_at_0p5s"], r_pos["err_at_1s"], r_pos["err_at_2s"],
                       r_pos["steady_pp_deg"], r_pos["overshoot_deg"]), flush=True)
                print("  反向: 0.5s剩%.0f%% 1s剩%.0f%% 2s剩%.0f%% 稳态p-p=%.2f° 过冲%.2f°" %
                      (r_neg["err_at_0p5s"], r_neg["err_at_1s"], r_neg["err_at_2s"],
                       r_neg["steady_pp_deg"], r_neg["overshoot_deg"]), flush=True)
                print("  正向轨迹(位移°@时间s):", " ".join(
                    "%.1f°@%.1fs" % (d, t) for t, d in sparse(pos)), flush=True)
                print("  反向轨迹(位移°@时间s):", " ".join(
                    "%.1f°@%.1fs" % (d, t) for t, d in sparse(neg)), flush=True)
            else:
                print("  采样不足")
    finally:
        ser.write(b"CMD:STOP")
        ser.write(b"CMD:CLEAR_FAULT")
        ser.close()

    print("\n================ %s 汇总 ================" % ("KD扫描" if args.mode == "kd" else "斜坡命令"))
    if args.mode == "kd":
        print("%-6s %-14s %-14s %-12s %-10s" % ("kd", "pos 1s剩%", "neg 1s剩%", "稳态pp均值", "过冲均值"))
        for r in rows:
            pp_avg = (r["pos"]["steady_pp_deg"] + r["neg"]["steady_pp_deg"]) / 2
            os_avg = (r["pos"]["overshoot_deg"] + r["neg"]["overshoot_deg"]) / 2
            print("%-6.2f %-14.1f %-14.1f %-12.3f %-10.3f" %
                  (r["kd"], r["pos"]["err_at_1s"], r["neg"]["err_at_1s"], pp_avg, os_avg))
    else:
        for r in rows:
            pp_avg = (r["pos"]["steady_pp_deg"] + r["neg"]["steady_pp_deg"]) / 2
            os_avg = (r["pos"]["overshoot_deg"] + r["neg"]["overshoot_deg"]) / 2
            print("斜坡 %.2f°/s: 1s剩%.0f%%/%.0f%% 稳态pp均值=%.3f° 过冲均值=%.3f°" %
                  (r["ramp_deg_s"], r["pos"]["err_at_1s"], r["neg"]["err_at_1s"],
                   pp_avg, os_avg))

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "overshoot_%s_%s.json" % (args.mode, ts))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "rows": rows}, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
