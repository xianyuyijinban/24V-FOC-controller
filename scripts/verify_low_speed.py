#!/usr/bin/env python3
"""新固件低速摩擦验证：斜坡滞后 + 阶跃过冲 + 稳态 pp（带轨迹打印诊断）
DIRECT kp=0.5/kd=0.03, 摩擦补偿 0.12A, COG OFF, POS_DIRECT_KI 运行时设。
用法: python scripts/verify_low_speed.py --port COM10 --power-ok
"""
import argparse
import json
import os
import sys
import threading
import time

import serial

DEG2RAD = 3.14159265358979 / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--step", type=float, default=6.0)
    ap.add_argument("--ramp-deg-s", type=float, default=2.0)
    ap.add_argument("--ki", type=float, default=1.5)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0
    step = args.step
    ramp_s = args.ramp_deg_s

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    def expect(cmd, prefix, timeout=1.5):
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.strip().startswith(prefix):
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

    def track(target_rad, duration, a0, is_step):
        """单线程交替: 斜坡每0.2s发一步PREF(低密度写,避免并发饿死读) + 主线程读。
        同句柄并发write/read实测饿死读(斜坡期间0帧), 必须单线程交替。"""
        samples = []
        t0 = time.time()
        last_sent = 0.0
        if not is_step:
            dur = abs(target_rad - a0 * DEG2RAD) / (ramp_s * DEG2RAD)
        else:
            dur = 0.0
            ser.write(b"CMD:PREF,%.5f\n" % target_rad)
        dl = t0 + duration
        while time.time() < dl:
            if (not is_step) and (time.time() - last_sent >= 0.2):
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
                            if d > 180.0: d -= 360.0
                            elif d < -180.0: d += 360.0
                            samples.append((time.time() - t0, d))
            else:
                time.sleep(0.005)
        return samples

    ser.write(b"CMD:UNLOCK,1\n")
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,%.4f,0.0300" % 0.5, "POS_DIRECT_GAIN,OK")
    expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK")
    ser.write(b"CMD:COG_CFG,0.00,0.0\n")
    expect("CMD:FRIC_COMP,0.120,0.120", "FRIC_COMP,OK")
    expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    time.sleep(0.5)
    a0 = read_angle()
    print("start angle: %.2f" % (a0 if a0 else -1))
    if a0 is None:
        print("no angle")
        ser.close()
        return 1
    # 钉住当前位置: 清 pos_ref 遗留(ENABLE后位置环可能被旧 pos_ref 猛拉)
    ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
    time.sleep(0.6)
    a0 = read_angle()
    print("pinned at: %.2f" % (a0 if a0 else -1))
    if a0 is None:
        print("no angle after pin")
        ser.close()
        return 1
    if a0 < 25 or a0 > 335:
        ser.write(b"CMD:PREF,%.5f\n" % (100.0 * DEG2RAD))
        time.sleep(1.2)
        a0 = read_angle()
        print("moved to: %.2f" % (a0 if a0 else -1))

    results = {}

    def sp(samples, target_deg):
        if len(samples) < 5:
            return None
        target = abs(target_deg)
        err = {}
        for tm in (0.5, 1.0, 2.0):
            b = min(samples, key=lambda s: abs(s[0] - tm))
            frac = abs(b[1]) / target
            err[tm] = round((1.0 - min(frac, 1.0)) * 100.0, 1)
        t_end = samples[-1][0]
        stab = [abs(abs(s[1]) - target) for s in samples if s[0] >= t_end - 1.0]
        ovs = max((abs(abs(s[1]) - target) for s in samples), default=0.0)
        return {"err": err, "pp": max(stab) - min(stab) if stab else 0.0,
                "ovs": max(0.0, ovs), "n": len(samples)}

    def traj_str(samples, every=0.5):
        out = []
        for i in range(int(samples[-1][0] * (1.0 / every)) + 1):
            tgt = i * every
            b = min(samples, key=lambda s: abs(s[0] - tgt))
            out.append("%.1f°@%.1fs" % (b[1], b[0]))
        return " ".join(out)

    try:
        # 1) 斜坡 +6°
        print("\n=== 斜坡 +%.1f° @%.1f°/s ===" % (step, ramp_s), flush=True)
        pos = track((a0 + step) * DEG2RAD, 3.0 + 3.0, a0, is_step=False)
        r = sp(pos, step)
        print("  n=%d %s" % (len(pos), traj_str(pos)), flush=True)
        if r:
            print("  到位%% 0.5s=%s 1s=%s 2s=%s | pp=%.3f° 过冲=%.2f°" %
                  (r["err"][0.5], r["err"][1.0], r["err"][2.0], r["pp"], r["ovs"]), flush=True)
            results["ramp_pos"] = r
        else:
            print("  采样不足!")
        # 回起点
        ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
        time.sleep(1.5)

        # 2) 阶跃 +6°
        a1 = read_angle()
        print("\n=== 阶跃 +%.1f° ===" % step, flush=True)
        step_t = track((a1 + step) * DEG2RAD, 3.0, a1, is_step=True)
        r = sp(step_t, step)
        print("  n=%d %s" % (len(step_t), traj_str(step_t, 0.25)), flush=True)
        if r:
            print("  到位%% 0.5s=%s 1s=%s 2s=%s | pp=%.3f° 过冲=%.2f°" %
                  (r["err"][0.5], r["err"][1.0], r["err"][2.0], r["pp"], r["ovs"]), flush=True)
            results["step_pos"] = r
        else:
            print("  采样不足!")
        # 回起点并测稳态
        ser.write(b"CMD:PREF,%.5f\n" % (a1 * DEG2RAD))
        time.sleep(2.0)
        a2 = read_angle()
        print("\n=== 稳态(2s) @%.2f° ===" % (a2 if a2 else -1), flush=True)
        stab_t = track(a2 * DEG2RAD, 2.0, a2, is_step=True)
        if len(stab_t) >= 5:
            pp = max(s[1] for s in stab_t) - min(s[1] for s in stab_t)
            print("  n=%d 稳态角度抖动 pp=%.3f°" % (len(stab_t), pp), flush=True)
            results["steady_pp"] = round(pp, 3)
        else:
            print("  采样不足")
    finally:
        ser.write(b"CMD:STOP")
        ser.write(b"CMD:CLEAR_FAULT")
        ser.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "verify_lowspeed_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=2)
    print("\n报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
