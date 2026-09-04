#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s2_ab_verify.py — S2 电压闭环 vs 电流模式同工况 A/B (V3 门入口, 2026-09-03)

三组工况 × 两模式 (电流模式定版 / 电压模式 S2):
  1. 0.5°/s 斜坡 (6° 目标): 跟踪率 + err_std
  2. 6° 阶跃: 到位时间 (2.0s 判据) + 稳态 pp
  3. 稳态保持: 回位+5s 后 2s 窗 pp

电压模式 S2 链: CMD:MODE,3 + CMD:POS_DIRECT,1 + POS_DIRECT_GAIN (电压口径 kp=1.078/kd=0.0154)
  + POS_DIRECT_KI (0.814) — 注意: POS_DIRECT_GAIN/KI 是电流口径命令, 电压末级内换算,
  所以命令值传电流口径 (0.49/0.007/0.37), 推导在 foc_app.c 1488 末级 ×Rs_phase×0.5。

电流模式链: CMD:MODE,2 + CMD:POS_DIRECT,1 同样配置 (对照。

用法: python scripts/low_speed/s2_ab_verify.py COM10 --power-ok
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
    ap.add_argument("--step", type=float, default=6.0)
    ap.add_argument("--ramp-deg-s", type=float, default=0.5)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--win", type=float, default=2.0)
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

    # JDIAG 审计
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
    if jline:
        print("JDIAG: %s" % jline[:120])
    else:
        print("JDIAG 无响应 — 中止"); ser.close(); return 1

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

    def track(target_rad, duration, a0, is_step):
        """单线程交替 PREF 写入 + N 帧读 (斜坡 0.2s 一步; 阶跃一步)。返回 [(t, angle_deg), ...]"""
        samples = []
        t0 = time.time()
        last_sent = 0.0
        if not is_step:
            dur = abs(target_rad - a0 * DEG2RAD) / (args.ramp_deg_s * DEG2RAD)
        else:
            dur = 0.0
            ser.write(b"CMD:PREF,%.5f\n" % target_rad)
        dl = t0 + duration
        while time.time() < dl:
            if not is_step and (time.time() - last_sent >= 0.2):
                frac = min((time.time() - t0) / dur, 1.0) if dur > 0 else 1.0
                cur = a0 * DEG2RAD + (target_rad - a0 * DEG2RAD) * frac
                ser.write(b"CMD:PREF,%.5f\n" % cur)
                last_sent = time.time()
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        if int(p[8], 16) != 0:
                            raise RuntimeError("fault: %s" % p[8])
                        a = float(p[3])
                        d = a - a0
                        if d > 180.0: d -= 360.0
                        elif d < -180.0: d += 360.0
                        samples.append((time.time() - t0, d))
            time.sleep(0.002)
        return samples

    def run_mode(mode_label, mode_cmd, kp, kd, ki):
        """单模式完整 A/B: 配置 → 钉位 → 斜坡 → 阶跃 → 稳态
        电压 S2 传半增益 (kp/2) — 固件末级只 ×Rs_phase (无×0.5), 半值=保守系数落地。"""
        print("\n===== 模式: %s =====" % mode_label, flush=True)
        # 配置链
        send("CMD:UNLOCK,1", 0.15)
        ser.reset_input_buffer()
        if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
            print("POS_DIRECT fail"); return None
        if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (kp, kd), "POS_DIRECT_GAIN,OK"):
            print("GAIN fail"); return None
        if not expect("CMD:POS_DIRECT_KI,%.2f" % ki, "POS_DIRECT_KI,OK"):
            print("KI fail"); return None
        send("CMD:COG_CFG,0.0,60.0", 0.15)
        if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
            print("FRIC fail"); return None
        if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
            print("AW fail"); return None
        if not expect(mode_cmd, "MODE,OK"):
            print("MODE fail", mode_cmd); return None
        time.sleep(0.3)
        en_ok = False
        for attempt in range(3):
            if expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
                en_ok = True
                break
            send("CMD:CLEAR_FAULT", 0.8)
        if not en_ok:
            print("ENABLE fail"); return None
        ser.write(b"CMD:ON\n")
        time.sleep(0.4)
        ser.reset_input_buffer()
        time.sleep(0.5)
        a0 = read_angle()
        if a0 is None:
            print("no angle"); return None
        print("start angle: %.2f" % a0, flush=True)
        # 钉住
        ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
        time.sleep(0.6)
        a0 = read_angle()
        if a0 is None:
            return None
        res = {"mode": mode_label}
        # 1) 斜坡 0.5°/s (6° 目标)
        ramp_dur = args.step / args.ramp_deg_s  # 12s
        ramp_samples = track((a0 + args.step) * DEG2RAD, ramp_dur + 3.0, a0, is_step=False)
        if len(ramp_samples) >= 5:
            # 跟踪率: 末段误差; err_std 保持段
            ramp_end = ramp_samples[-1][1]
            track_rate = abs(ramp_end) / args.step * 100
            # err_std: 斜坡后 2s 窗
            tail = [s[1] for s in ramp_samples if s[0] >= ramp_samples[-1][0] - 2.0]
            import statistics
            err_std = statistics.pstdev(tail) if tail else 0
            print("  斜坡: 到位=%.1f%% track_rate=%.1f%% err_std=%.3f°" %
                  (track_rate, track_rate, err_std), flush=True)
            res["ramp"] = {"track_pct": round(track_rate, 1), "err_std": round(err_std, 4)}
        # 回起点
        ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
        time.sleep(1.5)
        # 2) 阶跃 6°
        a1 = read_angle()
        if a1 is None:
            return None
        step_samples = track((a1 + args.step) * DEG2RAD, 3.0, a1, is_step=True)
        if len(step_samples) >= 5:
            # 到位时间: 首达 >95% 的时间
            t_arr = next((s[0] for s in step_samples if abs(s[1]) >= args.step * 0.95), None)
            # 稳态 pp: 2s 后窗
            tail = [s[1] for s in step_samples if s[0] >= 1.0]
            pp = (max(tail) - min(tail)) if tail else 0
            overshoot = max(abs(s[1]) for s in step_samples) / args.step * 100
            print("  阶跃: t95=%.2fs pp=%.3f° 过冲=%.1f%%" %
                  (t_arr if t_arr else -1, pp, overshoot), flush=True)
            res["step"] = {"t95_s": round(t_arr, 3) if t_arr else None,
                           "pp": round(pp, 4), "overshoot_pct": round(overshoot, 1)}
        # 3) 稳态保持: 回 a1 + 5s 后 2s 窗
        ser.write(b"CMD:PREF,%.5f\n" % (a1 * DEG2RAD))
        time.sleep(args.settle)
        a2 = read_angle()
        if a2 is None:
            return None
        stab_t = track(a2 * DEG2RAD, args.win, a2, is_step=True)
        if len(stab_t) >= 5:
            pp = max(s[1] for s in stab_t) - min(s[1] for s in stab_t)
            print("  稳态: pp=%.3f°" % pp, flush=True)
            res["steady_pp"] = round(pp, 4)
        return res

    results = []
    try:
        # 电流模式定版 (MODE,2) — 全增益
        r1 = run_mode("current_pid", "CMD:MODE,2", args.kp, args.kd, args.ki)
        if r1:
            results.append(r1)
        # 电压 S2 (MODE,3 + POS_DIRECT) — 半增益 (保守系数 0.5, 固件末级 ×Rs_phase)
        r2 = run_mode("voltage_s2", "CMD:MODE,3",
                      args.kp * 0.5, args.kd * 0.5, args.ki * 0.5)
        if r2:
            results.append(r2)
    finally:
        ser.write(b"CMD:VOLT_OFF")
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "s2_ab_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    print("=== A/B 对照 ===")
    for r in results:
        print("  %s: ramp=%s step=%s steady_pp=%s" %
              (r["mode"], r.get("ramp"), r.get("step"), r.get("steady_pp")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
