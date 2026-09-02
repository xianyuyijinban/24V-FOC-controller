#!/usr/bin/env python3
"""低速速度观测器(线性ESO)台架验证

核心前提验证: 静止时差分速度噪声 std ~0.5°/s(15bit量化), 观测器 omega_hat
应把静止噪声压到接近 0 —— 这是 0.1°/s 可辨的前提。然后验证匀速平滑度
与 ω0 带宽权衡。

用法:
  python scripts/observer_test.py --port COM10 --power-ok            # 静止噪声+ω0扫
  python scripts/observer_test.py --port COM10 --power-ok --mode ramp # 0.5°/s匀速对比
  python scripts/observer_test.py --port COM10 --power-ok --mode step --use-d 1 # D项观测器+kd阶跃
"""
import argparse
import math
import os
import statistics
import sys
import time

import serial

DEG2RAD = 3.14159265358979 / 180.0
OBS_W0_CANDIDATES = [8.0, 12.0, 15.0, 20.0, 30.0]


def parse_obs(resp):
    """OBS,OK,w0=..,use_d=..,valid=..,omega_hat=..,diff=..,T_hat=.."""
    out = {}
    for seg in resp.split(","):
        if "=" in seg:
            parts = seg.split("=", 1)
            if len(parts) == 2 and parts[1].strip():
                try:
                    out[parts[0].strip()] = float(parts[1])
                except ValueError:
                    pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--mode", default="idle_noise", choices=["idle_noise", "ramp", "step", "sweep_w0"])
    ap.add_argument("--w0", type=float, default=12.0, help="观测器带宽 rad/s")
    ap.add_argument("--use-d", type=int, default=0, help="1=POS_DIRECT D项用观测器速度")
    ap.add_argument("--kd", type=float, default=0.03, help="阶跃模式 D项增益")
    ap.add_argument("--deg-s", type=float, default=0.5, help="匀速速度 deg/s")
    ap.add_argument("--samples", type=int, default=200, help="静止噪声采样数")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    # 关高频电流流(C, 1kHz, 主要干扰源), 保留周期遥测(N, 供 read_angle 取角度)
    ser.write(b"TELEM:CUR,OFF\n")
    time.sleep(0.2)
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

    def read_obs():
        """发 CMD:OBS? 读一条, 返回 parse_obs 或 None"""
        ser.reset_input_buffer()
        ser.write(b"CMD:OBS?\n")
        dl = time.time() + 0.5
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.startswith("OBS,OK"):
                        return parse_obs(l.strip())
            else:
                time.sleep(0.002)
        return None

    def read_angle():
        dl = time.time() + 0.5
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

    # 使能流程
    expect("CMD:UNLOCK,1", "UNLOCK,OK")
    time.sleep(0.2)
    expect("CMD:MODE,2", "MODE,OK")  # POSITION
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,2.0,0.03", "POS_DIRECT_GAIN,OK")
    expect("CMD:POS_DIRECT_KI,0", "POS_DIRECT_KI,OK")
    expect("CMD:OBS_CFG,%.1f,%d" % (args.w0, args.use_d), "OBS_CFG,OK")
    if args.use_d:
        expect("CMD:POS_DIRECT_GAIN,2.0,%.3f" % args.kd, "POS_DIRECT_GAIN,OK")
    expect("CMD:ENABLE,1", "ENABLE,OK")
    time.sleep(1.0)

    if args.mode == "sweep_w0":
        # ω0 扫参: 静止噪声 vs 带宽 (无需 PREF)
        time.sleep(0.5)
        print("w0    omega_hat_std(/s)  diff_std(/s)  T_hat(N·m)")
        for w0 in OBS_W0_CANDIDATES:
            expect("CMD:OBS_CFG,%.1f,%d" % (w0, args.use_d), "OBS_CFG,OK")
            time.sleep(0.6)  # 观测器收敛
            obs_s, diff_s = [], []
            for _ in range(args.samples):
                o = read_obs()
                if o:
                    obs_s.append(o.get("omega_hat", 0.0))
                    diff_s.append(o.get("diff", 0.0))
                time.sleep(0.004)
            if obs_s:
                std_obs = statistics.pstdev(obs_s) * (180.0 / math.pi)
                std_diff = statistics.pstdev(diff_s) * (180.0 / math.pi)
                t_hat = obs_s[-1] * 0.0 + (read_obs() or {}).get("T_hat", 0.0)
                print("%-6.1f %-18.4f %-14.4f %.6f" % (w0, std_obs, std_diff, t_hat))
        return 0

    if args.mode == "idle_noise":
        # 使能后位置模式自动保持当前位置, 无需 PREF
        time.sleep(1.0)  # 观测器收敛
        obs_s, diff_s, t_s = [], [], []
        for _ in range(args.samples):
            o = read_obs()
            if o:
                obs_s.append(o.get("omega_hat", 0.0))
                diff_s.append(o.get("diff", 0.0))
                t_s.append(o.get("T_hat", 0.0))
            time.sleep(0.004)
        if not obs_s:
            print("ERR: no OBS samples"); return 1
        def deg(x): return x * (180.0 / math.pi)
        print("=== 静止速度噪声 (w0=%.1f, use_d=%d, n=%d) ===" % (args.w0, args.use_d, len(obs_s)))
        print("  观测器 omega_hat: mean %+.4f /s  std %.4f /s  pp %.4f /s" %
              (deg(statistics.fmean(obs_s)), deg(statistics.pstdev(obs_s)),
               deg(max(obs_s) - min(obs_s))))
        print("  差分  diff:       mean %+.4f /s  std %.4f /s  pp %.4f /s" %
              (deg(statistics.fmean(diff_s)), deg(statistics.pstdev(diff_s)),
               deg(max(diff_s) - min(diff_s))))
        print("  T_hat: mean %.6f N·m  (摩擦保持力矩收敛)" % statistics.fmean(t_s))
        return 0

    if args.mode == "ramp":
        # 0.5°/s 匀速 (MOTION_CFG 斜坡 + PREF 阶跃, 不依赖 read_angle):
        # 对比观测器 omega_hat vs 差分 diff 在匀速时的平滑度
        deg_s = args.deg_s
        target_rad = 0.20  # ~11.5°, 匀速段时间 = 0.20/(deg_s*DEG2RAD)
        dur = target_rad / (deg_s * DEG2RAD) + 2.0
        expect("CMD:POS_DIRECT,0", "POS_DIRECT,OK")  # 级联: MOTION 斜坡给速度指令, 速度环匀速
        expect("CMD:MOTION_CFG,%.4f,0.20,%.4f" % (deg_s * DEG2RAD, deg_s * DEG2RAD),
               "MOTION_CFG,OK")
        expect("CMD:PREF,%.6f" % target_rad, "PREF,OK")
        obs_s, diff_s = [], []
        t0 = time.time()
        while time.time() - t0 < min(dur, 20.0):
            o = read_obs()
            if o:
                obs_s.append(o.get("omega_hat", 0.0))
                diff_s.append(o.get("diff", 0.0))
            time.sleep(0.02)
        def deg(x): return x * (180.0 / math.pi)
        if not obs_s:
            print("ERR: no samples"); return 1
        print("=== %.2f°/s 匀速平滑度对比 (n=%d) ===" % (deg_s, len(obs_s)))
        print("  观测器 omega_hat: mean %+.3f /s  std %.4f /s" %
              (deg(statistics.fmean(obs_s)), deg(statistics.pstdev(obs_s))))
        print("  差分  diff:       mean %+.3f /s  std %.4f /s" %
              (deg(statistics.fmean(diff_s)), deg(statistics.pstdev(diff_s))))
        return 0

    if args.mode == "step":
        # 阶跃: 无use-d vs 有use-d 对比过冲(观测器D项允许更高kd)
        pos0 = read_angle()
        if pos0 is None:
            print("ERR: no telemetry"); return 1
        for label, w0, ud, kd in [
            ("diff-d   kd=0.03", 12.0, 0, 0.03),
            ("obs-d    kd=0.10", 12.0, 1, 0.10),
            ("obs-d    kd=0.20", 12.0, 1, 0.20),
        ]:
            expect("CMD:OBS_CFG,%.1f,%d" % (w0, ud), "OBS_CFG,OK")
            expect("CMD:POS_DIRECT_GAIN,2.0,%.3f" % kd, "POS_DIRECT_GAIN,OK")
            expect("CMD:PREF,%.6f" % pos0, "PREF,OK")
            time.sleep(1.0)
            peak = 0.0
            target = pos0 + 6.0 * DEG2RAD
            expect("CMD:PREF,%.6f" % target, "PREF,OK")
            t0 = time.time()
            while time.time() - t0 < 2.0:
                a = read_angle()
                if a is not None:
                    over = a - target
                    if abs(over) > peak:
                        peak = abs(over)
                time.sleep(0.01)
            print("%-18s 6°阶跃峰值过冲 %+.2f°" % (label, peak * (180.0 / math.pi)))
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
