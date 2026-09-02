#!/usr/bin/env python3
"""低速匀速纹波基线测量 — 量化低速平滑性(贴近商用云台的核心指标)

单线程斜坡 PREF 匀速扫一段, 采 N 帧角度/Iq/速度, 分析:
- 角度线性残差 pp°(匀速时电机应走直线, 齿槽/摩擦导致偏离)
- 速度纹波 pp/std (差分)
- Iq 纹波(匀速力矩恒定, 周期性波动=齿槽)
- FFT 主导谐波(窗口够时)

用法: python scripts/speed_ripple_measure.py --port COM10 --power-ok [--deg-s 2.0 --sweep 40.0]
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import serial

DEG2RAD = 3.14159265358979 / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--kp", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=0.03)
    ap.add_argument("--ki", type=float, default=-1.0, help="位置环积分增益(-1=不动用固件默认)")
    ap.add_argument("--deg-s", type=float, default=2.0, help="匀速速度 deg/s")
    ap.add_argument("--sweep", type=float, default=40.0, help="扫动幅度 deg")
    ap.add_argument("--comp", type=float, default=0.12, help="摩擦补偿 A")
    ap.add_argument("--direct", type=int, default=1, help="1=直连 0=级联")
    ap.add_argument("--cog-gain", type=float, default=0.0, help="齿槽 LUT 增益(0=关 1=满)")
    ap.add_argument("--cog-phase", type=float, default=0.0, help="齿槽 LUT 相位 deg")
    ap.add_argument("--dir", type=int, default=1, help="1=正向 -1=反向")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0

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

    ser.write(b"CMD:UNLOCK,1\n")
    time.sleep(0.2)
    expect("CMD:POS_DIRECT,%d" % args.direct, "POS_DIRECT,OK")
    if args.direct:
        expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK")
    if args.ki >= 0.0:
        expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK")
    ser.write(b"CMD:COG_CFG,%.2f,%.1f\n" % (args.cog_gain, args.cog_phase))
    expect("CMD:FRIC_COMP,%.3f,%.3f" % (args.comp, args.comp), "FRIC_COMP,OK")
    expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    time.sleep(0.5)
    a0 = read_angle()
    if a0 is None:
        print("no angle")
        ser.close()
        return 1
    # 钉住当前位置清 pos_ref 遗留
    ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
    time.sleep(0.6)
    a0 = read_angle()
    # 挪到安全区中部, 保证扫动不碰 wrap 边界
    SAFE = 160.0
    if a0 < 40 or a0 > 320:
        ser.write(b"CMD:PREF,%.5f\n" % (SAFE * DEG2RAD))
        time.sleep(1.5)
        a0 = read_angle()
    print("start: %.2f°  speed=%.1f°/s sweep=%.1f°" % (a0, args.deg_s, args.sweep))

    target_end = a0 + args.dir * args.sweep
    dur = args.sweep / args.deg_s
    t0 = time.time()
    last_sent = 0.0
    samples = []  # (t, angle)
    while time.time() - t0 < dur + 1.0:
        if time.time() - last_sent >= 0.2:
            frac = min((time.time() - t0) / dur, 1.0)
            cur = a0 + args.dir * args.sweep * frac
            ser.write(b"CMD:PREF,%.5f\n" % (cur * DEG2RAD))
            last_sent = time.time()
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        a = float(p[3])
                        iq = float(p[6])
                        samples.append((time.time() - t0, a, iq))
        else:
            time.sleep(0.005)
    ser.write(b"CMD:STOP")
    ser.write(b"CMD:CLEAR_FAULT")
    ser.close()

    if len(samples) < 20:
        print("采样不足: %d" % len(samples))
        return 1

    # 只用匀速段(排除起始加速): 从斜坡开始 0.5s 后到结束前
    t_start = 0.5
    t_end = samples[-1][0] - 0.3
    seg = [s for s in samples if t_start <= s[0] <= t_end]
    if len(seg) < 15:
        seg = samples
    t = [s[0] - seg[0][0] for s in seg]
    ang = [s[1] for s in seg]
    iq = [s[2] for s in seg]

    # 角度线性残差: 理想直线 = 起点 + dir*deg_s*t
    ideal = [ang[0] + args.dir * args.deg_s * ti for ti in t]
    resid = [ang[i] - ideal[i] for i in range(len(ang))]
    resid_pp = max(resid) - min(resid)
    resid_std = statistics.pstdev(resid)

    # 速度纹波(角度差分)
    vel = []
    for i in range(1, len(ang)):
        dt = t[i] - t[i - 1]
        if dt > 0.001:
            v = (ang[i] - ang[i - 1]) / dt
            if abs(v) < 5 * abs(args.deg_s):  # 去 outlier
                vel.append(v)
    vel_pp = (max(vel) - min(vel)) if vel else 0.0
    vel_std = statistics.pstdev(vel) if vel else 0.0

    # Iq 纹波
    iq_pp = max(iq) - min(iq)
    iq_std = statistics.pstdev(iq)

    # FFT 主导谐波(对角度残差)
    dom_harm = []
    if len(resid) >= 32:
        n = len(resid)
        dt_avg = sum(t[i] - t[i - 1] for i in range(1, n)) / (n - 1)
        fs = 1.0 / dt_avg if dt_avg > 0 else 50.0
        # 去均值
        x = [r - sum(resid) / n for r in resid]
        # 简单 DFT 扫 0.1~5 Hz
        freqs = {}
        for k in range(1, int(min(fs / 2, 20.0))):
            f = k * fs / n
            if f < 0.05 or f > 10.0:
                continue
            re = sum(x[i] * math.cos(2 * math.pi * k * i / n) for i in range(n))
            im = sum(x[i] * math.sin(2 * math.pi * k * i / n) for i in range(n))
            amp = math.hypot(re, im) * 2.0 / n
            freqs[f] = amp
        top = sorted(freqs.items(), key=lambda kv: -kv[1])[:5]
        dom_harm = [{"f_hz": round(f, 3), "amp_deg": round(a, 3)} for f, a in top]

    print("\n===== 匀速纹波 (%.1f°/s, %.0f°) =====" % (args.deg_s, args.sweep))
    print("样本: %d (%.2f~%.2fs), 平均采样 %.0f Hz" % (len(seg), t[0], t[-1], len(seg) / t[-1]))
    print("角度线性残差: pp=%.3f° std=%.3f°" % (resid_pp, resid_std))
    print("速度纹波: pp=%.4f°/s std=%.4f°/s (均值 %.2f°/s)" %
          (vel_pp, vel_std, sum(vel) / len(vel)))
    print("Iq纹波: pp=%.4fA std=%.4fA" % (iq_pp, iq_std))
    if dom_harm:
        print("主导谐波: " + ", ".join("%.3fHz/%.3f°" % (h["f_hz"], h["amp_deg"]) for h in dom_harm))

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "ripple_%s%s_%s.json" % ("pos" if args.dir > 0 else "neg",
                                                 args.deg_s, ts))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args),
                   "n": len(seg),
                   "resid_pp": resid_pp, "resid_std": resid_std,
                   "vel_pp": vel_pp, "vel_std": vel_std, "vel_mean": sum(vel) / len(vel),
                   "iq_pp": iq_pp, "iq_std": iq_std,
                   "dom_harm": dom_harm,
                   "t": [round(x, 3) for x in t],
                   "angle": [round(x, 4) for x in ang],
                   "iq": [round(x, 4) for x in iq]},
                  f, ensure_ascii=False, indent=2)
    print("\n报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
