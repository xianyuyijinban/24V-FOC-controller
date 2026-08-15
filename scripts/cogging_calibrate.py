#!/usr/bin/env python3
"""齿槽 LUT 标定 — 位置环直连低速匀速扫, 采集 Iq vs 角度, 提取齿槽力矩

原理: 位置环直连 + 摩擦补偿, 低速匀速扫一圈。稳态时位置环输出 ≈ 摩擦 + 齿槽抵抗。
摩擦为常数, Iq(θ) 的周期性波动 = 齿槽力矩。多圈平均 + FFT 提取主导谐波。

输出:
- 齿槽波形图数据(角度 bin vs 平均 Iq)
- FFT 主导谐波(次数/幅值)
- 生成 LUT 建议(neg 波形, 相位对齐后)

用法: python scripts/cogging_calibrate.py --port COM10 --power-ok [--deg-s 5.0 --sweep 360.0 --bins 264]
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
    ap.add_argument("--deg-s", type=float, default=5.0)
    ap.add_argument("--sweep", type=float, default=350.0, help="单圈扫动 deg(避开wrap)")
    ap.add_argument("--comp", type=float, default=0.12, help="摩擦补偿 A")
    ap.add_argument("--bins", type=int, default=264)
    ap.add_argument("--start-deg", type=float, default=100.0, help="起始角 deg")
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
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK")
    ser.write(b"CMD:COG_CFG,0.00,0.0\n")  # 关闭现有LUT
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
    # 挪到起始角
    ser.write(b"CMD:PREF,%.5f\n" % (args.start_deg * DEG2RAD))
    time.sleep(1.5)
    a0 = read_angle()
    print("start: %.2f°  speed=%.1f°/s sweep=%.0f° bins=%d" %
          (a0, args.deg_s, args.sweep, args.bins))

    dur = args.sweep / args.deg_s
    t0 = time.time()
    last_sent = 0.0
    samples = []  # (t, angle, iq_ref, iq)
    while time.time() - t0 < dur + 1.0:
        if time.time() - last_sent >= 0.2:
            frac = min((time.time() - t0) / dur, 1.0)
            cur = args.start_deg + args.sweep * frac
            ser.write(b"CMD:PREF,%.5f\n" % (cur * DEG2RAD))
            last_sent = time.time()
        if ser.in_waiting:
            for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        a = float(p[3])
                        iq_ref = float(p[19])
                        iq = float(p[6])
                        samples.append((time.time() - t0, a, iq_ref, iq))
        else:
            time.sleep(0.005)
    ser.write(b"CMD:STOP")
    ser.write(b"CMD:CLEAR_FAULT")
    ser.close()

    if len(samples) < 50:
        print("采样不足: %d" % len(samples))
        return 1

    # 只用匀速段(起始 0.5s 后)
    t_start = 0.5
    t_end = samples[-1][0] - 0.3
    seg = [s for s in samples if t_start <= s[0] <= t_end]
    if len(seg) < 30:
        seg = samples

    # 角度分 bin, 每 bin 平均 Iq_ref
    ang = [s[1] for s in seg]
    iqref = [s[2] for s in seg]
    bin_deg = 360.0 / args.bins
    bin_sum = [0.0] * args.bins
    bin_cnt = [0] * args.bins
    for a, i in zip(ang, iqref):
        b = int((a % 360.0) / bin_deg) % args.bins
        bin_sum[b] += i
        bin_cnt[b] += 1
    bin_avg = [bin_sum[b] / bin_cnt[b] if bin_cnt[b] else 0.0 for b in range(args.bins)]

    # 直流分量(摩擦) = 平均
    dc = sum(bin_avg) / len(bin_avg)
    cog = [dc - v for v in bin_avg]  # 齿槽力矩 = 直流 - 每bin均值 (负值=需前馈补偿)

    # FFT 主导谐波(齿槽 24N22P: 齿槽基频 24次/圈)
    n = args.bins
    dom = []
    for k in range(1, n // 2):
        re = sum(cog[i] * math.cos(2 * math.pi * k * i / n) for i in range(n))
        im = sum(cog[i] * math.sin(2 * math.pi * k * i / n) for i in range(n))
        amp = math.hypot(re, im) * 2.0 / n
        dom.append((k, amp))
    dom.sort(key=lambda x: -x[1])
    top = dom[:6]

    cog_pp = max(cog) - min(cog)

    print("\n===== 齿槽标定结果 =====")
    print("样本 %d, 覆盖角度 %.0f~%.0f°" % (len(seg), min(ang), max(ang)))
    print("直流(Iq均值=摩擦+粘滞): %.4fA" % dc)
    print("齿槽力矩波形: pp=%.4fA std=%.4fA" % (cog_pp, statistics.pstdev(cog)))
    print("主导谐波(次/圈, 幅值A):")
    for k, amp in top:
        marker = " <== 齿槽" if abs(k - 24) <= 1 or abs(k - 12) <= 1 or abs(k - 48) <= 1 else ""
        print("  %3d 次/圈  幅值 %.4fA%s" % (k, amp, marker))

    # 生成 LUT 建议(负齿槽力矩, 作为前馈补偿)
    lut = [-c for c in cog]  # 前馈 = 抵消齿槽
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "cogging_lut_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args),
                   "n": len(seg), "dc": dc,
                   "cog_pp": cog_pp, "cog_std": statistics.pstdev(cog),
                   "dom_harm": [{"k": k, "amp": round(amp, 5)} for k, amp in top],
                   "bins": args.bins,
                   "cog": [round(c, 5) for c in cog],
                   "lut": [round(x, 5) for x in lut]},
                  f, ensure_ascii=False, indent=2)
    print("\n报告: %s" % out)
    print("LUT 幅值范围: %.4f ~ %.4fA (写入需 gain 缩放)" % (min(lut), max(lut)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
