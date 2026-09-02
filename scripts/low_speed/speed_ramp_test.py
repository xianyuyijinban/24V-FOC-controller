#!/usr/bin/env python3
"""
匀速位置斜坡测量 — 模拟云台小幅扫动, 量化 DIRECT 结构匀速段平滑性 + 采集齿槽纹波

位置环是最短路径语义, PREF 无法表达多圈匀速。因此每档限制在单圈扫动
(幅度 SWEEP_RAD, 不跨 0/360 边界), 后台线程 50Hz 流式发 PREF 模拟平滑斜坡,
主线程采 N 帧(angle/speed/iq)。分析: 瞬时速度(angle差分)、Iq纹波、频谱找齿槽基频。

齿槽基频 = 264 * omega / 2pi Hz (24N22P, LCM=264)。
用法: python scripts/speed_ramp_test.py --port COM10 --power-ok [--speeds 0.02,0.05,0.1]
"""
import argparse
import cmath
import json
import math
import os
import statistics
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
from common.pos_direct_ab_test import N_IDX

DEG2RAD = 3.14159265358979 / 180.0
SWEEP_RAD = 1.0  # 每档扫动幅度 rad (~57°), 单圈内
BOUNDARY_DEG = 30.0  # 距 0/360 边界的安全距离


def analyze(samples, speed_target):
    """samples: list of (t, angle_deg, speed_field, iq_A, appFault). 返回统计 dict。"""
    if len(samples) < 10:
        return None
    ts = [s[0] for s in samples]
    angs = [s[1] for s in samples]
    speeds_field = [s[2] for s in samples]
    iqs = [s[3] for s in samples]
    faults = [s[4] for s in samples]

    # 瞬时速度: angle 差分(度/s -> rad/s); 单圈内无 wrap, 但兼容边界跨越
    inst = []
    for i in range(1, len(angs)):
        dt = ts[i] - ts[i - 1]
        d = angs[i] - angs[i - 1]
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        if dt > 0:
            inst.append(d * DEG2RAD / dt)
    if len(inst) < 8:
        return None
    n = len(inst)
    lo, hi = int(n * 0.25), int(n * 0.92)  # 丢起止加减速段
    inst_stable = inst[lo:hi]
    speeds_stable = speeds_field[lo:hi + 1]
    iq_stable = iqs[lo:hi + 1]

    # 剔除离群(角度跳变导致的假速度): 中位数±5σ
    med = statistics.median(inst_stable)
    sd = statistics.pstdev(inst_stable) if len(inst_stable) > 1 else 1.0
    inst_clean = [x for x in inst_stable if abs(x - med) < 5.0 * max(sd, 1e-3)]

    # FFT 找速度脉动主频
    fft_peaks = []
    if len(inst_clean) > 16:
        Nf = len(inst_clean)
        dt_avg = statistics.mean([ts[i] - ts[i - 1] for i in range(1, len(ts))]) or 0.02
        win = list(inst_clean[:Nf])
        wm = statistics.mean(win)
        win = [w - wm for w in win]
        fs = 1.0 / dt_avg
        try:
            fft = []
            for j in range(1, Nf // 2):
                mag = abs(sum(win[k] * cmath.exp(-2j * math.pi * k * j / Nf) for k in range(Nf)))
                fft.append((j * fs / Nf, mag))
            fft_peaks = sorted(fft, key=lambda x: -x[1])
        except Exception:
            fft_peaks = []

    cog_freq = 264.0 * abs(speed_target) / (2.0 * math.pi)
    cog_mag = 0.0
    for f, m in fft_peaks[:5]:
        if abs(f - cog_freq) < 0.5 * cog_freq or cog_freq < 0.5:
            cog_mag = m
            break
    peak = fft_peaks[0] if fft_peaks else (0.0, 0.0)

    return {
        "n": len(samples),
        "speed_target": speed_target,
        "inst_speed_mean_radps": statistics.mean(inst_clean),
        "inst_speed_pp_radps": max(inst_clean) - min(inst_clean),
        "inst_speed_std_radps": statistics.pstdev(inst_clean),
        "speed_field_pp_radps": max(speeds_stable) - min(speeds_stable),
        "iq_pp_A": max(iq_stable) - min(iq_stable),
        "iq_std_A": statistics.pstdev(iq_stable),
        "cog_freq_hz": cog_freq,
        "cog_peak_mag": cog_mag,
        "fft_peak_hz": peak[0],
        "max_fault": max(faults),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--speeds", default="0.02,0.05,0.1", help="匀速档位 rad/s")
    ap.add_argument("--dur", type=float, default=8.0, help="每档采样时长 s")
    ap.add_argument("--kp", type=float, default=0.5)
    ap.add_argument("--kd", type=float, default=0.03)
    ap.add_argument("--cog", type=float, default=0.0, help="COG gain: 0=OFF 0.25=ON")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok")
        return 0
    speeds = [float(x) for x in args.speeds.split(",")]

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

    def read_angle(timeout=2.0):
        dl = time.time() + timeout
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

    send("CMD:UNLOCK,1")
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK")
    send("CMD:COG_CFG,%.2f,0.0" % args.cog)
    expect("CMD:MODE,2", "MODE,OK")

    results = []
    try:
        if not expect("CMD:ENABLE,1", "ENABLE,OK"):
            print("ENABLE fail")
            return 1
        time.sleep(0.5)
        # 挪到安全区(远离 0/360 边界)
        a0 = read_angle()
        if a0 is None:
            print("can't read start angle")
            return 1
        sweep_deg = SWEEP_RAD * 180.0 / math.pi
        if a0 < BOUNDARY_DEG or a0 > 360.0 - BOUNDARY_DEG:
            safe = 90.0
            ser.write(b"CMD:PREF,%.5f\n" % (safe * DEG2RAD))
            time.sleep(1.2)
            a0 = read_angle()
            print("挪到安全区: 起点 %.1f deg" % (a0 or safe))

        for sp in speeds:
            print("\n=== 匀速 %.2f rad/s (%.1f deg/s) 齿槽基频 %.2f Hz ===" %
                  (sp, sp * 180 / math.pi, 264 * sp / (2 * math.pi)), flush=True)
            start_angle = a0 + sp * 1.0 * 180.0 / math.pi  # 预跑1s越过静摩擦
            # 校验扫动不跨边界
            travel = sp * args.dur * 180.0 / math.pi
            end_angle = start_angle + travel
            if end_angle > 360.0 - BOUNDARY_DEG or start_angle < BOUNDARY_DEG:
                print("  扫动跨边界, 跳过该档")
                continue
            ser.reset_input_buffer()
            samples = []
            t0 = time.time()
            stop = threading.Event()

            def pref_thread():
                ideal_next = start_angle
                cmd_deadline = 0.0
                while not stop.is_set():
                    now = time.time()
                    if now >= cmd_deadline:
                        dt = now - (cmd_deadline if cmd_deadline > 0 else now)
                        ideal_next += sp * 180.0 / math.pi * dt
                        ser.write(b"CMD:PREF,%.5f\n" % (ideal_next * DEG2RAD))
                        cmd_deadline = now + 0.02
                    time.sleep(0.001)

            th = threading.Thread(target=pref_thread, daemon=True)
            th.start()
            time.sleep(0.2)
            while time.time() - t0 < args.dur + 1.2:
                if ser.in_waiting:
                    for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                        if l.startswith("N,"):
                            p = l.split(",")
                            if len(p) >= 25:
                                samples.append((time.time(), float(p[3]), float(p[4]),
                                                float(p[19]), int(p[14])))
                else:
                    time.sleep(0.002)
            stop.set()
            th.join(timeout=1.0)
            ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
            time.sleep(0.8)
            r = analyze(samples, sp)
            if r is None:
                print("  数据不足 (n=%d)" % len(samples))
                continue
            results.append(r)
            print("  实测平均速度 %.3f rad/s  速度脉动 pp=%.4f std=%.4f (n=%d)" %
                  (r["inst_speed_mean_radps"], r["inst_speed_pp_radps"],
                   r["inst_speed_std_radps"], r["n"]))
            print("  Iq pp=%.4fA std=%.4f | FFT峰值 %.2fHz (齿槽%.2fHz, 幅%.4f) | speed字段pp %.4f" %
                  (r["iq_pp_A"], r["iq_std_A"], r["fft_peak_hz"],
                   r["cog_freq_hz"], r["cog_peak_mag"], r["speed_field_pp_radps"]))
            if r["max_fault"]:
                print("  !! AppFault=%d" % r["max_fault"])
    finally:
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        send("CMD:COG_CFG,0.25,60.0")  # 恢复默认
        ser.close()

    print("\n================ 匀速测量汇总 ================")
    for r in results:
        print("  速度 %.2f: 实测 %.3f 脉动pp %.4f std %.4f | Iq pp %.4f | FFT峰 %.2fHz(齿槽%.2f) | fault %d" %
              (r["speed_target"], r["inst_speed_mean_radps"], r["inst_speed_pp_radps"],
               r["inst_speed_std_radps"], r["iq_pp_A"], r["fft_peak_hz"],
               r["cog_freq_hz"], r["max_fault"]))
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "speed_ramp_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
