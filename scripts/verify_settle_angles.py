#!/usr/bin/env python3
"""verify_settle_angles.py — 稳态漂移角度触发扫描 (双态判据: ≥3°漂移态 vs ≤0.04°干净态)

2026-09-01: verify_lowspeed 稳态 pp 呈间歇双态 (6.31/0.04/6.24), 且 08-30 同日
0.02/3.53/0.03/0.03/5.62 也在切换。verify 脚本轮流钉住随机角度, 若漂移态是
位置触发 (齿槽/摩擦平衡点差异), 角度扫描应呈现相关性。

方法: 与 verify_low_speed 同配置 (kp=0.49/kd=0.007/ki=0.37, comp=0.022,
cog_gain=0, AW=1,0.03, DIRECT), 每个角度: PREF → 等5.5s 收敛 → 测 2s pp。
角度覆盖 240.69° (09-01 漂移点)。两轮过站测同角度内双态复现性。

用法: python scripts/verify_settle_angles.py --power-ok [--angles 30,75,120,...,320]
"""
import argparse
import json
import os
import sys
import time

import serial

DEG2RAD = 3.14159265358979 / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--angles", default="30,75,120,165,210,240,285,320")
    ap.add_argument("--settle", type=float, default=5.5, help="收敛等待 s")
    ap.add_argument("--window", type=float, default=2.0, help="测量窗口 s")
    ap.add_argument("--passes", type=int, default=2, help="每角度测量轮数")
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0
    angles = [float(a) for a in args.angles.split(",") if a.strip()]

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
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

    ser.write(b"CMD:UNLOCK,1\n")
    if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
        print("POS_DIRECT fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK"):
        print("POS_DIRECT_GAIN fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK"):
        print("POS_DIRECT_KI fail"); ser.close(); return 1
    ser.write(b"CMD:COG_CFG,0.000,60.0\n")
    if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
        print("FRIC_COMP fail"); ser.close(); return 1
    if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
        print("POS_AW_MODE fail"); ser.close(); return 1
    if not expect("CMD:MODE,2", "MODE,OK"):
        print("MODE fail"); ser.close(); return 1
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail"); ser.close(); return 1
    ser.write(b"CMD:ON\n")
    time.sleep(0.4)
    ser.reset_input_buffer()

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

    def measure(settle_s, win_s, a0_deg):
        """钉住 a0_deg 后等 settle_s, 测 win_s 绝对角 pp。返回 (pp, mean_angle, n)。
        2026-09-01 fix: settle 期间必须持续 drain (否则运动瞬态帧挤压在缓冲,
        测量窗开始后被 _drain() 一次性吞入 -> pp 虚高 30-45°, 误判 DRIFT)。"""
        ser.write(b"CMD:PREF,%.5f\n" % (a0_deg * DEG2RAD))
        # settle 期间持续排空 (丢弃瞬态)
        dl = time.time() + settle_s
        while time.time() < dl:
            lb._drain()
            time.sleep(0.002)
        # 确认缓冲清空 (残余瞬时帧清光, 确保测量窗只收集稳定后数据)
        time.sleep(0.2)
        lb._drain()
        t0 = time.time()
        angs = []
        dl = t0 + win_s
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        if int(p[8], 16) != 0:
                            raise RuntimeError("fault: %s" % p[8])
                        angs.append(float(p[3]))
            time.sleep(0.002)
        if len(angs) < 5:
            return None
        return (max(angs) - min(angs), sum(angs) / len(angs), len(angs))

    # 初始钉到第一点 (远离25/335边界)
    results = []
    try:
        for p in range(args.passes):
            # 每轮重新使能防中途漂移/锁死 (2026-09-01: 165° 后卡 119.6° 疑 PREF 锁)
            ser.write(b"CMD:OFF\n")
            time.sleep(0.4)
            ser.reset_input_buffer()
            lb.buf = ""
            if not expect("CMD:ENABLE,1", "ENABLE,OK"):
                print("pass %d ENABLE fail" % p, flush=True)
                break
            for a in angles:
                # 移动到目标 (隐式通过 PREF), 记录运动方向不影响稳态测
                r = measure(args.settle, args.window, a)
                if r is None:
                    print("  angle %7.1f pass %d: 采样不足" % (a, p), flush=True)
                    continue
                pp, mean_a, n = r
                cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.04 else "MID")
                print("  angle %7.1f pass %d: pp=%6.3f mean=%7.2f n=%3d -> %s"
                      % (a, p, pp, mean_a, n, cls), flush=True)
                results.append({"angle": a, "pass": p, "pp": round(pp, 4),
                                "mean": round(mean_a, 3), "n": n, "cls": cls})
    finally:
        ser.write(b"CMD:STOP")
        ser.write(b"CMD:OFF")
        ser.write(b"CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "settle_angles_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    drift = [r for r in results if r["cls"] == "DRIFT"]
    clean = [r for r in results if r["cls"] == "CLEAN"]
    print("汇总: %d 点 %d 漂移态 %d 干净态 %d MID" %
          (len(results), len(drift), len(clean),
           len(results) - len(drift) - len(clean)))
    if drift or clean:
        print("漂移态角度:", sorted(set(round(r["angle"], 1) for r in drift)))
        print("干净态角度:", sorted(set(round(r["angle"], 1) for r in clean)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
