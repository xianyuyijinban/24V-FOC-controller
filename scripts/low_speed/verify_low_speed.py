#!/usr/bin/env python3
"""新固件低速摩擦验证：斜坡滞后 + 阶跃过冲 + 稳态 pp（带轨迹打印诊断）
DIRECT kp=0.49/kd=0.007, 摩擦补偿 --comp, COG --cog-gain, POS_DIRECT_KI 运行时设。
用法: python scripts/verify_low_speed.py --port COM10 --power-ok [--comp 0.022] [--aw 1,0.03]
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
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--comp", type=float, default=0.022)
    ap.add_argument("--cog-gain", type=float, default=0.0,
                    help="COG LUT 增益 (默认0=固件定版OFF; 覆盖会改默认行为)")
    ap.add_argument("--aw", default="1,0.03", help="积分抗饱和律 'mode,rate' (默认 1,0.03)")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0
    step = args.step
    ramp_s = args.ramp_deg_s

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    # 20kHz 固件预清理: 停 N/C 流, 否则 ENABLE/MODE 响应被 40kHz 遥测流淹没,
    # expect 超时误报 "ENABLE fail" (2026-08-30 实测: 响应实际 ENABLE,OK,1 但 2.5s 未到)。
    # CMD:OFF 停主遥测(N 帧), TELEM:CUR,OFF 停电流流, POSDBG,0 停 PDB 流。
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
        ser.write(c)
        time.sleep(0.15)
    ser.reset_input_buffer()

    class LineBuffer:
        """累积式行缓存: 串口数据可能被切成任意片段, 跨迭代累积直到换行符, 保证每行完整。"""
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
                            raise RuntimeError("fault latched mid-run: flags=%s" % p[8])
                        a = float(p[3])
                        d = a - a0
                        if d > 180.0: d -= 360.0
                        elif d < -180.0: d += 360.0
                        samples.append((time.time() - t0, d))
            time.sleep(0.002)
        return samples

    ser.write(b"CMD:UNLOCK,1\n")
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK")
    expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK")
    ser.write(b"CMD:COG_CFG,%.3f,60.0\n" % args.cog_gain)
    expect("CMD:FRIC_COMP,%.3f,%.3f" % (args.comp, args.comp), "FRIC_COMP,OK")
    aw_mode, aw_rate = args.aw.split(",")
    expect("CMD:POS_AW_MODE,%s,%s" % (aw_mode, aw_rate), "POS_AW_MODE,OK")
    expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    # 恢复 N 帧 (预清理 CMD:OFF 关掉了主遥测, read_angle 需要 N 帧)
    ser.write(b"CMD:ON\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
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

    def sp(samples, target_deg, checkpoints=(0.5, 1.0, 2.0)):
        """到位判定。arrival=到位%(100=完全到位, >100=过冲); 检查点按指令发出后时刻。
        历史坑 (2026-08-30): 旧 err 字段是"剩余%"但标签写"到位%", 被倒读成 PASS;
        斜坡旧检查点 0.5/1/2s 全在斜坡内部, 保持段收敛从未被测。"""
        if len(samples) < 5:
            return None
        target = abs(target_deg)
        arr = {}
        for tm in checkpoints:
            b = min(samples, key=lambda s: abs(s[0] - tm))
            arr[tm] = round(min(abs(b[1]) / target, 1.2) * 100.0, 1)
        t_end = samples[-1][0]
        stab = [abs(abs(s[1]) - target) for s in samples if s[0] >= t_end - 1.0]
        dev = max((abs(abs(s[1]) - target) for s in samples), default=0.0)
        return {"arrival": arr, "pp": max(stab) - min(stab) if stab else 0.0,
                "max_dev": max(0.0, dev), "n": len(samples)}

    def traj_str(samples, every=0.5):
        out = []
        for i in range(int(samples[-1][0] * (1.0 / every)) + 1):
            tgt = i * every
            b = min(samples, key=lambda s: abs(s[0] - tgt))
            out.append("%.1f°@%.1fs" % (b[1], b[0]))
        return " ".join(out)

    try:
        # 1) 斜坡 +6° (重新锚定 a0, 钉住后位置可能漂移)
        a0 = read_angle()
        if a0 is None:
            raise RuntimeError("no angle for ramp")
        print("\n=== 斜坡 +%.1f° @%.1f°/s ===" % (step, ramp_s), flush=True)
        ramp_dur = step / ramp_s
        pos = track((a0 + step) * DEG2RAD, ramp_dur + 3.0, a0, is_step=False)
        # 检查点: 斜坡结束后 0.5/1.5/2.5s (保持段收敛才是判据; 斜坡内时刻无意义)
        r = sp(pos, step, checkpoints=(ramp_dur + 0.5, ramp_dur + 1.5, ramp_dur + 2.5))
        print("  n=%d %s" % (len(pos), traj_str(pos)), flush=True)
        if r:
            print("  到位%%(斜坡后) %s | pp=%.3f° 最大偏差=%.2f°" %
                  (" ".join("%.1fs=%s" % (k, v) for k, v in r["arrival"].items()),
                   r["pp"], r["max_dev"]), flush=True)
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
            print("  到位%% %s | pp=%.3f° 最大偏差=%.2f°" %
                  (" ".join("%.1fs=%s" % (k, v) for k, v in r["arrival"].items()),
                   r["pp"], r["max_dev"]), flush=True)
            results["step_pos"] = r
        else:
            print("  采样不足!")
        # 回起点并测稳态: 回位后稳定 5s 再测 2s pp (排除回程余振, 解释 002136 轮 4.18°)
        ser.write(b"CMD:PREF,%.5f\n" % (a1 * DEG2RAD))
        time.sleep(5.0)
        a2 = read_angle()
        print("\n=== 稳态(回位+5s后, 2s窗口) @%.2f° ===" % (a2 if a2 else -1), flush=True)
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
