#!/usr/bin/env python3
"""
POS_DIRECT A/B 判别实验 — 位置环直连电流环 vs 三环级联（低速平滑性对比）

背景:
  低速域速度估计信噪比差，速度环(P-only)扰动抑制弱。判别实验验证
  "位置环PD输出直接作为力矩指令(A)进电流环" 是否比三环级联更平滑。
  固件侧开关: CMD:POS_DIRECT,0(级联,默认) / CMD:POS_DIRECT,1(直连)。
  直连增益:   CMD:POS_DIRECT_GAIN,kp,kd  (默认 kp=1.0 A/rad, kd=0.03 A/(rad/s))

测试序列 (每轮, 位置模式 RAW MODE=2):
  1. 步进组: PREF 0° → +step_deg → 0° → -step_deg → 0°  (每步 dwell 秒)
  2. 斜坡组: PREF 小步斜坡到 +ramp_total_deg 再回 0° (ramp_step_deg / ramp_period_ms)
  每步稳态窗(后60% dwell)统计: 实际角度 p-p/std、Iq p-p/std、速度 p-p、fault。

对比指标: cascade 与 direct 在同序列下的纹波统计。
N 帧字段索引见 N_IDX (对应 uart_upload.c N,33字段布局)。
PREF 输入单位 = rad (固件 FOC_App_SetPositionRef 直接收 rad)。

用法:
  python scripts/pos_direct_ab_test.py --port COM7 --power-ok [--rounds 2]
安全:
  功率步骤要求显式 --power-ok；步进/斜坡幅度默认 ±2°/5° 小范围。
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. pip install pyserial")
    sys.exit(1)

T_SHORT = 0.3
RAD2DEG = 57.29577951308232
DEG2RAD = math.pi / 180.0

# N帧字段索引: "N,ts,state,angle,speed,Id,Iq,vbus,faultFlags,encDet,motorId,
#               stallArmed,stallOpenLoop,appWarn,appFault,ctrlMode,idRef,
#               speedRef,posRef,iqRef,Vd,Vq,Ia,Ib,Ic,..."
N_IDX = {
    "ts": 1, "state": 2, "angle": 3, "speed": 4, "id": 5, "iq": 6,
    "vbus": 7, "fault_flags": 8, "enc_det": 9, "motor_id": 10,
    "stall_armed": 11, "stall_open_loop": 12, "app_warn": 13,
    "app_fault": 14, "ctrl_mode": 15, "id_ref": 16, "speed_ref": 17,
    "pos_ref": 18, "iq_ref": 19, "vd": 20, "vq": 21,
    "ia": 22, "ib": 23, "ic": 24,
}


class SerialIO:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd, wait=T_SHORT):
        """发送命令，返回非遥测(N/C/B前缀)响应行。"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        time.sleep(wait)
        raw = self.ser.read(self.ser.in_waiting)
        text = raw.decode(errors="replace")
        return [l.strip() for l in text.split("\n")
                if l.strip() and not l.startswith(("N,", "C,", "B,"))]

    def expect_ok(self, cmd, prefix, timeout=1.5):
        """发送命令并等待包含 prefix 的响应，返回 (ok, lines)。"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        deadline = time.time() + timeout
        lines = []
        while time.time() < deadline:
            if self.ser.in_waiting:
                raw = self.ser.read(self.ser.in_waiting)
                for l in raw.decode(errors="replace").split("\n"):
                    l = l.strip()
                    if not l or l.startswith(("N,", "C,", "B,")):
                        continue
                    lines.append(l)
                    if l.startswith(prefix):
                        return True, lines
            else:
                time.sleep(0.01)
        return False, lines

    def collect_nframes(self, dur_s):
        frames = []
        deadline = time.time() + dur_s
        while time.time() < deadline:
            w = self.ser.in_waiting
            if w:
                for line in self.ser.read(w).decode(errors="replace").split("\n"):
                    line = line.strip()
                    if line.startswith("N,"):
                        parts = line.split(",")
                        if len(parts) >= 25:
                            frames.append(parts)
            else:
                time.sleep(0.005)
        return frames


def wrap_pi(x):
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def p2p(vals):
    return (max(vals) - min(vals)) if len(vals) > 1 else 0.0


def step_stats(frames, label):
    """稳态窗统计: 以窗内角度均值为基准, 输出位置/Iq/速度纹波。"""
    if not frames:
        return None
    angles = [float(f[N_IDX["angle"]]) for f in frames]
    iqs = [float(f[N_IDX["iq"]]) for f in frames]
    speeds = [float(f[N_IDX["speed"]]) for f in frames]
    faults = [int(f[N_IDX["app_fault"]]) for f in frames]
    base = statistics.mean(angles)
    err = [wrap_pi(a - base) for a in angles]
    return {
        "step": label,
        "n": len(frames),
        "angle_pp_deg": p2p(err) * RAD2DEG,
        "angle_std_deg": statistics.pstdev(err) * RAD2DEG,
        "iq_pp_a": p2p(iqs),
        "iq_std_a": statistics.pstdev(iqs),
        "speed_pp_radps": p2p(speeds),
        "max_fault": max(faults),
    }


def run_round(io, direct, kp, kd, args, verbose=True):
    """一轮: 设定结构 → 位置模式 → 步进组+斜坡组 → STOP。返回 (ok, steps, fault)"""
    steps = []

    if direct:
        ok, lines = io.expect_ok("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (kp, kd), "POS_DIRECT_GAIN,OK")
        if not ok:
            print("  [FAIL] POS_DIRECT_GAIN no ACK: %r" % lines)
            return False, steps, True
        if verbose:
            print("  direct gains: kp=%.4f A/rad, kd=%.4f A/(rad/s)" % (kp, kd))
    ok, lines = io.expect_ok("CMD:POS_DIRECT,%d" % (1 if direct else 0), "POS_DIRECT,OK")
    if not ok:
        print("  [FAIL] POS_DIRECT no ACK: %r" % lines)
        return False, steps, True
    if verbose:
        print("  structure: %s (ACK: %s)" % ("DIRECT" if direct else "CASCADE", lines[-1]))

    io.expect_ok("CMD:MODE,2", "MODE,OK")
    ok, lines = io.expect_ok("CMD:ENABLE,1", "ENABLE,OK")
    if not ok:
        print("  [FAIL] ENABLE no ACK: %r" % lines)
        return False, steps, True

    dwell = args.dwell
    settle_skip = max(0.2, dwell * 0.4)  # 前40%丢弃, 后60%稳态窗

    # ── 步进组 ──
    step_rad = args.step_deg * DEG2RAD
    for i, target_deg in enumerate([0.0, args.step_deg, 0.0, -args.step_deg, 0.0]):
        io.send("CMD:PREF,%.5f" % (target_deg * DEG2RAD))
        time.sleep(settle_skip)
        frames = io.collect_nframes(dwell - settle_skip)
        if i == 0:
            lbl = "start@0°"
        elif i == 2:
            lbl = "return0°"
        elif i == 4:
            lbl = "return0°b"
        elif target_deg > 0:
            lbl = "step+%.1f°" % args.step_deg
        else:
            lbl = "step-%.1f°" % args.step_deg
        s = step_stats(frames, lbl)
        if s is None:
            print("  [FAIL] no N-frames at step %d" % i)
            io.send("CMD:STOP")
            return False, steps, True
        steps.append(s)
        if s["max_fault"] != 0:
            print("  [FAIL] AppFault=%d at step %s — abort" % (s["max_fault"], s["step"]))
            io.send("CMD:STOP")
            return False, steps, True
        if verbose:
            print("  %s: angle_pp=%.4f° iq_pp=%.4fA speed_pp=%.4f" %
                  (s["step"], s["angle_pp_deg"], s["iq_pp_a"], s["speed_pp_radps"]))

    # ── 斜坡组 ──
    ramp_total = args.ramp_total_deg * DEG2RAD
    ramp_step = args.ramp_step_deg * DEG2RAD
    n_ramp = max(1, int(round(args.ramp_total_deg / args.ramp_step_deg)))
    for i in range(1, n_ramp + 1):
        io.send("CMD:PREF,%.5f" % (i * ramp_step))
        time.sleep(args.ramp_period_ms / 1000.0)
    time.sleep(settle_skip)
    frames = io.collect_nframes(dwell - settle_skip)
    s = step_stats(frames, "ramp_end+%.1f°" % args.ramp_total_deg)
    if s is None:
        print("  [FAIL] no N-frames at ramp end")
        io.send("CMD:STOP")
        return False, steps, True
    steps.append(s)
    if verbose:
        print("  %s: angle_pp=%.4f° iq_pp=%.4fA speed_pp=%.4f" %
              (s["step"], s["angle_pp_deg"], s["iq_pp_a"], s["speed_pp_radps"]))
    io.send("CMD:PREF,0")
    time.sleep(dwell)
    io.send("CMD:STOP")
    time.sleep(0.5)
    return True, steps, False


def aggregate(rounds):
    """多轮合并: 相同 step 标签取均值。"""
    keys = ["angle_pp_deg", "angle_std_deg", "iq_pp_a", "iq_std_a", "speed_pp_radps"]
    out = {}
    for r in rounds:
        for s in r["steps"]:
            e = out.setdefault(s["step"], {k: [] for k in keys})
            for k in keys:
                e[k].append(s[k])
            e.setdefault("max_fault", 0)
            e["max_fault"] = max(e["max_fault"], s["max_fault"])
    return {k: {kk: statistics.mean(vv) for kk, vv in v.items() if kk != "max_fault"}
            | {"max_fault": v["max_fault"]} for k, v in out.items()}


def main():
    ap = argparse.ArgumentParser(description="POS_DIRECT A/B 判别实验")
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true",
                    help="显式确认台架安全, 允许功率步骤 (电机转动)")
    ap.add_argument("--rounds", type=int, default=2, help="每种结构轮数 (交替进行)")
    ap.add_argument("--dwell", type=float, default=2.5, help="每步停留秒数")
    ap.add_argument("--step-deg", type=float, default=2.0, help="步进幅度 °")
    ap.add_argument("--ramp-total-deg", type=float, default=5.0, help="斜坡终点 °")
    ap.add_argument("--ramp-step-deg", type=float, default=0.1, help="斜坡每步 °")
    ap.add_argument("--ramp-period-ms", type=float, default=100.0, help="斜坡步周期 ms")
    ap.add_argument("--kp", type=float, default=1.0, help="直连Kp A/rad")
    ap.add_argument("--kd", type=float, default=0.03, help="直连Kd A/(rad/s)")
    args = ap.parse_args()

    if not args.power_ok:
        print("DRY-RUN: 功率步骤需要 --power-ok 显式确认 (电机将转动)。")
        print("计划: %d 轮交替, step=±%.1f°, ramp=%.1f°, dwell=%.1fs, kp=%.3f, kd=%.3f"
              % (args.rounds, args.step_deg, args.ramp_total_deg, args.dwell, args.kp, args.kd))
        return 0

    io = SerialIO(args.port, args.baud)
    print("连接 %s @ %d ..." % (args.port, args.baud))
    io.open()

    # 预检: UNLOCK + 电机已识别
    ok, lines = io.expect_ok("CMD:UNLOCK,1", "UNLOCK,OK")
    print("UNLOCK: %s" % ("OK" if ok else "FAIL %r" % lines))
    frames = io.collect_nframes(1.0)
    motor_id = int(frames[-1][N_IDX["motor_id"]]) if frames else 0
    if not motor_id:
        print("[ABORT] 电机未识别 (motorIdentified=0), 先跑 CMD:IDENTIFY,1")
        io.close()
        return 1
    print("motorIdentified=1 OK")

    results = {"cascade": [], "direct": []}
    try:
        for r in range(args.rounds):
            print("\n=== Round %d/%d — CASCADE (三环级联) ===" % (r + 1, args.rounds))
            ok, steps, _ = run_round(io, direct=False, kp=args.kp, kd=args.kd, args=args)
            if not ok:
                print("[ABORT] cascade round failed")
                break
            results["cascade"].append({"round": r + 1, "steps": steps})

            print("=== Round %d/%d — DIRECT (位置环直连) ===" % (r + 1, args.rounds))
            ok, steps, _ = run_round(io, direct=True, kp=args.kp, kd=args.kd, args=args)
            if not ok:
                print("[ABORT] direct round failed")
                break
            results["direct"].append({"round": r + 1, "steps": steps})
    finally:
        io.send("CMD:STOP")
        io.send("CMD:CLEAR_FAULT")
        io.close()

    if not results["cascade"] or not results["direct"]:
        print("\n[FAIL] 实验未完成 (无完整轮次数据)")
        return 1

    agg = {k: aggregate(v) for k, v in results.items()}
    labels = sorted(set(agg["cascade"]) | set(agg["direct"]))

    print("\n================ 对比报告 ================")
    print("%-14s %-14s %-12s %-12s %-12s" %
          ("step", "structure", "angle_pp°", "iq_pp A", "speed_pp"))
    for lbl in labels:
        for name in ("cascade", "direct"):
            if lbl in agg[name]:
                e = agg[name][lbl]
                print("%-14s %-14s %-12.4f %-12.4f %-12.4f" %
                      (lbl, name, e["angle_pp_deg"], e["iq_pp_a"], e["speed_pp_radps"]))
    print("-" * 64)
    for name in ("cascade", "direct"):
        e = agg[name]
        if e:
            print("%-14s angle_pp均值=%.4f°  iq_pp均值=%.4fA  speed_pp均值=%.4f" %
                  (name,
                   statistics.mean(v["angle_pp_deg"] for v in e.values()),
                   statistics.mean(v["iq_pp_a"] for v in e.values()),
                   statistics.mean(v["speed_pp_radps"] for v in e.values())))

    # 保存 JSON
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "pos_direct_ab_%s.json" % ts)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results, "aggregate": agg},
                  f, ensure_ascii=False, indent=2)
    print("\n报告已保存: %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
