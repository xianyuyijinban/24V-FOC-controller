#!/usr/bin/env python3
"""
齿槽LUT 相位扫描 — DIRECT kp=0.5/kd=0.03 + COG gain=0.25 下, 扫 phase 找 angle_pp 最低点

若某个 phase 显著低于 COG_OFF 基线 -> LUT 幅值对、相位错, 调相位即可生效。
"""
import argparse, json, os, statistics, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pos_direct_ab_test import SerialIO, step_stats, N_IDX

DEG2RAD = 3.14159265358979 / 180.0


def run_seq(io, rounds, dwell):
    """DIRECT kp=0.5/kd=0.03 + 当前 COG 配置, 跑 rounds 轮步进+斜坡"""
    angle_list, iq_list = [], []
    settle = max(0.2, dwell * 0.4)
    for _ in range(rounds):
        ok = io.expect_ok("CMD:ENABLE,1", "ENABLE,OK")
        if not ok[0]:
            io.send("CMD:STOP"); return None, None
        for t in (0.0, 2.0, 0.0):
            io.send("CMD:PREF,%.5f" % (t * DEG2RAD)); time.sleep(0.3)
        for i in range(1, 51):
            io.send("CMD:PREF,%.5f" % (i * 0.1 * DEG2RAD)); time.sleep(0.1)
        time.sleep(settle)
        frames = io.collect_nframes(dwell)
        s = step_stats(frames, "x")
        if s is None or s["max_fault"] != 0:
            io.send("CMD:STOP"); return None, None
        angle_list.append(s["angle_pp_deg"]); iq_list.append(s["iq_pp_a"])
        io.send("CMD:PREF,0"); io.send("CMD:STOP"); time.sleep(0.4)
    return angle_list, iq_list


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--dwell", type=float, default=1.5)
    ap.add_argument("--phases", default="-180,-120,-60,0,60,120")
    ap.add_argument("--gain", type=float, default=0.25)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok"); return 0
    phases = [float(x) for x in args.phases.split(",")]

    io = SerialIO(args.port, args.baud)
    io.open()
    io.expect_ok("CMD:UNLOCK,1", "UNLOCK,OK")
    io.expect_ok("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    io.expect_ok("CMD:POS_DIRECT_GAIN,0.5000,0.0300", "POS_DIRECT_GAIN,OK")
    io.expect_ok("CMD:MODE,2", "MODE,OK")

    rows = []
    try:
        # 基线: COG OFF
        io.send("CMD:COG_CFG,0.0,0.0")
        ang, iq = run_seq(io, args.rounds, args.dwell)
        if ang:
            rows.append({"phase": "OFF", "angle_pp": statistics.mean(ang), "iq_pp": statistics.mean(iq)})
            print("  phase=OFF   angle_pp=%.3f° iq_pp=%.4fA" % (rows[-1]["angle_pp"], rows[-1]["iq_pp"]), flush=True)
        for ph in phases:
            io.send("CMD:COG_CFG,%.2f,%.1f" % (args.gain, ph))
            ang, iq = run_seq(io, args.rounds, args.dwell)
            if ang:
                rows.append({"phase": ph, "angle_pp": statistics.mean(ang), "iq_pp": statistics.mean(iq)})
                print("  phase=%+.0f  angle_pp=%.3f° iq_pp=%.4fA" % (ph, rows[-1]["angle_pp"], rows[-1]["iq_pp"]), flush=True)
            else:
                print("  phase=%+.0f  [FAIL]" % ph, flush=True)
    finally:
        io.send("CMD:STOP"); io.send("CMD:CLEAR_FAULT")
        io.send("CMD:COG_CFG,0.25,60.0")  # 恢复默认
        io.close()

    print("\n================ 相位扫描 (按 angle_pp 升序) ================")
    for r in sorted(rows, key=lambda r: r["angle_pp"]):
        print("  phase=%+6s angle_pp=%.3f° iq_pp=%.4fA" % (str(r["phase"]), r["angle_pp"], r["iq_pp"]))
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "phase_sweep_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
