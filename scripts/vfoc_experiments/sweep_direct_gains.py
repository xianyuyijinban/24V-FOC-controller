#!/usr/bin/env python3
"""
直连 POS_DIRECT 增益扫参 — 找 kp(A/rad) × kd(A/(rad/s)) 最优组合

对每个 (kp, kd) 组合在 DIRECT 结构下跑: 步进 +2° + 斜坡 5°, 采稳态 angle_pp/iq_pp。
判定: angle_pp 越小越平滑(主指标), iq_pp 作参考。有 fault 的组合标记为 FAIL 跳过。

用法: python scripts/sweep_direct_gains.py --port COM10 --power-ok [--kp-list ...] [--kd-list ...]
"""
import argparse, json, os, statistics, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
from common.pos_direct_ab_test import SerialIO, step_stats, N_IDX

RAD2DEG = 57.29577951308232
DEG2RAD = 3.14159265358979 / 180.0


def run_combo(io, kp, kd, dwell):
    """在 DIRECT 结构下跑一个组合的步进+斜坡, 返回 (ok, [step_stats...])"""
    ok, lines = io.expect_ok("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    if not ok:
        print("  POS_DIRECT no ACK"); return False, []
    ok, lines = io.expect_ok("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (kp, kd), "POS_DIRECT_GAIN,OK")
    if not ok:
        print("  POS_DIRECT_GAIN no ACK %r" % lines); return False, []
    io.expect_ok("CMD:MODE,2", "MODE,OK")
    ok, lines = io.expect_ok("CMD:ENABLE,1", "ENABLE,OK")
    if not ok:
        print("  ENABLE no ACK %r" % lines); return False, []

    results = []
    settle = max(0.2, dwell * 0.4)
    # 步进: 0 -> +2 -> 0
    for target in (0.0, 2.0, 0.0):
        io.send("CMD:PREF,%.5f" % (target * DEG2RAD))
        time.sleep(settle)
        frames = io.collect_nframes(dwell - settle)
        s = step_stats(frames, "step%.0f" % target)
        if s is None or s["max_fault"] != 0:
            io.send("CMD:STOP")
            return False, results
        results.append(s)
    # 斜坡: 0 -> +5 (0.1°/步, 0.1s)
    for i in range(1, 51):
        io.send("CMD:PREF,%.5f" % (i * 0.1 * DEG2RAD))
        time.sleep(0.1)
    time.sleep(settle)
    frames = io.collect_nframes(dwell - settle)
    s = step_stats(frames, "ramp5")
    if s is None or s["max_fault"] != 0:
        io.send("CMD:STOP")
        return False, results
    results.append(s)
    io.send("CMD:PREF,0")
    time.sleep(dwell)
    io.send("CMD:STOP")
    time.sleep(0.4)
    return True, results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--dwell", type=float, default=1.5)
    ap.add_argument("--kp-list", default="0.5,1.0,2.0")
    ap.add_argument("--kd-list", default="0.03,0.06,0.10")
    ap.add_argument("--rounds", type=int, default=1, help="每组合轮数, 取均值")
    args = ap.parse_args()

    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok")
        return 0
    kps = [float(x) for x in args.kp_list.split(",")]
    kds = [float(x) for x in args.kd_list.split(",")]

    io = SerialIO(args.port, args.baud)
    io.open()
    io.expect_ok("CMD:UNLOCK,1", "UNLOCK,OK")
    frames = io.collect_nframes(1.0)
    if not frames or int(frames[-1][N_IDX["motor_id"]]) != 1:
        print("[ABORT] 电机未识别")
        io.close(); return 1

    rows = []
    try:
        for kp in kps:
            for kd in kds:
                print("\n=== kp=%.2f kd=%.3f ===" % (kp, kd), flush=True)
                all_stats = []
                fail = False
                for rnd in range(args.rounds):
                    ok, stats = run_combo(io, kp, kd, args.dwell)
                    if not ok:
                        fail = True
                        break
                    all_stats.extend(stats)
                if fail:
                    print("  [FAIL] 该组合故障或未完成, 跳过")
                    rows.append({"kp": kp, "kd": kd, "status": "FAIL"})
                    continue
                angle_pp = statistics.mean(s["angle_pp_deg"] for s in all_stats)
                iq_pp = statistics.mean(s["iq_pp_a"] for s in all_stats)
                speed_pp = statistics.mean(s["speed_pp_radps"] for s in all_stats)
                rows.append({"kp": kp, "kd": kd, "status": "OK",
                             "angle_pp_deg": angle_pp, "iq_pp_a": iq_pp,
                             "speed_pp_radps": speed_pp})
                print("  angle_pp=%.3f°  iq_pp=%.4fA  speed_pp=%.3f" % (angle_pp, iq_pp, speed_pp), flush=True)
    finally:
        io.send("CMD:STOP")
        io.send("CMD:CLEAR_FAULT")
        io.close()

    # 汇总
    ok_rows = [r for r in rows if r["status"] == "OK"]
    ok_rows.sort(key=lambda r: r["angle_pp_deg"])
    print("\n================ 直连增益扫参结果 (按 angle_pp 升序) ================")
    print("%-8s %-8s %-12s %-10s %-10s %s" % ("kp", "kd", "angle_pp°", "iq_ppA", "speed_pp", "status"))
    for r in ok_rows:
        print("%-8.2f %-8.3f %-12.3f %-10.4f %-10.3f %s" %
              (r["kp"], r["kd"], r["angle_pp_deg"], r["iq_pp_a"], r["speed_pp_radps"], r["status"]))
    for r in rows:
        if r["status"] != "OK":
            print("%-8.2f %-8.3f %-12s %-10s %-10s %s" % (r["kp"], r["kd"], "-", "-", "-", r["status"]))
    if ok_rows:
        best = ok_rows[0]
        print("\n最优: kp=%.2f kd=%.3f  angle_pp=%.3f° iq_pp=%.4fA" %
              (best["kp"], best["kd"], best["angle_pp_deg"], best["iq_pp_a"]))

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweep_direct_gains_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"rows": rows}, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
