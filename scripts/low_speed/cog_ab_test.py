#!/usr/bin/env python3
"""
COG 齿槽LUT 贡献量化 — DIRECT kp=0.5/kd=0.03 下, COG ON(0.25) vs OFF(0)

判定: angle_pp 是否显著变化。若 OFF 与 ON 相当 -> 齿槽LUT 对低速纹波无贡献(相位/gain不对);
若 OFF 明显变差 -> LUT 有效, 保留并考虑精修相位。
"""
import argparse, json, os, statistics, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
from common.pos_direct_ab_test import SerialIO, step_stats, N_IDX

DEG2RAD = 3.14159265358979 / 180.0


def run_seq(io, cog_gain, rounds, dwell):
    """固定 DIRECT kp=0.5 kd=0.03, 设 COG gain, 跑 rounds 轮步进+斜坡, 返回每轮 angle_pp/iq_pp 列表"""
    io.expect_ok("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    io.expect_ok("CMD:POS_DIRECT_GAIN,0.5000,0.0300", "POS_DIRECT_GAIN,OK")
    io.send("CMD:COG_CFG,%.2f,60.0" % cog_gain)  # 无 ACK, 直接设
    io.expect_ok("CMD:MODE,2", "MODE,OK")
    angle_list, iq_list = [], []
    settle = max(0.2, dwell * 0.4)
    for _ in range(rounds):
        ok = io.expect_ok("CMD:ENABLE,1", "ENABLE,OK")
        if not ok[0]:
            return None, None, "ENABLE fail"
        seq = [0.0, 2.0, 0.0] + list(range(1, 51))  # 步进 + 斜坡到5
        for i, t in enumerate(seq):
            if i < 3:
                io.send("CMD:PREF,%.5f" % (t * DEG2RAD))
                time.sleep(0.3)
            else:
                io.send("CMD:PREF,%.5f" % (t * 0.1 * DEG2RAD))
                time.sleep(0.1)
        time.sleep(settle)
        frames = io.collect_nframes(dwell)
        s = step_stats(frames, "x")
        if s is None or s["max_fault"] != 0:
            io.send("CMD:STOP")
            return None, None, "fault in round"
        angle_list.append(s["angle_pp_deg"])
        iq_list.append(s["iq_pp_a"])
        io.send("CMD:PREF,0")
        io.send("CMD:STOP")
        time.sleep(0.4)
    return angle_list, iq_list, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--dwell", type=float, default=1.5)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: 需要 --power-ok"); return 0

    io = SerialIO(args.port, args.baud)
    io.open()
    io.expect_ok("CMD:UNLOCK,1", "UNLOCK,OK")
    frames = io.collect_nframes(1.0)
    if not frames or int(frames[-1][N_IDX["motor_id"]]) != 1:
        print("[ABORT] 电机未识别"); io.close(); return 1

    results = {}
    try:
        for label, gain in (("COG_ON", 0.25), ("COG_OFF", 0.0)):
            print("\n=== %s (gain=%.2f) ===" % (label, gain), flush=True)
            ang, iq, err = run_seq(io, gain, args.rounds, args.dwell)
            if err:
                print("  [FAIL] %s" % err)
                results[label] = {"status": "FAIL", "err": err}
                continue
            a_mean = statistics.mean(ang); i_mean = statistics.mean(iq)
            results[label] = {"angle_pp": a_mean, "iq_pp": i_mean, "raw_angle": ang, "raw_iq": iq}
            print("  angle_pp=%.3f°  iq_pp=%.4fA  (n=%d)" % (a_mean, i_mean, len(ang)), flush=True)
    finally:
        io.send("CMD:STOP"); io.send("CMD:CLEAR_FAULT")
        io.send("CMD:COG_CFG,0.25,60.0")  # 恢复默认
        io.close()

    print("\n================ COG ON/OFF 对比 ================")
    for k, v in results.items():
        if "angle_pp" in v:
            print("  %-8s angle_pp=%.3f° iq_pp=%.4fA" % (k, v["angle_pp"], v["iq_pp"]))
        else:
            print("  %-8s FAIL: %s" % (k, v.get("err")))
    if all("angle_pp" in v for v in results.values()):
        on = results["COG_ON"]; off = results["COG_OFF"]
        d = on["angle_pp"] - off["angle_pp"]
        print("\n  结论: COG ON 比 OFF angle_pp %s %.3f° (%.1f%%)" %
              ("改善" if d < 0 else "恶化", abs(d), 100 * d / max(off["angle_pp"], 1e-6)))

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cog_ab_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print("报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
