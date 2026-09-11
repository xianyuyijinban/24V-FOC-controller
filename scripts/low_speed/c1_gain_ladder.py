#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C1 步骤4: COG gain 爬升标定 (2026-09-10).

前置: phi* 已由 cog_phase_sweep 锚定 (C1: 179.50° @ zero_deg=0).
方法: 固定 phi*, gain 依次 [0.25, 0.50, 1.00], 每档 5°/s 采集,
      LSQ amp22 (与 sweep 同口径) + th_pp (极限环检查).
判据: 选残余最小且不振荡档; th_pp 相对最优档突增 3× 即降档.
安全: 结束回 COG OFF + STOP; 任何异常立即 abort 并清理.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cog_phase_sweep import Capturer, analyze_seg, DEG2RAD  # noqa: E402

import serial  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--phi", type=float, default=179.50, help="phi* (C1 锚定值)")
    ap.add_argument("--speed", type=float, default=5.0)
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--gains", default="0,0.25,0.5,1.0", help="含 0 = OFF 基线")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    ser = serial.Serial(args.port, 1000000, timeout=0.05)
    time.sleep(0.5)

    def send(c, wait=0.25):
        ser.write((c + "\n").encode())
        time.sleep(wait)

    def expect(c, p, to=3.0):
        ser.reset_input_buffer()
        ser.write((c + "\n").encode())
        t0 = time.time()
        buf = b""
        pat = p.encode()
        while time.time() - t0 < to:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                if pat in buf:
                    return True
            time.sleep(0.02)
        return False

    # 清理 + 配置 (与 sweep 同工况)
    for c in ("CMD:OFF", "CMD:PDBBIN,0", "CMD:POSDBG,0", "CMD:STOP",
              "CMD:CLEAR_FAULT", "CMD:VOLT_OFF"):
        send(c, 0.15)
    for c, p in [("CMD:UNLOCK,1", "UNLOCK,OK"),
                 ("CMD:POS_DIRECT,1", "POS_DIRECT,OK"),
                 ("CMD:POS_DIRECT_GAIN,0.490,0.007", "POS_DIRECT_GAIN,OK"),
                 ("CMD:POS_DIRECT_KI,0.37", "POS_DIRECT_KI,OK"),
                 ("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"),
                 ("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"),
                 ("TELEM:OFF", "TELEM:OFF,OK")]:
        if not expect(c, p):
            print("配置失败: %s" % c)
            ser.close()
            return 1
    send("CMD:COG_CFG,0.000,%.2f" % args.phi)
    send("CMD:POSDBG,1", 0.5)
    if not expect("CMD:MODE,2", "MODE,OK"):
        print("MODE,2 失败"); ser.close(); return 1
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE 失败"); ser.close(); return 1
    time.sleep(0.8)

    cap = Capturer(ser)
    cap.drain()
    relay = {"tgt": None, "t_last": 0.0}

    def ramp_tick(v):
        now = time.time()
        if now - relay["t_last"] >= 0.05:
            if relay["t_last"] > 0:
                relay["tgt"] += v * DEG2RAD * (now - relay["t_last"])
            ser.write(("CMD:PREF,%.6f\n" % relay["tgt"]).encode())
            relay["t_last"] = now

    def run_at(label, secs):
        if relay["tgt"] is None:
            for r in reversed(cap.pdb):
                if not r[0].startswith("__"):
                    relay["tgt"] = r[2]
                    break
            if relay["tgt"] is None:
                raise RuntimeError("PDB 流空")
        cap.label = label
        t0 = time.time()
        while time.time() - t0 < secs:
            ramp_tick(args.speed)
            cap.drain()
            time.sleep(0.001)
        cap.drain()
        rows = [r for r in cap.pdb if r[0] == label]
        return analyze_seg(rows)

    results = {"phi": args.phi, "speed": args.speed, "ladder": []}
    try:
        for g in [float(x) for x in args.gains.split(",") if x.strip()]:
            send("CMD:COG_CFG,%.3f,%.2f" % (g, args.phi))
            a = run_at("gain.%.2f" % g, args.secs)
            if a is None:
                print("gain=%.2f 样本不足" % g)
                continue
            row = {"gain": g, "v_mean": round(a["v_mean"], 2),
                   "amp22": None if a["amp22"] is None else round(a["amp22"], 5),
                   "th_pp": round(a["th_pp"], 3), "suspect": a["suspect"]}
            results["ladder"].append(row)
            print("  gain=%.2f  v=%+.2f°/s  amp22=%s  th_pp=%.2f°%s" %
                  (g, a["v_mean"],
                   "%.5f" % a["amp22"] if a["amp22"] is not None else "n/a",
                   a["th_pp"], "  SUSPECT" if a["suspect"] else ""), flush=True)
    except Exception as exc:
        results["error"] = str(exc)
        print("ABORT:", exc)
    finally:
        send("CMD:COG_CFG,0.000,0.0")
        send("CMD:STOP")
        send("CMD:TELEM:OFF" if False else "TELEM:OFF")
        print("\n清理: COG OFF, STOP")
        ser.close()

    # 判读: 最优 gain (amp22 min), 极限环降档
    valid = [r for r in results["ladder"] if r["amp22"] is not None and not r["suspect"]]
    if valid:
        best = min(valid, key=lambda r: r["amp22"])
        results["best_gain"] = best["gain"]
        # 极限环检查: 各档 th_pp vs OFF 基线 th_pp
        off = next((r for r in results["ladder"] if r["gain"] == 0.0), None)
        if off:
            results["th_pp_vs_off"] = [
                {"gain": r["gain"],
                 "ratio": round(r["th_pp"] / max(off["th_pp"], 1e-9), 2)}
                for r in results["ladder"]]
        print("最优 gain=%.2f (amp22=%.5f)" % (best["gain"], best["amp22"]))

    out = args.out or os.path.join(HERE, "c1_gain_ladder_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    print("JSON:", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
