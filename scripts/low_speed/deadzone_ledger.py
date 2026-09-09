#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""门控死区三臂诊断 — 过程量分账表 (2026-09-10, 纯离线).

三过程量 (每轮, v2 帧口径):
  1. track_pct          斜坡跟踪率 (ramp.track_pct)
  2. ramp_ff_coulomb    斜坡段 ff_coulomb 轨迹 (traj t<=3.2s, v2 直读) + |max|
  3. ramp_pos_integral  斜坡段 pos_integral 轨迹 + 充电深度 max|.|

主表 = 用户指定 7 轮 (A×2 / B×2 / C rep1 / 定版复跑×2);
附录 = 非 7 轮口径参考 (A 首跑无 v2 列 / C 首 abort)。
数值直读 JSON 原文 (round 后的存储值), 不做再加工。
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RAMP_END = 3.2   # 斜坡 3.0s + 0.2s 裕量

MAIN = [
    ("A", "verify_lowspeed_20260910_012409.json", [1, 2]),
    ("B", "verify_lowspeed_20260910_012632.json", [1, 2]),
    ("C", "verify_lowspeed_20260910_013643.json", [1]),
    ("base", "verify_lowspeed_20260910_013856.json", [1, 2]),
]
AUX = [
    ("A-first", "verify_lowspeed_20260910_012049.json", [1, 2]),
    ("C-abort1", "verify_lowspeed_20260910_013401.json", [1]),
]


def extract(path, reps):
    d = json.load(open(os.path.join(HERE, path), encoding="utf-8"))
    out = []
    for i in reps:
        r = d["results"]["reps"][i - 1]
        ramp = [t for t in r.get("traj", []) if t["t"] <= RAMP_END]
        fc = [t.get("ff_coulomb") for t in ramp]
        pi = [t.get("pos_integral") for t in ramp]
        fc_ok = [x for x in fc if x is not None]
        pi_ok = [x for x in pi if x is not None]
        out.append({
            "rep": i,
            "track": r.get("ramp", {}).get("track_pct"),
            "step28": r.get("step", {}).get("arrival", {}).get("2.8"),
            "fc": fc, "pi": pi,
            "fc_absmax": round(max(abs(x) for x in fc_ok), 5) if fc_ok else None,
            "pi_depth": round(max(abs(x) for x in pi_ok), 5) if pi_ok else None,
            "v2_fc_absmax": r.get("v2_fields", {}).get("ff_coulomb_absmax"),
            "valid": d.get("run_status", {}).get("valid"),
        })
    return out


def show(title, runs):
    print("\n### %s" % title)
    print("%-8s %-4s %8s %8s  %-34s %-8s  %-34s %-8s" %
          ("臂", "轮", "track%", "step2.8%", "斜坡 ff_coulomb 轨迹 (A)", "|fc|max",
           "斜坡 pos_integral 轨迹 (A)", "深度"))
    for arm, path, reps in runs:
        for row in extract(path, reps):
            fc_s = " / ".join("None" if x is None else "%.5f" % x for x in row["fc"])
            pi_s = " / ".join("None" if x is None else "%.5f" % x for x in row["pi"])
            print("%-8s %-4d %8s %8s  %-34s %-8s  %-34s %-8s" %
                  (arm, row["rep"], row["track"], row["step28"],
                   fc_s[:34], row["fc_absmax"], pi_s[:34], row["pi_depth"]))
    # 臂内汇总
    print("-- %s 汇总 --" % title)
    by_arm = {}
    for arm, path, reps in runs:
        for row in extract(path, reps):
            by_arm.setdefault(arm, []).append(row)
    for arm, rows_ in by_arm.items():
        tracks = [r["track"] for r in rows_ if r["track"] is not None]
        fcms = [r["fc_absmax"] for r in rows_ if r["fc_absmax"] is not None]
        pids = [r["pi_depth"] for r in rows_ if r["pi_depth"] is not None]
        print("%-8s n=%d  track=[%s]  |fc|max=[%s]  pi_depth=[%s]" %
              (arm, len(rows_),
               ", ".join(str(x) for x in tracks),
               ", ".join(str(x) for x in fcms),
               ", ".join(str(x) for x in pids)))


if __name__ == "__main__":
    show("主表: 7 轮 (用户口径)", MAIN)
    show("附录: 参考轮 (非 7 轮口径)", AUX)
