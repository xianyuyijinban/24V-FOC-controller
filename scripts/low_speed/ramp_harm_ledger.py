#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ramp_harm_ledger.py — 门控死区危害量化 (2026-09-10, 纯离线).

任务卡四指标 (斜坡段): 锯齿包络 / 滑跳事件谱 / 速度纹波 / 回稳时间,
S2 vs 电流模式对照, 出口判据 2× 带.

**数据边界实测 (先于计算)**:
  verify JSON 的 traj = pdb_rows[::100] 抽稀产物 — N 共存流 80Hz 下
  中位 dt ≈ 1.2s (0.8Hz), 斜坡 3s 窗内每轮仅 2-3 点。
  四指标中的三个 (包络 RMS/滑跳差分/纹波 p-p) 需要 ≥10Hz 级采样 —
  **现有 JSON 物理上不可算**, 本工具显式输出 INSUFFICIENT 而非硬算。
  唯一含全速率列的 JSON 是 ladder 的 poserr_traj_full (200Hz), 但
  ladder 序列为"静止 6° 阶跃"不含 2°/s 斜坡段 — 口径不对, 不借用。

可算指标 (分辨率无关, 直读 JSON 原文):
  A. track_pct (斜坡跟踪率)
  B. 斜坡后收敛序列 arrival@3.5/4.5/5.5s
  C. 阶跃 arrival@0.5/1/2/2.8s + pp + max_dev
  D. 稳态 pp/resid (回稳质量的粗代理; 真实"回稳时间"需斜坡终点后
     亚秒级序列, 不可算 — 用 step 段 2.8s 检查点作替代口径)

纪律: 只读 JSON 不改数; 数值直读原文 (存储精度)。
"""
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
RAMP_END = 3.2

# 数据源 (validator OK 前置)
S2_RUNS = [
    ("S2-A", "verify_lowspeed_20260910_012409.json"),
    ("S2-B", "verify_lowspeed_20260910_012632.json"),
    ("S2-C(rep1)", "verify_lowspeed_20260910_013643.json"),   # run invalid (abort) — 标注
    ("S2-base", "verify_lowspeed_20260910_013856.json"),
]
CUR_RUNS = [
    ("cur-084549", "verify_lowspeed_20260909_084549.json"),
    ("cur-105401", "verify_lowspeed_20260905_105401.json"),
    ("cur-084215", "verify_lowspeed_20260909_084215.json"),
]
ALL_RUNS = S2_RUNS + CUR_RUNS

MIN_POINTS_FOR_ENVELOPE = 10   # 包络/差分类统计的最低点数


def load(path):
    with open(os.path.join(HERE, path), encoding="utf-8") as f:
        return json.load(f)


def reps_of(doc):
    rs = doc.get("results")
    if isinstance(rs, dict):
        return rs.get("reps", [])
    return rs or []


def ramp_traj(reps, i):
    return [t for t in reps[i].get("traj", []) if t["t"] <= RAMP_END]


def envelope_stats(traj):
    """锯齿包络: max|err| / RMS / 峰峰。点数不足返回 INSUFFICIENT。"""
    errs = [t["poserr_deg"] for t in traj if "poserr_deg" in t]
    if len(errs) < MIN_POINTS_FOR_ENVELOPE:
        return {"status": "INSUFFICIENT", "n": len(errs),
                "required": MIN_POINTS_FOR_ENVELOPE}
    return {"status": "OK", "n": len(errs),
            "absmax": round(max(abs(e) for e in errs), 4),
            "rms": round((sum(e * e for e in errs) / len(errs)) ** 0.5, 4),
            "pp": round(max(errs) - min(errs), 4)}


def slip_spectrum(traj):
    """滑跳事件谱: theta 差分辨力受 traj 采样率限制。"""
    if len(traj) < MIN_POINTS_FOR_ENVELOPE:
        return {"status": "INSUFFICIENT", "n": len(traj),
                "required": MIN_POINTS_FOR_ENVELOPE,
                "note": "traj 0.8Hz: 事件间隔<40ms/单次滑跳幅度均不可分辨"}
    # 仅当点数足够才做差分 (当前数据到不了这里)
    events = []
    for a, b in zip(traj, traj[1:]):
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        v = (b["theta_deg"] - a["theta_deg"]) / dt
        events.append(round(v, 3))
    return {"status": "OK", "n": len(events), "vel_samples_deg_s": events}


def ripple_stats(traj):
    """速度纹波: 差分速度 p-p + 2°/s±50% 带内时间占比。"""
    if len(traj) < MIN_POINTS_FOR_ENVELOPE:
        return {"status": "INSUFFICIENT", "n": len(traj),
                "required": MIN_POINTS_FOR_ENVELOPE}
    vs = []
    for a, b in zip(traj, traj[1:]):
        dt = b["t"] - a["t"]
        if dt > 0:
            vs.append((b["theta_deg"] - a["theta_deg"]) / dt)
    if not vs:
        return {"status": "INSUFFICIENT", "n": 0}
    in_band = sum(1 for v in vs if 1.0 <= v <= 3.0)   # 2±50%
    return {"status": "OK", "n": len(vs),
            "pp_deg_s": round(max(vs) - min(vs), 3),
            "in_band_pct": round(100.0 * in_band / len(vs), 1)}


def collect():
    rows = []
    for tag, path in ALL_RUNS:
        doc = load(path)
        valid = doc.get("run_status", {}).get("valid")
        for i, rep in enumerate(reps_of(doc)):
            traj = ramp_traj(reps_of(doc), i)
            rows.append({
                "tag": tag, "rep": i + 1, "valid": valid,
                "track": rep.get("ramp", {}).get("track_pct"),
                "ramp_arrival": rep.get("ramp", {}).get("arrival"),
                "step_arrival": rep.get("step", {}).get("arrival"),
                "step_pp": rep.get("step", {}).get("pp"),
                "steady_pp": rep.get("steady", {}).get("pp_deg"),
                "steady_resid": rep.get("steady", {}).get("resid_deg"),
                "ramp_n": len(traj),
                "envelope": envelope_stats(traj),
                "slip": slip_spectrum(traj),
                "ripple": ripple_stats(traj),
            })
    return rows


def fmt(v):
    return "None" if v is None else str(v)


def main():
    rows = collect()

    print("=" * 100)
    print("数据边界声明 (先于一切计算)")
    print("=" * 100)
    print("traj = pdb_rows[::100] 抽稀 (verify_low_speed.py:553); N 共存 80Hz 流下")
    print("中位 dt ≈ 1.2s → 0.8Hz。斜坡 3s 窗内点数实测: %s" %
          ", ".join("%s=%d" % (r["tag"], r["ramp_n"]) for r in rows))
    print("→ 锯齿包络/滑跳谱/速度纹波 需要 ≥%d 点 (≥3Hz 级); 实测 2-4 点," %
          MIN_POINTS_FOR_ENVELOPE)
    print("   三项指标 INSUFFICIENT, 不硬算。回稳时间需斜坡终点后亚秒序列, 同样不可算;")
    print("   以阶跃段 2.8s 检查点作替代口径 (标注)。")
    print()

    print("=" * 100)
    print("A. 斜坡跟踪率 + 斜坡后收敛序列 (直读 JSON 原文)")
    print("=" * 100)
    print("%-12s %-4s %-6s %8s  %-38s %s" %
          ("tag", "rep", "valid", "track%", "ramp arrival (3.5/4.5/5.5s)",
           "ramp traj n"))
    for r in rows:
        arr = r["ramp_arrival"] or {}
        arr_s = " / ".join(fmt(arr.get(k)) for k in sorted(arr, key=float)) if arr else "-"
        print("%-12s %-4d %-6s %8s  %-38s %d" %
              (r["tag"], r["rep"], fmt(r["valid"]), fmt(r["track"]), arr_s, r["ramp_n"]))

    print()
    print("=" * 100)
    print("B. 阶跃段指标 (替代口径: 回稳/阻尼)")
    print("=" * 100)
    print("%-12s %-4s  %-42s %8s %8s %8s" %
          ("tag", "rep", "step arrival (0.5/1/2/2.8s)", "pp", "max_dev", "resid"))
    for r in rows:
        arr = r["step_arrival"] or {}
        arr_s = " / ".join(fmt(arr.get(k)) for k in sorted(arr, key=float)) if arr else "-"
        print("%-12s %-4d  %-42s %8s %8s %8s" %
              (r["tag"], r["rep"], arr_s, fmt(r["step_pp"]),
               "-", fmt(r["steady_resid"])))

    print()
    print("=" * 100)
    print("C. 四指标状态 (任务卡口径)")
    print("=" * 100)
    for label, key in (("锯齿包络", "envelope"), ("滑跳事件谱", "slip"),
                       ("速度纹波", "ripple")):
        states = set(r[key]["status"] for r in rows)
        ns = sorted(set(r[key].get("n", 0) for r in rows))
        print("  %-10s: %s (每轮点数 n=%s)" % (label, "/".join(states), ns))
    print("  回稳时间  : INSUFFICIENT (需斜坡终点后亚秒序列; 替代口径见 B 段)")

    print()
    print("=" * 100)
    print("D. S2 vs 电流模式 对照 (可算指标并排 — 裁决锚点)")
    print("=" * 100)
    def arm_stats(prefixes):
        out = {}
        for r in rows:
            if any(r["tag"].startswith(p) for p in prefixes):
                out.setdefault(r["tag"], []).append(r)
        return out
    print("%-12s %10s %10s %10s %10s %10s" %
          ("tag", "track%", "ramp3.5", "step2.8", "steady_pp", "resid"))
    for r in rows:
        arr = r["ramp_arrival"] or {}
        sarr = r["step_arrival"] or {}
        k35 = next((k for k in arr if abs(float(k) - 3.5) < 0.1), None)
        k28 = next((k for k in sarr if abs(float(k) - 2.8) < 0.1), None)
        print("%-12s %10s %10s %10s %10s %10s" %
              (r["tag"], fmt(r["track"]),
               fmt(arr.get(k35) if k35 else None),
               fmt(sarr.get(k28) if k28 else None),
               fmt(r["steady_pp"]), fmt(r["steady_resid"])))

    print()
    print("=" * 100)
    print("出口判据 (任务卡预先钉死)")
    print("=" * 100)
    print("  S2 包络/纹波 ≤ 电流模式 2× → 同量级, 落 (a) 关账")
    print("  显著超 2× → 列应用场景黑名单, 落 (b) 立项")
    print("  → 当前: 包络/纹波 INSUFFICIENT (采样分辨力), 判据无法评估。")
    print("     唯一可对照的 track: S2 [142.0,146.9,126.9,133.1,139.7,143.1,124.2]")
    print("     vs 电流 [143.7,128.9,97.5,97.5,90.7,90.5,98.3,98.5] — 见 D 段。")


if __name__ == "__main__":
    main()
