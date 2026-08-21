#!/usr/bin/env python3
"""齿槽 LUT 谐波重构 — 从标定 JSON 提取主导谐波, 生成平滑 LUT 头文件

动机 (2026-08-20): 原始逐 bin LUT 噪声与信号同级 (pp 0.040A vs 22 阶幅值 0.0089A,
~6.8 样本/bin), 相邻 bin 阶跃达 0.03A — 注入即扰动 (台架: 任意相位 th_pp 爆表)。
只保留主导谐波 (默认 22/44 阶) 重构平滑波形: rfft 滤波严格保留原波形的
谐波幅值/相位 (对消正确性不变), 逐 bin 噪声清零。

流水线: cogging_calibrate.py -> JSON -> 本脚本 -> cogging_lut_cal.h (编译入固件)
相位帧注意: LUT bin 帧 = theta_user (theta_mech - mech_zero_offset), 固件索引帧 =
theta_mech + phase_offset 且 bin0 在 -180°。对齐靠运行时 COG_CFG phase
(phi* = (180 - zero_deg) mod 360 + 微调, 用 cog_phase_sweep.py 实测定位)。

用法: python scripts/cog_lut_rebuild.py [--json scripts/cogging_lut_xxx.json]
      [--harmonics 22,44] [--out MDK-ARM/code/cogging_lut_cal.h] [--dry-run]
"""
import argparse
import glob
import json
import os
import re
import sys
import time

import numpy as np


def latest_json(scripts_dir):
    cands = sorted(glob.glob(os.path.join(scripts_dir, "cogging_lut_*.json")))
    return cands[-1] if cands else None


def harm_stats(x, n, k):
    """k 阶 (次/圈) 幅值/相位, 与 cogging_calibrate.py 的 FFT 口径一致"""
    re = float(np.sum(x * np.cos(2 * np.pi * k * np.arange(n) / n)))
    im = float(np.sum(x * np.sin(2 * np.pi * k * np.arange(n) / n)))
    return 2.0 * np.hypot(re, im) / n, np.degrees(np.arctan2(im, re))


def parse_header(path):
    """读现有头文件的 float 数组 (校验流水线约定用); 不存在返回 None"""
    if not os.path.exists(path):
        return None
    txt = open(path, "r", encoding="utf-8").read()
    m = re.search(r"COGGING_LUT_CAL\[\d+\]\s*=\s*\{(.*?)\};", txt, re.S)
    if not m:
        return None
    vals = [float(v) for v in re.findall(r"(-?\d+\.?\d*)f", m.group(1))]
    return np.array(vals) if vals else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None, help="标定 JSON (缺省取 scripts/ 最新)")
    ap.add_argument("--harmonics", default="22,44", help="保留阶次 (次/圈, 逗号分隔)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    jpath = args.json or latest_json(scripts_dir)
    if not jpath:
        print("找不到 cogging_lut_*.json")
        return 1
    out = args.out or os.path.normpath(os.path.join(
        scripts_dir, "..", "MDK-ARM", "code", "cogging_lut_cal.h"))

    d = json.load(open(jpath, "r", encoding="utf-8"))
    cog = np.array(d["cog"], dtype=float)   # 齿槽力矩电流 (dc 已去, 均值≈0)
    n = len(cog)
    keep = [int(x) for x in args.harmonics.split(",") if x.strip()]
    print("源: %s (n=%d bins, cog_pp=%.4fA, dc=%.4fA)"
          % (os.path.basename(jpath), n, d["cog_pp"], d["dc"]))

    # ── rfft 谐波滤波: 只保留指定阶次 ──
    X = np.fft.rfft(cog)
    mask = np.zeros_like(X)
    for k in keep:
        if 0 < k < len(mask):
            mask[k] = 1.0
    cog_s = np.fft.irfft(X * mask, n=n)
    lut_s = -cog_s                       # 前馈 = 负齿槽 (与 calibrate 口径一致)

    # ── 校验: 保留阶次的幅值/相位必须与原始波形一致 ──
    print("\n阶次保留校验 (原始 vs 重构, 应严格一致):")
    for k in keep:
        a0, p0 = harm_stats(cog, n, k)
        a1, p1 = harm_stats(cog_s, n, k)
        print("  %3d 次/圈: %.5fA@%.1f°  ->  %.5fA@%.1f°  %s"
              % (k, a0, p0, a1, p1, "OK" if abs(a0 - a1) < 1e-9 else "MISMATCH"))

    raw_step = float(np.max(np.abs(np.diff(np.r_[cog, cog[0]]))))
    new_step = float(np.max(np.abs(np.diff(np.r_[cog_s, cog_s[0]]))))
    resid = cog - cog_s
    print("\n波形: pp %.4f -> %.4fA | 相邻bin最大阶跃 %.4f -> %.4fA | 剔除噪声 std %.4fA"
          % (np.ptp(cog), np.ptp(cog_s), raw_step, new_step, float(np.std(resid))))
    print("LUT 范围: %.5f ~ %.5fA (零均值, 摩擦 dc 由 FRIC_COMP 独立承担)"
          % (float(np.min(lut_s)), float(np.max(lut_s))))

    # ── 与现有头文件对拍 (验证"头文件=-cog"的流水线约定) ──
    old = parse_header(out)
    if old is not None and len(old) == n:
        old_lut_json = -cog
        c_hl = float(np.corrcoef(old, old_lut_json)[0, 1])
        c_hn = float(np.corrcoef(old, lut_s)[0, 1])
        print("\n现有头文件对拍: vs JSON lut 相关 %.3f %s | vs 新平滑 lut 相关 %.3f"
              % (c_hl, "(头文件=原始lut, 约定确认)" if c_hl > 0.99 else
                 "(!! 头文件≠原始lut, 流水线另有变换, 先查再写)", c_hn))
        if c_hl <= 0.99:
            if not args.dry_run:
                print("ABORT: 约定未确认, 不写文件 (用 --dry-run 查看)")
            return 1
    elif old is not None:
        print("WARN: 现有头文件长度 %d ≠ %d, 跳过对拍" % (len(old), n))

    if args.dry_run:
        print("\nDRY-RUN, 未写文件")
        return 0

    ts = time.strftime("%Y-%m-%d")
    lines = ["#ifndef __COGGING_LUT_CAL_H",
             "#define __COGGING_LUT_CAL_H",
             ("/* 齿槽 LUT (谐波重构平滑版) — %s cog_lut_rebuild.py" % ts),
             (" * 数据源: scripts/%s (谐波 %s 阶, rfft 滤波; 原始锯齿版见 git 历史)"
              % (os.path.basename(jpath), "/".join(str(k) for k in keep))),
             (" * LUT = -cogging_smooth(theta_user 帧 bin); 运行时相位对齐: COG_CFG gain,phi */"),
             "static const float COGGING_LUT_CAL[%d] = {" % n]
    for i in range(0, n, 12):
        lines.append("    " + ",".join("%.5ff" % v for v in lut_s[i:i + 12]) + ",")
    lines[-1] = lines[-1].rstrip(",")
    lines += ["};", "", "#endif /* __COGGING_LUT_CAL_H */", ""]
    open(out, "w", encoding="utf-8", newline="\n").write("\n".join(lines))
    print("\n已写入: %s (%d bins)" % (out, n))
    print("下一步: 重新编译烧录, 然后 cog_phase_sweep.py 定位 phi*")
    return 0


if __name__ == "__main__":
    sys.exit(main())
