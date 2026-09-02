#!/usr/bin/env python3
"""COG LUT 相位对齐扫描 v2 — 角度域 LSQ 指标 + PREF 锚点接力

v1 教训 (2026-08-20 台架数据复盘):
  1. PREF 锚点未接力: 每段采集都跳回第一段锚点 => 每段都是位置阶跃回拉瞬态,
     扫出来的 "最优相位" (电机卡死 cog_amp=0) 和 th_pp 20-34° 全是脚本伪影。
     v2: 锚点随每次 PREF 写入按实际 dt 增量推进, 全程连续斜坡无跳变。
  2. 时域谱指标在 10°/s 分辨率不足: 22x机速=0.61Hz 处本来就有大谱能量
     (速度差分噪声+漂移), OFF 基线 cog_amp 8-10 淹没有对消信号。
     v2: 换角度域 — PDB iq_cmd (FF前位置环输出) = 摩擦+残余齿槽, 对原始采样点做
     sin/cos(22θ)+sin/cos(44θ)+漂移 最小二乘拟合, amp22 即残余齿槽电流。
     对速度波动免疫, 与标定测量同构 (OFF 基线应≈标定值 0.0089A, 内置自检)。

相位帧关系 (代码实锤): 标定按 theta_user 分 bin, 固件按 theta_mech+phi 索引且
  bin0 在 -180° => phi* = (180 - zero_deg) mod 360 (zero=mech_zero_offset,
  CMD:FAULT_DETAIL 的 ThetaDiag zero= 字段读出)。环滞后等二阶量由扫描收编。
  齿槽 22/rev => ripple(phi) 以 16.36° 为周期, 扫 phi0±9° 覆盖全部等价类。

前置: 固件烧入 cog_lut_rebuild.py 生成的平滑 LUT (谐波重构, 锯齿版任意相位都是扰动)。

用法: python scripts/cog_phase_sweep.py COM10 --power-ok
      [--speed 5] [--secs 12] [--gain 1.0] [--center DEG(跳过FAULT_DETAIL)]
      [--verify-speeds 0.5,2,10,30,50] [--skip-recon] [--out xxx.json]
"""
import argparse
import json
import math
import os
import re
import sys
import time

import numpy as np
import serial

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
COG_ORDER = 22.0
COG_PERIOD_DEG = 360.0 / COG_ORDER   # 16.36° — ripple(phi) 周期
CALIB_AMP22 = 0.00892                # 标定 JSON 的 22 阶幅值 (OFF 基线自检锚点)


class Capturer:
    """PDB 文本流采集 (ripple_diag.py 同构, 仅 text 模式)"""

    def __init__(self, ser):
        self.ser = ser
        self.tbuf = ""
        self.pdb = []   # (label, tick, theta, err, iq_cmd, ff_total)
        self.label = ""

    def drain(self):
        n = self.ser.in_waiting
        if not n:
            return
        self.tbuf += self.ser.read(n).decode(errors="replace")
        lines = self.tbuf.split("\n")
        self.tbuf = lines.pop()
        for l in lines:
            l = l.strip()
            if l.startswith("PDB,"):
                p = l.split(",")
                if len(p) >= 12:
                    try:
                        self.pdb.append((self.label, int(p[11]), float(p[10]),
                                         float(p[1]), float(p[7]), float(p[9])))
                    except ValueError:
                        pass


def unwrap_arr(a, period):
    d = np.diff(a)
    d[d > period / 2.0] -= period
    d[d < -period / 2.0] += period
    return np.concatenate(([a[0]], a[0] + np.cumsum(d)))


def analyze_seg(rows, trim=1.0, min_periods=1.5):
    """PDB 段 -> dict(v_mean, v_std, th_pp, amp22, amp44, travel, suspect)
    amp22/amp44: iq_cmd 对 sin/cos(k·θ) 的角度域 LSQ 幅值 (A); 行程不足为 None"""
    tick = np.array([r[1] for r in rows], dtype=float)
    th = np.array([r[2] for r in rows], dtype=float)
    iq = np.array([r[4] for r in rows], dtype=float)
    if len(tick) < 100:
        return None
    t = (tick - tick[0]) / 2000.0
    m = t >= trim
    t, th, iq = t[m], th[m], iq[m]
    if t[-1] - t[0] < 3.0:
        return None
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    thu = unwrap_arr(th, 2.0 * math.pi)          # 连续 rad (用户帧, 仅用于漂移项/速度)
    travel = (thu[-1] - thu[0]) * RAD2DEG
    v_mean = float(np.polyfit(t, thu * RAD2DEG, 1)[0])
    # 副指标: 速度 std (25ms 窗) + 位置纹波
    th_deg = thu * RAD2DEG
    w = max(3, int(round(0.025 / dt)))
    v = np.gradient(np.convolve(th_deg, np.ones(w) / w, mode="same"), dt)
    v_std = float(np.std(v - np.polyval(np.polyfit(t, v, 1), t)))
    th_res = th_deg - np.polyval(np.polyfit(t, th_deg, 1), t)
    th_pp = float(np.subtract(*np.percentile(th_res, [99, 1])))
    # 角度域 LSQ: sin/cos 用原始 (wrap 安全: 整数阶次对 2π 周期连续)
    periods = abs(travel) / COG_PERIOD_DEG
    if periods >= min_periods:
        A = np.column_stack([np.sin(22.0 * th), np.cos(22.0 * th),
                             np.sin(44.0 * th), np.cos(44.0 * th),
                             np.ones_like(th), thu - np.mean(thu)])
        coef, *_ = np.linalg.lstsq(A, iq, rcond=None)
        amp22 = float(np.hypot(coef[0], coef[1]))
        amp44 = float(np.hypot(coef[2], coef[3]))
    else:
        amp22 = amp44 = None
    return {"v_mean": v_mean, "v_std": v_std, "th_pp": th_pp,
            "amp22": amp22, "amp44": amp44, "travel": float(travel),
            "rows": int(len(t)), "suspect": False}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--speed", type=float, default=5.0, help="扫描速度 °/s (匹配标定工况)")
    ap.add_argument("--secs", type=float, default=12.0, help="每测点采集时长 s")
    ap.add_argument("--gain", type=float, default=1.0, help="COG gain (平滑 LUT 已是 A 单位)")
    ap.add_argument("--center", type=float, default=None,
                    help="扫描中心 phi° (缺省: FAULT_DETAIL 读 zero 自动算)")
    ap.add_argument("--span", type=float, default=9.0, help="粗扫半宽° (>=8.18 覆盖全等价类)")
    ap.add_argument("--step", type=float, default=1.5)
    ap.add_argument("--fine-step", type=float, default=0.5)
    ap.add_argument("--no-fine", action="store_true")
    ap.add_argument("--verify-speeds", default="0.5,2,10,30,50", help="空串跳过验证矩阵")
    ap.add_argument("--skip-recon", action="store_true", help="C 通道硬件已修时用")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if not args.power_ok:
        print("DRY-RUN: 加 --power-ok 上台架")
        return 0

    ser = serial.Serial(args.port, 1000000, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def expect(cmd, prefix, timeout=1.5):
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.strip().startswith(prefix):
                        return True
            else:
                time.sleep(0.01)
        return False

    def send(cmd):
        ser.write((cmd + "\n").encode())
        time.sleep(0.02)

    def abort(msg):
        print("ABORT: %s" % msg)
        for c in ("CMD:COG_CFG,0.000,0.0", "CMD:SREF,0", "CMD:POSDBG,0",
                  "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()
        sys.exit(1)

    def read_zero_offset():
        ser.reset_input_buffer()
        ser.write(b"CMD:FAULT_DETAIL\n")
        dl = time.time() + 2.5
        buf = ""
        while time.time() < dl:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode(errors="replace")
                if "ThetaDiag" in buf and "enc_dir" in buf:
                    break
            else:
                time.sleep(0.02)
        m = re.search(r"zero=(-?\d+\.?\d*)\s*rad", buf)
        return float(m.group(1)) if m else None

    cap = Capturer(ser)
    relay = {"tgt": None, "t_last": 0.0}

    def ramp_tick(v_dps):
        """增量推进锚点 (按实际 dt), 全程连续斜坡 — v1 跳变 bug 的修复点"""
        now = time.time()
        if now - relay["t_last"] >= 0.05:
            if relay["t_last"] > 0:
                relay["tgt"] += v_dps * DEG2RAD * (now - relay["t_last"])
            ser.write(("CMD:PREF,%.6f\n" % relay["tgt"]).encode())
            relay["t_last"] = now

    def run_at(label, v_dps, secs):
        if relay["tgt"] is None:
            for r in reversed(cap.pdb):
                if not r[0].startswith("__"):
                    relay["tgt"] = r[2]
                    break
            if relay["tgt"] is None:
                abort("PDB 流空, 无起始角度")
        cap.label = label
        t0 = time.time()
        while time.time() - t0 < secs:
            ramp_tick(v_dps)
            cap.drain()
            time.sleep(0.001)
        cap.drain()
        rows = [r for r in cap.pdb if r[0] == label]
        a = analyze_seg(rows)
        if a is None:
            print("  %-24s 样本不足" % label)
            return None
        if abs(a["v_mean"]) < 0.5 * abs(v_dps):
            a["suspect"] = True
        a22 = "%.4f" % a["amp22"] if a["amp22"] is not None else " n/a  "
        a44 = "%.4f" % a["amp44"] if a["amp44"] is not None else " n/a  "
        print("  %-24s v=%+7.2f°/s  amp22=%s  amp44=%s  th_pp=%5.2f°  v_std=%6.1f%s"
              % (label, a["v_mean"], a22, a44, a["th_pp"], a["v_std"],
                 "  SUSPECT(掉队)" if a["suspect"] else ""), flush=True)
        return a

    # ── 配置 (与标定同工况: 定版参数 + 重构) ──
    for c, p in [("CMD:UNLOCK,1", "UNLOCK,OK"),
                 ("CMD:POS_DIRECT_GAIN,0.490,0.007", "POS_DIRECT_GAIN,OK"),
                 ("CMD:POS_DIRECT_KI,0.37", "POS_DIRECT_KI,OK"),
                 ("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"),
                 ("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"),
                 ("TELEM:OFF", "TELEM:OFF,OK")]:
        if not expect(c, p):
            abort("配置失败: %s" % c)
    if not args.skip_recon:
        send("CMD:CH_CFG,1.00,1")
        time.sleep(0.2)
    send("CMD:COG_CFG,0.000,0.0")

    # ── z -> phi0 ──
    z_rad = None
    if args.center is not None:
        phi0 = args.center % 360.0
        print("扫描中心: phi0=%.2f° (手动)" % phi0)
    else:
        z_rad = read_zero_offset()
        if z_rad is None:
            abort("FAULT_DETAIL 未解析到 zero= — 用 --center 手动给")
        phi0 = (180.0 - z_rad * RAD2DEG) % 360.0
        print("mech_zero_offset z=%.3f rad (%.1f°) -> phi0=(180-z)%%360=%.2f°"
              % (z_rad, z_rad * RAD2DEG, phi0))

    # ── PDB tick 列检查 ──
    expect("CMD:POSDBG,1", "POSDBG,OK")
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.5)
    has_tick = False
    if ser.in_waiting:
        for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
            if l.startswith("PDB,") and len(l.split(",")) >= 12:
                has_tick = True
    if not has_tick:
        abort("PDB 无 tick 列 — 需 2026-08-20+ 固件")

    # ── 进 pos_direct ──
    send("CMD:STOP")
    time.sleep(0.3)
    send("CMD:CLEAR_FAULT")
    if not expect("CMD:MODE,2", "MODE,OK"):
        abort("MODE,2 失败")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        abort("ENABLE 失败")
    time.sleep(0.8)
    expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    time.sleep(0.5)
    cap.drain()

    results = {"phi0": phi0, "zero_rad": z_rad, "gain": args.gain,
               "speed": args.speed, "sweep": [], "fine": [], "verify": []}
    try:
        print("\n── OFF 基线(前) @ %.1f°/s ──" % args.speed)
        a_off_pre = run_at("off.pre", args.speed, args.secs)

        n_side = int(round(args.span / args.step))
        coarse = [phi0 + k * args.step for k in range(-n_side, n_side + 1)]
        print("\n── 粗扫 phi %.1f° ± %.1f° (步进 %.1f°) @ %.1f°/s ──"
              % (phi0, args.span, args.step, args.speed))
        for phi in coarse:
            send("CMD:COG_CFG,%.3f,%.2f" % (args.gain, phi % 360.0))
            a = run_at("phi.%.2f" % phi, args.speed, args.secs)
            if a:
                results["sweep"].append({"phi": phi % 360.0, **a})
        cand = [r for r in results["sweep"] if r["amp22"] is not None and not r["suspect"]]
        if not cand:
            abort("粗扫无有效数据 (全部样本不足或掉队)")
        best = min(cand, key=lambda r: r["amp22"])
        print("粗扫最优: phi=%.2f°  amp22=%.4fA" % (best["phi"], best["amp22"]))

        if not args.no_fine:
            nf = int(round(args.step / args.fine_step))
            print("\n── 细扫 %.2f° ± %.1f° (步进 %.2f°) ──" % (best["phi"], args.step, args.fine_step))
            for k in range(-nf, nf + 1):
                phi = best["phi"] + k * args.fine_step
                if any(abs(r["phi"] - phi % 360.0) < 0.01 for r in results["sweep"]):
                    continue
                send("CMD:COG_CFG,%.3f,%.2f" % (args.gain, phi % 360.0))
                a = run_at("fine.%.2f" % phi, args.speed, args.secs)
                if a:
                    results["fine"].append({"phi": phi % 360.0, **a})
            cand = [r for r in results["sweep"] + results["fine"]
                    if r["amp22"] is not None and not r["suspect"]]
            best = min(cand, key=lambda r: r["amp22"])
            print("细扫最优: phi=%.2f°  amp22=%.4fA" % (best["phi"], best["amp22"]))

        phi_star = best["phi"]
        results["phi_star"] = phi_star

        print("\n── OFF 基线(后, 漂移检查) ──")
        send("CMD:COG_CFG,0.000,%.2f" % phi_star)
        a_off_post = run_at("off.post", args.speed, args.secs)

        vspeeds = [float(x) for x in args.verify_speeds.split(",") if x.strip()]
        if vspeeds:
            print("\n── 验证矩阵 (COG ON phi=%.2f° vs OFF) ──" % phi_star)
            for v in vspeeds:
                secs_v = 20.0 if abs(v) < 1.0 else (8.0 if abs(v) >= 30.0 else args.secs)
                send("CMD:COG_CFG,%.3f,%.2f" % (args.gain, phi_star))
                a_on = run_at("v%.1f.on" % v, v, secs_v)
                send("CMD:COG_CFG,0.000,%.2f" % phi_star)
                a_off = run_at("v%.1f.off" % v, v, secs_v)
                if a_on and a_off:
                    if a_on["amp22"] is not None and a_off["amp22"]:
                        ratio = a_on["amp22"] / a_off["amp22"]
                        verdict = "改善" if ratio < 0.7 else ("恶化" if ratio > 1.3 else "持平")
                        detail = "amp22 %.4f->%.4f" % (a_off["amp22"], a_on["amp22"])
                    else:
                        ratio = a_on["th_pp"] / max(a_off["th_pp"], 1e-9)
                        verdict = "改善" if ratio < 0.7 else ("恶化" if ratio > 1.3 else "持平")
                        detail = "th_pp %.2f->%.2f (行程不足一轮齿槽, 用位置纹波)" % (
                            a_off["th_pp"], a_on["th_pp"])
                    print("  >> %5.1f°/s: %s  %s" % (v, detail, verdict))
                    results["verify"].append({"speed": v, "on": a_on, "off": a_off,
                                              "verdict": verdict})

        print("\n════════ 结果 ════════")
        print("phi* = %.2f°  (gain=%.2f, amp22=%.4fA)" % (phi_star, args.gain, best["amp22"]))
        if z_rad is not None:
            dev = (phi_star - phi0 + COG_PERIOD_DEG / 2) % COG_PERIOD_DEG - COG_PERIOD_DEG / 2
            print("解析 phi0 = %.2f°, 实测偏差 %+.2f° (mod %.1f°) — %s"
                  % (phi0, dev, COG_PERIOD_DEG,
                     "帧分析成立" if abs(dev) <= 2.0 else "偏差>2°, 有未建模项(环滞后等)"))
        if a_off_pre and a_off_pre["amp22"]:
            print("OFF 基线 amp22 = %.4fA vs 标定 %.5fA — %s"
                  % (a_off_pre["amp22"], CALIB_AMP22,
                     "测量链自检通过" if abs(a_off_pre["amp22"] - CALIB_AMP22) < 0.003
                     else "与标定偏差大, 工况不一致?"))
        if a_off_pre and a_off_post and a_off_pre["amp22"]:
            drift = abs(a_off_post["amp22"] - a_off_pre["amp22"]) / a_off_pre["amp22"]
            print("OFF 漂移: %.4f -> %.4f (%.0f%%) %s"
                  % (a_off_pre["amp22"], a_off_post["amp22"], 100 * drift,
                     "" if drift < 0.3 else "— 漂移大, 扫描可信度打折"))
        print("推荐: CMD:COG_CFG,%.3f,%.2f" % (args.gain, phi_star))
        print("注意: phi* 依赖 mech_zero_offset — 重新 HOME 后须重扫")
    finally:
        print("\n清理: COG OFF, 流关, STOP")
        for c in ("CMD:COG_CFG,0.000,0.0", "CMD:SREF,0", "CMD:POSDBG,0",
                  "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "cog_phase_sweep_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print("JSON: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
