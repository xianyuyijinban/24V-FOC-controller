#!/usr/bin/env python3
"""全速域指数扫频 + 滞回对比 — 快->慢->快 (默认 1 rad/s -> 0.1°/s -> 1 rad/s)

目的 (2026-08-21): COG LUT 定版 (平滑版 + phi*=0°) + CH_CFG 重构后的全速域体检。
  - 指数扫频: 每数量级等时, 低速过渡区 (粘滑/环带宽穿越) 分辨率与高速段一致
  - 下行 vs 上行同速档配对: 滞回 (Stribeck/方向依赖) 检测
  - PREF 步长自适应 (20-100Hz, 步长<=0.3°): 消除阶梯伪影的速度缩放

指标 (PDB 流, 2kHz tick 硬件时基, 1s 窗/0.5s 步):
  - err_*    : 位置环跟随误差 (PDB p[1], control 帧 rad->deg) — 低速段主判据
  - v_std    : 25ms 窗速度 std (+100ms 对照; 低速段被编码器量化主导, 仅供参考)
  - th_pp    : 位置纹波 1-99% 摆幅 (窗内线性去趋势)
  - amp22    : iq_cmd 角度域 LSQ 22 阶幅值 (窗内行程>=25° 才有效; COG ON 时应~0.001A)

用法: python scripts/speed_sweep.py COM10 --power-ok
      [--vmin 0.1 --vmax 57.3] [--t-sweep 40] [--hold 8]
      [--cog-off] [--skip-recon] [--out xxx.json]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import serial

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
COG_PERIOD_DEG = 360.0 / 22.0


class Capturer:
    """PDB 文本流采集 (与 cog_phase_sweep.py 同构)"""

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


def analyze_win(rows):
    """单个窗口 PDB rows -> dict 或 None"""
    tick = np.array([r[1] for r in rows], dtype=float)
    th = np.array([r[2] for r in rows], dtype=float)
    err = np.array([r[3] for r in rows], dtype=float)
    iq = np.array([r[4] for r in rows], dtype=float)
    if len(tick) < 60:
        return None
    t = (tick - tick[0]) / 2000.0
    if t[-1] - t[0] < 0.5:
        return None
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    thu = unwrap_arr(th, 2.0 * math.pi)
    th_deg = thu * RAD2DEG
    travel = float(th_deg[-1] - th_deg[0])
    v_mean = float(np.polyfit(t, th_deg, 1)[0])
    w25 = max(3, int(round(0.025 / dt)))
    v = np.gradient(np.convolve(th_deg, np.ones(w25) / w25, mode="same"), dt)
    v_res = v - np.polyval(np.polyfit(t, v, 1), t)
    v_std = float(np.std(v_res))
    w100 = max(3, int(round(0.100 / dt)))
    v100 = np.gradient(np.convolve(th_deg, np.ones(w100) / w100, mode="same"), dt)
    v_std100 = float(np.std(v100 - np.polyval(np.polyfit(t, v100, 1), t)))
    th_res = th_deg - np.polyval(np.polyfit(t, th_deg, 1), t)
    th_pp = float(np.subtract(*np.percentile(th_res, [99, 1])))
    err_deg = err * RAD2DEG
    amp22 = None
    if abs(travel) >= 1.5 * COG_PERIOD_DEG:
        A = np.column_stack([np.sin(22.0 * th), np.cos(22.0 * th),
                             np.sin(44.0 * th), np.cos(44.0 * th),
                             np.ones_like(th), thu - np.mean(thu)])
        coef, *_ = np.linalg.lstsq(A, iq, rcond=None)
        amp22 = float(np.hypot(coef[0], coef[1]))
    return {"t_mid": float(t[0] + (t[-1] - t[0]) / 2), "v_mean": v_mean,
            "travel": travel, "v_std": v_std, "v_std100": v_std100,
            "th_pp": th_pp, "err_mean": float(np.mean(err_deg)),
            "err_std": float(np.std(err_deg)),
            "err_p99": float(np.percentile(np.abs(err_deg), 99)),
            "amp22": amp22, "rows": int(len(t))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--vmin", type=float, default=0.1, help="扫频低端 °/s")
    ap.add_argument("--vmax", type=float, default=57.3, help="扫频高端 °/s (1 rad/s)")
    ap.add_argument("--t-sweep", type=float, default=40.0, help="单程扫频时长 s")
    ap.add_argument("--hold", type=float, default=8.0, help="低端停留时长 s")
    ap.add_argument("--spin", type=float, default=2.0, help="起步加速/收尾减速时长 s")
    ap.add_argument("--cog-off", action="store_true", help="对照组: COG 关闭")
    ap.add_argument("--skip-recon", action="store_true", help="C 通道硬件已修时用")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    vmin, vmax = abs(args.vmin), abs(args.vmax)
    if not (0.01 <= vmin < vmax <= 200.0):
        print("参数范围错误: 需要 0.01 <= vmin < vmax <= 200 (°/s)")
        return 1
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

    cap = Capturer(ser)

    # ── 配置前先停 N/C 流: 20kHz 下 N/C 帧密集, 会淹没 expect 响应 ──
    # (2026-08-30 实测: UNLOCK/FRIC_COMP 等 expect 在流开时超时失败)
    send("CMD:OFF")
    send("TELEM:CUR,OFF")
    send("CMD:POSDBG,0")
    time.sleep(0.3)

    # ── 配置 (当前定版) ──
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
    send("CMD:COG_CFG,0.000,0.0" if args.cog_off else "CMD:COG_CFG,1.000,0.0")
    time.sleep(0.2)

    # PDB tick 列检查
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

    # ── live LUT 身份 preflight (Flash 遮蔽守门) ──
    # 2026-08-30 教训: CAL:ALL 重识别会重新生成内部拖拽 LUT 并写 Flash,
    # 下次上电遮蔽编译版 -> "COG ON" 实际注入幅值标尺错误的旧类 LUT。
    # 必须先停 N/C 遥测流: JDIAG 走 P1 队列, 不暂停 N/C 会被淹没导致假阴性 (jdiag=null)。
    send("CMD:OFF")
    send("TELEM:CUR,OFF")
    send("CMD:POSDBG,0")   # PDB 流也是 P1, 不停会挤掉 JDIAG 响应 (2026-08-30 实测)
    time.sleep(0.3)
    ser.reset_input_buffer()
    jdiag_line = None
    ser.write(b"CMD:JDIAG\n")
    dl = time.time() + 5.0
    jbuf = ""
    while time.time() < dl:
        if ser.in_waiting:
            jbuf += ser.read(ser.in_waiting).decode(errors="replace")
            if "JDIAG," in jbuf:
                break
        else:
            time.sleep(0.02)
    print("preflight jbuf=%r" % jbuf[:120])
    for l in jbuf.replace("\r", "").split("\n"):
        if l.strip().startswith("JDIAG,"):
            jdiag_line = l.strip()
            break
    if jdiag_line:
        try:
            kv = dict(f.split("=", 1) for f in jdiag_line.split(",") if "=" in f)
            cmin, cmax = float(kv["cog_min"]), float(kv["cog_max"])
            is_compiled = abs(cmin + 0.0093) < 0.002 and abs(cmax - 0.0133) < 0.002
            print("live LUT: min=%.4f max=%.4f -> %s" % (cmin, cmax,
                  "编译平滑版 OK" if is_compiled else
                  "!! 非编译版 (Flash 遮蔽?) — 先 CMD:COG_LUT,USE_COMPILED 再扫"))
            if not is_compiled and not args.cog_off:
                abort("live LUT 非编译平滑版 — 中止 (或先 COG_LUT,USE_COMPILED)")
        except (KeyError, ValueError):
            # 2026-08-30 终审: fail-open -> fail-closed, 身份未确认 + COG ON 必须中止
            if args.cog_off:
                print("WARN: JDIAG cog_min/max 解析失败 (COG OFF 放行): %s" % jdiag_line)
            else:
                abort("JDIAG cog_min/max 解析失败 — LUT 身份未确认, COG ON 中止: %s" % jdiag_line)
    else:
        # 2026-08-30 终审: 无响应只 WARN 是洞 (112716 run jdiag=null 实证, 守门形同虚设)
        if args.cog_off:
            print("WARN: JDIAG 无响应 (COG OFF 放行)")
        else:
            abort("JDIAG 无响应 — live LUT 身份未确认, COG ON 中止")

    send("CMD:ON")
    send("CMD:POSDBG,1")
    time.sleep(0.3)

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

    # ── 速度剖面 (解析积分, 绝对 PREF, 无累积漂移) ──
    # 阶段: spin(线性加速 0->vmax) -> down(指数 vmax->vmin) -> hold(vmin)
    #       -> up(指数 vmin->vmax) -> decel(线性 vmax->0)
    r = vmin / vmax
    T, TH, TS = args.t_sweep, args.hold, args.spin
    ln_r = math.log(r)

    def profile(t):
        """t(s) -> (theta_offset rad, v_dps); 从锚点起的累计位移与即时速度"""
        if t < 0.0:
            return 0.0, 0.0
        if t < TS:                       # spin: v = vmax·t/TS
            th = 0.5 * vmax * t * t / TS
            return th * DEG2RAD, vmax * t / TS
        t -= TS
        if t < T:                        # down: v = vmax·e^(ln_r·t/T)
            k = ln_r / T
            th = 0.5 * vmax * TS + vmax * (math.exp(k * t) - 1.0) / k
            return th * DEG2RAD, vmax * math.exp(k * t)
        t -= T
        th_base = 0.5 * vmax * TS + vmax * (r - 1.0) / (ln_r / T)
        if t < TH:                       # hold
            return (th_base + vmin * t) * DEG2RAD, vmin
        t -= TH
        th_base += vmin * TH
        if t < T:                        # up: v = vmin·e^(-ln_r·t/T)
            k = -ln_r / T
            th = th_base + vmin * (math.exp(k * t) - 1.0) / k
            return th * DEG2RAD, vmin * math.exp(k * t)
        t -= T
        th_base += vmin * (math.exp(-ln_r) - 1.0) / (-ln_r / T)
        if t < TS:                       # decel: v = vmax·(1-t/TS)
            th = th_base + vmax * (t - 0.5 * t * t / TS)
            return th * DEG2RAD, vmax * (1.0 - t / TS)
        return (th_base + 0.5 * vmax * TS) * DEG2RAD, 0.0

    t_total = TS + T + TH + T + TS
    print("剖面: spin %.0fs -> 下行 %.0fs -> 停留 %.0fs -> 上行 %.0fs -> 减速 %.0fs (共 %.0fs)"
          % (TS, T, TH, T, TS, t_total))
    print("配置: COG %s | CH_CFG %s | AW1/0.03 comp0.022"
          % ("OFF" if args.cog_off else "ON g=1.0 phi=0",
             "OFF" if args.skip_recon else "RECON"))

    windows = []
    try:
        # 锚点
        anchor = None
        for row in reversed(cap.pdb):
            anchor = row[2]
            break
        if anchor is None:
            abort("PDB 流空, 无起始角度")

        cap.label = "sweep"
        t0 = time.time()
        last_send = 0.0
        last_prog = -10.0
        while True:
            t = time.time() - t0
            th_off, v_dps = profile(t)
            if t > t_total:
                break
            # PREF 步长自适应: 步长 <= 0.3°, 率 20-100Hz
            rate = min(100.0, max(20.0, abs(v_dps) / 0.3))
            if t - last_send >= 1.0 / rate:
                ser.write(("CMD:PREF,%.6f\n" % (anchor + th_off)).encode())
                last_send = t
            cap.drain()
            if t - last_prog >= 10.0:
                print("  t=%4.0fs  v=%8.3f°/s ..." % (t, v_dps), flush=True)
                last_prog = t
            time.sleep(0.001)
        cap.drain()
        send("CMD:STOP")
        time.sleep(0.3)
        print("采集完成, PDB 行数 %d" % len(cap.pdb))

        # ── 滑窗分析 (1s 窗 / 0.5s 步) ──
        rows = [x for x in cap.pdb if x[0] == "sweep"]
        if len(rows) < 200:
            abort("PDB 样本不足 (%d)" % len(rows))
        tick0 = rows[0][1]
        ticks = np.array([x[1] for x in rows], dtype=float)
        n = len(rows)
        i = 0
        while i < n:
            t_lo = (ticks[i] - tick0) / 2000.0
            j = i
            while j < n and (ticks[j] - tick0) / 2000.0 < t_lo + 1.0:
                j += 1
            if j == i:                # 流空隙 >1s: 逐行跳过防空转
                i += 1
                continue
            if j - i >= 60:
                a = analyze_win(rows[i:j])
                if a:
                    a["t_abs"] = t_lo + 0.5
                    _, v_prog = profile(t_lo + 0.5)
                    a["v_prog"] = v_prog
                    # 掉队守卫: 剖面速度>=1°/s 时实测偏离>30% 标记
                    a["suspect"] = bool(v_prog >= 1.0 and
                                        abs(a["v_mean"] - v_prog) > 0.3 * v_prog)
                    windows.append(a)
            i = j - (j - i) // 2      # 0.5s hop (半窗交叠)

        # ── 覆盖度守卫 (2026-08-30 终审新增) ──
        # 高速段 PDB 欠采样 -> 滑窗行数不足被丢 -> 报告 silently 缺段 (112716 run:
        # v_prog 只覆盖到 14.6°/s 却被写成"全速域 57")。覆盖不足必须大声宣告。
        coverage_ok = bool(windows) and max(w["v_prog"] for w in windows) >= 0.7 * vmax
        if not coverage_ok:
            v_cov = max((w["v_prog"] for w in windows), default=0.0)
            print("\n!! 覆盖度不足: v_prog max=%.1f°/s < 0.7×vmax(%.1f) — 高速段窗口丢失,"
                  " 结果只覆盖低速段, 禁止外推全速域" % (v_cov, vmax))

        # ── 下行/上行同速档配对 (滞回) ──
        anchors = [50, 30, 20, 10, 5, 2, 1, 0.5, 0.2, 0.1]
        t_split = TS + T + TH / 2.0   # 下行/上行分界 (停留中点)

        def nearest(v_nom, phase):
            cands = [w for w in windows
                     if (w["t_abs"] < t_split) == (phase == "down")
                     and not w["suspect"] and w["v_prog"] > 0.05]
            if not cands:
                return None
            return min(cands, key=lambda w: abs(abs(w["v_mean"]) - v_nom))

        print("\n════════ 滞回对比表 (下行 vs 上行同速档) ════════")
        print("%8s | %-39s | %-39s" % ("v°/s", "下行", "上行"))
        print("%8s | %6s %6s %6s %7s %7s | %6s %6s %6s %7s %7s"
              % ("", "v实测", "v_std", "e_std", "e_p99", "amp22",
                 "v实测", "v_std", "e_std", "e_p99", "amp22"))
        pairs = []
        for vn in anchors:
            d = nearest(vn, "down")
            u = nearest(vn, "up")
            if not d and not u:
                continue

            def fmt(w):
                if w is None:
                    return ["  --  ", "  --  ", "  --  ", "  --  ", "  --  "]
                # v实测 必须进表 — 2026-08-30 教训: 锚点标签 ≠ 窗口实际速度,
                # 丢窗时 14.6°/s 的窗会被贴到 50 锚点上抄进报告
                return ["%6.1f" % abs(w["v_mean"]), "%6.1f" % w["v_std"],
                        "%6.2f" % w["err_std"], "%6.2f" % w["err_p99"],
                        ("%7.4f" % w["amp22"]) if w["amp22"] is not None else "   --  "]
            fd, fu = fmt(d), fmt(u)
            print("%8.2f | %s %s %s %s° %sA | %s %s %s %s° %sA"
                  % (vn, fd[0], fd[1], fd[2], fd[3], fd[4],
                     fu[0], fu[1], fu[2], fu[3], fu[4]))
            pairs.append({"v_nom": vn, "down": d, "up": u})

        # ── 停留段 (vmin 深低速) ──
        hold_w = [w for w in windows
                  if TS + T < w["t_abs"] < TS + T + TH]
        if hold_w:
            es = [w["err_std"] for w in hold_w]
            ep = [w["err_p99"] for w in hold_w]
            print("\n低端停留 @ %.2f°/s (%d 窗): err_std %.2f-%.2f°  err_p99 %.2f-%.2f°"
                  % (vmin, len(hold_w), min(es), max(es), min(ep), max(ep)))

        # ── 最差窗口 ──
        bad = sorted([w for w in windows if not w["suspect"]],
                     key=lambda w: -w["err_p99"])[:5]
        print("\nerr_p99 最差 5 窗:")
        for w in bad:
            print("  t=%5.1fs  v_prog=%7.2f  v_mean=%+7.2f  err_p99=%.2f°  err_mean=%+.2f°"
                  % (w["t_abs"], w["v_prog"], w["v_mean"], w["err_p99"], w["err_mean"]))
        nsusp = sum(1 for w in windows if w["suspect"])
        if nsusp:
            print("掉队/卡死嫌疑窗口: %d 个" % nsusp)
    finally:
        print("\n清理: COG OFF, 流关, STOP")
        for c in ("CMD:COG_CFG,0.000,0.0", "CMD:SREF,0", "CMD:POSDBG,0",
                  "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "speed_sweep_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "jdiag": jdiag_line, "coverage_ok": coverage_ok,
                   "pairs": pairs, "windows": windows}, f,
                  ensure_ascii=False, indent=1)
    print("JSON: %s (%d 窗)" % (out, len(windows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
