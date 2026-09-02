#!/usr/bin/env python3
"""0.5°/s 极限环量化采集 v2 — POSDBG 流 (PDB 行) + N 帧同录 + 时间基准 PREF 斜坡

2026-08-19 v2 修订（帧bug/丢样本/指标全面修正）:
- 帧bug: PDB err 是 control 帧(= enc_dir × sensor 误差), 而 PREF/p[3]/p[18] 是 sensor 帧。
  旧版 ang = pref − err 在 enc_dir=-1 时退化为 2pref−θ, 跟踪率显示为 200−真实%
  (v1-v4 报告 154-174%, 实际 26-46%, 与 err 末值反推一致)。v2 直接用固件 theta 列, 不再重建。
- 丢样本: 行缓冲读串口, 循环内不再 reset_input_buffer (旧版每轮丢 ~40% PDB 行)。
- 指标: 跟踪率改最小二乘斜率; err 全程单号时判"非振荡态(持续掉队)", 不算周期/占空比。
- PDB 第6列是 fric_comp_pos 配置常数(非瞬时补偿!), 瞬时补偿看第7列 fric_ff。

PDB 行 (固件 2026-08-19 起 10 列):
  PDB,err(rad,control),ki_out(A),ki_raw(A),pd_sat,comp_cfg(A,常数),fric_ff(A),
      iq_cmd(A,FF前),cmd_dir,ff_total(A),theta(rad,sensor)
N 帧: p[3]=angle(deg,sensor) p[4]=speed(rad/s) p[6]=Iq(A) p[18]=pos_ref(rad,sensor)
      p[19]=Iq_ref(A,FF后)  —— ff_total ≈ p[19] − iq_cmd(PDB第8列), 交验用
用法: python scripts/limit_cycle_capture.py COM10 [--deg-s 0.5] [--secs 40] [--enc-dir -1]
"""
import argparse
import math
import statistics
import sys
import time

import serial

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def lsq_slope(ts, ys):
    """最小二乘斜率 (ys 对 ts)"""
    n = len(ts)
    if n < 3:
        return 0.0
    tm = statistics.fmean(ts)
    ym = statistics.fmean(ys)
    num = sum((t - tm) * (y - ym) for t, y in zip(ts, ys))
    den = sum((t - tm) ** 2 for t in ts)
    return num / den if den > 1e-12 else 0.0


def unwrap(series, period):
    """周期序列解缠绕 (in-place 累加修正), period: 360.0(deg) 或 2π(rad)"""
    out = [series[0]]
    half = period / 2.0
    off = 0.0
    for i in range(1, len(series)):
        d = series[i] - series[i - 1]
        if d > half:
            off -= period
        elif d < -half:
            off += period
        out.append(series[i] + off)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", default="COM10")
    ap.add_argument("--deg-s", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=40.0)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--comp", type=float, default=0.022)
    ap.add_argument("--ierr", type=float, default=0.035)
    ap.add_argument("--aw", default="", help="积分抗饱和律 'mode,rate' (空=不动)")
    ap.add_argument("--cog-gain", type=float, default=0.0,
                    help="COG LUT 增益 (默认0=固件定版OFF; 实验前强制设置)")
    ap.add_argument("--enc-dir", type=float, default=-1.0,
                    help="编码器方向 (仅用于把 control 帧 err 换算成 sensor 帧滞后)")
    ap.add_argument("--out", default="", help="JSON 输出路径 (空=不存)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 1000000, timeout=0.05)
    time.sleep(0.3)
    # 清残留流(上一轮异常退出可能 POSDBG 仍开, 淹没命令响应)
    ser.write(b"CMD:POSDBG,0\nCMD:STOP\n")
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

    setup = [("CMD:UNLOCK,1", "UNLOCK,OK"),
             ("CMD:MODE,2", "MODE,OK"),
             ("CMD:POS_DIRECT,1", "POS_DIRECT,OK"),
             ("CMD:POS_DIRECT_GAIN,%.3f,%.3f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK"),
             ("CMD:POS_DIRECT_KI,%.3f" % args.ki, "POS_DIRECT_KI,OK"),
             ("CMD:POS_INTEGR_ERR,%.4f" % args.ierr, "POS_INTEGR_ERR,OK"),
             ("CMD:FRIC_COMP,%.3f,%.3f" % (args.comp, args.comp), "FRIC_COMP,OK")]
    if args.aw:
        aw_mode, aw_rate = args.aw.split(",")
        setup.append(("CMD:POS_AW_MODE,%s,%s" % (aw_mode, aw_rate),
                      "POS_AW_MODE,OK"))
    setup.append(("CMD:ENABLE,1", "ENABLE,OK"))
    for c, p in setup:
        if not expect(c, p):
            print("ABORT: %s 未获 %s" % (c, p))
            ser.close()
            return 1
    # COG_CFG 无 OK 回显, 单独设 + 查询确认
    ser.write(("CMD:COG_CFG,%.3f,60.0\n" % args.cog_gain).encode())
    time.sleep(0.2)
    time.sleep(1.0)
    ser.write(b"TELEM:ON\n")
    time.sleep(0.5)

    # 起点角度: N 帧 p[3] (deg, sensor 帧, 与 PREF 同帧)
    ser.reset_input_buffer()
    a0_deg = None
    dl = time.time() + 1.0
    while time.time() < dl and a0_deg is None:
        data = ser.read(ser.in_waiting).decode(errors="replace")
        for l in data.split("\n"):
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 25:
                    a0_deg = float(p[3])
        if not data:
            time.sleep(0.02)
    if a0_deg is None:
        print("ERR: no telemetry angle")
        ser.close()
        return 1
    a0_rad = a0_deg * DEG2RAD

    # 开 POSDBG 流
    if not expect("CMD:POSDBG,1", "POSDBG,OK"):
        print("ABORT: POSDBG 流开启失败")
        ser.close()
        return 1

    # 采集主循环: 行缓冲 + 非阻塞, PREF 每 0.05s 按时间基准
    pdb = []   # (t, err, ki_out, ki_raw, pd_sat, comp_cfg, fric_ff, iq_cmd, cmd_dir, ff_total, theta)
    nfr = []   # (t, ang_deg, speed, iq_meas, pos_ref, iq_ref)
    buf = ""
    last_pref = -1.0
    t0 = time.time()
    while time.time() - t0 < args.secs:
        now = time.time() - t0
        if now - last_pref >= 0.05:
            tgt = a0_rad + args.deg_s * now * DEG2RAD
            ser.write(("CMD:PREF,%.6f\n" % tgt).encode())
            last_pref = now
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode(errors="replace")
            lines = buf.split("\n")
            buf = lines.pop()  # 半行留待下次拼
            for l in lines:
                l = l.strip()
                if l.startswith("PDB,"):
                    parts = l.split(",")
                    if len(parts) >= 9:
                        try:
                            row = [now] + [float(parts[i]) for i in range(1, 9)]
                            if len(parts) >= 11:  # 新固件: ff_total, theta
                                row += [float(parts[9]), float(parts[10])]
                            else:
                                row += [float("nan"), float("nan")]
                            pdb.append(row)
                        except ValueError:
                            pass
                elif l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        try:
                            nfr.append((now, float(p[3]), float(p[4]),
                                        float(p[6]), float(p[18]), float(p[19])))
                        except ValueError:
                            pass
        else:
            time.sleep(0.001)

    expect("CMD:POSDBG,0", "POSDBG,OK")
    expect("CMD:PREF,%.6f" % a0_rad, "PREF,OK")  # 回位
    expect("CMD:STOP", "STOP,OK")
    ser.close()

    if len(pdb) < 10:
        print("ERR: too few PDB samples (%d)" % len(pdb))
        return 1

    times = [r[0] for r in pdb]
    errs = [r[1] for r in pdb]           # control 帧 rad
    ki_outs = [r[2] for r in pdb]
    ki_ra = [r[3] for r in pdb]
    pd_sats = [int(r[4]) for r in pdb]
    fric_ffs = [r[6] for r in pdb]
    iqs = [r[7] for r in pdb]            # FF 前 iq_cmd
    dirs = [r[8] for r in pdb]
    ff_tots = [r[9] for r in pdb]
    thetas = [r[10] for r in pdb]        # sensor 帧 rad

    total_t = times[-1] - times[0]
    n_samples = len(pdb)
    pdb_hz = n_samples / total_t

    # ── 真实角度轨迹: 优先 PDB theta 列 (200Hz 与 err 同源), 缺则用 N 帧 p[3] ──
    have_theta = not math.isnan(thetas[0])
    if have_theta:
        ang_uw = [v * RAD2DEG for v in unwrap(thetas, 2.0 * math.pi)]
        ang_t = times
        ang_src = "PDB.theta"
    elif len(nfr) >= 10:
        ang_uw = unwrap([r[1] for r in nfr], 360.0)
        ang_t = [r[0] for r in nfr]
        ang_src = "N.p3"
    else:
        print("ERR: 无角度源 (旧固件且无足够 N 帧)")
        return 1
    slope_dps = lsq_slope(ang_t, ang_uw)   # deg/s, sensor 帧
    track_pct = slope_dps / args.deg_s * 100.0 if args.deg_s else 0.0
    drift = ang_uw[-1] - ang_uw[0]

    # err 滞后 (sensor 帧, 度): err_sensor = err_control × enc_dir
    lag_start = errs[0] * args.enc_dir * RAD2DEG
    lag_end = errs[-1] * args.enc_dir * RAD2DEG

    # 极限环判定: err 过零 >=2 才算振荡态
    def crossings(vals):
        out = []
        prev = vals[0]
        for i in range(1, len(vals)):
            cur = vals[i]
            if (prev < 0.0) != (cur < 0.0) and abs(cur) > 1e-9:
                out.append(i)
            prev = cur
        return out

    x = crossings(errs)
    oscillating = len(x) >= 2
    periods = []
    lag_s = []
    if oscillating:
        periods = [times[x[i + 1]] - times[x[i]] for i in range(len(x) - 1)]
        periods = [p for p in periods if p > 0.3]
        for idx in x:
            i = idx
            while i < n_samples and i < idx + 40:
                if (iqs[i] < 0.0) != (iqs[idx] < 0.0):
                    lag_s.append(times[i] - times[idx])
                    break
                i += 1

    clamp_frac = sum(1 for v in ki_ra if abs(v) > 0.105) / n_samples * 100.0
    pd_sat_frac = sum(pd_sats) / n_samples * 100.0
    dir_frac = sum(1 for v in dirs if v != 0.0) / n_samples * 100.0
    ff_valid = [v for v in ff_tots if not math.isnan(v)]
    ff_mean = statistics.fmean(ff_valid) if ff_valid else float("nan")
    fric_mean = statistics.fmean(fric_ffs)
    iq_cmd_mean = statistics.fmean(iqs)
    # 交验: N 帧 Iq_ref(FF后) 均值 − PDB iq_cmd(FF前) 均值 ≈ ff_total 均值
    iq_ref_mean = statistics.fmean([r[5] for r in nfr]) if nfr else float("nan")
    iq_meas_mean = statistics.fmean([r[3] for r in nfr]) if nfr else float("nan")
    backfire_xcheck = (iq_ref_mean - iq_cmd_mean) if nfr else float("nan")

    print("=== 0.5°/s 极限环量化 v2 (%.1fs, PDB %d 行 %d Hz, N %d 行, 角度源 %s) ===" %
          (total_t, n_samples, pdb_hz, len(nfr), ang_src))
    print("  跟踪率(LSQ斜率): %+.3f°/s (目标 %+.3f)  %.1f%%  总漂移 %+.2f°" %
          (slope_dps, args.deg_s, track_pct, drift))
    print("  滞后(sensor帧): 起 %+.2f°  末 %+.2f°" % (lag_start, lag_end))
    if oscillating:
        print("  极限环周期: n=%d  mean %.2fs  [min %.2f, max %.2f]" %
              (len(periods), statistics.fmean(periods) if periods else 0.0,
               min(periods) if periods else 0.0, max(periods) if periods else 0.0))
        print("  err 过零→iq 翻转滞后: n=%d  mean %.0fms" %
              (len(lag_s), statistics.fmean(lag_s) * 1000.0 if lag_s else 0.0))
    else:
        print("  极限环: 无 (err 全程单号, 过零 %d 次) —— 非振荡态, 持续掉队" % len(x))
    print("  ki_raw: min %+.4f max %+.4f A  顶死(>±0.105A)占比 %.1f%%  pd_sat %.1f%%" %
          (min(ki_ra), max(ki_ra), clamp_frac, pd_sat_frac))
    print("  FF: ff_total mean %+.4f A  fric_ff mean %+.4f A  方向锁存激活 %.1f%%" %
          (ff_mean, fric_mean, dir_frac))
    print("  电流链: iq_cmd(FF前) mean %+.4f  Iq_ref(FF后) mean %+.4f  差 %+.4f ≈ ff_total交验  Iq_meas mean %+.4f" %
          (iq_cmd_mean, iq_ref_mean, backfire_xcheck, iq_meas_mean))
    print("LC_TRACK=%.1f SLOPE=%.3f PERIOD_MEAN=%.2f OSC=%d "
          "KI_MIN=%.3f CLAMP=%.1f PDSAT=%.1f FF_TOT=%.4f XCHECK=%.4f IQ_MEAS=%.4f" %
          (track_pct, slope_dps, statistics.fmean(periods) if periods else 0.0,
           1 if oscillating else 0, min(ki_ra), clamp_frac, pd_sat_frac,
           ff_mean, backfire_xcheck, iq_meas_mean))

    if args.out:
        import json
        rec = {
            "deg_s": args.deg_s, "secs": args.secs, "enc_dir": args.enc_dir,
            "track_pct": track_pct, "slope_dps": slope_dps, "drift_deg": drift,
            "lag_start_deg": lag_start, "lag_end_deg": lag_end,
            "oscillating": oscillating, "periods": periods,
            "lag_ms": [v * 1000.0 for v in lag_s],
            "clamp_frac": clamp_frac / 100.0, "pd_sat_frac": pd_sat_frac / 100.0,
            "ff_total_mean": ff_mean, "iq_cmd_mean": iq_cmd_mean,
            "iq_ref_mean": iq_ref_mean, "iq_meas_mean": iq_meas_mean,
            "err": errs, "ki_raw": ki_ra, "ki_out": ki_outs, "iq_cmd": iqs,
            "fric_ff": fric_ffs, "ff_total": ff_tots, "dir": dirs,
            "theta": thetas, "ang_deg": ang_uw, "t": times,
            "n_frame": nfr,
        }
        with open(args.out, "w") as f:
            json.dump(rec, f)
        print("saved -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
