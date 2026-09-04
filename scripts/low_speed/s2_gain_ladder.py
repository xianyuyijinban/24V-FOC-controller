#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s2_gain_ladder.py — S2 电压模式位置环增益阶梯 (2026-09-04 Kimi 规格)

增益阶梯 (电压口径 V/rad; POS_DIRECT_GAIN 发电流口径 A/rad, 固件末级 ×Rs_phase 4.4):
  G1: 1.016 → 发 0.231/0.0035 + KI 0.185
  G2: 2.033 → 发 0.462/0.0070 + KI 0.370
  G3: 3.050 → 发 0.693/0.0105 + KI 0.555

每档判读:
  阶跃 +6°: t95 (PDBBIN pos_err 收敛到 ±0.1°), 稳态 pp (测量窗修复版)
  0.5°/s 斜坡: 跟踪率 = 实测位移/6° (>90% 目标)
  极限环: PDBBIN pos_err 后段 1s 峰峰 (首档 5.15° → 2.57° → 1.72° 预测)

测量窗: foclink.MeasureWindow (窗前排空积压计数 + host_rx 时间戳过滤)
  — 观测家族第五条 (2026-09-04 Kimi 终审), 垃圾数据不入判读

用法: python scripts/low_speed/s2_gain_ladder.py COM10 --power-ok [--rounds 3]
"""
import argparse
import json
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0

# 电流口径 POS_DIRECT_GAIN (A/rad); 电压口径 = kp × Rs_phase(Rs/2≈4.149):
#   G1 = 半值 0.245 → 1.016V/rad, G2 = 全值 0.490 → 2.033V/rad, G3 = 1.5× 0.735 → 3.05V/rad
LADDER = [
    ("G1", 0.245, 0.0035, 0.185),
    ("G2", 0.490, 0.0070, 0.370),
    ("G3", 0.735, 0.0105, 0.555),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=3, help="每档重复轮数")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:VOLT_OFF\n"):
        ser.write(c)
        time.sleep(0.2)
    ser.reset_input_buffer()

    class LineBuffer:
        def __init__(self):
            self.buf = ""

        def _drain(self):
            n = ser.in_waiting
            if not n:
                return []
            self.buf += ser.read(n).decode(errors="replace")
            if "\n" not in self.buf:
                return []
            lines = self.buf.split("\n")
            self.buf = lines.pop()
            return lines

    lb = LineBuffer()

    def expect(cmd, prefix, timeout=3.0):
        ser.reset_input_buffer()
        lb.buf = ""
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            for l in lb._drain():
                if l.strip().startswith(prefix):
                    return True
            time.sleep(0.01)
        return False

    def send(cmd, wait=0.3):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)
    reader_ready = [False]

    def expect2(cmd, prefix, timeout=5.0):
        """reader 启动后的 expect: 走 resp_q (线程安全), 不直读串口。
        为避免 250 lines/s C/N 流挤掉响应 (KI fail 根因), 响应去重 + 增大限流。"""
        resp_q.clear()   # 此刻旧行都是垃圾
        ser.reset_output_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        seen = set()
        while time.time() < dl:
            while resp_q:
                l = resp_q.pop(0)
                if l in seen:
                    continue
                seen.add(l)
                if l.startswith(prefix):
                    return True
            time.sleep(0.005)
        return False

    # JDIAG 参数指纹预检
    ser.reset_input_buffer()
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
    jline = next((l.strip() for l in jbuf.replace("\r", "").split("\n")
                  if l.startswith("JDIAG,")), None)
    print("JDIAG:", jline[:150] if jline else "无响应", flush=True)

    # 单线程 reader (PDBBIN + N帧) — 主线程不得再直读串口 (家族 segfault 教训)
    pdb_rows = []
    nframe_q = []   # (host_rx, p1_ts, p3_ang_deg, p6_iq, p8_fault, p2_state)
    resp_q = []     # reader 填充的响应行 (threading 启动后 expect 走它)
    resp_seen = set()   # 已处理行去重
    stop = [False]

    def on_pdb(s):
        # (hrx, tick, flags, pos_err, iq_cmd, theta, ff_total, iq_act)
        pdb_rows.append((s.host_rx_time, s.tick_2khz, s.flags,
                         s.pos_err_rad, s.iq_cmd, s.theta_user_rad,
                         s.ff_total, s.iq_act))

    def on_line(line):
        l = line.strip()
        resp_q.append(l)
        while len(resp_q) > 8000:
            del resp_q[0:4000]
        if l.startswith("N,"):
            p = l.split(",")
            if len(p) >= 21:
                try:
                    nframe_q.append((time.time(), p[1], float(p[3]),
                                     float(p[6]), p[8], p[2]))
                except ValueError:
                    pass

    parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb)

    def reader():
        while not stop[0]:
            try:
                n = ser.in_waiting
                if n:
                    parser.feed(ser.read(n))
                else:
                    time.sleep(0.001)
            except Exception:
                break

    # 配置 (电压模式 S2)
    send("CMD:UNLOCK,1", 0.15)
    ser.reset_input_buffer()
    if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
        print("POS_DIRECT fail"); ser.close(); return 1
    send("CMD:COG_CFG,0.0,60.0", 0.15)
    if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
        print("FRIC fail"); ser.close(); return 1
    if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
        print("AW fail"); ser.close(); return 1
    if not expect("CMD:MODE,3", "MODE,OK"):
        print("MODE,3 fail"); ser.close(); return 1
    time.sleep(0.3)
    en_ok = False
    for attempt in range(3):
        if expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
            en_ok = True
            break
        send("CMD:CLEAR_FAULT", 0.8)
    if not en_ok:
        print("ENABLE fail"); ser.close(); return 1
    ser.write(b"CMD:ON\n")
    time.sleep(0.4)
    ser.reset_input_buffer()
    time.sleep(0.5)

    # 锚定位
    a0 = None
    dl = time.time() + 1.0
    while time.time() < dl and a0 is None:
        for l in lb._drain():
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 21:
                    try:
                        a0 = float(p[3])
                        break
                    except ValueError:
                        pass
        if a0 is None:
            time.sleep(0.02)
    if a0 is None:
        print("no angle"); ser.close(); return 1
    print("锚定位: %.2f°" % a0, flush=True)
    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
    time.sleep(2.0)
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    import threading
    threading.Thread(target=reader, daemon=True).start()
    reader_ready[0] = True   # 此后所有串口读走 resp_q / nframe_q (家族: 双线程直读串口 segfault)
    lb.buf = ""   # 清零残余半行 (已交 reader 线程)

    mw = foclink.MeasureWindow(nframe_q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)

    results = []
    try:
        for gname, kp, kd, ki in LADDER:
            for rnd in range(args.rounds):
                rb = len(pdb_rows)
                if not expect2("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (kp, kd),
                               "POS_DIRECT_GAIN,OK"):
                    print("GAIN fail"); raise SystemExit(1)
                if not expect2("CMD:POS_DIRECT_KI,%.2f" % ki, "POS_DIRECT_KI,OK"):
                    resp_tail = resp_q[-15:]
                    print("KI fail resp_tail=%r" % (resp_tail,))
                    raise SystemExit(1)

                # ── 静止阶跃 +6° (从 a0 静止发, pos_err 进 ±0.1°) — t95 ──
                # (2026-09-04 Kimi 纠错: 阶跃从斜坡末尾发是协议退化, t95=无效
                #  本序列: 阶跃从静止发 → 斜坡回程测跟踪 → 窗在 a0 目标)
                target_rad = (a0 + 6) * DEG2RAD
                t_step = time.time()
                ser.write(b"CMD:PREF,%.6f\n" % target_rad)
                t95 = None
                dl2 = t_step + 6.0
                while time.time() < dl2:
                    if pdb_rows:
                        pr = pdb_rows[-1]
                        if pr[0] >= t_step and abs(pr[3]) < 0.1 * DEG2RAD:
                            t95 = time.time() - t_step
                            break
                    time.sleep(0.02)
                time.sleep(1.0)

                # ── 斜坡回程 6° (12s @0.5°/s, a0+6 → a0) — 跟踪率 ──
                dur = 12.0
                t1 = time.time()
                last = 0.0
                while time.time() - t1 < dur:
                    t = time.time() - t1
                    if t - last >= 0.2:
                        frac = min(t / dur, 1.0)
                        cur_deg = a0 + 6.0 * (1.0 - frac)
                        ser.write(b"CMD:PREF,%.6f\n" % (cur_deg * DEG2RAD))
                        last = t
                    time.sleep(0.01)
                ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))

                # ── 稳定门 → 测量窗(修复版, 目标=a0) ──
                ok, waited = mw.wait_stable(a0, timeout=15.0)
                if not ok:
                    print("%s 轮%d: TIMEOUT gate(%.1fs)" % (gname, rnd, waited), flush=True)
                    results.append({"gain": gname, "round": rnd, "gate": "TIMEOUT",
                                    "gate_wait_s": round(waited, 2)})
                    continue
                backlog = mw.begin()
                frames, _ = mw.collect()
                angs = []
                for f in frames:
                    d = f[2] - a0
                    while d > 180.0: d -= 360.0
                    while d < -180.0: d += 360.0
                    angs.append(d)
                pp = (max(angs) - min(angs)) if angs else -1.0
                # 窗内偏差: mean 相对回位目标 a0 (=0) — Kimi 纠错: 原脚本按 +6 减是基准 bug
                resid_deg = (sum(angs) / len(angs)) if angs else None

                # ── 极限环 (PDBBIN pos_err 后段 1s) ──
                traj = pdb_rows[rb:]
                t_now = time.time()
                pes = [r[3] for r in traj if r[0] > t_now - 1.0] if traj else []
                lc_pp_deg = ((max(pes) - min(pes)) / DEG2RAD) if len(pes) > 5 else None

                # ── 跟踪率 (PDBBIN theta 相对斜坡, 后 4-8s 段) ──
                track = None
                ramp_pdb = [r for r in traj if r[0] >= t1 and r[0] <= t1 + dur]
                if ramp_pdb:
                    seg = [r for r in ramp_pdb if 3.5 <= (r[0] - t1) <= 8.5]
                    if len(seg) >= 5:
                        d = seg[-1][5] - seg[0][5]   # r[5] = theta_user_rad
                        while d > 180.0 * DEG2RAD: d -= 360.0 * DEG2RAD
                        while d < -180.0 * DEG2RAD: d += 360.0 * DEG2RAD
                        ideal = -6.0 * DEG2RAD * (seg[-1][0] - seg[0][0]) / dur  # 回程负向
                        track = (d / ideal * 100.0) if abs(ideal) > 0 else None

                ts = time.strftime("%H:%M:%S")
                print("%s 轮%d: ts=%s overflow_backlog=%d gate=%.1fs pp=%.3f° "
                      "t95=%s track=%s lc_pp=%s pdb=%d" %
                      (gname, rnd, ts, backlog, waited, pp,
                       ("%.2fs" % t95) if t95 else "N/A",
                       ("%.0f%%" % track) if track else "N/A",
                       ("%.2f°" % lc_pp_deg) if lc_pp_deg else "N/A", len(traj)), flush=True)
                results.append({
                    "gain": gname, "kp": kp, "kd": kd, "ki": ki,
                    "round": rnd, "ts": ts, "backlog": backlog,
                    "gate_ok": ok, "gate_wait_s": round(waited, 2),
                    "steady_pp_deg": round(pp, 4),
                    "steady_resid_deg": round(resid_deg, 4) if resid_deg is not None else None,
                    "t95_s": round(t95, 3) if t95 else None,
                    "track_pct": round(track, 1) if track else None,
                    "limit_cycle_pp_deg": round(lc_pp_deg, 4) if lc_pp_deg is not None else None,
                    "pdb_n": len(traj),
                    "theta_traj": [(round(r[0], 3), round(r[5] / DEG2RAD, 4)) for r in traj[::8]],
                    "poserr_traj": [(round(r[0], 3), round(r[3] / DEG2RAD, 4), round(r[4], 5))
                                    for r in traj[::8]],
                    "ff_iq_traj": [(round(r[0], 3), round(r[6], 5), round(r[7], 5))
                                   for r in traj[::8]],   # (hrx, ff_total, iq_act)
                })
                time.sleep(1.0)
    finally:
        stop[0] = True
        time.sleep(0.3)
        ser.write(b"CMD:VOLT_OFF")
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        send("CMD:PDBBIN,0")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "s2_gain_ladder_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("\nJSON: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
