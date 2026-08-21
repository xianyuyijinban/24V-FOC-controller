#!/usr/bin/env python3
"""旋转抖动根因判别 — E0~E3 隔离实验矩阵（全硬件节拍测量，杜绝 PC 时间戳伪影）

设计逻辑（每个候选根因对应一个隔离实验）：
  E0 静止基线     : 位置保持，测噪声地板（theta/Iq）
  E1 力矩开环     : IREF 扫档。无速度环/位置环/无 PREF 阶梯 —— 纯功率级+电流环+电机。
                    若纹波在此出现 → 本体侧（齿槽/dq失锁/电流环），与控制结构无关
  E2 速度模式闭环 : SREF 扫档（固件 2kHz 内部斜坡 = 平滑指令，无阶梯）。
                    E2-E1 差 → 速度 PI 贡献
  E3 pos_direct 阶梯: PREF 20Hz 阶梯扫速（定版结构）。E3-E2 差 → 阶梯指令贡献

测量链路（两条分时独占，避免带宽互挤）：
  spd 通道: POSDBG 文本流 —— theta 用固件 2kHz tick 列做时基（t=tick/2000），
            主循环发送抖动不影响 dt 重建。**需要含 tick 列的固件（2026-08-20 起）**
  cur 通道: TELEM:CUR,BIN,2000 二进制流 —— ia/ib/ic/id/iq，ISR 推送 seq 计数即硬件时基

判别输出（自动判定）：
  - 齿槽     : 22 阶（每机械转 22 次）谱线，E1 即有 → 本体签名
  - dq 失锁  : 高速段 id 不纯（id_std 大）/ |Idq| 收缩 / 电频率(11×机速)谱线
  - 阶梯指令 : E3 独有 20Hz 谱线 且 E3 v_std ≫ E2
  - 控制结构 : E1 干净但 E2/E3 差 → 环结构问题；E1 就差 → 本体问题

用法: python scripts/ripple_diag.py COM10 --power-ok [--secs 6] [--ana 4]
      [--speeds 0.5,2,10,30,50] [--irefs 0.05,0.10,0.20] [--only E1,E3]
      [--out scripts/ripple_diag_20260820.json]
"""
import argparse
import json
import math
import statistics
import sys
import time

import numpy as np
import serial

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
POLE_PAIRS = 11          # 24N22P
COG_ORDER = 22.0         # 齿槽主导阶次 (每机械转)
FRAME_LEN = 25
SYNC = b"\xa5\x5a"


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class Capturer:
    """串口接收：文本行（PDB）与二进制帧（CUR）分时解析，行/字节缓冲。"""

    def __init__(self, ser):
        self.ser = ser
        self.tbuf = ""
        self.bbuf = bytearray()
        self.pdb = []   # (label, tick, theta, err, iq_cmd, ff_total)
        self.cur = []   # (label, seq, ia, ib, ic, id, iq)
        self.label = ""
        self.mode = None  # 'text' | 'bin'

    def drain(self):
        n = self.ser.in_waiting
        if not n:
            return
        data = self.ser.read(n)
        if self.mode == "text":
            self.tbuf += data.decode(errors="replace")
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
        elif self.mode == "bin":
            self.bbuf += data
            while True:
                i = self.bbuf.find(SYNC)
                if i < 0:
                    if len(self.bbuf) > FRAME_LEN * 4:
                        del self.bbuf[:-1]
                    return
                if i > 0:
                    del self.bbuf[:i]
                if len(self.bbuf) < FRAME_LEN:
                    return
                fr = bytes(self.bbuf[:FRAME_LEN])
                if fr[2] == ord("C") and fr[3] == 20 and crc8(fr[:24]) == fr[24]:
                    seq = fr[4] | (fr[5] << 8)
                    vals = struct_from(fr)
                    self.cur.append((self.label, seq) + vals)
                    del self.bbuf[:FRAME_LEN]
                else:
                    del self.bbuf[:1]


def struct_from(fr):
    """payload: ia,ib,ic,id,iq (i16 LE) — 返回这 5 个 (A)"""
    def s16(i):
        v = fr[i] | (fr[i + 1] << 8)
        return v - 65536 if v >= 32768 else v
    return (s16(10) / 1000.0, s16(12) / 1000.0, s16(14) / 1000.0,
            s16(16) / 1000.0, s16(18) / 1000.0)


def unwrap_arr(a, period):
    d = np.diff(a)
    d[d > period / 2.0] -= period
    d[d < -period / 2.0] += period
    return np.concatenate(([a[0]], a[0] + np.cumsum(d)))


def fft_peaks(x, dt, fmin=0.5, fmax=120.0, top=4):
    """返回 [(freq, amp)] 前 top 个谱峰（去直流）"""
    n = len(x)
    if n < 32:
        return []
    w = np.hanning(n)
    sp = np.abs(np.fft.rfft((x - np.mean(x)) * w)) * 2.0 / np.sum(w) * 2.0
    fr = np.fft.rfftfreq(n, dt)
    m = (fr >= fmin) & (fr <= fmax)
    idx = np.argsort(sp[m])[::-1][:top]
    return [(float(fr[m][i]), float(sp[m][i])) for i in idx]


def amp_at(x, dt, freq, tol=0.15):
    """指定频率附近的谱幅（±tol Hz 内最大）"""
    n = len(x)
    if n < 32 or freq <= 0:
        return 0.0
    w = np.hanning(n)
    sp = np.abs(np.fft.rfft((x - np.mean(x)) * w)) * 2.0 / np.sum(w) * 2.0
    fr = np.fft.rfftfreq(n, dt)
    m = (fr >= freq - tol) & (fr <= freq + tol)
    return float(np.max(sp[m])) if np.any(m) else 0.0


def analyze_spd(rows, ana_secs):
    """PDB 段分析: 返回 dict(v_mean, v_std, v_pp, dt, peaks, cog_amp, elec_amp, line20, ...)
    v2: 双窗 v_std (25ms/100ms — 量化伪影交叉验证: 差分量化噪声随窗长坍塌, 真实低频运动不塌)
        + th_pp (位置纹波, 低速下比速度稳健)"""
    tick = np.array([r[1] for r in rows], dtype=float)
    th = np.array([r[2] for r in rows], dtype=float)
    if len(tick) < 40:
        return None
    t = (tick - tick[0]) / 2000.0
    if t[-1] < ana_secs + 1.0:
        return None
    m = t >= (t[-1] - ana_secs)
    t, th = t[m], th[m]
    thu = unwrap_arr(th, 2.0 * math.pi) * RAD2DEG
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    # 位置纹波: 去线性趋势后的 1-99% 摆幅 (°)
    th_res = thu - np.polyval(np.polyfit(t, thu, 1), t)
    q1t, q99t = np.percentile(th_res, [1, 99])
    th_pp = float(q99t - q1t)

    def v_std_of(win_s):
        w = max(3, int(round(win_s / dt)))
        ker = np.ones(w) / w
        ths = np.convolve(thu, ker, mode="same")
        v = np.gradient(ths, dt)
        return v, float(np.std(v - np.polyval(np.polyfit(t, v, 1), t)))

    v25, std25 = v_std_of(0.025)
    _, std100 = v_std_of(0.100)
    v_mean = float(np.polyfit(t, v25, 1)[0])
    v_res = v25 - np.polyval(np.polyfit(t, v25, 1), t)
    q1, q99 = np.percentile(v_res, [1, 99])
    mech_hz = abs(v_mean) / 360.0
    return {
        "v_mean": v_mean, "v_std": std25, "v_std_100ms": std100,
        "v_pp": float(q99 - q1), "th_pp": th_pp,
        "dt": dt, "rows": int(len(t)),
        "peaks": fft_peaks(v_res, dt),
        "cog_amp": amp_at(v_res, dt, COG_ORDER * mech_hz, tol=0.3),
        "elec_amp": amp_at(v_res, dt, POLE_PAIRS * mech_hz, tol=0.3),
        "line20": amp_at(v_res, dt, 20.0, tol=0.5),
        "cog_hz": COG_ORDER * mech_hz, "elec_hz": POLE_PAIRS * mech_hz,
    }


def analyze_cur(rows, ana_secs):
    """CUR 段分析: iq/id 纹波 + dq 纯度 + 谱"""
    if len(rows) < 400:
        return None
    seq = np.array([r[1] for r in rows], dtype=np.int64)
    ia = np.array([r[2] for r in rows]); ib = np.array([r[3] for r in rows])
    ic = np.array([r[4] for r in rows]); id_ = np.array([r[5] for r in rows])
    iq = np.array([r[6] for r in rows])
    d = np.diff(seq); d[d < 0] += 65536
    drops = int(np.sum(d[d > 1] - 1))
    t = np.concatenate(([0.0], np.cumsum(d))) / 2000.0
    if t[-1] < ana_secs + 1.0:
        return None
    m = t >= (t[-1] - ana_secs)
    t, ia, ib, ic, id_, iq = t[m], ia[m], ib[m], ic[m], id_[m], iq[m]
    dt = float(np.median(np.diff(t))) or 0.0005
    idq = np.hypot(id_, iq)
    iq_res = iq - np.polyval(np.polyfit(t, iq, 1), t)
    # 相电流基波(电频率)直测: ia 主峰
    pk = fft_peaks(ia, dt, fmin=0.5, fmax=200.0, top=1)
    elec_hz_meas = pk[0][0] if pk else 0.0
    # 相电流幅值对称性 (RMS) — 2×e 纹波归因: 通道不平衡 vs 齿槽
    # 三相 RMS 应近似相等; max/min 显著 >1.1 = 通道增益不平衡 (A/B 交叉补偿残留)
    rms = [float(np.sqrt(np.mean(x ** 2))) for x in (ia, ib, ic)]
    imb = max(rms) / max(min(rms), 1e-9)
    iq_2e_amp = amp_at(iq_res, dt, 2.0 * elec_hz_meas, tol=0.4) if elec_hz_meas > 0 else 0.0
    return {
        "iq_mean": float(np.mean(iq)), "iq_std": float(np.std(iq_res)),
        "iq_pp": float(np.ptp(iq)),
        "id_mean": float(np.mean(id_)), "id_std": float(np.std(id_)),
        "idq_mean": float(np.mean(idq)),
        "id_purity": float(abs(np.mean(id_)) / max(abs(np.mean(iq)), 1e-6)),
        "elec_hz_meas": elec_hz_meas,
        "iabc_rms": rms, "iabc_imbalance": imb,
        "iq_2e_amp": iq_2e_amp,
        "peaks": fft_peaks(iq_res, dt),
        "drops": drops, "rows": int(len(t)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--secs", type=float, default=6.0, help="每档采集时长")
    ap.add_argument("--ana", type=float, default=4.0, help="段末分析窗口")
    ap.add_argument("--speeds", default="0.5,2,10,30,50", help="E2/E3 速档 °/s")
    ap.add_argument("--irefs", default="0.03,0.05,0.10",
                    help="E1 电流档 A (硬上限 0.35; 自由转为正常, 看交付 iq 与实测速度)")
    ap.add_argument("--max-dps", type=float, default=200.0, help="E1 超速保护 °/s")
    ap.add_argument("--only", default="", help="只跑列出的相 (如 E1,E3)")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    speeds = [float(x) for x in args.speeds.split(",")]
    irefs = [min(float(x), 0.35) for x in args.irefs.split(",")]
    phases = ["E0", "E1", "E2", "E3"] if not args.only else \
             [p.strip().upper() for p in args.only.split(",")]

    print("=== 旋转抖动根因判别 (E0-E3) ===")
    print("相: %s  速档: %s°/s  电流档: %sA  每档 %.0fs (分析窗 %.0fs)" %
          (",".join(phases), speeds, irefs, args.secs, args.ana))
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

    def expect_text(cmd, prefix, timeout=1.5):
        """发命令并返回匹配行的完整文本 (COG_CFG? 等无 OK 回显的查询用)"""
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.strip().startswith(prefix):
                        return l.strip()
            else:
                time.sleep(0.01)
        return None

    def send(cmd):
        ser.write((cmd + "\n").encode())
        time.sleep(0.05)

    def abort(msg):
        print("ABORT: %s" % msg)
        for c in ("CMD:IREF,0.000,0.000", "CMD:SREF,0", "CMD:POSDBG,0",
                  "TELEM:CUR,OFF", "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()
        sys.exit(1)

    cap = Capturer(ser)

    def run_segment(label, seconds, tick_fn=None):
        cap.label = label
        t0 = time.time()
        while time.time() - t0 < seconds:
            if tick_fn is not None:
                tick_fn(time.time() - t0)
            cap.drain()
            time.sleep(0.001)
        cap.drain()

    def idle(seconds):
        run_segment("__idle__", seconds)

    def stream_on(ps):
        if ps == "spd":
            send("TELEM:CUR,OFF")
            if not expect("CMD:POSDBG,1", "POSDBG,OK"):
                abort("POSDBG 开启失败")
        else:
            send("CMD:POSDBG,0")
            if not expect("TELEM:CUR,BIN,2000", "CUR_STREAM,OK"):
                abort("CUR BIN 开启失败")
        cap.mode = "text" if ps == "spd" else "bin"
        time.sleep(0.4)
        cap.drain()   # 清掉流开启瞬间的缓冲, 让 last_theta() 能拿到新行

    def stream_off(ps):
        send("CMD:POSDBG,0" if ps == "spd" else "TELEM:CUR,OFF")
        time.sleep(0.2)

    # ── 公共配置 (定版) ──
    for c, p in [("CMD:UNLOCK,1", "UNLOCK,OK"),
                 ("CMD:POS_DIRECT_GAIN,0.490,0.007", "POS_DIRECT_GAIN,OK"),
                 ("CMD:POS_DIRECT_KI,0.37", "POS_DIRECT_KI,OK"),
                 ("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"),
                 ("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"),
                 ("TELEM:OFF", "TELEM:OFF,OK")]:
        if not expect(c, p):
            abort("配置失败: %s" % c)
    send("CMD:COG_CFG,0.000,60.0")   # COG 定版 OFF (无回显, 查询确认)
    r = expect_text("CMD:COG_CFG?", "COG_CFG,gain=")
    if r and "gain=0.000" not in r:
        print("WARN: COG gain 未确认到 0: %s" % r)

    # tick 列检查 (新固件)
    expect("CMD:POSDBG,1", "POSDBG,OK")
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(0.5)
    has_tick = False
    if ser.in_waiting:
        for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
            if l.startswith("PDB,") and len(l.split(",")) >= 12:
                has_tick = True
    send("CMD:POSDBG,0")
    if not has_tick:
        abort("PDB 无 tick 列 —— 先烧录 2026-08-20 含 2kHz 时基的固件")

    def enter_mode(mode_num):
        send("CMD:STOP")
        time.sleep(0.3)
        send("CMD:CLEAR_FAULT")
        if not expect("CMD:MODE,%d" % mode_num, "MODE,OK"):
            abort("MODE,%d 失败" % mode_num)
        if not expect("CMD:ENABLE,1", "ENABLE,OK"):
            abort("ENABLE 失败")
        time.sleep(0.8)

    def last_theta():
        for r in reversed(cap.pdb):
            if r[0] != "__idle__":
                return r[2]
        return None

    try:
        # ───────── E0 静止基线 ─────────
        if "E0" in phases:
            print("\n── E0 静止基线 (位置保持) ──")
            enter_mode(2)
            expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
            for ps in ("spd", "cur"):
                stream_on(ps)
                run_segment("E0.hold", max(args.secs, args.ana + 1.0))
                stream_off(ps)

        # ───────── E1 力矩开环 ─────────
        if "E1" in phases:
            print("\n── E1 力矩开环 (IREF 扫档, 无速度/位置环) ──")
            enter_mode(0)
            for ps in ("spd", "cur"):
                stream_on(ps)
                for iv in irefs:
                    print("  IREF %+.2fA (%s)" % (iv, ps), flush=True)
                    send("CMD:IREF,0.000,%.3f" % iv)
                    run_segment("E1.%.2f" % iv, args.secs)
                    send("CMD:IREF,0.000,0.000")
                    idle(1.0)
                    # 超速保护 (spd 通道判; theta 需先解缠绕)
                    if ps == "spd":
                        seg = [r for r in cap.pdb if r[0] == "E1.%.2f" % iv]
                        if len(seg) > 40:
                            th = np.array([r[2] for r in seg])
                            tk = np.array([r[1] for r in seg])
                            thu = unwrap_arr(th, 2.0 * math.pi)
                            v_est = abs((thu[-1] - thu[len(thu) // 2]) /
                                        max((tk[-1] - tk[len(tk) // 2]) / 2000.0, 1e-6)) * RAD2DEG
                            if v_est > args.max_dps:
                                print("  超速 %.0f°/s > %.0f, 截断 E1 剩余档" % (v_est, args.max_dps))
                                send("CMD:IREF,0.000,0.000")
                                break
                stream_off(ps)

        # ───────── E2 速度模式闭环 ─────────
        if "E2" in phases:
            print("\n── E2 速度模式 (SREF 扫档, 固件平滑斜坡) ──")
            enter_mode(1)
            for ps in ("spd", "cur"):
                stream_on(ps)
                for v in speeds:
                    print("  SREF %+.3f rad/s (%.1f°/s, %s)" % (v * DEG2RAD, v, ps), flush=True)
                    send("CMD:SREF,%.5f" % (v * DEG2RAD))
                    run_segment("E2.%.1f" % v, args.secs)
                    send("CMD:SREF,0")
                    idle(1.5)
                stream_off(ps)

        # ───────── E3 pos_direct 阶梯 ─────────
        if "E3" in phases:
            print("\n── E3 pos_direct (PREF 20Hz 阶梯, 定版结构) ──")
            enter_mode(2)
            expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
            # 锚点用"最后发送的 PREF 目标"接力: 指令在速档/通道切换处无阶跃
            # (cur 通道无 PDB 可读角度; 用目标接力而非实测角度, 电机自然滞后无妨)
            e3 = {"last_tgt": None}
            for ps in ("spd", "cur"):
                stream_on(ps)
                for v in speeds:
                    if e3["last_tgt"] is None:
                        th0 = last_theta()
                        if th0 is None:
                            abort("E3 无起始角度 (PDB 流空)")
                        e3["last_tgt"] = th0
                    state = {"last": -1.0, "t0": time.time(),
                             "a0": e3["last_tgt"], "v": v, "e3": e3}

                    def tick_fn(t, st=state):
                        if t - st["last"] >= 0.05:
                            tgt = st["a0"] + st["v"] * DEG2RAD * (time.time() - st["t0"])
                            ser.write(("CMD:PREF,%.6f\n" % tgt).encode())
                            st["last"] = t
                            st["e3"]["last_tgt"] = tgt

                    print("  PREF 阶梯 %.1f°/s (%s)" % (v, ps), flush=True)
                    run_segment("E3.%.1f" % v, args.secs, tick_fn)
                    idle(0.5)
                stream_off(ps)
    finally:
        print("\n清理: IREF/SREF 归零, 流关闭, STOP")
        for c in ("CMD:IREF,0.000,0.000", "CMD:SREF,0", "CMD:POSDBG,0",
                  "TELEM:CUR,OFF", "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()

    # ═════════ 分析 ═════════
    print("\n════════ 分析 ════════")
    labels = sorted(set([r[0] for r in cap.pdb] + [r[0] for r in cap.cur]) - {"__idle__", ""})
    results = {}
    for lb in labels:
        spd_rows = [r for r in cap.pdb if r[0] == lb]
        cur_rows = [r for r in cap.cur if r[0] == lb]
        a_spd = analyze_spd(spd_rows, args.ana) if spd_rows else None
        a_cur = analyze_cur(cur_rows, args.ana) if cur_rows else None
        results[lb] = {"spd": a_spd, "cur": a_cur}
        line = "  %-10s" % lb
        if a_spd:
            line += " v %+6.2f±%5.2f°/s(100ms窗 %5.2f) th_pp %5.2f°" % (
                a_spd["v_mean"], a_spd["v_std"], a_spd["v_std_100ms"], a_spd["th_pp"])
        else:
            line += " v ---"
        if a_cur:
            line += " | iq %.3f±%.3fA id_std %.3f |Idq| %.3f 三相失衡 %.2f iq@2×e %.3fA" % (
                a_cur["iq_mean"], a_cur["iq_std"], a_cur["id_std"], a_cur["idq_mean"],
                a_cur["iabc_imbalance"], a_cur["iq_2e_amp"])
        print(line, flush=True)
        if a_spd:
            pk = "  ".join("%.1fHz:%.2f" % (f, a) for f, a in a_spd["peaks"][:3])
            print("      谱峰: %s | 20Hz线 %.2f  齿槽(%.1fHz) %.2f  电频(%.1fHz) %.2f" %
                  (pk or "无", a_spd["line20"], a_spd["cog_hz"], a_spd["cog_amp"],
                   a_spd["elec_hz"], a_spd["elec_amp"]))
        if a_cur and a_cur["drops"] > 0:
            print("      CUR 丢帧: %d" % a_cur["drops"])

    # ═════════ 判别 ═════════
    print("\n════════ 判别汇总 ════════")
    def g(lb, ch, key):
        v = results.get(lb, {}).get(ch)
        return v.get(key) if v else None

    # 1) 本体 vs 控制: 找 E1/E2/E3 覆盖相近速档
    for v in speeds:
        e1_lb = None
        # E1 按实测均速匹配最近档
        best, best_d = None, 1e9
        for lb in labels:
            if lb.startswith("E1."):
                vm = g(lb, "spd", "v_mean")
                if vm is not None and abs(abs(vm) - v) < best_d:
                    best, best_d = lb, abs(abs(vm) - v)
        e1_std = g(best, "spd", "v_std") if best else None
        e1_vm = g(best, "spd", "v_mean") if best else None
        e2_std = g("E2.%.1f" % v, "spd", "v_std")
        e3_std = g("E3.%.1f" % v, "spd", "v_std")
        if e1_std is None and e2_std is None and e3_std is None:
            continue
        print("  ~%.1f°/s: E1(实测%s)std=%s  E2 std=%s  E3 std=%s" % (
            v, "%+.1f" % e1_vm if e1_vm is not None else "—",
            "%.2f" % e1_std if e1_std is not None else "—",
            "%.2f" % e2_std if e2_std is not None else "—",
            "%.2f" % e3_std if e3_std is not None else "—"))
    # 2) 齿槽签名
    cog_hits = [lb for lb in labels
                if (g(lb, "spd", "cog_amp") or 0) > 0.5 * max((g(lb, "spd", "v_std") or 1.0), 1e-9)
                and (g(lb, "spd", "cog_amp") or 0) > 0.3]
    print("  齿槽(22阶)显著段: %s" % (",".join(cog_hits) if cog_hits else "无"))
    # 3) dq 失锁迹象
    dq_bad = [lb for lb in labels
              if (g(lb, "cur", "id_purity") or 0) > 0.35
              and abs(g(lb, "cur", "iq_mean") or 0) > 0.02]
    print("  id 不纯(|id|>35%%|iq|)段: %s" % (",".join(dq_bad) if dq_bad else "无"))
    # 4) 阶梯签名
    l20 = [(lb, g(lb, "spd", "line20")) for lb in labels if lb.startswith("E3.")]
    l20 = ["%s:%.2f" % (lb, a) for lb, a in l20 if a and a > 0.3]
    print("  E3 20Hz 阶梯线: %s" % (",".join(l20) if l20 else "无/弱"))
    # 5) 通道不平衡 vs 齿槽归因 (2×e 与 22/圈齿槽在 24N22P 上同阶, 靠三相幅值对称性分)
    imb_bad = [(lb, g(lb, "cur", "iabc_imbalance")) for lb in labels
               if (g(lb, "cur", "iabc_imbalance") or 1.0) > 1.10]
    print("  三相 RMS 失衡 >1.10 段: %s" %
          (",".join("%s:%.2f" % x for x in imb_bad) if imb_bad else "无"))
    iq2e = [(lb, g(lb, "cur", "iq_2e_amp")) for lb in labels]
    iq2e = ["%s:%.3fA" % (lb, a) for lb, a in iq2e if a and a > 0.02]
    print("  iq@2x电频 >20mA 段: %s" % (",".join(iq2e) if iq2e else "无"))

    print("""
判读指南:
  E1 std 已大        -> 本体侧 (齿槽/dq/电流环交付), 控制结构无辜
  E1 干净 E2 起纹波  -> 速度环 (增益/反馈噪声)
  E3 >> E2 且 20Hz 线 -> PREF 阶梯指令, 需固件轨迹插值
  齿槽段=E1 段       -> 齿槽转矩 (LUT 标定重新排上日程)
  id 不纯在高速段    -> dq 帧失锁 (3.18 旧账), 查 theta_elec/Park
  三相失衡>1.10 且 iq@2xe 大 -> 采样通道增益不平衡 (A/B 交叉补偿残留), 电流环实测被污染
  iq@2xe 大但三相对称        -> 齿槽/磁阻转矩 (同阶次, 归本体)
  v_std(25ms) >> v_std(100ms) -> 差分量化/噪声伪影, 绝对量级不可信, 看 th_pp""")

    if args.out:
        # 原始序列降采样存档 (波形级复核: v_std 量级争议时直接看图)
        series = {}
        for lb in labels:
            spd_rows = [r for r in cap.pdb if r[0] == lb]
            cur_rows = [r for r in cap.cur if r[0] == lb]
            s = {}
            if spd_rows:
                step = max(1, len(spd_rows) // 800)
                s["pdb"] = [list(r) for r in spd_rows[::step]]   # (label,tick,theta,err,iq_cmd,ff_total)
            if cur_rows:
                step = max(1, len(cur_rows) // 1200)
                s["cur"] = [list(r) for r in cur_rows[::step]]   # (label,seq,ia,ib,ic,id,iq)
            series[lb] = s
        ser_out = {
            "args": vars(args), "results": results, "series": series,
            "pdb_rows": len(cap.pdb), "cur_rows": len(cap.cur),
        }
        with open(args.out, "w") as f:
            json.dump(ser_out, f)
        print("saved -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
