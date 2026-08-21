#!/usr/bin/env python3
"""通道真值实验 — 固定 IREF + 手转一整圈, 测三通道 增益/偏移/相位 真值表

原理: 力矩模式固定 Iq -> dq 电流矢量在转子系恒定。手转电机一整圈,
每个采样通道被迫轮流经历同一个完整正弦 (真实电流 KCL 必零和):
  - 幅值比   -> 通道增益比 (与绕组无关: 同一电流矢量在通道间轮转)
  - DC 偏移  -> ADC 零偏残差
  - 相位间隔 -> 采样时刻/映射错误 (应为 120°)
  - |ia+ib+ic|/Σ|i| -> KCL 违约率 (物理真值 0; 波纹污染 -> 采样时刻/窗口问题)
  - iq 2xe 纹波 -> 闭环实际会吃到的污染量

为什么不用闭环数据: 闭环下测量污染自我实现 (测歪->环按歪的调->真实也歪),
只有开环固定矢量下手转, 真实电流才天然对称, 测出的不对称 = 纯测量误差。

操作: 台架上电, 电机轴可触及。脚本给 IREF 后倒计时 3s, 然后采集 ~12s —
采集期间用手匀速转电机轴至少一整圈 (约 10s/圈, 方向不限, 尽量匀速)。

用法: python scripts/channel_truth.py COM10 --power-ok [--iref 0.15] [--secs 12]
"""
import argparse
import json
import sys
import time

import numpy as np
import serial

FRAME_LEN = 25
SYNC = b"\xa5\x5a"


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--iref", type=float, default=0.15, help="固定 Iq (A), 硬上限 0.35")
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    iref = min(args.iref, 0.35)
    print("=== 通道真值实验: IREF %.2fA, 采集 %.0fs ===" % (iref, args.secs))
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
        time.sleep(0.05)

    for c, p in [("CMD:UNLOCK,1", "UNLOCK,OK"),
                 ("TELEM:OFF", "TELEM:OFF,OK"),
                 ("CMD:STOP", "CTRL:STOP"),
                 ("CMD:MODE,0", "MODE,OK"),
                 ("CMD:ENABLE,1", "ENABLE,OK")]:
        if not expect(c, p):
            print("ABORT: %s" % c)
            ser.close()
            return 1
    send("CMD:CLEAR_FAULT")   # 无稳定回显, 不校验

    frames = []
    buf = bytearray()
    try:
        if not expect("TELEM:CUR,BIN,2000", "CUR_STREAM,OK"):
            print("ABORT: CUR BIN 开启失败")
            return 1
        send("CMD:IREF,0.000,%.3f" % iref)
        print("IREF 已施加。3s 后开始采集 —— 请准备转轴", flush=True)
        time.sleep(3.0)
        ser.reset_input_buffer()
        print("采集中... 请匀速转动电机轴一整圈 (约 10s/圈)", flush=True)
        t0 = time.time()
        while time.time() - t0 < args.secs:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                while True:
                    i = buf.find(SYNC)
                    if i < 0:
                        if len(buf) > FRAME_LEN * 4:
                            del buf[:-1]
                        break
                    if i > 0:
                        del buf[:i]
                    if len(buf) < FRAME_LEN:
                        break
                    fr = bytes(buf[:FRAME_LEN])
                    if fr[2] == ord("C") and fr[3] == 20 and crc8(fr[:24]) == fr[24]:
                        def s16(i):
                            v = fr[i] | (fr[i + 1] << 8)
                            return v - 65536 if v >= 32768 else v
                        frames.append((fr[4] | (fr[5] << 8),
                                       s16(10) / 1000.0, s16(12) / 1000.0, s16(14) / 1000.0,
                                       s16(16) / 1000.0, s16(18) / 1000.0))
                        del buf[:FRAME_LEN]
                    else:
                        del buf[:1]
            else:
                time.sleep(0.001)
    finally:
        for c in ("CMD:IREF,0.000,0.000", "TELEM:CUR,OFF", "TELEM:ON", "CMD:STOP"):
            send(c)
        ser.close()

    if len(frames) < 2000:
        print("ERR: 帧数不足 (%d)" % len(frames))
        return 1

    a = np.array(frames, dtype=float)
    seq = a[:, 0].astype(np.int64)
    ia, ib, ic, idc, iq = a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5]
    d = np.diff(seq); d[d < 0] += 65536
    drops = int(np.sum(d[d > 1] - 1))

    print("\n=== 通道真值表 (n=%d, 丢帧 %d) ===" % (len(frames), drops))
    names = ("A", "B", "C")
    amps, offs = [], []
    for x, nm in zip((ia, ib, ic), names):
        hi, lo = np.percentile(x, 99), np.percentile(x, 1)
        amp = (hi - lo) / 2.0
        off = (hi + lo) / 2.0
        amps.append(amp)
        offs.append(off)
        print("  通道 %s: 幅值 %.4f A  偏移 %+.4f A  (1-99%% 包络)" % (nm, amp, off))
    amp_arr = np.array(amps)
    print("  幅值比: A/B %.3f  B/C %.3f  A/C %.3f  (增益真值比)"
          % (amps[0] / max(amps[1], 1e-9), amps[1] / max(amps[2], 1e-9),
             amps[0] / max(amps[2], 1e-9)))
    print("  偏移: %+.4f / %+.4f / %+.4f A" % tuple(offs))

    kcl = np.mean(np.abs(ia + ib + ic)) / np.mean(np.abs(ia) + np.abs(ib) + np.abs(ic))
    print("  KCL 违约率: %.3f  (健康 < 0.05)" % kcl)

    # 相位间隔: 互相关搜索电角度滞后 (ia->ib 应为 +120°, ib->ic +120°)
    def phase_lag(x, y):
        x0, y0 = x - x.mean(), y - y.mean()
        n = len(x0)
        xc = np.correlate(x0, y0, mode="full")[n - 1 - n // 4: n - 1 + n // 4 + 1]
        lag = np.argmax(xc) - n // 4
        dd = np.diff(seq); dd[dd < 0] += 65536
        t = np.concatenate(([0.0], np.cumsum(dd))) / 2000.0
        fe = 0.0
        w = np.hanning(len(t))
        sp = np.abs(np.fft.rfft(x0 * w)); fr = np.fft.rfftfreq(len(t), np.median(np.diff(t)))
        m = fr > 0.05
        if np.any(m):
            fe = fr[m][np.argmax(sp[m])]
        return lag, (lag / 2000.0) * fe * 360.0 if fe > 0 else 0.0, fe

    lag_ab, deg_ab, fe = phase_lag(ia, ib)
    lag_bc, deg_bc, _ = phase_lag(ib, ic)
    print("  相位间隔: A->B %.0f°  B->C %.0f°  (|间隔|应相等, 120°或240°取决于方向约定; 电频 %.2fHz)" % (deg_ab, deg_bc, fe))

    # iq 纹波 (闭环实际会吃到的污染): 2xe 幅值
    t = np.concatenate(([0.0], np.cumsum(np.where(np.diff(seq) < 0, np.diff(seq) + 65536, np.diff(seq))))) / 2000.0
    if fe > 0.05:
        c2 = 2 * np.mean(iq * np.exp(-2j * np.pi * 2 * fe * t[:len(iq)]))
        print("  iq: mean %+.4f A  std %.4f  |2xe| %.4f A" % (iq.mean(), iq.std(), abs(c2)))
    else:
        print("  iq: mean %+.4f A  std %.4f" % (iq.mean(), iq.std()))

    print("""
判读:
  幅值比偏离 1 超过 ±10%  -> 通道增益不等 -> 标定补偿表可修
  某通道幅值系统性最小     -> 该通道物理缩水 (分流/走线/虚焊), 先查硬件
  相位间隔偏离 120° 超过 ±5° -> 采样时刻逐相偏斜或映射错, 查 ADC 触发链路
  KCL 违约率随幅值等比       -> 增益型; 随信号减小而恶化 -> 偏移型; 高幅值时恶化 -> 窗口/时刻型""")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"args": vars(args), "amps": amps, "offs": offs,
                       "kcl": float(kcl), "frames": len(frames), "drops": drops,
                       "deg_ab": deg_ab, "deg_bc": deg_bc, "fe": fe,
                       "iq_mean": float(iq.mean()), "iq_std": float(iq.std())}, f)
        print("saved -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
