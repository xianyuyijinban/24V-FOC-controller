#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""e1_angle_rerun.py — S1 E1 角度对齐复核 + S1.2 死区阈值合并 (2026-09-02 任务卡)

判据 (Kimi 钉死):
- 堵转注入 Vq 阶梯 0.2/0.3/0.5/1.0/1.5V, 各到稳后测 10s
- 纯阻预期: 电流矢量与电压同相 → |Iq|≫|Id|, 夹角 atan(|Id|/|Iq|) < 15°
- 四字段: N 帧 p[5]/p[6] = Id/Iq 测量, p[20]/p[21] = Vd/Vq 实际输出
  (注意: p[16]/p[17] 是 Id_ref/speed_ref 指令, 电压模式恒零/残留, 读错会假象复现病样)
- 斜率 dVq/dIq → Rs 标尺验证; 截距 Vq(Iq=0) → 20kHz 死区压降 (E4 200-300mV 是否漂移)
- 低档 Iq 若系统性偏离线性 (死区吃掉 20-30%) → 死区补偿标定输入数据

固件细节: 内置 Vq 斜坡 0.05V/s (FOC_VOLTAGE_VQ_RAMP_V_PER_S), 每档等待
t_settle = vq/0.05 + 2s 余量后再测 10s (0.2V≈4s, 1.5V≈30s)。

用法: python scripts/low_speed/e1_angle_rerun.py COM10 --power-ok  (需手限位锁轴)
"""
import argparse
import json
import os
import sys
import time

import serial

DEG2RAD = 3.14159265358979 / 180.0
RAMP_V_PER_S = 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--steps", default="200,300,500,1000,1500", help="Vq 阶梯 mV")
    ap.add_argument("--win", type=float, default=10.0, help="每档测量窗 s")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok 才上台架")
        return 0
    steps = [float(s) for s in args.steps.split(",") if s.strip()]

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

    def expect(cmd, prefix, timeout=1.5):
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

    # 预检: JDIAG 身份
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
    jline = next((l.strip() for l in jbuf.replace("\r", "").split("\n") if l.startswith("JDIAG,")), None)
    if jline:
        kv = dict(f.split("=", 1) for f in jline.split(",") if "=" in f)
        print("JDIAG: enc=%s Pn审计见下方" % kv.get("enc", "?"))
        print("JDIAG: %s" % jline[:130])
    else:
        print("JDIAG 无响应 — 中止")
        ser.close()
        return 1

    # 使能链 (停流状态, 响应不被淹没)
    send("CMD:UNLOCK,1", 0.15)
    ser.reset_input_buffer()
    if not expect("CMD:MODE,3", "MODE,OK"):
        print("MODE,3 fail"); ser.close(); return 1
    time.sleep(0.3)
    if not expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
        print("ENABLE fail (试 CLEAR_FAULT 重试)"); ser.close(); return 1

    # POWER 提示: 手限位确认 (无交互 — 打印后 15s 准备窗口)
    print("\n*** 即将上功率: 请手限位锁轴 (转子固定) ***")
    print("*** 15s 准备窗口后开始 Vq 阶梯 ***", flush=True)
    time.sleep(15.0)

    # 开 N 帧 (读 Id/Iq/Vd/Vq 需要)
    ser.write(b"CMD:ON\n")
    time.sleep(0.4)
    ser.reset_input_buffer()

    results = []
    try:
        for step_mv in steps:
            vq_v = step_mv / 1000.0
            settle_s = vq_v / RAMP_V_PER_S + 2.0
            print("\n=== Vq=%dmV (斜坡到位 ≈%.0fs + 测 %ds) ===" % (step_mv, settle_s, args.win), flush=True)
            # 设置目标
            ser.reset_input_buffer()
            lb.buf = ""
            ser.write(b"CMD:VOLT,%.0f\n" % step_mv)
            time.sleep(0.5)
            # 排空 VOLT 响应 + 等待斜坡
            time.sleep(settle_s)
            lb._drain()
            # 测量窗: 静默采集 N 帧
            t0 = time.time()
            rows = []
            while time.time() - t0 < args.win:
                for l in lb._drain():
                    if l.startswith("N,"):
                        p = l.split(",")
                        if len(p) >= 25:
                            rows.append(p)
                time.sleep(0.002)
            if not rows:
                print("  无 N 帧数据!"); continue
            # 解析: p[5]=Id, p[6]=Iq, p[20]=Vd, p[21]=Vq, p[8]=faultFlags(hex)
            ids, iqs, vds, vqs = [], [], [], []
            faults = 0
            for p in rows:
                try:
                    ids.append(float(p[5]))
                    iqs.append(float(p[6]))
                    vds.append(float(p[20]))
                    vqs.append(float(p[21]))
                    if int(p[8], 16) != 0:
                        faults += 1
                except ValueError:
                    continue
            if not iqs:
                continue
            n = len(iqs)
            mean_id = sum(ids) / n
            mean_iq = sum(iqs) / n
            mean_vd = sum(vds) / n
            mean_vq = sum(vqs) / n
            # 夹角 (纯阻同相: atan(|Id|/|Iq|))
            import math
            ang = math.degrees(math.atan2(abs(mean_id), abs(mean_iq))) if mean_iq != 0 else 90.0
            # 协方差斜率 (Vq vs Iq, 若 Iq 测量可信)
            slope = None
            if abs(mean_iq) > 1e-6:
                slope = mean_vq / mean_iq  # 简化: 平均斜率
            else:
                slope = mean_vq / 1e-6
            verdict = "PASS(同相)" if ang < 15.0 else "FAIL(错位!)"
            print("  n=%d Id=%+8.4f Iq=%+8.4f Vd=%+7.4f Vq=%+7.4f 夹角=%.1f° %s fault=%d"
                  % (n, mean_id, mean_iq, mean_vd, mean_vq, ang, verdict, faults), flush=True)
            results.append({
                "vq_cmd_mV": step_mv, "n": n,
                "id_mean": round(mean_id, 5), "iq_mean": round(mean_iq, 5),
                "vd_mean": round(mean_vd, 5), "vq_mean": round(mean_vq, 5),
                "angle_deg": round(ang, 2), "verdict": verdict,
                "slope_vq_iq": round(slope, 4) if slope else None, "fault_count": faults,
            })
    finally:
        ser.write(b"CMD:VOLT_OFF")
        send("CMD:OFF")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        ser.close()

    # 线性拟合: Vq vs Iq (通过全部档)
    import math
    xs = [r["iq_mean"] for r in results]
    ys = [r["vq_mean"] for r in results]
    fit = None
    if len(results) >= 2:
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        den = sum((x - mx) ** 2 for x in xs)
        if abs(den) > 1e-12:
            slope_fit = num / den
            intercept = my - slope_fit * mx
            fit = {"slope_Rs": round(slope_fit, 4), "intercept_V": round(intercept, 4)}
            print("\n=== 线性拟合 Vq = R·Iq + V0 ===")
            print("  Rs_slope = %.4f Ω (识别 Rs 对照)" % slope_fit)
            print("  截距 V0 = %.4f V (死区压降, E4 预期 0.2-0.3V)" % intercept)
    print("\n=== 汇总 ===")
    for r in results:
        print("  %4.0fmV: Id=%+8.4f Iq=%+8.4f Vd=%+7.4f Vq=%+7.4f 夹角=%.1f° %s"
              % (r["vq_cmd_mV"], r["id_mean"], r["iq_mean"], r["vd_mean"],
                 r["vq_mean"], r["angle_deg"], r["verdict"]))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "e1_angle_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results, "fit": fit}, f,
                  ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
