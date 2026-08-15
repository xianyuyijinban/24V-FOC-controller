#!/usr/bin/env python3
"""
板子静态检查 — 无编码器/SWD线束时经 UART 的静态验证 + ADC 噪声测量

安全规则: 只发查询/遥测命令，不发任何功率命令
(UNLOCK/ENABLE/IDENTIFY/PREF/SREF/IREF/VDQ_TEST 全部禁止)。

流程:
  1. FW_INFO?        固件版本/参数版本
  2. CMD:PWM_DIAG    PWM 配置 + 电流采样单行诊断
  3. CMD:TLE_RAW     编码器通信状态（线束未接时预期 FAIL）
  4. CMD:FAULT_DETAIL 故障状态
  5. CMD:UART_RX?    UART RX 状态
  6. TELEM:CUR,BIN,1000 采 N 秒 → 解析电流流 → Ia/Ib/Ic/Id/Iq 噪声统计
  7. TELEM:CUR,OFF   恢复 N 帧遥测
输出: 控制台报告 + JSON 保存。

用法:
  python scripts/static_noise_check.py --port COM10 --baud 1000000 [--seconds 5]
"""

import argparse
import json
import os
import statistics
import sys
import time

import serial

# 项目根加入 sys.path, 复用 HostComputer 的 BIN 帧解析器
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from HostComputer.data_parser import BinaryCurrentParser  # noqa: E402

TELEM_PREFIXES = ("N,", "C,", "B,")


def read_responses(ser, dur=1.0):
    """读 dur 秒，返回非遥测行。"""
    lines = []
    deadline = time.time() + dur
    while time.time() < deadline:
        w = ser.in_waiting
        if w:
            raw = ser.read(w)
            for l in raw.decode(errors="replace").split("\n"):
                l = l.strip()
                if l and not l.startswith(TELEM_PREFIXES):
                    lines.append(l)
        else:
            time.sleep(0.01)
    return lines


def send_cmd(ser, cmd, dur=1.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    return read_responses(ser, dur)


def fmt_stats(vals, scale=1000.0):
    """mean/std/pp, 单位 mA(scale=1000)"""
    if not vals:
        return "n/a"
    m = statistics.mean(vals) * scale
    s = statistics.pstdev(vals) * scale if len(vals) > 1 else 0.0
    p = (max(vals) - min(vals)) * scale
    return "mean=%+8.3f  std=%7.3f  pp=%8.3f (mA)" % (m, s, p)


def main():
    ap = argparse.ArgumentParser(description="板子静态检查 + ADC 噪声测量")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--seconds", type=float, default=5.0, help="电流流采集秒数")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    report = {"port": args.port, "baud": args.baud, "sections": {}}

    # ── 1. FW_INFO? ──
    lines = send_cmd(ser, "SYS:FW_INFO?")
    if not lines:
        print("[FAIL] FW_INFO? 无响应 — 检查波特率/线缆，或板子未运行")
        ser.close()
        return 1
    print("== FW_INFO ==")
    for l in lines:
        print("  " + l)
    report["sections"]["fw_info"] = lines

    # ── 2-5. 诊断查询 ──
    for label, cmd in [
        ("pwm_diag", "CMD:PWM_DIAG"),
        ("tle_raw", "CMD:TLE_RAW"),
        ("fault_detail", "CMD:FAULT_DETAIL"),
        ("uart_rx", "CMD:UART_RX?"),
    ]:
        lines = send_cmd(ser, cmd, dur=1.5)
        print("== %s ==" % label.upper())
        for l in lines:
            print("  " + l)
        report["sections"][label] = lines

    # ── 6. BIN1000 噪声采集 ──
    print("== CURRENT_STREAM BIN1000 (%ss) ==" % args.seconds)
    ser.reset_input_buffer()
    ser.write(b"TELEM:CUR,BIN,1000\n")
    parser = BinaryCurrentParser()
    samples = []
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        w = ser.in_waiting
        if w:
            raw = ser.read(w)
            got, wheel, _ = parser.feed_all(raw)
            samples.extend(got)
        else:
            time.sleep(0.005)
    ser.write(b"TELEM:CUR,OFF\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    n = len(samples)
    print("  帧数: %d, 采集 %.1fs → %.0f fps" % (n, args.seconds, n / args.seconds))
    if n == 0:
        print("[FAIL] 无 BIN 帧 — 旧固件可能不支持 BIN1000，改用 TELEM:CUR,ASCII 重试")
    else:
        ch = {"ia": [s.ia for s in samples], "ib": [s.ib for s in samples],
              "ic": [s.ic for s in samples], "id": [s.id for s in samples],
              "iq": [s.iq for s in samples], "vbus": [s.vbus for s in samples]}
        noise = {}
        for k, v in ch.items():
            print("  %-5s %s" % (k.upper(), fmt_stats(v)))
            noise[k] = {
                "mean": statistics.mean(v), "std": statistics.pstdev(v) if len(v) > 1 else 0.0,
                "pp": max(v) - min(v),
            }
        # 简易判读
        ia_std_ma = noise["ia"]["std"] * 1000.0
        print("  ---")
        if ia_std_ma < 5.0:
            print("  [OK] Ia 静态噪声 std=%.2f mA — 低于 5mA 参考线，ADC 底噪健康" % ia_std_ma)
        elif ia_std_ma < 15.0:
            print("  [WARN] Ia 静态噪声 std=%.2f mA — 中等，建议对比示波器看纹波来源" % ia_std_ma)
        else:
            print("  [HIGH] Ia 静态噪声 std=%.2f mA — 偏大，支撑硬件重设计评估" % ia_std_ma)
        report["noise"] = noise
    report["noise_n_frames"] = n

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "static_noise_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("报告已保存: %s" % out)

    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
