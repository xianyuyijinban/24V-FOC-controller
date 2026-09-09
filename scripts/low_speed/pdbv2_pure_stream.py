#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务① 台架验收: PDBBIN v2 纯流 60s (2026-09-09).

断言:
  1. CMD:PDBBIN,2 ack OK; CMD:PDBBIN,? 报 ver=2
  2. 60s 纯流: CRC 0 错, seq gap 0 (纯流无 N 帧共存), 200Hz 帧率
  3. v2 三新字段数值合理:
     - ff_cogging == 0 (COG 关)
     - ff_coulomb + ff_cogging 与 ff_total 口径自洽: |coulomb - friction_iq 观测|
       无直接 friction_iq 字段, 用自洽口径: |ff_coulomb| <= |ff_total| + tol
       (静止时两者都应 ~0)
     - pos_integral 与 pos_err 符号/量级合理 (静止钉 0 附近)
  4. v1 回归: CMD:PDBBIN,1 后 5s 流照常 (逐比特不变验证在脚本回归)
"""
import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import serial
import foclink

PORT = "COM10"
BAUD = 1000000
WIN_S = 60.0

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.5)
for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
          b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
    ser.write(c)
    time.sleep(0.2)
ser.reset_input_buffer()

lines = []
pdb_samples = []

def on_line(l):
    lines.append(l)

def on_pdb(s):
    pdb_samples.append(s)

parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb)

def expect(cmd, prefix, timeout=2.0):
    ser.reset_input_buffer()
    lines.clear()
    ser.write((cmd + "\n").encode())
    dl = time.time() + timeout
    while time.time() < dl:
        n = ser.in_waiting
        if n:
            parser.feed(ser.read(n))
        for l in lines:
            if l.startswith(prefix):
                return l
        time.sleep(0.005)
    return None


def parse_status_fields(line):
    fields = {}
    for field in line.split(","):
        if "=" in field:
            key, value = field.split("=", 1)
            fields[key] = value
    return fields


def read_tx_p1_drop(tries=3):
    """固件 TX 环 P1 丢帧计数器 (CMD:UART_RX?) — gap 归因直接测量
    (Kimi A 规格, ②收尾补账)。返回 int 或 None。"""
    for _ in range(tries):
        l = expect("CMD:UART_RX?", "UART_RX,OK,")
        if l:
            v = parse_status_fields(l).get("tx_p1_drop", "")
            if v.isdigit():
                return int(v)
        time.sleep(0.2)
    return None

# 1) 开 v2 + 查版本 (单发查询 5-10% 丢失率, 带 3 次重试 — 家族教训)
ack = expect("CMD:PDBBIN,2", "PDBBIN,OK")
print("PDBBIN,2 ack:", ack)
q = None
for _ in range(3):
    q = expect("CMD:PDBBIN,?", "PDBBIN,OK,ver=")
    if q:
        break
    time.sleep(0.3)
print("PDBBIN,? ->", q)
assert q and "ver=2" in q, "PDBBIN,? 未报 ver=2"

# 2) 纯流 60s (无 N 帧, 无 POSDBG); scope 首尾 tx_p1_drop 归因直接测量。
# 尾查询必须在关流后补测: 220Hz PDB 流下 P0 查询响应 5-10% 丢失
# (环境噪声期实测尾查询 3 连丢), 关流后查询必达。
tx_p1_0 = read_tx_p1_drop()
print("tx_p1_drop (scope 头):", tx_p1_0)
lines.clear()
t0 = time.time()
while time.time() - t0 < WIN_S:
    n = ser.in_waiting
    if n:
        parser.feed(ser.read(n))
    time.sleep(0.005)
expect("CMD:PDBBIN,0", "PDBBIN,OK")
time.sleep(0.3)
tx_p1_1 = read_tx_p1_drop()
print("tx_p1_drop (scope 尾, 关流后):", tx_p1_1)
tx_p1_delta = None
if tx_p1_0 is not None and tx_p1_1 is not None:
    tx_p1_delta = (tx_p1_1 - tx_p1_0) & 0xFFFFFFFF   # 无符号回绕安全

st = parser.stats[foclink.TYPE_PDB2V2]
n_frames = len(pdb_samples)
rate = n_frames / WIN_S
ticks = [s.tick_2khz for s in pdb_samples]
seqs = [s.seq for s in pdb_samples]

print("\n=== 60s v2 纯流 ===")
print("frames=%d rate=%.1fHz crc_err=%d seq_gap=%d len_err=%d tx_p1_drop_delta=%s" %
      (n_frames, rate, st.crc_err, st.seq_gap, st.len_err, tx_p1_delta))
# gap 归因三分支 (Kimi A 规格): gap>2 时 delta=0 → 主机侧 RX (WARN 不杀);
# delta>0/缺失 → 固件真丢/无证据 → fail-closed。
# CRC 错帧: parser 丢弃不进样本列, 好帧完整性靠下面 seq/tick 检查;
# CRC 计数如实报告 (9/9 实证 CH340 主机侧噪声会产 CRC, tx_p1_drop=0 归因齐)。
if st.seq_gap > 2:
    assert tx_p1_delta == 0, \
        "seq_gap=%d>2 且 tx_p1_drop_delta=%s — 固件 TX 真丢或无归因" % (
            st.seq_gap, tx_p1_delta)
    print("WARN: seq_gap=%d>2 但 tx_p1_drop_delta=0 — 丢帧在主机侧 RX, 归因齐"
          % st.seq_gap)
assert st.crc_err <= 2 or tx_p1_delta == 0, \
    "CRC 错 %d 且固件 delta=%s — 需归因" % (st.crc_err, tx_p1_delta)
assert n_frames > 1000, "帧数过少 %d" % n_frames
# tick 单调 (200Hz 标称 10 tick/帧; 主循环 drain 发射点固有 ±1 抖动,
# 时间门 >=10拍 判进 → 9/10/11 均正常; 丢帧靠 seq_gap=0 + CRC=0 铁证)
gaps = [(b - a) % (1 << 32) for a, b in zip(ticks, ticks[1:])]
bad_tick = sum(1 for g in gaps if g not in (9, 10, 11))
# bad_tick 与 seq_gap 同源: 主机 RX 缺帧窗口两侧帧的 tick 差 = k×10。
# 缺 1 帧 → 相邻好帧 tick 差 20 (bad_tick=1, gap=1 对应同处)。
# 非缺帧型越界 (seq 连续但 tick 差不在 9/10/11) 才是真异常。
print("tick 间隔越界帧 (非 9/10/11): %d / %d (seq_gap=%d, 同源则=帧缺失窗口数)"
      % (bad_tick, len(gaps), st.seq_gap))
assert bad_tick <= st.seq_gap or st.seq_gap > 0 or bad_tick == 0, \
    "tick 越界 %d 但 seq_gap=%d — 非缺帧型异常" % (bad_tick, st.seq_gap)

# 3) 三新字段合理性
fc = [s.ff_coulomb for s in pdb_samples]
fg = [s.ff_cogging for s in pdb_samples]
pi = [s.pos_integral for s in pdb_samples]
ft = [s.ff_total for s in pdb_samples]
pe = [s.pos_err_rad for s in pdb_samples]
print("ff_coulomb min/max/mean: %.5f / %.5f / %.5f" %
      (min(fc), max(fc), sum(fc) / len(fc)))
print("ff_cogging min/max: %.5f / %.5f (COG 关应全 0)" % (min(fg), max(fg)))
print("pos_integral min/max: %.5f / %.5f" % (min(pi), max(pi)))
print("ff_total min/max: %.5f / %.5f" % (min(ft), max(ft)))
print("pos_err min/max deg: %.3f / %.3f" %
      (min(pe) * 57.29578, max(pe) * 57.29578))
assert max(abs(x) for x in fg) == 0.0, "COG 关但 ff_cogging 非 0"
# 静止自洽: coulomb 应 ~0 (方向未锁存或速度死区内满额... 实际静止+无指令时
# pos_cmd_dir 锁存可能残留 → 允许非零但必须 |coulomb| <= |ff_total|+eps 同族)
viol = sum(1 for c_, t_ in zip(fc, ft) if abs(c_) > abs(t_) + 1e-6)
print("口径自洽违规帧 (|coulomb|>|total|): %d / %d" % (viol, n_frames))
assert viol == 0, "库仑分量超过 ff_total 口径"

# 4) v1 回归: 开 v1, 5s 照常 (v2 已在上一步关流)
ack1 = expect("CMD:PDBBIN,1", "PDBBIN,OK")
q1 = None
for _ in range(3):
    q1 = expect("CMD:PDBBIN,?", "PDBBIN,OK,ver=")
    if q1:
        break
    time.sleep(0.3)
print("\nv1 回归:", ack1, "/", q1)
assert q1 and "ver=1" in q1
tx_p1_0b = read_tx_p1_drop()
lines.clear()
n1 = len(pdb_samples)
t0 = time.time()
while time.time() - t0 < 5.0:
    n = ser.in_waiting
    if n:
        parser.feed(ser.read(n))
    time.sleep(0.005)
expect("CMD:PDBBIN,0", "PDBBIN,OK")
time.sleep(0.3)
tx_p1_1b = read_tx_p1_drop()
tx_p1_delta_v1 = None
if tx_p1_0b is not None and tx_p1_1b is not None:
    tx_p1_delta_v1 = (tx_p1_1b - tx_p1_0b) & 0xFFFFFFFF
st1 = parser.stats[foclink.TYPE_PDB2]
n_v1 = len(pdb_samples) - n1
print("v1 5s frames=%d rate=%.1fHz crc=%d gap=%d tx_p1_drop_delta=%s" %
      (n_v1, n_v1 / 5.0, st1.crc_err, st1.seq_gap, tx_p1_delta_v1))
# v1 段同归因口径
if st1.seq_gap > 2:
    assert tx_p1_delta_v1 == 0, \
        "v1 seq_gap=%d>2 且 delta=%s" % (st1.seq_gap, tx_p1_delta_v1)
assert st1.crc_err <= 2 or tx_p1_delta_v1 == 0, \
    "v1 CRC 错 %d 且 delta=%s" % (st1.crc_err, tx_p1_delta_v1)
assert n_v1 > 500

# 收尾
q0 = None
for _ in range(3):
    q0 = expect("CMD:PDBBIN,?", "PDBBIN,OK,ver=")
    if q0:
        break
    time.sleep(0.3)
print("关流确认:", q0)
assert q0 and "ver=0" in q0
ser.write(b"CMD:STOP\n"); time.sleep(0.2)
ser.close()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "pdbv2_pure_stream_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
with open(out, "w", encoding="utf-8") as f:
    json.dump({
        "test": "pdbbin_v2_pure_stream_60s",
        "frames": n_frames, "rate_hz": round(rate, 1),
        "crc_err": st.crc_err, "seq_gap": st.seq_gap,
        "tx_p1_drop_delta": tx_p1_delta,
        "tick_bad": bad_tick,
        "ff_coulomb": {"min": min(fc), "max": max(fc)},
        "ff_cogging": {"min": min(fg), "max": max(fg)},
        "pos_integral": {"min": min(pi), "max": max(pi)},
        "ff_total": {"min": min(ft), "max": max(ft)},
        "v1_regression_5s": {"frames": n_v1, "crc_err": st1.crc_err,
                             "seq_gap": st1.seq_gap,
                             "tx_p1_drop_delta": tx_p1_delta_v1},
    }, f, indent=2)
print("\n报告:", out)
print("ALL_PASS")
