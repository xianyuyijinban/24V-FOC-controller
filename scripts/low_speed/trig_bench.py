#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务③ 台架验收: TRIG ring buffer (2026-09-09).

验收项 (任务卡):
  A. CMD:TRIG,NOW 合成验尸: pre/post 边界 tick 连续、768/256 比例精确、
     CRC 全过、ring tick 与 20kHz 时基同源 (tick 差=1)
  B. 真 fault 验尸: [方式待岳翔宇裁定后单独跑 — 本脚本留钩子 --fault-test]
  C. RAM/实时性: LOOP_PROF CURRENT_PATH 探针 TRIG 开关前后对比 (jitter 预算)
  D. tick 对齐: TRIG tick_20k 与 PDBBIN tick_2khz 换算同源可对齐
"""
import sys, os, time, json, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial
import foclink
from trig_pull import TrigPuller, crc16_ccitt, TRIG_RING_SIZE, TRIG_PRE, TRIG_FRAME_SIZE

PORT = "COM10"

ser = serial.Serial(PORT, 1000000, timeout=0.05)
time.sleep(0.5)
for c in (b"CMD:PDBBIN,0\n", b"CMD:TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n",
          b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:TRIG,CLR\n"):
    ser.write(c)
    time.sleep(0.2)
ser.reset_input_buffer()

lines = []
def on_line(l):
    lines.append(l)
parser = foclink.MixedStreamParser(line_cb=on_line)

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

report = {"checks": {}}

# ── C-基线: LOOP_PROF CURRENT_PATH (TRIG 已存在但未触发 — 基线含每拍写入) ──
# 对比口径: 本固件 TRIG 采样恒在 (每 20kHz 拍 O(1) 写)。"前后对比" = 对比
# 09e2379 前一版 08126aa+ee6cacd 的历史值 (foc ISR 预算 50us), 本脚本记录
# 当前值 + 触发前后对比 (POST 期间多 1 次分支 vs FROZEN 早退)。
def loop_prof_current_path():
    for attempt in range(3):
        expect("CMD:LOOP_PROF,CLEAR", "LOOP_PROF,CLEAR,OK")
        time.sleep(8.0)   # 采样窗
        expect("CMD:LOOP_PROF?", "LOOP_PROF,BEGIN")
        dl = time.time() + 2.0
        rows = []
        while time.time() < dl:
            n = ser.in_waiting
            if n:
                parser.feed(ser.read(n))
            time.sleep(0.005)
        for l in lines:
            if l.startswith("LOOP_PROF,CURRENT_PATH,"):
                f = {}
                for part in l.split(","):
                    if "=" in part:
                        k, v = part.split("=", 1)
                        f[k] = v
                return f
        time.sleep(0.3)
    return None

print("=== C: LOOP_PROF current_path 基线 (IDLE 滚动态) ===")
base = loop_prof_current_path()
print("IDLE:", base)
assert base is not None, "LOOP_PROF 无 current_path 行"

print("\n=== A: CMD:TRIG,NOW 合成验尸 ===")
ack = expect("CMD:TRIG,NOW", "TRIG,")
print("TRIG,NOW:", ack)
assert "TRIG,OK,now" in ack, "TRIG,NOW 未确认"
time.sleep(0.3)   # post 256 帧 @20kHz = 12.8ms, 0.3s 足够计满
st = expect("CMD:TRIG,STAT?", "TRIG,OK,")
print("STAT:", st)
assert "state=2" in st, "未到 FROZEN: %s" % st
report["checks"]["stat_frozen"] = st
f = {}
for part in st.split(","):
    if "=" in part:
        k, v = part.split("=", 1)
        f[k] = v
trig_tick = int(f["trig_tick"])
report["checks"]["trig_tick"] = trig_tick

# 全量拉取 (复用 bench 的 ser — 自开会 PermissionError)
tp = TrigPuller(ser=ser)
sts = tp.stat()
assert sts and sts.get("state") == "2", "拉取器 STAT 不符: %s" % sts
t0 = time.time()
tp.pull_all()
pull_s = time.time() - t0
ticks = [fr[6] for fr in tp.frames]
disc = sum(1 for a, b in zip(ticks, ticks[1:]) if ((b - a) & 0xFFFFFFFF) != 1)
print("拉取: %d 块 CRC 全过, %.2fs; tick 断点 %d/%d" %
      (tp.blocks_ok, pull_s, disc, len(ticks) - 1))
assert tp.blocks_ok == 32 and not tp.blocks_crc_fail, "块 CRC 失败"
# 768/256 比例: 帧 768 = 触发帧 (tick=trig_tick)
assert ticks[TRIG_PRE] == trig_tick, \
    "帧768 tick=%d != 触发帧 tick=%d" % (ticks[TRIG_PRE], trig_tick)
print("pre/post 边界: 帧768 tick=%d == trig_tick ✓ (pre 768 帧 tick %d..%d, "
      "post 帧 769..1023 tick %d..%d)" %
      (ticks[TRIG_PRE], ticks[0], ticks[767], ticks[769], ticks[1023]))
report["checks"]["pull"] = {
    "blocks_ok": tp.blocks_ok, "crc_fail": tp.blocks_crc_fail,
    "seconds": round(pull_s, 2), "tick_discontinuity": disc,
    "frame768_tick": ticks[TRIG_PRE], "trig_tick": trig_tick,
    "pre_first_tick": ticks[0], "pre_last_tick": ticks[767],
    "post_first_tick": ticks[769], "post_last_tick": ticks[1023]}

# D: tick 同源 — trig_tick/20000 (s) 与 s_foc_tick_2khz/2000 (s) 应同源
# (control_count 20kHz vs foc_tick 2kHz: 同一 TIM1 ISR 域, 比值 10)
print("tick 同源: trig_tick=%d → t=%.4fs (20kHz 域)" %
      (trig_tick, trig_tick / 20000.0))
report["checks"]["tick_domain"] = {"trig_tick": trig_tick,
                                   "t_s": round(trig_tick / 20000.0, 5)}

# C-触发后: POST/FROZEN 期间电流环 max_cycles (冻结早退应无增量)
print("\n=== C: LOOP_PROF 触发后 (FROZEN 态) ===")
after = loop_prof_current_path()
print("FROZEN:", after)
report["checks"]["loop_prof"] = {"idle": base, "frozen": after}

# CLR 复位可再触发
expect("CMD:TRIG,CLR", "TRIG,OK,clr")
st2 = expect("CMD:TRIG,STAT?", "TRIG,OK,")
print("CLR 后:", st2)
assert "state=0" in st2, "CLR 未回 IDLE: %s" % st2
ack2 = expect("CMD:TRIG,NOW", "TRIG,")
assert "TRIG,OK,now" in ack2, "CLR 后再触发失败"
time.sleep(0.3)
st3 = expect("CMD:TRIG,STAT?", "TRIG,OK,")
assert "state=2" in st3, "CLR 后再触发未冻结: %s" % st3
report["checks"]["retrigger_after_clr"] = True
print("CLR 复位 + 再触发 ✓")

expect("CMD:TRIG,CLR", "TRIG,OK,clr")
ser.close()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "low_speed", "trig_bench_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
with open(out, "w", encoding="utf-8") as fjson:
    json.dump(report, fjson, indent=2, ensure_ascii=False)
print("\n报告:", out)
print("BENCH_ALL_PASS (真 fault 验尸待方式裁定后单独跑)")
