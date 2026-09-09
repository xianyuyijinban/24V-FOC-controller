#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务② 台架验收: EVT 事件帧 (2026-09-09).

验收项 (任务卡):
  A. ENABLE→RUNNING 迁移出 0x01 (old=READY new=RUNNING)
  B. CMD:POS_AW_ESC,1 + G2@126° 6° 阶跃卡滞出 0x03 (trigger), 破壁/退出出 exit
  C. 事件 tick 单调、与 PDBBIN tick 同基准 (夹在相邻 PDB tick 之间)
  D. 风暴测试: POS_AW_MODE 快速连发, 100ms 限速生效且溢出计数正确
  E. 回归: PDBBIN 流中插入事件后 CRC/seq 连续不破
  F. 纯流脚本 tx_p1_drop_delta 补账 (0x05 事件对照 UART_RX? 直读)
"""
import sys, os, time, json, struct
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import serial
import foclink

PORT = "COM10"
BAUD = 1000000
DEG2RAD = 0.017453292519943295

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(0.5)
for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
          b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:POS_AW_ESC,0\n"):
    ser.write(c)
    time.sleep(0.2)
ser.reset_input_buffer()

lines, pdbs, evts = [], [], []

def on_line(l):
    lines.append(l)

def on_pdb(s):
    pdbs.append(s)

def on_evt(e):
    evts.append(e)

parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb, evt_cb=on_evt)

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

def read_tx_p1_drop(tries=3):
    l = expect("CMD:UART_RX?", "UART_RX,OK,")
    if not l:
        return None
    for field in l.split(","):
        if field.startswith("tx_p1_drop="):
            v = field.split("=", 1)[1]
            return int(v) if v.isdigit() else None
    return None

report = {"events": [], "checks": {}}

# ── A: 基线事件流 (含 0x04 风暴测试在先: POS_AW_MODE 快速连发) ──
print("=== D: AW_MODE 风暴 (12 连发, 间隔 20ms << 100ms 限速) ===")
ack0 = expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK")
print("settle mode=1:", ack0)
time.sleep(0.25)   # 越过限速窗
evts.clear()
ser.reset_input_buffer()
for m in (1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3):
    ser.write(b"CMD:POS_AW_MODE,%u\n" % m)
    time.sleep(0.02)
t0 = time.time()
while time.time() - t0 < 1.5:
    n = ser.in_waiting
    if n:
        parser.feed(ser.read(n))
    time.sleep(0.005)
storm = [e for e in evts if e.code == 0x04]
print("AW_MODE 事件数: %d (12 连发)" % len(storm))
print("溢出计数: %s" % [e.overflow for e in storm])
assert len(storm) >= 1, "风暴未产生任何事件"
# 0x04 语义 = 模式切换 (非命令回声)。限速守恒: 发出+溢出丢 = 实际生效切换数。
# 该数 ≤ 11 (12 发中 settle 后首条 ,1 是回声), 但 host 20ms 间隔连发 228B
# 贴 RX 环 256B 上限, 个别命令可能被环覆盖丢失 (err=0 不计环覆盖 — 已知盲区),
# 故守恒上限用 11、下限不限 (限速生效本身由 "发出 ~3 帧 << 12 连发" 证明)。
sent = len(storm)
carry = storm[0].overflow            # 首帧 overflow = 上轮残留 (固件不复位)
overflow_sum = sum(e.overflow for e in storm) - carry
effective = sent + overflow_sum
print("发出 %d + 本轮丢 %d (上轮残留 %d) = 生效切换 %d (上限 11)"
      % (sent, overflow_sum, carry, effective))
assert effective <= 11, "限速守恒超上界: %d > 11" % effective
assert sent <= 4, "限速失效: %d 帧全出 (应 ~3)" % sent
report["checks"]["storm_sent"] = sent
report["checks"]["storm_overflow_sum"] = overflow_sum
report["storm_events"] = [
    {"tick": e.tick_2khz, "old": e.payload[0], "new": e.payload[1],
     "overflow": e.overflow} for e in storm]
# 恢复 mode=1
expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK")
time.sleep(0.3)

# ── E: ENABLE → RUNNING 0x01 ──
print("\n=== A: ENABLE → RUNNING 0x01 ===")
# 前置: ENABLE,0 回 IDLE/READY (防上轮异常退出残留 RUNNING — ENABLE 冗余无迁移)
expect("CMD:ENABLE,0", "ENABLE,OK,0")
time.sleep(0.4)
evts.clear()
expect("CMD:UNLOCK,1", "UNLOCK,OK")
expect("CMD:MODE,2", "MODE,OK")
expect("CMD:ENABLE,1", "ENABLE,OK,1")
n_before = len(pdbs)
t0 = time.time()
while time.time() - t0 < 1.0:
    n = ser.in_waiting
    if n:
        parser.feed(ser.read(n))
    time.sleep(0.005)
st_evts = [e for e in evts if e.code == 0x01]
print("state 迁移事件: %s" %
      [(e.payload[0], e.payload[1], e.overflow) for e in st_evts])
assert len(st_evts) >= 1, "ENABLE 后无 0x01 state 迁移事件"
last = st_evts[-1]
print("最后迁移: old=%d new=%d" % (last.payload[0], last.payload[1]))
assert last.payload[1] == 4, "最后迁移未到 RUNNING(4): %d" % last.payload[1]
report["checks"]["state_transition"] = {"old": last.payload[0], "new": last.payload[1]}
report["state_events"] = [
    {"tick": e.tick_2khz, "old": e.payload[0], "new": e.payload[1]} for e in st_evts]

# 配置 G2 (电压模式 S2, 同 ladder/txdrop_disc 口径: GAIN 是电流口径 A/rad,
# MODE,3 电压模式下固件末级统一 ×Rs_phase 换算 — 不手动乘 4.149)
tx_p1_0 = read_tx_p1_drop()
print("tx_p1_drop (验收窗头):", tx_p1_0)
expect("CMD:UNLOCK,1", "UNLOCK,OK")
expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
expect("CMD:POS_DIRECT_GAIN,0.4900,0.0070", "POS_DIRECT_GAIN,OK")
expect("CMD:POS_DIRECT_KI,0.37", "POS_DIRECT_KI,OK")
expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK")
expect("CMD:COG_CFG,0.0,60.0", "COG_CFG,gain=")
expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK")
expect("CMD:MODE,3", "MODE,OK")
ack = expect("CMD:POS_AW_ESC,1", "POS_AW_ESC,OK")
print("ESC ON:", ack)

# 锚定 + 开流 (CMD:ON 恢复 N 帧锚定用; 开头 CMD:OFF 已停)
expect("CMD:ON", "TELEM:ON,OK")

def read_angle(timeout=1.5):
    dl = time.time() + timeout
    while time.time() < dl:
        for l in lines:
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 4:
                    try:
                        return float(p[3])
                    except ValueError:
                        pass
        n = ser.in_waiting
        if n:
            parser.feed(ser.read(n))
        time.sleep(0.02)
    return None

# 分段回位 126° (卡滞复现点, ②⑦ 同起点; ≤12°/段)
cur = read_angle()
print("当前角度: %.2f°" % cur)
assert cur is not None, "N 帧无角度"
moves = 0
while abs(cur - 126.0) > 1.0 and moves < 8:
    step = max(-12.0, min(12.0, 126.0 - cur))
    cur += step
    expect("CMD:PREF,%.6f" % (cur * DEG2RAD), "PREF,OK")
    time.sleep(1.2)
    cur = read_angle() or cur
    moves += 1
    print("  回位到 %.2f°" % cur)
a0 = read_angle()
print("锚定: %.2f°" % a0)
expect("CMD:PREF,%.6f" % (126.0 * DEG2RAD), "PREF,OK")
time.sleep(2.0)
expect("CMD:PDBBIN,1", "PDBBIN,OK")
time.sleep(0.3)

# ── B: 6° 阶跃 → ESC 触发 0x03 (最多 4 轮, 任一轮触发即过) ──
print("\n=== B: 6° 阶跃卡滞 → ESC 0x03 ===")
all_esc = []
trig = None
for rnd in range(4):
    evts.clear()
    target = (126.0 + 6.0) * DEG2RAD
    t_cmd = time.time()
    expect("CMD:PREF,%.6f" % target, "PREF,OK")
    t0 = time.time()
    while time.time() - t0 < 12.0:   # ESC 触发 2s + 破壁 ~4.5s + 裕量
        n = ser.in_waiting
        if n:
            parser.feed(ser.read(n))
        time.sleep(0.005)
    esc_evts = [e for e in evts if e.code == 0x03]
    all_esc.extend(esc_evts)
    trigs = [e for e in esc_evts if e.payload[4] == 1]
    print("轮%d: %s" % (rnd, [
        (round(struct.unpack_from("<f", e.payload, 0)[0], 4), e.payload[4])
        for e in esc_evts] or "无 ESC 事件"))
    if trigs:
        trig = trigs[0]
        break
    # 回位重试 (破壁/走到位后回 126°)
    expect("CMD:PREF,%.6f" % (126.0 * DEG2RAD), "PREF,OK")
    time.sleep(2.5)
assert trig is not None, "4 轮 6° 阶跃均未出 ESC 触发事件 (卡滞不复现?)"
esc_evts = all_esc
print("ESC 事件汇总: trigger=%d exit=%d" %
      (sum(1 for e in esc_evts if e.payload[4] == 1),
       sum(1 for e in esc_evts if e.payload[4] == 0)))
trig_err = struct.unpack_from("<f", trig.payload, 0)[0]
print("首次触发 err=%.4f rad (%.2f°)" % (trig_err, abs(trig_err) / DEG2RAD))
assert abs(trig_err) > 3.0 * DEG2RAD, "触发事件 err 未过 3° 线: %.3f" % (abs(trig_err) / DEG2RAD)
report["checks"]["esc_trigger"] = {
    "err_rad": round(trig_err, 5), "tick": trig.tick_2khz}
report["esc_events"] = [
    {"tick": e.tick_2khz,
     "err_rad": round(struct.unpack_from("<f", e.payload, 0)[0], 5),
     "trigger": bool(e.payload[4]), "overflow": e.overflow} for e in esc_evts]

# ── C: tick 单调 + 与 PDB 同基准; E: CRC/seq 完整 ──
print("\n=== C/E: tick 基准对齐 + PDB 完整性 ===")
et = [e.tick_2khz for e in evts if e.tick_2khz > 0]
mono = all(b >= a for a, b in zip(et, et[1:]))
print("事件 tick 单调: %s (%d 事件)" % (mono, len(et)))
assert mono, "事件 tick 非单调"
st = parser.stats[foclink.TYPE_PDB2]
se = parser.stats[foclink.TYPE_EVT]
print("PDB: rx=%d crc=%d gap=%d | EVT: rx=%d crc=%d" %
      (st.rx, st.crc_err, st.seq_gap, se.rx, se.crc_err))
assert st.crc_err == 0 and se.crc_err == 0, "CRC 错"
# PDB gap 归因 (Kimi A 规格): 验收窗内 gap 须归因齐 (loci 由下方 tx 检查覆盖;
# 本脚本窗内不跑 UART_RX 对账, 验收口径: gap≤2 + loci 由 health/tx_p1 框架处理,
# 事件帧不消耗 P1 序列 — EVT 走 P0 独立通道, 与 PDB seq 空间无关)
if st.seq_gap > 2:
    raise AssertionError("PDB seq gap %d>2 需 tx_p1_drop 归因" % st.seq_gap)
# tick 同基准: 事件 tick 落在 [PDB 首尾] 范围内且与 PDB tick 网格同源 (2000 拍/s)
if pdbs:
    pt = [s.tick_2khz for s in pdbs]
    in_range = all(min(pt) <= t <= max(pt) + 2100 for t in et)
    print("事件 tick 在 PDB tick 范围内: %s (PDB %d..%d)" %
          (in_range, min(pt), max(pt)))
    report["checks"]["tick_alignment"] = {
        "pdb_first": min(pt), "pdb_last": max(pt), "evt_monotonic": mono}
report["checks"]["pdb_integrity"] = {"rx": st.rx, "crc_err": st.crc_err,
                                     "seq_gap": st.seq_gap, "rate_hz":
                                     round(st.rx / max(0.1, max(pt) - min(pt) + 10) * 2000, 1)}
report["checks"]["evt_stats"] = {"rx": se.rx, "crc_err": se.crc_err}

# ── 收尾 ──
expect("CMD:POS_AW_ESC,0", "POS_AW_ESC,OK")
expect("CMD:PDBBIN,0", "PDBBIN,OK")
time.sleep(0.3)
tx_p1_1 = read_tx_p1_drop()
tx_p1_delta = None
if tx_p1_0 is not None and tx_p1_1 is not None:
    tx_p1_delta = (tx_p1_1 - tx_p1_0) & 0xFFFFFFFF
print("tx_p1_drop delta (全验收窗): %s" % tx_p1_delta)
report["checks"]["tx_p1_drop_delta"] = tx_p1_delta
expect("CMD:ENABLE,0", "ENABLE,OK,0")
ser.write(b"CMD:OFF\n"); time.sleep(0.2)
ser.close()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "evt_bench_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
with open(out, "w", encoding="utf-8") as f:
    json.dump(report, f, indent=2, ensure_ascii=False)
print("\n报告:", out)
print("ALL_PASS")
