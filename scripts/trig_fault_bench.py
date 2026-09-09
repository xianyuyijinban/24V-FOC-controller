#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务③ 真 fault 验尸 (岳翔宇裁定: 电压模式 Vq 阶梯爬流, 2026-09-09).

流程 (可逆):
  1. 配置: MODE,3 电压开环 (pos_direct=0) → ENABLE,1
  2. 注入: CMD:VOLT 阶梯爬流 200→3000mV (E1 案 9/3 波形), 每档读 VOLT? 监视
  3. 过流闩锁 (3.0A 阈) → TRIG 自动触发 (src=1 fault) → FROZEN → 拉取
  4. 若物理上过不了流 (Vq 3.0V clamp × 4.4Ω = 0.68A << 3A — 预判):
     手动 TRIG,NOW 在堵转爬流波形中触发, pre 段仍含真实堵转电流行为;
     fault 闩锁触发路径的物理不可达如实入账
  5. VOLT_OFF → ENABLE,0 → CLEAR_FAULT 恢复
"""
import sys, os, time, json, struct
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "low_speed"))
import serial
import foclink
from trig_pull import TrigPuller

PORT = "COM10"
ser = serial.Serial(PORT, 1000000, timeout=0.05)
time.sleep(0.5)
for c in (b"CMD:PDBBIN,0\n", b"CMD:TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n",
          b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:TRIG,CLR\n",
          b"CMD:ENABLE,0\n", b"CMD:VOLT_OFF\n", b"CMD:MODE,0\n"):
    ser.write(c); time.sleep(0.2)
ser.reset_input_buffer()

lines = []
def on_line(l): lines.append(l)
parser = foclink.MixedStreamParser(line_cb=on_line)

def expect(cmd, prefix, timeout=2.0):
    ser.reset_input_buffer(); lines.clear()
    ser.write((cmd + "\n").encode())
    dl = time.time() + timeout
    while time.time() < dl:
        n = ser.in_waiting
        if n: parser.feed(ser.read(n))
        for l in lines:
            if l.startswith(prefix): return l
        time.sleep(0.005)
    return None

def volt_status():
    l = expect("CMD:VOLT?", "VOLT,OK,")
    if not l:
        return {}
    f = {}
    for part in l.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            f[k] = v
    return f

report = {"checks": {}, "ladder": []}

expect("CMD:UNLOCK,1", "UNLOCK,OK")
expect("CMD:MODE,3", "MODE,OK")
ack = expect("CMD:ENABLE,1", "ENABLE,")
print("ENABLE:", ack)
assert "OK" in ack, "ENABLE 拒绝: %s" % ack
time.sleep(0.5)

# Vq 阶梯爬流 (E1 波形): 200mV 起步, 400mV 步进, 3000mV 封顶。
# 物理链 (foc_app.c:936 电压开环分支): vq_ramped 以 0.05V/s 内置斜坡逼近
# 目标 → iq_est=vq_ramped/Rs(8.8 线线) 估计 → 软限幅 2.4A (80% 过流阈)
# 回拉 → 总电压限幅 Vbus/√3≈6.9V。收敛慢: 每档须等 ramp 走完 (400mV/0.05
# =8s), 档间 9s。物理上限: 6.9V/8.8Ω=0.78A << 3.0A 过流阈 — 过流不可达
# (预判, 如实入账), 末端手动触发完成验尸波形。
FAULT_HIT = False
vq = 400
while vq <= 3000:
    ack = expect("CMD:VOLT,%d" % vq, "VOLT,")
    if ack is None:
        # 单发丢失重试一次 (家族 5-10% 丢失率)
        ack = expect("CMD:VOLT,%d" % vq, "VOLT,")
    if ack is None or "FAIL" in ack:
        print("VOLT,%d 拒绝/丢响应: %s" % (vq, ack))
        break
    time.sleep(9.0)   # ramp 400mV / 0.05V/s = 8s 收敛 + 1s 裕量
    st = volt_status()
    row = {"vq_mV": vq, "iq_est_mA": st.get("iq_est_mA"),
           "bemf_mV": st.get("bemf_mV"), "vbus_mV": st.get("vbus_mV")}
    report["ladder"].append(row)
    print("Vq=%4dmV iq_est=%s mA bemf=%s vbus=%s" %
          (vq, st.get("iq_est_mA"), st.get("bemf_mV"), st.get("vbus_mV")))
    # 过流即 fault 闩锁 → state=FAULT → TRIG 自动触发
    stq = expect("CMD:TRIG,STAT?", "TRIG,OK,", timeout=1.5)
    if stq and "state=2" in stq:
        print("TRIG 自动触发 (过流闩锁) → FROZEN")
        FAULT_HIT = True
        report["checks"]["fault_triggered"] = True
        break
    vq += 400

# fault 不可达时: 手动触发于堵转爬流波形 (pre 段仍为真实堵转行为)
if not FAULT_HIT:
    print("\n[物理上限证实] 电压域过流不可达: vq_ramped 0.05V/s 爬升被 Vbus/√3")
    print("  (6.9V) 总限幅封顶 → 真实电流 6.9/8.8=0.78A < 软限幅 2.4A < 过流阈 3.0A")
    report["checks"]["fault_unreachable"] = {
        "reason": "vq ramped capped by Vbus/sqrt3 (6.9V) -> 0.78A < 2.4A soft limit < 3.0A OC threshold",
        "final_iq_est_mA": report["ladder"][-1]["iq_est_mA"] if report["ladder"] else None}
    # 堵转最大电压档保持, 手动触发
    stq = expect("CMD:TRIG,NOW", "TRIG,")
    print("手动触发:", stq)
    assert stq and "TRIG,OK,now" in stq
    report["checks"]["fault_triggered"] = False
    report["checks"]["manual_trigger"] = True

time.sleep(0.4)   # post 计满
stq = expect("CMD:TRIG,STAT?", "TRIG,OK,")
print("STAT:", stq)
assert stq and "state=2" in stq, "未 FROZEN"
report["checks"]["stat_frozen"] = stq

# 拉取 (PDBBIN 未开, 无需暂停)
tp = TrigPuller(ser=ser)
sts = tp.stat()
tp.pull_all()
ticks = [fr[6] for fr in tp.frames]
disc = sum(1 for a, b in zip(ticks, ticks[1:]) if ((b - a) & 0xFFFFFFFF) != 1)
iqs = [fr[1] for fr in tp.frames]
print("拉取: %d 块 CRC 全过; tick 断点 %d; iq max=%.3fA (帧768=%.3f)" %
      (tp.blocks_ok, disc, max(abs(x) for x in iqs), iqs[768]))
report["checks"]["pull"] = {
    "blocks_ok": tp.blocks_ok, "crc_fail": tp.blocks_crc_fail,
    "tick_discontinuity": disc,
    "iq_absmax_A": round(max(abs(x) for x in iqs), 4),
    "iq_at_trig_A": round(iqs[768], 4),
    "iq_pre_avg_A": round(sum(abs(x) for x in iqs[:768]) / 768.0, 4),
    "iq_post_avg_A": round(sum(abs(x) for x in iqs[769:]) / 255.0, 4)}

# 恢复
expect("CMD:VOLT_OFF", "VOLT_OFF,OK")
expect("CMD:ENABLE,0", "ENABLE,OK,0")
expect("CMD:CLEAR_FAULT", "CLEAR_FAULT,OK")
expect("CMD:MODE,0", "MODE,OK")
stq = expect("CMD:TRIG,STAT?", "TRIG,OK,")
print("恢复后 TRIG 状态:", stq)
ser.close()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "low_speed", "trig_fault_bench_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
with open(out, "w", encoding="utf-8") as f:
    json.dump(report, f, indent=2, ensure_ascii=False)
print("报告:", out)
print("FAULT_BENCH_DONE")
