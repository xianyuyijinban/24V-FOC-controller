#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vfoc_nopower_verify.py — FOC_MODE_VOLTAGE 无功率命令链路验证 (S0, 移植自 vfoc 分支)

C9 移植审查 (参考不照抄, 三处历史 bug 逐项核对):
1. drain: 固定窗口 + 上限 (20 批), 防遥测流死循环 — vfoc 版保留
2. gating: vfoc 版只停 CMD:OFF+RATE,0; 当前分支新增 PDBBIN 二进制流 (200Hz) 会
   撞碎文本响应 — 必须加 CMD:PDBBIN,0 (历史 08-30 教训: 二进制帧撞碎文本行)
3. 环绕: 本脚本无角度计算, 不涉及

无功率 = 不上 ENABLE, 只验证命令链 (MODE 3 / VOLT / DT 全套) 返回状态。
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = 1000000


def drain(ser, t=0.3):
    """固定窗口读（总量上限防遥测流死循环），过滤 C,/N, 遥测帧"""
    time.sleep(t)
    out = b""
    for _ in range(20):
        if not ser.in_waiting:
            break
        out += ser.read(ser.in_waiting)
        time.sleep(0.01)
    text = out.decode(errors="replace")
    lines = [l.strip() for l in text.splitlines()
             if l.strip() and not l.startswith(("C,", "N,"))]
    for line in lines:
        print(f"    {line}")
    return "\n".join(lines)


def cmd(ser, c, wait=0.4):
    ser.write((c + "\r\n").encode())
    time.sleep(0.05)
    resp = drain(ser, wait)
    print(f">>> {c}")
    for line in resp.splitlines():
        if line.strip():
            print(f"    {line.strip()}")
    return resp


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(0.5)
    drain(ser, 1.0)

    print("=== 0. Silence telemetry (avoid flooding) ===")
    # 当前分支: CMD:OFF 停 N 帧 + CMD:PDBBIN,0 停二进制流 (vfoc 版无 PDBBIN, 新增)
    cmd(ser, "CMD:OFF")
    cmd(ser, "CMD:RATE,0")
    cmd(ser, "CMD:PDBBIN,0")
    cmd(ser, "CMD:POSDBG,0")
    cmd(ser, "CMD:STOP")
    cmd(ser, "CMD:CLEAR_FAULT")

    print("=== 1. Firmware alive check ===")
    r = cmd(ser, "CMD:VOLT?")
    assert "VOLT,OK" in r, "FAIL: firmware not responding"

    print("=== 2. Enter voltage mode (CMD:MODE,3) ===")
    r = cmd(ser, "CMD:MODE,3")
    assert "MODE,OK,3" in r, "FAIL: mode switch rejected"

    print("=== 3. Set Vq=200mV ===")
    r = cmd(ser, "CMD:VOLT,200")
    assert "VOLT,OK,200" in r, "FAIL: VOLT set rejected"

    print("=== 4. Query state ===")
    r = cmd(ser, "CMD:VOLT?")
    assert "vq_ref_mV=200" in r, "FAIL: vq_ref not reflected"

    print("=== 5. Range check: 9999mV should clamp to 3000mV ===")
    cmd(ser, "CMD:VOLT,9999")
    r = cmd(ser, "CMD:VOLT?")
    assert "vq_ref_mV=3000" in r, "FAIL: clamp not working"

    print("=== 6. VOLT_OFF ===")
    r = cmd(ser, "CMD:VOLT_OFF")
    assert "VOLT_OFF,OK" in r
    r = cmd(ser, "CMD:VOLT?")
    assert "vq_ref_mV=0" in r, "FAIL: vq_ref not zeroed"

    print("=== 7. VOLT rejected outside voltage mode ===")
    cmd(ser, "CMD:MODE,0")
    r = cmd(ser, "CMD:VOLT,200")
    assert "VOLT,FAIL" in r, "FAIL: VOLT should be rejected in torque mode"

    print("=== 8. Regression: torque mode still works (MODE,0) ===")
    r = cmd(ser, "CMD:MODE,0")
    assert "MODE,OK,0" in r

    print("=== 9. MODE range check: 4 rejected ===")
    r = cmd(ser, "CMD:MODE,4")
    assert "MODE,FAIL" in r

    print("=== 10. Deadtime comp: query default (off, 150mV) ===")
    r = cmd(ser, "CMD:DT?")
    assert "en=0" in r and "amp_mV=150" in r, "FAIL: DT default state wrong"

    print("=== 11. Deadtime comp: enable + amplitude + disable ===")
    r = cmd(ser, "CMD:DT,1")
    assert "DT,OK,1" in r
    r = cmd(ser, "CMD:DT_V,220")
    assert "DT_V,OK,220mV" in r
    r = cmd(ser, "CMD:DT?")
    assert "en=1" in r and "amp_mV=220" in r
    r = cmd(ser, "CMD:DT,0")
    assert "DT,OK,0" in r
    r = cmd(ser, "CMD:DT?")
    assert "en=0" in r

    print("=== 12. DT_V range check: 2000mV rejected ===")
    r = cmd(ser, "CMD:DT_V,2000")
    assert "DT_V,FAIL" in r

    print("\n*** ALL CHECKS PASSED (no-power command chain, 12/12) ***")
    ser.close()


if __name__ == "__main__":
    main()
