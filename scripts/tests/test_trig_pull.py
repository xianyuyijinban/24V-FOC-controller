#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TRIG 拉取协议单测 (2026-09-09 ③).

对照固件 trig_ring.c 语义合成块流验证:
  - CRC16-CCITT-FALSE 与固件实现一致 (向量: "123456789" → 0x29B1)
  - 帧重组 7 字段解码 + tick 连续性检查
  - 块 CRC 损坏检测
  - 帧序语义: 帧 768 = 触发帧 (pre 768/post 256)
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_crc_vector():
    """CRC-16/CCITT-FALSE 标准检验值: CRC('123456789') = 0x29B1"""
    assert crc16_ccitt(b"123456789") == 0x29B1
    print("PASS: test_crc_vector")


def test_frame_pack_unpack():
    """帧 28B 小端 packed ↔ 解码 (与固件 TrigFrame_t 一致: 6 float + 1 u32)"""
    vals = (0.001, -0.022, 0.05, -1.2, 3.7, 0.37, 123456)
    packed = struct.pack("<ffffffi", *vals)   # 显式 int32 防标准尺寸差异
    assert len(packed) == 28
    out = struct.unpack("<ffffffi", packed)
    assert out[6] == 123456 and abs(out[1] - (-0.022)) < 1e-9
    print("PASS: test_frame_pack_unpack")


def test_reassemble_with_crc():
    """两块重组: 每块尾缀 CRC 验证, 帧序 tick 连续"""
    frames = []
    tick = 100000
    for i in range(256):
        f = (0.001 * i, -0.022, 0.0, 0.5, 0.1 * i, 0.37, tick + i)
        frames.append(struct.pack("<ffffffi", *f))
    # 块 1: 帧 0..127, 块 2: 帧 128..255
    blocks = []
    for off in (0, 128):
        data = b"".join(frames[off:off + 128])
        crc = crc16_ccitt(data)
        blocks.append(data + struct.pack("<H", crc))
    ok = 0
    for payload in blocks:
        data, crc_rx = payload[:-2], struct.unpack("<H", payload[-2:])[0]
        assert crc16_ccitt(data) == crc_rx
        ok += 1
    got = []
    for payload in blocks:
        for i in range(128):
            got.append(struct.unpack_from("<ffffffi", payload, i * 28))
    assert len(got) == 256
    ticks = [f[6] for f in got]
    assert all(b - a == 1 for a, b in zip(ticks, ticks[1:]))
    assert ok == 2
    print("PASS: test_reassemble_with_crc")


def test_crc_corruption_detected():
    """块内 1 字节翻转 → CRC 必须不符"""
    data = b"".join(struct.pack("<ffffffi", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i)
                    for i in range(128))
    crc = crc16_ccitt(data)
    bad = bytearray(data)
    bad[100] ^= 0xFF
    assert crc16_ccitt(bytes(bad)) != crc
    print("PASS: test_crc_corruption_detected")


def test_frame_semantics():
    """pre/post 语义: 1024 帧中帧 768 = 触发帧"""
    # 合成: IDLE 滚动 (tick 1..2048, 环内留最新 1024 = tick 1025..2048)。
    # 触发发生在 tick=2000 帧写入后: trig_idx = (2000-1) & 1023 = 975+... 按模算。
    # 固件 Pull 语义: 帧 i → ring[(trig_idx + i - 768) & 1023];
    # 帧 768 = 触发帧 (tick 2000), 帧 0 = tick 1232, 帧 1023 = tick 2464?
    # 不 — 环内最新只到 2048: 触发后不再写入 (合成场景直接冻结),
    # pre 段 (帧 0..767) = tick 1232..1999 全在环内 (环内最老 1025 < 1232)。
    # post 段 (帧 769..1023) 在真实固件里是触发后写入的; 合成场景用
    # "触发前已滚到 2048" 模拟不出 post — 本测试只验证 pre 段映射 + 触发帧。
    ring = [None] * 1024
    for t in range(1, 2049):
        ring[(t - 1) & 1023] = t            # 环形覆写, 与固件滚动一致
    trig_tick = 2000
    trig_idx = (trig_tick - 1) & 1023
    # pre 段: 帧 i (0..767) → tick 1232+i (环内都有)
    for i in (0, 767):
        ring_i = (trig_idx + i - 768) & 1023
        assert ring[ring_i] == trig_tick - 768 + i, \
            "帧%d tick %d != %d" % (i, ring[ring_i], trig_tick - 768 + i)
    # 触发帧: 帧 768 → tick 2000
    assert ring[trig_idx & 1023] == trig_tick
    # 触发帧前 1 帧 (帧 767) = tick 1999
    assert ring[(trig_idx - 1) & 1023] == trig_tick - 1
    print("PASS: test_frame_semantics")


def main():
    test_crc_vector()
    test_frame_pack_unpack()
    test_reassemble_with_crc()
    test_crc_corruption_detected()
    test_frame_semantics()
    print("ALL_PASS")


if __name__ == "__main__":
    main()
