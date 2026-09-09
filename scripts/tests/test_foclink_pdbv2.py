#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""foclink MixedStreamParser PDBBIN v2 帧 (2026-09-09 ①) 解析单测.

合成 v2 字节流 (49B payload, 含边界值/负值) 验证:
  - v2 帧正确分流 (type 0x21), 三新字段解码正确
  - v1 流回归不破 (type 0x20, 37B)
  - v1/v2 混合流共存不串扰 (seq per-type 独立追踪)
  - CRC 错误 / 长度不匹配帧被丢弃计数
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import foclink


def crc8(data):
    return foclink.MixedStreamParser._crc8(data)


def build_frame(type_byte, payload):
    # CRC 覆盖 = sync(2) + type(1) + len(1) + payload (与 CurStream_BuildFrame 一致)
    body = b"\xA5\x5A" + bytes([type_byte, len(payload)]) + payload
    return body + bytes([crc8(body)])


def pdb1_payload(seq, tick, flags):
    vals = (1.5, -2.0, 0.3, 1.0, 0.05, -0.4, 6.0)   # 7 float
    return struct.pack("<BII", seq, tick, flags) + struct.pack("<fffffff", *vals)


def pdb2_payload(seq, tick, flags):
    vals = (1.5, -2.0, 0.3, 1.0, 0.05, -0.4, 6.0)   # v1 前 7 float 逐比特同 v1
    extra = (-0.022, 0.0093, -0.37)                 # coulomb/cogging/pos_integral (含负值)
    return struct.pack("<BII", seq, tick, flags) + struct.pack("<ffffffffff", *(vals + extra))


def test_v2_parse():
    samples = []
    p = foclink.MixedStreamParser(pdb2_cb=samples.append)
    p.feed(build_frame(foclink.TYPE_PDB2V2, pdb2_payload(0, 1000, 0x00010002)))
    assert len(samples) == 1
    s = samples[0]
    assert s.ver == 2
    assert s.seq == 0 and s.tick_2khz == 1000
    assert s.flags == 0x00010002          # state=1, fault=2, esc bit16=0
    assert abs(s.ff_coulomb - (-0.022)) < 1e-6   # float32 往返精度 ~5e-9@0.37 量级
    assert abs(s.ff_cogging - 0.0093) < 1e-6
    assert abs(s.pos_integral - (-0.37)) < 1e-6
    assert abs(s.pos_err_rad - 1.5) < 1e-6 and abs(s.iq_act - 0.05) < 1e-6
    print("PASS: test_v2_parse")


def test_v2_boundary_values():
    """边界值: u32 回绕 tick / 极大 flags / float 极值"""
    samples = []
    p = foclink.MixedStreamParser(pdb2_cb=samples.append)
    payload = (struct.pack("<BII", 255, 0xFFFFFFFF, 0xFFFFFFFF) +
               struct.pack("<ffffffffff",
                           3.4e38, -3.4e38, 1e-38, -1.0, 0.0,
                           1e30, -6.28, 0.05, 0.0, 1.17549435e-38))
    p.feed(build_frame(foclink.TYPE_PDB2V2, payload))
    s = samples[0]
    assert s.seq == 255 and s.tick_2khz == 0xFFFFFFFF and s.flags == 0xFFFFFFFF
    # float32 亚正常数往返有精度损失, 用相对量级校验 (次正常 ~1e-38 保序)
    assert 0 < s.ff_total < 1e-37 and 0 < s.pos_integral < 1e-37
    # 字段序: pos_err, iq_cmd, ff_total, theta, iq_act, v_mech, pos_ref, coulomb, cogging, integral
    assert abs(s.v_mech_rad_s - 1e30) < 1e23 and abs(s.pos_ref_rad - (-6.28)) < 1e-5
    assert abs(s.ff_coulomb - 0.05) < 1e-6 and s.ff_cogging == 0.0
    print("PASS: test_v2_boundary_values")


def test_v1_regression():
    """v1 流回归: 37B 帧照旧解码, ver=1, 新字段默认 0"""
    samples = []
    p = foclink.MixedStreamParser(pdb2_cb=samples.append)
    p.feed(build_frame(foclink.TYPE_PDB2, pdb1_payload(7, 4242, 0x400)))
    s = samples[0]
    assert s.ver == 1
    assert s.seq == 7 and s.tick_2khz == 4242 and s.flags == 0x400
    assert s.ff_coulomb == 0.0 and s.ff_cogging == 0.0 and s.pos_integral == 0.0
    print("PASS: test_v1_regression")


def test_mixed_v1_v2():
    """混合流: v1/v2 交替, seq per-type 独立追踪, 互不串扰"""
    samples = []
    p = foclink.MixedStreamParser(pdb2_cb=samples.append)
    stream = (build_frame(foclink.TYPE_PDB2, pdb1_payload(0, 100, 0)) +
              build_frame(foclink.TYPE_PDB2V2, pdb2_payload(0, 200, 0)) +
              build_frame(foclink.TYPE_PDB2, pdb1_payload(1, 300, 0)) +
              build_frame(foclink.TYPE_PDB2V2, pdb2_payload(1, 400, 0)))
    p.feed(stream)
    assert len(samples) == 4
    assert [s.ver for s in samples] == [1, 2, 1, 2]
    assert [s.tick_2khz for s in samples] == [100, 200, 300, 400]
    assert p.stats[foclink.TYPE_PDB2].seq_gap == 0
    assert p.stats[foclink.TYPE_PDB2V2].seq_gap == 0
    print("PASS: test_mixed_v1_v2")


def test_crc_and_len_guards():
    """CRC 错 / len 不匹配 → 帧丢弃计数, 不污染后续好帧"""
    samples = []
    p = foclink.MixedStreamParser(pdb2_cb=samples.append)
    good = build_frame(foclink.TYPE_PDB2V2, pdb2_payload(0, 100, 0))
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0xFF
    bad_len = b"\xA5\x5A" + bytes([foclink.TYPE_PDB2V2, 48]) + \
        pdb2_payload(1, 200, 0) + b"\x00"
    p.feed(bytes(bad_crc) + bad_len + good)
    assert p.stats[foclink.TYPE_PDB2V2].crc_err == 1
    assert p.stats[foclink.TYPE_PDB2V2].len_err == 1
    assert len(samples) == 1 and samples[0].tick_2khz == 100
    print("PASS: test_crc_and_len_guards")


def main():
    test_v2_parse()
    test_v2_boundary_values()
    test_v1_regression()
    test_mixed_v1_v2()
    test_crc_and_len_guards()
    print("ALL_PASS")


if __name__ == "__main__":
    main()
