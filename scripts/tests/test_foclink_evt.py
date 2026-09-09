#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""foclink MixedStreamParser EVT 事件帧 (2026-09-09 ②) 解析单测.

合成事件帧验证:
  - 5 类事件解码正确 (STATE/FAULT/ESC/AW_MODE/TX_P1_DROP) 含溢出槽
  - EVT 与 PDBBIN v1/v2 共存不串扰 (PDB seq 连续性不被事件破坏)
  - CRC 错 / len 不匹配防护
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import foclink


def crc8(data):
    return foclink.MixedStreamParser._crc8(data)


def build_frame(type_byte, payload):
    body = b"\xA5\x5A" + bytes([type_byte, len(payload)]) + payload
    return body + bytes([crc8(body)])


def evt_payload(tick, code, p8):
    assert len(p8) == 8
    return struct.pack("<IB", tick, code) + p8


def test_evt_decode_all_codes():
    evts = []
    p = foclink.MixedStreamParser(evt_cb=evts.append)
    # 0x01 STATE: old=3(READY) new=4(RUNNING)
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(1000, 0x01, bytes([3, 4, 0, 0, 0, 0, 0, 0]))))
    # 0x02 FAULT: mask=5 set
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(2000, 0x02, struct.pack("<IB", 5, 1) + b"\0" * 3)))
    # 0x03 ESC trigger: err=-0.104 rad
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(3000, 0x03, struct.pack("<fB", -0.104, 1) + b"\0" * 3)))
    # 0x04 AW_MODE: 1→2
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(4000, 0x04, bytes([1, 2, 0, 0, 0, 0, 0, 0]))))
    # 0x05 TX_P1_DROP: 3 帧丢
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(5000, 0x05, struct.pack("<I", 3) + b"\0" * 4)))
    assert len(evts) == 5
    e0 = evts[0]
    assert e0.code == 0x01 and e0.name == "STATE"
    assert e0.decoded() == {"old_state": 3, "new_state": 4}
    assert e0.t == 0.5
    d1 = evts[1].decoded()
    assert d1 == {"fault_mask": 5, "set": True}
    d2 = evts[2].decoded()
    assert abs(d2["pos_err_rad"] - (-0.104)) < 1e-6 and d2["trigger"] is True
    assert evts[3].decoded() == {"old_mode": 1, "new_mode": 2}
    assert evts[4].decoded() == {"tx_p1_drop": 3}
    assert all(e.overflow == 0 for e in evts)
    print("PASS: test_evt_decode_all_codes")


def test_evt_overflow_slot():
    """溢出槽 = 事件 payload[7] (帧 payload[12])"""
    evts = []
    p = foclink.MixedStreamParser(evt_cb=evts.append)
    p.feed(build_frame(foclink.TYPE_EVT,
                       evt_payload(1000, 0x01, bytes([4, 0, 0, 0, 0, 0, 0, 7]))))
    assert evts[0].overflow == 7
    print("PASS: test_evt_overflow_slot")


def test_evt_pdb_coexist():
    """EVT 插入 PDBBIN 流: PDB seq 连续性不受事件影响, CRC 全过"""
    pdbs, evts = [], []
    p = foclink.MixedStreamParser(pdb2_cb=pdbs.append, evt_cb=evts.append)

    def pdb1(seq, tick):
        vals = (1.5, -2.0, 0.3, 1.0, 0.05, -0.4, 6.0)
        return (struct.pack("<BII", seq, tick, 0) +
                struct.pack("<fffffff", *vals))

    stream = (build_frame(foclink.TYPE_PDB2, pdb1(0, 100)) +
              build_frame(foclink.TYPE_EVT,
                          evt_payload(105, 0x01, bytes([3, 4] + [0] * 6))) +
              build_frame(foclink.TYPE_PDB2, pdb1(1, 110)) +
              build_frame(foclink.TYPE_PDB2, pdb1(2, 120)) +
              build_frame(foclink.TYPE_EVT,
                          evt_payload(126, 0x03,
                                      struct.pack("<fB", -0.09, 1) + b"\0" * 3)) +
              build_frame(foclink.TYPE_PDB2, pdb1(3, 130)))
    p.feed(stream)
    assert len(pdbs) == 4 and len(evts) == 2
    assert [s.seq for s in pdbs] == [0, 1, 2, 3]   # PDB seq 连续, 不被事件打断
    assert p.stats[foclink.TYPE_PDB2].seq_gap == 0
    assert p.stats[foclink.TYPE_PDB2].crc_err == 0
    assert p.stats[foclink.TYPE_EVT].crc_err == 0
    # tick 可对齐: 事件 tick 夹在相邻 PDB tick 之间 (同 2kHz 基准)
    assert pdbs[0].tick_2khz < evts[0].tick_2khz < pdbs[1].tick_2khz
    print("PASS: test_evt_pdb_coexist")


def test_evt_guards():
    """CRC 错 / len 错帧丢弃计数, 不污染后续好帧"""
    evts = []
    p = foclink.MixedStreamParser(evt_cb=evts.append)
    good = build_frame(foclink.TYPE_EVT,
                       evt_payload(1000, 0x01, bytes([3, 4] + [0] * 6)))
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0xFF
    bad_len = b"\xA5\x5A" + bytes([foclink.TYPE_EVT, 15]) + \
        evt_payload(2000, 0x01, bytes([0] * 8)) + b"\x00"
    p.feed(bytes(bad_crc) + bad_len + good)
    assert p.stats[foclink.TYPE_EVT].crc_err == 1
    assert p.stats[foclink.TYPE_EVT].len_err == 1
    assert len(evts) == 1 and evts[0].tick_2khz == 1000
    print("PASS: test_evt_guards")


def main():
    test_evt_decode_all_codes()
    test_evt_overflow_slot()
    test_evt_pdb_coexist()
    test_evt_guards()
    print("ALL_PASS")


if __name__ == "__main__":
    main()
