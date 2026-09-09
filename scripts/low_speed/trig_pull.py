#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""trig_pull.py — TRIG 验尸 ring buffer 拉取工具 (2026-09-09 ③).

用法: python trig_pull.py [port] [--out out.json]
流程:
  1. CMD:TRIG,STAT? 确认 FROZEN (state=2), 读 trig_tick/post
  2. 分块 CMD:TRIG,PULL,off,len (≤128 帧/块) 全量拉取 1024 帧
     每块响应 = TRIG,BIN,<n>,<binary n×28B+CRC16(2)> — 头走行解析,
     二进制尾缀按 n 精确读; CRC16-CCITT-FALSE 逐块验
  3. 重组 1024×28B → 7 字段解码 (id/iq/vd/vq/theta_elec/iq_ref/tick_20k)
  4. 存 JSON (帧列 + meta: trig_tick/state/src/块CRC统计)

帧序: 帧 0 = 触发前 768 帧 (pre 段头), 帧 768 = 触发帧, 帧 1023 = post 末帧。
"""
import argparse
import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import serial

TRIG_RING_SIZE = 1024
TRIG_FRAME_SIZE = 28
TRIG_PRE = 768
BLOCK_FRAMES = 128          # 128×28=3584B 数据+CRC < RX 缓冲安全


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class TrigPuller:
    def __init__(self, port, baud=1000000):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.frames = []
        self.blocks_ok = 0
        self.blocks_crc_fail = []
        self.trig_stat = {}

    def _query_line(self, cmd, prefix, timeout=2.0):
        """单发文本查询 (拉取前须 PDBBIN 已停 — 调用方保证)"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        buf = b""
        while time.time() < dl:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                if b"\n" in buf and prefix.encode() in buf:
                    for l in buf.decode(errors="replace").split("\n"):
                        if l.strip().startswith(prefix):
                            return l.strip()
            time.sleep(0.005)
        return None

    def stat(self, tries=3):
        for _ in range(tries):
            l = self._query_line("CMD:TRIG,STAT?", "TRIG,OK,")
            if l:
                fields = {}
                for f in l.split(","):
                    if "=" in f:
                        k, v = f.split("=", 1)
                        fields[k] = v
                self.trig_stat = fields
                return fields
            time.sleep(0.3)
        return None

    def pull_block(self, off, nframes, timeout=3.0):
        """拉一块: 头行 TRIG,BIN,<n>, 后跟 n 字节二进制。返回 payload 或 None。"""
        expect_bytes = nframes * TRIG_FRAME_SIZE + 2   # 数据 + CRC16
        self.ser.reset_input_buffer()
        self.ser.write(("CMD:TRIG,PULL,%d,%d\n" % (off, nframes)).encode())
        dl = time.time() + timeout
        buf = b""
        while time.time() < dl:
            nn = self.ser.in_waiting
            if nn:
                buf += self.ser.read(nn)
            # 头完整 + 期望字节数齐 → 结束
            idx = buf.find(b"TRIG,BIN,")
            if idx >= 0:
                hdr_end = buf.find(b",", idx + 9)
                if hdr_end >= 0:
                    try:
                        n = int(buf[idx + 9:hdr_end])
                    except ValueError:
                        return None
                    need = hdr_end + 1 + n
                    if len(buf) >= need:
                        payload = buf[hdr_end + 1:need]
                        # 期望字节数与头声明一致才认
                        if n != expect_bytes:
                            return None
                        return payload
            time.sleep(0.002)
        return None

    def pull_all(self):
        for off in range(0, TRIG_RING_SIZE, BLOCK_FRAMES):
            payload = None
            for retry in range(2):
                payload = self.pull_block(off, BLOCK_FRAMES)
                if payload is not None:
                    break
                time.sleep(0.2)
            if payload is None:
                raise RuntimeError("块 off=%d 拉取失败 (2 次重试)" % off)
            data, crc_rx = payload[:-2], struct.unpack("<H", payload[-2:])[0]
            crc_calc = crc16_ccitt(data)
            if crc_calc != crc_rx:
                self.blocks_crc_fail.append(off)
                raise RuntimeError("块 off=%d CRC 不符: rx=0x%04X calc=0x%04X"
                                   % (off, crc_rx, crc_calc))
            self.blocks_ok += 1
            for i in range(BLOCK_FRAMES):
                self.frames.append(struct.unpack_from("<ffffffi", data,
                                                      i * TRIG_FRAME_SIZE))
        assert len(self.frames) == TRIG_RING_SIZE

    def close(self):
        self.ser.close()

    def frame_dict(self, i):
        f = self.frames[i]
        return {"id": round(f[0], 6), "iq": round(f[1], 6),
                "vd": round(f[2], 6), "vq": round(f[3], 6),
                "theta_elec": round(f[4], 6), "iq_ref": round(f[5], 6),
                "tick_20k": f[6]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    p = TrigPuller(args.port)
    st = p.stat()
    if st is None:
        print("STAT? 无响应")
        return 1
    print("STAT:", st)
    if st.get("state") != "2":
        print("非 FROZEN (state=%s) — 先触发或等待 post 计满" % st.get("state"))
        p.close()
        return 1

    t0 = time.time()
    try:
        p.pull_all()
    except RuntimeError as e:
        print("FAIL:", e)
        p.close()
        return 1
    dt = time.time() - t0
    print("拉取完成: %d 块全过 CRC, %.2fs" % (p.blocks_ok, dt))

    # tick 连续性: 20kHz 帧序 tick_20k 递增 1 (帧 0..1023)
    ticks = [f[6] for f in p.frames]
    tick_discont = sum(1 for a, b in zip(ticks, ticks[1:])
                       if ((b - a) & 0xFFFFFFFF) != 1)
    print("tick 断点: %d / %d (应全为 1)" % (tick_discont, len(ticks) - 1))

    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "trig_dump_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    doc = {
        "test": "trig_pull_full",
        "meta": {
            "trig_stat": p.trig_stat,
            "blocks_ok": p.blocks_ok,
            "blocks_crc_fail": p.blocks_crc_fail,
            "pull_seconds": round(dt, 2),
            "tick_discontinuity": tick_discont,
            "frame_size": TRIG_FRAME_SIZE,
            "pre_frames": TRIG_PRE,
            "trig_frame_index": TRIG_PRE,
        },
        "frames": [p.frame_dict(i) for i in range(TRIG_RING_SIZE)],
    }
    with open(out, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False)
    print("报告:", out)
    p.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
