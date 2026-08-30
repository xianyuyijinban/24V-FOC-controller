#!/usr/bin/env python3
"""foclink.py — 混合流(ASCII + A5 5A 二进制)链路解析层

2026-08-30 任务卡 T3/H6: 移植 HostComputer/data_parser.py 的成熟状态机逻辑
(BinaryCurrentParser: sync->type->len->payload->CRC 校验->分发), 不重造轮子。

通道:
  - 'C'  (0x43): 电流流 CurrentSample (payload 20B)
  - 'W'  (0x57): 轮事件 WheelEvent   (payload 16B)
  - 'D'  (0x20 DBG_TYPE_PDB2): PDBBIN 调试流 (payload 37B) — 本任务新增

per-channel 计数: rx_count / crc_err / seq_gap (seq 按 mod 256 判 gap)。

查询助手按 H8 纪律: reset_input_buffer -> 发 -> sleep -> 全读 -> 去 \\r 按 \\n split。
"""
import struct
from dataclasses import dataclass

SYNC = b"\xA5\x5A"

TYPE_CURRENT = 0x43   # 'C'
TYPE_WHEEL   = 0x57   # 'W'
TYPE_PDB2    = 0x20   # DBG_TYPE_PDB2 — debug_stream.h


@dataclass
class PdbBinSample:
    """PDBBIN 37B payload (小端 packed)."""
    seq: int = 0
    tick_2khz: int = 0
    flags: int = 0
    pos_err_rad: float = 0.0
    iq_cmd: float = 0.0
    ff_total: float = 0.0
    theta_user_rad: float = 0.0
    iq_act: float = 0.0
    v_mech_rad_s: float = 0.0
    pos_ref_rad: float = 0.0

    @property
    def t(self) -> float:
        return self.tick_2khz / 2000.0

    @property
    def theta_user_deg(self) -> float:
        return self.theta_user_rad * 57.29577951308232

    @property
    def pos_err_deg(self) -> float:
        return self.pos_err_rad * 57.29577951308232


@dataclass
class ChannelStats:
    rx: int = 0
    crc_err: int = 0
    seq_gap: int = 0
    len_err: int = 0


class MixedStreamParser:
    """混合流解析: 文本行 + A5 5A 二进制帧。

    移植 data_parser.BinaryCurrentParser 的成熟状态机:
      - 找 sync -> 读 type/len -> 校验 payload_len 匹配 -> 攒帧 -> CRC-8 -> 分发
      - 未知 type / CRC 错: 跳 sync 后重建同步 (与 data_parser 相同策略)
      - 残留文本字节聚合为行, 由调用方(line_cb)处理
    """

    _CRC8_TABLE = [
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
        0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
        0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
        0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
        0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
        0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
        0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
        0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
        0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
        0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
        0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
        0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
        0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
        0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
        0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
        0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
    ]

    PAYLOAD_LEN = {
        TYPE_CURRENT: 20,
        TYPE_WHEEL:   16,
        TYPE_PDB2:    37,
    }

    def __init__(self, line_cb=None, pdb2_cb=None, current_cb=None, wheel_cb=None):
        self._buf = bytearray()
        self._linebuf = b""
        self.stats = {t: ChannelStats() for t in self.PAYLOAD_LEN}
        self.unknown_types = 0
        self._last_seq = {t: -1 for t in self.PAYLOAD_LEN}
        self.line_cb = line_cb          # fn(bytes_line)
        self.pdb2_cb = pdb2_cb          # fn(PdbBinSample)
        self.current_cb = current_cb    # fn(CurrentSample-like)
        self.wheel_cb = wheel_cb        # fn(WheelEvent-like)

    # ── 对外: 喂原始字节 ──
    def feed(self, data: bytes) -> None:
        self._buf.extend(data)
        self._drain()

    # ── 状态机主体 (移植 data_parser.feed_all 骨架) ──
    def _drain(self) -> None:
        while len(self._buf) >= 6:
            sync_idx = self._buf.find(SYNC)
            if sync_idx == -1:
                self._emit_text(self._buf)
                self._buf.clear()
                break
            if sync_idx > 0:
                self._emit_text(self._buf[:sync_idx])
                del self._buf[:sync_idx]
            if len(self._buf) < 4:
                break

            payload_len = self._buf[3]
            type_byte = self._buf[2]

            if type_byte not in self.PAYLOAD_LEN:
                self.unknown_types += 1
                self._emit_text(self._buf[:1])
                del self._buf[:1]
                continue

            if payload_len != self.PAYLOAD_LEN[type_byte]:
                self.stats[type_byte].len_err += 1
                self._emit_text(self._buf[:1])
                del self._buf[:1]
                continue

            total = 4 + payload_len + 1
            if len(self._buf) < total:
                break  # 等完整帧

            candidate = bytes(self._buf[:total])
            crc = self._crc8(candidate[:-1])
            if crc != candidate[-1]:
                self.stats[type_byte].crc_err += 1
                self._emit_text(self._buf[:1])
                del self._buf[:1]
                continue

            self._dispatch(type_byte, candidate[4:4 + payload_len])
            self.stats[type_byte].rx += 1
            del self._buf[:total]

        # 残留字节: 若以 sync 开头则保留, 否则作文本冲洗
        if self._buf:
            if self._buf[-1] == SYNC[0] and len(self._buf) >= 1:
                # 可能是截断的 sync 前缀, 保留
                pass
            elif SYNC in self._buf:
                pass  # 等 _drain 下一轮处理
            else:
                self._emit_text(self._buf)
                self._buf.clear()

        # 文本行冲洗 (行以 \n 结尾才上行回调; 半行保留)
        self._flush_lines()

    def _emit_text(self, chunk: bytes) -> None:
        if not chunk:
            return
        self._linebuf += chunk
        self._flush_lines()

    def _flush_lines(self) -> None:
        while b"\n" in self._linebuf:
            line, self._linebuf = self._linebuf.split(b"\n", 1)
            line = line.rstrip(b"\r")
            if line and self.line_cb:
                self.line_cb(line.decode(errors="replace"))

    def _dispatch(self, type_byte: int, payload: bytes) -> None:
        if type_byte == TYPE_PDB2:
            s = self._decode_pdb2(payload)
            self._track_seq(type_byte, s.seq)
            if self.pdb2_cb:
                self.pdb2_cb(s)
        elif type_byte == TYPE_CURRENT:
            s = self._decode_c(payload)
            self._track_seq(type_byte, s[0])
            if self.current_cb:
                self.current_cb(s)
        elif type_byte == TYPE_WHEEL:
            s = self._decode_w(payload)
            self._track_seq(type_byte, s[0])
            if self.wheel_cb:
                self.wheel_cb(s)

    # ── 解码 ──
    @staticmethod
    def _decode_pdb2(p: bytes) -> PdbBinSample:
        seq, tick, flags, e, iq, ff, th, iqa, v, pr = struct.unpack("<BIIfffffff", p)
        return PdbBinSample(seq=seq, tick_2khz=tick, flags=flags, pos_err_rad=e,
                            iq_cmd=iq, ff_total=ff, theta_user_rad=th, iq_act=iqa,
                            v_mech_rad_s=v, pos_ref_rad=pr)

    @staticmethod
    def _decode_c(p: bytes):
        return struct.unpack("<HIhhhhhHH", p)   # 与原 data_parser 同序

    @staticmethod
    def _decode_w(p: bytes):
        return struct.unpack("<HIhihH", p)      # 与原 data_parser 同序

    def _track_seq(self, type_byte: int, seq: int) -> None:
        last = self._last_seq[type_byte]
        if last >= 0:
            expected = (last + 1) & 0xFF if type_byte == TYPE_PDB2 else (last + 1) & 0xFFFF
            if seq != expected:
                self.stats[type_byte].seq_gap += 1
        self._last_seq[type_byte] = seq

    @classmethod
    def _crc8(cls, data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc = cls._CRC8_TABLE[crc ^ b]
        return crc


# ── H8 查询助手 ────────────────────────────────────────────────
def query(ser, cmd: bytes, timeout: float = 1.5) -> bytes:
    """reset_input_buffer -> 发 -> 睡 -> 全读 (H8 纪律: 不增量 startswith 判响应)。"""
    ser.reset_input_buffer()
    ser.write(cmd)
    buf = b""
    dl = __import__("time").time() + timeout
    while __import__("time").time() < dl:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        else:
            __import__("time").sleep(0.01)
    return buf.replace(b"\r", b"")


def query_lines(ser, cmd: bytes, timeout: float = 1.5):
    """query() + 按 \\n split 的文本行列表。"""
    return [l for l in query(ser, cmd, timeout).split(b"\n") if l]
