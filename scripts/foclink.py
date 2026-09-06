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
    host_rx_time: float = 0.0    # 解析层打戳: PC 收讫时刻 (epoch s)

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
            s.host_rx_time = __import__("time").time()   # C4: 收讫时刻
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


# ── LOOP_PROF 快照解析 (2026-08-31 主循环黑洞专项) ──────────────────


def parse_loop_prof(lines) -> dict:
    """LOOP_PROF,.. 行列表 -> {'begin': {...}, 'probes': {...}, 'raw': [...]}。

    行格式:
      LOOP_PROF,BEGIN,probe_en=,cpu_hz=,iter_n=,iter_avg_us=,iter_max_cyc=,isr_avg_cyc=,isr_max_cyc=
      LOOP_PROF,<PROBE>,n=,min_cyc=,avg_cyc=,max_cyc=,min_us=,avg_us=,max_us=
      LOOP_PROF,END
    """
    import re
    out = {"begin": {}, "probes": {}, "raw": []}
    for l in lines:
        l = l.strip()
        if not l.startswith("LOOP_PROF,"):
            continue
        out["raw"].append(l)
        parts = l.split(",")
        if len(parts) < 3:
            continue
        kind = parts[1]
        if kind == "BEGIN":
            out["begin"] = dict(re.findall(r"([a-z_0-9]+)=([^,]+)", l))
            for k in ("probe_en", "cpu_hz", "iter_n", "iter_avg_us", "iter_max_cyc",
                      "isr_avg_cyc", "isr_max_cyc"):
                if k in out["begin"]:
                    out["begin"][k] = int(out["begin"][k])
        elif kind in ("END", "BUSY"):
            continue
        else:
            kv = dict(re.findall(r"([a-z_0-9]+)=([^,]+)", l))
            kv["n"] = int(kv.get("n", 0))
            kv["min_cyc"] = int(kv.get("min_cyc", 0))
            kv["avg_cyc"] = int(kv.get("avg_cyc", 0))
            kv["max_cyc"] = int(kv.get("max_cyc", 0))
            kv["min_us"] = float(kv.get("min_us", 0))
            kv["avg_us"] = float(kv.get("avg_us", 0))
            kv["max_us"] = float(kv.get("max_us", 0))
            out["probes"][kind] = kv
    return out


def fetch_loop_prof(ser, timeout: float = 2.0) -> dict:
    """发 CMD:LOOP_PROF? 取快照并解析 (全读, 不按行增量判)。

    前置: 调用方必须已停 PDBBIN 等二进制流 (A5 5A 帧会撞碎文本行, BEGIN/END 丢失)。
    返回 BUSY 视为一次重试机会 (调用方可重试)。
    """
    ser.reset_input_buffer()
    ser.write(b"CMD:LOOP_PROF?\n")
    buf = b""
    dl = __import__("time").time() + timeout
    while __import__("time").time() < dl:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if b"LOOP_PROF,END" in buf or b"LOOP_PROF,BUSY" in buf:
                break
        else:
            __import__("time").sleep(0.01)
    lines = buf.replace(b"\r", b"").split(b"\n")
    return parse_loop_prof([l.decode(errors="replace") for l in lines])


# ─────────────────────────────────────────────────────────────────────────────
# 测量窗 helper (2026-09-04 Kimi 双态案定案: 积压帧混入测量窗伪影家族第五条)
#
# 历史坑族: drain / 环绕 / 字段索引 / settle 时序 / 【积压帧混入窗】
# 病根: 回位/保持 sleep 期间无人排空队列, 测量窗第一批 pop 出来的是几秒前的旧帧,
#       pp 报出"伪漂移" (幅度 ≈ 序列行程本身)。
# 用法:
#   win = MeasureWindow(nframe_q)          # N帧队列 (host_rx, p1, ...) 元组
#   win.begin()                            # 窗前排空+记录积压数
#   frame = win.pop()                      # 只返回窗内帧 (按 host_rx >= win_start)
#   win.finish()                           # 返回 (frames, backlog_n)
#   win.wait_stable(...)                   # 稳定门 (连续 gate_window 滑动窗 pp < gate_pp)
#
# 帧格式约定: 队列成员至少含 (host_rx: float, ...) — host_rx 为入队时刻 (time.time())。
# ─────────────────────────────────────────────────────────────────────────────

class MeasureWindow:
    """测量窗: 排空积压 + 按 host_rx 时间戳过滤只收窗内帧 + 稳定门。"""

    def __init__(self, nframe_q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0,
                 gate_span_eps=0.1):
        self.q = nframe_q
        self.win_seconds = win_seconds
        self.gate_pp = gate_pp
        self.gate_window = gate_window
        # span 判定的帧间隔余量: 离散采样下严格 >= gate_window 数学永假
        # (2026-09-06 A2 gate 全超时根因; 0.1s 覆盖 22Hz 帧 45ms + 抖动)
        self._gate_span_eps = gate_span_eps
        self.win_start = 0.0
        self.backlog_n = 0
        self.last_gate_span_s = None
        self.last_gate_frames = 0
        self.last_gate_pp = None
        self.last_gate_first_rx = None
        self.last_gate_last_rx = None

    def begin(self):
        """窗开始: 排空积压帧并计数 (旧帧不入窗)。返回积压数。"""
        self.backlog_n = len(self.q)
        self.q.clear()
        self.win_start = __import__("time").time()
        return self.backlog_n

    def pop(self):
        """取一帧, 只返回 host_rx 在窗开始后的 (时间戳过滤)。无则 None。
        兼容 list.pop(0) 与 deque.popleft (2026-09-05 单测暴露)。"""
        while self.q:
            f = self.q.popleft() if hasattr(self.q, "popleft") else self.q.pop(0)
            if f[0] >= self.win_start:
                return f
        return None

    def drain_in_window(self):
        """消费窗内全部帧, 返回列表 (按时间序)。"""
        out = []
        while True:
            f = self.pop()
            if f is None:
                break
            out.append(f)
        return out

    def collect(self, duration=None, health_cb=None):
        """收集 duration 秒 (默认 self.win_seconds) 的窗内帧。返回 (frames, elapsed)。

        health_cb 在等待期间周期调用；回调抛出的异常直接传给调用方，
        使串流健康守卫不会被阻塞的测量窗绕过。
        """
        t0 = __import__("time").time()
        d = duration if duration is not None else self.win_seconds
        frames = []
        while __import__("time").time() - t0 < d:
            if health_cb:
                health_cb()
            frames.extend(self.drain_in_window())
            __import__("time").sleep(0.002)
        return frames, __import__("time").time() - t0

    def wait_stable(self, target_deg, timeout=15.0, angle_index=2, health_cb=None):
        """稳定门: 滑动样本 (最近 gate_window 内帧, 按 host_rx 剪) 的
        last_rx - first_rx >= gate_window 且窗内 pp < gate_pp (相对 target_deg)。
        返回 (ok, waited_s)。frames 元组中角度在 angle_index 位置 (默认 2 = (hrx, p1, ang,...))
        2026-09-06 恢复规划原文修正 (Kimi 裁决): 实现回到"滑动样本"语义 —
        先前"调用后累积全帧"把回位大摆帧永远留在窗里 → pp 永不收敛 → 4 rep 全 10s
        超时 (稳态真值 pp 0.02-0.09°)。滑动窗下大摆滑出即可过; 5 帧 span 0.1s
        假稳态仍被 span>=gate_window 拦下 (假稳态帧 span 不足)。保留门前排空
        (积压旧帧秒过 bug)。"""
        self.last_gate_span_s = None
        self.last_gate_frames = 0
        self.last_gate_pp = None
        self.last_gate_first_rx = None
        self.last_gate_last_rx = None
        self.q.clear()                              # 门前排空: 只判调用后的新帧
        self.win_start = __import__("time").time()  # 2026-09-05 积压秒过 bug
        t0 = __import__("time").time()
        recent = []
        while __import__("time").time() - t0 < timeout:
            if health_cb:
                health_cb()
            f = self.pop()
            if f is not None:
                ang = f[angle_index]
                d = ang - target_deg
                while d > 180.0: d -= 360.0
                while d < -180.0: d += 360.0
                recent.append((f[0], d))   # f[0]=host_rx (帧入队时刻)
                # 滑动窗剪: 只保留最近 gate_window 内的帧 (回位大摆滑出即可过门)
                # 剪窗阈给一个帧间隔余量: 离散采样下 span 数学上限 = gate_window
                #   - 帧间隔, 严格 >= gate_window 永假 (生产 22Hz 帧间隔 45ms;
                #   T5 合成帧同时入队掩盖了该缺陷 — 第二次"夹具匹配实现"教训)
                cutoff = recent[-1][0] - self.gate_window - self._gate_span_eps
                recent = [x for x in recent if x[0] >= cutoff]
                self.last_gate_frames = len(recent)
                self.last_gate_first_rx = recent[0][0]
                self.last_gate_last_rx = recent[-1][0]
                self.last_gate_span_s = (recent[-1][0] - recent[0][0])
                if len(recent) >= 5:
                    span = self.last_gate_span_s
                    if span >= self.gate_window - self._gate_span_eps:
                        # 必须覆盖完整窗 (2026-09-05 恢复规划; eps=帧间隔余量)
                        pp = max(x[1] for x in recent) - min(x[1] for x in recent)
                        self.last_gate_pp = pp
                        if pp < self.gate_pp:
                            return True, __import__("time").time() - t0
            else:
                __import__("time").sleep(0.01)
        return False, __import__("time").time() - t0
