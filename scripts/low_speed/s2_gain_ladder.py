#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s2_gain_ladder.py — S2 电压模式位置环增益阶梯 (2026-09-04 Kimi 规格)

增益阶梯 (电压口径 V/rad; POS_DIRECT_GAIN 发电流口径 A/rad, 固件末级 ×Rs_phase 4.4):
  G1: 1.016 → 发 0.231/0.0035 + KI 0.185
  G2: 2.033 → 发 0.462/0.0070 + KI 0.370
  G3: 3.050 → 发 0.693/0.0105 + KI 0.555

每档判读:
  阶跃 +6°: t95 (离开初始窗 → 进 5%×step 窗并保持 0.5s), 稳态 pp (测量窗修复版)
  0.5°/s 斜坡: 跟踪率 = 实测位移/6° (>90% 目标)
  极限环: PDBBIN pos_err 后段 1s 峰峰 (首档 5.15° → 2.57° → 1.72° 预测)

测量窗: foclink.MeasureWindow (窗前排空积压计数 + host_rx 时间戳过滤)
  — 观测家族第五条 (2026-09-04 Kimi 终审), 垃圾数据不入判读

用法: python scripts/low_speed/s2_gain_ladder.py COM10 --power-ok [--rounds 3]
"""
import argparse
import json
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0

# flags 编码 (stm32h7xx_it.c 打包): 高 8 位=state (bits 15:8, 原值 0-5),
# 低 8 位=fault_code。枚举定义在 MDK-ARM/code/foc_app.h:169-175:
# IDLE=0/INIT=1/PARAM_IDENTIFY=2/READY=3/RUNNING=4/FAULT=5
FOC_STATE_RUNNING = 4
T95_STEP_DEG = 6.0
T95_LEAVE_ERR_DEG = 0.1
T95_WINDOW_PCT = 0.05
T95_DWELL_S = 0.5
PDB_MIN_RATE_HZ = 50.0

# 电流口径 POS_DIRECT_GAIN (A/rad); 电压口径 = kp × Rs_phase(Rs/2≈4.149):
#   G1 = 半值 0.245 → 1.016V/rad, G2 = 全值 0.490 → 2.033V/rad, G3 = 1.5× 0.735 → 3.05V/rad
LADDER = [
    ("G1", 0.245, 0.0035, 0.185),
    ("G2", 0.490, 0.0070, 0.370),
    ("G3", 0.735, 0.0105, 0.555),
]


def parse_status_fields(line):
    """Parse comma-separated k=v fields after a protocol status prefix."""
    fields = {}
    for field in line.split(","):
        if "=" in field:
            key, value = field.split("=", 1)
            fields[key] = value
    return fields


def parse_dt_enabled(line):
    """Return the acknowledged DT enable state, or None for an invalid ACK."""
    if not line or not line.startswith("DT,OK"):
        return None
    fields = parse_status_fields(line)
    if "en" in fields:
        value = fields["en"]
    else:
        parts = line.split(",")
        value = parts[2].strip() if len(parts) > 2 else None
    if value not in ("0", "1"):
        return None
    return value == "1"


def advance_t95(state, frame_host_rx, pos_err_rad, command_host_rx,
                step_rad=T95_STEP_DEG * DEG2RAD):
    """Advance t95 state from one new PDB frame.

    All elapsed times come from frame host timestamps. Re-processing the same
    frame cannot advance the dwell timer.
    """
    if frame_host_rx < command_host_rx or state["t95_rx"] is not None:
        return

    error_deg = abs(pos_err_rad) / DEG2RAD
    if state["leave_rx"] is None:
        if error_deg >= T95_LEAVE_ERR_DEG:
            state["leave_rx"] = frame_host_rx
        return

    if state["settle_0p1_rx"] is None and error_deg < T95_LEAVE_ERR_DEG:
        state["settle_0p1_rx"] = frame_host_rx

    window_rad = T95_WINDOW_PCT * abs(step_rad)
    if abs(pos_err_rad) > window_rad:
        state["window_enter_rx"] = None
        return
    if state["window_enter_rx"] is None:
        state["window_enter_rx"] = frame_host_rx
        return
    if frame_host_rx - state["window_enter_rx"] >= T95_DWELL_S:
        state["t95_rx"] = frame_host_rx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=3, help="每档重复轮数")
    ap.add_argument("--gains", default="G1,G2,G3",
                    help="档位过滤 (逗号分隔, 如 G1,G2; 默认全跑)")
    ap.add_argument("--ki-retry", type=int, default=3,
                    help="GAIN/KI 命令重试次数 (reader 瞬断/挤占恢复, 每次重发)")
    ap.add_argument("--esc", action="store_true",
                    help="开启僵持积分逃逸 CMD:POS_AW_ESC,1 (默认关; ESC 轮断言 "
                         "esc_count>0, 无触发=数据无法归因 → invalid)")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: --power-ok")
        return 0

    gains_sel = [g.strip() for g in args.gains.split(",") if g.strip()]
    ladder = [g for g in LADDER if g[0] in gains_sel]
    if not ladder:
        print("无有效档位 (--gains=%s)" % args.gains)
        return 1
    print("档位: %s" % ", ".join(g[0] for g in ladder), flush=True)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.5)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n", b"CMD:VOLT_OFF\n"):
        ser.write(c)
        time.sleep(0.2)
    ser.reset_input_buffer()

    class LineBuffer:
        def __init__(self):
            self.buf = ""

        def _drain(self):
            n = ser.in_waiting
            if not n:
                return []
            self.buf += ser.read(n).decode(errors="replace")
            if "\n" not in self.buf:
                return []
            lines = self.buf.split("\n")
            self.buf = lines.pop()
            return lines

    lb = LineBuffer()

    def expect(cmd, prefix, timeout=3.0):
        ser.reset_input_buffer()
        lb.buf = ""
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            for l in lb._drain():
                if l.strip().startswith(prefix):
                    return l.strip()
            time.sleep(0.01)
        return None

    def send(cmd, wait=0.3):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)
    reader_ready = [False]

    def _query(cmd, timeout=3.0):
        """reader 启动前直读: 发命令并收集响应行 (含可能的多行)。"""
        ser.reset_input_buffer()
        lb.buf = ""
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        lines = []
        while time.time() < dl:
            for l in lb._drain():
                lines.append(l.strip())
            time.sleep(0.01)
        return lines

    # ── fail-closed 预检: FW_INFO/JDIAG/CH_CFG 解析校验 ──
    # 2026-09-05 恢复规划: 必需项缺失即中止, 不落有效结果。
    # 2026-09-06 响应丢失家族 (F1 仲裁桌面放大): 单发 _query 实测丢率 ~5-10%
    #   (预检 JDIAG 丢 1 次 / init FRIC 稳定复现 DT? 后丢 1 次) — 预检查询带重试
    def _query_retry(cmd, prefix, timeout=2.0, tries=3):
        """带重试的查询: 每轮 reset 后发, 直到收到 prefix 开头行或重试耗尽。"""
        for i in range(tries):
            lines = _query(cmd, timeout)
            hit = next((l for l in lines if l.startswith(prefix)), None)
            if hit is not None:
                return hit
            time.sleep(0.2)
        return None

    def _precheck():
        fw = _query_retry("CMD:FW_INFO?", "FW_INFO,OK,")
        if fw is None:
            print("PRECHECK FAIL: FW_INFO 无有效响应")
            return None
        fw_ver = parse_status_fields(fw)
        for required in ("version", "param", "baseline"):
            if not fw_ver.get(required):
                print("PRECHECK FAIL: FW_INFO 缺 %s" % required)
                return None

        jd = _query_retry("CMD:JDIAG", "JDIAG,")
        if jd is None:
            print("PRECHECK FAIL: JDIAG 无响应")
            return None
        # JDIAG,v6,J=..,B=..,Tc=..,enc=..,valid=0x..,cog_valid=..,cog_size=..,cog_bins=..,
        #   cog_save=..,cog_gain=..,cog_phase=..,cog_min=..,cog_max=..
        jd_map = parse_status_fields(jd)
        for req in ("J", "enc", "valid", "cog_valid", "cog_save",
                    "cog_gain", "cog_phase", "cog_min", "cog_max"):
            if not jd_map.get(req):
                print("PRECHECK FAIL: JDIAG 缺 %s" % req)
                return None

        cc = _query_retry("CMD:CH_CFG?", "CH_CFG,OK,")
        if cc is None:
            print("PRECHECK FAIL: CH_CFG 无有效响应")
            return None
        cc_parsed = parse_status_fields(cc)
        for req in ("gain_c", "recon"):
            if not cc_parsed.get(req):
                print("PRECHECK FAIL: CH_CFG 缺 %s" % req)
                return None

        return {
            "fw_info": fw_ver, "jdiag": jd_map, "ch_cfg": cc_parsed,
            "jdiag_raw": jd, "fw_raw": fw, "ch_raw": cc,
        }

    pre = _precheck()
    if pre is None:
        ser.close()
        return 1
    print("PRECHECK:", pre["fw_info"].get("version"), "enc=%s" % pre["jdiag"].get("enc"),
          "cog_gain=%s" % pre["jdiag"].get("cog_gain"), flush=True)
    ser.reset_input_buffer()

    dt_cmd_sent = False
    # DT,0 + DT? 双确认带重试 (响应丢失家族: DT? 后首条 expect 丢率最高, 手测复现)
    dt_command_lines = _query("CMD:DT,0", 2.0)
    dt_cmd_sent = True
    dt_ack = next((l for l in dt_command_lines if l.startswith("DT,OK")), None)
    dt_enabled = parse_dt_enabled(dt_ack)
    dt_status = None
    dt_status_enabled = None
    for _ in range(3):
        dt_status_lines = _query("CMD:DT?", 2.0)
        dt_status = next((l for l in dt_status_lines if l.startswith("DT,OK")), None)
        dt_status_enabled = parse_dt_enabled(dt_status)
        if dt_status_enabled is not False:
            time.sleep(0.2)
    if dt_enabled is not False or dt_status_enabled is not False:
        print("PRECHECK FAIL: DT,0 未获确认 (ack=%r status=%r)" %
              (dt_ack, dt_status))
        ser.close()
        return 1
    run_meta = {
        "fw_info": pre["fw_info"],
        "jdiag": pre["jdiag"],
        "ch_cfg": pre["ch_cfg"],
        # raw 身份行必须入 meta (2026-09-07 实证: 缺失 → validator V1 三连 FAIL;
        # verify 脚本一直带, ladder 漏 — 9/5 起 8 份 ladder JSON 身份链均不完整)
        "fw_raw": pre["fw_raw"],
        "jdiag_raw": pre["jdiag_raw"],
        "ch_raw": pre["ch_raw"],
        "dt_enabled": False,
        "dt_cmd_sent": dt_cmd_sent,
        "dt_ack": dt_ack,
        "dt_status": dt_status,
        "config_ack": {},
        "n_coexist": True,   # N 帧共存 (PDBBIN P1 与 N 帧 P2 仲裁, ~20-24Hz; 见 verify)
        "esc_enabled": bool(args.esc),   # 僵持积分逃逸 (CMD:POS_AW_ESC, 默认关)
    }

    def expect2(cmd, prefix, timeout=5.0):
        """reader 启动后的 expect: 走 resp_q (线程安全), 不直读串口。
        为避免 250 lines/s C/N 流挤掉响应 (KI fail 根因), 响应去重 + 增大限流。"""
        resp_q.clear()   # 此刻旧行都是垃圾
        ser.reset_output_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        seen = set()
        while time.time() < dl:
            while resp_q:
                l = resp_q.pop(0)
                if l in seen:
                    continue
                seen.add(l)
                if l.startswith(prefix):
                    return l
            time.sleep(0.005)
        return None

    def record_config_ack(name, ack):
        """Keep the observed response for every safety-relevant setup command."""
        run_meta["config_ack"].setdefault(name, []).append(ack)
        return bool(ack)

    def read_tx_p1_drop():
        """固件 TX 环 P1 丢帧计数器 (CMD:UART_RX?, uart_upload.c:1876) —
        gap 归因直接测量 (2026-09-09 Kimi A 规格): delta=0 → 丢帧在主机侧 RX,
        delta>0 → 固件真丢。返回 int 或 None (查询丢响应不阻塞主流程)。"""
        for _ in range(2):
            l = expect2("CMD:UART_RX?", "UART_RX,OK,")
            if l:
                f = parse_status_fields(l)
                v = f.get("tx_p1_drop", "")
                if v.isdigit():
                    return int(v)
            time.sleep(0.2)
        return None

    # 单线程 reader (PDBBIN + N帧) — 主线程不得再直读串口 (家族 segfault 教训)
    pdb_rows = []
    nframe_q = []   # (host_rx, p1_ts, p3_ang_deg, p6_iq, p8_fault, p2_state)
    resp_q = []     # reader 填充的响应行 (threading 启动后 expect 走它)
    resp_seen = set()   # 已处理行去重
    stop = [False]
    health = {          # 逐帧健康状态 (2026-09-05 恢复规划 #48)
        "scope_start": None, "first_hrx": None, "last_hrx": 0.0,
        "global_last_seq": None, "global_last_tick": None,
        "last_seq": -1, "last_tick": None, "seq_gap": 0, "tick_stall": 0,
        "bad_state": 0, "bad_fault": 0, "pdb_n": 0,
        "parser_seq_gap": 0, "crc_err": 0,
        "scope_seq_gap_base": 0, "scope_crc_err_base": 0,
        "run_seq_gap_base": 0, "run_crc_err_base": 0,
        "seq_gap_loci": [],   # [(tick_2khz, phase)] — 全局层记录 (同源, 2026-09-06)
        "esc_frames": 0,      # scope 内帧数 (ESC 观测分母)
        "esc_active_frames": 0,   # flags bit16=1 帧数 (pos_aw_esc_active 逃逸态)
    }
    phase_tag = ["init"]   # 当前阶段标记 (gap 归因用)

    def on_pdb(s):
        # (hrx, tick, flags, pos_err, iq_cmd, theta, ff_total, iq_act, v_mech)
        # v_mech=PDBBIN v_mech_rad_s (it.c:653) — 卡滞时 v_smooth 钉 0.2 地板
        # 而 v_mech≈0 → 直接判噪源是观测器 omega_lpf 而非差分 (2026-09-05 Kimi)
        pdb_rows.append((s.host_rx_time, s.tick_2khz, s.flags,
                         s.pos_err_rad, s.iq_cmd, s.theta_user_rad,
                         s.ff_total, s.iq_act, s.v_mech_rad_s, s.seq))
        # ── loci 全局层 (2026-09-06 Kimi 同源裁决): seq 推进用 parser 全局 last_seq,
        # gap 一律记 loci (记当时 phase_tag) — run/scope 计数从 loci 切片派生,
        # 计数与归因同源自洽 (r2 实证: scope 外 gap 计入 run 级但 loci 缺 → 不同源)
        seq_gap_now = False
        if health["global_last_seq"] is not None:
            expected = (health["global_last_seq"] + 1) & 0xFF
            if s.seq != expected:
                seq_gap_now = True
                if len(health["seq_gap_loci"]) < 16:
                    health["seq_gap_loci"].append((s.tick_2khz, phase_tag[0]))
        health["global_last_seq"] = s.seq
        # last_tick 同源化 (tick_stall 也全局判)
        if health["global_last_tick"] is not None and health["global_last_tick"] == s.tick_2khz:
            health["tick_stall"] += 1
        health["global_last_tick"] = s.tick_2khz
        if (health["scope_start"] is None or
                s.host_rx_time < health["scope_start"]):
            return
        # health: scope 内计数从 loci 派生 (同源) + state/fault 本地
        health["pdb_n"] += 1
        if health["first_hrx"] is None:
            health["first_hrx"] = s.host_rx_time
        health["last_hrx"] = s.host_rx_time
        if seq_gap_now:
            health["seq_gap"] += 1
        state = (s.flags >> 8) & 0xFF
        fault = s.flags & 0xFF
        if state != FOC_STATE_RUNNING:
            health["bad_state"] += 1
        if fault:
            health["bad_fault"] += 1
        # ESC 观测链 (2026-09-07): flags bit16 = pos_aw_esc_active (it.c:659)
        health["esc_frames"] += 1
        if (s.flags >> 16) & 1:
            health["esc_active_frames"] += 1

    def on_line(line):
        l = line.strip()
        resp_q.append(l)
        while len(resp_q) > 8000:
            del resp_q[0:4000]
        if l.startswith("N,"):
            p = l.split(",")
            if len(p) >= 21:
                try:
                    nframe_q.append((time.time(), p[1], float(p[3]),
                                     float(p[6]), p[8], p[2]))
                except ValueError:
                    pass

    parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb)

    def begin_health_scope(start_time):
        """Reset per-round health counters without discarding captured PDB rows."""
        stats = parser.stats[foclink.TYPE_PDB2]
        health.update({
            "scope_start": start_time, "first_hrx": None, "last_hrx": 0.0,
            "last_seq": -1, "last_tick": None, "seq_gap": 0,
            "tick_stall": 0, "bad_state": 0, "bad_fault": 0, "pdb_n": 0,
            "parser_seq_gap": 0, "crc_err": 0,
            "scope_seq_gap_base": stats.seq_gap,
            "scope_crc_err_base": stats.crc_err,
            "scope_loci_base": len(health["seq_gap_loci"]),   # loci 切片起点
            "esc_frames": 0, "esc_active_frames": 0,
        })

    def reader():
        while not stop[0]:
            try:
                n = ser.in_waiting
                if n:
                    parser.feed(ser.read(n))
                else:
                    time.sleep(0.001)
            except Exception:
                break

    def watch_health(t0, runid):
        """逐帧健康检查 (2026-09-05 恢复规划 #48): 流静默/掉帧/state/fault 越界
        即 raise SystemExit — 该 run invalid, 不让其混入汇总。"""
        now = time.time()
        stats = parser.stats[foclink.TYPE_PDB2]
        parser_seq_gap = stats.seq_gap - health["run_seq_gap_base"]
        crc_err = stats.crc_err - health["run_crc_err_base"]
        health["parser_seq_gap"] = max(0, parser_seq_gap)
        health["crc_err"] = max(0, crc_err)
        health["seq_gap"] = max(health["seq_gap"], health["parser_seq_gap"])
        if health["pdb_n"] == 0:
            if now - t0 > 1.0:
                raise SystemExit("HEALTH FAIL run=%s: PDBBIN 静默 >1s" % runid)
            return
        delta = now - health["last_hrx"]
        if delta > 1.0:
            raise SystemExit("HEALTH FAIL run=%s: PDBBIN 流静默 %.1fs (pdb=%d)"
                             % (runid, delta, health["pdb_n"]))
        if health["bad_state"] > 0:
            raise SystemExit("HEALTH FAIL run=%s: state!=RUNNING ×%d"
                             % (runid, health["bad_state"]))
        if health["bad_fault"] > 0:
            raise SystemExit("HEALTH FAIL run=%s: fault!=0 ×%d"
                             % (runid, health["bad_fault"]))
        # seq_gap 不当场杀 (2026-09-09): 固件 tx_p1_drop=0 证明丢帧在主机侧 RX
        # 抖动, 数据本体无损 (CRC=0); loci 已全局记录, 判定完全交给 validator
        # (每轮 ≤2+归因)。watch 只保留对数据本体致命的项: 静默/state/fault/
        # crc/tick_stall。012849 实证: r2 同相成簇 3 帧 (真拥塞) 与 r1 跨轮散布
        # (RX 抖动) 在 watch 层无法区分, 强行判 = 口径发明 — 归因材料齐全留给
        # validator 裁。
        if health["crc_err"] > 0:
            raise SystemExit("HEALTH FAIL run=%s: PDB CRC error ×%d" % (runid, health["crc_err"]))
        if health["tick_stall"] > 0:
            raise SystemExit("HEALTH FAIL run=%s: PDB tick stalled ×%d"
                             % (runid, health["tick_stall"]))
        if health["pdb_n"] >= 2:
            span = health["last_hrx"] - health["first_hrx"]
            if span >= 1.0:
                rate = (health["pdb_n"] - 1) / span
                if rate < PDB_MIN_RATE_HZ:
                    raise SystemExit("HEALTH FAIL run=%s: PDB sample rate %.1fHz < %.1fHz"
                                     % (runid, rate, PDB_MIN_RATE_HZ))

    def health_summary():
        """Return a serializable snapshot for both successful and TIMEOUT rounds."""
        if health["first_hrx"] is not None and health["last_hrx"] > health["first_hrx"]:
            sample_rate = ((health["pdb_n"] - 1) /
                           (health["last_hrx"] - health["first_hrx"]))
        else:
            sample_rate = None
        # seq_gap 同源: scope 计数 = loci 切片长度 (2026-09-06 Kimi 同源裁决) —
        # 本地 seq_gap 与 loci 都在 scope 内累计, 长度必然一致
        scope_loci = health["seq_gap_loci"][health["scope_loci_base"]:]
        return {
            "pdb_n": health["pdb_n"],
            "seq_gap": len(scope_loci),
            "tick_stall": health["tick_stall"],
            "bad_state": health["bad_state"],
            "bad_fault": health["bad_fault"],
            "crc_err": max(health["crc_err"],
                            parser.stats[foclink.TYPE_PDB2].crc_err -
                            health["run_crc_err_base"]),
            "sample_rate_hz": round(sample_rate, 1) if sample_rate is not None else None,
            "seq_gap_loci": scope_loci,
            "esc_frames": health["esc_frames"],
            "esc_active_frames": health["esc_active_frames"],
        }

    # 配置 (电压模式 S2) — 整体重试 ≤3 次 (初始化读超时/reader 残留, 非真失败)
    # 2026-09-06 实测: COG_CFG? 后紧跟 FRIC_COMP 偶发丢响应 (手测复现 1 次,
    #   单发 5/5 过) — 每条 config expect 带 2 次重试
    def expect_retry(cmd, prefix, timeout=3.0, tries=2):
        """成功返回响应原文 (config_ack 证据链必须是行, 不是布尔 — 2026-09-07
        validator 实证: FRIC_COMP=[True] 存布尔不构成身份证据), 失败 None。"""
        ack = None
        for i in range(tries):
            ack = expect(cmd, prefix, timeout=timeout)
            if ack:
                return ack
            time.sleep(0.2)
        return ack

    init_ok = False
    for iatt in range(3):
        # 全部 ack 预置 None: POS_DIRECT 首试丢响应时重试打印不炸
        # (2026-09-08 实证: UnboundLocalError cog_ack, 台架 G2-ESC 首跑即崩)
        unlock_ack = pos_direct_ack = cog_ack = None
        fric_ack = aw_ack = mode_ack = None
        unlock_ack = expect("CMD:UNLOCK,1", "UNLOCK,OK", timeout=3.0)
        record_config_ack("UNLOCK", unlock_ack)
        pos_direct_ack = expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK", timeout=3.0)
        record_config_ack("POS_DIRECT", pos_direct_ack)
        if pos_direct_ack:
            send("CMD:COG_CFG,0.0,60.0", 0.15)
            # 固件 COG_CFG? 响应 = "COG_CFG,gain=..,phase_deg=.." (uart_upload.c:1743),
            # 无 OK 字样 — 检查 gain= 存在即视为有效身份 (2026-09-06 init 3/3 根因)
            cog_lines = _query("CMD:COG_CFG?", 1.5)
            cog_ack = next((l for l in cog_lines if l.startswith("COG_CFG,gain=")), None)
            record_config_ack("COG_CFG", cog_ack)
            fric_ack = expect_retry("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK")
            record_config_ack("FRIC_COMP", fric_ack)
            aw_ack = expect_retry("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK")
            record_config_ack("POS_AW_MODE", aw_ack)
            mode_ack = expect_retry("CMD:MODE,3", "MODE,OK")
            record_config_ack("MODE", mode_ack)
            if (unlock_ack and fric_ack and aw_ack and mode_ack and
                    cog_ack is not None):
                init_ok = True
                break
        print("init 配置重试 %d/3 (u=%r p=%r c=%r f=%r a=%r m=%r)"
              % (iatt + 1, unlock_ack, pos_direct_ack, cog_ack,
                 fric_ack, aw_ack, mode_ack), flush=True)
        time.sleep(0.8)
    if not init_ok:
        print("初始配置失败 (POS_DIRECT/FRIC/AW/MODE), 停止"); ser.close(); return 1
    time.sleep(0.3)
    en_ok = False
    for attempt in range(3):
        enable_ack = expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0)
        record_config_ack("ENABLE", enable_ack)
        if enable_ack:
            en_ok = True
            break
        send("CMD:CLEAR_FAULT", 0.8)
    if not en_ok:
        print("ENABLE fail"); ser.close(); return 1

    # 僵持积分逃逸 (2026-09-07): --esc 时开 (enable 前不碰电机动态), 命令+回读双确认。
    # POS_AW_ESC,1 同时清零 count_diag → 每轮 count=本轮逃逸次数, 可归因。
    esc_confirmed = not args.esc
    if args.esc:
        esc_ack = expect_retry("CMD:POS_AW_ESC,1", "POS_AW_ESC,OK", tries=3)
        record_config_ack("POS_AW_ESC", esc_ack)
        esc_q = _query("CMD:POS_AW_ESC?", 1.5)
        esc_state = next((l for l in esc_q if l.startswith("POS_AW_ESC,OK,en=1")), None)
        record_config_ack("POS_AW_ESC_Q", esc_state)
        esc_confirmed = bool(esc_ack) and esc_state is not None
        if not esc_confirmed:
            print("ESC enable 未确认 (ack=%r q=%r), 停止" % (esc_ack, esc_state))
            ser.write(b"CMD:POS_AW_ESC,0\n")   # 兜底关闭, 不带病续跑
            ser.close()
            return 1
        print("ESC: enabled (count 清零)")
    ser.write(b"CMD:ON\n")
    time.sleep(0.4)
    ser.reset_input_buffer()
    time.sleep(0.5)

    # 锚定位
    a0 = None
    dl = time.time() + 1.0
    while time.time() < dl and a0 is None:
        for l in lb._drain():
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) >= 21:
                    try:
                        a0 = float(p[3])
                        break
                    except ValueError:
                        pass
        if a0 is None:
            time.sleep(0.02)
    if a0 is None:
        print("no angle"); ser.close(); return 1
    print("锚定位: %.2f°" % a0, flush=True)
    ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))
    time.sleep(2.0)
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    import threading
    stats = parser.stats[foclink.TYPE_PDB2]
    health["run_seq_gap_base"] = stats.seq_gap
    health["run_crc_err_base"] = stats.crc_err
    threading.Thread(target=reader, daemon=True).start()
    reader_ready[0] = True   # 此后所有串口读走 resp_q / nframe_q (家族: 双线程直读串口 segfault)
    lb.buf = ""   # 清零残余半行 (已交 reader 线程)

    mw = foclink.MeasureWindow(nframe_q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)

    results = []
    run_completed = False
    abort_reason = None
    try:
        for gname, kp, kd, ki in ladder:
            for rnd in range(args.rounds):
                rb = len(pdb_rows)
                round_started = time.time()
                begin_health_scope(round_started)
                round_acks = {}
                gain_ok = False
                for gar in range(args.ki_retry):
                    gain_ack = expect2("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (kp, kd),
                                       "POS_DIRECT_GAIN,OK")
                    round_acks.setdefault("POS_DIRECT_GAIN", []).append(gain_ack)
                    if gain_ack:
                        gain_ok = True
                        break
                    if gar < args.ki_retry - 1:
                        print("GAIN retry %d/%d (reader busy)"
                              % (gar + 1, args.ki_retry), flush=True)
                        time.sleep(0.5)
                if not gain_ok:
                    print("GAIN fail"); raise SystemExit(1)
                ki_ok = False
                for kia in range(args.ki_retry):
                    ki_ack = expect2("CMD:POS_DIRECT_KI,%.2f" % ki, "POS_DIRECT_KI,OK")
                    round_acks.setdefault("POS_DIRECT_KI", []).append(ki_ack)
                    if ki_ack:
                        ki_ok = True
                        break
                    if kia < args.ki_retry - 1:
                        print("KI retry %d/%d (reader busy)"
                              % (kia + 1, args.ki_retry), flush=True)
                        time.sleep(0.5)
                if not ki_ok:
                    resp_tail = resp_q[-15:]
                    print("KI fail resp_tail=%r" % (resp_tail,))
                    raise SystemExit(1)

                # ── ESC 轮级观测 (2026-09-07): 轮初/轮末 count 差 = 本轮逃逸次数 ──
                esc_count0 = None
                esc_count1 = None
                esc_active_end = None
                # gap 归因直接测量取样点 (Kimi A 规格): scope 首尾 tx_p1_drop
                tx_p1_0 = read_tx_p1_drop()
                if args.esc:
                    l0 = expect2("CMD:POS_AW_ESC?", "POS_AW_ESC,OK,en=")
                    f0 = parse_status_fields(l0) if l0 else {}
                    esc_count0 = int(f0["count"]) if f0.get("count", "").isdigit() else None
                    round_acks.setdefault("POS_AW_ESC_Q", []).append(l0)

                # ── 静止阶跃 +6° (从 a0 静止发) — t95 重定义 (恢复规划 #50) ──
                # 旧 t95 缺陷: 首帧 |err|<0.1° 即达标 (阶跃未生效/旧帧), 12:09 轮0
                #   t95=0.02s 却随后走到 -5.13° — 假到位。
                # 新定义: ①err 离开初始窗 |err|>=离去阈值(0.1°, 阶跃可见)
                #   ②重新进入 |err|<=5%*step(=0.3°) 并持续 dwell(0.5s) → 到位。
                # ±0.1° 指标改名 t_settle_0p1_deg (保留, 不叫 t95)。
                phase_tag[0] = "%s-r%d-step" % (gname, rnd)
                target_rad = (a0 + 6) * DEG2RAD
                t_cmd = time.time()
                t_step = t_cmd
                ser.write(b"CMD:PREF,%.6f\n" % target_rad)
                t95_state = {
                    "leave_rx": None, "window_enter_rx": None,
                    "t95_rx": None, "settle_0p1_rx": None,
                }
                step_scan = len(pdb_rows)
                dl2 = time.time() + 8.0
                while time.time() < dl2:
                    watch_health(t_step, "r%d" % rnd)   # 逐帧健康 (流静默/掉帧/state/fault)
                    new_end = len(pdb_rows)
                    if new_end > step_scan:
                        for pr in list(pdb_rows[step_scan:new_end]):
                            advance_t95(t95_state, pr[0], pr[3], t_cmd)
                        step_scan = new_end
                        if t95_state["t95_rx"] is not None:
                            break
                    time.sleep(0.02)
                post_t95_t0 = time.time()
                while time.time() - post_t95_t0 < 1.0:
                    watch_health(post_t95_t0, "r%d" % rnd)
                    time.sleep(0.02)

                # ── 斜坡回程 6° (12s @0.5°/s, a0+6 → a0) — 跟踪率 ──
                phase_tag[0] = "%s-r%d-ramp" % (gname, rnd)
                dur = 12.0
                t1 = time.time()
                last = 0.0
                while time.time() - t1 < dur:
                    watch_health(t1, "r%d" % rnd)   # 逐帧健康
                    t = time.time() - t1
                    if t - last >= 0.2:
                        frac = min(t / dur, 1.0)
                        cur_deg = a0 + 6.0 * (1.0 - frac)
                        ser.write(b"CMD:PREF,%.6f\n" % (cur_deg * DEG2RAD))
                        last = t
                    time.sleep(0.01)
                ser.write(b"CMD:PREF,%.6f\n" % (a0 * DEG2RAD))

                # ── 稳定门 → 测量窗(修复版, 目标=a0) ──
                phase_tag[0] = "%s-r%d-gate" % (gname, rnd)
                gate_t0 = time.time()
                ok, waited = mw.wait_stable(
                    a0, timeout=15.0,
                    health_cb=lambda: watch_health(gate_t0, "r%d" % rnd))
                if not ok:
                    print("%s 轮%d: TIMEOUT gate(%.1fs)" % (gname, rnd, waited), flush=True)
                    results.append({"gain": gname, "round": rnd, "gate": "TIMEOUT",
                                    "gate_wait_s": round(waited, 2),
                                    "gate_span_s": round(mw.last_gate_span_s, 3)
                                    if mw.last_gate_span_s is not None else None,
                                    "gate_frames": mw.last_gate_frames,
                                    "gate_pp_deg": round(mw.last_gate_pp, 6)
                                    if mw.last_gate_pp is not None else None,
                                    "config_ack": round_acks,
                                    "health": health_summary(),
                                    "pdb_n": health["pdb_n"]})
                    continue
                backlog = mw.begin()
                frames, _ = mw.collect(
                    health_cb=lambda: watch_health(gate_t0, "r%d" % rnd))
                angs = []
                for f in frames:
                    d = f[2] - a0
                    while d > 180.0: d -= 360.0
                    while d < -180.0: d += 360.0
                    angs.append(d)
                pp = (max(angs) - min(angs)) if angs else -1.0
                # 窗内偏差: mean 相对回位目标 a0 (=0) — Kimi 纠错: 原脚本按 +6 减是基准 bug
                resid_deg = (sum(angs) / len(angs)) if angs else None

                # ── 极限环 (PDBBIN pos_err 后段 1s) ──
                traj = pdb_rows[rb:]
                t_now = time.time()
                pes = [r[3] for r in traj if r[0] > t_now - 1.0] if traj else []
                lc_pp_deg = ((max(pes) - min(pes)) / DEG2RAD) if len(pes) > 5 else None

                # ── 跟踪率 (PDBBIN theta 相对斜坡, 后 4-8s 段) ──
                track = None
                ramp_pdb = [r for r in traj if r[0] >= t1 and r[0] <= t1 + dur]
                if ramp_pdb:
                    seg = [r for r in ramp_pdb if 3.5 <= (r[0] - t1) <= 8.5]
                    if len(seg) >= 5:
                        d = seg[-1][5] - seg[0][5]   # r[5] = theta_user_rad
                        while d > 180.0 * DEG2RAD: d -= 360.0 * DEG2RAD
                        while d < -180.0 * DEG2RAD: d += 360.0 * DEG2RAD
                        ideal = -6.0 * DEG2RAD * (seg[-1][0] - seg[0][0]) / dur  # 回程负向
                        track = (d / ideal * 100.0) if abs(ideal) > 0 else None

                t_leave = t95_state["leave_rx"]
                t95_enter = t95_state["window_enter_rx"]
                t95 = t95_state["t95_rx"]
                t_settle_0p1_deg = t95_state["settle_0p1_rx"]
                round_health = health_summary()
                # gap 归因直接测量 (Kimi A 规格): scope 尾取样 + 无符号回绕安全差值
                tx_p1_1 = read_tx_p1_drop()
                if tx_p1_0 is not None and tx_p1_1 is not None:
                    round_health["tx_p1_drop_delta"] = (tx_p1_1 - tx_p1_0) & 0xFFFFFFFF
                else:
                    round_health["tx_p1_drop_delta"] = None

                ts = time.strftime("%H:%M:%S")
                print("%s 轮%d: ts=%s overflow_backlog=%d gate=%.1fs pp=%.3f° "
                      "t95=%s t_leave=%s track=%s lc_pp=%s pdb=%d" %
                      (gname, rnd, ts, backlog, waited, pp,
                        ("%.2fs" % (t95 - t_cmd)) if t95 is not None else "N/A",
                       ("%.2fs" % (t_leave - t_cmd)) if t_leave is not None else "N/A",
                       ("%.0f%%" % track) if track else "N/A",
                       ("%.2f°" % lc_pp_deg) if lc_pp_deg else "N/A", len(traj)), flush=True)
                if args.esc:
                    l1 = expect2("CMD:POS_AW_ESC?", "POS_AW_ESC,OK,en=")
                    f1 = parse_status_fields(l1) if l1 else {}
                    esc_count1 = (int(f1["count"])
                                  if f1.get("count", "").isdigit() else None)
                    esc_active_end = f1.get("active")
                    round_acks.setdefault("POS_AW_ESC_Q", []).append(l1)
                    # ESC 轮断言 (Kimi 规格): count 差>0 = 逃逸确实触发过, 数据可归因;
                    # count 差 0 → 该轮数据无法归因 (ESC 没触发=测的是旧行为) → invalid
                    esc_delta = (esc_count1 - esc_count0
                                 if esc_count1 is not None and esc_count0 is not None
                                 else None)
                    if esc_delta is None or esc_delta <= 0:
                        print("ESC FAIL %s 轮%d: esc_count %s→%s 无触发 — "
                              "数据无法归因, run invalid" %
                              (gname, rnd, esc_count0, esc_count1), flush=True)
                        raise SystemExit(1)
                    print("ESC: count %d→%d (delta=%d) active_end=%s" %
                          (esc_count0, esc_count1, esc_delta, esc_active_end),
                          flush=True)
                results.append({
                    "gain": gname, "kp": kp, "kd": kd, "ki": ki,
                    "round": rnd, "ts": ts, "backlog": backlog,
                    "gate_ok": ok, "gate_wait_s": round(waited, 2),
                    "gate_span_s": round(mw.last_gate_span_s, 3),
                    "gate_frames": mw.last_gate_frames,
                    "gate_pp_deg": round(mw.last_gate_pp, 6),
                    "config_ack": round_acks,
                    "health": round_health,
                    "steady_pp_deg": round(pp, 4),
                    "steady_resid_deg": round(resid_deg, 4) if resid_deg is not None else None,
                    "t_cmd_host_rx": round(t_cmd, 6),
                    "t95_host_rx": round(t95, 6) if t95 is not None else None,
                    "t95_window_enter_host_rx": round(t95_enter, 6)
                    if t95_enter is not None else None,
                    "t_leave_host_rx": round(t_leave, 6) if t_leave is not None else None,
                    "t_settle_0p1_deg_host_rx": round(t_settle_0p1_deg, 6)
                    if t_settle_0p1_deg is not None else None,
                    "t95_s": round((t95 - t_cmd), 3) if t95 is not None else None,
                    "t95_window_enter_s": round((t95_enter - t_cmd), 3)
                    if t95_enter is not None else None,
                    "t_leave_s": round((t_leave - t_cmd), 3)
                    if t_leave is not None else None,
                    "t_settle_0p1_deg_s": round((t_settle_0p1_deg - t_cmd), 3)
                    if t_settle_0p1_deg is not None else None,
                    "track_pct": round(track, 1) if track else None,
                    "limit_cycle_pp_deg": round(lc_pp_deg, 4) if lc_pp_deg is not None else None,
                    "esc_count_delta": (esc_count1 - esc_count0)
                    if args.esc and esc_count1 is not None
                    and esc_count0 is not None else None,
                    "esc_active_end": esc_active_end if args.esc else None,
                    "pdb_n": len(traj),
                    "theta_traj": [(round(r[0], 3), round(r[5] / DEG2RAD, 4)) for r in traj[::8]],
                    "poserr_traj": [(round(r[0], 3), round(r[3] / DEG2RAD, 4), round(r[4], 5))
                                    for r in traj[::8]],
                    "poserr_traj_full": [(round(r[0], 6), round(r[3] / DEG2RAD, 6), round(r[4], 6))
                                         for r in traj],
                    "ff_iq_traj": [(round(r[0], 3), round(r[6], 5), round(r[7], 5))
                                   for r in traj[::8]],   # (hrx, ff_total, iq_act)
                    "v_mech_traj": [(round(r[0], 3), round(r[8], 6))
                                    for r in traj[::8]],  # (hrx, v_mech_rad_s) — Stribeck噪源判别
                })
                time.sleep(1.0)
        run_completed = True
    except SystemExit as exc:
        abort_reason = str(exc)
        raise
    except Exception as exc:
        abort_reason = "%s: %s" % (type(exc).__name__, exc)
        raise
    finally:
        stop[0] = True
        time.sleep(0.3)
        ser.write(b"CMD:POS_AW_ESC,0\n")   # 兜底: 掉链即关 ESC, 恢复旧行为 (Kimi 兜底规格)
        time.sleep(0.2)
        ser.write(b"CMD:VOLT_OFF\n")
        send("CMD:OFF")
        send("CMD:MODE,0")
        send("CMD:STOP")
        send("CMD:CLEAR_FAULT")
        send("CMD:PDBBIN,0")
        ser.close()
        # 写盘放 finally: 任何退出路径 (含 raise SystemExit) 都落盘
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "s2_gain_ladder_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        meta = dict(run_meta)
        # run 级健康快照入 meta (090952 实证: r0 中途杀时 results 空,
        #   loci 只在 health dict — 无落盘则超容忍 abort 无法归因)
        meta["run_health"] = {
            "seq_gap_loci": list(health["seq_gap_loci"]),
            "esc_active_frames": health["esc_active_frames"],
            "tick_stall": health["tick_stall"],
        }
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"schema": "s2_gain_ladder.v2", "args": vars(args),
                       "run_status": {"valid": run_completed,
                                       "abort_reason": abort_reason},
                       "meta": meta, "results": results},
                      f, ensure_ascii=False, indent=1)
        print("\nJSON: %s (rounds=%d)" % (out, len(results)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
