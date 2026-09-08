#!/usr/bin/env python3
"""新固件低速摩擦验证：斜坡滞后 + 阶跃过冲 + 稳态 pp（带轨迹打印诊断）
DIRECT kp=0.49/kd=0.007, 摩擦补偿 --comp, COG --cog-gain, POS_DIRECT_KI 运行时设。

2026-09-05 迁移 (Kimi 定案):
  - 稳态窗 → foclink.MeasureWindow (begin排压 + collect窗内帧 + wait_stable门)
  - 斜坡/阶跃采集 → PDBBIN (200Hz浮点 + seq/CRC校验), fault/state 逐帧健康检查
  - 回归只认迁移后的数据 (数据管道烧过三次, 审查是固定流程)
2026-09-06 补 #52 (同 ladder 标准): fail-closed 预检 (FW_INFO/JDIAG/CH_CFG) +
  DT,0 双确认 + config_ack + schema verify_low_speed.v2 + 逐帧 health 摘要。
用法: python scripts/verify_low_speed.py --port COM10 --power-ok [--comp 0.022]
      [--aw 1,0.03] [--reps 2]
Note: 本机 GBK 控制台中文输出乱码, 跑前 set PYTHONIOENCODING=utf-8 或 chcp 65001。
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0

# flags 编码 (stm32h7xx_it.c 打包): 高 8 位=state (bits 15:8, 原值 0-5),
# 低 8 位=fault_code。枚举定义在 MDK-ARM/code/foc_app.h:169-175:
# IDLE=0/INIT=1/PARAM_IDENTIFY=2/READY=3/RUNNING=4/FAULT=5
FOC_STATE_RUNNING = 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--step", type=float, default=6.0)
    ap.add_argument("--ramp-deg-s", type=float, default=2.0)
    ap.add_argument("--ki", type=float, default=0.37)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--comp", type=float, default=0.022)
    ap.add_argument("--cog-gain", type=float, default=0.0,
                    help="COG LUT 增益 (默认0=固件定版OFF; 覆盖会改默认行为)")
    ap.add_argument("--aw", default="1,0.03", help="积分抗饱和律 'mode,rate' (默认 1,0.03)")
    ap.add_argument("--reps", type=int, default=1, help="斜坡+阶跃+稳态 重复轮数 (回归 ×2)")
    ap.add_argument("--esc", action="store_true",
                    help="开启僵持积分逃逸 CMD:POS_AW_ESC,1 (惰性证明轮: 断言 "
                         "esc_count==0, >0 即 fail — 方向与 ladder 相反)")
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0
    step = args.step
    ramp_s = args.ramp_deg_s

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    ser.reset_input_buffer()

    # 20kHz 固件预清理: 停 N/C/PDB 流, 否则 ENABLE/MODE 响应被遥测流淹没 (2026-08-30 实测)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:STOP\n",
              b"CMD:CLEAR_FAULT\n", b"CMD:VOLT_OFF\n"):
        ser.write(c)
        time.sleep(0.15)
    ser.reset_input_buffer()

    class LineBuffer:
        """累积式行缓存: 串口数据可能被切成任意片段, 跨迭代累积直到换行符。"""
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

    def expect(cmd, prefix, timeout=1.5):
        """仅 reader 启动前可用 (直读串口)。"""
        ser.reset_input_buffer()
        lb.buf = ""
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            for l in lb._drain():
                if l.strip().startswith(prefix):
                    return True
            time.sleep(0.01)
        return False

    def read_angle():
        """reader 启动前直读 N 帧角度 (config 锚定段专用)。"""
        dl = time.time() + 0.8
        while time.time() < dl:
            for l in lb._drain():
                if l.startswith("N,"):
                    p = l.split(",")
                    if len(p) >= 25:
                        return float(p[3])
            time.sleep(0.01)
        return None

    def send(cmd, wait=0.3):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)

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

    # ── fail-closed 预检 (2026-09-06 恢复规划 #52, 同 ladder 标准): FW_INFO/JDIAG/CH_CFG ──
    def _precheck():
        lines = _query("CMD:FW_INFO?", 2.0)
        fw = next((l for l in lines if l.startswith("FW_INFO,")), None)
        if fw is None:
            print("PRECHECK FAIL: FW_INFO 无响应"); ser.close(); return 1
        fw_ver = dict(kv.split("=", 1) for kv in fw.split(",")[2:] if "=" in kv)
        if "version" not in fw_ver:
            print("PRECHECK FAIL: FW_INFO 缺 version"); ser.close(); return 1

        lines = _query("CMD:JDIAG", 2.0)
        jd = next((l for l in lines if l.startswith("JDIAG,")), None)
        if jd is None:
            print("PRECHECK FAIL: JDIAG 无响应"); ser.close(); return 1
        jd_map = {}
        for kv in jd.split(",")[2:]:
            if "=" in kv:
                k, v = kv.split("=", 1)
                jd_map[k] = v
        for req in ("J", "enc", "valid", "cog_gain", "cog_phase"):
            if req not in jd_map:
                print("PRECHECK FAIL: JDIAG 缺 %s" % req); ser.close(); return 1

        lines = _query("CMD:CH_CFG?", 2.0)
        cc = next((l for l in lines if l.startswith("CH_CFG,")), None)
        cc_map = None
        if cc is None:
            print("PRECHECK WARN: CH_CFG 无响应 (记录, 不中止)")
        else:
            cc_map = dict(kv.split("=", 1) for kv in cc.split(",")[2:] if "=" in kv)

        # DT,0 双确认: DT,OK(0) + DT? 查询 en=0 (恢复规划: DT 固定关闭)
        dt_ack = None
        if expect("CMD:DT,0", "DT,OK", timeout=2.0):
            dt_ack = "DT,OK,0"
        dt_lines = _query("CMD:DT?", 2.0)
        dt_status = next((l for l in dt_lines if l.startswith("DT,OK")), None)
        if dt_status is None or "en=0" not in dt_status:
            # 老格式可能 DT,OK,0; DT,OK,en=0
            if not (dt_ack and dt_ack.endswith(",0")):
                print("PRECHECK WARN: DT? 未确认 en=0 (记录, 不中止)")

        return {
            "fw_info": fw_ver, "jdiag": jd_map, "ch_cfg": cc_map,
            "jdiag_raw": jd, "fw_raw": fw, "ch_raw": cc,
            "dt_ack": dt_ack, "dt_status": dt_status,
        }

    pre = _precheck()
    if pre == 1:
        return 1
    print("PRECHECK:", pre["fw_info"].get("version"), "enc=%s" % pre["jdiag"].get("enc"),
          "cog_gain=%s" % pre["jdiag"].get("cog_gain"), flush=True)
    ser.reset_input_buffer()

    config_ack = {}   # 2026-09-06 #52: 每条 config 命令 ack 记录 (validator V1 用)
    send("CMD:UNLOCK,1")   # UNLOCK 无 OK 响应
    config_ack["UNLOCK"] = True
    config_ack["POS_DIRECT"] = expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK")
    config_ack["POS_DIRECT_GAIN"] = expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK")
    config_ack["POS_DIRECT_KI"] = expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK")
    ser.write(b"CMD:COG_CFG,%.3f,60.0\n" % args.cog_gain)
    config_ack["FRIC_COMP"] = expect("CMD:FRIC_COMP,%.3f,%.3f" % (args.comp, args.comp), "FRIC_COMP,OK")
    aw_mode, aw_rate = args.aw.split(",")
    config_ack["POS_AW_MODE"] = expect("CMD:POS_AW_MODE,%s,%s" % (aw_mode, aw_rate), "POS_AW_MODE,OK")
    config_ack["MODE"] = expect("CMD:MODE,2", "MODE,OK")
    if not expect("CMD:ENABLE,1", "ENABLE,OK"):
        print("ENABLE fail")
        ser.close()
        return 1
    config_ack["ENABLE"] = True

    # 僵持积分逃逸 (2026-09-07): --esc 时开 + 回读确认; verify 断言方向 = count==0
    esc_confirmed = not args.esc
    if args.esc:
        esc_ack = expect("CMD:POS_AW_ESC,1", "POS_AW_ESC,OK")
        config_ack["POS_AW_ESC"] = esc_ack
        esc_q = _query("CMD:POS_AW_ESC?", 2.0)
        esc_state = next((l for l in esc_q if l.startswith("POS_AW_ESC,OK,en=1")), None)
        config_ack["POS_AW_ESC_Q"] = esc_state
        esc_confirmed = bool(esc_ack) and esc_state is not None
        if not esc_confirmed:
            print("ESC enable 未确认 (ack=%r q=%r), 停止" % (esc_ack, esc_state))
            ser.write(b"CMD:POS_AW_ESC,0\n")
            ser.close()
            return 1
        print("ESC: enabled (count 清零)")

    # 恢复 N 帧 (预清理 CMD:OFF 关掉了主遥测)
    ser.write(b"CMD:ON\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    time.sleep(0.5)
    a0 = read_angle()
    print("start angle: %.2f" % (a0 if a0 else -1))
    if a0 is None:
        print("no angle")
        ser.close()
        return 1
    # 钉住当前位置: 清 pos_ref 遗留(ENABLE后位置环可能被旧 pos_ref 猛拉)
    ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
    time.sleep(0.6)
    a0 = read_angle()
    print("pinned at: %.2f" % (a0 if a0 else -1))
    if a0 is None:
        print("no angle after pin")
        ser.close()
        return 1
    if a0 < 25 or a0 > 335:
        ser.write(b"CMD:PREF,%.5f\n" % (100.0 * DEG2RAD))
        time.sleep(1.2)
        a0 = read_angle()
        print("moved to: %.2f" % (a0 if a0 else -1))

    # ── reader 线程 (PDBBIN + N帧) — 此后主线程不得直读串口 (家族 segfault 教训) ──
    pdb_rows = []    # (hrx, tick, flags, pos_err, iq_cmd, theta_user_rad, ff_total, iq_act)
    nframe_q = []    # (hrx, p1, p3_ang_deg, p6_iq, p8_fault, p2_state)
    resp_q = []      # reader 填充的文本响应行 (ESC count 查询用, 同 ladder)
    stop = [False]
    phase_tag = ["init"]   # 当前阶段标记 (on_pdb 归因 gap 位置用, list 可变共享)
    health = {       # 逐帧健康摘要 (2026-09-06 #52, 同 ladder 标准)
        "pdb_n": 0, "seq_gap": 0, "bad_state": 0, "bad_fault": 0,
        "crc_err": 0, "tick_stall": 0,
        "seq_gap_loci": [],   # [(tick_2khz, phase_tag)] — gap 位置归因 (Kimi 2026-09-06)
    }

    def on_pdb(s):
        pdb_rows.append((s.host_rx_time, s.tick_2khz, s.flags,
                         s.pos_err_rad, s.iq_cmd, s.theta_user_rad,
                         s.ff_total, s.iq_act))
        health["pdb_n"] += 1
        state = (s.flags >> 8) & 0xFF
        fault = s.flags & 0xFF
        if state != FOC_STATE_RUNNING:
            health["bad_state"] += 1
        if fault:
            health["bad_fault"] += 1
        if health.get("_last_seq", -2) >= 0 and s.seq != ((health["_last_seq"] + 1) & 0xFF):
            health["seq_gap"] += 1
            # gap 位置归因: 容忍度的前提是归因材料 (Kimi 2026-09-06 补救规格)
            if len(health["seq_gap_loci"]) < 16:   # 防爆表 (极端流断只记前 16 处)
                health["seq_gap_loci"].append((s.tick_2khz, phase_tag[0]))
        health["_last_seq"] = s.seq

    def on_line(line):
        l = line.strip()
        resp_q.append(l)   # reader 后 expect_q 查询用 (ESC count 等)
        while len(resp_q) > 2000:
            del resp_q[0:1000]
        if l.startswith("N,"):
            p = l.split(",")
            if len(p) >= 21:
                try:
                    nframe_q.append((time.time(), p[1], float(p[3]),
                                     float(p[6]), p[8], p[2]))
                except ValueError:
                    pass

    parser = foclink.MixedStreamParser(line_cb=on_line, pdb2_cb=on_pdb)

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

    def read_angle_q(timeout=2.0):
        """reader 启动后读最新 N 帧角度 (nframe_q)。"""
        dl = time.time() + timeout
        while time.time() < dl:
            if nframe_q:
                f = nframe_q[-1]
                if time.time() - f[0] < 0.3:
                    return f[2]
            time.sleep(0.02)
        return None

    def expect_q(cmd, prefix, timeout=2.0):
        """reader 启动后的 expect: 走 resp_q (线程安全), 不直读串口 (家族教训)。"""
        resp_q.clear()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            for l in resp_q:
                if l.startswith(prefix):
                    return l
            time.sleep(0.005)
        return None

    # 开 PDBBIN 后再启 reader (二进制帧由 parser 消化, 不乱文本行)
    ser.reset_input_buffer()
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    threading.Thread(target=reader, daemon=True).start()
    time.sleep(0.3)

    mw = foclink.MeasureWindow(nframe_q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)

    def watch_health(t_ref):
        """轨迹健康检查: PDBBIN 帧推进 + fault/state。返回最新 hrx。
        数据流本身是观测链 (18:43 案教训): 静默/掉态/fault 全部当场爆。"""
        if not pdb_rows:
            if time.time() - t_ref > 1.0:
                raise RuntimeError("PDBBIN stream silent >1s (环死?)")
            return t_ref
        pr = pdb_rows[-1]
        flags = pr[2]
        state = (flags >> 8) & 0xFF
        fault = flags & 0xFF
        if fault:
            raise RuntimeError("fault latched mid-run: flags=0x%X (fault=%d)" % (flags, fault))
        if state != FOC_STATE_RUNNING:
            raise RuntimeError("state dropped mid-run: flags=0x%X (state=%d)" % (flags, state))
        return pr[0]

    def angle_traj(pdbs, t0, a0_deg):
        """PDBBIN theta_user → (t-t0, 相对a0的deg) 采样列 (环绕修正)。"""
        out = []
        for r in pdbs:
            d = (r[5] / DEG2RAD) - a0_deg
            while d > 180.0: d -= 360.0
            while d < -180.0: d += 360.0
            out.append((r[0] - t0, d))
        return out

    def sp(samples, target_deg, checkpoints=(0.5, 1.0, 2.0, 2.8)):
        """到位判定。arrival=到位%(100=完全到位, >100=过冲); 检查点按指令发出后时刻。
        历史坑 (2026-08-30): 旧 err 字段是"剩余%"但标签写"到位%", 被倒读成 PASS;
        斜坡旧检查点 0.5/1/2s 全在斜坡内部, 保持段收敛从未被测。
        2.8s 检查点 = 2026-09-05 回归判据「阶跃 2.8s 内到 5.8° 级」。"""
        if len(samples) < 5:
            return None
        target = abs(target_deg)
        arr = {}
        for tm in checkpoints:
            b = min(samples, key=lambda s: abs(s[0] - tm))
            arr[tm] = round(min(abs(b[1]) / target, 1.2) * 100.0, 1)
        t_end = samples[-1][0]
        stab = [abs(abs(s[1]) - target) for s in samples if s[0] >= t_end - 1.0]
        dev = max((abs(abs(s[1]) - target) for s in samples), default=0.0)
        return {"arrival": arr, "pp": max(stab) - min(stab) if stab else 0.0,
                "max_dev": max(0.0, dev), "n": len(samples)}

    def traj_str(samples, every=0.5):
        out = []
        for i in range(int(samples[-1][0] * (1.0 / every)) + 1):
            tgt = i * every
            b = min(samples, key=lambda s: abs(s[0] - tgt))
            out.append("%.1f°@%.1fs" % (b[1], b[0]))
        return " ".join(out)

    results = {"reps": []}
    run_error = None
    try:
        for rep in range(args.reps):
            rep_res = {}
            phase_tag[0] = "rep%d-ramp" % (rep + 1)

            # ── 1) 斜坡 +step @ramp_s (判据: 跟踪率 ≥95%, 定版 98.8%) ──
            a0 = read_angle_q()
            if a0 is None:
                raise RuntimeError("no angle for ramp")
            target_rad = (a0 + step) * DEG2RAD
            dur = abs(target_rad - a0 * DEG2RAD) / (ramp_s * DEG2RAD)
            t0 = time.time()
            last_sent = 0.0
            rb = len(pdb_rows)
            # rep 起点 health 快照 (2026-09-06 #52: 本 rep 期间的差值)
            hp0 = dict(health)   # 快照 (含 pdb_n/seq_gap/bad_state/bad_fault)
            # ESC 惰性证明 (2026-09-07): rep 起点 count 快照
            esc_c0 = None
            if args.esc:
                lq = expect_q("CMD:POS_AW_ESC?", "POS_AW_ESC,OK,en=")
                fq = parse_status_fields(lq) if lq else {}
                esc_c0 = int(fq["count"]) if fq.get("count", "").isdigit() else None
            print("\n=== rep%d 斜坡 +%.1f° @%.1f°/s (dur=%.1fs) ===" %
                  (rep + 1, step, ramp_s, dur), flush=True)
            while time.time() - t0 < dur + 3.0:
                t = time.time() - t0
                if t - last_sent >= 0.2:
                    frac = min(t / dur, 1.0)
                    cur = a0 * DEG2RAD + (target_rad - a0 * DEG2RAD) * frac
                    ser.write(b"CMD:PREF,%.5f\n" % cur)
                    last_sent = t
                watch_health(t0)
                time.sleep(0.01)
            samp = angle_traj(pdb_rows[rb:], t0, a0)
            # 跟踪率: 斜坡中后段 (0.35-0.85)×dur 实测位移/理想位移
            track = None
            seg = [r for r in pdb_rows[rb:] if t0 + 0.35 * dur <= r[0] <= t0 + 0.85 * dur]
            if len(seg) >= 5:
                d = seg[-1][5] - seg[0][5]
                while d > 180.0 * DEG2RAD: d -= 360.0 * DEG2RAD
                while d < -180.0 * DEG2RAD: d += 360.0 * DEG2RAD
                ideal = (target_rad - a0 * DEG2RAD) * (seg[-1][0] - seg[0][0]) / dur
                track = (d / ideal * 100.0) if abs(ideal) > 1e-9 else None
            r = sp(samp, step, checkpoints=(dur + 0.5, dur + 1.5, dur + 2.5))
            print("  n=%d %s" % (len(samp), traj_str(samp)), flush=True)
            if r:
                print("  到位%%(斜坡后) %s | pp=%.3f° 最大偏差=%.2f° | 跟踪率=%s" %
                      (" ".join("%.1fs=%s" % (k, v) for k, v in r["arrival"].items()),
                       r["pp"], r["max_dev"],
                       ("%.1f%%" % track) if track else "N/A"), flush=True)
                r["track_pct"] = round(track, 1) if track else None
                rep_res["ramp"] = r
            else:
                print("  采样不足!")
                rep_res["ramp"] = {"track": None}
            # 回起点
            phase_tag[0] = "rep%d-return" % (rep + 1)
            ser.write(b"CMD:PREF,%.5f\n" % (a0 * DEG2RAD))
            time.sleep(1.5)

            # ── 2) 阶跃 +6° (判据: 2.8s 内到 5.8° 级) ──
            phase_tag[0] = "rep%d-step" % (rep + 1)
            a1 = read_angle_q()
            print("\n=== rep%d 阶跃 +%.1f° ===" % (rep + 1, step), flush=True)
            t2 = time.time()
            rb2 = len(pdb_rows)
            ser.write(b"CMD:PREF,%.5f\n" % ((a1 + step) * DEG2RAD))
            while time.time() - t2 < 3.0:
                watch_health(t2)
                time.sleep(0.01)
            samp2 = angle_traj(pdb_rows[rb2:], t2, a1)
            r = sp(samp2, step)
            print("  n=%d %s" % (len(samp2), traj_str(samp2, 0.25)), flush=True)
            if r:
                print("  到位%% %s | pp=%.3f° 最大偏差=%.2f°" %
                      (" ".join("%.1fs=%s" % (k, v) for k, v in r["arrival"].items()),
                       r["pp"], r["max_dev"]), flush=True)
                rep_res["step"] = r
            else:
                print("  采样不足!")
                rep_res["step"] = None
            # 回起点并测稳态 (2026-09-05 迁移: 稳定门 + MeasureWindow)
            phase_tag[0] = "rep%d-gate" % (rep + 1)
            ser.write(b"CMD:PREF,%.5f\n" % (a1 * DEG2RAD))
            ok_gate, waited = mw.wait_stable(a1, timeout=10.0)
            phase_tag[0] = "rep%d-steady" % (rep + 1)
            if not ok_gate:
                print("  稳态门 TIMEOUT(%.1fs) — 数据仍采集但门未过" % waited, flush=True)
            backlog = mw.begin()
            frames, _ = mw.collect()
            angs = []
            for f in frames:
                d = f[2] - a1
                while d > 180.0: d -= 360.0
                while d < -180.0: d += 360.0
                angs.append(d)
            if len(angs) >= 5:
                pp = max(angs) - min(angs)
                resid = sum(angs) / len(angs)
                print("  n=%d 稳态pp=%.3f° resid=%.3f° backlog=%d gate=%.1fs" %
                      (len(angs), pp, resid, backlog, waited), flush=True)
                rep_res["steady"] = {"pp_deg": round(pp, 3), "resid_deg": round(resid, 3),
                                     "backlog": backlog, "gate_ok": ok_gate,
                                     "gate_wait_s": round(waited, 2), "n": len(angs)}
            else:
                print("  稳态采样不足 (n=%d)" % len(angs), flush=True)
                rep_res["steady"] = {"n": len(angs), "gate_ok": ok_gate}

            # 轨迹列 (判读爬行/过冲/掉零用, 每 100 帧抽稀 ≈2Hz)
            rep_res["traj"] = [
                {"t": round(r[0] - t0, 3),
                 "theta_deg": round(r[5] / DEG2RAD, 4),
                 "poserr_deg": round(r[3] / DEG2RAD, 4),
                 "ff_iq": round(r[6], 5),
                 "iq_act": round(r[7], 5)}
                for r in pdb_rows[rb:][::100]]
            # health 摘要 (2026-09-06 #52, validator V4/V6 用) — rep 起点快照差
            dur_s = max(0.1, time.time() - t0)
            rep_res["health"] = {
                "pdb_n": health["pdb_n"] - hp0["pdb_n"],
                "seq_gap": health["seq_gap"] - hp0["seq_gap"],
                "tick_stall": 0,
                "bad_state": health["bad_state"] - hp0["bad_state"],
                "bad_fault": health["bad_fault"] - hp0["bad_fault"],
                "crc_err": 0,
                "sample_rate_hz": round((health["pdb_n"] - hp0["pdb_n"]) / dur_s, 1),
                # gap 位置归因 (2026-09-06 Kimi 补救规格): 本 rep 的 loci 切片
                "seq_gap_loci": health["seq_gap_loci"][hp0.get("_loci_n", 0):],
            }
            health["_loci_n"] = len(health["seq_gap_loci"])   # 下 rep 起点
            # ESC 惰性断言 (2026-09-07, 方向与 ladder 相反): count 必须不变。
            # 电流模式全场景 err 离 3° 触发线 4 倍裕量 → 任何触发 = 惰性破坏 = fail
            if args.esc:
                lq = expect_q("CMD:POS_AW_ESC?", "POS_AW_ESC,OK,en=")
                fq = parse_status_fields(lq) if lq else {}
                esc_c1 = int(fq["count"]) if fq.get("count", "").isdigit() else None
                if esc_c1 is None or esc_c0 is None or esc_c1 != esc_c0:
                    print("ESC FAIL rep%d: count %s→%s 变化 — 惰性破坏, fail" %
                          (rep + 1, esc_c0, esc_c1), flush=True)
                    rep_res["esc_violation"] = True
                    raise RuntimeError("ESC count changed in current-mode verify")
                rep_res["esc_count"] = esc_c1
            results["reps"].append(rep_res)
    except (RuntimeError, SystemExit) as exc:
        run_error = str(exc)
        print("RUN ABORT: %s" % run_error, flush=True)   # 数据带回: JSON 照落 (run_status invalid)
    finally:
        stop[0] = True
        time.sleep(0.3)
        ser.write(b"CMD:POS_AW_ESC,0\n")   # 兜底: 掉链即关 ESC (Kimi 兜底规格)
        ser.write(b"CMD:PDBBIN,0\n")   # 缺 \n 命令不进解析器 → 电机保持使能 (Kimi 2026-09-05)
        ser.write(b"CMD:STOP\n")
        ser.write(b"CMD:CLEAR_FAULT\n")
        ser.close()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "verify_lowspeed_%s.json" % ts)
    meta = {} if pre == 1 else pre
    if isinstance(meta, dict):
        meta = {"fw_info": meta.get("fw_info"), "jdiag": meta.get("jdiag"),
                "ch_cfg": meta.get("ch_cfg"), "fw_raw": meta.get("fw_raw"),
                "jdiag_raw": meta.get("jdiag_raw"), "ch_raw": meta.get("ch_raw"),
                "dt_enabled": False, "dt_cmd_sent": True,
                "dt_ack": meta.get("dt_ack"), "dt_status": meta.get("dt_status"),
                "config_ack": config_ack,
                # N 帧共存: PDBBIN P1 与 N 帧 P2 仲裁, 实测 ~20-24Hz (8/31 TX 泵
                #   专项一致); validator mode-aware rate 用 (2026-09-06 Kimi)
                "n_coexist": True,
                "esc_enabled": bool(args.esc)}
    doc = {"schema": "verify_low_speed.v2",
           "run_status": {"valid": bool(pre != 1 and run_error is None),
                          "abort_reason": run_error},
           "args": vars(args), "meta": meta, "results": results}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    print("\n报告: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
