#!/usr/bin/env python3
"""loop_prof_exp.py — 主循环黑洞专项 P2 实验矩阵 (E0-E4, 每档 30s)

2026-08-31 任务卡「主循环黑洞专项」§2:
  E0: CMD:ON 关, 无 PREF, 电机禁能静止, PDBBIN 200Hz         — 基线五段 + 迭代率
  E1: E0 + CMD:ON (N 帧 50Hz)                                — 疑点1: SEG_NFRAME 实测 vs 纸面
  E2: E0 + PREF 20Hz (电机禁能, 纯命令负载)                   — 疑点2: SEG_PREF 单条真实成本
  E3: 电机使能, PREF 斜坡 10°/s, CMD:ON 关                   — 疑点3: ISR 暴涨复现+五段状态
  E4: 全组合 (CMD:ON + PREF 20Hz + 转动)                     — 原始场景复现对照

C4: 每档落盘 PDBBIN 原始三元组 (seq/tick/host_rx_time) + LOOP_PROF 快照 JSON。

用法: python scripts/loop_prof_exp.py COM10 --power-ok [--each 30] [--out dir]
"""
import argparse
import json
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foclink  # noqa: E402  MixedStreamParser / fetch_loop_prof


class RawTripletCapture:
    """PDBBIN 原始三元组 (seq, tick, host_rx_time) — C4 落盘用。"""

    def __init__(self, ser):
        self.ser = ser
        self.parser = foclink.MixedStreamParser(pdb2_cb=self._on)
        self.triplets = []
        self._stop = False
        self._thread = None

    def _on(self, s: foclink.PdbBinSample):
        self.triplets.append((s.seq, s.tick_2khz, round(s.host_rx_time, 6)))

    def start_reader(self):
        import threading
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop:
            try:
                n = self.ser.in_waiting
                if n:
                    self.parser.feed(self.ser.read(n))
                else:
                    time.sleep(0.001)
            except Exception:
                break

    def stop_reader(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=2.0)

    def stats(self):
        st = self.parser.stats[foclink.TYPE_PDB2]
        return {"rx": st.rx, "crc_err": st.crc_err, "seq_gap": st.seq_gap}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--each", type=float, default=30.0, help="每档时长 s")
    ap.add_argument("--out", default="", help="输出目录 (默认为脚本目录)")
    ap.add_argument("--no-pref", action="store_true", help="跳过 E2 (PREF 禁能负载)")
    args = ap.parse_args()

    if not args.power_ok:
        print("DRY-RUN: 加 --power-ok 上台架")
        return 0

    outdir = args.out or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(outdir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")

    ser = serial.Serial(args.port, 1000000, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def send(cmd, pause=0.05):
        ser.write((cmd + "\n").encode())
        time.sleep(pause)

    def expect(cmd, prefix, timeout=1.5):
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode())
        dl = time.time() + timeout
        while time.time() < dl:
            if ser.in_waiting:
                for l in ser.read(ser.in_waiting).decode(errors="replace").split("\n"):
                    if l.strip().startswith(prefix):
                        return True
            else:
                time.sleep(0.01)
        return False

    def abort(msg):
        print("ABORT: %s" % msg)
        for c in ("CMD:STOP", "CMD:PDBBIN,0", "CMD:OFF", "TELEM:CUR,OFF"):
            send(c)
        ser.close()
        sys.exit(1)

    # 预检 (C2 精神): 清场, 确认 JDIAG 身份 + 无故障 (完整清单在 speed_sweep;
    # 本实验只探主循环耗时, 配置保持定版即可, 参数复核在 speed_sweep 前已做)
    send("CMD:OFF"); send("TELEM:CUR,OFF"); send("CMD:POSDBG,0"); send("CMD:PDBBIN,0")
    send("CMD:STOP"); send("CMD:CLEAR_FAULT")
    time.sleep(0.5)

    status = ser.read(ser.in_waiting if ser.in_waiting else 4096).decode(errors="replace")
    if "JDIAG" not in status:
        jbuf = b""
        ser.reset_input_buffer()
        ser.write(b"CMD:JDIAG\n")
        dl = time.time() + 3.0
        while time.time() < dl:
            if ser.in_waiting:
                jbuf += ser.read(ser.in_waiting)
                if b"JDIAG," in jbuf:
                    break
            else:
                time.sleep(0.02)
        status = jbuf.decode(errors="replace")
    print("preflight JDIAG:", status.split("JDIAG,")[1][:80].strip() if "JDIAG," in status else "MISSING")

    cap = RawTripletCapture(ser)
    runs = []

    def snap(name):
        # C4 关键: 取 LOOP_PROF 快照前必须彻底停 PDBBIN (残留帧字节会撞碎文本行)
        send("CMD:PDBBIN,0")
        time.sleep(0.3)
        # 排空残留 (二进制帧可跨块残在 FIFO; 不排空则 BEGIN 行被污染)
        dl = time.time() + 1.0
        while time.time() < dl and ser.in_waiting:
            ser.read(ser.in_waiting)
            time.sleep(0.05)
        lp = foclink.fetch_loop_prof(ser, timeout=3.0)
        # BUSY (上档事务机残留): 重试一次; 仍 BUSY 则记录诊断
        if lp["raw"] and lp["raw"][0].startswith("LOOP_PROF,BUSY"):
            ser.reset_input_buffer()
            send("CMD:LOOP_PROF,CLEAR")
            time.sleep(0.3)
            dl = time.time() + 1.0
            while time.time() < dl and ser.in_waiting:
                ser.read(ser.in_waiting)
                time.sleep(0.05)
            lp = foclink.fetch_loop_prof(ser, timeout=3.0)
        st = cap.stats()
        rec = {"name": name, "loop_prof": lp, "pdbbin": st}
        runs.append(rec)
        print("  [%s] iter_n=%s iter_avg_us=%s | PDBBIN rx=%d gap=%d" %
              (name, lp["begin"].get("iter_n"), lp["begin"].get("iter_avg_us"),
               st["rx"], st["seq_gap"]))
        for k in ("SEG_CMD", "SEG_PREF", "SEG_PDB", "SEG_NFRAME", "SEG_OTHER"):
            if k in lp["probes"]:
                p = lp["probes"][k]
                print("    %-9s n=%6d avg=%7.2fus max=%7.2fus" %
                      (k, p["n"], p["avg_us"], p["max_us"]))
        return rec

    def run_phase(name, setup, gen=None):
        print("== %s ==" % name)
        # LOOP_PROF 清零后立即开始记账 (FOC_TIME,CLEAR 同通道, 已含 SEG 统计)
        ser.reset_input_buffer()
        ser.write(b"CMD:FOC_TIME,CLEAR\n")
        time.sleep(0.2)
        send("CMD:LOOP_PROF,CLEAR")
        time.sleep(0.2)
        cap.triplets = []
        start = time.time()
        setup()
        cap.start_reader()
        t0 = time.time()
        if gen:
            last = 0.0
            while time.time() - t0 < args.each:
                t = time.time() - t0
                gen(t)
                time.sleep(0.001)
        else:
            time.sleep(args.each)
        cap.stop_reader()
        send("CMD:STOP")
        time.sleep(0.3)
        rec = snap(name)
        rec["triplets"] = cap.triplets
        return rec

    # E0: 基线 (PDB 全速)
    run_phase("E0", lambda: (send("CMD:PDBBIN,1"), time.sleep(0.3)))

    # E1: +CMD:ON (N 帧)
    run_phase("E1", lambda: (send("CMD:PDBBIN,1"), send("CMD:ON"), time.sleep(0.3)))

    # E2: +PREF 20Hz (禁能纯命令负载)
    if not args.no_pref:
        phase2 = [0.0]
        def pref_gen(t):
            if t - phase2[0] >= 0.05:
                phase2[0] = t
                ser.write(b"CMD:PREF,0.0\n")
        run_phase("E2", lambda: (send("CMD:PDBBIN,1"), time.sleep(0.3)), pref_gen)

    # E3: 使能 + PREF 斜坡 10°/s (CMD:ON 关)
    send("CMD:OFF")
    send("CMD:MODE,2")  # 直接位置模式 (与 speed_sweep 相同); expect 被流挤, 用全读确认
    time.sleep(0.3)
    send("CMD:ENABLE,1")
    time.sleep(0.8)
    send("CMD:POS_DIRECT,1")
    time.sleep(0.3)
    phase3 = [0.0, 0.0]
    def ramp_gen(t):
        # 10°/s 斜坡: 步长 0.5°@20Hz
        if t - phase3[0] >= 0.05:
            phase3[0] = t
            phase3[1] += 0.5
            ser.write(("CMD:PREF,%.6f\n" % (phase3[1] * 0.017453292519943295)).encode())
    run_phase("E3", lambda: (send("CMD:PDBBIN,1"), time.sleep(0.3)), ramp_gen)

    # E4: 全组合 (CMD:ON + PREF 20Hz + 转动)
    phase4 = [0.0, 0.0]
    def combo_gen(t):
        if t - phase4[0] >= 0.05:
            phase4[0] = t
            phase4[1] += 0.5
            ser.write(("CMD:PREF,%.6f\n" % (phase4[1] * 0.017453292519943295)).encode())
    run_phase("E4", lambda: (send("CMD:PDBBIN,1"), send("CMD:ON"), time.sleep(0.3)), combo_gen)

    # 清理
    for c in ("CMD:STOP", "CMD:PDBBIN,0", "CMD:OFF", "TELEM:CUR,OFF"):
        send(c)
    ser.close()

    out = os.path.join(outdir, "loop_prof_exp_%s.json" % ts)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"runs": runs}, f, ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
