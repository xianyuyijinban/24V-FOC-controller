#!/usr/bin/env python3
"""verify_drift_probe.py — 双态漂移收官判据: verify 序列连跑 4 轮 (2026-09-01 Kimi 任务卡收官)

复现 verify_low_speed 的完整稳态测量序列 (阶跃+回位+5s+测2s), 每轮 PDBBIN 全程落盘
(pos_err/iq_cmd/ff_total), 漂移态现形时抓状态字 (AW 积分/position_friction_active/
pos_cmd_dir), 干净轮也抓一份作对照。

序列 (与 verify_low_speed 同):
  1. 钉住当前角 a0 -> 阶跃 +6° (PREF a0+6) -> 3s -> 回位 (PREF a0) -> 等 5s
  2. 测 2s 稳态窗 (目标角 a0, 但 verify 实际钉的是 240.69° 锚点附近)
  3. 判定: pp>=3° 漂移态 / <=0.1° 干净 / else MID
  4. 漂移态 or 干净轮: 抓 JDIAG/状态字 (命令: CMD:POS_AW_MODE? 拿当前 mode;
     drift 发生时 PDBBIN 行已含 iq_cmd/ff_total; 状态字另行查询)

用法: python scripts/verify_drift_probe.py COM10 --power-ok [--rounds 4] [--step 6.0]
"""
import argparse
import json
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--step", type=float, default=6.0, help="阶跃幅度 °")
    ap.add_argument("--anneal", type=float, default=5.0, help="回位后等待 s")
    ap.add_argument("--win", type=float, default=2.0, help="稳态测量窗 s")
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.4)
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:PDBBIN,0\n",
              b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
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

    def expect(cmd, prefix, timeout=1.5):
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

    def send(cmd, wait=0.3):
        ser.write((cmd + "\n").encode())
        time.sleep(wait)

    # JDIAG 审计
    ser.reset_input_buffer()
    ser.write(b"CMD:JDIAG\n")
    dl = time.time() + 5.0
    jbuf = ""
    while time.time() < dl:
        if ser.in_waiting:
            jbuf += ser.read(ser.in_waiting).decode(errors="replace")
            if "JDIAG," in jbuf:
                break
        else:
            time.sleep(0.02)
    jline = next((l.strip() for l in jbuf.replace("\r", "").split("\n") if l.startswith("JDIAG,")), None)
    if jline:
        kv = dict(f.split("=", 1) for f in jline.split(",") if "=" in f)
        cmin, cmax = float(kv.get("cog_min", 0)), float(kv.get("cog_max", 0))
        print("JDIAG: %s" % jline[:110])
        if abs(cmin + 0.0093) < 0.002 and abs(cmax - 0.0133) < 0.002:
            print("  LUT: 编译平滑版 OK")
        else:
            print("  !! LUT 非编译版 (Flash 遮蔽?) 中止")
            ser.close()
            return 1
    else:
        print("JDIAG 无响应 — 中止")
        ser.close()
        return 1

    # 配置 (与 verify 同)
    send("CMD:UNLOCK,1", 0.15)
    ser.reset_input_buffer()
    if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
        print("POS_DIRECT fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK"):
        print("GAIN fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK"):
        print("KI fail"); ser.close(); return 1
    send("CMD:COG_CFG,0.0,60.0", 0.15)
    if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
        print("FRIC fail"); ser.close(); return 1
    if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
        print("AW fail"); ser.close(); return 1
    if not expect("CMD:MODE,2", "MODE,OK"):
        print("MODE fail"); ser.close(); return 1
    time.sleep(0.3)
    en_ok = False
    for attempt in range(3):
        if expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
            en_ok = True
            break
        send("CMD:CLEAR_FAULT", 0.8)
    if not en_ok:
        print("ENABLE fail (3次)"); ser.close(); return 1

    # PDBBIN capture
    rows = []

    def row(s):
        return (round(s.host_rx_time, 4), s.tick_2khz, s.theta_user_rad,
                s.pos_err_rad, s.iq_cmd, s.ff_total, s.v_mech_rad_s, s.iq_act, s.pos_ref_rad)

    class Cap:
        def __init__(self):
            self.rows = []
            self.parser = foclink.MixedStreamParser(pdb2_cb=self._on)

        def _on(self, s):
            self.rows.append(row(s))

        def start(self):
            import threading
            self._stop = False
            threading.Thread(target=self._loop, daemon=True).start()

        def _loop(self):
            while not self._stop:
                try:
                    n = ser.in_waiting
                    if n:
                        self.parser.feed(ser.read(n))
                    else:
                        time.sleep(0.001)
                except Exception:
                    break

        def stop(self):
            self._stop = True
            time.sleep(0.3)

    cap = Cap()
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    cap.start()

    def read_state_words():
        """抓状态字: CMD:DIR? —— dir/hold/integral/iq_cmd/fric_pos/fric_neg.
        DIR? 走 P0 队列, 会被 PDBBIN 淹没 — 必须: PDBBIN,0 等 0.5s 再 DIR? 抓完恢复。
        2026-09-01 实测: 流全停后 DIR? 正常 (dir/hold/integral/fric_pos_neg)。"""
        send("CMD:PDBBIN,0", 0.5)
        time.sleep(0.4)
        ser.reset_input_buffer()
        ser.write(b"CMD:DIR?\n")
        time.sleep(0.5)
        s = b""
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if ser.in_waiting:
                s += ser.read(ser.in_waiting)
            else:
                time.sleep(0.05)
        send("CMD:PDBBIN,1", 0.4)
        for l in s.decode(errors="replace").split("\n"):
            if l.strip().startswith("DIR,OK"):
                return l.strip()
        return None

    def near360(x, tgt):
        while x - tgt > 180.0: x -= 360.0
        while x - tgt < -180.0: x += 360.0
        return x

    def round_probe(idx, a0_deg):
        """单轮: 阶跃 +6° -> 回位 a0 -> 等 anneal -> 测 win 窗。返回 dict."""
        target_step = a0_deg + args.step
        # 移动
        send("CMD:PREF,%.6f" % (target_step * DEG2RAD), 3.0)
        time.sleep(3.0)
        # 回位
        send("CMD:PREF,%.6f" % (a0_deg * DEG2RAD), 0.3)
        time.sleep(args.anneal)
        # 稳态窗测量 (PDBBIN 新行)
        row_base = len(cap.rows)
        t0 = time.time()
        angs, iqs, errs, ffs = [], [], [], []
        consumed = row_base
        while time.time() - t0 < args.win:
            if len(cap.rows) > consumed:
                for r in cap.rows[consumed:]:
                    angs.append(near360(r[2] * 180.0 / math.pi, a0_deg))
                    iqs.append(r[4])
                    errs.append(r[3] * 180.0 / math.pi)
                    ffs.append(r[5])
                consumed = len(cap.rows)
            time.sleep(0.02)
        if not angs:
            return None
        pp = max(angs) - min(angs)
        cls = "DRIFT" if pp >= 3.0 else ("CLEAN" if pp <= 0.1 else "MID")
        st = read_state_words()
        return {"round": idx, "a0": a0_deg, "pp": pp, "end": angs[-1], "cls": cls,
                "iq_min": min(iqs), "iq_max": max(iqs), "iq_mean": sum(iqs) / len(iqs),
                "err_end": errs[-1], "ff_end": ffs[-1], "n": len(angs),
                "state": st, "traj": [(r[0], r[2], r[3], r[4], r[5]) for r in cap.rows[row_base:]]}

    # 初始钉到 240.69° 附近 (verify 历史漂移点)
    send("CMD:PREF,%.6f" % (240.69 * DEG2RAD), 4.0)
    time.sleep(4.0)
    a0 = None
    # 读当前角 (PDBBIN 最后一行)
    if cap.rows:
        a0 = near360(cap.rows[-1][2] * 180.0 / math.pi, 240.69)
    print("initial settle at %.2f°" % (a0 if a0 else -1), flush=True)

    results = []
    try:
        for i in range(args.rounds):
            r = round_probe(i, a0)
            if r is None:
                print("轮 %d: 采样不足" % i, flush=True)
                continue
            print("轮 %d: pp=%.3f end=%.2f iq[%.4f,%.4f] err_end=%.3f ff=%.4f n=%d %s state=%s"
                  % (i, r["pp"], r["end"], r["iq_min"], r["iq_max"], r["err_end"],
                     r["ff_end"], r["n"], r["cls"], r["state"]), flush=True)
            results.append(r)
    finally:
        cap.stop()
        send("CMD:STOP")
        send("CMD:PDBBIN,0")
        send("CMD:OFF")
        send("CMD:CLEAR_FAULT")
        ser.close()

    # 切片轨迹（防 JSON 过大）
    for r in results:
        if len(r["traj"]) > 1200:
            step = len(r["traj"]) // 200
            r["traj"] = r["traj"][::step]

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "drift_probe_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    drift = sum(1 for r in results if r["cls"] == "DRIFT")
    clean = sum(1 for r in results if r["cls"] == "CLEAN")
    print("4轮汇总: %d 漂移态 %d 干净态 %d MID" % (drift, clean, len(results) - drift - clean))
    return 0


if __name__ == "__main__":
    sys.exit(main())
