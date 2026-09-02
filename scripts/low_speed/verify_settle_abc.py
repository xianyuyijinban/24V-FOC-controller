#!/usr/bin/env python3
"""verify_settle_abc.py — E2/E3 稳态漂移 A/B (2026-09-01 Kimi 判别矩阵)

E2: AW 三模 A/B (POS_AW_MODE 0=条件冻结旧律 / 1=超阈值回拉定版 / 3=非对称泄放)
    在问题位测稳态 pp — 三模切换直接改变现象 -> 回拉律实锤
E3: COG ON/OFF A/B (COG_CFG gain 1.0/0) 同位置 — 排除 LUT 残差注入

点位: 240.69° (09-01 6.24° 漂移发生点) + 165.0° (历史拒动嫌疑位, E1 已证伪)

用法: python scripts/verify_settle_abc.py COM10 --power-ok [--angles 240.69,165.0]
"""
import argparse
import json
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
import foclink  # noqa: E402

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--angles", default="240.69,165.0")
    ap.add_argument("--settle", type=float, default=5.5)
    ap.add_argument("--win", type=float, default=2.0)
    ap.add_argument("--kp", type=float, default=0.49)
    ap.add_argument("--kd", type=float, default=0.007)
    ap.add_argument("--ki", type=float, default=0.37)
    args = ap.parse_args()
    if not args.power_ok:
        print("DRY-RUN: need --power-ok")
        return 0
    angles = [float(a) for a in args.angles.split(",") if a.strip()]

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

    # 配置序列 (与 E1 同)
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

    def near360(x, tgt):
        while x - tgt > 180.0: x -= 360.0
        while x - tgt < -180.0: x += 360.0
        return x

    def measure(a_deg):
        """钉住 a_deg, 等 settle, 测 win 窗 pp (PDBBIN, 每次只取窗内新行)。"""
        target_rad = a_deg * DEG2RAD
        ser.write(b"CMD:PREF,%.6f\n" % target_rad)
        time.sleep(args.settle)
        row_base = len(cap.rows)   # settle 后标记; 窗口只收其后新行
        t0 = time.time()
        angs, iqs, errs, ffs = [], [], [], []
        consumed = row_base
        while time.time() - t0 < args.win:
            if len(cap.rows) > consumed:
                for r in cap.rows[consumed:]:
                    angs.append(near360(r[2] * 180.0 / math.pi, a_deg))
                    iqs.append(r[4])
                    errs.append(r[3] * 180.0 / math.pi)
                    ffs.append(r[5])
                consumed = len(cap.rows)
            time.sleep(0.02)
        if not angs:
            return None
        return {"pp": max(angs) - min(angs),
                "end": angs[-1],
                "iq_min": min(iqs), "iq_max": max(iqs), "iq_mean": sum(iqs) / len(iqs),
                "err_end": errs[-1], "ff_end": ffs[-1], "n": len(angs)}

    # E2 AW 三模 + E3 COG A/B 合并跑
    configs = [
        ("E2_AW1_基线", "CMD:POS_AW_MODE,1,0.03"),
        ("E2_AW0_条件冻结", "CMD:POS_AW_MODE,0,0.03"),
        ("E2_AW3_非对称泄放", "CMD:POS_AW_MODE,3,0.03"),
        ("E3_COG_OFF", "CMD:COG_CFG,0.0,60.0"),
        ("E3_COG_ON", "CMD:COG_CFG,1.0,60.0"),   # 最后测 COG ON (结束时恢复 COG_OFF)
    ]
    results = []
    try:
        for label, cfg_cmd in configs:
            send(cfg_cmd, 0.2)
            for a in angles:
                m = measure(a)
                if m is None:
                    print("  %s angle %.1f: 采样不足" % (label, a), flush=True)
                    continue
                cls = "DRIFT" if m["pp"] >= 3.0 else ("CLEAN" if m["pp"] <= 0.1 else "MID")
                print("  %s angle %.1f: pp=%6.3f end=%7.2f iq[%5.3f,%5.3f] mean=%5.3f err_end=%6.3f ff_end=%6.3f -> %s"
                      % (label, a, m["pp"], m["end"], m["iq_min"], m["iq_max"], m["iq_mean"],
                         m["err_end"], m["ff_end"], cls), flush=True)
                results.append({"config": label, "angle": a, "pp": round(m["pp"], 4),
                                "end": round(m["end"], 3),
                                "iq_min": round(m["iq_min"], 4), "iq_max": round(m["iq_max"], 4),
                                "iq_mean": round(m["iq_mean"], 4),
                                "err_end": round(m["err_end"], 4), "ff_end": round(m["ff_end"], 4),
                                "cls": cls, "n": m["n"]})
    finally:
        send("CMD:COG_CFG,0.0,60.0")   # 恢复定版 COG OFF
        time.sleep(0.2)
        cap.stop()
        send("CMD:STOP")
        send("CMD:PDBBIN,0")
        send("CMD:OFF")
        send("CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "settle_abc_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
