#!/usr/bin/env python3
"""verify_settle_angles2.py — 稳态漂移角度矩阵 E1 (2026-09-01 Kimi 判别矩阵)

零烧录, 全在现有命令面内。PDBBIN 位置线 (200Hz, 无陈旧帧问题), 每点 30s 八点:
0/30/75/110/120/165/200/240°。落 PDBBIN 原始三元组 + 统计。

核心: settle 结束 (最后一次 PREF 后 ~10s 的 PDBBIN 行) 显式判定:
  - 到达目标 (|settle_end - target| < 1°) AND 后 20s 无漂移 -> CLEAN
  - settle_end 未到目标 (>1°) -> STUCK (拒动)
  - settle_end 到目标但后 20s 漂移 -> DRIFT

用法: python scripts/verify_settle_angles2.py COM10 --power-ok [--angles 0,30,110,120,165,200,240]
"""
import argparse
import json
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'common'))
import foclink  # noqa: E402

DEG2RAD = 3.14159265358979 / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM10")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--power-ok", action="store_true")
    ap.add_argument("--angles", default="0,30,75,110,120,165,200,240")
    ap.add_argument("--each", type=float, default=30.0, help="每点总时长 s (含 PREF 发送+settle+测量)")
    ap.add_argument("--settle-frac", type=float, default=0.35, help="settle 占总时长比例 (前段用于收敛)")
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
    for c in (b"CMD:OFF\n", b"TELEM:CUR,OFF\n", b"CMD:POSDBG,0\n", b"CMD:STOP\n", b"CMD:CLEAR_FAULT\n"):
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

    # 预检: JDIAG 身份 + LUT (fail-closed)
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
        print("JDIAG: %s" % jline[:120])
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

    # 配置 (定版默认; 全部用 expect 等 OK 响应 — 2026-09-01 验证此序列可靠)
    ser.write(b"CMD:UNLOCK,1\n")
    time.sleep(0.15)
    ser.reset_input_buffer()
    if not expect("CMD:POS_DIRECT,1", "POS_DIRECT,OK"):
        print("POS_DIRECT fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_GAIN,%.4f,%.4f" % (args.kp, args.kd), "POS_DIRECT_GAIN,OK"):
        print("GAIN fail"); ser.close(); return 1
    if not expect("CMD:POS_DIRECT_KI,%.2f" % args.ki, "POS_DIRECT_KI,OK"):
        print("KI fail"); ser.close(); return 1
    ser.write(b"CMD:COG_CFG,0.0,60.0\n")
    time.sleep(0.15)
    if not expect("CMD:FRIC_COMP,0.022,0.022", "FRIC_COMP,OK"):
        print("FRIC fail"); ser.close(); return 1
    if not expect("CMD:POS_AW_MODE,1,0.03", "POS_AW_MODE,OK"):
        print("AW fail"); ser.close(); return 1
    if not expect("CMD:MODE,2", "MODE,OK"):
        print("MODE fail"); ser.close(); return 1
    time.sleep(0.3)
    # ENABLE 可能因 fault/状态短暂拒绝 — 重试 3 次并打印实际响应
    en_ok = False
    for attempt in range(3):
        if expect("CMD:ENABLE,1", "ENABLE,OK", timeout=2.0):
            en_ok = True
            break
        print("ENABLE 重试 %d/3" % (attempt + 1), flush=True)
        send("CMD:CLEAR_FAULT")
        time.sleep(0.8)
    if not en_ok:
        print("ENABLE fail (重试3次)"); ser.close(); return 1

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

    # PDBBIN 流开启 (CMD:OFF 保持 N 帧关, PDBBIN 全速 200Hz; 模态/使能已就绪)
    cap = Cap()
    ser.write(b"CMD:PDBBIN,1\n")
    time.sleep(0.3)
    cap.start()

    results = []
    try:
        for a in angles:
            row_start = len(cap.rows)
            target_rad = a * DEG2RAD
            ser.write(b"CMD:PREF,%.6f\n" % target_rad)
            t0 = time.time()
            while time.time() - t0 < args.each:
                time.sleep(0.05)
            row_end = len(cap.rows)
            rows = cap.rows[row_start:row_end]
            if not rows:
                print("  angle %7.1f: PDBBIN 无数据" % a, flush=True)
                continue
            # settle_end = 第 settle_frac*each 秒的位置 (收敛后); 角度解环绕 (PDBBIN theta 在 ±180)
            settle_idx = int(len(rows) * args.settle_frac)
            rows_stable = rows[settle_idx:]
            settle_end_deg = rows_stable[0][2] * 180.0 / 3.14159265358979
            # 用完整轨迹解环绕: 目标角按连续角基准
            angs_raw = [r[2] * 180.0 / 3.14159265358979 for r in rows_stable]
            # unwrap: 从 target 出发比较 (简单: 最近等效角)
            def near360(x, tgt):
                while x - tgt > 180.0: x -= 360.0
                while x - tgt < -180.0: x += 360.0
                return x
            angs = [near360(x, a) for x in angs_raw]
            settle_end = angs[0]
            pp_deg = max(angs) - min(angs)
            # 拒动: settle_end 未到目标
            err_deg = abs(settle_end - a)
            if err_deg > 1.0:
                cls = "STUCK"
            elif pp_deg >= 3.0:
                cls = "DRIFT"
            elif pp_deg <= 0.1:
                cls = "CLEAN"
            else:
                cls = "MID"
            # 每点 iq_cmd 全程 max/min/mean (E4 用)
            iqs = [r[4] for r in rows]
            print("  angle %7.1f: settle_end=%7.2f err=%5.2f pp(后20s)=%6.3f iq_min/max/mean=%5.3f/%5.3f/%5.3f -> %s"
                  % (a, settle_end, err_deg, pp_deg, min(iqs), max(iqs), sum(iqs) / len(iqs), cls), flush=True)
            results.append({"angle": a, "settle_end": round(settle_end, 3),
                            "err_deg": round(err_deg, 3), "pp_deg": round(pp_deg, 4),
                            "iq_min": round(min(iqs), 4), "iq_max": round(max(iqs), 4),
                            "iq_mean": round(sum(iqs) / len(iqs), 4), "cls": cls,
                            "n_rows": len(rows), "settle_frac": args.settle_frac,
                            "traj": [(r[0], r[2], r[3], r[4], r[5]) for r in rows]})
    finally:
        cap.stop()
        ser.write(b"CMD:STOP")
        ser.write(b"CMD:PDBBIN,0")
        ser.write(b"CMD:OFF")
        ser.write(b"CMD:CLEAR_FAULT")
        ser.close()

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "settle_mat_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"args": vars(args), "results": results}, f, ensure_ascii=False, indent=1)
    print("JSON: %s" % out)
    cls_cnt = {}
    for r in results:
        cls_cnt[r["cls"]] = cls_cnt.get(r["cls"], 0) + 1
    print("汇总: %s" % cls_cnt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
