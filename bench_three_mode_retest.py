"""
24V FOC Controller — 三模式复测（力矩/速度/位置）
=================================================
Follows: docs/bench_full_regression_current_baseline.md
COM6 @ 230400, 12V bench motor (24N22P, Pn=11)

Sections:
  A — 上电静态检查
  B — 参数识别（如已识别则跳过）
  C — 力矩模式安全短脉冲
  D — 速度模式回归
  E — 位置模式回归（重点）
  F — 最终故障与恢复
"""

import sys
import io
import time
import os
import math
import re
from datetime import datetime

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
sys.stderr = sys.stdout

os.makedirs("test_data", exist_ok=True)

# ── Serial setup ────────────────────────────────────────────────────────────
import serial
from serial.tools import list_ports

BAUD = 230400
PORT = "COM6"
PREFIX = datetime.now().strftime("retest_%Y%m%d_%H%M%S")

print(f"[SERIAL] Opening {PORT} @ {BAUD}")
s = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2.0)
s.reset_input_buffer()

# ── Globals ──────────────────────────────────────────────────────────────────
RESULTS = {}
FAILED_STAGE = None
START_TS = time.time()

# ── Helpers ──────────────────────────────────────────────────────────────────

def send(cmd):
    s.write((cmd + "\n").encode("utf-8"))
    print(f"  [TX] {cmd}")

def _safe_read():
    """Read available serial data, ignoring transient Windows permission errors."""
    try:
        n = s.in_waiting
        if n > 0:
            return s.read(n)
    except Exception:
        pass
    return b""

def drain():
    data = b""
    deadline = time.time() + 0.5
    while time.time() < deadline:
        chunk = _safe_read()
        if chunk:
            data += chunk
        else:
            time.sleep(0.01)
    return data

def collect(sec):
    data = b""
    deadline = time.time() + sec
    while time.time() < deadline:
        chunk = _safe_read()
        if chunk:
            data += chunk
        else:
            time.sleep(0.01)
    return data

def get_latest_n(raw_data=None):
    if raw_data is None:
        raw_data = drain()
    text = raw_data.decode("utf-8", errors="replace")
    for line in reversed(text.split("\n")):
        line = line.strip()
        if line.startswith("N,"):
            return line.split(",")
    return None

def save_csv(name, raw_data):
    lines = raw_data.decode("utf-8", errors="replace").split("\n")
    path = f"test_data/{PREFIX}_{name}.csv"
    with open(path, "w", encoding="utf-8") as f:
        f.write("ts,foc,angle,spd,Vbus,Id,Iq,Id_ref,Iq_ref,spd_ref,pos_ref,Vd,Vq\n")
        count = 0
        for l in lines:
            if l.startswith("N,"):
                p = l.split(",")
                if len(p) < 22:
                    continue
                try:
                    f.write(
                        f"{p[1]},{p[2]},{p[3]},{p[4]},{p[7]},{p[5]},{p[6]},"
                        f"{p[16]},{p[19]},{p[17]},{p[18]},{p[20]},{p[21]}\n"
                    )
                    count += 1
                except (ValueError, IndexError):
                    pass
    print(f"  [CSV] {path} ({count} N-lines)")
    return path

def save_fault_detail(label, raw_data):
    path = f"test_data/{PREFIX}_{label}_fault_detail.txt"
    with open(path, "w", encoding="utf-8") as f:
        f.write(raw_data.decode("utf-8", errors="replace"))
    print(f"  [FD]  {path}")
    return path

def send_fault_detail():
    """Send FAULT_DETAIL command and return decoded text."""
    send("CMD:FAULT_DETAIL")
    time.sleep(0.5)
    data = collect(2)
    text = data.decode("utf-8", errors="replace")
    for line in text.split("\n"):
        line_s = line.strip()
        if any(
            k in line_s
            for k in [
                "Power",
                "CurrentDQ",
                "SpeedLoopDiag",
                "PositionLoopDiag",
                "PrefDiag",
                "AppFault",
                "FAULT1",
                "VGS2",
                "Latched",
                "ParamDiag",
                "ThetaDiag",
                "valid=",
                "State:",
                "Error:",
                "RsDiag",
                "PnDiag",
                "DirDiag",
                "enc_dir",
                "Rs=",
            ]
        ):
            print(f"    [DIAG] {line_s[:250]}")
    return text

def collect_with_fault_detail(sec):
    data = collect(sec)
    fd_text = send_fault_detail()
    return data, fd_text

def parse_position_diag(fault_detail_text):
    result = {}
    patterns = {
        "err": r"(?:positionLoopErr|err)\s*[=:]\s*([\-\d\.]+)",
        "pd_out": r"(?:positionLoopPdOut|pd_out)\s*[=:]\s*([\-\d\.]+)",
        "pd_sat": r"(?:pd_sat|positionLoopPdOutSat)\s*[=:]\s*(\d+)",
        "ramp_sat": r"(?:ramp_sat|positionLoopSpeedRampSat)\s*[=:]\s*(\d+)",
        "iq_pos_sat": r"(?:iq_pos_sat|positionLoopIqPosSat)\s*[=:]\s*(\d+)",
        "iq_neg_sat": r"(?:iq_neg_sat|positionLoopIqNegSat)\s*[=:]\s*(\d+)",
        "friction": r"(?:pos_friction|positionFriction)\s*[=:]\s*([\-\d\.]+)",
        "iq_cmd": r"(?:pos_iq_cmd|positionIqCmd)\s*[=:]\s*([\-\d\.]+)",
    }
    for key, pat in patterns.items():
        m = re.search(pat, fault_detail_text)
        if m:
            try:
                if "sat" in key:
                    result[key] = int(m.group(1))
                else:
                    result[key] = float(m.group(1))
            except ValueError:
                pass
    return result

def parse_pref_diag(fault_detail_text):
    result = {}
    patterns = {
        "pref_count": r"(?:prefCmdCount|PrefDiag\.count)\s*[=:]\s*(\d+)",
        "pref_raw": r"(?:prefRaw|PrefDiag\.raw)\s*[=:]\s*([\-\d\.]+)",
        "pref_mapped": r"(?:prefMapped|PrefDiag\.mapped)\s*[=:]\s*([\-\d\.]+)",
        "pref_after": r"(?:prefAfter|PrefDiag\.after)\s*[=:]\s*([\-\d\.]+)",
    }
    for key, pat in patterns.items():
        m = re.search(pat, fault_detail_text)
        if m:
            try:
                result[key] = float(m.group(1))
            except ValueError:
                try:
                    result[key] = int(m.group(1))
                except ValueError:
                    pass
    return result

def check_runaway(packets_text, threshold=5.0):
    max_speed = 0.0
    for line in packets_text.split("\n"):
        if line.startswith("N,"):
            p = line.split(",")
            if len(p) > 4:
                try:
                    spd = abs(float(p[4]))
                    if spd > max_speed:
                        max_speed = spd
                except ValueError:
                    pass
    runaway = max_speed > threshold
    if runaway:
        print(f"  [WARN] Runaway: max_speed={max_speed:.2f} rad/s (threshold={threshold})")
    if max_speed > 50.0:
        print(f"  [CRITICAL] ~90 rad/s class runaway: {max_speed:.2f}")
    return max_speed, runaway

def abort_stage(stage_name, reason):
    global FAILED_STAGE
    FAILED_STAGE = stage_name
    print(f"\n{'='*60}")
    print(f"[ABORT] {stage_name}: {reason}")
    print(f"{'='*60}")

def record_result(stage, key, value):
    if stage not in RESULTS:
        RESULTS[stage] = {}
    RESULTS[stage][key] = value

def stat_n(raw_data):
    text = raw_data.decode("utf-8", errors="replace")
    spds, iqs, vbs = [], [], []
    foc4 = 0
    max_iq = 0
    for line in text.split("\n"):
        if line.startswith("N,"):
            p = line.split(",")
            if len(p) < 8:
                continue
            try:
                spd = float(p[4])
                iq = float(p[6])
                spds.append(spd)
                iqs.append(iq)
                vbs.append(float(p[7]))
                if abs(iq) > abs(max_iq):
                    max_iq = iq
                if p[2] == "4":
                    foc4 += 1
            except (ValueError, IndexError):
                pass
    n = len(spds)
    return {
        "n_packets": n,
        "avg_speed": sum(spds) / n if n else 0,
        "max_speed": max(spds) if spds else 0,
        "min_speed": min(spds) if spds else 0,
        "avg_Iq": sum(iqs) / n if n else 0,
        "max_Iq_abs": abs(max_iq),
        "avg_Vbus": sum(vbs) / n if n else 0,
        "foc4_count": foc4,
    }

def ensure_state_ready():
    """Clear fault and unlock to get to READY/IDLE state."""
    send("CMD:ENABLE,0")
    time.sleep(0.3)
    send("CMD:CLEAR_FAULT")
    time.sleep(0.3)
    s.reset_input_buffer()

def ensure_unlocked_ready():
    """Get board to UNLOCKED READY state for operations."""
    ensure_state_ready()
    send("CMD:UNLOCK,1")
    time.sleep(0.2)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION A — 上电静态检查
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print(f"SECTION A — 上电静态检查")
print(f"  Prefix: {PREFIX}")
print(f"  Time:   {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
print("=" * 60)

time.sleep(1.0)
data_a1 = collect(3)
n_a1 = data_a1.decode("utf-8", errors="replace").count("N,")
print(f"  A1: {n_a1} N-packets in 3s")

if n_a1 == 0:
    abort_stage("A1", "无 N 包 — 检查 COM 口 / 供电 / 固件")
else:
    latest = get_latest_n(data_a1)
    if latest and len(latest) > 9:
        try:
            vbus = float(latest[7])
            encoder = int(latest[9])
            motor_identified = int(latest[10]) if len(latest) > 10 else 0
            Id = float(latest[5]) if len(latest) > 5 else 0
            Iq = float(latest[6]) if len(latest) > 6 else 0
            foc_state = int(latest[2])
            print(f"  Vbus={vbus:.2f}V, encoder={encoder}, motor_identified={motor_identified}")
            print(f"  foc_state={foc_state}, Id={Id:.4f}A, Iq={Iq:.4f}A")
            record_result("A_static", "vbus", round(vbus, 3))
            record_result("A_static", "encoder", encoder)
            record_result("A_static", "motor_identified", motor_identified)
            record_result("A_static", "n_packets_3s", n_a1)
            record_result("A_static", "foc_state", foc_state)
            if vbus < 10 or vbus > 14:
                print(f"  [WARN] Vbus 不在 12V 范围: {vbus:.2f}V")
            if encoder != 1:
                abort_stage("A1", "编码器离线 — 不能进入闭环测试")
        except (ValueError, IndexError) as e:
            print(f"  [WARN] Parse error: {e}")

print()
print("  A2: FAULT_DETAIL 静态检查")
fd_a2 = send_fault_detail()

# Check for encoder_dir in fault detail
enc_dir_match = re.search(r"enc_dir\s*[=:]\s*(-?\d+)", fd_a2)
if enc_dir_match:
    enc_dir = int(enc_dir_match.group(1))
    print(f"  encoder_dir = {enc_dir}")
    record_result("A_static", "encoder_dir", enc_dir)

# Also check for Rs value
rs_match = re.search(r"Rs[=:]\s*([\d\.]+)", fd_a2)
if rs_match:
    print(f"  Rs = {rs_match.group(1)} Ohm")
    record_result("A_static", "Rs_ohm", float(rs_match.group(1)))

if FAILED_STAGE:
    print(f"\n[ABORTED at {FAILED_STAGE}]")
    s.close()
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION B — 参数识别
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print("SECTION B — 参数识别")
print("=" * 60)

# Check if motor is already identified
latest_b0 = get_latest_n()
motor_already_identified = False
if latest_b0 and len(latest_b0) > 10:
    motor_already_identified = int(latest_b0[10]) == 1

if motor_already_identified:
    print("  [SKIP] 电机已识别 (motor_identified=1)，跳过识别阶段")
    record_result("B_identify", "status", "already_identified")
else:
    s.reset_input_buffer()
    send("CMD:VBUS_LIMIT,9.000,16.000")
    send("CMD:MOTOR_PN,11")
    send("CMD:CLEAR_FAULT")
    time.sleep(0.3)
    s.reset_input_buffer()
    send("CMD:UNLOCK,1")
    time.sleep(0.2)
    send("CMD:IDENTIFY,1")

    ident_ok = False
    identify_log = b""

    for i in range(150):
        time.sleep(1)
        chunk = s.read(4000)
        identify_log += chunk
        text = chunk.decode("utf-8", errors="replace")
        for line in text.split("\n"):
            if line.startswith("N,"):
                p = line.split(",")
                if len(p) > 10 and p[10] == "1":
                    ident_ok = True
                    print(f"  [识别成功] {i+1}s")
                    break
                if len(p) > 28 and p[2] == "2" and (i + 1) % 15 == 0:
                    print(
                        f"  ...{i+1}s identify_state={p[25]} identify_error={p[26]} angle={p[3]} Vbus={p[7]}"
                    )
            elif "MI_ERR" in line or "FAULT DETECTED" in line:
                print(f"  [FAULT] {line.strip()}")
        if ident_ok:
            break
        if (i + 1) % 30 == 0:
            send("CMD:FAULT_DETAIL")
            time.sleep(0.2)
            identify_log += s.read(6000)
        if (i + 1) % 15 == 0 and not ident_ok:
            print(f"  ...等待中 ({i+1}s)")

    time.sleep(1)
    identify_log += collect(1)

    if not ident_ok:
        send("CMD:FAULT_DETAIL")
        time.sleep(0.5)
        identify_log += s.read(12000)
        save_fault_detail("identify", identify_log)
        save_csv("identify", identify_log)
        abort_stage("B", "识别未完成 — 保存 FAULT_DETAIL，中止后续闭环测试")
        print(f"\n[ABORTED at {FAILED_STAGE}]")
        s.close()
        sys.exit(1)

    s.reset_input_buffer()
    save_csv("identify", identify_log)
    save_fault_detail("identify", identify_log)
    record_result("B_identify", "status", "completed")

latest_b = get_latest_n()
if latest_b and len(latest_b) > 10:
    try:
        record_result("B_identify", "foc", int(latest_b[2]))
        record_result("B_identify", "identified", int(latest_b[10]))
        record_result("B_identify", "angle", round(float(latest_b[3]), 2))
        print(f"  识别后: foc={latest_b[2]}, ident={latest_b[10]}, angle={latest_b[3]}")
    except (ValueError, IndexError):
        pass

# ══════════════════════════════════════════════════════════════════════════════
# SECTION C — 力矩模式安全短脉冲
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print("SECTION C — 力矩模式安全短脉冲 (MODE=0)")
print("=" * 60)

ensure_unlocked_ready()
send("CMD:MODE,0")
time.sleep(0.3)
s.reset_input_buffer()

# C1: Positive pulse (user +0.1A)
print("\n  C1: 力矩脉冲 +0.1A (短脉冲 ~0.4s)")
send("CMD:ENABLE,1")
time.sleep(0.1)
data_c1a = drain()
send("CMD:IREF,0.000,0.100")
data_c1a += collect(0.5)
send("CMD:IREF,0.000,0.000")
time.sleep(0.1)
data_c1a += drain()
send("CMD:ENABLE,0")
time.sleep(0.3)
data_c1a += drain()
s.reset_input_buffer()

save_csv("torque_pulse_p0.1A", data_c1a)
st_c1a = stat_n(data_c1a)
max_spd_c1a, runaway_c1a = check_runaway(data_c1a.decode("utf-8", errors="replace"))
print(f"    n={st_c1a['n_packets']}, max_spd={max_spd_c1a:.3f}rad/s, avg_Iq={st_c1a['avg_Iq']:.4f}A, Iq_max_abs={st_c1a['max_Iq_abs']:.4f}A")
record_result("C_torque_+0.1A", "max_speed", round(max_spd_c1a, 3))
record_result("C_torque_+0.1A", "avg_Iq", round(st_c1a["avg_Iq"], 4))
record_result("C_torque_+0.1A", "max_Iq_abs", round(st_c1a["max_Iq_abs"], 4))

if runaway_c1a:
    print(f"    [STOP] 力矩脉冲飞车 speed={max_spd_c1a:.2f}，跳过后续力矩测试")
    abort_stage("C_torque_+0.1A", f"飞车 speed={max_spd_c1a:.2f}")
    FAILED_STAGE = None  # Continue to speed/position

# C2: Negative pulse (user -0.1A)
if not FAILED_STAGE or FAILED_STAGE == "C_torque_+0.1A":
    if FAILED_STAGE == "C_torque_+0.1A":
        FAILED_STAGE = None
    print("\n  C2: 力矩脉冲 -0.1A (短脉冲 ~0.4s)")
    send("CMD:ENABLE,1")
    time.sleep(0.1)
    data_c1b = drain()
    send("CMD:IREF,0.000,-0.100")
    data_c1b += collect(0.5)
    send("CMD:IREF,0.000,0.000")
    time.sleep(0.1)
    data_c1b += drain()
    send("CMD:ENABLE,0")
    time.sleep(0.3)
    data_c1b += drain()
    s.reset_input_buffer()

    save_csv("torque_pulse_m0.1A", data_c1b)
    st_c1b = stat_n(data_c1b)
    max_spd_c1b, runaway_c1b = check_runaway(data_c1b.decode("utf-8", errors="replace"))
    print(f"    n={st_c1b['n_packets']}, max_spd={max_spd_c1b:.3f}rad/s, avg_Iq={st_c1b['avg_Iq']:.4f}A, Iq_max_abs={st_c1b['max_Iq_abs']:.4f}A")
    record_result("C_torque_-0.1A", "max_speed", round(max_spd_c1b, 3))
    record_result("C_torque_-0.1A", "avg_Iq", round(st_c1b["avg_Iq"], 4))
    record_result("C_torque_-0.1A", "max_Iq_abs", round(st_c1b["max_Iq_abs"], 4))

    if runaway_c1b:
        abort_stage("C_torque_-0.1A", f"飞车 speed={max_spd_c1b:.2f}")
        FAILED_STAGE = None

# Cleanup
ensure_state_ready()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION D — 速度模式回归
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print("SECTION D — 速度模式回归 (MODE=1)")
print("=" * 60)

ensure_unlocked_ready()
send("CMD:MODE,1")
time.sleep(0.3)
s.reset_input_buffer()

speed_tests = [
    ("SREF,0.100", "speed_p0.1"),
    ("SREF,0.200", "speed_p0.2"),
    ("SREF,-0.100", "speed_m0.1"),
    ("SREF,-0.200", "speed_m0.2"),
    ("SREF,0.000", "speed_0"),
]

for cmd, csv_label in speed_tests:
    if FAILED_STAGE:
        break
    print(f"\n  D: {cmd}")
    send(f"CMD:{cmd}")
    time.sleep(0.1)
    send("CMD:ENABLE,1")
    data = collect(10)
    send("CMD:ENABLE,0")
    time.sleep(0.5)
    s.reset_input_buffer()

    save_csv(csv_label, data)
    st = stat_n(data)
    max_speed, runaway = check_runaway(data.decode("utf-8", errors="replace"))
    print(
        f"    n={st['n_packets']}, avg_spd={st['avg_speed']:.4f}, "
        f"max_spd={max_speed:.3f}, min_spd={st['min_speed']:.3f}, "
        f"avg_Iq={st['avg_Iq']:.4f}, foc4={st['foc4_count']}"
    )

    if runaway:
        abort_stage(f"D_{csv_label}", f"飞车 speed={max_speed:.2f}")
    record_result(f"D_{csv_label}", "avg_speed", round(st["avg_speed"], 4))
    record_result(f"D_{csv_label}", "max_speed", round(max_speed, 3))
    record_result(f"D_{csv_label}", "min_speed", round(st["min_speed"], 3))
    record_result(f"D_{csv_label}", "avg_Iq", round(st["avg_Iq"], 4))
    record_result(f"D_{csv_label}", "foc4_count", st["foc4_count"])

if FAILED_STAGE:
    print(f"\n[ABORTED at {FAILED_STAGE}]")
    s.close()
    sys.exit(1)

# Get SpeedLoopDiag
print("\n  D-diag: SpeedLoopDiag")
fd_d = send_fault_detail()
ensure_state_ready()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION E — 位置模式回归
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print("SECTION E — 位置模式回归 (MODE=2)")
print("=" * 60)

ensure_unlocked_ready()
send("CMD:MODE,2")
time.sleep(0.3)
s.reset_input_buffer()

# E0 — Hold
print("\n  E0: 位置保持 (10s)")
send("CMD:ENABLE,1")
data_e0 = collect(10)

# Parse hold angle
hold_angle_deg = 0.0
text_e0 = data_e0.decode("utf-8", errors="replace")
for line in text_e0.split("\n"):
    if line.startswith("N,"):
        p = line.split(",")
        if len(p) > 3 and p[2] == "4":
            try:
                hold_angle_deg = float(p[3])
                break
            except ValueError:
                pass
print(f"    Hold angle H = {hold_angle_deg:.2f} deg")
record_result("E0_hold", "angle", round(hold_angle_deg, 2))

st_e0 = stat_n(data_e0)
print(f"    n={st_e0['n_packets']}, avg_spd={st_e0['avg_speed']:.4f}")
record_result("E0_hold", "avg_speed", round(st_e0["avg_speed"], 4))
record_result("E0_hold", "max_speed", round(st_e0["max_speed"], 3))

fd_e0_text = send_fault_detail()
pos_diag_e0 = parse_position_diag(fd_e0_text)
pref_diag_e0 = parse_pref_diag(fd_e0_text)
print(f"    PositionLoopDiag: {pos_diag_e0}")
print(f"    PrefDiag: {pref_diag_e0}")

save_csv("pos_hold", data_e0)
save_fault_detail("pos_hold", fd_e0_text.encode("utf-8"))

# Check hold status
hold_sat_pd = pos_diag_e0.get("pd_sat", 0)
hold_sat_iq_neg = pos_diag_e0.get("iq_neg_sat", 0)
if hold_sat_iq_neg:
    print(f"    [WARN] Hold 中 sat=iq-:{hold_sat_iq_neg}")

# E1 — +5 deg step
print(f"\n  E1: 小步进 +5 deg ({hold_angle_deg:.1f} -> {hold_angle_deg + 5:.1f})")
h_rad = math.radians(hold_angle_deg)
s5_rad = math.radians(5)

target_rad = h_rad + s5_rad
print(f"    PREF +5deg = {target_rad:.6f} rad ({math.degrees(target_rad):.1f} deg)")
send(f"CMD:PREF,{target_rad:.6f}")
data_e1a, fd_e1a_text = collect_with_fault_detail(10)
save_csv("pos_step_p5deg", data_e1a)
save_fault_detail("pos_step_p5deg", fd_e1a_text.encode("utf-8"))

pos_diag_e1a = parse_position_diag(fd_e1a_text)
pref_diag_e1a = parse_pref_diag(fd_e1a_text)
max_spd_e1a, runaway_e1a = check_runaway(data_e1a.decode("utf-8", errors="replace"))
print(f"    PositionLoopDiag: {pos_diag_e1a}")
print(f"    PrefDiag: {pref_diag_e1a}")
print(f"    max_spd={max_spd_e1a:.3f}")
record_result("E1_p5deg", "max_speed", round(max_spd_e1a, 3))
record_result("E1_p5deg", "pos_diag", {k: v for k, v in pos_diag_e1a.items() if v != 0})
record_result("E1_p5deg", "pref_count", pre_fd := pref_diag_e1a.get("pref_count", "N/A"))

if runaway_e1a:
    abort_stage("E1_p5deg", f"飞车 speed={max_spd_e1a:.2f}")
elif max_spd_e1a > 50:
    abort_stage("E1_p5deg", f"严重飞车: {max_spd_e1a:.2f}")

# E1b — Return to H (-5 deg step)
if not FAILED_STAGE:
    print(f"\n  E1b: 回到 H ({h_rad:.6f} rad, ~{hold_angle_deg:.1f} deg)")
    send(f"CMD:PREF,{h_rad:.6f}")
    data_e1b, fd_e1b_text = collect_with_fault_detail(10)
    save_csv("pos_step_m5deg", data_e1b)
    save_fault_detail("pos_step_m5deg", fd_e1b_text.encode("utf-8"))

    pos_diag_e1b = parse_position_diag(fd_e1b_text)
    pref_diag_e1b = parse_pref_diag(fd_e1b_text)
    max_spd_e1b, runaway_e1b = check_runaway(data_e1b.decode("utf-8", errors="replace"))
    print(f"    PositionLoopDiag: {pos_diag_e1b}")
    print(f"    PrefDiag: {pref_diag_e1b}")
    print(f"    max_spd={max_spd_e1b:.3f}")
    record_result("E1_m5deg", "max_speed", round(max_spd_e1b, 3))
    record_result("E1_m5deg", "pos_diag", {k: v for k, v in pos_diag_e1b.items() if v != 0})

    if runaway_e1b:
        abort_stage("E1_m5deg", f"飞车 speed={max_spd_e1b:.2f}")
    elif max_spd_e1b > 50:
        abort_stage("E1_m5deg", f"严重飞车: {max_spd_e1b:.2f}")

if FAILED_STAGE:
    send("CMD:ENABLE,0")
    print(f"\n[ABORTED at {FAILED_STAGE}]")
    s.close()
    sys.exit(1)

# E2 — Absolute targets
print("\n  E2: 绝对位置目标")
abs_targets = [
    (0.000000, "0deg", "pos_abs_0deg"),
    (0.785398, "45deg", "pos_abs_45deg"),
    (1.570796, "90deg", "pos_abs_90deg"),
    (3.141593, "180deg", "pos_abs_180deg"),
]

pref_counts = []
for target, label, csv_label in abs_targets:
    if FAILED_STAGE:
        break
    print(f"    PREF {label} = {target:.6f} rad")
    send(f"CMD:PREF,{target:.6f}")
    data_e2, fd_e2_text = collect_with_fault_detail(10)
    save_csv(csv_label, data_e2)
    pos_diag = parse_position_diag(fd_e2_text)
    pref_diag = parse_pref_diag(fd_e2_text)
    max_spd, runaway = check_runaway(data_e2.decode("utf-8", errors="replace"))
    pc = pref_diag.get("pref_count", "?")
    pref_counts.append(pc)
    print(f"      PrefDiag.count={pc}, max_spd={max_spd:.3f}, PositionLoopDiag: {pos_diag}")
    record_result(f"E2_{label}", "max_speed", round(max_spd, 3))
    record_result(f"E2_{label}", "pref_count", pc)
    if pos_diag.get("iq_neg_sat"):
        print(f"      [WARN] sat=iq-:{pos_diag['iq_neg_sat']}")

    if runaway:
        abort_stage(f"E2_{label}", f"飞车 speed={max_spd:.2f}")
    elif max_spd > 50:
        abort_stage(f"E2_{label}", f"严重飞车: {max_spd:.2f}")

# E3 — Cross 0 deg
if not FAILED_STAGE:
    print("\n  E3: 跨 0 度 — 350 -> 10 deg")
    send("CMD:PREF,6.108652")
    data_e3a = collect(8)
    save_csv("pos_cross0_350deg", data_e3a)

    send("CMD:PREF,0.174533")
    data_e3b, fd_e3b_text = collect_with_fault_detail(10)
    save_csv("pos_cross0_10deg", data_e3b)

    max_spd_x, runaway_x = check_runaway(data_e3b.decode("utf-8", errors="replace"))
    pos_diag_x = parse_position_diag(fd_e3b_text)
    pref_diag_x = parse_pref_diag(fd_e3b_text)
    print(f"    Cross-0: PrefDiag.count={pref_diag_x.get('pref_count','?')}, max_spd={max_spd_x:.3f}")
    record_result("E3_cross0", "max_speed", round(max_spd_x, 3))
    record_result("E3_cross0", "pref_count", pref_diag_x.get("pref_count", "N/A"))

    if max_spd_x > 50:
        abort_stage("E3_cross0", f"严重飞车: {max_spd_x:.2f}")

send("CMD:ENABLE,0")
time.sleep(0.3)
drain()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION F — 最终故障与恢复
# ══════════════════════════════════════════════════════════════════════════════
print()
print("=" * 60)
print("SECTION F — 最终故障检查与恢复")
print("=" * 60)

send("CMD:ENABLE,0")
time.sleep(0.5)

fd_f_text = send_fault_detail()
save_fault_detail("final", fd_f_text.encode("utf-8"))

send("CMD:CLEAR_FAULT")
time.sleep(0.5)
collect(1)

# Summary
elapsed = time.time() - START_TS
print()
print("=" * 60)
print(f"三模式复测结果汇总  [{datetime.now().strftime('%H:%M:%S')} / {elapsed:.0f}s]")
print("=" * 60)

# Determine pass/fail for each section
def fmt_stage(name, data):
    """Format result line."""
    if isinstance(data, dict):
        parts = []
        for k, v in data.items():
            if isinstance(v, float):
                parts.append(f"{k}={v:.3f}")
            elif isinstance(v, dict):
                for dk, dv in v.items():
                    parts.append(f"{dk}={dv}")
            else:
                parts.append(f"{v}")
        return ", ".join(parts)[:80]
    return str(data)[:80]

stages_order = [
    "A_static",
    "B_identify",
    "C_torque_+0.1A",
    "C_torque_-0.1A",
    "D_speed_p0.1",
    "D_speed_p0.2",
    "D_speed_m0.1",
    "D_speed_m0.2",
    "E0_hold",
    "E1_p5deg",
    "E1_m5deg",
    "E2_0deg",
    "E2_45deg",
    "E2_90deg",
    "E2_180deg",
    "E3_cross0",
]

print(f"\n{'阶段':<25} {'关键数据'}")
print("-" * 90)
for stage in stages_order:
    data = RESULTS.get(stage, {})
    if data:
        print(f"{stage:<25} {fmt_stage(stage, data)}")
    else:
        print(f"{stage:<25} [NO DATA]")

# Overall assessment
print("-" * 90)
if FAILED_STAGE:
    print(f"\n  ABORTED at {FAILED_STAGE}")
else:
    # Check key criteria
    warnings = []
    # Check speed mode for runaway
    for spd_stage in ["D_speed_p0.1", "D_speed_p0.2", "D_speed_m0.1", "D_speed_m0.2"]:
        if spd_stage in RESULTS:
            ms = RESULTS[spd_stage].get("max_speed", 0)
            if ms > 5.0:
                warnings.append(f"{spd_stage}: max_spd={ms:.2f} > 5.0")
    # Check position for runaway
    for pos_stage in ["E1_p5deg", "E1_m5deg", "E2_0deg", "E2_45deg", "E2_90deg", "E2_180deg", "E3_cross0"]:
        if pos_stage in RESULTS:
            ms = RESULTS[pos_stage].get("max_speed", 0)
            if ms > 50:
                warnings.append(f"{pos_stage}: max_spd={ms:.2f} CRITICAL")

    if warnings:
        print(f"\n  WARNINGS ({len(warnings)}):")
        for w in warnings:
            print(f"    - {w}")
        print(f"\n  三模式复测: PASS with warnings")
    else:
        print(f"\n  三模式复测: ALL PASS")

print(f"\n  数据文件: test_data/{PREFIX}_*")
print(f"  CSV 目录: test_data/")

s.close()
print("\n=== THREE-MODE RETEST COMPLETE ===")
