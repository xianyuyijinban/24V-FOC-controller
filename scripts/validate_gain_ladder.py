#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""validate_gain_ladder.py - offline fail-closed validator for S2 JSON.

The validator accepts only schema v2 runs with complete identity, per-round
health, a measured stable-gate span, and a replayable t95 event. TIMEOUT is a
recoverable measurement outcome and is reported as WARN; an explicitly
invalid run or any health/identity violation is a failure.

Usage: python validate_gain_ladder.py <json> [--strict]
Return codes: 0 = valid (possibly with WARN), 1 = validation failure,
2 = input error.

Note: 本机 GBK 控制台中文输出乱码 (codepage 问题, 不影响逻辑)。
      跑前 set PYTHONIOENCODING=utf-8 或 chcp 65001 (2026-09-06 Kimi 记录)。
"""
import json
import math
import sys


DEG2RAD = 0.017453292519943295
STEP_DEG = 6.0
T95_PCT = 0.05
T95_WINDOW_DEG = STEP_DEG * T95_PCT
T95_DWELL = 0.5
LEAVE_ERR_DEG = 0.1
GATE_WINDOW = 2.0
GATE_PP_DEG = 0.1
GATE_PP_HALF_LSB = 0.0054   # 半 LSB (编码器 0.0107°/LSB) — gate_pp 压线带
# rate 阈值 mode-aware (2026-09-06 Kimi 裁决): N 帧共存挤占 P1, 实测 ~21-24Hz;
#   N 共存 ≥18Hz (对最差 20.9 留 15% 裕量), 纯流 ≥150Hz。机制与 8/31 TX 泵专项一致。
PDB_MIN_RATE_HZ_COEXIST = 18.0
PDB_MIN_RATE_HZ_PURE = 150.0
MIN_PDB_N = 100
MIN_GATE_FRAMES = 5


def _rate_floor(meta):
    """mode-aware rate 下限: meta.n_coexist=true → 18Hz, 否则 150Hz。"""
    return PDB_MIN_RATE_HZ_COEXIST if (meta.get("n_coexist") is True) else PDB_MIN_RATE_HZ_PURE


def _number(value):
    """Return a finite float, or None for a missing/malformed value."""
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _has_fields(mapping, keys):
    return (isinstance(mapping, dict) and
            all(mapping.get(key) not in (None, "") for key in keys))


def _ack_present(value, prefix):
    values = value if isinstance(value, list) else [value]
    return any(isinstance(item, str) and item.startswith(prefix) for item in values)


def _dt_state(line):
    """Parse DT,OK,0 and DT,OK,en=0 forms; return bool or None."""
    if not isinstance(line, str) or not line.startswith("DT,OK"):
        return None
    fields = {}
    parts = line.split(",")
    for field in parts:
        if "=" in field:
            key, value = field.split("=", 1)
            fields[key] = value
    value = fields.get("en")
    if value is None and len(parts) > 2:
        value = parts[2].strip()
    if value not in ("0", "1"):
        return None
    return value == "1"


def _trajectory(result):
    """Decode the full (host_rx, error_deg, iq_cmd) trajectory."""
    raw = result.get("poserr_traj_full")
    if not isinstance(raw, list) or not raw:
        return []
    decoded = []
    for sample in raw:
        if not isinstance(sample, (list, tuple)) or len(sample) < 2:
            return []
        host_rx = _number(sample[0])
        error_deg = _number(sample[1])
        if host_rx is None or error_deg is None:
            return []
        decoded.append((host_rx, error_deg))
    return decoded


def _validate_meta(doc, errors):
    if doc.get("schema") != "s2_gain_ladder.v2":
        errors.append("V1 FAIL: schema 不是 s2_gain_ladder.v2")

    run_status = doc.get("run_status")
    if not isinstance(run_status, dict) or run_status.get("valid") is not True:
        errors.append("V1 FAIL: run_status.valid 不是 true (run invalid 或缺失)")

    meta = doc.get("meta")
    if not isinstance(meta, dict):
        errors.append("V1 FAIL: meta 缺失")
        return

    fw = meta.get("fw_info")
    if not _has_fields(fw, ("version", "param", "baseline")):
        errors.append("V1 FAIL: meta.fw_info 缺 version/param/baseline")
    if not isinstance(meta.get("fw_raw"), str) or not meta["fw_raw"].startswith("FW_INFO,OK,"):
        errors.append("V1 FAIL: meta.fw_raw 缺 FW_INFO,OK 身份")

    jdiag = meta.get("jdiag")
    required_jdiag = ("J", "enc", "valid", "cog_valid", "cog_save",
                      "cog_gain", "cog_phase", "cog_min", "cog_max")
    if not _has_fields(jdiag, required_jdiag):
        errors.append("V1 FAIL: meta.jdiag 身份字段不完整")
    if not isinstance(meta.get("jdiag_raw"), str) or not meta["jdiag_raw"].startswith("JDIAG,"):
        errors.append("V1 FAIL: meta.jdiag_raw 缺 JDIAG 身份")

    ch_cfg = meta.get("ch_cfg")
    if not _has_fields(ch_cfg, ("gain_c", "recon")):
        errors.append("V1 FAIL: meta.ch_cfg 缺 gain_c/recon")
    if not isinstance(meta.get("ch_raw"), str) or not meta["ch_raw"].startswith("CH_CFG,OK,"):
        errors.append("V1 FAIL: meta.ch_raw 缺 CH_CFG,OK 身份")

    if meta.get("dt_cmd_sent") is not True:
        errors.append("V1 FAIL: dt_cmd_sent 不是 true (DT,0 未记录)")
    if meta.get("dt_enabled") is not False:
        errors.append("V1 FAIL: dt_enabled 不是 false (恢复规划要求 DT OFF)")
    if _dt_state(meta.get("dt_ack")) is not False:
        errors.append("V1 FAIL: dt_ack 未确认 DT disabled")
    if _dt_state(meta.get("dt_status")) is not False:
        errors.append("V1 FAIL: dt_status 未确认 en=0")

    config_ack = meta.get("config_ack")
    required_acks = {
        "UNLOCK": "UNLOCK,OK",
        "POS_DIRECT": "POS_DIRECT,OK",
        "COG_CFG": "COG_CFG,OK",
        "FRIC_COMP": "FRIC_COMP,OK",
        "POS_AW_MODE": "POS_AW_MODE,OK",
        "MODE": "MODE,OK",
        "ENABLE": "ENABLE,OK",
    }
    if not isinstance(config_ack, dict):
        errors.append("V1 FAIL: meta.config_ack 缺失")
    else:
        for name, prefix in required_acks.items():
            if not _ack_present(config_ack.get(name), prefix):
                errors.append("V1 FAIL: meta.config_ack 缺 %s" % name)


def _validate_health(result, label, errors, rate_floor=PDB_MIN_RATE_HZ_PURE):
    health = result.get("health")
    if not isinstance(health, dict):
        errors.append("V4 FAIL: %s 缺 health 摘要" % label)
        return None

    required = ("pdb_n", "seq_gap", "tick_stall", "bad_state", "bad_fault",
                "crc_err", "sample_rate_hz")
    missing = [key for key in required if key not in health]
    if missing:
        errors.append("V4 FAIL: %s health 缺 %s" % (label, "/".join(missing)))
        return None

    for key in ("pdb_n", "seq_gap", "tick_stall", "bad_state", "bad_fault", "crc_err"):
        value = _number(health.get(key))
        if value is None:
            errors.append("V4 FAIL: %s health.%s 非数值" % (label, key))
        elif key == "seq_gap":
            # 容忍前提 = 归因材料 (2026-09-06 Kimi 补救规格): gap>0 必须带 loci
            if value != 0:
                loci = health.get("seq_gap_loci")
                if not isinstance(loci, list) or len(loci) != int(value):
                    errors.append("V4 FAIL: %s seq_gap=%d 但 seq_gap_loci 缺失/长度"
                                  "不符 — 无归因不容忍" % (label, value))
        elif key != "pdb_n" and value != 0:
            errors.append("V4 FAIL: %s health.%s=%s" % (label, key, health[key]))

    pdb_n = _number(health.get("pdb_n"))
    if pdb_n is None or pdb_n < MIN_PDB_N:
        errors.append("V6 FAIL: %s health.pdb_n=%s 流静默/断开" %
                      (label, health.get("pdb_n")))

    sample_rate = _number(health.get("sample_rate_hz"))
    if sample_rate is None or sample_rate < rate_floor:
        errors.append("V4 FAIL: %s sample_rate_hz=%s < %.1fHz (mode-aware)" %
                      (label, health.get("sample_rate_hz"), rate_floor))
    return sample_rate


def _validate_gate(result, label, errors, warns=None):
    if result.get("gate_ok") is not True:
        errors.append("V5 FAIL: %s gate_ok 不是 true" % label)
    for key in ("gate_wait_s", "gate_span_s", "gate_frames", "gate_pp_deg"):
        if _number(result.get(key)) is None:
            errors.append("V5 FAIL: %s 缺 %s" % (label, key))
    span = _number(result.get("gate_span_s"))
    if span is not None and span < GATE_WINDOW:
        errors.append("V5 FAIL: %s gate_span_s=%.3fs < %.1fs" %
                      (label, span, GATE_WINDOW))
    frames = _number(result.get("gate_frames"))
    if frames is not None and frames < MIN_GATE_FRAMES:
        errors.append("V5 FAIL: %s gate_frames=%d < %d" %
                      (label, frames, MIN_GATE_FRAMES))
    gate_pp = _number(result.get("gate_pp_deg"))
    if gate_pp is not None:
        # borderline 带 (2026-09-06 Kimi 裁决): [gate_pp, gate_pp+半LSB) → WARN。
        # 编码器 0.0107°/LSB, 半 LSB=0.0054°; pp 恰好压线 (0.1000) 是量化巧合
        # 非 0.1+ 违规; >= 上限仍 FAIL
        if gate_pp >= GATE_PP_DEG + GATE_PP_HALF_LSB:
            errors.append("V5 FAIL: %s gate_pp_deg=%.6f >= %.2f+半LSB" %
                          (label, gate_pp, GATE_PP_DEG))
        elif gate_pp >= GATE_PP_DEG:
            if warns is not None:
                warns.append("V5: %s gate_pp_deg=%.6f 压线 [%.2f, %.4f) 半LSB — 量化巧合"
                             % (label, gate_pp, GATE_PP_DEG,
                                GATE_PP_DEG + GATE_PP_HALF_LSB))


def _validate_round_config(result, label, errors):
    acks = result.get("config_ack")
    if not isinstance(acks, dict):
        errors.append("V1 FAIL: %s 缺 config_ack" % label)
        return
    if not _ack_present(acks.get("POS_DIRECT_GAIN"), "POS_DIRECT_GAIN,OK"):
        errors.append("V1 FAIL: %s 缺 POS_DIRECT_GAIN,OK" % label)
    if not _ack_present(acks.get("POS_DIRECT_KI"), "POS_DIRECT_KI,OK"):
        errors.append("V1 FAIL: %s 缺 POS_DIRECT_KI,OK" % label)


def _validate_t95(result, label, sample_rate, errors, warns):
    t95 = _number(result.get("t95_s"))
    if t95 is None:
        warns.append("V2/V3: %s 没有 t95 (该轮数据可追溯但未达到 t95)" % label)
        return

    event_keys = ("t_cmd_host_rx", "t_leave_host_rx",
                  "t95_window_enter_host_rx", "t95_host_rx")
    event = {key: _number(result.get(key)) for key in event_keys}
    if any(value is None for value in event.values()):
        errors.append("V2/V3 FAIL: %s t95 事件绝对时间戳不完整" % label)
        return

    cmd_rx = event["t_cmd_host_rx"]
    leave_rx = event["t_leave_host_rx"]
    enter_rx = event["t95_window_enter_host_rx"]
    t95_rx = event["t95_host_rx"]
    if not (cmd_rx <= leave_rx <= enter_rx <= t95_rx):
        errors.append("V2/V3 FAIL: %s t95 事件顺序非法" % label)
    if t95_rx - enter_rx < T95_DWELL:
        errors.append("V3 FAIL: %s t95 窗保持 %.3fs < %.1fs" %
                      (label, t95_rx - enter_rx, T95_DWELL))

    leave_s = _number(result.get("t_leave_s"))
    enter_s = _number(result.get("t95_window_enter_s"))
    if leave_s is None or enter_s is None:
        errors.append("V2/V3 FAIL: %s 缺 t_leave_s/t95_window_enter_s" % label)
    else:
        if not (0.0 <= leave_s <= enter_s <= t95):
            errors.append("V2/V3 FAIL: %s t95 相对事件顺序非法" % label)
        if t95 - enter_s < T95_DWELL:
            errors.append("V3 FAIL: %s t95 窗相对保持 %.3fs < %.1fs" %
                          (label, t95 - enter_s, T95_DWELL))

    trajectory = _trajectory(result)
    if not trajectory:
        errors.append("V2/V3 FAIL: %s 缺完整 poserr_traj_full" % label)
        return

    after_command = [abs(error) for host_rx, error in trajectory if host_rx >= cmd_rx]
    if not after_command or max(after_command) < LEAVE_ERR_DEG:
        errors.append("V2 FAIL: %s t95=%.2fs 但阶跃后未离开初始窗 (max|err|=%.3f°)" %
                      (label, t95, max(after_command) if after_command else 0.0))

    dwell_frames = [(host_rx, error) for host_rx, error in trajectory
                    if enter_rx - 1e-6 <= host_rx <= t95_rx + 1e-6]
    if not dwell_frames:
        errors.append("V3 FAIL: %s t95 窗期间没有轨迹帧" % label)
    else:
        if any(abs(error) > T95_WINDOW_DEG + 1e-6 for _, error in dwell_frames):
            errors.append("V3 FAIL: %s t95 窗期间存在 |err|>%.2f° 帧" %
                          (label, T95_WINDOW_DEG))
        dwell_s = max(0.0, t95_rx - enter_rx)
        rate_ref = sample_rate or PDB_MIN_RATE_HZ_COEXIST
        expected = max(MIN_GATE_FRAMES,
                       int(math.ceil(dwell_s * max(rate_ref, PDB_MIN_RATE_HZ_COEXIST) * 0.5)))
        if len(dwell_frames) < expected:
            errors.append("V3 FAIL: %s t95 窗帧数=%d < 预计下限%d" %
                          (label, len(dwell_frames), expected))


def _validate_verify_meta(doc, errors):
    """verify_low_speed.v2 的 meta 专用检查 (2026-09-06 schema 分支化)。"""
    if doc.get("schema") != "verify_low_speed.v2":
        errors.append("V1 FAIL: schema 不是 verify_low_speed.v2")
    run_status = doc.get("run_status")
    if not isinstance(run_status, dict) or run_status.get("valid") is not True:
        errors.append("V1 FAIL: run_status.valid 不是 true")
    meta = doc.get("meta")
    if not isinstance(meta, dict):
        errors.append("V1 FAIL: meta 缺失")
        return None
    fw = meta.get("fw_info")
    if not _has_fields(fw, ("version", "param", "baseline")):
        errors.append("V1 FAIL: meta.fw_info 缺 version/param/baseline")
    if not isinstance(meta.get("fw_raw"), str) or not meta["fw_raw"].startswith("FW_INFO,OK,"):
        errors.append("V1 FAIL: meta.fw_raw 缺 FW_INFO,OK 身份")
    jdiag = meta.get("jdiag")
    if not _has_fields(jdiag, ("J", "enc", "valid", "cog_gain", "cog_phase")):
        errors.append("V1 FAIL: meta.jdiag 身份字段不完整")
    if not isinstance(meta.get("jdiag_raw"), str) or not meta["jdiag_raw"].startswith("JDIAG,"):
        errors.append("V1 FAIL: meta.jdiag_raw 缺 JDIAG 身份")
    ch_cfg = meta.get("ch_cfg")
    if not _has_fields(ch_cfg, ("gain_c", "recon")):
        errors.append("V1 FAIL: meta.ch_cfg 缺 gain_c/recon")
    if not isinstance(meta.get("ch_raw"), str) or not meta["ch_raw"].startswith("CH_CFG,OK,"):
        errors.append("V1 FAIL: meta.ch_raw 缺 CH_CFG,OK 身份")
    if meta.get("dt_cmd_sent") is not True:
        errors.append("V1 FAIL: dt_cmd_sent 不是 true (DT,0 未记录)")
    if meta.get("dt_enabled") is not False:
        errors.append("V1 FAIL: dt_enabled 不是 false (恢复规划要求 DT OFF)")
    # config_ack: verify 用自身命令集 (UNLOCK/POS_DIRECT/POS_DIRECT_GAIN/POS_DIRECT_KI/
    #   FRIC_COMP/POS_AW_MODE/MODE/ENABLE) — 与 ladder 的 required_acks 不同
    config_ack = meta.get("config_ack")
    if not isinstance(config_ack, dict):
        errors.append("V1 FAIL: meta.config_ack 缺失")
    return meta


def _validate_verify(doc, errors, warns):
    """verify_low_speed.v2: results={reps:[...]}, 每 rep 校验 health + gate + 关键字段。
    与 ladder 不同的校验集 — verify 无 t95 事件/gain config, 但有 ramp/step/steady。"""
    run_status = doc.get("run_status")
    if not isinstance(run_status, dict) or run_status.get("valid") is not True:
        errors.append("V1 FAIL: verify run_status.valid 不是 true")
    meta = doc.get("meta") or {}
    rate_floor = _rate_floor(meta)
    results = doc.get("results")
    reps = results.get("reps") if isinstance(results, dict) else None
    if not isinstance(reps, list) or not reps:
        errors.append("V1 FAIL: verify results.reps 缺失或为空")
        return 1
    for index, rep in enumerate(reps):
        if not isinstance(rep, dict):
            errors.append("V4 FAIL: verify rep[%d] 不是对象" % index)
            continue
        label = "verify rep%d" % (index + 1)
        # health 摘要 (同 ladder 标准)
        h = rep.get("health")
        if not isinstance(h, dict):
            errors.append("V4 FAIL: %s 缺 health" % label)
        else:
            for key in ("pdb_n", "seq_gap", "tick_stall", "bad_state",
                        "bad_fault", "crc_err", "sample_rate_hz"):
                if _number(h.get(key)) is None:
                    errors.append("V4 FAIL: %s health.%s 缺失/非数值" % (label, key))
            # seq_gap 阈值: N 共存容忍 ≤2 帧 (F1 N 帧仲裁投影候选, 8/31 TX 泵专项);
            # 容忍前提 = gap_loci 位置归因材料齐全 (2026-09-06 Kimi 补救规格) —
            # loci 缺失/长度不符 = 无归因 = 直接 fail
            gap_tol = 2 if meta.get("n_coexist") is True else 0
            for key in ("seq_gap", "tick_stall", "bad_state", "bad_fault", "crc_err"):
                val = _number(h.get(key))
                if val is None:
                    errors.append("V4 FAIL: %s health.%s 缺失/非数值" % (label, key))
                elif key == "seq_gap":
                    if val > gap_tol:
                        errors.append("V4 FAIL: %s health.seq_gap=%s > 容忍%d (N共存)"
                                      % (label, h.get(key), gap_tol))
                    elif val > 0:
                        loci = h.get("seq_gap_loci")
                        if not isinstance(loci, list) or len(loci) != int(val):
                            errors.append("V4 FAIL: %s seq_gap=%s 但 seq_gap_loci "
                                          "缺失/长度不符 (%r) — 无归因不容忍"
                                          % (label, h.get(key), loci))
                elif val != 0:
                    errors.append("V4 FAIL: %s health.%s=%s" % (label, key, h.get(key)))
            pdb_n = _number(h.get("pdb_n"))
            if pdb_n is None or pdb_n < MIN_PDB_N:
                errors.append("V6 FAIL: %s health.pdb_n=%s 流静默/断开" %
                              (label, h.get("pdb_n")))
            rate = _number(h.get("sample_rate_hz"))
            if rate is None or rate < rate_floor:
                errors.append("V4 FAIL: %s sample_rate_hz=%s < %.1fHz (mode-aware)"
                              % (label, h.get("sample_rate_hz"), rate_floor))
        # ramp/step/steady 必需
        for key in ("ramp", "step", "steady"):
            if key not in rep:
                errors.append("V4 FAIL: %s 缺 %s" % (label, key))
    return 1 if errors else 0


def validate(path, strict=False):
    errors = []
    warns = []
    try:
        with open(path, encoding="utf-8") as handle:
            doc = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        print("VALIDATOR Fatal: %s" % exc)
        return 2

    if not isinstance(doc, dict):
        print("VALIDATOR Fatal: JSON 根节点不是对象")
        return 2
    schema = doc.get("schema")
    # schema-aware 分支 (2026-09-06 Kimi 裁决): dispatch 先于 meta 检查 —
    # meta 检查按 schema 分支化, 否则 verify 文件被 ladder 标准打一轮假失败
    if schema == "verify_low_speed.v2":
        _validate_verify_meta(doc, errors)
        _validate_verify(doc, errors, warns)
    else:
        _validate_meta(doc, errors)
        _validate_ladder_results(doc, errors, warns)

    if errors:
        print("VALIDATOR FAIL (%d 条):" % len(errors))
        for error in errors:
            print("  " + error)
        if warns:
            print("WARN:")
            for warning in warns:
                print("  " + warning)
        return 1
    if warns:
        print("VALIDATOR OK (有 WARN):")
        for warning in warns:
            print("  " + warning)
        return 0
    print("VALIDATOR OK")
    return 0


def _validate_ladder_results(doc, errors, warns):
    results = doc.get("results")
    rate_floor = _rate_floor(doc.get("meta") or {})
    if not isinstance(results, list) or not results:
        errors.append("V1 FAIL: results 缺失或为空")
        return
    for index, result in enumerate(results):
        if not isinstance(result, dict):
            errors.append("V4 FAIL: round[%d] 不是对象" % index)
            continue
        label = "%s 轮%s" % (result.get("gain", "?"), result.get("round", "?"))
        if result.get("gate") == "TIMEOUT":
            warns.append("V4/V5: %s 稳定门超时 (记录为 WARN, 需要复跑)" % label)
            continue
        _validate_round_config(result, label, errors)
        sample_rate = _validate_health(result, label, errors, rate_floor)
        _validate_gate(result, label, errors, warns)
        reported_n = _number(result.get("pdb_n"))
        if reported_n is None or reported_n < MIN_PDB_N:
            errors.append("V6 FAIL: %s pdb_n=%s 流静默/断开" %
                          (label, result.get("pdb_n")))
        _validate_t95(result, label, sample_rate, errors, warns)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python validate_gain_ladder.py <json>")
        sys.exit(2)
    sys.exit(validate(sys.argv[1], "--strict" in sys.argv))
