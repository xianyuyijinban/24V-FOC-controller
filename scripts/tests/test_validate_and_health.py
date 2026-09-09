#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synthetic fail-closed tests for validate_gain_ladder.py.

Each valid round contains the same identity, health, gate, ACK, and full t95
event fields that the live ladder writes. Invalid fixtures then remove or
corrupt one safety condition at a time.
"""
import copy
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import validate_gain_ladder  # noqa: E402


def valid_meta():
    return {
        "fw_info": {"version": "1.4.0", "param": "1", "baseline": "12V_STANDARD"},
        "fw_raw": "FW_INFO,OK,version=1.4.0,param=1,baseline=12V_STANDARD,git=test",
        "jdiag": {
            "J": "5.294e-5", "enc": "-1", "valid": "0x1",
            "cog_valid": "1", "cog_save": "1", "cog_gain": "0.000",
            "cog_phase": "60.0", "cog_min": "-0.0093", "cog_max": "0.0133",
        },
        "jdiag_raw": "JDIAG,v6,J=5.294e-5,enc=-1,valid=0x1,cog_valid=1,cog_save=1,"
                     "cog_gain=0.000,cog_phase=60.0,cog_min=-0.0093,cog_max=0.0133",
        "ch_cfg": {"gain_c": "1.0", "recon": "0"},
        "ch_raw": "CH_CFG,OK,gain_c=1.0,recon=0",
        "dt_enabled": False,
        "dt_cmd_sent": True,
        "dt_ack": "DT,OK,0",
        "dt_status": "DT,OK,en=0,amp_mV=0,comp=0",
        "n_coexist": True,   # 真实 ladder/verify: N 帧共存 (P1/P2 仲裁 ~21-24Hz)
        "config_ack": {
            # ack 必须存响应原文 (2026-09-07 实证: expect_retry 存布尔 →
            # FRIC_COMP=[True] 不构成身份证据); COG_CFG 无 OK 字样 (uart_upload.c:1743)
            "UNLOCK": ["UNLOCK,OK,1"],
            "POS_DIRECT": ["POS_DIRECT,OK,1"],
            "COG_CFG": ["COG_CFG,gain=0.000,phase_deg=60.0"],
            "FRIC_COMP": ["FRIC_COMP,OK"],
            "POS_AW_MODE": ["POS_AW_MODE,OK"],
            "MODE": ["MODE,OK"],
            "ENABLE": ["ENABLE,OK,1"],
        },
    }


def valid_trajectory(cmd=1000.0, enter=1002.0, t95=1002.5):
    frames = [(cmd - 0.1, 0.0, 0.0), (cmd + 0.01, 1.0, 0.02)]
    for index in range(51):
        frames.append((enter + index * 0.01, 0.1, 0.01))
    assert abs(frames[-1][0] - t95) < 1e-9
    return frames


def valid_round():
    cmd = 1000.0
    enter = 1002.0
    t95 = 1002.5
    return {
        "gain": "G2",
        "round": 0,
        "gate_ok": True,
        "gate_wait_s": 2.1,
        "gate_span_s": 2.0,
        "gate_frames": 100,
        "gate_pp_deg": 0.05,
        "config_ack": {
            "POS_DIRECT_GAIN": ["POS_DIRECT_GAIN,OK,kp=0.49,kd=0.007"],
            "POS_DIRECT_KI": ["POS_DIRECT_KI,OK,ki=0.37"],
        },
        "health": {
            "pdb_n": 1000,
            "seq_gap": 0,
            "tick_stall": 0,
            "bad_state": 0,
            "bad_fault": 0,
            "crc_err": 0,
            "sample_rate_hz": 100.0,   # N 共存实测 20-24, >18 floor
        },
        "t_cmd_host_rx": cmd,
        "t_leave_host_rx": 1001.0,
        "t95_window_enter_host_rx": enter,
        "t95_host_rx": t95,
        "t_leave_s": 1.0,
        "t95_window_enter_s": 2.0,
        "t95_s": 2.5,
        "poserr_traj_full": valid_trajectory(cmd, enter, t95),
        "pdb_n": 1000,
    }


def make_doc(results=None, meta=None, valid=True):
    return {
        "schema": "s2_gain_ladder.v2",
        "run_status": {"valid": valid, "abort_reason": None},
        "meta": copy.deepcopy(valid_meta() if meta is None else meta),
        "results": copy.deepcopy([valid_round()] if results is None else results),
    }


def write_doc(tmp, doc, name="synth.json"):
    path = os.path.join(tmp, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(doc, handle)
    return path


def assert_fail(tmp, doc, label):
    rc = validate_gain_ladder.validate(write_doc(tmp, doc, label + ".json"))
    assert rc != 0, "%s 未 fail-closed" % label


def test_missing_identity(tmp):
    doc = {"results": [valid_round()]}
    assert_fail(tmp, doc, "missing_identity")


def test_invalid_run_status(tmp):
    assert_fail(tmp, make_doc(valid=False), "invalid_run_status")


def test_bad_dt_and_ch_cfg(tmp):
    meta = valid_meta()
    meta["dt_ack"] = "DT,OK,1"
    meta["ch_raw"] = "CH_CFG,ERR"
    assert_fail(tmp, make_doc(meta=meta), "bad_dt_ch_cfg")


def test_bad_health(tmp):
    result = valid_round()
    result["health"]["bad_state"] = 1
    result["health"]["crc_err"] = 1
    assert_fail(tmp, make_doc([result]), "bad_health")


def test_short_gate(tmp):
    result = valid_round()
    result["gate_span_s"] = 0.1
    result["gate_frames"] = 5
    assert_fail(tmp, make_doc([result]), "short_gate")


def test_t95_without_leave(tmp):
    result = valid_round()
    result["poserr_traj_full"] = [(host, 0.05, 0.0)
                                   for host, _, _ in valid_trajectory()]
    assert_fail(tmp, make_doc([result]), "t95_without_leave")


def test_t95_dwell_insufficient(tmp):
    result = valid_round()
    result["t95_host_rx"] = 1002.2
    result["t95_s"] = 2.2
    result["poserr_traj_full"] = valid_trajectory(t95=1002.5)
    assert_fail(tmp, make_doc([result]), "t95_dwell_short")


def test_silent_stream(tmp):
    result = valid_round()
    result["health"]["pdb_n"] = 10
    result["pdb_n"] = 10
    assert_fail(tmp, make_doc([result]), "silent_stream")


def test_timeout_warns(tmp):
    timeout = {"gain": "G2", "round": 0, "gate": "TIMEOUT", "gate_wait_s": 15.0}
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc([timeout]), "timeout.json"))
    assert rc == 0, "TIMEOUT 应为 WARN，不应误杀"


def test_valid_round(tmp):
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc(), "valid.json"))
    assert rc == 0, "合规数据被拒 (rc=%s)" % rc


def test_gap_loci_mismatch(tmp):
    """r2 场景 (2026-09-06 Kimi): seq_gap>0 但 loci 缺失/长度不符 → 无归因不容忍"""
    result = valid_round()
    result["health"]["seq_gap"] = 1
    result["health"]["seq_gap_loci"] = []   # 计数 1 但 loci 空 — 不同源
    assert_fail(tmp, make_doc([result]), "gap_loci_missing")


def test_gap_loci_matched(tmp):
    """seq_gap=1 + loci 1 条同长 → 容忍生效 (N 共存)"""
    result = valid_round()
    result["health"]["seq_gap"] = 1
    result["health"]["seq_gap_loci"] = [[6213190, "G2-r1-gate"]]
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc([result]),
                                                "gap_loci_ok.json"))
    assert rc == 0, "带归因 loci 的单帧 gap 应容忍 (rc=%s)" % rc


def test_gate_pp_borderline_warns(tmp):
    """gate_pp 压线带 (半 LSB 0.0054): 0.1000 → WARN 不 FAIL; 0.1060 → FAIL"""
    result = valid_round()
    result["gate_pp_deg"] = 0.100000
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc([result]),
                                                "gate_pp_border.json"))
    assert rc == 0, "压线 0.1000 应 WARN 不 FAIL (rc=%s)" % rc
    result2 = valid_round()
    result2["gate_pp_deg"] = 0.106000   # >= 0.1+0.0054 → FAIL
    assert_fail(tmp, make_doc([result2]), "gate_pp_over")


def test_gap5_delta0_warns(tmp):
    """tx_p1_drop 三分支 (2026-09-09 Kimi A 规格): gap>2 + delta=0 →
    丢帧在主机侧 RX (固件 TX 零丢), 归因齐 → WARN 不 FAIL"""
    result = valid_round()
    result["health"]["seq_gap"] = 5
    result["health"]["seq_gap_loci"] = [[100, "r0-step"], [200, "r0-ramp"],
                                        [300, "r0-gate"], [400, "r1-step"],
                                        [500, "r1-ramp"]]
    result["health"]["tx_p1_drop_delta"] = 0
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc([result]),
                                                "gap5_delta0.json"))
    assert rc == 0, "gap5+delta0 应 WARN 不 FAIL (rc=%s)" % rc


def test_gap5_delta_pos_fails(tmp):
    """gap>2 + delta>0 → 固件 TX 真丢帧 → FAIL"""
    result = valid_round()
    result["health"]["seq_gap"] = 5
    result["health"]["seq_gap_loci"] = [[100, "r0-step"], [200, "r0-ramp"],
                                        [300, "r0-gate"], [400, "r1-step"],
                                        [500, "r1-ramp"]]
    result["health"]["tx_p1_drop_delta"] = 3
    assert_fail(tmp, make_doc([result]), "gap5_delta3")


def test_gap5_no_delta_fails(tmp):
    """gap>2 + delta 缺失 → 无归因证据 fail-closed → FAIL"""
    result = valid_round()
    result["health"]["seq_gap"] = 5
    result["health"]["seq_gap_loci"] = [[100, "r0-step"], [200, "r0-ramp"],
                                        [300, "r0-gate"], [400, "r1-step"],
                                        [500, "r1-ramp"]]
    # tx_p1_drop_delta 不写 = 缺失
    assert_fail(tmp, make_doc([result]), "gap5_no_delta")


def test_pdbbin_ver2_accepts(tmp):
    """①: meta.pdbbin_ver=2 → 合法, validator 容忍 (帧版本升级不杀数据)"""
    meta = valid_meta()
    meta["pdbbin_ver"] = 2
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc(meta=meta),
                                                "pdbver2.json"))
    assert rc == 0, "pdbbin_ver=2 应通过 (rc=%s)" % rc


def test_pdbbin_ver3_fails(tmp):
    """①: meta.pdbbin_ver=3 → 非法版本 FAIL (fail-closed)"""
    meta = valid_meta()
    meta["pdbbin_ver"] = 3
    assert_fail(tmp, make_doc(meta=meta), "pdbbin_ver3")


def test_pdbbin_ver_absent_passes(tmp):
    """①: 旧 JSON 无 pdbbin_ver 字段 → 按 v1 过 (容忍, 不破坏旧链路)"""
    # valid_meta() 不带 pdbbin_ver — 即此场景
    rc = validate_gain_ladder.validate(write_doc(tmp, make_doc(), "pdbver_absent.json"))
    assert rc == 0, "无 pdbbin_ver 字段 (旧 JSON) 应通过 (rc=%s)" % rc


if __name__ == "__main__":
    tmp = tempfile.mkdtemp(prefix="gain_ladder_validate_")
    tests = [
        test_missing_identity,
        test_invalid_run_status,
        test_bad_dt_and_ch_cfg,
        test_bad_health,
        test_short_gate,
        test_t95_without_leave,
        test_t95_dwell_insufficient,
        test_silent_stream,
        test_timeout_warns,
        test_valid_round,
        test_gap_loci_mismatch,
        test_gap_loci_matched,
        test_gate_pp_borderline_warns,
        test_gap5_delta0_warns,
        test_gap5_delta_pos_fails,
        test_gap5_no_delta_fails,
        test_pdbbin_ver2_accepts,
        test_pdbbin_ver3_fails,
        test_pdbbin_ver_absent_passes,
    ]
    for test in tests:
        test(tmp)
        print("PASS: %s" % test.__name__)
    print("ALL_PASS")
