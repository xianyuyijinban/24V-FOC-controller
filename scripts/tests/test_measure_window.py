#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_measure_window.py — MeasureWindow.wait_stable 稳定门单测

2026-09-05 恢复规划: 旧 wait_stable 累计 5 帧且 pp 合格即返回, 不要求样本
时间跨度达到 gate_window → "5 帧但仅 0.1s" 假稳态放行 (12:09 A 轮1 gate=0.1s)。
修复: span = latest - first >= gate_window 才判定 pp。
本测试用合成 N 帧队列覆盖:
  T1: 5 帧仅 0.1s span 但 pp 合格 → 必须 False (假稳态拦下)
  T2: 覆盖完整 gate_window 且 pp 合格 → True
  T3: 旧积压帧 (调用前入队) 被门前排空 → False (除非新帧也达标)
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import foclink  # noqa: E402


def make_q(frames):
    """frames: (age, ang) 列表, age 为相对 now 的负秒数 → 元组 (hrx, p1, ang, ...)"""
    now = time.time()
    return [(now + age, "1", ang, "0") for age, ang in frames]


def test_5frames_01s_fake_stable():
    """T1: 5 帧 span=0.1s (< gate_window=2.0) 但 pp=0.04 合格 → 必须 False"""
    q = make_q([(-0.1, 120.0), (-0.09, 120.02), (-0.06, 120.01),
                (-0.03, 120.02), (0.0, 120.01)])
    mw = foclink.MeasureWindow(q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    # wait_stable 会先门前排空 (调用前已有帧被删) — 这里只放新帧
    # 重新构造: 调用后逐帧喂入 5 帧 span 0.1s
    q2 = []
    mw2 = foclink.MeasureWindow(q2, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    import threading

    def feeder():
        time.sleep(0.01)
        base = time.time()
        for i, dy in enumerate([0.01, 0.03, 0.02, 0.02, 0.03]):
            # span 0.1s: 5 帧散在 0.1s 内
            ts = base + i * 0.02
            q2.append((ts, "1", 120.0 + dy, "0"))
    t = threading.Thread(target=feeder, daemon=True)
    t.start()
    ok, waited = mw2.wait_stable(120.0, timeout=2.0, angle_index=2)
    # span 最大 0.08s < 2.0 → 必须 False
    assert not ok, "T1 FAIL: 5帧0.1s假稳态被放行 (ok=%s, waited=%.2f)" % (ok, waited)


def test_full_2s_stable_true():
    """T2: 覆盖完整 gate_window (2s) 且 pp 合格 → True"""
    q2 = []
    mw2 = foclink.MeasureWindow(q2, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    import threading

    def feeder():
        time.sleep(0.01)
        for i in range(45):  # 45 帧 × 0.05s = 2.2s span (>gate_window), 真实节拍
            q2.append((time.time(), "1", 120.0 + (0.01 if i % 2 else -0.01), "0"))
            time.sleep(0.05)
    t = threading.Thread(target=feeder, daemon=True)
    t.start()
    ok, waited = mw2.wait_stable(120.0, timeout=3.5, angle_index=2)
    assert ok, "T2 FAIL: 完整2s稳定未放行 (waited=%.2f)" % waited


def test_old_backlog_drained():
    """T3: 调用前已入队的旧积压帧 (窗口前) 被门前排空 → 不混入"""
    q = make_q([(-5.0, 120.0), (-4.5, 120.0), (-4.0, 120.0), (-3.5, 120.0), (-3.0, 120.0)])
    mw = foclink.MeasureWindow(q, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    old_n = len(q)
    ok, waited = mw.wait_stable(120.0, timeout=0.5, angle_index=2)
    assert not ok, "T3 FAIL: 旧积压帧未被排空 (ok=%s waited=%.2f)" % (ok, waited)
    # 门前排空后队列应为空 (旧帧全删)
    assert len(q) == 0, "T3 FAIL: 门前排空后队列还应剩 %d" % len(q)


def test_late_enter_exit():
    """T4: 进入 5% 窗又离开再进 → 重计时 (t95 相关, wait_stable 用于稳定门;
    此处只测 wait_stable: 帧在窗内 pp 合格但 span 未满 → False)"""
    q2 = []
    mw2 = foclink.MeasureWindow(q2, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    import threading

    def feeder():
        time.sleep(0.01)
        base = time.time()
        # 前 1s 帧 pp 合格, 后 1s 有大摆 → span 有效但部分越界
        for i in range(40):
            if i < 20:
                ang = 120.0 + 0.02
            else:
                ang = 120.0 + 2.0  # 大摆 (pp 不合格)
            q2.append((base + i * 0.05, "1", ang, "0"))
    t = threading.Thread(target=feeder, daemon=True)
    t.start()
    ok, waited = mw2.wait_stable(120.0, timeout=2.5, angle_index=2)
    assert not ok, "T4 FAIL: 大摆帧未被拦下 (ok=%s)" % ok


def test_wobble_then_stable_slides_out():
    """T5 (2026-09-06 Kimi 裁决): 先摆后稳 — 滑动窗下回位大摆滑出后过门。
    序列: 前 3s 大摆 (±3°), 后 3s 稳 (±0.02°)。滑动窗 (gate_window=2s) 下,
    大摆段滑出窗口后窗内只剩稳态帧 → pp 收敛 → 过门。
    feeder 用真实 20Hz 节拍 (sleep 0.05) — 首版合成帧同时入队掩盖了
    span<=gate_window-帧间隔 的数学缺陷 (第二次"夹具匹配实现"教训)。"""
    q2 = []
    mw2 = foclink.MeasureWindow(q2, win_seconds=2.0, gate_pp=0.1, gate_window=2.0)
    import threading

    def feeder():
        time.sleep(0.01)
        base = time.time()
        for i in range(120):  # 6s @20Hz 真实节拍
            if i < 60:  # 前 3s 大摆
                ang = 120.0 + (3.0 if i % 2 else -3.0)
            else:       # 后 3s 稳
                ang = 120.0 + (0.02 if i % 2 else -0.02)
            q2.append((time.time(), "1", ang, "0"))
            time.sleep(0.05)
    t = threading.Thread(target=feeder, daemon=True)
    t.start()
    ok, waited = mw2.wait_stable(120.0, timeout=8.0, angle_index=2)
    # 大摆后滑出 (3s + 2s 窗) ≈ 5s, 稳态应过门
    assert ok, "T5 FAIL: 先摆后稳未在滑动窗下过门 (waited=%.2f)" % waited
    assert waited < 6.0, "T5 FAIL: 过门太慢 %.1f (应 ~5s)" % waited


if __name__ == "__main__":
    tests = [test_5frames_01s_fake_stable, test_full_2s_stable_true,
             test_old_backlog_drained, test_late_enter_exit,
             test_wobble_then_stable_slides_out]
    for t in tests:
        t()
        print("PASS: %s" % t.__name__)
    print("ALL_PASS")
