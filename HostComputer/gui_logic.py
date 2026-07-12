from __future__ import annotations

import csv
import io
import json
import math
import re
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List

try:
    from .data_parser import CommandBuilder, FOCDataPacket, CurrentSample, control_to_user_angle, AckResult
except ImportError:
    from data_parser import CommandBuilder, FOCDataPacket, CurrentSample, control_to_user_angle, AckResult


MODE_LABELS = {
    0: "Iq_ref (A)",
    1: "速度 (rad/s)",
    2: "位置 (deg)",
}

MODE_NAMES = {
    0: "力矩",
    1: "速度",
    2: "位置",
}

# V1.2: Chinese display mapping for product modes (protocol token → Chinese label)
APP_MODE_CN = {
    "RAW": "原始控制",
    "JOINT_POS": "关节位置",
    "GIMBAL_SPEED": "云台速度",
    "HOLD": "位置保持",
    "SPRING_DAMPER": "弹簧阻尼",
    "DETENT": "卡点旋钮",
    "SCROLL_WHEEL": "滚轮鼠标",
}

APP_MODE_TOKENS = list(APP_MODE_CN.keys())  # canonical display order

def app_mode_cn(token: str) -> str:
    """Return Chinese label for a protocol token, falling back to the token itself."""
    return APP_MODE_CN.get(token, token)

ACK_TIMEOUT_MS = 500  # ACK response timeout in milliseconds

FOC_STATE_IDLE = 0
FOC_STATE_INIT = 1
FOC_STATE_PARAM_IDENTIFY = 2
FOC_STATE_READY = 3
FOC_STATE_RUNNING = 4
FOC_STATE_FAULT = 5

APP_FAULT_LABELS = {
    0: "无",
    1: "过流",
    2: "过压",
    3: "欠压",
    4: "编码器",
    5: "DRV8350S",
    6: "参数无效",
    7: "ADC采样",
}

APP_WARNING_LABELS = {
    0x00000001: "欠压",
    0x00000002: "过压",
    0x00000003: "欠压/过压",
}

LOG_LEVELS = ("INFO", "TX", "RX", "ERROR")
PLOT_CHANNELS = (
    "angle",
    "speed",
    "speed_ref",
    "pos_ref_deg",
    "Vbus",
    "Ia",
    "Ib",
    "Ic",
    "Id",
    "Iq",
    "Id_ref",
    "Iq_ref",
    "Vd",
    "Vq",
)
DEFAULT_PROFILE_PATH = Path.home() / ".24v_foc_host_gui.json"
GUI_PROFILE_SCHEMA_VERSION = 4
POSITION_DEG_MIN = 0.0
POSITION_DEG_MAX = 360.0
FAULT_ERROR_KEYWORDS = (
    "fault",
    "故障",
    "过流",
    "overcurrent",
    "过压",
    "overvoltage",
    "欠压",
    "undervoltage",
    "drv8350",
    "tle5012",
    "编码器",
    "encoder",
)


@dataclass
class LoopTuning:
    kp: float = 0.0
    ki: float = 0.0


@dataclass
class PositionLoopTuning:
    kp: float = 0.0
    kd: float = 0.0


@dataclass
class AdcNoiseChannelStats:
    minimum: int
    maximum: int
    mean: int
    peak_to_peak: int
    stddev: int


@dataclass
class AdcNoiseResult:
    ok: bool
    status: str
    display_text: str
    samples: int | None = None
    channels: dict[str, AdcNoiseChannelStats] = field(default_factory=dict)


def _default_current_tuning() -> LoopTuning:
    return LoopTuning(kp=0.03, ki=0.5)


def _default_speed_tuning() -> LoopTuning:
    return LoopTuning(kp=0.10, ki=0.0)


def _default_position_tuning() -> PositionLoopTuning:
    return PositionLoopTuning(kp=2.0, kd=0.08)


@dataclass
class GuiProfile:
    last_port: str = ""
    baud_rate: int = 1000000  # V1.1 baseline
    selected_mode: int = 0
    log_filters: list[str] = field(default_factory=lambda: list(LOG_LEVELS))
    undervoltage_limit: float = 18.0
    overvoltage_limit: float = 30.0
    motor_pn: int = 11
    current_target: tuple[float, float] = (0.0, 0.0)
    speed_target: float = 0.0
    position_target: float = 0.0
    current_pi: LoopTuning = field(default_factory=_default_current_tuning)
    speed_pi: LoopTuning = field(default_factory=_default_speed_tuning)
    position_pd: PositionLoopTuning = field(default_factory=_default_position_tuning)

    def to_dict(self) -> dict:
        return {
            "schema_version": GUI_PROFILE_SCHEMA_VERSION,
            "last_port": self.last_port,
            "baud_rate": self.baud_rate,
            "selected_mode": self.selected_mode,
            "log_filters": list(self.log_filters),
            "undervoltage_limit": self.undervoltage_limit,
            "overvoltage_limit": self.overvoltage_limit,
            "motor_pn": self.motor_pn,
            "current_target": [self.current_target[0], self.current_target[1]],
            "speed_target": self.speed_target,
            "position_target": self.position_target,
            "current_pi": {"kp": self.current_pi.kp, "ki": self.current_pi.ki},
            "speed_pi": {"kp": self.speed_pi.kp, "ki": self.speed_pi.ki},
            "position_pd": {"kp": self.position_pd.kp, "kd": self.position_pd.kd},
        }

    @classmethod
    def from_dict(cls, payload: dict) -> "GuiProfile":
        profile = cls()
        schema_version = int(payload.get("schema_version", 0))
        current_target = payload.get("current_target", profile.current_target)
        if isinstance(current_target, (list, tuple)) and len(current_target) == 2:
            profile.current_target = (float(current_target[0]), float(current_target[1]))

        profile.last_port = str(payload.get("last_port", profile.last_port))
        profile.baud_rate = int(payload.get("baud_rate", profile.baud_rate))
        profile.selected_mode = int(payload.get("selected_mode", profile.selected_mode))
        profile.log_filters = [
            str(level)
            for level in payload.get("log_filters", profile.log_filters)
            if str(level) in LOG_LEVELS
        ] or list(LOG_LEVELS)
        profile.undervoltage_limit = float(payload.get("undervoltage_limit", profile.undervoltage_limit))
        profile.overvoltage_limit = float(payload.get("overvoltage_limit", profile.overvoltage_limit))
        profile.motor_pn = int(payload.get("motor_pn", profile.motor_pn))
        profile.speed_target = float(payload.get("speed_target", profile.speed_target))
        profile.position_target = float(payload.get("position_target", profile.position_target))

        for key in ("current_pi", "speed_pi"):
            raw_loop = payload.get(key, {})
            if not isinstance(raw_loop, dict):
                continue
            tuning = LoopTuning(
                kp=float(raw_loop.get("kp", getattr(profile, key).kp)),
                ki=float(raw_loop.get("ki", getattr(profile, key).ki)),
            )
            setattr(profile, key, tuning)

        raw_position_pd = payload.get("position_pd")
        if isinstance(raw_position_pd, dict):
            profile.position_pd = PositionLoopTuning(
                kp=float(raw_position_pd.get("kp", profile.position_pd.kp)),
                kd=float(raw_position_pd.get("kd", profile.position_pd.kd)),
            )
        else:
            legacy_position_pi = payload.get("position_pi", {})
            if isinstance(legacy_position_pi, dict):
                profile.position_pd = PositionLoopTuning(
                    kp=float(legacy_position_pi.get("kp", profile.position_pd.kp)),
                    kd=profile.position_pd.kd,
                )
        if schema_version < GUI_PROFILE_SCHEMA_VERSION:
            cls._migrate_12v_bench_defaults(profile)
        return profile

    @staticmethod
    def _migrate_12v_bench_defaults(profile: "GuiProfile"):
        old_current_defaults = (
            LoopTuning(kp=0.2, ki=0.01),
            LoopTuning(kp=0.3, ki=0.0),
        )
        old_speed_defaults = (
            LoopTuning(kp=1.0, ki=0.1),
            LoopTuning(kp=0.3, ki=0.0),
        )
        old_position_defaults = (
            PositionLoopTuning(kp=10.0, kd=0.10),
            PositionLoopTuning(kp=4.0, kd=0.12),
        )

        if any(GuiProfile._loop_tuning_matches(profile.current_pi, old) for old in old_current_defaults):
            profile.current_pi = _default_current_tuning()
        if any(GuiProfile._loop_tuning_matches(profile.speed_pi, old) for old in old_speed_defaults):
            profile.speed_pi = _default_speed_tuning()
        if any(GuiProfile._position_tuning_matches(profile.position_pd, old) for old in old_position_defaults):
            profile.position_pd = _default_position_tuning()
        if profile.undervoltage_limit == 9.0:
            profile.undervoltage_limit = 18.0
        if profile.overvoltage_limit == 16.0:
            profile.overvoltage_limit = 30.0

    @staticmethod
    def _loop_tuning_matches(left: LoopTuning, right: LoopTuning) -> bool:
        return math.isclose(left.kp, right.kp) and math.isclose(left.ki, right.ki)

    @staticmethod
    def _position_tuning_matches(left: PositionLoopTuning, right: PositionLoopTuning) -> bool:
        return math.isclose(left.kp, right.kp) and math.isclose(left.kd, right.kd)


@dataclass
class HostAppState:
    is_connected: bool = False
    control_mode: int = 0          # underlying FOC control mode (0=torque,1=speed,2=position)
    power_unlocked: bool = False
    motor_enabled: bool = False
    identify_active: bool = False
    motor_identified: bool = False
    stall_mode_armed: bool = False
    stall_open_loop_active: bool = False
    encoder_detected: Optional[bool] = None
    encoder_dir: int = -1
    foc_state: Optional[int] = None
    fault_active: bool = False
    available_ports: list[str] = field(default_factory=list)
    last_packet: Optional[FOCDataPacket] = None
    last_packet_received_at_ms: Optional[int] = None

    # ── ACK / Command tracking (V1.2) ──────────────────────────────────────
    pending_command: Optional[str] = None           # command awaiting ACK, e.g. "UNLOCK", "ENABLE"
    pending_command_value: Optional[str] = None     # pending target payload, e.g. "0", "1", "HOLD"
    pending_command_sent_at_ms: Optional[int] = None
    last_ack: Optional[str] = None                  # raw ACK text, e.g. "UNLOCK,OK"
    last_ack_at_ms: Optional[int] = None
    last_command_error: Optional[str] = None        # e.g. "ENABLE,FAIL,not unlocked"
    firmware_ack_supported: bool = True             # set False after first ACK timeout

    # ── Joint Product Mode state (V1.2) ───────────────────────────────────
    app_mode: Optional[str] = None           # RAW|...|DETENT -- firmware confirmed
    app_mode_selected: str = "RAW"           # GUI selected mode (may differ from confirmed)
    app_mode_ctrl: Optional[int] = None      # underlying ctrl_mode (0=torque,1=speed,2=position)
    joint_limit_enabled: bool = False
    joint_limit_min: Optional[float] = None  # degrees
    joint_limit_max: Optional[float] = None  # degrees
    gimbal_ramp_accel: Optional[float] = None  # rad/s^2
    spring_K: Optional[float] = None
    spring_D: Optional[float] = None
    spring_limit: Optional[float] = None
    detent_count: Optional[int] = None
    detent_strength: Optional[float] = None
    detent_width: Optional[float] = None
    detent_limit: Optional[float] = None

    # -- Command Sequence Queue (V1.2+) --
    pending_sequence: Optional[list[str]] = None
    pending_seq_index: int = 0


class RollingPlotBuffer:
    def __init__(self, max_samples: int | None = None, history_window_ms: int = 30000):
        self.history_window_ms = max(1, int(history_window_ms))
        self.max_samples = max(1, int(max_samples)) if max_samples is not None else None
        self._rows: deque[dict[str, float]] = deque(maxlen=self.max_samples)

    def append_packet(self, packet: FOCDataPacket):
        previous = dict(self._rows[-1]) if self._rows else {}
        # Firmware stores pos_ref in control frame (multiplied by encoder_dir).
        # Convert back to user-facing degrees for display.
        enc_dir = (
            packet.motor_param_encoder_dir
            if packet.motor_param_encoder_dir is not None
            else -1
        )
        user_pos_rad = control_to_user_angle(packet.pos_ref, enc_dir)
        row = {
            "timestamp_ms": float(packet.timestamp),
            "angle": float(packet.angle),
            "speed": float(packet.speed),
            "speed_ref": float(packet.speed_ref),
            "pos_ref_deg": radians_to_degrees(user_pos_rad),
            "Vbus": float(packet.vbus),
            "Ia": float(packet.Ia),
            "Ib": float(packet.Ib),
            "Ic": float(packet.Ic),
            "Id": float(packet.Id),
            "Iq": float(packet.Iq),
            "Id_ref": float(packet.Id_ref),
            "Iq_ref": float(packet.Iq_ref),
            "Vd": float(packet.Vd),
            "Vq": float(packet.Vq),
        }
        if getattr(packet, "phase_current_only", False):
            for channel in PLOT_CHANNELS:
                if channel not in {"Ia", "Ib", "Ic"} and channel in previous:
                    row[channel] = previous[channel]

        self._rows.append(row)
        self._trim_history()

    def _trim_history(self):
        if not self._rows:
            return
        cutoff = self._rows[-1]["timestamp_ms"] - float(self.history_window_ms)
        while self._rows and self._rows[0]["timestamp_ms"] < cutoff:
            self._rows.popleft()

    def has_rows(self) -> bool:
        return bool(self._rows)

    def series(self, channel: str) -> tuple[list[float], list[float]]:
        if channel not in PLOT_CHANNELS:
            raise KeyError(f"Unknown channel: {channel}")
        timestamps = [row["timestamp_ms"] for row in self._rows]
        values = [row[channel] for row in self._rows]
        return timestamps, values

    def export_rows(self, channels: Optional[list[str]] = None) -> list[dict[str, float]]:
        selected = channels or list(PLOT_CHANNELS)
        rows = []
        for row in self._rows:
            export = {"timestamp_ms": int(row["timestamp_ms"])}
            for channel in selected:
                if channel in row:
                    export[channel] = row[channel]
            rows.append(export)
        return rows


def mode_target_label(mode: int) -> str:
    return MODE_LABELS.get(mode, "目标值")


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, "自定义")


def parse_float_field(
    raw_value: str,
    field_name: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    text = str(raw_value).strip()
    if not text:
        raise ValueError(f"{field_name} 为必填项。")

    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"{field_name} 必须是有效数字。") from exc

    if not math.isfinite(value):
        raise ValueError(f"{field_name} 必须是有限数值。")
    if minimum is not None and value < minimum:
        raise ValueError(f"{field_name} 必须 >= {minimum}。")
    if maximum is not None and value > maximum:
        raise ValueError(f"{field_name} 必须 <= {maximum}。")
    return value


def build_current_ref_command(id_raw: str, iq_raw: str) -> str:
    id_ref = parse_float_field(id_raw, "Id_ref")
    iq_ref = parse_float_field(iq_raw, "Iq_ref")
    return CommandBuilder.set_current_ref(id_ref, iq_ref)


def build_speed_ref_command(speed_raw: str) -> str:
    MAX_SPEED_RAD_PER_S = 8.0   # matches FOC_MOTION_CFG_SPEED_LIMIT_MAX
    speed = parse_float_field(speed_raw, "速度", minimum=-MAX_SPEED_RAD_PER_S, maximum=MAX_SPEED_RAD_PER_S)
    return CommandBuilder.set_speed_ref(speed)


def degrees_to_radians(degrees: float) -> float:
    return float(degrees) * math.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    return float(radians) * 180.0 / math.pi


def build_position_ref_command(position_raw: str) -> str:
    position_deg = parse_float_field(
        position_raw,
        "位置",
        minimum=POSITION_DEG_MIN,
        maximum=POSITION_DEG_MAX,
    )
    return CommandBuilder.set_position_ref(degrees_to_radians(position_deg))


def build_vbus_limit_command(undervoltage_raw: str, overvoltage_raw: str) -> str:
    undervoltage = parse_float_field(undervoltage_raw, "欠压阈值", minimum=0.0)
    overvoltage = parse_float_field(overvoltage_raw, "过压阈值", minimum=0.0)
    if overvoltage <= undervoltage:
        raise ValueError("过压阈值必须 > 欠压阈值。")
    return CommandBuilder.set_vbus_limits(undervoltage, overvoltage)


def build_encoder_dir_command(dir_raw: str) -> str:
    direction = parse_float_field(dir_raw, "编码器方向", minimum=-1.0, maximum=1.0)
    if abs(direction) < 0.5:
        raise ValueError("编码器方向必须是 +1 或 -1。")
    return CommandBuilder.set_encoder_dir(int(direction))


def build_motor_pn_command(pole_pairs_raw: str) -> str:
    pole_pairs_value = parse_float_field(pole_pairs_raw, "极对数", minimum=1.0, maximum=50.0)
    if not float(pole_pairs_value).is_integer():
        raise ValueError("极对数必须是整数。")
    return CommandBuilder.set_motor_pn(int(pole_pairs_value))


def build_adc_noise_command(samples_raw: str) -> str:
    samples = parse_float_field(samples_raw, "ADC噪声样本数", minimum=16.0, maximum=4096.0)
    if not float(samples).is_integer():
        raise ValueError("ADC噪声样本数必须是整数。")
    return CommandBuilder.adc_noise_test(int(samples))


def parse_adc_noise_response(message: str) -> AdcNoiseResult | None:
    text = str(message).strip()
    if not text.startswith("ADC_NOISE,"):
        return None

    if text.startswith("ADC_NOISE,BUSY"):
        return AdcNoiseResult(
            ok=False,
            status="BUSY",
            display_text="ADC噪声测试：当前电机运行或识别中，不能测试。",
        )

    if text.startswith("ADC_NOISE,START"):
        sample_match = re.search(r"\bn=(\d+)", text)
        samples = int(sample_match.group(1)) if sample_match else None
        return AdcNoiseResult(
            ok=True,
            status="START",
            samples=samples,
            display_text=f"ADC噪声测试：固件已开始采样，n={samples if samples is not None else '--'}，等待结果...",
        )

    if text.startswith("ADC_NOISE,ERR"):
        reason = text.split(",", 2)[2] if text.count(",") >= 2 else "unknown"
        return AdcNoiseResult(
            ok=False,
            status="ERR",
            display_text=f"ADC噪声测试失败：{reason}",
        )

    if not text.startswith("ADC_NOISE,OK"):
        return AdcNoiseResult(
            ok=False,
            status="UNKNOWN",
            display_text=f"ADC噪声测试返回未知格式：{text}",
        )

    sample_match = re.search(r"\bn=(\d+)", text)
    samples = int(sample_match.group(1)) if sample_match else None
    channels: dict[str, AdcNoiseChannelStats] = {}
    for name in ("A", "B", "C", "VBUS"):
        match = re.search(
            rf"{name}:min=(\d+),max=(\d+),mean=(\d+),pp=(\d+),std=(\d+)",
            text,
        )
        if match:
            channels[name] = AdcNoiseChannelStats(
                minimum=int(match.group(1)),
                maximum=int(match.group(2)),
                mean=int(match.group(3)),
                peak_to_peak=int(match.group(4)),
                stddev=int(match.group(5)),
            )

    lines = [f"ADC噪声测试完成：n={samples if samples is not None else '--'}"]
    for name in ("A", "B", "C", "VBUS"):
        stats = channels.get(name)
        if stats is None:
            continue
        label = name.ljust(2)
        lines.append(
            f"{label} min={stats.minimum} max={stats.maximum} "
            f"mean={stats.mean} pp={stats.peak_to_peak} std={stats.stddev}"
        )

    return AdcNoiseResult(
        ok=True,
        status="OK",
        display_text="\n".join(lines),
        samples=samples,
        channels=channels,
    )


def build_loop_gain_command(loop_name: str, kp_raw: str, gain2_raw: str) -> str:
    loop_text = {
        "current": "电流环",
        "speed": "速度环",
        "position": "位置环",
    }.get(loop_name, loop_name)
    kp = parse_float_field(kp_raw, f"{loop_text} Kp", minimum=0.0)
    gain2_name = "Kd" if loop_name == "position" else "Ki"
    gain2 = parse_float_field(gain2_raw, f"{loop_text} {gain2_name}", minimum=0.0)
    builders = {
        "current": CommandBuilder.set_current_pi,
        "speed": CommandBuilder.set_speed_pi,
        "position": CommandBuilder.set_position_pd,
    }
    if loop_name not in builders:
        raise ValueError(f"不支持的环路参数类型：{loop_name}")
    return builders[loop_name](kp, gain2)


def button_enable_state(state: HostAppState) -> dict[str, bool]:
    connected = bool(state.is_connected)
    unlocked = bool(state.power_unlocked)
    enabled = bool(state.motor_enabled)
    identify_active = bool(state.identify_active)
    fault_active = bool(state.fault_active or state.foc_state == FOC_STATE_FAULT)
    pending = bool(state.pending_command)
    bridge_managed_wheel = (state.app_mode_selected or "RAW") == "SCROLL_WHEEL"
    can_quick_arm = connected and not enabled and not identify_active and not fault_active and not pending
    return {
        "can_unlock": connected and not unlocked and not pending,
        "can_lock": connected and unlocked,
        "can_enable": connected and unlocked and not enabled and not identify_active and not fault_active and not pending,
        "can_quick_arm": can_quick_arm,
        "can_disable": connected and enabled and not fault_active,
        "can_clear_fault": connected,
        "can_identify_start": connected and unlocked and not identify_active and not enabled and not fault_active and not pending,
        "can_identify_stop": connected and identify_active,
        "can_send_target": connected and not identify_active and not fault_active and not pending,
        # V1.2 — Joint Product Mode
        "app_mode_selector": connected and not identify_active and not fault_active and not pending,
        "joint_limit_config": connected and not identify_active and not fault_active and not pending,
        "gimbal_ramp_config": connected and not identify_active and not fault_active and not pending,
        "spring_detent_config": connected and not identify_active and not fault_active and not pending,
        "motion_target": connected and unlocked and enabled and not identify_active and not fault_active and not pending,
        "hold_button": connected and unlocked and enabled and not identify_active and not fault_active and not pending,
        # V1.2+ -- Product-mode-aware enable/arm
        "app_enable": connected and unlocked and not enabled and not identify_active and not fault_active and not pending and not bridge_managed_wheel,
        "app_arm": can_quick_arm and not bridge_managed_wheel,
    }


def connection_command_state(is_connected: bool) -> dict[str, bool]:
    state = button_enable_state(HostAppState(is_connected=is_connected))
    state["can_identify"] = state["can_identify_start"] or state["can_identify_stop"]
    return state


def is_safe_fallback_command(command: str) -> bool:
    """Commands that move the controller toward a safer or diagnostic state."""
    text = str(command).strip()
    return (
        text in {
            "CMD:ENABLE,0",
            "CMD:UNLOCK,0",
            "CMD:CLEAR_FAULT",
            "CMD:STOP",
            "CMD:FAULT_DETAIL",
        }
        or text.endswith("?")
    )


def is_motion_target_command(command: str) -> bool:
    text = str(command).strip()
    return text.startswith(("CMD:IREF,", "CMD:SREF,", "CMD:PREF,"))


def can_dispatch_command(state: HostAppState, command: str) -> tuple[bool, str]:
    """Host-side command gate used by both buttons and dispatch paths."""
    text = str(command).strip()
    safe_fallback = is_safe_fallback_command(text)
    fault_active = bool(state.fault_active or state.foc_state == FOC_STATE_FAULT)

    if not state.is_connected:
        return False, "当前未连接，命令已拦截。"

    if state.pending_command and not safe_fallback:
        return False, f"正在等待 {state.pending_command} 确认，命令已拦截。"

    if fault_active and not safe_fallback:
        return False, "当前处于故障状态，只允许停机、上锁、清故障或诊断命令。"

    if text == "CMD:ENABLE,1":
        if not state.power_unlocked:
            return False, "请先解锁功率级。"
        if state.motor_enabled:
            return False, "电机已经使能。"
        if state.identify_active:
            return False, "参数识别中，不能使能电机。"

    if text == "CMD:IDENTIFY,1":
        if not state.power_unlocked:
            return False, "请先解锁功率级。"
        if state.motor_enabled:
            return False, "电机已使能，不能开始参数识别。"
        if state.identify_active:
            return False, "参数识别已经在进行中。"

    if is_motion_target_command(text):
        if not state.power_unlocked or not state.motor_enabled:
            return False, "电机未解锁或未使能，不能发送目标值。"
        if state.identify_active:
            return False, "参数识别中，不能发送运动目标。"

    return True, ""


def should_confirm_stall_mode_enable(state: HostAppState) -> bool:
    fault_active = bool(state.fault_active or state.foc_state == FOC_STATE_FAULT)
    requires_stall_mode = (not bool(state.motor_identified)) or (state.encoder_detected is not True)
    return (
        bool(state.is_connected)
        and bool(state.power_unlocked)
        and not bool(state.motor_enabled)
        and not bool(state.identify_active)
        and not fault_active
        and requires_stall_mode
        and not bool(state.stall_mode_armed)
    )


def stall_mode_confirmation_text(state: HostAppState) -> str:
    missing_identify = not bool(state.motor_identified)
    missing_encoder = (state.encoder_detected is not True)

    if missing_identify and missing_encoder:
        return "当前电机尚未完成识别，且未检测到 TLE5012 编码器。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"
    if missing_encoder:
        return "当前未检测到 TLE5012 编码器。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"
    return "当前电机尚未完成识别。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"


# ══════════════════════════════════════════════════════════════════════════
# V1.2+ — APP_MODE sync helpers
# ══════════════════════════════════════════════════════════════════════════

def is_app_mode_synced(state: HostAppState) -> bool:
    """Return True if the GUI selected mode matches the firmware-confirmed mode."""
    return state.app_mode_selected == (state.app_mode or "RAW")


HAPTIC_MODES = frozenset({"SPRING_DAMPER", "DETENT", "SCROLL_WHEEL"})
POSITION_MODES = frozenset({"JOINT_POS", "HOLD"})

def get_app_mode_prerequisites(state: HostAppState, app_mode: str) -> tuple[bool, str]:
    """Check whether the given app mode can be enabled given current state.

    Returns (ok: bool, reason: str).
    """
    encoder_ok = bool(state.encoder_detected)
    identified_ok = bool(state.motor_identified)
    mode_upper = app_mode.upper() if app_mode else "RAW"

    if mode_upper in HAPTIC_MODES:
        if not encoder_ok and not identified_ok:
            return False, "弹簧阻尼/卡点模式需要编码器在线且电机已完成识别。不能使用堵转开环。"
        if not encoder_ok:
            return False, "弹簧阻尼/卡点模式需要编码器在线（TLE5012）。"
        if not identified_ok:
            return False, "弹簧阻尼/卡点模式需要电机已完成参数识别。不能使用堵转开环。"
        return True, ""

    if mode_upper in POSITION_MODES:
        if not encoder_ok:
            return False, "位置类模式（关节位置/位置保持）需要编码器在线。"
        return True, ""

    # RAW, GIMBAL_SPEED — no special prereqs
    return True, ""


def build_app_mode_command(app_mode: str) -> str:
    """Build CMD:APP_MODE,<token> from a mode token."""
    return CommandBuilder.app_mode_set(app_mode)


def build_app_enable_sequence(app_mode: str, state: HostAppState) -> list[str]:
    """Build the full command sequence for enabling in a product mode.

    Sequence: APP_MODE -> (STALL_MODE if needed) -> (UNLOCK if needed) -> ENABLE,1
    """
    commands: list[str] = []
    commands.append(build_app_mode_command(app_mode))
    requires_stall = (not bool(state.motor_identified)) or (state.encoder_detected is not True)
    if requires_stall and not state.stall_mode_armed and not state.power_unlocked:
        commands.append("CMD:STALL_MODE,1")
    if not state.power_unlocked:
        commands.append("CMD:UNLOCK,1")
    commands.append("CMD:ENABLE,1")
    return commands


def build_app_arm_sequence(app_mode: str, state: HostAppState) -> list[str]:
    """Build the unlock+enable sequence for product mode quick-arm."""
    commands: list[str] = []
    commands.append(build_app_mode_command(app_mode))
    if not state.power_unlocked:
        requires_stall = (not bool(state.motor_identified)) or (state.encoder_detected is not True)
        if requires_stall and not state.stall_mode_armed:
            commands.append("CMD:STALL_MODE,1")
        commands.append("CMD:UNLOCK,1")
    commands.append("CMD:ENABLE,1")
    return commands


def build_app_target_sequence(app_mode: str, target_command: str) -> list[str]:
    """Build sequence: APP_MODE,<mode> -> target_command.

    Use for PREF / SREF in JOINT_POS / GIMBAL_SPEED modes.
    """
    return [build_app_mode_command(app_mode), target_command]


def build_app_config_sequence(app_mode: str, config_command: str) -> list[str]:
    """Build sequence: APP_MODE,<mode> -> config_command.

    Use for SPRING:CFG / DETENT:CFG in haptic modes.
    """
    return [build_app_mode_command(app_mode), config_command]


def start_command_sequence(state: HostAppState, commands: list[str]) -> str | None:
    """Store a sequence and return the first command to emit.

    Returns None if the sequence is empty.
    """
    if not commands:
        state.pending_sequence = None
        state.pending_seq_index = 0
        return None
    state.pending_sequence = list(commands)
    state.pending_seq_index = 0
    return state.pending_sequence[0]


def advance_command_sequence(state: HostAppState) -> str | None:
    """Advance to the next command in a pending sequence.

    Call after a successful ACK for the current sequence step.
    Returns the next command to emit, or None if the sequence is complete.
    """
    if state.pending_sequence is None:
        return None
    state.pending_seq_index += 1
    if state.pending_seq_index >= len(state.pending_sequence):
        state.pending_sequence = None
        state.pending_seq_index = 0
        return None
    return state.pending_sequence[state.pending_seq_index]


def abort_command_sequence(state: HostAppState, reason: str = "序列已中止"):
    """Cancel any pending command sequence."""
    state.pending_sequence = None
    state.pending_seq_index = 0
    state.last_command_error = reason


def apply_command_effects(state: HostAppState, command: str):
    """Set pending_command on key state-changing commands.

    V1.2: no longer optimistically sets power_unlocked / motor_enabled etc.
    Those booleans are only updated by apply_ack_effects() on confirmed ACK,
    or by apply_packet_effects() on N-frame convergence.

    Destructive actions (lock, disable) still take immediate effect because
    they cannot fail in a meaningful way — the firmware always accepts them.
    """
    text = command.strip()

    if text == "CMD:UNLOCK,1":
        state.pending_command = "UNLOCK"
        state.pending_command_value = "1"
    elif text == "CMD:UNLOCK,0":
        state.pending_command = "UNLOCK"
        state.pending_command_value = "0"
        # Destructive: lock cascades immediately
        state.power_unlocked = False
        state.motor_enabled = False
        state.identify_active = False
        state.stall_mode_armed = False
        state.stall_open_loop_active = False
    elif text == "CMD:ENABLE,1":
        state.pending_command = "ENABLE"
        state.pending_command_value = "1"
    elif text == "CMD:ENABLE,0":
        state.pending_command = "ENABLE"
        state.pending_command_value = "0"
        state.stall_open_loop_active = False
    elif text == "CMD:IDENTIFY,1":
        state.pending_command = "IDENTIFY"
        state.pending_command_value = "1"
    elif text == "CMD:IDENTIFY,0":
        state.pending_command = "IDENTIFY"
        state.pending_command_value = "0"
    elif text == "CMD:STALL_MODE,1":
        state.pending_command = "STALL_MODE"
        state.pending_command_value = "1"
    elif text == "CMD:STALL_MODE,0":
        state.pending_command = "STALL_MODE"
        state.pending_command_value = "0"
        state.stall_open_loop_active = False
    elif text.startswith("CMD:MODE,"):
        state.pending_command = "MODE"
        state.pending_command_value = text.split(",", 1)[1]
    # V1.2 — Joint Product Mode
    elif text.startswith("CMD:APP_MODE,"):
        state.pending_command = "APP_MODE"
        state.pending_command_value = text.split(",", 1)[1]
    elif text == "JOINT:LIMIT,OFF":
        # Product-mode local state — immediate is fine
        state.joint_limit_enabled = False
        state.joint_limit_min = None
        state.joint_limit_max = None
    elif text.startswith("DETENT:CFG,") or text.startswith("CMD:DETENT_CFG,"):
        state.pending_command = "DETENT:CFG"
        state.pending_command_value = text
    elif text.startswith("SPRING:CFG,") or text.startswith("CMD:CFG,"):
        state.pending_command = "SPRING:CFG"
        state.pending_command_value = text


def apply_ack_effects(state: HostAppState, ack: AckResult, now_ms: int = 0):
    """Update HostAppState from a parsed ACK/FAIL response.

    Called when the firmware sends a command acknowledgement.
    On OK → set the corresponding state boolean.
    On FAIL → record error, do NOT change state.
    """
    pending_value = state.pending_command_value
    if pending_value is None:
        pending_value = ack.command_value
    state.pending_command = None
    state.pending_command_value = None
    state.pending_command_sent_at_ms = None
    state.last_ack = ack.raw
    state.last_ack_at_ms = now_ms

    if ack.ok:
        state.last_command_error = None
        if ack.command == "UNLOCK":
            state.power_unlocked = pending_value != "0"
            if not state.power_unlocked:
                state.motor_enabled = False
                state.identify_active = False
                state.stall_mode_armed = False
                state.stall_open_loop_active = False
        elif ack.command == "ENABLE":
            state.motor_enabled = pending_value != "0"
            if not state.motor_enabled:
                state.stall_open_loop_active = False
        elif ack.command == "IDENTIFY":
            state.identify_active = pending_value != "0"
        elif ack.command == "STALL_MODE":
            state.stall_mode_armed = pending_value != "0"
            if not state.stall_mode_armed:
                state.stall_open_loop_active = False
        elif ack.command == "MODE" and ack.mode_value is not None:
            state.control_mode = ack.mode_value
        elif ack.command == "APP_MODE" and ack.app_mode_name is not None:
            state.app_mode = ack.app_mode_name
            if ack.app_mode_ctrl is not None:
                state.app_mode_ctrl = ack.app_mode_ctrl
    else:
        state.last_command_error = ack.reason or "未知原因"


def apply_packet_effects(state: HostAppState, packet: FOCDataPacket):
    """Sync HostAppState from incoming N-frame telemetry (authoritative).

    N-frame ONLY updates the underlying FOC control_mode — never app_mode.
    APP_MODE is set exclusively via ACK (apply_ack_effects) or
    APP_MODE response parsing.

    Also performs N-frame convergence: if a pending_command matches the
    runtime state the N-frame reports, the pending is cleared. This
    provides a fallback for firmware that does not send ACKs.
    """
    state.last_packet = packet
    state.foc_state = int(packet.foc_state)
    if packet.control_mode is not None:
        state.control_mode = int(packet.control_mode)
    state.fault_active = packet_has_active_fault(packet)
    state.motor_identified = bool(packet.motor_identified)
    state.stall_mode_armed = bool(packet.stall_mode_armed)
    state.stall_open_loop_active = bool(packet.stall_open_loop_active)
    state.encoder_detected = packet.encoder_detected
    if packet.motor_param_encoder_dir is not None:
        state.encoder_dir = int(packet.motor_param_encoder_dir)
    if state.foc_state == FOC_STATE_PARAM_IDENTIFY:
        state.identify_active = True
        state.motor_enabled = False
    elif state.foc_state == FOC_STATE_RUNNING:
        state.identify_active = False
        state.motor_enabled = True
    else:
        state.identify_active = False
        state.motor_enabled = False

    # V1.2: N-frame convergence — if pending command matches runtime state, clear pending
    _converge_pending_from_nframe(state)


def _converge_pending_from_nframe(state: HostAppState):
    """Clear pending_command when N-frame confirms the expected state.

    This provides backward compatibility with firmware that does not send
    per-command ACKs — the N-frame telemetry serves as implicit confirmation.
    """
    cmd = state.pending_command
    if cmd is None:
        return
    value = state.pending_command_value

    converged = False
    if cmd == "UNLOCK" and value == "1" and state.power_unlocked:
        converged = True
    elif cmd == "UNLOCK" and value == "0" and not state.power_unlocked:
        converged = True
    elif cmd == "ENABLE" and value == "1" and state.motor_enabled:
        converged = True
    elif cmd == "ENABLE" and value == "0" and not state.motor_enabled:
        converged = True
    elif cmd == "IDENTIFY" and value == "1" and state.identify_active:
        converged = True
    elif cmd == "IDENTIFY" and value == "0" and not state.identify_active:
        converged = True
    elif cmd == "STALL_MODE" and value == "1" and state.stall_mode_armed:
        converged = True
    elif cmd == "STALL_MODE" and value == "0" and not state.stall_mode_armed:
        converged = True
    elif cmd == "MODE":
        converged = True  # N-frame already updated control_mode above
    elif cmd == "APP_MODE":
        # N-frame does not carry APP_MODE, so only APP_MODE ACK/query responses
        # may confirm this command.  Do not let raw control-mode telemetry make
        # the GUI claim DETENT/SPRING/HOLD succeeded when firmware did not ACK.
        converged = False

    if converged:
        state.pending_command = None
        state.pending_command_value = None
        state.pending_command_sent_at_ms = None


# ── Joint Product Mode Presets (V1.2) ────────────────────────────────────────

SPRING_PRESETS = {
    "soft":     (0.25, 0.03, 0.15),   # 柔和
    "standard": (0.50, 0.05, 0.30),   # 标准
    "hard":     (0.80, 0.08, 0.30),   # 偏硬
}

DETENT_PRESETS = {
    "light":    (12, 3.00, 0.24, 0.08, 0.40),   # 轻卡点
    "standard": (12, 6.00, 0.24, 0.10, 0.60),   # 标准
    "dense":    (24, 7.00, 0.16, 0.10, 0.70),   # 密集
}


# ── Joint Product Mode Response Parsers (V1.2) ───────────────────────────────

def parse_app_mode_response(line: str) -> Optional[dict]:
    """Parse APP_MODE set/query responses.

    Accepted forms:
    - APP_MODE,OK,RAW
    - APP_MODE,OK,RAW (ctrl_mode=0)
    """
    m = re.search(r'^APP_MODE,OK,(\w+)(?:\s*\(ctrl_mode=(\d+)\))?', line.strip())
    if m:
        ctrl = int(m.group(2)) if m.group(2) is not None else None
        return {"mode": m.group(1), "ctrl": ctrl}
    return None


def parse_joint_limit_response(line: str) -> Optional[dict]:
    """Parse 'JOINT:LIMIT,OK,min=-30.0deg,max=30.0deg' or 'JOINT:LIMIT,OK,OFF'."""
    stripped = line.strip()
    if stripped == "JOINT:LIMIT,OK,OFF":
        return {"enabled": False, "min_deg": None, "max_deg": None}
    m = re.search(r'^JOINT:LIMIT,OK,min=([-\d.]+)deg,max=([-\d.]+)deg', stripped)
    if m:
        return {"enabled": True, "min_deg": float(m.group(1)), "max_deg": float(m.group(2))}
    return None


def parse_gimbal_ramp_response(line: str) -> Optional[dict]:
    """Parse 'GIMBAL:RAMP,OK,accel=2.0radps2' → {'accel': float}."""
    m = re.search(r'^GIMBAL:RAMP,OK,accel=([\d.]+)radps2', line.strip())
    if m:
        return {"accel": float(m.group(1))}
    return None


def parse_spring_cfg_response(line: str) -> Optional[dict]:
    """Parse 'SPRING:CFG,OK,K=0.500,D=0.050,limit=0.300' → {'K','D','limit': float}."""
    m = re.search(r'^SPRING:CFG,OK,K=([\d.]+),D=([\d.]+),limit=([\d.]+)', line.strip())
    if m:
        return {"K": float(m.group(1)), "D": float(m.group(2)), "limit": float(m.group(3))}
    return None


def parse_detent_cfg_response(line: str) -> Optional[dict]:
    """Parse 'DETENT:CFG,OK,count=12,strength=3.000,width=0.220,damping=0.080,limit=0.300'."""
    m = re.search(
        r'^DETENT:CFG,OK,count=(\d+),strength=([\d.]+),width=([\d.]+),damping=([\d.]+),limit=([\d.]+)',
        line.strip(),
    )
    if m:
        return {
            "count": int(m.group(1)),
            "strength": float(m.group(2)),
            "width": float(m.group(3)),
            "damping": float(m.group(4)),
            "limit": float(m.group(5)),
        }
    return None


def parse_wheel_cfg_response(line: str) -> Optional[dict]:
    """Parse 'WHEEL:CFG,OK,count=24,strength=6.000,width=0.160,damping=0.100,limit=0.300'."""
    m = re.search(
        r'^WHEEL:CFG,OK,count=(\d+),strength=([\d.]+),width=([\d.]+),damping=([\d.]+),limit=([\d.]+)',
        line.strip(),
    )
    if m:
        return {
            "count": int(m.group(1)),
            "strength": float(m.group(2)),
            "width": float(m.group(3)),
            "damping": float(m.group(4)),
            "limit": float(m.group(5)),
        }
    return None


def build_wheel_cfg_command(count: int, strength: float, width: float, damping: float, limit: float) -> str:
    """Build WHEEL:CFG,<count>,<strength>,<width>,<damping>,<limit> command."""
    return f"WHEEL:CFG,{int(count)},{strength:.3f},{width:.3f},{damping:.3f},{limit:.3f}\n"


def packet_snapshot(packet: FOCDataPacket) -> dict[str, str]:
    enc_dir = (
        packet.motor_param_encoder_dir
        if packet.motor_param_encoder_dir is not None
        else -1
    )
    user_pos_rad = control_to_user_angle(packet.pos_ref, enc_dir)
    pos_ref_deg = radians_to_degrees(user_pos_rad)
    return {
        "timestamp": f"{packet.timestamp} ms",
        "angle": f"{packet.angle:.2f} deg",
        "speed": f"{packet.speed:.2f} rad/s",
        "currents": f"Id {packet.Id:.2f} A / Iq {packet.Iq:.2f} A",
        "refs": (
            f"Id_ref {packet.Id_ref:.2f} / Iq_ref {packet.Iq_ref:.2f} / "
            f"pos_ref {pos_ref_deg:.2f} deg / speed_ref {packet.speed_ref:.2f} rad/s"
        ),
        "position_ref": f"pos_ref {pos_ref_deg:.2f} deg / speed_ref {packet.speed_ref:.2f} rad/s",
        "voltages": f"Vd {packet.Vd:.2f} V / Vq {packet.Vq:.2f} V / Vbus {packet.vbus:.2f} V",
        "state": str(packet.foc_state),
    }


def can_edit_vbus_limits(state: HostAppState) -> bool:
    return (
        bool(state.is_connected)
        and not bool(state.motor_enabled)
        and not bool(state.identify_active)
        and not bool(state.pending_command)
    )


def application_fault_text(app_fault_code: int) -> str:
    return APP_FAULT_LABELS.get(int(app_fault_code), f"未知故障({int(app_fault_code)})")


def application_warning_text(app_warning_flags: int) -> str:
    flags = int(app_warning_flags)
    if flags in APP_WARNING_LABELS:
        return APP_WARNING_LABELS[flags]

    parts = []
    if flags & 0x00000001:
        parts.append("欠压")
    if flags & 0x00000002:
        parts.append("过压")
    if parts:
        return "/".join(parts)
    return f"未知告警(0x{flags:08X})"


def packet_has_active_fault(packet: FOCDataPacket) -> bool:
    return bool(
        packet.is_fault_active
        or packet.foc_state == FOC_STATE_FAULT
        or getattr(packet, "app_fault_code", 0) != 0
    )


def fault_summary_text(packet: FOCDataPacket) -> dict[str, str]:
    fault_active = packet_has_active_fault(packet)
    app_fault_code = int(packet.app_fault_code)
    app_warning_flags = int(packet.app_warning_flags)
    app_fault = application_fault_text(app_fault_code)

    if fault_active:
        if app_fault_code != 0:
            state_text = f"故障激活（{app_fault}）"
            fault1_text = f"应用故障 {app_fault} (code={app_fault_code})"
        elif packet.foc_state == FOC_STATE_FAULT and packet.fault_flags == 0:
            state_text = "故障激活（应用层故障）"
            fault1_text = "应用故障 未上报故障码（旧固件）"
        else:
            state_text = "故障激活"
            fault1_text = f"FAULT1 0x{packet.fault_status1:04X}"
    elif app_warning_flags != 0:
        app_warning = application_warning_text(app_warning_flags)
        state_text = f"告警激活（{app_warning}）"
        fault1_text = f"应用告警 {app_warning} (0x{app_warning_flags:08X})"
    else:
        state_text = "正常"
        fault1_text = f"FAULT1 0x{packet.fault_status1:04X}"

    return {
        "state": state_text,
        "fault1": fault1_text,
        "vgs2": f"VGS2 0x{packet.vgs_status2:04X}",
        "timestamp": f"{packet.timestamp} ms",
    }


def _normalize_fault_signature_text(text: str) -> str:
    normalized = str(text).replace("\r\n", "\n").replace("\r", "\n").strip()
    normalized = re.sub(r"Time:\s*\d+\s*ms", "Time:<ts>", normalized, flags=re.IGNORECASE)
    normalized = re.sub(r"\b\d+\s+ms\b", "<ts> ms", normalized)
    normalized = re.sub(r"[ \t]+", " ", normalized)
    return normalized


def _fault_detail_reason_signature(text: str) -> str:
    normalized = _normalize_fault_signature_text(text)
    fields: list[str] = []

    for name, pattern in (
        ("app", r"AppFault:\s*(\d+)"),
        ("mi_state", r"\[Motor Identification\][\s\S]*?State:\s*(\d+)"),
        ("mi_error", r"\[Motor Identification\][\s\S]*?Error:\s*(\d+)"),
        ("fault1", r"FAULT1:\s*0x([0-9A-Fa-f]+)"),
        ("vgs2", r"VGS2:\s*0x([0-9A-Fa-f]+)"),
        ("encoder", r"Detected:\s*(YES|NO)"),
    ):
        match = re.search(pattern, normalized, flags=re.IGNORECASE)
        if match:
            fields.append(f"{name}={match.group(1).upper()}")

    reason_lines = re.findall(r"\[(CRIT|WARN|FAIL|COMM|BOOT)\]\s*([^\n\r]+)", normalized, flags=re.IGNORECASE)
    fields.extend(f"{level.upper()}={message.strip()}" for level, message in reason_lines)

    return "|".join(fields) if fields else normalized


def _encoder_fault_text(encoder_detected: Optional[bool]) -> str:
    if encoder_detected is True:
        return "在线"
    if encoder_detected is False:
        return "未检测到"
    return "未知"


def fault_packet_log_entry(packet: FOCDataPacket) -> tuple[str, str] | None:
    raw_text = str(packet.raw_text or "").strip()
    fault_active = packet_has_active_fault(packet)
    app_fault_code = getattr(packet, "app_fault_code", 0)
    app_fault = application_fault_text(app_fault_code)
    identify_state = getattr(packet, "identify_state", None)
    identify_error = getattr(packet, "identify_error", None)

    if "FAULT DETECTED" in raw_text.upper():
        if not fault_active:
            return None
        signature = (
            f"detail:{packet.foc_state}:{packet.fault_flags:08X}:"
            f"{packet.fault_status1:04X}:{packet.vgs_status2:04X}:"
            f"{app_fault_code}:{_fault_detail_reason_signature(raw_text)}"
        )
        return signature, raw_text

    if raw_text.startswith("F,") or fault_active:
        if app_fault_code != 0:
            app_fault_text = f"应用故障={app_fault}(code={app_fault_code}) | "
        elif packet.foc_state == FOC_STATE_FAULT and packet.fault_flags == 0:
            app_fault_text = "应用故障=未上报故障码（旧固件） | "
        else:
            app_fault_text = ""
        text = (
            f"{packet.timestamp} ms | 故障摘要 | 状态={packet.foc_state} | "
            f"{app_fault_text}"
            f"FaultFlags=0x{packet.fault_flags:08X} | "
            f"FAULT1=0x{packet.fault_status1:04X} | "
            f"VGS2=0x{packet.vgs_status2:04X} | "
            f"编码器={_encoder_fault_text(packet.encoder_detected)}"
        )
        if identify_state is not None or identify_error is not None:
            text += (
                f" | 识别状态={identify_state if identify_state is not None else '--'}"
                f" | 识别错误={identify_error if identify_error is not None else '--'}"
            )
        signature = (
            f"summary:{packet.foc_state}:{packet.fault_flags:08X}:"
            f"{app_fault_code}:{packet.fault_status1:04X}:{packet.vgs_status2:04X}:"
            f"{1 if packet.encoder_detected is True else 0 if packet.encoder_detected is False else 2}:"
            f"{identify_state if identify_state is not None else 'x'}:"
            f"{identify_error if identify_error is not None else 'x'}"
        )
        return signature, text

    return None


def fault_error_log_entry(level: str, message: str) -> tuple[str, str] | None:
    if str(level).upper() != "ERROR":
        return None

    text = str(message).strip()
    lowered = text.lower()
    if not any(keyword in lowered or keyword in text for keyword in FAULT_ERROR_KEYWORDS):
        return None

    signature = f"error:{_normalize_fault_signature_text(text)}"
    return signature, f"错误提示 | {text}"


def log_line_text(level: str, message: str) -> str:
    level_text = {
        "INFO": "信息",
        "TX": "发送",
        "RX": "接收",
        "ERROR": "错误",
    }.get(level, level)
    return f"[{level_text}] {message}"


def is_data_stale(last_packet_received_at_ms: Optional[int], now_ms: int, threshold_ms: int = 1000) -> bool:
    if last_packet_received_at_ms is None:
        return False
    return (int(now_ms) - int(last_packet_received_at_ms)) > int(threshold_ms)


def save_gui_profile(path: Path | str, profile: GuiProfile):
    profile_path = Path(path)
    profile_path.parent.mkdir(parents=True, exist_ok=True)
    profile_path.write_text(json.dumps(profile.to_dict(), indent=2), encoding="utf-8")


def load_gui_profile(path: Path | str) -> GuiProfile:
    profile_path = Path(path)
    if not profile_path.exists():
        return GuiProfile()
    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError):
        return GuiProfile()
    if not isinstance(payload, dict):
        return GuiProfile()
    return GuiProfile.from_dict(payload)


def format_plot_csv(rows: list[dict[str, float]]) -> str:
    if not rows:
        return "timestamp_ms\n"
    fieldnames = list(rows[0].keys())
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().replace("\r\n", "\n")


# ── Current Stream Ring Buffer ────────────────────────────────────────────────

class CurrentStreamRing:
    """Thread-safe ring buffer for CurrentSample from the 2kHz binary stream.

    Capacity: 20000 samples (~10s at 2kHz).
    """
    def __init__(self, capacity: int = 20000):
        self._buffer: deque[CurrentSample] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._total_received: int = 0
        self._total_dropped: int = 0
        self._crc_errors: int = 0
        self._seq_gaps: int = 0

    def extend(self, samples: List[CurrentSample]) -> int:
        """Thread-safe append. Returns number of samples dropped due to overflow."""
        dropped = 0
        with self._lock:
            old_len = len(self._buffer)
            self._buffer.extend(samples)
            new_len = len(self._buffer)
            # dequeue with maxlen auto-drops oldest; approximate drop count
            expected = old_len + len(samples)
            if new_len < expected:
                dropped = expected - new_len
            self._total_received += len(samples)
            self._total_dropped += dropped
        return dropped

    def get_recent(self, count: int) -> List[CurrentSample]:
        """Get the most recent N samples (thread-safe snapshot)."""
        with self._lock:
            items = list(self._buffer)
            if count >= len(items):
                return items
            return items[-count:]

    def get_all(self) -> List[CurrentSample]:
        """Get all samples (thread-safe snapshot)."""
        with self._lock:
            return list(self._buffer)

    def stats(self) -> dict:
        with self._lock:
            return {
                "total_received": self._total_received,
                "total_dropped": self._total_dropped,
                "crc_errors": self._crc_errors,
                "seq_gaps": self._seq_gaps,
                "ring_fill": len(self._buffer),
                "ring_capacity": self._buffer.maxlen,
            }

    def update_parser_stats(self, crc_errors: int, seq_gaps: int):
        with self._lock:
            self._crc_errors = crc_errors
            self._seq_gaps = seq_gaps

    def clear(self):
        with self._lock:
            self._buffer.clear()
            self._total_received = 0
            self._total_dropped = 0
            self._crc_errors = 0
            self._seq_gaps = 0
