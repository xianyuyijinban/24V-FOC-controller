from __future__ import annotations

import csv
import io
import json
import math
import re
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

try:
    from .data_parser import CommandBuilder, FOCDataPacket, control_to_user_angle
except ImportError:
    from data_parser import CommandBuilder, FOCDataPacket, control_to_user_angle


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
    baud_rate: int = 230400
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
    selected_mode: int = 0
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
    speed = parse_float_field(speed_raw, "速度")
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
    return {
        "can_unlock": connected and not unlocked,
        "can_lock": connected and unlocked,
        "can_enable": connected and unlocked and not enabled and not identify_active and not fault_active,
        "can_disable": connected and enabled and not fault_active,
        "can_clear_fault": connected,
        "can_identify_start": connected and unlocked and not identify_active and not enabled and not fault_active,
        "can_identify_stop": connected and identify_active,
        "can_send_target": connected and not identify_active and not fault_active,
    }


def connection_command_state(is_connected: bool) -> dict[str, bool]:
    state = button_enable_state(HostAppState(is_connected=is_connected))
    state["can_identify"] = state["can_identify_start"] or state["can_identify_stop"]
    return state


def should_confirm_stall_mode_enable(state: HostAppState) -> bool:
    fault_active = bool(state.fault_active or state.foc_state == FOC_STATE_FAULT)
    requires_stall_mode = (not bool(state.motor_identified)) or (state.encoder_detected is False)
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
    missing_encoder = (state.encoder_detected is False)

    if missing_identify and missing_encoder:
        return "当前电机尚未完成识别，且未检测到 TLE5012 编码器。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"
    if missing_encoder:
        return "当前未检测到 TLE5012 编码器。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"
    return "当前电机尚未完成识别。是否进入堵转模式（开环试转）？使能后建议先给小 Iq，再给小 speed。"


def apply_command_effects(state: HostAppState, command: str):
    text = command.strip()
    if text == "CMD:UNLOCK,1":
        state.power_unlocked = True
    elif text == "CMD:UNLOCK,0":
        state.power_unlocked = False
        state.motor_enabled = False
        state.identify_active = False
        state.stall_mode_armed = False
        state.stall_open_loop_active = False
    elif text == "CMD:ENABLE,1":
        state.motor_enabled = True
    elif text == "CMD:ENABLE,0":
        state.motor_enabled = False
        state.stall_open_loop_active = False
    elif text == "CMD:IDENTIFY,1":
        state.identify_active = True
    elif text == "CMD:IDENTIFY,0":
        state.identify_active = False
    elif text == "CMD:STALL_MODE,1":
        state.stall_mode_armed = True
    elif text == "CMD:STALL_MODE,0":
        state.stall_mode_armed = False
        state.stall_open_loop_active = False
    elif text.startswith("CMD:MODE,"):
        try:
            state.selected_mode = int(text.split(",", 1)[1])
        except (IndexError, ValueError):
            pass


def apply_packet_effects(state: HostAppState, packet: FOCDataPacket):
    state.last_packet = packet
    state.foc_state = int(packet.foc_state)
    if packet.control_mode is not None:
        state.selected_mode = int(packet.control_mode)
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
    return bool(state.is_connected) and not bool(state.motor_enabled) and not bool(state.identify_active)


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
