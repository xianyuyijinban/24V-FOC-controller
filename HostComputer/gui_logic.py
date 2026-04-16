from __future__ import annotations

import csv
import io
import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

try:
    from .data_parser import CommandBuilder, FOCDataPacket
except ImportError:
    from data_parser import CommandBuilder, FOCDataPacket


MODE_LABELS = {
    0: "Iq_ref (A)",
    1: "Speed (rad/s)",
    2: "Position (rad)",
}

MODE_NAMES = {
    0: "Torque",
    1: "Speed",
    2: "Position",
}

FOC_STATE_IDLE = 0
FOC_STATE_INIT = 1
FOC_STATE_PARAM_IDENTIFY = 2
FOC_STATE_READY = 3
FOC_STATE_RUNNING = 4
FOC_STATE_FAULT = 5

LOG_LEVELS = ("INFO", "TX", "RX", "ERROR")
PLOT_CHANNELS = ("angle", "speed", "Id", "Iq", "Id_ref", "Iq_ref", "Vd", "Vq")
DEFAULT_PROFILE_PATH = Path.home() / ".24v_foc_host_gui.json"


@dataclass
class LoopTuning:
    kp: float = 0.0
    ki: float = 0.0


@dataclass
class PositionLoopTuning:
    kp: float = 0.0
    kd: float = 0.0


def _default_current_tuning() -> LoopTuning:
    return LoopTuning(kp=0.2, ki=0.01)


def _default_speed_tuning() -> LoopTuning:
    return LoopTuning(kp=1.0, ki=0.1)


def _default_position_tuning() -> PositionLoopTuning:
    return PositionLoopTuning(kp=10.0, kd=0.10)


@dataclass
class GuiProfile:
    last_port: str = ""
    baud_rate: int = 115200
    selected_mode: int = 0
    log_filters: list[str] = field(default_factory=lambda: list(LOG_LEVELS))
    current_target: tuple[float, float] = (0.0, 0.0)
    speed_target: float = 0.0
    position_target: float = 0.0
    current_pi: LoopTuning = field(default_factory=_default_current_tuning)
    speed_pi: LoopTuning = field(default_factory=_default_speed_tuning)
    position_pd: PositionLoopTuning = field(default_factory=_default_position_tuning)

    def to_dict(self) -> dict:
        return {
            "last_port": self.last_port,
            "baud_rate": self.baud_rate,
            "selected_mode": self.selected_mode,
            "log_filters": list(self.log_filters),
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
        return profile


@dataclass
class HostAppState:
    is_connected: bool = False
    selected_mode: int = 0
    power_unlocked: bool = False
    motor_enabled: bool = False
    identify_active: bool = False
    foc_state: Optional[int] = None
    fault_active: bool = False
    available_ports: list[str] = field(default_factory=list)
    last_packet: Optional[FOCDataPacket] = None
    last_packet_received_at_ms: Optional[int] = None


class RollingPlotBuffer:
    def __init__(self, max_samples: int = 300):
        self.max_samples = max(1, int(max_samples))
        self._rows: deque[dict[str, float]] = deque(maxlen=self.max_samples)

    def append_packet(self, packet: FOCDataPacket):
        self._rows.append(
            {
                "timestamp_ms": float(packet.timestamp),
                "angle": float(packet.angle),
                "speed": float(packet.speed),
                "Id": float(packet.Id),
                "Iq": float(packet.Iq),
                "Id_ref": float(packet.Id_ref),
                "Iq_ref": float(packet.Iq_ref),
                "Vd": float(packet.Vd),
                "Vq": float(packet.Vq),
            }
        )

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
    return MODE_LABELS.get(mode, "Target")


def mode_name(mode: int) -> str:
    return MODE_NAMES.get(mode, "Custom")


def parse_float_field(
    raw_value: str,
    field_name: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    text = str(raw_value).strip()
    if not text:
        raise ValueError(f"{field_name} is required.")

    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"{field_name} must be a valid number.") from exc

    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite.")
    if minimum is not None and value < minimum:
        raise ValueError(f"{field_name} must be >= {minimum}.")
    if maximum is not None and value > maximum:
        raise ValueError(f"{field_name} must be <= {maximum}.")
    return value


def build_current_ref_command(id_raw: str, iq_raw: str) -> str:
    id_ref = parse_float_field(id_raw, "Id_ref")
    iq_ref = parse_float_field(iq_raw, "Iq_ref")
    return CommandBuilder.set_current_ref(id_ref, iq_ref)


def build_speed_ref_command(speed_raw: str) -> str:
    speed = parse_float_field(speed_raw, "Speed")
    return CommandBuilder.set_speed_ref(speed)


def build_position_ref_command(position_raw: str) -> str:
    position = parse_float_field(position_raw, "Position")
    return CommandBuilder.set_position_ref(position)


def build_loop_gain_command(loop_name: str, kp_raw: str, gain2_raw: str) -> str:
    kp = parse_float_field(kp_raw, f"{loop_name.title()} Kp", minimum=0.0)
    gain2_name = "Kd" if loop_name == "position" else "Ki"
    gain2 = parse_float_field(gain2_raw, f"{loop_name.title()} {gain2_name}", minimum=0.0)
    builders = {
        "current": CommandBuilder.set_current_pi,
        "speed": CommandBuilder.set_speed_pi,
        "position": CommandBuilder.set_position_pd,
    }
    if loop_name not in builders:
        raise ValueError(f"Unsupported loop gains: {loop_name}")
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


def apply_command_effects(state: HostAppState, command: str):
    text = command.strip()
    if text == "CMD:UNLOCK,1":
        state.power_unlocked = True
    elif text == "CMD:UNLOCK,0":
        state.power_unlocked = False
        state.motor_enabled = False
        state.identify_active = False
    elif text == "CMD:ENABLE,1":
        state.motor_enabled = True
    elif text == "CMD:ENABLE,0":
        state.motor_enabled = False
    elif text == "CMD:IDENTIFY,1":
        state.identify_active = True
    elif text == "CMD:IDENTIFY,0":
        state.identify_active = False
    elif text.startswith("CMD:MODE,"):
        try:
            state.selected_mode = int(text.split(",", 1)[1])
        except (IndexError, ValueError):
            pass


def apply_packet_effects(state: HostAppState, packet: FOCDataPacket):
    state.last_packet = packet
    state.foc_state = int(packet.foc_state)
    state.fault_active = bool(packet.is_fault_active)
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
    return {
        "timestamp": f"{packet.timestamp} ms",
        "angle": f"{packet.angle:.2f} deg",
        "speed": f"{packet.speed:.2f} rad/s",
        "currents": f"Id {packet.Id:.2f} A / Iq {packet.Iq:.2f} A",
        "refs": f"Id_ref {packet.Id_ref:.2f} / Iq_ref {packet.Iq_ref:.2f}",
        "voltages": f"Vd {packet.Vd:.2f} V / Vq {packet.Vq:.2f} V",
        "state": str(packet.foc_state),
    }


def fault_summary_text(packet: FOCDataPacket) -> dict[str, str]:
    return {
        "state": "ACTIVE" if packet.is_fault_active else "NORMAL",
        "fault1": f"FAULT1 0x{packet.fault_status1:04X}",
        "vgs2": f"VGS2 0x{packet.vgs_status2:04X}",
        "timestamp": f"{packet.timestamp} ms",
    }


def log_line_text(level: str, message: str) -> str:
    return f"[{level}] {message}"


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
