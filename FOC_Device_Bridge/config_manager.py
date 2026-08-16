"""
FOC Device Bridge — Configuration Manager

QSettings-based persistence for bridge preferences:
  - Last-used serial port and baud rate
  - Wheel config (count, strength, width, damping, limit)
  - Reverse direction toggle

Stored in %APPDATA%/FOC_Device_Bridge/settings.ini
"""

from PySide6.QtCore import QSettings


class BridgeConfig:
    """Persistent bridge configuration."""

    ORG = "FOC_Device_Bridge"
    APP = "Bridge"

    def __init__(self):
        self.last_port: str = ""
        self.last_baud: int = 1000000
        self.wheel_count: float = 24.0
        self.wheel_strength: float = 6.0
        self.wheel_width: float = 0.16
        self.wheel_damping: float = 0.10
        self.wheel_limit: float = 0.30
        self.reverse_direction: bool = False

    # ── Load / Save ────────────────────────────────────────────────

    @classmethod
    def load(cls) -> "BridgeConfig":
        cfg = cls()
        settings = QSettings(cls.ORG, cls.APP)
        cfg.last_port = str(settings.value("serial/last_port", ""))
        cfg.last_baud = int(settings.value("serial/last_baud", 1000000))
        cfg.wheel_count = float(settings.value("wheel/count", 24.0))
        cfg.wheel_strength = float(settings.value("wheel/strength", 6.0))
        cfg.wheel_width = float(settings.value("wheel/width", 0.16))
        cfg.wheel_damping = float(settings.value("wheel/damping", 0.10))
        cfg.wheel_limit = float(settings.value("wheel/limit", 0.30))
        cfg.reverse_direction = _to_bool(settings.value("wheel/reverse", False))
        return cfg

    def save(self):
        settings = QSettings(self.ORG, self.APP)
        settings.setValue("serial/last_port", self.last_port)
        settings.setValue("serial/last_baud", self.last_baud)
        settings.setValue("wheel/count", self.wheel_count)
        settings.setValue("wheel/strength", self.wheel_strength)
        settings.setValue("wheel/width", self.wheel_width)
        settings.setValue("wheel/damping", self.wheel_damping)
        settings.setValue("wheel/limit", self.wheel_limit)
        settings.setValue("wheel/reverse", self.reverse_direction)

    # ── Helpers ─────────────────────────────────────────────────────

    def wheel_config_dict(self) -> dict:
        """Return wheel config as a dict for IPC / command building."""
        return {
            "count": int(self.wheel_count),
            "strength": self.wheel_strength,
            "width": self.wheel_width,
            "damping": self.wheel_damping,
            "limit": self.wheel_limit,
        }

    def apply_wheel_config_dict(self, d: dict):
        """Update wheel config from a dict (from IPC WHEEL_CONFIG)."""
        if "count" in d:
            self.wheel_count = max(1.0, float(d["count"]))
        if "strength" in d:
            self.wheel_strength = max(0.0, float(d["strength"]))
        if "width" in d:
            self.wheel_width = max(0.01, float(d["width"]))
        if "damping" in d:
            self.wheel_damping = max(0.0, float(d["damping"]))
        if "limit" in d:
            self.wheel_limit = max(0.01, min(1.0, float(d["limit"])))
        if "reverse" in d:
            self.reverse_direction = bool(d["reverse"])


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)
