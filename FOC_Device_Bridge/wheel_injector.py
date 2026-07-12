"""
FOC Device Bridge — Mouse Wheel Injector

Abstract WheelInjector + Win32 SendInput implementation.
1 delta_step → 1 wheel notch → WHEEL_DELTA (120) in dwData.
"""

import ctypes
from abc import ABC, abstractmethod
from ctypes import wintypes

# ── Win32 SendInput types ──────────────────────────────────────────

INPUT_MOUSE = 0
MOUSEEVENTF_WHEEL = 0x0800
WHEEL_DELTA = 120


class MOUSEINPUT(ctypes.Structure):
    _fields_ = [
        ("dx",          wintypes.LONG),
        ("dy",          wintypes.LONG),
        ("mouseData",   wintypes.DWORD),
        ("dwFlags",     wintypes.DWORD),
        ("time",        wintypes.DWORD),
        ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong)),
    ]


class INPUT(ctypes.Structure):
    _fields_ = [
        ("type", wintypes.DWORD),
        ("mi",   MOUSEINPUT),
    ]


# ── Injector ABC ────────────────────────────────────────────────────

class WheelInjector(ABC):
    """Abstract interface for wheel injection."""

    @abstractmethod
    def inject(self, delta_steps: int) -> None:
        """Inject `delta_steps` mouse wheel notches."""
        ...


class Win32WheelInjector(WheelInjector):
    """Inject mouse wheel events via Win32 SendInput.

    Each delta_step → WHEEL_DELTA (120) in dwData.
    reverse=True negates the direction.
    """

    def __init__(self, reverse: bool = False):
        self._reverse = bool(reverse)

    def set_reverse(self, reverse: bool):
        self._reverse = bool(reverse)

    def inject(self, delta_steps: int) -> None:
        if delta_steps == 0:
            return
        steps = -delta_steps if self._reverse else delta_steps
        dw_data = steps * WHEEL_DELTA

        inp = INPUT()
        inp.type = INPUT_MOUSE
        inp.mi.dwFlags = MOUSEEVENTF_WHEEL
        inp.mi.mouseData = dw_data

        try:
            ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))
        except Exception:
            pass  # SendInput should never fail, but guard anyway


class FakeWheelInjector(WheelInjector):
    """Records injection calls for testing."""

    def __init__(self, reverse: bool = False):
        self.calls: list[int] = []
        self._reverse = bool(reverse)

    def set_reverse(self, reverse: bool):
        self._reverse = bool(reverse)

    def inject(self, delta_steps: int) -> None:
        steps = -delta_steps if self._reverse else delta_steps
        self.calls.append(steps)

    def clear(self):
        self.calls.clear()
