try:
    from .data_parser import FOCDataPacket
except ImportError:
    from data_parser import FOCDataPacket


MODE_LABELS = {
    0: "Iq_ref (A)",
    1: "Speed (rad/s)",
    2: "Position (rad)",
}


def mode_target_label(mode: int) -> str:
    return MODE_LABELS.get(mode, "Target")


def connection_command_state(is_connected: bool) -> dict[str, bool]:
    enabled = bool(is_connected)
    return {
        "can_unlock": enabled,
        "can_lock": enabled,
        "can_enable": enabled,
        "can_disable": enabled,
        "can_clear_fault": enabled,
        "can_identify": enabled,
        "can_send_target": enabled,
    }


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
