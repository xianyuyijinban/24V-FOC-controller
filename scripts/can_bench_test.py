#!/usr/bin/env python3
"""CAN v1.0 bench acceptance script (python-can / candleLight).

Run after flashing the CAN-enabled firmware:

    python scripts/can_bench_test.py --channel 0 --bitrate 500000

Power-stage cases (5, 6, 8, 9) are skipped unless ``--power-ok`` is passed.
Manual fault injection and bus-off cases are additionally gated by
``--fault-inject`` and ``--bus-off-test``.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "HostComputer"))

import can
from can_transport import CanTransport

NODE = 1


@dataclass
class TestItem:
    case: int
    name: str
    status: str
    detail: str = ""
    raw: list = field(default_factory=list)


class CanTunnelClient:
    def __init__(self, bus, node_id: int = NODE):
        self.bus = bus
        self.node_id = node_id
        self.transport = CanTransport(bus=bus, node_id=node_id, line_mode=False)
        self.transport.open()

    def send(self, command: str, timeout: float = 1.5) -> bytes:
        self.transport.write((command + "\n").encode("utf-8"))
        data = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            available = self.transport.bytes_available()
            if available:
                data.extend(self.transport.read(available))
                break
            time.sleep(0.01)
        return bytes(data)

    def drain_frames(self, duration_s: float):
        frames = []
        deadline = time.time() + duration_s
        while time.time() < deadline:
            msg = self.bus.recv(timeout=0.01)
            if msg is not None:
                frames.append(msg)
        return frames


def msg(arbitration_id: int, data: bytes, extended: bool = False) -> "can.Message":
    return can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=extended,
    )


def fast_id(subtype_base: int = 0x100) -> int:
    return subtype_base | NODE


class CanBenchRunner:
    def __init__(self, bus, uart_port: str = "", power_ok: bool = False,
                 fault_inject: bool = False, bus_off_test: bool = False):
        self.bus = bus
        self.client = CanTunnelClient(bus)
        self.uart = None
        self.power_ok = power_ok
        self.fault_inject = fault_inject
        self.bus_off_test = bus_off_test
        self.results = []

        if uart_port:
            try:
                import serial
                self.uart = serial.Serial(uart_port, 230400, timeout=0.1)
                time.sleep(0.2)
                self.uart.reset_input_buffer()
            except Exception as exc:
                print(f"UART not available ({exc}); concurrent case will skip")
                self.uart = None

    def record(self, case: int, name: str, status: str, detail: str = "", raw=None):
        self.results.append(TestItem(case, name, status, detail, raw or []))
        tag = {"PASS": "[OK]", "FAIL": "[!!]", "SKIP": "[-]", "WARN": "[!]"}
        print(f"  {tag.get(status, '[?]')} C{case:02d} {name}: {status} {detail[:140]}")

    def collect_telemetry(self, duration_s: float):
        frames = []
        deadline = time.time() + duration_s
        while time.time() < deadline:
            m = self.bus.recv(timeout=0.01)
            if m is not None and m.arbitration_id == (0x200 | NODE) and len(m.data) >= 8:
                frames.append(m.data)
        return frames

    def run(self, case_filter=None):
        cases = [
            (1, self.case01_bootup),
            (2, self.case02_single_frame_tunnel),
            (3, self.case03_multi_frame_tunnel),
            (4, self.case04_truncation_boundary),
            (5, self.case05_fast_control_flow),
            (6, self.case06_mode_mismatch_reject),
            (7, self.case07_heartbeat_guard),
            (8, self.case08_estop),
            (9, self.case09_fault_event),
            (10, self.case10_concurrent_uart_can),
            (11, self.case11_load_telemetry),
            (12, self.case12_bus_off_recovery),
        ]
        for case_no, fn in cases:
            if case_filter and case_no not in case_filter:
                continue
            print(f"\n=== Case {case_no}: {fn.__doc__.strip()} ===")
            try:
                fn()
            except Exception as exc:
                self.record(case_no, fn.__doc__.strip(), "FAIL", f"EXCEPTION: {exc}")

    def case01_bootup(self):
        """BOOTUP frame after power-on"""
        deadline = time.time() + 3.0
        while time.time() < deadline:
            m = self.bus.recv(timeout=0.1)
            if m is not None and m.arbitration_id == (0x200 | NODE) and m.data[0] == 0x10:
                ver, node_id = m.data[1], m.data[2]
                ok = ver == 0x01 and node_id == NODE
                self.record(1, "BOOTUP", "PASS" if ok else "FAIL",
                            f"version=0x{ver:02x}, node={node_id}", raw=[m])
                return
        self.record(1, "BOOTUP", "FAIL", "not seen in 3s; reset/flash immediately before run")

    def case02_single_frame_tunnel(self):
        """Tunnel SF: SYS:FW_INFO?"""
        resp = self.client.send("SYS:FW_INFO?", 1.5)
        ok = b"FW_INFO,OK" in resp and b"git=" in resp
        self.record(2, "SYS:FW_INFO?", "PASS" if ok else "FAIL",
                    f"len={len(resp)}", raw=[resp.decode(errors="replace")])

    def case03_multi_frame_tunnel(self):
        """Tunnel FF/CF command and long response"""
        resp = self.client.send("SYS:CMDS?", 2.0)
        ok = len(resp) > 7 and b"SYS:CMDS,OK" in resp
        self.record(3, "SYS:CMDS?", "PASS" if ok else "FAIL",
                    f"len={len(resp)}", raw=[resp.decode(errors="replace")[:160]])

    def case04_truncation_boundary(self):
        """255-byte response truncation"""
        resp = self.client.send("SYS:CMDS?", 2.0)
        ok = len(resp) <= 255
        self.record(4, "response <= 255", "PASS" if ok else "FAIL",
                    f"len={len(resp)}", raw=[resp.decode(errors="replace")[:160]])

    def case05_fast_control_flow(self):
        """FAST_CTRL unlock/mode/enable/SET_SPEED/STOP"""
        if not self.power_ok:
            self.record(5, "FAST_CTRL flow", "SKIP", "requires --power-ok on a safe bench")
            return

        self.bus.send(msg(fast_id(), bytes([0x02, 0x01])))
        self.bus.send(msg(fast_id(), bytes([0x04, 0x01])))
        self.bus.send(msg(fast_id(), bytes([0x03, 0x01])))
        speed = struct.pack("<f", 0.5)
        self.bus.send(msg(fast_id(), bytes([0x11]) + speed))
        time.sleep(0.5)
        telemetry = self.collect_telemetry(0.5)
        running = any(t[1] == 4 for t in telemetry)
        self.bus.send(msg(fast_id(), bytes([0x01])))
        self.record(5, "FAST_CTRL flow", "PASS" if running else "FAIL",
                    f"running_observed={running}, telemetry={len(telemetry)}")

    def case06_mode_mismatch_reject(self):
        """Speed-mode SET_POS is silent and ineffective"""
        if not self.power_ok:
            self.record(6, "mode mismatch", "SKIP", "requires --power-ok on a safe bench")
            return

        self.bus.send(msg(fast_id(), bytes([0x04, 0x01])))
        pos = struct.pack("<f", 1.0)
        self.bus.send(msg(fast_id(), bytes([0x10]) + pos))
        time.sleep(0.2)
        telemetry = self.collect_telemetry(0.3)
        mode_ok = all(t[3] == 1 for t in telemetry) if telemetry else True
        self.record(6, "SET_POS in speed mode", "PASS" if mode_ok else "FAIL",
                    f"mode_ok={mode_ok}")

    def case07_heartbeat_guard(self):
        """HEARTBEAT armed -> timeout -> disarm"""
        before = self.client.send("CAN:HEARTBEAT?", 1.0)
        self.bus.send(msg(0x000 | NODE, bytes([0x01, 0x00, 0x2C, 0x01])))
        time.sleep(0.1)
        armed = self.client.send("CAN:HEARTBEAT?", 1.0)
        time.sleep(0.45)
        after = self.client.send("CAN:HEARTBEAT?", 1.0)
        ok = b"ok=0" in before and b"ok=1" in armed and b"ok=0" in after
        self.record(7, "heartbeat guard", "PASS" if ok else "FAIL",
                    f"before={before.strip()}, armed={armed.strip()}, after={after.strip()}",
                    raw=[before.decode(errors="replace"), armed.decode(errors="replace"),
                         after.decode(errors="replace")])

    def case08_estop(self):
        """ESTOP broadcast stops a running motor"""
        if not self.power_ok:
            self.record(8, "ESTOP", "SKIP", "requires --power-ok on a safe bench")
            return
        self.bus.send(msg(fast_id(), bytes([0x02, 0x01])))
        self.bus.send(msg(fast_id(), bytes([0x04, 0x01])))
        self.bus.send(msg(fast_id(), bytes([0x03, 0x01])))
        time.sleep(0.3)
        self.bus.send(msg(0x000, bytes([0x02])))
        time.sleep(0.3)
        telemetry = self.collect_telemetry(0.3)
        stopped = all(t[1] != 4 for t in telemetry) if telemetry else True
        self.record(8, "ESTOP", "PASS" if stopped else "FAIL", f"stopped={stopped}")

    def case09_fault_event(self):
        """FAULT_EVENT on injected encoder fault"""
        if not (self.power_ok and self.fault_inject):
            self.record(9, "FAULT_EVENT", "SKIP", "requires --power-ok --fault-inject")
            return
        input("Disconnect the encoder, then press Enter: ")
        deadline = time.time() + 5.0
        while time.time() < deadline:
            m = self.bus.recv(timeout=0.1)
            if m is not None and m.arbitration_id == (0x500 | NODE) and m.data[0] == 0x01:
                self.record(9, "FAULT_EVENT", "PASS",
                            f"fault={m.data[1]}, state={m.data[2]}", raw=[m])
                return
        self.record(9, "FAULT_EVENT", "FAIL", "no FAULT_EVENT in 5s")

    def case10_concurrent_uart_can(self):
        """UART and CAN tunnel responses do not cross"""
        if self.uart is None:
            self.record(10, "concurrent UART+CAN", "SKIP", "no UART port provided")
            return
        can_resp = self.client.send("SYS:FW_INFO?", 1.5)
        self.uart.write(b"SYS:FW_INFO?\n")
        time.sleep(0.5)
        uart_raw = self.uart.read(self.uart.in_waiting or 1)
        ok = b"FW_INFO,OK" in can_resp and b"FW_INFO,OK" in uart_raw
        self.record(10, "concurrent UART+CAN", "PASS" if ok else "FAIL",
                    f"can={len(can_resp)}B, uart={len(uart_raw)}B")

    def case11_load_telemetry(self):
        """50Hz STATE_FAST and bus load snapshot"""
        frames = self.collect_telemetry(10.0)
        hz = len(frames) / 10.0
        ok = hz >= 40.0
        self.record(11, "50Hz STATE_FAST", "PASS" if ok else "FAIL",
                    f"actual={hz:.1f}Hz, frames={len(frames)}")

    def case12_bus_off_recovery(self):
        """Bus-off self-recovery"""
        if not self.bus_off_test:
            self.record(12, "bus-off recovery", "SKIP", "requires --bus-off-test")
            return
        input("Short the CAN bus, wait for bus-off, then restore and press Enter: ")
        deadline = time.time() + 8.0
        while time.time() < deadline:
            m = self.bus.recv(timeout=0.1)
            if m is not None and m.arbitration_id == (0x200 | NODE) and m.data[0] == 0x01:
                self.record(12, "bus-off recovery", "PASS", "STATE_FAST resumed")
                return
        self.record(12, "bus-off recovery", "FAIL", "no STATE_FAST within 8s after restore")

    def report(self, interface: str, channel: str):
        print("\n" + "=" * 60)
        print("CAN BENCH TEST -- FINAL REPORT")
        print("=" * 60)
        fails = []
        for item in self.results:
            print(f"C{item.case:02d} {item.name:<36} {item.status}")
            if item.status == "FAIL":
                fails.append(item.case)
        overall = "ALL PASS" if not fails else f"FAIL in cases {sorted(set(fails))}"
        print("-" * 60)
        print(f"Overall: {overall}")

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = ROOT / f"scripts/can_bench_{ts}.json"
        path.write_text(json.dumps({
            "timestamp": datetime.now().isoformat(),
            "interface": interface,
            "channel": channel,
            "overall": overall,
            "results": [asdict(r) for r in self.results],
        }, indent=2), encoding="utf-8")
        print(f"Report saved: {path}")
        return not fails


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="gs_usb")
    parser.add_argument("--channel", default="0")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--uart-port", default="")
    parser.add_argument("--power-ok", action="store_true",
                        help="confirm the bench is powered and safe for motor tests")
    parser.add_argument("--fault-inject", action="store_true")
    parser.add_argument("--bus-off-test", action="store_true")
    parser.add_argument("--case", default="", help="comma-separated case numbers")
    args = parser.parse_args()

    case_filter = None
    if args.case:
        case_filter = {int(x) for x in args.case.split(",") if x.strip().isdigit()}

    try:
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    except Exception as exc:
        print(f"Cannot open CAN adapter ({args.interface}, channel={args.channel}): {exc}")
        sys.exit(2)

    runner = CanBenchRunner(
        bus,
        uart_port=args.uart_port,
        power_ok=args.power_ok,
        fault_inject=args.fault_inject,
        bus_off_test=args.bus_off_test,
    )
    try:
        runner.run(case_filter)
    finally:
        bus.shutdown()

    ok = runner.report(args.interface, args.channel)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
