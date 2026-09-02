"""Measure STM32 FOC runtime with the persistent DWT profiler.

The script is intentionally ASCII-only so it runs cleanly in Windows GBK
terminals. It supports mixed ASCII telemetry and 25-byte binary current frames.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional

import serial


BAUD_RATE = 1_000_000
TIM1_BUDGET_US = 50.0
CURRENT_FRAME_SIZE = 25
CURRENT_SYNC = b"\xA5\x5A"
WHEEL_SESSION_ID = 0x465043

DEFAULT_SCENARIOS = [
    "READY_IDLE",
    "RAW_SPEED_POS",
    "RAW_SPEED_NEG",
    "POSITION_HOLD",
    "DETENT_MANUAL",
    "SCROLL_WHEEL_MANUAL",
    "SPEED_BIN1000",
]


def crc8_poly07(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class MixedStreamDecoder:
    """Split ASCII lines from interleaved binary current-stream frames."""

    def __init__(self) -> None:
        self._raw = bytearray()
        self._text = bytearray()
        self.binary_frames = 0
        self.binary_crc_errors = 0

    def feed(self, data: bytes) -> List[str]:
        self._raw.extend(data)
        lines: List[str] = []

        while self._raw:
            if self._raw.startswith(CURRENT_SYNC):
                if len(self._raw) < CURRENT_FRAME_SIZE:
                    break
                frame = bytes(self._raw[:CURRENT_FRAME_SIZE])
                if frame[2] == 0x43 and frame[3] == 20 and crc8_poly07(frame[:-1]) == frame[-1]:
                    del self._raw[:CURRENT_FRAME_SIZE]
                    self.binary_frames += 1
                    continue
                del self._raw[0]
                self.binary_crc_errors += 1
                continue

            sync_at = self._raw.find(CURRENT_SYNC)
            text_end = sync_at if sync_at >= 0 else len(self._raw)
            if sync_at < 0 and self._raw[-1] == CURRENT_SYNC[0]:
                text_end -= 1
            if text_end <= 0:
                break

            self._text.extend(self._raw[:text_end])
            del self._raw[:text_end]
            while b"\n" in self._text:
                raw_line, _, remainder = self._text.partition(b"\n")
                self._text = bytearray(remainder)
                line = raw_line.rstrip(b"\r").decode("ascii", errors="replace")
                if line:
                    lines.append(line)

        return lines


@dataclass(frozen=True)
class UartLinkStats:
    rx_errors: int = 0
    restart_failures: int = 0
    tx_p0_drops: int = 0
    tx_p1_drops: int = 0
    tx_p2_drops: int = 0

    def delta_from(self, before: "UartLinkStats") -> "UartLinkStats":
        return UartLinkStats(
            rx_errors=(self.rx_errors - before.rx_errors) & 0xFFFFFFFF,
            restart_failures=(self.restart_failures - before.restart_failures) & 0xFFFFFFFF,
            tx_p0_drops=(self.tx_p0_drops - before.tx_p0_drops) & 0xFFFFFFFF,
            tx_p1_drops=(self.tx_p1_drops - before.tx_p1_drops) & 0xFFFFFFFF,
            tx_p2_drops=(self.tx_p2_drops - before.tx_p2_drops) & 0xFFFFFFFF,
        )


class FocSerial:
    def __init__(self, port: str, baud: int, serial_port: Optional[Any] = None) -> None:
        self.ser = serial_port if serial_port is not None else serial.Serial(
            port, baudrate=baud, timeout=0.01
        )
        self.decoder = MixedStreamDecoder()
        self.latest_n_frame: Optional[List[str]] = None
        self.all_lines: List[str] = []
        self.last_uart_stats: Optional[UartLinkStats] = None

    def close(self) -> None:
        self.ser.close()

    def write(self, command: str) -> None:
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()

    def _read_available(self) -> List[str]:
        waiting = self.ser.in_waiting
        data = self.ser.read(waiting if waiting > 0 else 1)
        lines = self.decoder.feed(data) if data else []
        for line in lines:
            self.all_lines.append(line)
            if line.startswith("N,"):
                self.latest_n_frame = line.split(",")
            elif line.startswith("UART_RX,OK"):
                self.last_uart_stats = parse_uart_stats(line)
        return lines

    def _drain_pending_input(self) -> List[str]:
        """Decode already-buffered traffic without using it as a future ACK."""
        lines: List[str] = []
        while self.ser.in_waiting > 0:
            lines.extend(self._read_available())
        return lines

    def _query_uart_stats_best_effort(self, timeout: float = 0.25) -> Optional[UartLinkStats]:
        """Collect timeout evidence without retrying the state-changing command."""
        self.write("DIAG:UART_RX?")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self._read_available():
                if line.startswith("UART_RX,OK"):
                    return parse_uart_stats(line)
            time.sleep(0.001)
        return self.last_uart_stats

    def read_for(self, seconds: float, keepalive: Optional[Callable[[], None]] = None) -> List[str]:
        deadline = time.monotonic() + seconds
        next_keepalive = time.monotonic()
        lines: List[str] = []
        while time.monotonic() < deadline:
            lines.extend(self._read_available())
            now = time.monotonic()
            if keepalive is not None and now >= next_keepalive:
                keepalive()
                next_keepalive = now + 0.4
            time.sleep(0.001)
        lines.extend(self._read_available())
        return lines

    def command(self, command: str, expected: str, timeout: float = 1.0) -> List[str]:
        self._drain_pending_input()
        frames_before = self.decoder.binary_frames
        crc_before = self.decoder.binary_crc_errors
        stats_before = self.last_uart_stats
        self.write(command)
        deadline = time.monotonic() + timeout
        lines: List[str] = []
        while time.monotonic() < deadline:
            new_lines = self._read_available()
            lines.extend(new_lines)
            if any(line.startswith(expected) for line in new_lines):
                return lines
            time.sleep(0.001)
        stats_after = self.last_uart_stats
        if command not in {"DIAG:UART_RX?", "CMD:UART_RX?", "CMD:UART_RX_STAT?"}:
            stats_after = self._query_uart_stats_best_effort()
        stats_delta = None
        if stats_before is not None and stats_after is not None:
            stats_delta = stats_after.delta_from(stats_before)
        raise RuntimeError(
            f"ACK timeout: command={command!r}, expected_prefix={expected!r}, "
            f"recent_lines={lines[-10:]}, "
            f"binary_frames={self.decoder.binary_frames - frames_before}, "
            f"binary_crc_errors={self.decoder.binary_crc_errors - crc_before}, "
            f"uart_delta={stats_delta}, last_uart_stats={stats_after}"
        )

    def query_foc_time(self, timeout: float = 2.0) -> List[str]:
        self._drain_pending_input()
        self.write("DIAG:FOC_TIME?")
        deadline = time.monotonic() + timeout
        lines: List[str] = []
        started = False
        while time.monotonic() < deadline:
            new_lines = self._read_available()
            for line in new_lines:
                if line.startswith("FOC_TIME,BEGIN"):
                    started = True
                    lines = [line]
                elif started and line.startswith("FOC_TIME,"):
                    lines.append(line)
                    if line.startswith("FOC_TIME,END"):
                        return lines
            time.sleep(0.001)
        raise RuntimeError(f"FOC_TIME query timeout: lines={lines}")

    def query_uart_stats(self) -> UartLinkStats:
        lines = self.command("DIAG:UART_RX?", "UART_RX,OK", timeout=1.0)
        for line in lines:
            if line.startswith("UART_RX,OK"):
                return parse_uart_stats(line)
        raise RuntimeError("UART_RX response missing after matching ACK")

    def query_uart_errors(self) -> int:
        return self.query_uart_stats().rx_errors

    def latest_fault_code(self) -> Optional[int]:
        fields = self.latest_n_frame
        if fields is None or len(fields) <= 14:
            return None
        try:
            return int(fields[14], 0)
        except ValueError:
            return None


def parse_key_values(line: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for field in line.split(","):
        if "=" in field:
            key, value = field.split("=", 1)
            values[key] = value
    return values


def parse_uart_stats(line: str) -> UartLinkStats:
    values = parse_key_values(line)
    return UartLinkStats(
        rx_errors=int(values.get("err", "0"), 0),
        restart_failures=int(values.get("restart_fail", "0"), 0),
        tx_p0_drops=int(values.get("tx_p0_drop", "0"), 0),
        tx_p1_drops=int(values.get("tx_p1_drop", "0"), 0),
        tx_p2_drops=int(values.get("tx_p2_drop", "0"), 0),
    )


@dataclass
class ProbeResult:
    scenario: str
    repeat: int
    probe: str
    count: int
    rate_hz: float
    min_cycles: int
    avg_cycles: int
    max_cycles: int
    min_us: float
    avg_us: float
    max_us: float
    budget_us: float
    overrun: int
    irq_jitter_us: float
    uart_errors: Optional[int]
    tx_p0_drop_delta: Optional[int]
    tx_p1_drop_delta: Optional[int]
    tx_p2_drop_delta: Optional[int]
    fault_code: Optional[int]
    status: str = "N/A"
    avg_cpu_pct: Optional[float] = None
    max_cpu_pct: Optional[float] = None
    remaining_us: Optional[float] = None
    outlier_candidate: bool = False
    note: str = ""


@dataclass
class ScenarioRun:
    scenario: str
    repeat: int
    duration_s: float
    profiler_lines: List[str]
    uart_errors: Optional[int]
    tx_p0_drop_delta: Optional[int]
    tx_p1_drop_delta: Optional[int]
    tx_p2_drop_delta: Optional[int]
    fault_code: Optional[int]
    binary_frames: int
    binary_crc_errors: int
    results: List[ProbeResult] = field(default_factory=list)


def classify_tim1(max_us: float, overrun: int) -> str:
    if overrun > 0 or max_us >= TIM1_BUDGET_US:
        return "FAIL"
    if max_us >= 40.0:
        return "RED"
    if max_us >= 35.0:
        return "YELLOW"
    return "GREEN"


def parse_profiler_run(run: ScenarioRun) -> None:
    jitter_us = 0.0
    for line in run.profiler_lines:
        if line.startswith("FOC_TIME,END"):
            jitter_us = float(parse_key_values(line).get("jitter_us", "0"))

    for line in run.profiler_lines:
        parts = line.split(",")
        if len(parts) < 3 or parts[0] != "FOC_TIME" or parts[1] in {"BEGIN", "END"}:
            continue
        values = parse_key_values(line)
        probe = parts[1]
        result = ProbeResult(
            scenario=run.scenario,
            repeat=run.repeat,
            probe=probe,
            count=int(values.get("n", "0")),
            rate_hz=int(values.get("n", "0")) / run.duration_s if run.duration_s > 0.0 else 0.0,
            min_cycles=int(values.get("min_cyc", "0")),
            avg_cycles=int(values.get("avg_cyc", "0")),
            max_cycles=int(values.get("max_cyc", "0")),
            min_us=float(values.get("min_us", "0")),
            avg_us=float(values.get("avg_us", "0")),
            max_us=float(values.get("max_us", "0")),
            budget_us=float(values.get("budget_us", "0")),
            overrun=int(values.get("overrun", "0")),
            irq_jitter_us=jitter_us,
            uart_errors=run.uart_errors,
            tx_p0_drop_delta=run.tx_p0_drop_delta,
            tx_p1_drop_delta=run.tx_p1_drop_delta,
            tx_p2_drop_delta=run.tx_p2_drop_delta,
            fault_code=run.fault_code,
        )
        if probe == "TIM1_ISR":
            result.status = classify_tim1(result.max_us, result.overrun)
            result.avg_cpu_pct = result.avg_us * 100.0 / TIM1_BUDGET_US
            result.max_cpu_pct = result.max_us * 100.0 / TIM1_BUDGET_US
            result.remaining_us = TIM1_BUDGET_US - result.max_us
            if not 18000.0 <= result.rate_hz <= 22000.0:
                result.note = f"TIM1 ISR rate outside 20kHz tolerance: {result.rate_hz:.1f}Hz"
        elif probe == "FOC_RUN":
            if run.scenario == "READY_IDLE" and result.count != 0:
                result.note = "FOC_RUN should be inactive in READY_IDLE"
            elif run.scenario != "READY_IDLE" and not 8500.0 <= result.rate_hz <= 11500.0:
                result.note = f"FOC_RUN rate outside 10kHz tolerance: {result.rate_hz:.1f}Hz"
        run.results.append(result)


def send_stop(link: FocSerial) -> None:
    try:
        link.command("TELEM:CUR,OFF", "CUR_STREAM,OK", timeout=1.0)
    except RuntimeError:
        pass
    link.command("CMD:STOP", "CTRL:STOP,OK", timeout=1.0)


def prepare_power(link: FocSerial) -> None:
    link.command("CMD:UNLOCK,1", "UNLOCK,OK,1")


def setup_speed(link: FocSerial, target: float, binary_stream: bool = False) -> Optional[Callable[[], None]]:
    send_stop(link)
    prepare_power(link)
    link.command("CMD:APP_MODE,RAW", "APP_MODE,OK,RAW")
    link.command("CMD:MODE,1", "MODE,OK,1")
    link.command(f"CMD:SREF,{target:.3f}", "SREF,OK")
    link.command("CMD:ENABLE,1", "ENABLE,OK,1")
    if binary_stream:
        link.command("TELEM:CUR,BIN,1000", "CUR_STREAM,OK")
    return None


def setup_position_hold(link: FocSerial) -> Optional[Callable[[], None]]:
    send_stop(link)
    prepare_power(link)
    link.command("CMD:APP_MODE,RAW", "APP_MODE,OK,RAW")
    link.command("CMD:MODE,2", "MODE,OK,2")
    link.command("CMD:ENABLE,1", "ENABLE,OK,1")
    return None


def setup_detent(link: FocSerial) -> Optional[Callable[[], None]]:
    send_stop(link)
    prepare_power(link)
    link.command("CMD:APP_MODE,DETENT", "APP_MODE,OK,DETENT")
    link.command("CMD:ENABLE,1", "ENABLE,OK,1")
    return None


def setup_scroll_wheel(link: FocSerial) -> Callable[[], None]:
    send_stop(link)
    prepare_power(link)
    link.command("CMD:APP_MODE,SCROLL_WHEEL", "APP_MODE,OK,SCROLL_WHEEL")
    link.command(f"WHEEL:SESSION,{WHEEL_SESSION_ID},1000", "WHEEL:SESSION,OK")
    link.command("CMD:ENABLE,1", "ENABLE,OK,1")

    def keepalive() -> None:
        link.write(f"WHEEL:KEEPALIVE,{WHEEL_SESSION_ID}")

    return keepalive


def setup_ready_idle(link: FocSerial) -> Optional[Callable[[], None]]:
    send_stop(link)
    return None


SCENARIO_SETUP: Dict[str, Callable[[FocSerial], Optional[Callable[[], None]]]] = {
    "READY_IDLE": setup_ready_idle,
    "RAW_SPEED_POS": lambda link: setup_speed(link, 0.5),
    "RAW_SPEED_NEG": lambda link: setup_speed(link, -0.5),
    "POSITION_HOLD": setup_position_hold,
    "DETENT_MANUAL": setup_detent,
    "SCROLL_WHEEL_MANUAL": setup_scroll_wheel,
    "SPEED_BIN1000": lambda link: setup_speed(link, 0.5, binary_stream=True),
}


def prompt_manual(scenario: str, manual_mode: str) -> bool:
    if not scenario.endswith("_MANUAL"):
        return True
    if manual_mode == "skip":
        return False
    if manual_mode == "prompt":
        input(f"Prepare to rotate the motor continuously for {scenario}. Press Enter to start...")
    return True


def run_scenario(
    link: FocSerial,
    scenario: str,
    repeat: int,
    duration: float,
    warmup: float,
    manual_mode: str,
) -> Optional[ScenarioRun]:
    if not prompt_manual(scenario, manual_mode):
        print(f"SKIP {scenario} repeat {repeat}")
        return None

    print(f"RUN  {scenario} repeat {repeat}: warmup={warmup:.1f}s capture={duration:.1f}s")
    link.latest_n_frame = None
    binary_before = link.decoder.binary_frames
    crc_before = link.decoder.binary_crc_errors
    keepalive = SCENARIO_SETUP[scenario](link)
    link.read_for(warmup, keepalive=keepalive)
    uart_stats_before = link.query_uart_stats()
    link.command("DIAG:FOC_TIME,CLEAR", "FOC_TIME,CLEAR,OK")
    link.read_for(duration, keepalive=keepalive)

    if scenario == "SPEED_BIN1000":
        link.command("TELEM:CUR,OFF", "CUR_STREAM,OK", timeout=1.5)

    profiler_lines = link.query_foc_time()
    uart_stats_after = link.query_uart_stats()
    uart_delta = uart_stats_after.delta_from(uart_stats_before)
    link.read_for(0.2, keepalive=keepalive)
    fault_code = link.latest_fault_code()
    send_stop(link)

    run = ScenarioRun(
        scenario=scenario,
        repeat=repeat,
        duration_s=duration,
        profiler_lines=profiler_lines,
        uart_errors=uart_delta.rx_errors,
        tx_p0_drop_delta=uart_delta.tx_p0_drops,
        tx_p1_drop_delta=uart_delta.tx_p1_drops,
        tx_p2_drop_delta=uart_delta.tx_p2_drops,
        fault_code=fault_code,
        binary_frames=link.decoder.binary_frames - binary_before,
        binary_crc_errors=link.decoder.binary_crc_errors - crc_before,
    )
    parse_profiler_run(run)
    return run


def mark_outliers(results: List[ProbeResult]) -> None:
    grouped: Dict[tuple[str, str], List[ProbeResult]] = {}
    for result in results:
        grouped.setdefault((result.scenario, result.probe), []).append(result)

    for group in grouped.values():
        if len(group) < 3:
            continue
        maxima = [item.max_us for item in group]
        for item in group:
            peers = [value for value, other in zip(maxima, group) if other is not item]
            peer_median = statistics.median(peers)
            if peer_median > 0.0 and item.max_us > peer_median * 1.2:
                item.outlier_candidate = True
                outlier_note = "max_us exceeds peer median by >20%; keep as preemption/anomaly candidate"
                item.note = f"{item.note}; {outlier_note}" if item.note else outlier_note


def write_csv(path: Path, results: Iterable[ProbeResult]) -> None:
    fieldnames = list(ProbeResult.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow(result.__dict__)


def fmt_optional(value: Optional[float], decimals: int = 3) -> str:
    return "" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{decimals}f}"


def write_markdown(path: Path, runs: List[ScenarioRun], results: List[ProbeResult]) -> None:
    tim1_results = [result for result in results if result.probe == "TIM1_ISR"]
    timing_ok = bool(tim1_results) and all(item.status != "FAIL" for item in tim1_results)
    communication_ok = bool(runs) and all(
        run.uart_errors == 0
        and run.tx_p0_drop_delta == 0
        and run.binary_crc_errors == 0
        for run in runs
    )
    overall = "PASS" if timing_ok and communication_ok else "FAIL"

    with path.open("w", encoding="utf-8") as handle:
        handle.write("# FOC Runtime Profile\n\n")
        handle.write(f"- Generated: {datetime.now().isoformat(timespec='seconds')}\n")
        handle.write(f"- Overall deadline result: **{overall}**\n")
        handle.write(f"- TIM1 hard budget: {TIM1_BUDGET_US:.3f} us\n\n")
        handle.write("## TIM1 ISR Summary\n\n")
        handle.write("| Scenario | Repeat | Avg us | Max us | Avg CPU % | Max CPU % | Remaining us | Overrun | Status | Outlier |\n")
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---|---|\n")
        for item in tim1_results:
            handle.write(
                f"| {item.scenario} | {item.repeat} | {item.avg_us:.3f} | {item.max_us:.3f} | "
                f"{fmt_optional(item.avg_cpu_pct, 1)} | {fmt_optional(item.max_cpu_pct, 1)} | "
                f"{fmt_optional(item.remaining_us)} | {item.overrun} | {item.status} | "
                f"{'YES' if item.outlier_candidate else ''} |\n"
            )

        handle.write("\n## All Probes\n\n")
        handle.write("| Scenario | Repeat | Probe | Count | Rate Hz | Min us | Avg us | Max us | Budget us | Overrun |\n")
        handle.write("|---|---:|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for item in results:
            handle.write(
                f"| {item.scenario} | {item.repeat} | {item.probe} | {item.count} | {item.rate_hz:.1f} | "
                f"{item.min_us:.3f} | {item.avg_us:.3f} | {item.max_us:.3f} | "
                f"{item.budget_us:.3f} | {item.overrun} |\n"
            )

        handle.write("\n## Communication And Fault Checks\n\n")
        handle.write(
            "| Scenario | Repeat | UART errors | P0 drops | P1 drops | P2 drops | "
            "Fault code | Binary frames | Binary CRC errors |\n"
        )
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for run in runs:
            handle.write(
                f"| {run.scenario} | {run.repeat} | {run.uart_errors if run.uart_errors is not None else ''} | "
                f"{run.tx_p0_drop_delta if run.tx_p0_drop_delta is not None else ''} | "
                f"{run.tx_p1_drop_delta if run.tx_p1_drop_delta is not None else ''} | "
                f"{run.tx_p2_drop_delta if run.tx_p2_drop_delta is not None else ''} | "
                f"{run.fault_code if run.fault_code is not None else ''} | {run.binary_frames} | "
                f"{run.binary_crc_errors} |\n"
            )

        outliers = [item for item in results if item.outlier_candidate]
        if outliers:
            handle.write("\n## Retained Outlier Candidates\n\n")
            for item in outliers:
                handle.write(f"- {item.scenario} repeat {item.repeat} {item.probe}: {item.note}\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the persistent FOC DWT runtime experiment")
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenarios", default=",".join(DEFAULT_SCENARIOS))
    parser.add_argument("--manual-mode", choices=["prompt", "auto", "skip"], default="prompt")
    parser.add_argument("--output-dir", type=Path, default=Path("."))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scenarios = [item.strip().upper() for item in args.scenarios.split(",") if item.strip()]
    unknown = [item for item in scenarios if item not in SCENARIO_SETUP]
    if unknown:
        raise SystemExit(f"Unknown scenarios: {unknown}")
    if args.duration <= 0.0 or args.warmup < 0.0 or args.repeats <= 0:
        raise SystemExit("duration/repeats must be positive and warmup must be non-negative")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"foc_runtime_profile_{timestamp}.csv"
    md_path = args.output_dir / f"foc_runtime_profile_{timestamp}.md"

    link = FocSerial(args.port, args.baud)
    runs: List[ScenarioRun] = []
    try:
        time.sleep(0.2)
        link.ser.reset_input_buffer()
        link.command("CMD:FW_INFO?", "FW_INFO,OK", timeout=1.5)
        for scenario in scenarios:
            for repeat in range(1, args.repeats + 1):
                run = run_scenario(
                    link,
                    scenario,
                    repeat,
                    args.duration,
                    args.warmup,
                    args.manual_mode,
                )
                if run is not None:
                    runs.append(run)
                    tim1 = next((item for item in run.results if item.probe == "TIM1_ISR"), None)
                    if tim1 is not None:
                        print(
                            f"DONE {scenario} repeat {repeat}: avg={tim1.avg_us:.3f}us "
                            f"max={tim1.max_us:.3f}us remaining={TIM1_BUDGET_US - tim1.max_us:.3f}us "
                            f"overrun={tim1.overrun} status={tim1.status}"
                        )
    except KeyboardInterrupt:
        print("Interrupted by user; writing partial results.")
    finally:
        try:
            send_stop(link)
        except Exception as exc:  # best-effort safety shutdown
            print(f"STOP warning: {exc}", file=sys.stderr)
        link.close()

    results = [result for run in runs for result in run.results]
    mark_outliers(results)
    write_csv(csv_path, results)
    write_markdown(md_path, runs, results)
    print(f"CSV: {csv_path.resolve()}")
    print(f"MD:  {md_path.resolve()}")
    return 0 if results else 2


if __name__ == "__main__":
    raise SystemExit(main())
