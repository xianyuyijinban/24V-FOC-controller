"""UART command alias regression test.

Sends DIAG:UART_RX? and CMD:UART_RX_STAT? 20 times each,
validating responses through MixedStreamDecoder without
reset_input_buffer() (except one initial flush).
"""

import os
import sys
import time
import serial

# Import MixedStreamDecoder from the profile script
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
from common.foc_runtime_profile import MixedStreamDecoder

BAUD = 1_000_000
PORT = "COM7"


def test_commands(port: str) -> dict:
    ser = serial.Serial(port, baudrate=BAUD, timeout=0.01)
    time.sleep(0.3)

    # ONE-TIME flush at startup (allowed)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Stop any current stream
    ser.write(b"TELEM:CUR,OFF\n")
    ser.flush()
    time.sleep(0.3)
    ser.write(b"CMD:STOP\n")
    ser.flush()
    time.sleep(0.3)

    decoder = MixedStreamDecoder()

    def send_and_wait(cmd: str, timeout: float = 1.5) -> tuple[str | None, list[str]]:
        ser.reset_output_buffer()
        ser.write((cmd + "\n").encode("ascii"))
        ser.flush()
        deadline = time.monotonic() + timeout
        all_lines: list[str] = []
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            if waiting > 0:
                data = ser.read(waiting)
                lines = decoder.feed(data)
                all_lines.extend(lines)
                for line in lines:
                    if line.startswith("UART_RX,OK"):
                        return line, all_lines
            time.sleep(0.001)
        return None, all_lines

    results: dict = {
        "DIAG:UART_RX?": [],
        "CMD:UART_RX_STAT?": [],
    }

    for label, cmd in [("DIAG:UART_RX?", "DIAG:UART_RX?"),
                        ("CMD:UART_RX_STAT?", "CMD:UART_RX_STAT?")]:
        for i in range(20):
            resp, _ = send_and_wait(cmd)
            results[label].append({
                "index": i + 1,
                "response": resp,
                "ok": resp is not None and resp.startswith("UART_RX,OK"),
            })
            time.sleep(0.05)

    ser.close()
    return results


def parse_fields(line: str) -> dict:
    """Parse key=value fields from UART_RX,OK,... line."""
    fields = {}
    # format: UART_RX,OK,err=X,restart_fail=X,last_pos=X,buf=X,tx_p0_drop=X,tx_p1_drop=X,tx_p2_drop=X
    for part in line.split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            fields[k] = v
    return fields


def validate(results: dict) -> tuple[bool, str]:
    report_parts = []

    for label in ["DIAG:UART_RX?", "CMD:UART_RX_STAT?"]:
        entries = results[label]
        ok_count = sum(1 for e in entries if e["ok"])
        report_parts.append(f"\n## {label}")
        report_parts.append(f"Success: {ok_count}/20")

        if ok_count < 20:
            for e in entries:
                if not e["ok"]:
                    report_parts.append(
                        f"  FAIL #{e['index']}: response={e['response'][:80] if e['response'] else 'None'}"
                    )
        else:
            # Check field consistency
            first = parse_fields(entries[0]["response"])
            last = parse_fields(entries[-1]["response"])

            required = ["err", "restart_fail", "last_pos", "buf",
                        "tx_p0_drop", "tx_p1_drop", "tx_p2_drop"]
            missing = [f for f in required if f not in first]
            if missing:
                report_parts.append(f"  WARNING: missing fields in response: {missing}")
            else:
                report_parts.append(f"  Fields present: {sorted(required)}")

            err_delta = int(last.get("err", "0")) - int(first.get("err", "0"))
            rf_delta = int(last.get("restart_fail", "0")) - int(first.get("restart_fail", "0"))
            p0_delta = int(last.get("tx_p0_drop", "0")) - int(first.get("tx_p0_drop", "0"))
            p1_delta = int(last.get("tx_p1_drop", "0")) - int(first.get("tx_p1_drop", "0"))
            p2_delta = int(last.get("tx_p2_drop", "0")) - int(first.get("tx_p2_drop", "0"))

            report_parts.append(f"  err delta: {err_delta}")
            report_parts.append(f"  restart_fail delta: {rf_delta}")
            report_parts.append(f"  tx_p0_drop delta: {p0_delta}")
            report_parts.append(f"  tx_p1_drop delta: {p1_delta}")
            report_parts.append(f"  tx_p2_drop delta: {p2_delta}")

    # Overall
    diag_ok = all(e["ok"] for e in results["DIAG:UART_RX?"])
    cmd_ok = all(e["ok"] for e in results["CMD:UART_RX_STAT?"])
    all_ok = diag_ok and cmd_ok

    if all_ok:
        diag_first = parse_fields(results["DIAG:UART_RX?"][0]["response"])
        diag_last = parse_fields(results["DIAG:UART_RX?"][-1]["response"])
        err_ok = int(diag_last.get("err", "0")) - int(diag_first.get("err", "0")) == 0
        rf_ok = int(diag_last.get("restart_fail", "0")) - int(diag_first.get("restart_fail", "0")) == 0
        p0_ok = int(diag_last.get("tx_p0_drop", "0")) - int(diag_first.get("tx_p0_drop", "0")) == 0

        if err_ok and rf_ok and p0_ok and all_ok:
            report_parts.append("\n## VERDICT: PASS")
        else:
            report_parts.append("\n## VERDICT: FAIL (non-zero deltas)")
    else:
        report_parts.append("\n## VERDICT: FAIL (ACK timeout)")

    return all_ok, "\n".join(report_parts)


def main():
    print(f"UART Command Alias Regression Test")
    print(f"Port: {PORT}, Baud: {BAUD}")
    print(f"MixedStreamDecoder with no reset_input_buffer after startup")
    print("=" * 60)

    results = test_commands(PORT)
    ok, report = validate(results)

    print(report)

    if ok:
        print("\n[PASS] Both commands 20/20, all required fields present, deltas zero.")
        return 0
    else:
        print("\n[FAIL] See details above.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
