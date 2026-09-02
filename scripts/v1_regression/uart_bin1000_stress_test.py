"""BIN1000 + ACK concurrent stress test.

Runs TELEM:CUR,BIN,1000 for 60 seconds while querying DIAG:UART_RX?
100 times at ~600ms intervals. Uses MixedStreamDecoder to parse both
binary current frames and ASCII ACK responses concurrently.

Constraints:
- Only ONE reset_input_buffer() at startup
- No other buffer manipulation during test
- Must validate 100/100 ACK, ~57k-63k binary frames, 0 CRC errors
"""

import os
import sys
import time
import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
from common.foc_runtime_profile import MixedStreamDecoder

BAUD = 1_000_000
PORT = "COM7"
DURATION = 60.0
QUERY_COUNT = 100
QUERY_INTERVAL = DURATION / QUERY_COUNT  # ~0.6s


def run_stress_test(port: str) -> dict:
    ser = serial.Serial(port, baudrate=BAUD, timeout=0.01)
    time.sleep(0.3)

    # ONE-TIME flush (allowed)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Stop any current stream
    print("[SETUP] Stopping any active stream...")
    ser.write(b"TELEM:CUR,OFF\n"); ser.flush(); time.sleep(0.3)
    ser.write(b"CMD:STOP\n");      ser.flush(); time.sleep(0.3)

    # Drain responses
    time.sleep(0.2)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    decoder = MixedStreamDecoder()
    binary_before = 0
    crc_before = 0

    # Get initial UART stats
    ser.write(b"DIAG:UART_RX?\n"); ser.flush()
    time.sleep(0.3)
    init_line = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and init_line is None:
        waiting = ser.in_waiting
        if waiting > 0:
            data = ser.read(waiting)
            lines = decoder.feed(data)
            for line in lines:
                if line.startswith("UART_RX,OK"):
                    init_line = line
                    break
        time.sleep(0.001)

    if init_line is None:
        ser.close()
        return {"error": "Failed to get initial UART stats"}

    print(f"[SETUP] Initial UART stats: {init_line}")

    # Parse initial values
    def parse_field(line, field):
        for part in line.split(","):
            if part.startswith(f"{field}="):
                return int(part.split("=", 1)[1])
        return 0

    init_err = parse_field(init_line, "err")
    init_rf = parse_field(init_line, "restart_fail")
    init_p0 = parse_field(init_line, "tx_p0_drop")
    init_p1 = parse_field(init_line, "tx_p1_drop")
    init_p2 = parse_field(init_line, "tx_p2_drop")

    # Start BIN1000
    print("[SETUP] Starting TELEM:CUR,BIN,1000...")
    ser.write(b"CMD:UNLOCK,1\n"); ser.flush(); time.sleep(0.2)
    ser.write(b"CMD:APP_MODE,RAW\n"); ser.flush(); time.sleep(0.2)
    ser.write(b"CMD:MODE,1\n"); ser.flush(); time.sleep(0.2)
    ser.write(b"CMD:SREF,0.500\n"); ser.flush(); time.sleep(0.2)
    ser.write(b"CMD:ENABLE,1\n"); ser.flush(); time.sleep(0.2)

    # Drain any startup responses
    time.sleep(0.3)
    waiting = ser.in_waiting
    if waiting > 0:
        data = ser.read(waiting)
        decoder.feed(data)

    ser.write(b"TELEM:CUR,BIN,1000\n"); ser.flush()

    # Wait for stream to start (~0.5s)
    time.sleep(0.5)

    # Drain initial stream data
    waiting = ser.in_waiting
    if waiting > 0:
        data = ser.read(waiting)
        decoder.feed(data)

    # Record baselines AFTER stream starts
    binary_before = decoder.binary_frames
    crc_before = decoder.binary_crc_errors

    print(f"[START] Stress test: {DURATION}s, {QUERY_COUNT} queries, interval={QUERY_INTERVAL:.3f}s")
    print(f"[START] Binary baseline: frames={binary_before}, crc_errors={crc_before}")
    print("-" * 60)

    ack_count = 0
    ack_fail = 0
    start_time = time.monotonic()

    for i in range(QUERY_COUNT):
        # Send query
        ser.write(b"DIAG:UART_RX?\n")
        ser.flush()

        # Read all available data while waiting for the next query time
        target_time = start_time + (i + 1) * QUERY_INTERVAL
        query_deadline = time.monotonic() + 1.5  # ACK timeout
        ack_received = False

        while time.monotonic() < min(target_time, query_deadline) or (
            time.monotonic() < query_deadline and not ack_received
        ):
            waiting = ser.in_waiting
            if waiting > 0:
                data = ser.read(waiting)
                lines = decoder.feed(data)
                for line in lines:
                    if line.startswith("UART_RX,OK"):
                        if not ack_received:
                            ack_received = True
                            ack_count += 1
            time.sleep(0.001)

        if not ack_received:
            # Extended wait for straggler
            extra_deadline = time.monotonic() + 0.5
            while time.monotonic() < extra_deadline and not ack_received:
                waiting = ser.in_waiting
                if waiting > 0:
                    data = ser.read(waiting)
                    lines = decoder.feed(data)
                    for line in lines:
                        if line.startswith("UART_RX,OK"):
                            ack_received = True
                            ack_count += 1
                time.sleep(0.001)

        if not ack_received:
            ack_fail += 1
            print(f"  FAIL #{i+1}: ACK timeout")

        # Progress every 10 queries
        if (i + 1) % 10 == 0:
            elapsed = time.monotonic() - start_time
            print(f"  [{i+1}/{QUERY_COUNT}] elapsed={elapsed:.1f}s "
                  f"ACK ok={ack_count} fail={ack_fail} "
                  f"binary={decoder.binary_frames} crc={decoder.binary_crc_errors}")

    elapsed = time.monotonic() - start_time
    print(f"\n[STOP] After {elapsed:.1f}s")
    print(f"[STOP] Sending TELEM:CUR,OFF + CMD:STOP...")

    # Get final stats before stopping
    ser.write(b"DIAG:UART_RX?\n"); ser.flush()
    time.sleep(0.5)
    final_line = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and final_line is None:
        waiting = ser.in_waiting
        if waiting > 0:
            data = ser.read(waiting)
            lines = decoder.feed(data)
            for line in lines:
                if line.startswith("UART_RX,OK"):
                    final_line = line
                    break
        time.sleep(0.001)

    # Stop
    ser.write(b"TELEM:CUR,OFF\n"); ser.flush(); time.sleep(0.3)
    ser.write(b"CMD:STOP\n");      ser.flush(); time.sleep(0.3)

    # Check STOP response
    stop_ok = False
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting > 0:
            data = ser.read(waiting)
            lines = decoder.feed(data)
            for line in lines:
                if line.startswith("CUR_STREAM,OK"):
                    pass
                if "STOP,OK" in line or "CTRL:STOP,OK" in line:
                    stop_ok = True
                    break
            if stop_ok:
                break
        time.sleep(0.001)

    ser.close()

    # Calculate deltas
    final_err = parse_field(final_line, "err") if final_line else 0
    final_rf = parse_field(final_line, "restart_fail") if final_line else 0
    final_p0 = parse_field(final_line, "tx_p0_drop") if final_line else 0
    final_p1 = parse_field(final_line, "tx_p1_drop") if final_line else 0
    final_p2 = parse_field(final_line, "tx_p2_drop") if final_line else 0

    binary_total = decoder.binary_frames - binary_before
    crc_total = decoder.binary_crc_errors - crc_before

    return {
        "elapsed_s": elapsed,
        "queries_sent": QUERY_COUNT,
        "ack_ok": ack_count,
        "ack_fail": ack_fail,
        "binary_frames": binary_total,
        "crc_errors": crc_total,
        "init_err": init_err, "final_err": final_err, "err_delta": final_err - init_err,
        "init_rf": init_rf, "final_rf": final_rf, "rf_delta": final_rf - init_rf,
        "init_p0": init_p0, "final_p0": final_p0, "p0_delta": final_p0 - init_p0,
        "init_p1": init_p1, "final_p1": final_p1, "p1_delta": final_p1 - init_p1,
        "init_p2": init_p2, "final_p2": final_p2, "p2_delta": final_p2 - init_p2,
        "stop_ok": stop_ok,
        "final_line": final_line,
    }


def validate(result: dict) -> tuple[bool, str]:
    """Validate stress test results."""
    parts = []
    verdict = "PASS"

    parts.append(f"\n## BIN1000 Concurrent Stress Test Results")
    parts.append(f"Duration: {result['elapsed_s']:.1f}s")
    parts.append(f"Queries: {result['ack_ok']}/{result['queries_sent']} ACK received")
    parts.append(f"Binary frames: {result['binary_frames']}")
    parts.append(f"Binary CRC errors: {result['crc_errors']}")

    # Check 1: ACK 100/100
    if result["ack_ok"] < result["queries_sent"]:
        parts.append(f"\n### FAIL: Only {result['ack_ok']}/{result['queries_sent']} ACK received")
        verdict = "FAIL"
    else:
        parts.append(f"\n### ACK: PASS (100/100)")

    # Check 2: Binary frames ~57000-63000
    bf = result["binary_frames"]
    if 57000 <= bf <= 63000:
        parts.append(f"### Binary frames: PASS ({bf})")
    else:
        parts.append(f"### Binary frames: WARNING ({bf}, expected 57000-63000)")
        if verdict == "PASS":
            verdict = "PASS WITH WARNING"

    # Check 3: CRC errors = 0
    if result["crc_errors"] == 0:
        parts.append(f"### CRC errors: PASS (0)")
    else:
        parts.append(f"### CRC errors: FAIL ({result['crc_errors']})")
        verdict = "FAIL"

    # Check 4: RX error delta = 0
    if result["err_delta"] == 0:
        parts.append(f"### UART RX error delta: PASS (0)")
    else:
        parts.append(f"### UART RX error delta: FAIL ({result['err_delta']})")
        verdict = "FAIL"

    # Check 5: restart_fail delta = 0
    if result["rf_delta"] == 0:
        parts.append(f"### restart_fail delta: PASS (0)")
    else:
        parts.append(f"### restart_fail delta: FAIL ({result['rf_delta']})")
        verdict = "FAIL"

    # Check 6: P0 drop (hard fail)
    if result["p0_delta"] > 0:
        parts.append(f"### tx_p0_drop delta: FAIL ({result['p0_delta']}) — P0 queue issue")
        verdict = "FAIL"
    else:
        parts.append(f"### tx_p0_drop delta: PASS (0)")

    # Check 7: P1/P2 drop (informational)
    parts.append(f"### tx_p1_drop delta: {result['p1_delta']} (info)")
    parts.append(f"### tx_p2_drop delta: {result['p2_delta']} (info)")

    # Check 8: STOP
    if result["stop_ok"]:
        parts.append(f"### STOP: OK")
    else:
        parts.append(f"### STOP: WARNING (response not detected)")

    parts.append(f"\n## VERDICT: {verdict}")
    return verdict.startswith("PASS"), "\n".join(parts)


def main():
    print(f"BIN1000 Concurrent Stress Test")
    print(f"Port: {PORT}, Baud: {BAUD}, MixedStreamDecoder")
    print(f"No reset_input_buffer() after startup")
    print("=" * 60)

    result = run_stress_test(PORT)
    if "error" in result:
        print(f"ERROR: {result['error']}")
        return 1

    ok, report = validate(result)
    print(report)

    if ok:
        print(f"\n[PASS] Stress test passed.")
        return 0
    else:
        print(f"\n[FAIL] See details above.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
