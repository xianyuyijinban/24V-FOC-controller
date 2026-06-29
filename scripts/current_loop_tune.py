"""
Current Loop PI Tuning Sweep — v2 (fixes UNLOCK + N-frame-only tracking)
"""
import serial
import time
import csv
import sys
import os
from datetime import datetime
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"
BAUD = 1152000

STAGES = [
    ("S1_Kp0.30_Ki300",  0.30, 300),
    ("S2_Kp0.60_Ki800",  0.60, 800),
    ("S3_Kp1.00_Ki1500", 1.00, 1500),
    ("S4_Kp1.50_Ki2500", 1.50, 2500),
]

PULSES = [
    (0.0,  0.05, 0.8, "+0.05A"),
    (0.0, -0.05, 0.8, "-0.05A"),
    (0.0,  0.10, 0.8, "+0.10A"),
    (0.0, -0.10, 0.8, "-0.10A"),
]

OUTPUT_DIR = Path(__file__).resolve().parent / "current_loop_logs"


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = OUTPUT_DIR / f"tune_{timestamp}.csv"

    n_packets = []  # N-frame only

    def on_packet(pkt):
        # Only track N frames (they have foc_state > 0 or raw_text starts with "N,")
        if pkt.raw_text.startswith("N,"):
            n_packets.append(pkt)

    parser = FOCDataParser()
    parser.set_packet_callback(on_packet)

    print(f"Opening {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)

    def drain(duration_s=0.3):
        deadline = time.time() + duration_s
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            ser.timeout = max(0.01, min(remaining, 0.1))
            data = ser.read(4096)
            if data:
                parser.feed_data(data)

    def send(cmd_str, wait=0.2):
        ser.write(cmd_str.encode('utf-8'))
        drain(wait)

    def send_and_log(cmd_str, label="", wait=0.2):
        print(f"  [{label}] {cmd_str.strip()}")
        ser.write(cmd_str.encode('utf-8'))
        drain(wait)

    # ─── Pre-flight ───────────────────────────────────────────
    print("\n=== Pre-flight ===")
    drain(0.5)

    # 1. Ensure Vbus limits are correct for 12V bench
    print("  Setting Vbus limits 8V-30V...")
    send("CMD:VBUS_LIMIT,8.000,30.000\n", wait=0.2)

    # 2. Clear any fault
    print("  Clearing fault (if any)...")
    send("CMD:CLEAR_FAULT\n", wait=0.2)

    # 3. UNLOCK power stage
    print("  Unlocking power stage...")
    send("CMD:UNLOCK,1\n", wait=0.3)

    # 4. Set torque mode
    send_and_log(CommandBuilder.set_mode(0), "MODE=0", wait=0.2)

    # 5. ENABLE
    send_and_log(CommandBuilder.enable_motor(True), "ENABLE=1", wait=0.3)

    # Verify state via FAULT_DETAIL
    print("  Requesting diagnostic...")
    send("CMD:FAULT_DETAIL\n", wait=0.3)
    drain(0.2)

    # Check latest N frame
    if n_packets:
        last_n = n_packets[-1]
        print(f"  Latest N: state={last_n.foc_state}, vbus={last_n.vbus:.2f}V, "
              f"mode={last_n.control_mode}, fault={last_n.app_fault_code}")
        if last_n.foc_state == 5:
            print("  ⚠ Still in FAULT — aborting")
            ser.close()
            return
        if last_n.foc_state not in (4,):  # 4=RUNNING
            print(f"  ⚠ State={last_n.foc_state} (expected 4=RUNNING). "
                  f"PWM may not be active.")
    else:
        print("  ⚠ No N frames received!")

    # ─── Sweep ────────────────────────────────────────────────
    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "stage", "pulse_label", "timestamp_ms",
        "Id_ref", "Iq_ref", "Id", "Iq",
        "Vd", "Vq", "vbus", "speed",
        "foc_state", "fault_flags", "app_fault_code",
    ])

    for stage_label, kp, ki_cmd in STAGES:
        print(f"\n=== Stage: {stage_label} (Kp={kp}, Ki_cmd={ki_cmd}) ===")

        send_and_log(
            CommandBuilder.set_current_pi(kp, float(ki_cmd)),
            "PI_CURRENT",
            wait=0.2,
        )

        for id_ref, iq_ref, hold_s, pulse_label in PULSES:
            print(f"  Pulse: {pulse_label} (Iq_ref={iq_ref:.2f}A, {hold_s}s)")

            # Snapshot packet count before pulse
            n_before = len(n_packets)

            send_and_log(
                CommandBuilder.set_current_ref(id_ref, iq_ref),
                f"IREF {pulse_label}",
                wait=0.05,
            )

            t_start = time.time()
            while time.time() - t_start < hold_s:
                drain(0.1)

            send_and_log(
                CommandBuilder.set_current_ref(0.0, 0.0),
                "IREF 0",
                wait=0.05,
            )

            # Log new N frames from this pulse
            pulse_packets = n_packets[n_before:]
            for pkt in pulse_packets:
                csv_writer.writerow([
                    stage_label, pulse_label, pkt.timestamp,
                    pkt.Id_ref, pkt.Iq_ref, pkt.Id, pkt.Iq,
                    pkt.Vd, pkt.Vq, pkt.vbus, pkt.speed,
                    pkt.foc_state, pkt.fault_flags, pkt.app_fault_code,
                ])

            n_pkts = len(pulse_packets)
            if n_pkts > 0:
                iq_vals = [p.Iq for p in pulse_packets]
                iq_ref_vals = [p.Iq_ref for p in pulse_packets]
                iq_avg = sum(iq_vals) / len(iq_vals)
                iqr_avg = sum(iq_ref_vals) / len(iq_ref_vals)
                iq_min, iq_max = min(iq_vals), max(iq_vals)
                print(f"    → {n_pkts} N-frames, Iq: avg={iq_avg:+.4f}A "
                      f"[{iq_min:+.3f}, {iq_max:+.3f}], "
                      f"Iq_ref avg={iqr_avg:+.4f}A")
            else:
                print(f"    → 0 N-frames (warning!)")

            drain(0.5)  # cool-down

    # ─── Shutdown ─────────────────────────────────────────────
    print("\n=== Shutdown ===")
    send_and_log(CommandBuilder.enable_motor(False), "ENABLE=0", wait=0.2)
    send("CMD:UNLOCK,0\n", wait=0.2)
    csv_file.close()
    ser.close()

    print(f"\nLog: {log_path}")

    # ─── Summary ──────────────────────────────────────────────
    print("\n=== Summary ===")
    import collections
    stage_data = collections.defaultdict(list)
    with open(log_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            stage_data[(row["stage"], row["pulse_label"])].append(float(row["Iq"]))

    for (stage, pulse), iqs in sorted(stage_data.items()):
        avg = sum(iqs) / len(iqs)
        # Check if Iq tracks ref direction
        target = float(pulse.replace("A", ""))
        ratio = avg / target if abs(target) > 0.001 else 0
        print(f"  {stage:30s} {pulse:8s}: Iq_avg={avg:+.4f}A  "
              f"track={ratio:+.0%}  (n={len(iqs)})")


if __name__ == "__main__":
    main()
