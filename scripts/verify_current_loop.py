"""Current loop verification with BEMF disabled."""
import serial
import time
import sys
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"
BAUD = 1152000

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    parser = FOCDataParser()

    pkts = []
    diag_lines = []
    parser.set_packet_callback(lambda p: pkts.append(p))
    parser.set_diagnostic_callback(lambda line: diag_lines.append(line))

    def drain(dur):
        deadline = time.time() + dur
        while time.time() < deadline:
            ser.timeout = max(0.01, min(deadline - time.time(), 0.1))
            data = ser.read(4096)
            if data:
                parser.feed_data(data)

    def send(cmd, wait=0.3):
        ser.write(cmd.encode() if isinstance(cmd, str) else cmd)
        drain(wait)

    # Initial drain
    time.sleep(0.3)
    drain(0.5)

    # Setup
    print("=== Setup ===")
    send("CMD:VBUS_LIMIT,8.000,30.000\n")
    send("CMD:CLEAR_FAULT\n")
    send("CMD:UNLOCK,1\n")
    send("CMD:MODE,0\n")
    send("CMD:PI_CURRENT,0.60,800\n")
    send("CMD:ENABLE,1\n")

    # BEMF status
    print("=== BEMF Status ===")
    diag_lines.clear()
    send("CMD:BEMF_CFG?\n", wait=0.4)
    for line in diag_lines:
        if 'BEMF_CFG' in line:
            print(f"  {line.strip()}")

    # Verify BEMF is disabled
    diag_lines.clear()
    send("CMD:FAULT_DETAIL\n", wait=0.6)
    for line in diag_lines:
        if any(kw in line for kw in ['BEMF Ctrl', 'CurrentLoopDiag', 'CurrentDQ:',
                                       'Power:', 'State:', 'AppFault']):
            print(f"  {line.strip()}")

    # Pulse tests
    print("\n=== Pulse Tests (BEMF off) ===")
    for iq_ref, label in [(0.05, '+0.05A'), (-0.05, '-0.05A'), (0.10, '+0.10A'), (-0.10, '-0.10A')]:
        pkts.clear()
        diag_lines.clear()

        # Start pulse
        send(f"CMD:IREF,0.000,{iq_ref:.3f}\n", wait=0.5)

        # Capture FAULT_DETAIL mid-pulse
        send("CMD:FAULT_DETAIL\n", wait=0.6)

        # Extract key lines
        for line in diag_lines:
            if any(kw in line for kw in ['CurrentLoopDiag', 'BEMF Ctrl', 'CurrentDQ:',
                                           'PreSat', 'AppFault']):
                print(f"  [{label}] {line.strip()[:150]}")

        # Stop pulse
        send("CMD:IREF,0.000,0.000\n", wait=0.4)

    # Check N-frames for Iq tracking
    if pkts:
        n_frames = [p for p in pkts if p.raw_text.startswith('N,')]
        if n_frames:
            last = n_frames[-1]
            print(f"\n  Last N-frame: Iq_ref={last.Iq_ref:.4f}, Iq={last.Iq:.4f}, "
                  f"Vq={last.Vq:.3f}, speed={last.speed:.2f}")

    # Shutdown
    send("CMD:IREF,0.000,0.000\n", wait=0.2)
    send("CMD:ENABLE,0\n")
    send("CMD:UNLOCK,0\n")
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
