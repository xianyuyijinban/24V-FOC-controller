"""Speed mode verification — prevent runaway, then current loop tests."""
import serial, time, sys
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"; BAUD = 1152000

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    parser = FOCDataParser()
    pkts = []
    parser.set_packet_callback(lambda p: pkts.append(p))

    def drain(dur):
        deadline = time.time() + dur
        while time.time() < deadline:
            ser.timeout = max(0.01, min(deadline - time.time(), 0.1))
            data = ser.read(4096)
            if data: parser.feed_data(data)

    def send(cmd, wait=0.3):
        ser.write(cmd.encode() if isinstance(cmd, str) else cmd)
        drain(wait)

    def latest_n():
        for p in reversed(pkts):
            if p.raw_text.startswith('N,'):
                return p
        return None

    time.sleep(0.3); drain(0.5)

    # Setup
    print("=== Setup ===")
    send("CMD:VBUS_LIMIT,8.000,30.000\n")
    send("CMD:CLEAR_FAULT\n")
    send("CMD:UNLOCK,1\n")

    # ─── SPEED MODE TEST ───
    print("\n=== Speed Mode: SREF=0.5 rad/s ===")
    send("CMD:PI_CURRENT,0.03,0\n")       # default low Kp
    send("CMD:PI_SPEED,0.30,0\n")
    send("CMD:MODE,1\n")                    # speed mode
    send("CMD:ENABLE,1\n")

    pkts.clear()
    send("CMD:SREF,0.500\n", wait=0.5)
    drain(1.0)  # 1 second of data

    n_frames = [p for p in pkts if p.raw_text.startswith('N,')]
    if n_frames:
        speeds = [p.speed for p in n_frames]
        iq_refs = [p.Iq_ref for p in n_frames]
        iqs = [p.Iq for p in n_frames]
        vqs = [p.Vq for p in n_frames]
        print(f"  {len(n_frames)} N-frames")
        print(f"  Speed: avg={sum(speeds)/len(speeds):.3f} [{min(speeds):.3f}, {max(speeds):.3f}] rad/s")
        print(f"  Iq_ref: avg={sum(iq_refs)/len(iq_refs):.4f} [{min(iq_refs):.4f}, {max(iq_refs):.4f}] A")
        print(f"  Iq: avg={sum(iqs)/len(iqs):.4f} [{min(iqs):.4f}, {max(iqs):.4f}] A")
        print(f"  Vq: avg={sum(vqs)/len(vqs):.3f} [{min(vqs):.3f}, {max(vqs):.3f}] V")

    # ─── STOP, test torque mode briefly ───
    send("CMD:SREF,0.000\n", wait=0.5)
    drain(0.5)

    print("\n=== Torque Mode: Iq_ref=±0.05 with Kp=0.60 ===")
    send("CMD:ENABLE,0\n", wait=0.3)
    send("CMD:MODE,0\n")
    send("CMD:PI_CURRENT,0.60,800\n")      # test Kp
    send("CMD:ENABLE,1\n", wait=0.5)

    for iq_ref, label in [(0.05, '+0.05'), (-0.05, '-0.05')]:
        pkts.clear()
        send(f"CMD:IREF,0.000,{iq_ref:.3f}\n", wait=0.3)
        drain(0.6)
        n_frames = [p for p in pkts if p.raw_text.startswith('N,')]
        if n_frames:
            iqs = [p.Iq for p in n_frames]
            speeds = [p.speed for p in n_frames]
            vqs = [p.Vq for p in n_frames]
            print(f"  {label}: Iq_avg={sum(iqs)/len(iqs):+.4f} "
                  f"speed_avg={sum(speeds)/len(speeds):.2f} "
                  f"Vq_avg={sum(vqs)/len(vqs):.2f}V "
                  f"fault={n_frames[-1].app_fault_code}")

        send("CMD:IREF,0.000,0.000\n", wait=0.4)

    # Shutdown
    send("CMD:IREF,0.000,0.000\n", wait=0.2)
    send("CMD:ENABLE,0\n")
    send("CMD:UNLOCK,0\n")
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
