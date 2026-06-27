"""
Negative Speed Diagnosis — Phase 1+2 per user plan.
Phase 1: SREF ±0.5 with FAULT_DETAIL capture → classify (current loop vs speed loop)
Phase 2: Torque mode ±Iq pulses, RS_FF_SCALE sweep (0, 0.5, 1.0)
"""
import serial, time, sys, re
from pathlib import Path

HOST_DIR = Path(__file__).resolve().parent.parent / "HostComputer"
sys.path.insert(0, str(HOST_DIR))
from data_parser import FOCDataParser, CommandBuilder

PORT = "COM9"; BAUD = 230400

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

    def read_text(dur=0.5):
        """Read raw text from serial, return decoded string."""
        deadline = time.time() + dur
        parts = []
        while time.time() < deadline:
            ser.timeout = max(0.01, min(deadline - time.time(), 0.1))
            data = ser.read(4096)
            if data:
                parts.append(data.decode('utf-8', errors='ignore'))
        return ''.join(parts)

    def latest_n():
        for p in reversed(pkts):
            if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,'):
                return p
        return None

    def n_frames_since(idx):
        """Return N-frames received since pkts[idx:]."""
        return [p for p in pkts[idx:] if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]

    # Initial drain
    time.sleep(0.3); drain(0.5)

    # ============================================================
    # Baseline setup
    # ============================================================
    print("=" * 70)
    print("NEGATIVE SPEED DIAGNOSIS — Phase 1+2")
    print("=" * 70)

    print("\n>>> Baseline setup: BEMF OFF, COG default, RS_FF=1.0, Kp=0.20")
    send("CMD:VBUS_LIMIT,8.000,30.000\n")
    send("CMD:CLEAR_FAULT\n")
    send("CMD:UNLOCK,1\n")
    send("CMD:BEMF_CFG,0\n")          # BEMF OFF
    send("CMD:COG_CFG,0.25,60\n")     # COG default
    send("CMD:RS_FF_SCALE,1.0\n")     # RsFF full
    send("CMD:PI_CURRENT,0.20,0\n")   # Kp=0.20, Ki=0
    send("CMD:PI_SPEED,0.30,0.3\n")   # speed PI

    # Confirm setup
    print("\n--- Board state ---")
    send("CMD:BEMF_CFG?\n", wait=0.2)
    text = read_text(0.3)
    for line in text.splitlines():
        if 'BEMF' in line or 'bemf' in line.lower():
            print(f"  {line.strip()}")

    send("CMD:FAULT_DETAIL\n", wait=0.3)
    text = read_text(0.5)
    for line in text.splitlines():
        line = line.strip()
        if any(k in line for k in ['CurrentLoopDiag', 'BEMF Ctrl', 'Power:',
                                     'State:', 'SpeedLoopDiag', 'MotionCfg']):
            print(f"  {line}")

    # ============================================================
    # PHASE 1: SREF ±0.5 with FAULT_DETAIL
    # ============================================================
    print("\n" + "=" * 70)
    print("PHASE 1: Speed mode SREF ±0.5 — classify problem")
    print("=" * 70)

    send("CMD:MODE,1\n")   # speed mode
    send("CMD:ENABLE,1\n")

    for sref, label in [(0.5, "POS+0.5"), (-0.5, "NEG-0.5")]:
        print(f"\n--- {label}: SREF={sref:+0.1f} rad/s ---")
        pkts.clear()
        send(f"CMD:SREF,{sref:.3f}\n", wait=0.3)
        drain(1.0)  # collect 1s of data

        nf = [p for p in pkts if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]
        if nf:
            speeds = [p.speed for p in nf]
            iq_refs = [p.Iq_ref for p in nf]
            iqs = [p.Iq for p in nf]
            vqs = [p.Vq for p in nf]
            vds = [p.Vd for p in nf]
            faults = [p.app_fault_code for p in nf]
            print(f"  N-frames: {len(nf)}")
            print(f"  Speed:   avg={sum(speeds)/len(speeds):+.3f} [{min(speeds):+.3f}, {max(speeds):+.3f}] rad/s")
            print(f"  Iq_ref:  avg={sum(iq_refs)/len(iq_refs):+.4f} [{min(iq_refs):+.4f}, {max(iq_refs):+.4f}] A")
            print(f"  Iq:      avg={sum(iqs)/len(iqs):+.4f} [{min(iqs):+.4f}, {max(iqs):+.4f}] A")
            print(f"  Vq:      avg={sum(vqs)/len(vqs):+.3f} [{min(vqs):+.3f}, {max(vqs):+.3f}] V")
            print(f"  Vd:      avg={sum(vds)/len(vds):+.3f} [{min(vds):+.3f}, {max(vds):+.3f}] V")
            print(f"  Faults:  {set(faults)}")
        else:
            print("  WARNING: No N-frames received!")

        # Capture FAULT_DETAIL after each direction
        send("CMD:FAULT_DETAIL\n", wait=0.3)
        text = read_text(0.5)
        for line in text.splitlines():
            line = line.strip()
            if any(k in line for k in ['CurrentLoopDiag', 'BEMF Ctrl',
                                         'SpeedLoopDiag', 'Power:', 'State:']):
                print(f"  DIAG: {line}")

        # Stop between directions
        send("CMD:SREF,0.000\n", wait=0.5)
        drain(0.3)

    # Disable
    send("CMD:SREF,0.000\n", wait=0.3)
    send("CMD:ENABLE,0\n", wait=0.5)
    drain(0.3)

    # ============================================================
    # PHASE 2: Torque mode negative current pulses + RS_FF_SCALE sweep
    # ============================================================
    print("\n" + "=" * 70)
    print("PHASE 2: Torque mode ±Iq pulses, RS_FF_SCALE sweep")
    print("=" * 70)

    send("CMD:MODE,0\n")   # torque mode
    send("CMD:PI_CURRENT,0.20,0\n")

    for rs_scale, rs_label in [(0.0, "RS_FF=0.0"), (0.5, "RS_FF=0.5"), (1.0, "RS_FF=1.0")]:
        print(f"\n--- {rs_label} ---")
        send(f"CMD:RS_FF_SCALE,{rs_scale:.1f}\n")
        send("CMD:ENABLE,1\n", wait=0.3)

        for iq_val, iq_label in [(0.05, "+0.05"), (-0.05, "-0.05"),
                                  (0.10, "+0.10"), (-0.10, "-0.10")]:
            pkts.clear()
            send(f"CMD:IREF,0.000,{iq_val:.3f}\n", wait=0.2)
            drain(0.4)

            nf = [p for p in pkts if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]
            if nf:
                iqs = [p.Iq for p in nf]
                iq_refs = [p.Iq_ref for p in nf]
                vqs = [p.Vq for p in nf]
                speeds = [p.speed for p in nf]
                print(f"  IREF{iq_label}A: Iq_avg={sum(iqs)/len(iqs):+.4f}A  "
                      f"Iqref_avg={sum(iq_refs)/len(iq_refs):+.4f}A  "
                      f"Vq_avg={sum(vqs)/len(vqs):+.3f}V  "
                      f"spd={sum(speeds)/len(speeds):+.2f}rad/s  "
                      f"fault={nf[-1].app_fault_code}")
            else:
                print(f"  IREF{iq_label}A: NO N-FRAMES")

            # Zero between pulses
            send("CMD:IREF,0.000,0.000\n", wait=0.2)

        # Capture FAULT_DETAIL at end of each RS_FF_SCALE setting
        send("CMD:FAULT_DETAIL\n", wait=0.3)
        text = read_text(0.4)
        for line in text.splitlines():
            line = line.strip()
            if 'CurrentLoopDiag' in line:
                print(f"  DIAG: {line}")

        send("CMD:ENABLE,0\n", wait=0.3)

    # ============================================================
    # Cleanup
    # ============================================================
    print("\n=== Cleanup ===")
    send("CMD:IREF,0.000,0.000\n")
    send("CMD:ENABLE,0\n")
    send("CMD:UNLOCK,0\n")
    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
