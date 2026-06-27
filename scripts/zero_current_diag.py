"""
Minimal diagnostic: torque mode, IREF=0, check current loop at rest.
Answer: is the current sensing chain clean at zero current?
"""
import serial, time, sys
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
        deadline = time.time() + dur
        parts = []
        while time.time() < deadline:
            ser.timeout = max(0.01, min(deadline - time.time(), 0.1))
            data = ser.read(4096)
            if data:
                parts.append(data.decode('utf-8', errors='ignore'))
        return ''.join(parts)

    time.sleep(0.3); drain(0.5)

    # ── Step 1: ADC zero with PWM OFF ──
    print("=" * 60)
    print("STEP 1: ADC zero (PWM OFF)")
    print("=" * 60)
    send("CMD:CLEAR_FAULT\n")
    send("CMD:ADC_ZERO,100\n", wait=0.5)
    text = read_text(0.5)
    for line in text.splitlines():
        line = line.strip()
        if line and ('ADC' in line or 'offset' in line or 'raw' in line or 'Ia' in line
                      or 'Ib' in line or 'Ic' in line or 'Zero' in line):
            print(f"  {line}")

    # ── Step 2: ENABLE with IREF=0, RS_FF_SCALE=0 ──
    print("\n" + "=" * 60)
    print("STEP 2: Torque mode, IREF=0, RS_FF=0 (pure PI only)")
    print("=" * 60)
    send("CMD:VBUS_LIMIT,8.000,30.000\n")
    send("CMD:UNLOCK,1\n")
    send("CMD:BEMF_CFG,0\n")
    send("CMD:COG_CFG,0,60\n")         # keep P0 off effectively
    send("CMD:RS_FF_SCALE,0.0\n")      # ZERO Rs feedforward
    send("CMD:PI_CURRENT,0.10,0\n")    # moderate Kp
    send("CMD:PI_SPEED,0.30,0.3\n")
    send("CMD:MODE,0\n")               # torque mode

    send("CMD:ENABLE,1\n", wait=0.3)
    pkts.clear()
    send("CMD:IREF,0.000,0.000\n", wait=0.3)
    drain(0.8)

    nf = [p for p in pkts if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]
    if nf:
        iqs = [p.Iq for p in nf]
        ids = [p.Id for p in nf]
        vqs = [p.Vq for p in nf]
        vds = [p.Vd for p in nf]
        iqrefs = [p.Iq_ref for p in nf]
        idrefs = [p.Id_ref for p in nf]
        print(f"  N-frames: {len(nf)}")
        print(f"  Id_ref: {sum(idrefs)/len(idrefs):+.4f}A   Id: {sum(ids)/len(ids):+.4f}A")
        print(f"  Iq_ref: {sum(iqrefs)/len(iqrefs):+.4f}A   Iq: {sum(iqs)/len(iqs):+.4f}A")
        print(f"  Vd: {sum(vds)/len(vds):+.3f}V   Vq: {sum(vqs)/len(vqs):+.3f}V")
        # Check if motor is still
        speeds = [abs(p.speed) for p in nf]
        print(f"  |speed|: avg={sum(speeds)/len(speeds):.3f} max={max(speeds):.3f} rad/s")
        print(f"  faults: {set(p.app_fault_code for p in nf)}")

        # Critical check: with IREF=0 and RS_FF=0, Vq should be near 0
        avg_vq = sum(vqs)/len(vqs)
        if abs(avg_vq) > 0.1:
            print(f"  *** WARNING: Vq={avg_vq:.3f}V with zero reference! Something driving output.")
        else:
            print(f"  OK: Vq near zero with IREF=0 and RS_FF=0")

        avg_iq = sum(iqs)/len(iqs)
        if abs(avg_iq) > 0.02:
            print(f"  *** WARNING: Iq={avg_iq:.4f}A with zero reference! ADC offset or angle error.")
        else:
            print(f"  OK: Iq near zero at idle")
    else:
        print("  NO N-FRAMES!")

    # ── Step 3: RS_FF_SCALE=1.0, IREF=0 ──
    print("\n" + "=" * 60)
    print("STEP 3: IREF=0, RS_FF=1.0 (Rs feedforward with zero ref)")
    print("=" * 60)
    send("CMD:RS_FF_SCALE,1.0\n")
    pkts.clear()
    drain(0.8)

    nf = [p for p in pkts if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]
    if nf:
        iqs = [p.Iq for p in nf]
        vqs = [p.Vq for p in nf]
        print(f"  Iq: {sum(iqs)/len(iqs):+.4f}A   Vq: {sum(vqs)/len(vqs):+.3f}V")
        if abs(sum(iqs)/len(iqs)) > 0.02:
            print(f"  *** Iq non-zero with RS_FF=1.0 but IREF=0!")

    # ── Step 4: Small positive current, RS_FF=1.0 ──
    print("\n" + "=" * 60)
    print("STEP 4: IREF=+0.05A, RS_FF=1.0")
    print("=" * 60)
    for iref in [0.05, -0.05, 0.10, -0.10]:
        pkts.clear()
        send(f"CMD:IREF,0.000,{iref:.3f}\n", wait=0.2)
        drain(0.5)
        nf = [p for p in pkts if hasattr(p, 'raw_text') and p.raw_text and p.raw_text.startswith('N,')]
        if nf:
            iqs = [p.Iq for p in nf]
            vqs = [p.Vq for p in nf]
            print(f"  IREF={iref:+.2f}A: Iq={sum(iqs)/len(iqs):+.4f}A  Vq={sum(vqs)/len(vqs):+.3f}V  "
                  f"fault={nf[-1].app_fault_code}")

        # Zero between
        send("CMD:IREF,0.000,0.000\n", wait=0.2)

    # ── Step 5: FAULT_DETAIL ──
    print("\n" + "=" * 60)
    print("STEP 5: FAULT_DETAIL")
    print("=" * 60)
    send("CMD:FAULT_DETAIL\n", wait=0.3)
    text = read_text(0.5)
    for line in text.splitlines():
        line = line.strip()
        if any(k in line for k in ['CurrentLoopDiag', 'BEMF Ctrl', 'SpeedLoopDiag',
                                     'Power:', 'State:', 'CurrentDQ']):
            print(f"  {line}")

    # Cleanup
    send("CMD:IREF,0.000,0.000\n")
    send("CMD:ENABLE,0\n")
    send("CMD:UNLOCK,0\n")
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
