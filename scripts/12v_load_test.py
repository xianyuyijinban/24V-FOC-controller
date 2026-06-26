"""
12V Load Margin Test — manual hand-grab
Prompts user to apply/release load at each speed point.
"""
import serial, time, sys, statistics
PORT='COM9'; BAUD=230400

s = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
s.read(s.in_waiting or 65536)

def cmd(c, w=0.3):
    s.reset_input_buffer()
    s.write((c+'\r\n').encode())
    time.sleep(w)
    return s.read(s.in_waiting or 65536).decode('ascii', errors='replace')

def capture(dur=1.0):
    s.reset_input_buffer()
    time.sleep(dur)
    data = s.read(s.in_waiting or 65536).decode('ascii', errors='replace')
    frames=[]
    for line in data.split('\n'):
        line=line.strip()
        if line.startswith('N,'):
            parts=line.split(',')
            if len(parts)>=25:
                try:
                    frames.append({
                        'speed':float(parts[4]),'Iq':float(parts[6]),
                        'fault':parts[8].strip(),'Vq':float(parts[20]),
                        'Vbus':float(parts[7]),'state':int(parts[2]),
                    })
                except: pass
    return frames

def fault(frames):
    for f in frames:
        if f['fault']!='0x00000000': return f['fault']
    return None

# Setup
print("=== Setup ===")
for c in ['CMD:CLEAR_FAULT','CMD:VBUS_LIMIT,10,16','CMD:PI_CURRENT,0.50,0',
          'CMD:PI_SPEED,0.25,0.001','CMD:RS_FF_MODE,1','CMD:RS_FF_SCALE,0.20',
          'CMD:RS_FF_ADAPTIVE,0','CMD:COG_CFG,0.25,60','CMD:BEMF_CFG,0',
          'CMD:UNLOCK,1','CMD:ENABLE,1']:
    cmd(c, 0.2)
time.sleep(0.3)
fs = capture(0.3)
print(f"  State={fs[-1]['state'] if fs else '?'}  Vbus={fs[-1]['Vbus'] if fs else 0:.1f}V  Fault={fault(fs) or 'none'}")

# ── Load test ──
test_points = [
    (0.3, +1, "低速正向"),
    (0.3, -1, "低速反向"),
    (0.5, +1, "中速正向"),
    (0.5, -1, "中速反向"),
    (0.7, +1, "高速正向"),
    (0.7, -1, "高速反向"),
]

for sref, sign, label in test_points:
    val = sref * sign
    print(f"\n{'='*50}")
    print(f">>> {label}: SREF={val:+.1f} rad/s <<<")
    print(f"    电机正在转动，请准备好...")

    cmd(f"CMD:SREF,{val:.3f}", 0.2)
    time.sleep(1.5)

    # Baseline (no load, 2s)
    print(f"    采集空载基线 2s...")
    fs0 = capture(2.0)
    if not fs0:
        print("    NO DATA!"); continue
    fl0 = fault(fs0)
    w0 = statistics.mean([f['speed'] for f in fs0])
    iq0 = statistics.mean([f['Iq'] for f in fs0])
    vq0 = max(abs(f['Vq']) for f in fs0)
    print(f"    空载: ω={w0:+.3f}  Iq={iq0:+.3f}A  Vq_pk={vq0*1000:.0f}mV")

    if fl0:
        print(f"    ** FAULT: {fl0} ** — 跳过加载")
        cmd("CMD:CLEAR_FAULT",0.3); cmd("CMD:UNLOCK,1",0.2); cmd("CMD:ENABLE,1",0.5)
        continue

    # Load phase — user has 5s to grab motor
    print(f"    >>> 5秒内用手捏电机轴施加负载... <<<")
    time.sleep(5.0)
    fs_load = capture(3.0)
    if not fs_load:
        print("    NO DATA!"); cmd("CMD:SREF,0",0.3); continue
    fl_load = fault(fs_load)
    w_load = statistics.mean([f['speed'] for f in fs_load])
    iq_load = statistics.mean([f['Iq'] for f in fs_load])
    vq_load = max(abs(f['Vq']) for f in fs_load)
    iq_peak = max(abs(f['Iq']) for f in fs_load)

    print(f"    加载: ω={w_load:+.3f}  Iq={iq_load:+.3f}A  Iq_peak={iq_peak:+.3f}A  Vq_pk={vq_load*1000:.0f}mV")
    dw = w0 - w_load
    diq = iq_load - iq0
    print(f"    Δ:    dω={dw:+.3f}  dIq={diq:+.3f}A  fault={fl_load or 'none'}")

    if fl_load:
        print(f"    ** 加载触发 FAULT: {fl_load} **")
        cmd("CMD:CLEAR_FAULT",0.3); cmd("CMD:UNLOCK,1",0.2); cmd("CMD:ENABLE,1",0.5)

    # Recovery
    print(f"    >>> 松开电机，等待恢复... <<<")
    time.sleep(1.5)
    fs_rec = capture(2.0)
    if fs_rec:
        fl_rec = fault(fs_rec)
        w_rec = statistics.mean([f['speed'] for f in fs_rec])
        iq_rec = statistics.mean([f['Iq'] for f in fs_rec])
        print(f"    恢复: ω={w_rec:+.3f}  Iq={iq_rec:+.3f}A  fault={fl_rec or 'none'}")
        if fl_rec:
            cmd("CMD:CLEAR_FAULT",0.3); cmd("CMD:UNLOCK,1",0.2); cmd("CMD:ENABLE,1",0.5)

    # Stop between points
    cmd("CMD:SREF,0", 0.3)
    time.sleep(0.8)

# Cleanup
cmd("CMD:SREF,0",0.3); cmd("CMD:ENABLE,0",0.3); cmd("CMD:UNLOCK,0",0.3)
s.close()
print("\n=== 负载测试完成 ===")
