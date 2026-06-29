"""
Re-identify motor parameters with free rotor.
Monitor identify progress, verify enc_dir and motion_verify pass.
"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

ser = serial.Serial('COM9', 230400, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

def cmd(s, wait=0.3):
    ser.write((s + '\r\n').encode()); ser.flush(); time.sleep(wait)

def drain():
    ser.read(ser.in_waiting)

LOG = []
def log(msg):
    print(msg)
    LOG.append(msg)

# Clear any fault, unlock
log("=== Step 1: CLEAR_FAULT + UNLOCK ===")
drain()
cmd("CMD:CLEAR_FAULT", 0.5)
drain()
cmd("CMD:UNLOCK,1", 0.5)
cmd("CMD:SAFETY_STAT", 0.4)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    if line.startswith('N,'):
        p = line.split(',')
        if len(p) >= 15:
            log(f"  state={p[2]} appFault={p[14]}")

# Start identify
log("\n=== Step 2: START IDENTIFY ===")
cmd("CMD:IDENTIFY,1", 0.5)

# Monitor for up to 120 seconds
log("\n=== Step 3: MONITORING (120s max) ===")
log("Watching for state changes, identify progress...")
start_time = time.time()
last_state = -1
last_error = -1
last_id_state = -1
identify_frames = []

while time.time() - start_time < 120:
    if ser.in_waiting:
        raw = ser.read(ser.in_waiting)
        for line in raw.decode('utf-8', errors='replace').split('\n'):
            s = line.strip()
            if not s:
                continue

            # Capture N-frames for identify state
            if s.startswith('N,'):
                p = s.split(',')
                if len(p) >= 27:
                    try:
                        foc_st = int(p[2])
                        id_st = int(p[25])
                        id_err = int(p[26])
                        ang = float(p[3])
                        iq = float(p[6])

                        if id_st != last_id_state:
                            state_names = {0:'IDLE',1:'PN',2:'RS',3:'LS',4:'KE',5:'J',6:'ENCODER_ALIGN',7:'MOTION_VERIFY',8:'COGGING',9:'COMPLETE',10:'ERROR'}
                            elapsed = time.time() - start_time
                            log(f"  [{elapsed:.0f}s] IDENTIFY: {state_names.get(id_st, str(id_st))} (st={id_st}) err={id_err} foc_st={foc_st} ang={ang}° iq={iq:.3f}A")
                            last_id_state = id_st

                        if foc_st != last_state or id_err != last_error:
                            if foc_st == 5:  # FAULT
                                log(f"  ⚠️ FAULT! foc_state={foc_st} appFault={p[14]}")
                            last_state = foc_st
                            last_error = id_err
                    except:
                        pass

            # Capture identify-specific text output
            elif any(kw in s for kw in ['IDENTIFY','COMPLETE','ERROR','FAIL','State:','AppFault','Motion','identify']):
                log(f"  DIAG: {s[:200]}")

    time.sleep(0.05)

# Check final state
log("\n=== Step 4: FINAL STATUS ===")
cmd("CMD:FAULT_DETAIL", 2.0)
time.sleep(2.5)
for line in ser.read(ser.in_waiting).decode('utf-8', errors='replace').split('\n'):
    s = line.strip()
    if any(kw in s for kw in ['State:','AppFault','Identified','ThetaDiag','DirDiag','ParamDiag',
                               'DRV8350S Communication','Comm:','FAULT1:','COMPLETE']):
        log(f"  {s[:250]}")

ser.close()

with open(r"C:\Users\xiangyu\Desktop\reidentify_output.txt", 'w', encoding='utf-8') as f:
    f.write('\n'.join(LOG))
log("\nOutput written to reidentify_output.txt")
