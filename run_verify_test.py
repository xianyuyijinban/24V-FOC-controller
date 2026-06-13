"""
MOTION_VERIFY UART test script.
Sends identification command sequence and monitors verify_accum/verify_phase/locked_dir.
"""
import serial
import time
import sys
import io

# Force UTF-8 stdout to avoid GBK encoding issues on Windows
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = "COM9"
BAUD = 230400
TIMEOUT = 0.1

COMMANDS = [
    ("CMD:CLEAR_FAULT", 2.0),
    ("CMD:MOTOR_PN,11", 1.0),
    ("CMD:ENCODER_DIR,-1", 1.0),
    ("CMD:UNLOCK,1", 1.0),
    ("CMD:IDENTIFY,1", 90.0),  # long wait for full identification
]

KEYWORDS = [
    'verify_accum', 'verify_phase', 'locked_dir',
    'MOTION_VERIFY', 'status', 'error_code',
    'IDENTIFY', 'COMPLETE', 'FAULT', 'PN_VERIFY',
    'MotorPn', 'RS', 'LS', 'KE', 'J', 'ENCODER',
]

def main():
    print(f"Opening {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    print(f"Connected. Waiting for boot / data...\n")
    sys.stdout.flush()

    ser.reset_input_buffer()

    overall_start = time.time()
    cmd_idx = 0
    cmd_timer = time.time()

    try:
        while True:
            now = time.time()

            # Read and display incoming data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='replace')
                # Highlight key MOTION_VERIFY fields
                for kw in KEYWORDS:
                    if kw.lower() in text.lower():
                        print(f"\n{'='*60}")
                        print(f">>> {kw}")
                        print(f"{'='*60}")
                        sys.stdout.flush()
                print(text, end='', flush=True)

            # Send next command when delay elapsed
            if cmd_idx < len(COMMANDS):
                cmd, delay = COMMANDS[cmd_idx]
                if now - cmd_timer >= delay:
                    print(f"\n--- SENDING: {cmd} ---")
                    sys.stdout.flush()
                    ser.write((cmd + "\r\n").encode('utf-8'))
                    ser.flush()
                    cmd_timer = time.time()
                    cmd_idx += 1
            else:
                # All commands sent, keep reading until timeout
                if now - cmd_timer > 120:
                    print("\n--- Test timeout (120s post-IDENTIFY) reached ---")
                    break
                time.sleep(0.05)
                continue

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        ser.close()
        elapsed = time.time() - overall_start
        print(f"\nSerial port closed. Total runtime: {elapsed:.1f}s")

if __name__ == "__main__":
    main()
