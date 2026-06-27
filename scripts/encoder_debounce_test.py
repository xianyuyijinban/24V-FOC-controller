"""Phase 1: Static check after encoder fault debounce change."""
import serial
import time
import sys

def main():
    ser = serial.Serial('COM9', 1152000, timeout=0.5, rtscts=False, dsrdtr=False)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    def cmd(c, wait=0.4):
        ser.write((c + '\r\n').encode())
        time.sleep(wait)
        out = b''
        while ser.in_waiting > 0:
            out += ser.read(ser.in_waiting)
            time.sleep(0.03)
        result = out.decode('ascii', errors='ignore').strip()
        print(f"[{c}] -> {result[:200]}")
        return result

    # Read initial boot messages
    time.sleep(0.5)
    boot = b''
    while ser.in_waiting > 0:
        boot += ser.read(ser.in_waiting)
        time.sleep(0.02)
    boot_str = boot.decode('ascii', errors='ignore')
    if boot_str:
        print(f"BOOT: {boot_str[:500]}")

    print("=== Phase 1: Static Check ===")
    cmd('CMD:CLEAR_FAULT')
    cmd('CMD:FAULT_DETAIL')
    cmd('CMD:TLE_RAW')

    ser.close()
    return 0

if __name__ == '__main__':
    sys.exit(main())
