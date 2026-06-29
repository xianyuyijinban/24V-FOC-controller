"""V1.1 Release Regression Test — run before tagging."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def cmd(ser, s):
    ser.reset_input_buffer()
    ser.write((s + '\r\n').encode())
    time.sleep(0.10)
    raw = ser.read(ser.in_waiting)
    for line in raw.decode('utf-8', errors='replace').split('\n'):
        l = line.strip()
        if l and l[0].isprintable() and not l.startswith(('C,', 'N,')):
            return l
    return '(ok)'

def count_frames(ser, dur=2.0):
    ser.reset_input_buffer()
    time.sleep(dur)
    raw = ser.read(ser.in_waiting)
    return sum(1 for i in range(len(raw)-1) if raw[i] == 0xA5 and raw[i+1] == 0x5A)

def fw_info(ser, n=50):
    ok = 0
    for _ in range(n):
        ser.reset_input_buffer()
        ser.write(b'SYS:FW_INFO?\r\n')
        time.sleep(0.07)
        if b'FW_INFO' in ser.read(ser.in_waiting):
            ok += 1
    return ok

def main():
    ser = serial.Serial('COM9', 1000000, timeout=0.05)
    time.sleep(3.0)
    ser.read(ser.in_waiting)

    tests = []
    print('V1.1 RELEASE REGRESSION')
    print('=' * 48)

    # 1. Initial
    ser.reset_input_buffer()
    time.sleep(2.0)
    nf0 = ser.read(ser.in_waiting).count(b'\nN,')
    tests.append(('N-frame baseline 50Hz', 45 <= nf0 <= 55, f'{nf0}/2s'))

    # 2. BIN 1000
    cmd(ser, 'TELEM:CUR,BIN,1000')
    time.sleep(0.5)
    bf1 = count_frames(ser)
    fw1 = fw_info(ser)
    tests.append(('BIN 1000: 950-1050fps', 950 <= bf1/2 <= 1050, f'{bf1/2:.0f}fps'))
    tests.append(('BIN 1000: FW_INFO? 50/50', fw1 == 50, f'{fw1}/50'))

    # 3. BIN 2000
    cmd(ser, 'TELEM:CUR,BIN,2000')
    time.sleep(0.5)
    bf2 = count_frames(ser)
    fw2 = fw_info(ser)
    stop2 = 'OK' in cmd(ser, 'CTRL:STOP')
    tests.append(('BIN 2000: >1200fps', bf2/2 > 1200, f'{bf2/2:.0f}fps'))
    tests.append(('BIN 2000: FW_INFO? 50/50', fw2 == 50, f'{fw2}/50'))
    tests.append(('BIN 2000: STOP response', stop2, 'OK' if stop2 else 'FAIL'))

    # 4. OFF
    cmd(ser, 'TELEM:CUR,OFF')
    time.sleep(0.5)
    ser.reset_input_buffer()
    time.sleep(2.0)
    nf3 = ser.read(ser.in_waiting).count(b'\nN,')
    tests.append(('OFF: N-frame restore 50Hz', 45 <= nf3 <= 55, f'{nf3}/2s'))

    print()
    for name, ok, detail in tests:
        print(f'  {"PASS" if ok else "FAIL"}  {name}: {detail}')

    passed = sum(1 for _, ok, _ in tests if ok)
    print(f'\n{passed}/{len(tests)} PASSED')
    ser.close()

if __name__ == '__main__':
    main()
