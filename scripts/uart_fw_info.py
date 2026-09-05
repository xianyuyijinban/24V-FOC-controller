#!/usr/bin/env python3
"""uart_fw_info.py — 烧录后存活校验: 发 SYS:FW_INFO? 并打印响应。

台架流程 (2026-09-05 定案): -f 后 pyocd reset/go, 再跑本脚本确认固件活着 + version/baseline 正确。
注意: Keil 构建下 git= 段为 "unknown" (FOC_GIT_HASH 仅 build.ps1 注入), 不能用于区分 commit,
       commit 断言由 flash_deploy.ps1 在构建前做 (git HEAD + 工作树检查)。
"""
import sys
import time

import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
    except serial.SerialException as e:
        print("OPEN_FAIL: %s" % e)
        return 1
    ser.reset_input_buffer()
    ser.write(b"SYS:FW_INFO?\n")
    buf = b""
    dl = time.time() + 3.0
    while time.time() < dl:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if b"FW_INFO" in buf:
                break
        else:
            time.sleep(0.02)
    ser.close()
    text = buf.decode(errors="replace").strip()
    print(text if text else "NO_RESPONSE")
    if "FW_INFO,OK" in text and "version=1.4.0" in text and "baseline=12V_STANDARD" in text:
        print("FW_INFO_OK")
        return 0
    print("FW_INFO_ABNORMAL")
    return 2


if __name__ == "__main__":
    sys.exit(main())
