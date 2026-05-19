"""
FOC Controller 串口调试工具
Usage:
  python serial_debug.py                  # 交互模式
  python serial_debug.py COM3             # 直接连接指定端口
  python serial_debug.py COM3 --baud 230400
"""
import sys
import threading
import time
import argparse
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else ".")
from HostComputer.data_parser import FOCDataParser, CommandBuilder


def list_available_ports():
    ports = list_ports.comports()
    if not ports:
        print("未检测到串口")
        return []
    for p in ports:
        print(f"  {p.device} - {p.description}")
    return [p.device for p in ports]


class SerialDebugger:
    def __init__(self, port: str, baud: int = 230400):
        self.port = port
        self.baud = baud
        self.ser: serial.Serial = None
        self.parser = FOCDataParser()
        self.parser.set_packet_callback(self._on_packet)
        self.running = False
        self.read_thread: threading.Thread = None

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.05,
        )
        print(f"[OK] 已连接到 {self.port} @ {self.baud}")
        return True

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[断开]")

    def start_reading(self):
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    data = self.ser.read(waiting)
                    if data:
                        self.parser.feed_data(data)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"[ERR] 读取错误: {e}")
                break

    def send(self, text: str):
        if not self.ser or not self.ser.is_open:
            print("[ERR] 未连接")
            return
        data = text.encode("utf-8")
        self.ser.write(data)
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{ts} TX] {text.strip()}")

    def send_command(self, cmd: str):
        self.send(cmd + "\n")

    def _on_packet(self, packet):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        state_names = {0: "IDLE", 1: "ALIGN", 2: "OPENLOOP", 3: "CLOSEDLOOP", 5: "FAULT"}
        sname = state_names.get(packet.foc_state, f"UNKNOWN({packet.foc_state})")
        line = (
            f"[{ts} RX] t={packet.timestamp:6d} | {sname:11s}"
            f" | Vbus={packet.vbus:7.3f}V | speed={packet.speed:8.3f} rad/s"
            f" | Id={packet.Id:+7.3f} Iq={packet.Iq:+7.3f}"
            f" | angle={packet.angle:7.2f}°"
        )
        if packet.fault_flags:
            line += f" | FAULT=0x{packet.fault_flags:08X}"
        if packet.app_fault_code:
            line += f" | AppFault={packet.app_fault_code}"
        if packet.undervoltage_limit is not None:
            line += f" | UV={packet.undervoltage_limit:.2f} OV={packet.overvoltage_limit:.2f}"
        print(line)


def interactive_mode(dbg: SerialDebugger):
    cmds = {
        "1": ("解锁功率", "CMD:UNLOCK,1"),
        "2": ("上锁功率", "CMD:UNLOCK,0"),
        "3": ("使能电机", "CMD:ENABLE,1"),
        "4": ("禁用电机", "CMD:ENABLE,0"),
        "5": ("力矩模式", "CMD:MODE,0"),
        "6": ("速度模式", "CMD:MODE,1"),
        "7": ("位置模式", "CMD:MODE,2"),
        "8": ("清除故障", "CMD:CLEAR_FAULT"),
        "9": ("启动参数识别", "CMD:IDENTIFY,1"),
        "0": ("停止参数识别", "CMD:IDENTIFY,0"),
    }

    print("\n===== 快捷键命令 =====")
    for k, (desc, _) in cmds.items():
        print(f"  [{k}] {desc}")
    print("  [i] 设置 Id/Iq 电流 (交互输入)")
    print("  [s] 设置速度 (交互输入)")
    print("  [p] 设置位置 (交互输入)")
    print("  [q] 退出")
    print("  [回车] 直接输入原始命令")
    print("======================")

    while True:
        try:
            inp = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not inp:
            continue
        if inp == "q":
            break

        if inp in cmds:
            dbg.send_command(cmds[inp][1])
        elif inp == "i":
            id_val = input("  Id (A): ").strip()
            iq_val = input("  Iq (A): ").strip()
            try:
                dbg.send_command(f"CMD:IREF,{float(id_val):.3f},{float(iq_val):.3f}")
            except:
                print("[ERR] 数字格式错误")
        elif inp == "s":
            speed_val = input("  Speed (rad/s): ").strip()
            try:
                dbg.send_command(f"CMD:SREF,{float(speed_val):.3f}")
            except:
                print("[ERR] 数字格式错误")
        elif inp == "p":
            pos_val = input("  Position (deg): ").strip()
            try:
                dbg.send_command(f"CMD:PREF,{float(pos_val):.3f}")
            except:
                print("[ERR] 数字格式错误")
        else:
            dbg.send_command(inp)


def main():
    parser = argparse.ArgumentParser(description="FOC Controller 串口调试工具")
    parser.add_argument("port", nargs="?", help="COM端口号，如 COM3")
    parser.add_argument("--baud", type=int, default=230400, help="波特率 (默认 230400)")
    parser.add_argument("--list", action="store_true", help="列出可用串口")
    args = parser.parse_args()

    if args.list or not args.port:
        print("可用串口:")
        ports = list_available_ports()
        if args.list:
            return
        if not args.port and ports:
            args.port = ports[0]
            print(f"\n默认选择第一个端口: {args.port}")
            confirm = input("按回车继续，或输入端口号: ").strip()
            if confirm:
                args.port = confirm

    if not args.port:
        return

    dbg = SerialDebugger(args.port, args.baud)
    try:
        dbg.connect()
    except serial.SerialException as e:
        print(f"[ERR] 连接失败: {e}")
        sys.exit(1)

    dbg.start_reading()
    interactive_mode(dbg)
    dbg.disconnect()


if __name__ == "__main__":
    main()
