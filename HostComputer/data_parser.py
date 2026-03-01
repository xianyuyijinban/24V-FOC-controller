"""
FOC Controller Data Parser
解析从STM32上传的文本格式数据
"""
import re
from dataclasses import dataclass
from typing import Optional, Callable


@dataclass
class FOCDataPacket:
    """FOC数据包"""
    timestamp: int = 0
    
    # 编码器数据
    angle: float = 0.0          # 角度 (度)
    raw_angle: int = 0
    crc_error: bool = False
    
    # DRV8350S数据
    fault_status1: int = 0
    vgs_status2: int = 0
    driver_ctrl: int = 0
    ocp_ctrl: int = 0
    fault_flags: int = 0
    is_fault_active: bool = False
    
    # FOC控制数据
    Id: float = 0.0             # D轴电流
    Iq: float = 0.0             # Q轴电流
    Vd: float = 0.0             # D轴电压
    Vq: float = 0.0             # Q轴电压
    speed: float = 0.0          # 转速 (rad/s)
    Id_ref: float = 0.0         # D轴电流参考
    Iq_ref: float = 0.0         # Q轴电流参考
    foc_state: int = 0          # FOC状态
    
    # 原始文本
    raw_text: str = ""


class FOCDataParser:
    """FOC数据解析器"""
    
    # 数据包起始和结束标记
    PACKET_STARTS = (
        "========== FOC Controller Status ==========",
        "========== !!! FAULT DETECTED !!! ==========",
    )
    PACKET_END = "======================================"
    MAX_BUFFER_SIZE = 8192
    
    def __init__(self):
        self.buffer = ""
        self.packet_callback: Optional[Callable[[FOCDataPacket], None]] = None
        
    def set_packet_callback(self, callback: Callable[[FOCDataPacket], None]):
        """设置数据包接收回调"""
        self.packet_callback = callback
        
    def feed_data(self, data: bytes):
        """喂入原始数据"""
        try:
            text = data.decode('utf-8', errors='ignore')
            self.buffer += text
            if len(self.buffer) > self.MAX_BUFFER_SIZE:
                # 限制无效输入场景下的内存增长
                self.buffer = self.buffer[-self.MAX_BUFFER_SIZE:]
            self._process_buffer()
        except Exception as e:
            print(f"Data parse error: {e}")
            
    def _process_buffer(self):
        """处理缓冲区，提取完整数据包"""
        while self.PACKET_END in self.buffer:
            start_idx = -1
            for marker in self.PACKET_STARTS:
                idx = self.buffer.find(marker)
                if idx != -1 and (start_idx == -1 or idx < start_idx):
                    start_idx = idx

            if start_idx == -1:
                # 丢弃无法识别的前缀噪声，避免缓冲区无限增长
                end_idx = self.buffer.find(self.PACKET_END)
                self.buffer = self.buffer[end_idx + len(self.PACKET_END):]
                continue

            # 找到数据包结束位置（在起始位置之后）
            end_idx = self.buffer.find(self.PACKET_END, start_idx)
            
            if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
                packet_end_idx = end_idx + len(self.PACKET_END)

                # 兼容不同长度的 '=' 分隔行，避免留下尾巴字符
                while packet_end_idx < len(self.buffer) and self.buffer[packet_end_idx] == "=":
                    packet_end_idx += 1
                while packet_end_idx < len(self.buffer) and self.buffer[packet_end_idx] in ("\r", "\n"):
                    packet_end_idx += 1

                # 提取完整数据包
                packet_text = self.buffer[start_idx:packet_end_idx]
                # 从缓冲区中移除
                self.buffer = self.buffer[packet_end_idx:]
                # 解析数据包
                self._parse_packet(packet_text)
            else:
                break
                
    def _parse_packet(self, text: str):
        """解析单个数据包"""
        packet = FOCDataPacket(raw_text=text)
        
        try:
            # 解析时间戳
            timestamp_match = re.search(r'Time:\s*(\d+)\s*ms', text)
            if timestamp_match:
                packet.timestamp = int(timestamp_match.group(1))
            
            # 解析编码器数据
            angle_match = re.search(r'Angle:\s*([\d.\-]+)\s*deg', text)
            if angle_match:
                packet.angle = float(angle_match.group(1))
                
            raw_match = re.search(r'Raw:\s*(\d+)', text)
            if raw_match:
                packet.raw_angle = int(raw_match.group(1))
                
            crc_match = re.search(r'CRC:\s*(ERROR!|OK)', text)
            if crc_match:
                packet.crc_error = (crc_match.group(1) == "ERROR!")
            
            # 解析FOC数据
            id_match = re.search(r'Id\s*:\s*([\d.\-]+)\s*A', text)
            if id_match:
                packet.Id = float(id_match.group(1))
            id_ref_match = re.search(r'Id\s*:\s*[\d.\-]+\s*A\s*\(ref:\s*([\d.\-]+)\)', text)
            if id_ref_match:
                packet.Id_ref = float(id_ref_match.group(1))
                
            iq_match = re.search(r'Iq\s*:\s*([\d.\-]+)\s*A', text)
            if iq_match:
                packet.Iq = float(iq_match.group(1))
            iq_ref_match = re.search(r'Iq\s*:\s*[\d.\-]+\s*A\s*\(ref:\s*([\d.\-]+)\)', text)
            if iq_ref_match:
                packet.Iq_ref = float(iq_ref_match.group(1))
                
            vd_match = re.search(r'Vd\s*:\s*([\d.\-]+)\s*V', text)
            if vd_match:
                packet.Vd = float(vd_match.group(1))
                
            vq_match = re.search(r'Vq\s*:\s*([\d.\-]+)\s*V', text)
            if vq_match:
                packet.Vq = float(vq_match.group(1))
                
            speed_match = re.search(r'Speed\s*:\s*([\d.\-]+)\s*rad/s', text)
            if speed_match:
                packet.speed = float(speed_match.group(1))
                
            state_match = re.search(r'State\s*:\s*(\d+)', text)
            if state_match:
                packet.foc_state = int(state_match.group(1))

            fault1_match = re.search(r'FAULT1:\s*0x([0-9A-Fa-f]+)', text)
            if fault1_match:
                packet.fault_status1 = int(fault1_match.group(1), 16)

            vgs2_match = re.search(r'VGS2:\s*0x([0-9A-Fa-f]+)', text)
            if vgs2_match:
                packet.vgs_status2 = int(vgs2_match.group(1), 16)
            
            # 解析故障信息
            fault_match = re.search(r'FAULT:\s*(Active|None)', text)
            if fault_match:
                packet.is_fault_active = (fault_match.group(1) == "Active")
            else:
                status_match = re.search(r'Status:\s*(>>> FAULT <<<|Normal)', text)
                if status_match:
                    packet.is_fault_active = ("FAULT" in status_match.group(1))
                elif "FAULT DETECTED" in text:
                    packet.is_fault_active = True
            
            # 调用回调
            if self.packet_callback:
                self.packet_callback(packet)
                
        except Exception as e:
            print(f"Packet parse error: {e}")
            print(f"Text: {text[:200]}...")


class CommandBuilder:
    """命令构建器 - 构建发送给下位机的命令"""
    
    @staticmethod
    def enable_motor(enable: bool) -> str:
        """使能/禁用电机"""
        return f"CMD:ENABLE,{1 if enable else 0}\n"
    
    @staticmethod
    def set_mode(mode: int) -> str:
        """设置控制模式 0=力矩 1=速度 2=位置"""
        return f"CMD:MODE,{mode}\n"
    
    @staticmethod
    def set_current_ref(id_ref: float, iq_ref: float) -> str:
        """设置电流参考值"""
        return f"CMD:IREF,{id_ref:.3f},{iq_ref:.3f}\n"
    
    @staticmethod
    def set_speed_ref(speed: float) -> str:
        """设置速度参考值"""
        return f"CMD:SREF,{speed:.3f}\n"
    
    @staticmethod
    def set_position_ref(pos: float) -> str:
        """设置位置参考值"""
        return f"CMD:PREF,{pos:.3f}\n"
    
    @staticmethod
    def start_identify() -> str:
        """启动参数识别"""
        return "CMD:IDENTIFY,1\n"
    
    @staticmethod
    def stop_identify() -> str:
        """停止参数识别"""
        return "CMD:IDENTIFY,0\n"
    
    @staticmethod
    def clear_fault() -> str:
        """清除故障"""
        return "CMD:CLEAR_FAULT\n"
    
    @staticmethod
    def set_current_pi(kp: float, ki: float) -> str:
        """设置电流环PI参数"""
        return f"CMD:PI_CURRENT,{kp:.6f},{ki:.6f}\n"
    
    @staticmethod
    def set_speed_pi(kp: float, ki: float) -> str:
        """设置速度环PI参数"""
        return f"CMD:PI_SPEED,{kp:.6f},{ki:.6f}\n"
    
    @staticmethod
    def set_position_pi(kp: float, ki: float) -> str:
        """设置位置环PI参数"""
        return f"CMD:PI_POS,{kp:.6f},{ki:.6f}\n"
