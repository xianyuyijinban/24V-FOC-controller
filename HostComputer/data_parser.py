"""
FOC Controller Data Parser
解析从STM32上传的文本格式数据
"""
import re
import struct
from dataclasses import dataclass
from typing import Optional, Callable, List, Tuple


@dataclass
class FOCDataPacket:
    """FOC数据包"""
    timestamp: int = 0
    
    # 编码器数据
    angle: float = 0.0          # 角度 (度)
    raw_angle: int = 0
    crc_error: bool = False
    encoder_detected: Optional[bool] = None
    
    # DRV8350S数据
    fault_status1: int = 0
    vgs_status2: int = 0
    driver_ctrl: int = 0
    ocp_ctrl: int = 0
    fault_flags: int = 0
    is_fault_active: bool = False
    app_warning_flags: int = 0
    app_fault_code: int = 0
    
    # FOC控制数据
    Id: float = 0.0             # D轴电流
    Iq: float = 0.0             # Q轴电流
    Ia: float = 0.0             # A相电流
    Ib: float = 0.0             # B相电流
    Ic: float = 0.0             # C相电流
    Vd: float = 0.0             # D轴电压
    Vq: float = 0.0             # Q轴电压
    vbus: float = 0.0           # 母线电压
    speed: float = 0.0          # 转速 (rad/s)
    Id_ref: float = 0.0         # D轴电流参考
    Iq_ref: float = 0.0         # Q轴电流参考
    speed_ref: float = 0.0      # 速度参考
    pos_ref: float = 0.0        # 位置参考
    foc_state: int = 0          # FOC状态
    control_mode: Optional[int] = None  # 固件当前控制模式
    motor_identified: bool = False
    stall_mode_armed: bool = False
    stall_open_loop_active: bool = False
    undervoltage_limit: Optional[float] = None
    overvoltage_limit: Optional[float] = None
    identify_state: Optional[int] = None
    identify_error: Optional[int] = None
    adc_calib_status: Optional[int] = None
    adc_offset_a: Optional[int] = None
    adc_offset_b: Optional[int] = None
    adc_offset_c: Optional[int] = None
    motor_param_rs: Optional[float] = None
    motor_param_ld: Optional[float] = None
    motor_param_lq: Optional[float] = None
    motor_param_ke: Optional[float] = None
    motor_param_pn: Optional[int] = None
    motor_param_encoder_dir: Optional[int] = None
    motor_param_theta_offset: Optional[float] = None
    motor_param_mech_zero: Optional[float] = None
    phase_current_only: bool = False
    
    # 原始文本
    raw_text: str = ""


class FOCDataParser:
    """FOC数据解析器"""
    
    # 数据包起始和结束标记
    DETAILED_PACKET_START = "========== FOC Diagnostic Snapshot =========="
    PACKET_STARTS = (
        "========== FOC Controller Status ==========",
        DETAILED_PACKET_START,
        "========== !!! FAULT DETECTED !!! ==========",
    )
    COMPACT_PREFIXES = ("N,", "F,", "C,")
    PACKET_END = "======================================"
    MAX_BUFFER_SIZE = 8192
    FOC_STATE_FAULT = 5
    DETAILED_STATE_MAP = {
        "IDLE": 0,
        "INIT": 1,
        "IDENTIFY": 2,
        "READY": 3,
        "RUNNING": 4,
        "FAULT": 5,
    }
    
    def __init__(self):
        self.buffer = ""
        self.packet_callback: Optional[Callable[[FOCDataPacket], None]] = None
        self.diagnostic_callback: Optional[Callable[[str], None]] = None
        
    def set_packet_callback(self, callback: Callable[[FOCDataPacket], None]):
        """设置数据包接收回调"""
        self.packet_callback = callback

    def set_diagnostic_callback(self, callback: Callable[[str], None]):
        """Expose complete non-packet text lines such as boot breadcrumbs."""
        self.diagnostic_callback = callback

    @staticmethod
    def _is_bool_token(token: str) -> bool:
        return token in ("0", "1")

    @staticmethod
    def _try_parse_decimal(token: str) -> Optional[float]:
        if token.startswith(("0x", "0X")):
            return None
        try:
            return float(token)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _try_parse_int(token: str) -> Optional[int]:
        try:
            return int(str(token), 0)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _is_compact_packet_marker_at(text: str, marker: str, index: int) -> bool:
        if not text.startswith(marker, index):
            return False
        timestamp_start = index + len(marker)
        if timestamp_start >= len(text):
            return True
        return text[timestamp_start].isdigit()

    @classmethod
    def _parse_state_token(cls, token: str) -> Optional[int]:
        token = str(token).strip()
        if token.isdigit():
            return int(token)
        return cls.DETAILED_STATE_MAP.get(token.upper())

    @classmethod
    def _detailed_packet_has_active_fault(cls, text: str, packet: FOCDataPacket) -> bool:
        fault_match = re.search(r'FAULT:\s*(Active|None)', text)
        if fault_match:
            return fault_match.group(1) == "Active"

        status_match = re.search(r'Status:\s*(>>> FAULT <<<|Normal)', text)
        if status_match:
            return "FAULT" in status_match.group(1)

        if packet.foc_state == cls.FOC_STATE_FAULT or packet.app_fault_code != 0:
            return True

        if packet.fault_flags != 0 or packet.fault_status1 != 0 or packet.vgs_status2 != 0:
            return True

        if re.search(r'\[(CRIT|FAIL|COMM|BOOT|FAULT)\]', text, flags=re.IGNORECASE):
            return True

        return False
        
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

    def _emit_diagnostic_lines(self, text: str):
        if self.diagnostic_callback is None:
            return
        for line in text.splitlines():
            line = line.strip()
            if line:
                self.diagnostic_callback(line)
            
    def _process_buffer(self):
        """处理缓冲区，提取完整数据包"""
        while self.buffer:
            start_idx = -1
            start_marker = None

            for marker in self.COMPACT_PREFIXES + self.PACKET_STARTS:
                idx = self.buffer.find(marker)
                if idx != -1 and marker in self.COMPACT_PREFIXES:
                    while idx != -1 and not self._is_compact_packet_marker_at(self.buffer, marker, idx):
                        idx = self.buffer.find(marker, idx + 1)
                if idx != -1 and (start_idx == -1 or idx < start_idx):
                    start_idx = idx
                    start_marker = marker

            if start_idx == -1:
                newline_idx = max(self.buffer.rfind("\n"), self.buffer.rfind("\r"))
                if newline_idx != -1:
                    diagnostic_text = self.buffer[: newline_idx + 1]
                    self.buffer = self.buffer[newline_idx + 1 :]
                    self._emit_diagnostic_lines(diagnostic_text)
                    continue
                return

            if start_idx > 0:
                self._emit_diagnostic_lines(self.buffer[:start_idx])
                self.buffer = self.buffer[start_idx:]
                continue

            if start_marker in self.COMPACT_PREFIXES:
                end_idx = self.buffer.find("\n")
                if end_idx == -1:
                    break
                packet_text = self.buffer[: end_idx + 1]
                self.buffer = self.buffer[end_idx + 1 :]
                self._parse_compact_packet(packet_text.strip())
                continue

            end_idx = self.buffer.find(self.PACKET_END, start_idx)
            if end_idx == -1 or end_idx <= start_idx:
                break

            packet_end_idx = end_idx + len(self.PACKET_END)
            while packet_end_idx < len(self.buffer) and self.buffer[packet_end_idx] == "=":
                packet_end_idx += 1
            while packet_end_idx < len(self.buffer) and self.buffer[packet_end_idx] in ("\r", "\n"):
                packet_end_idx += 1

            packet_text = self.buffer[start_idx:packet_end_idx]
            self.buffer = self.buffer[packet_end_idx:]
            self._parse_packet(packet_text)

    def _parse_compact_packet(self, text: str):
        packet = FOCDataPacket(raw_text=text)

        try:
            fields = text.split(",")
            if not fields:
                return

            if fields[0] == "C" and len(fields) >= 5:
                packet.timestamp = int(fields[1])
                packet.Ia = float(fields[2])
                packet.Ib = float(fields[3])
                packet.Ic = float(fields[4])
                packet.phase_current_only = True
            elif fields[0] == "N" and len(fields) >= 12:
                remaining = fields[12:]
                extra_fields = remaining

                packet.timestamp = int(fields[1])
                packet.foc_state = int(fields[2])
                packet.angle = float(fields[3])
                packet.speed = float(fields[4])
                packet.Id = float(fields[5])
                packet.Iq = float(fields[6])
                packet.vbus = float(fields[7])
                packet.fault_flags = int(fields[8], 0)
                packet.encoder_detected = (fields[9] == "1")
                packet.motor_identified = (fields[10] == "1")
                packet.stall_mode_armed = (fields[11] == "1")
                if extra_fields and self._is_bool_token(extra_fields[0]):
                    packet.stall_open_loop_active = (extra_fields[0] == "1")
                    extra_fields = extra_fields[1:]

                if len(extra_fields) >= 4:
                    app_warning_flags = self._try_parse_int(extra_fields[0])
                    app_fault_code = self._try_parse_int(extra_fields[1])
                    uv_index = 14 if len(extra_fields) >= 16 else -2
                    ov_index = 15 if len(extra_fields) >= 16 else -1
                    uv = self._try_parse_decimal(extra_fields[uv_index])
                    ov = self._try_parse_decimal(extra_fields[ov_index])
                    if app_warning_flags is not None and app_fault_code is not None and uv is not None and ov is not None:
                        packet.app_warning_flags = app_warning_flags
                        packet.app_fault_code = app_fault_code
                        packet.undervoltage_limit = uv
                        packet.overvoltage_limit = ov
                        if len(extra_fields) >= 11:
                            control_mode = self._try_parse_int(extra_fields[2])
                            id_ref = self._try_parse_decimal(extra_fields[3])
                            speed_ref = self._try_parse_decimal(extra_fields[4])
                            pos_ref = self._try_parse_decimal(extra_fields[5])
                            iq_ref = self._try_parse_decimal(extra_fields[6])
                            vd = self._try_parse_decimal(extra_fields[7])
                            vq = self._try_parse_decimal(extra_fields[8])
                            if len(extra_fields) >= 14:
                                ia = self._try_parse_decimal(extra_fields[9])
                                ib = self._try_parse_decimal(extra_fields[10])
                                ic = self._try_parse_decimal(extra_fields[11])
                                if ia is not None:
                                    packet.Ia = ia
                                if ib is not None:
                                    packet.Ib = ib
                                if ic is not None:
                                    packet.Ic = ic
                            if len(extra_fields) >= 16:
                                identify_state = self._try_parse_int(extra_fields[12])
                                identify_error = self._try_parse_int(extra_fields[13])
                                if identify_state is not None:
                                    packet.identify_state = identify_state
                                if identify_error is not None:
                                    packet.identify_error = identify_error
                            if len(extra_fields) >= 20:
                                adc_calib_status = self._try_parse_int(extra_fields[16])
                                adc_offset_a = self._try_parse_int(extra_fields[17])
                                adc_offset_b = self._try_parse_int(extra_fields[18])
                                adc_offset_c = self._try_parse_int(extra_fields[19])
                                if adc_calib_status is not None:
                                    packet.adc_calib_status = adc_calib_status
                                if adc_offset_a is not None:
                                    packet.adc_offset_a = adc_offset_a
                                if adc_offset_b is not None:
                                    packet.adc_offset_b = adc_offset_b
                                if adc_offset_c is not None:
                                    packet.adc_offset_c = adc_offset_c
                            if control_mode is not None:
                                packet.control_mode = control_mode
                            if id_ref is not None:
                                packet.Id_ref = id_ref
                            if speed_ref is not None:
                                packet.speed_ref = speed_ref
                            if pos_ref is not None:
                                packet.pos_ref = pos_ref
                            if iq_ref is not None:
                                packet.Iq_ref = iq_ref
                            if vd is not None:
                                packet.Vd = vd
                            if vq is not None:
                                packet.Vq = vq
                        extra_fields = []

                if len(extra_fields) >= 3:
                    app_fault_code = self._try_parse_int(extra_fields[0])
                    uv = self._try_parse_decimal(extra_fields[-2])
                    ov = self._try_parse_decimal(extra_fields[-1])
                    if app_fault_code is not None and uv is not None and ov is not None:
                        packet.app_fault_code = app_fault_code
                        packet.undervoltage_limit = uv
                        packet.overvoltage_limit = ov
                        extra_fields = []

                if len(extra_fields) >= 2:
                    uv = self._try_parse_decimal(extra_fields[-2])
                    ov = self._try_parse_decimal(extra_fields[-1])
                    if uv is not None and ov is not None:
                        packet.undervoltage_limit = uv
                        packet.overvoltage_limit = ov
                elif len(extra_fields) == 1:
                    app_fault_code = self._try_parse_int(extra_fields[0])
                    if app_fault_code is not None:
                        packet.app_fault_code = app_fault_code

                packet.is_fault_active = (
                    (packet.fault_flags != 0)
                    or (packet.foc_state == self.FOC_STATE_FAULT)
                    or (packet.app_fault_code != 0)
                )
            elif fields[0] == "F" and len(fields) >= 9:
                remaining = fields[6:]
                parse_index = 0
                float_tail = []

                packet.timestamp = int(fields[1])
                packet.foc_state = int(fields[2])
                packet.fault_flags = int(fields[3], 0)
                packet.is_fault_active = True
                packet.encoder_detected = (fields[5] == "1")
                if remaining and self._is_bool_token(remaining[0]):
                    packet.stall_open_loop_active = (remaining[0] == "1")
                    parse_index = 1

                if len(remaining) >= (parse_index + 1):
                    packet.fault_status1 = int(remaining[parse_index], 0)
                if len(remaining) >= (parse_index + 2):
                    packet.vgs_status2 = int(remaining[parse_index + 1], 0)

                trailing_tokens = remaining[parse_index + 3:]
                if len(trailing_tokens) >= 7:
                    app_warning_flags = self._try_parse_int(trailing_tokens[0])
                    app_fault_code = self._try_parse_int(trailing_tokens[1])
                    identify_state = self._try_parse_int(trailing_tokens[2])
                    identify_error = self._try_parse_int(trailing_tokens[3])
                    if (
                        app_warning_flags is not None
                        and app_fault_code is not None
                        and identify_state is not None
                        and identify_error is not None
                    ):
                        packet.app_warning_flags = app_warning_flags
                        packet.app_fault_code = app_fault_code
                        packet.identify_state = identify_state
                        packet.identify_error = identify_error
                        trailing_tokens = trailing_tokens[4:]

                if len(trailing_tokens) >= 7:
                    adc_calib_status = self._try_parse_int(trailing_tokens[0])
                    adc_offset_a = self._try_parse_int(trailing_tokens[1])
                    adc_offset_b = self._try_parse_int(trailing_tokens[2])
                    adc_offset_c = self._try_parse_int(trailing_tokens[3])
                    if (
                        adc_calib_status is not None
                        and adc_offset_a is not None
                        and adc_offset_b is not None
                        and adc_offset_c is not None
                    ):
                        packet.adc_calib_status = adc_calib_status
                        packet.adc_offset_a = adc_offset_a
                        packet.adc_offset_b = adc_offset_b
                        packet.adc_offset_c = adc_offset_c
                        trailing_tokens = trailing_tokens[4:]

                if len(trailing_tokens) >= 5:
                    app_warning_flags = self._try_parse_int(trailing_tokens[0])
                    app_fault_code = self._try_parse_int(trailing_tokens[1])
                    if app_warning_flags is not None and app_fault_code is not None:
                        packet.app_warning_flags = app_warning_flags
                        packet.app_fault_code = app_fault_code
                        trailing_tokens = trailing_tokens[2:]

                if len(trailing_tokens) >= 4:
                    app_fault_code = self._try_parse_int(trailing_tokens[0])
                    if app_fault_code is not None:
                        packet.app_fault_code = app_fault_code
                        trailing_tokens = trailing_tokens[1:]
                elif len(trailing_tokens) == 1:
                    app_fault_code = self._try_parse_int(trailing_tokens[0])
                    if app_fault_code is not None:
                        packet.app_fault_code = app_fault_code
                        trailing_tokens = []

                if len(trailing_tokens) >= 4:
                    adc_calib_status = self._try_parse_int(trailing_tokens[-4])
                    adc_offset_a = self._try_parse_int(trailing_tokens[-3])
                    adc_offset_b = self._try_parse_int(trailing_tokens[-2])
                    adc_offset_c = self._try_parse_int(trailing_tokens[-1])
                    if (
                        adc_calib_status is not None
                        and adc_offset_a is not None
                        and adc_offset_b is not None
                        and adc_offset_c is not None
                    ):
                        packet.adc_calib_status = adc_calib_status
                        packet.adc_offset_a = adc_offset_a
                        packet.adc_offset_b = adc_offset_b
                        packet.adc_offset_c = adc_offset_c
                        trailing_tokens = trailing_tokens[:-4]

                for token in trailing_tokens:
                    value = self._try_parse_decimal(token)
                    if value is not None:
                        float_tail.append(value)

                if len(float_tail) >= 3:
                    packet.vbus = float_tail[-3]
                    packet.undervoltage_limit = float_tail[-2]
                    packet.overvoltage_limit = float_tail[-1]
                elif len(float_tail) >= 2:
                    packet.undervoltage_limit = float_tail[-2]
                    packet.overvoltage_limit = float_tail[-1]
                packet.is_fault_active = True
            else:
                return

            if self.packet_callback:
                self.packet_callback(packet)
        except Exception as e:
            print(f"Compact packet parse error: {e}")
            print(f"Text: {text[:200]}...")
                
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
            else:
                raw_match = re.search(r'AngleRaw:\s*(\d+)', text)
                if raw_match:
                    packet.raw_angle = int(raw_match.group(1))

            if (not angle_match) and packet.raw_angle:
                packet.angle = float(packet.raw_angle) * 360.0 / 65536.0
                
            crc_match = re.search(r'CRC:\s*(ERROR!|OK)', text)
            if crc_match:
                packet.crc_error = (crc_match.group(1) == "ERROR!")

            detected_match = re.search(r'Detected:\s*(YES|NO)', text, flags=re.IGNORECASE)
            if detected_match:
                packet.encoder_detected = (detected_match.group(1).upper() == "YES")
            
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

            vbus_match = re.search(r'Vbus\s*:\s*([\d.\-]+)\s*V', text)
            if vbus_match:
                packet.vbus = float(vbus_match.group(1))
                
            state_match = re.search(r'State\s*:\s*([A-Za-z_]+|\d+)', text)
            if state_match:
                state_value = self._parse_state_token(state_match.group(1))
                if state_value is not None:
                    packet.foc_state = state_value

            identified_match = re.search(r'Identified:\s*(YES|NO)', text, flags=re.IGNORECASE)
            if identified_match:
                packet.motor_identified = (identified_match.group(1).upper() == "YES")

            stall_mode_match = re.search(r'StallMode:\s*(ARMED|OFF)', text, flags=re.IGNORECASE)
            if stall_mode_match:
                packet.stall_mode_armed = (stall_mode_match.group(1).upper() == "ARMED")

            stall_open_loop_match = re.search(r'StallOpenLoop:\s*(ACTIVE|OFF)', text, flags=re.IGNORECASE)
            if stall_open_loop_match:
                packet.stall_open_loop_active = (stall_open_loop_match.group(1).upper() == "ACTIVE")

            fault1_match = re.search(r'FAULT1:\s*0x([0-9A-Fa-f]+)', text)
            if fault1_match:
                packet.fault_status1 = int(fault1_match.group(1), 16)

            vgs2_match = re.search(r'VGS2:\s*0x([0-9A-Fa-f]+)', text)
            if vgs2_match:
                packet.vgs_status2 = int(vgs2_match.group(1), 16)

            mode_match = re.search(r'Power:\s*.*?\bmode=(\d+)', text, flags=re.IGNORECASE)
            if mode_match:
                packet.control_mode = int(mode_match.group(1))
            
            # 解析故障信息
            app_fault_match = re.search(r'AppFault:\s*(\d+)', text)
            if app_fault_match:
                packet.app_fault_code = int(app_fault_match.group(1))

            param_diag_match = re.search(
                r'ParamDiag:\s*.*?Rs=([\d.\-]+)\s+Ohm\s*\|\s*Ld=([\d.\-]+)\s+H\s*\|\s*'
                r'Lq=([\d.\-]+)\s+H\s*\|\s*Ke=([\d.\-]+)\s*\|\s*Pn=(\-?\d+)\s*\|\s*enc_dir=(\-?\d+)',
                text,
            )
            if param_diag_match:
                packet.motor_param_rs = float(param_diag_match.group(1))
                packet.motor_param_ld = float(param_diag_match.group(2))
                packet.motor_param_lq = float(param_diag_match.group(3))
                packet.motor_param_ke = float(param_diag_match.group(4))
                packet.motor_param_pn = int(param_diag_match.group(5))
                packet.motor_param_encoder_dir = int(param_diag_match.group(6))

            # 解析 ThetaDiag 行: offset 和 zero
            theta_diag_match = re.search(
                r'ThetaDiag:.*?offset=([\d.\-]+)\s*rad.*?zero=([\d.\-]+)\s*rad',
                text,
            )
            if theta_diag_match:
                packet.motor_param_theta_offset = float(theta_diag_match.group(1))
                packet.motor_param_mech_zero = float(theta_diag_match.group(2))

            # Detailed diagnostic snapshots may reuse the historical
            # "FAULT DETECTED" envelope even while the controller is RUNNING.
            # Treat the packet as a fault only when the payload carries a real
            # fault source, not merely because of the envelope title.
            packet.is_fault_active = self._detailed_packet_has_active_fault(text, packet)
            
            # 调用回调
            if self.packet_callback:
                self.packet_callback(packet)
                
        except Exception as e:
            print(f"Packet parse error: {e}")
            print(f"Text: {text[:200]}...")


def control_to_user_angle(pos_ref_control: float, encoder_dir: int) -> float:
    """将固件内部控制帧的pos_ref逆转换为用户角度（rad）。

    固件FOC_App_SetPositionRef会将用户PREF乘以encoder_dir再归一化存储，
    上位机显示时需要做逆转换。encoder_dir=-1时乘(-1)等于取反再归一化。
    """
    import math
    dir_f = float(encoder_dir) if encoder_dir != 0 else 1.0
    raw = pos_ref_control * dir_f
    # 归一化到 [0, 2π)
    two_pi = 2.0 * math.pi
    raw = raw - two_pi * math.floor(raw / two_pi)
    if raw < 0.0:
        raw += two_pi
    return raw


class CommandBuilder:
    """命令构建器 - 构建发送给下位机的命令"""

    @staticmethod
    def unlock_power(unlock: bool) -> str:
        """解锁/上锁功率级"""
        return f"CMD:UNLOCK,{1 if unlock else 0}\n"
    
    @staticmethod
    def enable_motor(enable: bool) -> str:
        """使能/禁用电机"""
        return f"CMD:ENABLE,{1 if enable else 0}\n"
    
    @staticmethod
    def set_mode(mode: int) -> str:
        """设置控制模式 0=力矩 1=速度 2=位置"""
        return f"CMD:MODE,{mode}\n"

    @staticmethod
    def set_stall_mode(enable: bool) -> str:
        """设置堵转模式授权"""
        return f"CMD:STALL_MODE,{1 if enable else 0}\n"

    @staticmethod
    def set_motor_pn(pole_pairs: int) -> str:
        """设置电机极对数"""
        return f"CMD:MOTOR_PN,{int(pole_pairs)}\n"

    @staticmethod
    def set_encoder_dir(direction: int) -> str:
        """设置编码器方向（+1 或 -1）"""
        return f"CMD:ENCODER_DIR,{int(direction)}\n"
    
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
    def set_motion_cfg(speed: float, accel: float, cruise: float) -> str:
        """Set V5 position-mode motion config."""
        return f"CMD:MOTION_CFG,{speed:.3f},{accel:.3f},{cruise:.3f}\n"

    @staticmethod
    def query_motion_cfg() -> str:
        """Query V5 position-mode motion config."""
        return "CMD:MOTION_CFG?\n"

    @staticmethod
    def reset_motion_cfg() -> str:
        """Reset V5 motion config to firmware defaults."""
        return "CMD:MOTION_CFG_RESET\n"

    @staticmethod
    def set_cogging_cfg(gain: float, phase_deg: float) -> str:
        """Set P0 cogging feedforward gain and phase in degrees."""
        return f"CMD:COG_CFG,{gain:.3f},{phase_deg:.3f}\n"

    @staticmethod
    def query_cogging_cfg() -> str:
        """Query P0 cogging feedforward config."""
        return "CMD:COG_CFG?\n"
    
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
    def set_vbus_limits(undervoltage: float, overvoltage: float) -> str:
        """设置欠压/过压阈值"""
        return f"CMD:VBUS_LIMIT,{undervoltage:.3f},{overvoltage:.3f}\n"

    @staticmethod
    def adc_noise_test(samples: int) -> str:
        """启动ADC噪声统计诊断"""
        return f"CMD:ADC_NOISE,{int(samples)}\n"

    @staticmethod
    def adc_phase_scan(samples: int) -> str:
        """启动ADC触发相位扫描诊断"""
        return f"CMD:ADC_PHASE_SCAN,{int(samples)}\n"

    @staticmethod
    def adc_sector_scan(samples: int) -> str:
        """启动ADC扇区采样统计诊断"""
        return f"CMD:ADC_SECTOR_SCAN,{int(samples)}\n"

    @staticmethod
    def tle_gpio_diag(enable: bool) -> str:
        """启动/停止TLE5012 GPIO诊断"""
        return f"CMD:TLE_GPIO_DIAG,{1 if enable else 0}\n"

    @staticmethod
    def tle_raw() -> str:
        """读取TLE5012原始诊断帧"""
        return "CMD:TLE_RAW\n"

    @staticmethod
    def fault_detail() -> str:
        """请求下位机立即上传完整故障/运行诊断"""
        return "CMD:FAULT_DETAIL\n"
    
    @staticmethod
    def set_current_pi(kp: float, ki: float) -> str:
        """设置电流环PI参数"""
        return f"CMD:PI_CURRENT,{kp:.6f},{ki:.6f}\n"
    
    @staticmethod
    def set_speed_pi(kp: float, ki: float) -> str:
        """设置速度环PI参数"""
        return f"CMD:PI_SPEED,{kp:.6f},{ki:.6f}\n"
    
    @staticmethod
    def set_position_pd(kp: float, kd: float) -> str:
        """设置位置环PD参数"""
        return f"CMD:PD_POS,{kp:.6f},{kd:.6f}\n"

    @staticmethod
    def set_home() -> str:
        """将当前位置设为机械零点"""
        return "CMD:HOME\n"

    @staticmethod
    def clear_home() -> str:
        """清除机械零点偏移"""
        return "CMD:CLEAR_HOME\n"

    # ── Joint Product Mode (V1.2) ──────────────────────────────────────────

    _VALID_APP_MODES = {"RAW", "JOINT_POS", "GIMBAL_SPEED", "HOLD", "SPRING_DAMPER", "DETENT"}

    @staticmethod
    def app_mode_query() -> str:
        """Query current APP_MODE. Response: APP_MODE,OK,<NAME> (ctrl_mode=<N>)"""
        return "CMD:APP_MODE?\n"

    @staticmethod
    def app_mode_set(mode: str) -> str:
        """Set APP_MODE. mode: RAW|JOINT_POS|GIMBAL_SPEED|HOLD|SPRING_DAMPER|DETENT"""
        upper = mode.upper()
        if upper not in CommandBuilder._VALID_APP_MODES:
            raise ValueError(f"Invalid APP_MODE '{mode}'. Must be one of {sorted(CommandBuilder._VALID_APP_MODES)}")
        return f"CMD:APP_MODE,{upper}\n"

    @staticmethod
    def joint_limit_query() -> str:
        """Query joint soft limits. Response: JOINT:LIMIT,OK,min=X.Xdeg,max=Y.Ydeg or JOINT:LIMIT,OK,OFF"""
        return "JOINT:LIMIT?\n"

    @staticmethod
    def joint_limit_set(min_deg: float, max_deg: float) -> str:
        """Set joint soft limits in degrees."""
        return f"JOINT:LIMIT,{min_deg:.1f},{max_deg:.1f}\n"

    @staticmethod
    def joint_limit_off() -> str:
        """Disable joint soft limits."""
        return "JOINT:LIMIT,OFF\n"

    @staticmethod
    def gimbal_ramp_query() -> str:
        """Query gimbal ramp acceleration. Response: GIMBAL:RAMP,OK,accel=X.Xradps2"""
        return "GIMBAL:RAMP?\n"

    @staticmethod
    def gimbal_ramp_set(accel: float) -> str:
        """Set gimbal ramp acceleration in rad/s^2. Range 0.1-20.0."""
        return f"GIMBAL:RAMP,{accel:.1f}\n"

    @staticmethod
    def spring_cfg_query() -> str:
        """Query spring-damper config. Response: SPRING:CFG,OK,K=X.XXX,D=Y.YYY,limit=Z.ZZZ"""
        return "SPRING:CFG?\n"

    @staticmethod
    def spring_cfg_set(K: float, D: float, limit: float) -> str:
        """Set spring-damper params: stiffness K, damping D, torque limit (3 params required)."""
        return f"SPRING:CFG,{K:.3f},{D:.3f},{limit:.3f}\n"

    @staticmethod
    def detent_cfg_query() -> str:
        """Query detent config. Response: DETENT:CFG,OK,count=X,strength=Y.YYY,width=Z.ZZZ,limit=W.WWW"""
        return "DETENT:CFG?\n"

    @staticmethod
    def detent_cfg_set(count: int, strength: float, width: float, limit: float) -> str:
        """Set detent params: count, strength, width, torque limit."""
        return f"DETENT:CFG,{int(count)},{strength:.3f},{width:.3f},{limit:.3f}\n"


# ── Binary Current Stream ────────────────────────────────────────────────────

@dataclass
class CurrentSample:
    """Single sample from the 2kHz binary current stream."""
    seq: int = 0
    tick_ms: int = 0
    ia: float = 0.0       # A
    ib: float = 0.0
    ic: float = 0.0
    id: float = 0.0
    iq: float = 0.0
    vbus: float = 0.0     # V
    flags: int = 0


class BinaryCurrentParser:
    """Scans byte stream for A5 5A binary current frames, validates CRC-8,
    and returns decoded CurrentSample objects. Residual text bytes are returned
    for the ASCII line parser.

    Binary frame format (25 bytes):
      0-1:   Sync  0xA5 0x5A
      2:     Type  0x43 ('C')
      3:     Len   20
      4-23:  Payload (20 bytes, little-endian)
      24:    CRC-8 (poly 0x07, init 0x00, over bytes 0-23)
    """

    SYNC = b'\xA5\x5A'
    FRAME_LEN = 25
    PAYLOAD_LEN = 20

    # CRC-8 table (poly 0x07, init 0x00)
    _CRC8_TABLE = [
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
        0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
        0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
        0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
        0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
        0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
        0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
        0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
        0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
        0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
        0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
        0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
        0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
        0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
        0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
        0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
    ]

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors: int = 0
        self.frames_decoded: int = 0
        self.last_seq: int = -1
        self.seq_gaps: int = 0

    def reset_stats(self):
        self.crc_errors = 0
        self.frames_decoded = 0
        self.last_seq = -1
        self.seq_gaps = 0

    def feed(self, data: bytes) -> Tuple[List[CurrentSample], bytes]:
        """Feed raw bytes. Returns (samples, residual_text_bytes).

        Binary frames (A5 5A ...) are consumed and decoded.
        Bytes not belonging to any valid binary frame are returned as residual text.
        """
        self._buf.extend(data)
        samples: List[CurrentSample] = []
        residual = bytearray()

        while len(self._buf) >= self.FRAME_LEN:
            # Find next sync
            sync_idx = self._buf.find(self.SYNC)
            if sync_idx == -1:
                # No sync found — all remaining bytes are text
                residual.extend(self._buf)
                self._buf.clear()
                break

            if sync_idx > 0:
                # Bytes before sync are text
                residual.extend(self._buf[:sync_idx])
                del self._buf[:sync_idx]

            # Need at least a full frame
            if len(self._buf) < self.FRAME_LEN:
                break

            # Peek at the candidate frame
            candidate = self._buf[:self.FRAME_LEN]

            # Quick sanity: type byte must be 0x43, length must be 20
            if candidate[2] != 0x43 or candidate[3] != self.PAYLOAD_LEN:
                # False sync — skip the first sync byte, emit as text
                residual.append(self._buf.pop(0))
                continue

            # Validate CRC-8
            crc = self._compute_crc8(candidate[:24])
            if crc != candidate[24]:
                self.crc_errors += 1
                # Skip the sync marker, try again
                residual.append(self._buf.pop(0))
                residual.append(self._buf.pop(0))  # pop both sync bytes
                continue

            # Valid frame — decode
            sample = self._decode_frame(candidate)
            samples.append(sample)
            del self._buf[:self.FRAME_LEN]

        return samples, bytes(residual)

    def _compute_crc8(self, data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc = self._CRC8_TABLE[crc ^ b]
        return crc

    def _decode_frame(self, frame: bytes) -> CurrentSample:
        """Decode a validated 25-byte binary frame."""
        payload = frame[4:24]
        seq, tick_ms, ia_mA, ib_mA, ic_mA, id_mA, iq_mA, vbus_mV, flags = \
            struct.unpack('<H I h h h h h H H', payload)

        # Seq gap detection
        if self.last_seq >= 0:
            expected = (self.last_seq + 1) & 0xFFFF
            if seq != expected:
                self.seq_gaps += 1
        self.last_seq = seq
        self.frames_decoded += 1

        return CurrentSample(
            seq=seq,
            tick_ms=tick_ms,
            ia=ia_mA * 0.001,
            ib=ib_mA * 0.001,
            ic=ic_mA * 0.001,
            id=id_mA * 0.001,
            iq=iq_mA * 0.001,
            vbus=vbus_mV * 0.001,
            flags=flags,
        )
