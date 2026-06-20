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
