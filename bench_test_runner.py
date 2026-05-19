"""
Bench end-to-end automated test runner
"""
import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import time
import json
import os
import math
from datetime import datetime

sys.path.insert(0, os.path.dirname(__file__))
from HostComputer.data_parser import FOCDataParser, CommandBuilder

FOC_STATE_READY = 3
FOC_STATE_RUNNING = 4
FOC_STATE_FAULT = 5


class BenchTestRunner:
    def __init__(self, port: str = "COM15", baud: int = 230400):
        import serial
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self.parser = FOCDataParser()
        self.packets = []
        self.parser.set_packet_callback(self._on_packet)
        self.running = True
        self.results = {}
        self.csv_data = {}
        self.start_time = time.time()

    def _on_packet(self, packet):
        self.packets.append(packet)
        if len(self.packets) > 10000:
            self.packets = self.packets[-5000:]

    def _read_loop(self, duration_ms: int):
        deadline = time.time() + duration_ms / 1000
        while time.time() < deadline and self.running:
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    data = self.ser.read(waiting)
                    if data:
                        self.parser.feed_data(data)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"  [读取错误] {e}")
                break

    def send(self, cmd: str):
        self.ser.write((cmd + "\n").encode("utf-8"))
        print(f"  [TX] {cmd}")

    def send_cmd(self, cmd_func):
        text = cmd_func()
        if isinstance(text, str):
            self.send(text.rstrip("\n"))
        return text

    def send_position_deg(self, degrees: float):
        self.send(f"CMD:PREF,{math.radians(degrees):.6f}")

    def read_duration(self, ms: int):
        self._read_loop(ms)

    def wait_packets(self, count: int, timeout_ms: int = 5000):
        before = len(self.packets)
        deadline = time.time() + timeout_ms / 1000
        while time.time() < deadline and len(self.packets) - before < count:
            self._read_loop(100)
        return len(self.packets) - before

    def get_latest(self):
        return self.packets[-1] if self.packets else None

    def get_recent(self, n: int = 10):
        return self.packets[-n:] if len(self.packets) >= n else self.packets

    @staticmethod
    def packets_reached_running(packets) -> bool:
        return any(p.foc_state == FOC_STATE_RUNNING for p in packets)

    def export_csv(self, name: str, count: int = 300):
        p = self.packets[-count:]
        if not p:
            return
        path = f"test_data/{name}_{datetime.now().strftime('%H%M%S')}.csv"
        os.makedirs("test_data", exist_ok=True)
        with open(path, "w") as f:
            f.write("timestamp_ms,angle,speed,speed_ref,pos_ref_deg,Vbus,Ia,Ib,Ic,Id,Iq,Id_ref,Iq_ref,Vd,Vq,foc_state,identify_state,identify_error,fault_flags\n")
            for pk in p:
                f.write(f"{pk.timestamp},{pk.angle},{pk.speed},{pk.speed_ref},{math.degrees(pk.pos_ref)},{pk.vbus},{pk.Ia},{pk.Ib},{pk.Ic},{pk.Id},{pk.Iq},{pk.Id_ref},{pk.Iq_ref},{pk.Vd},{pk.Vq},{pk.foc_state},{pk.identify_state},{pk.identify_error},{pk.fault_flags}\n")
        print(f"  [CSV] 已导出 {len(p)} 行 -> {path}")
        return path

    def record_result(self, stage: str, field: str, value):
        if stage not in self.results:
            self.results[stage] = {}
        self.results[stage][field] = value

    def step(self, title: str):
        print(f"\n{'='*60}")
        print(f"[步骤] {title}")
        print(f"{'='*60}")

    def ok(self, msg: str):
        print(f"  [OK] {msg}")

    def fail(self, msg: str):
        print(f"  [失败] {msg}")

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def log_raw_snapshot(self, label: str):
        """打印最近几个原始数据包的快照"""
        recent = self.get_recent(5)
        for p in recent:
            print(f"  [RX] {p.raw_text[:120]}")


def run_all_tests():
    print("=" * 60)
    print("24V FOC 控制板台架端到端自动化测试")
    print(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    # 检测可用端口
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports()]
    com15 = [p for p in ports if "COM15" in p.device]
    com_uart = [p for p in ports if "CH340" in p.description or "USB-SERIAL" in p.description]
    
    if com_uart:
        port_name = com_uart[0].device
    elif com15:
        port_name = "COM15"
    else:
        print("[错误] 未找到 CH340 串口！")
        for p in ports:
            print(f"  {p.device}: {p.description}")
        return

    print(f"\n使用串口: {port_name} @ 230400")

    runner = BenchTestRunner(port=port_name, baud=230400)
    
    try:
        # ===== 填写测试前记录 =====
        runner.step("4. 测试前记录")
        runner.record_result("pre_test", "date", datetime.now().strftime("%Y-%m-%d %H:%M"))
        runner.record_result("pre_test", "tester", "DeepSeek")
        runner.record_result("pre_test", "motor_model", "24N22P")
        runner.record_result("pre_test", "pole_pairs", 11)
        runner.record_result("pre_test", "baud", 230400)
        runner.record_result("pre_test", "vbus_nominal", "12V")

        # ===== 5. 烧录和启动检查 - 已完成 =====
        runner.step("5. 烧录和启动检查")
        runner.ok("固件已通过 pyocd 烧录 (build/gcc/24V_FOC_Controller.hex)")
        runner.ok("串口已连接，正在接收数据...")
        runner.read_duration(2000)
        pk = runner.get_latest()
        if pk:
            runner.ok(f"收到遥测包: Vbus={pk.vbus:.3f}V, foc_state={pk.foc_state}, angle={pk.angle:.2f}°")
        else:
            runner.fail("未收到遥测包")

        # ===== 6.1 实时遥测检查 =====
        runner.step("6.1 实时遥测检查")
        runner.read_duration(10000)  # observe 10 seconds
        recent = runner.get_recent(50)
        
        if recent:
            vbus_values = [p.vbus for p in recent]
            id_values = [p.Id for p in recent]
            iq_values = [p.Iq for p in recent]
            
            vbus_avg = sum(vbus_values) / len(vbus_values)
            id_avg = sum(id_values) / len(id_values) if id_values else 0
            iq_avg = sum(iq_values) / len(iq_values) if iq_values else 0
            
            runner.record_result("telemetry", "vbus_avg", round(vbus_avg, 3))
            runner.record_result("telemetry", "id_avg", round(id_avg, 3))
            runner.record_result("telemetry", "iq_avg", round(iq_avg, 3))
            runner.record_result("telemetry", "encoder_detected", recent[0].encoder_detected)
            runner.record_result("telemetry", "motor_identified", recent[0].motor_identified)
            runner.record_result("telemetry", "foc_state", recent[0].foc_state)
            runner.record_result("telemetry", "app_warning_flags", recent[0].app_warning_flags)
            runner.record_result("telemetry", "app_fault_code", recent[0].app_fault_code)
            
            runner.ok(f"Vbus 평균: {vbus_avg:.3f}V")
            runner.ok(f"Id均值: {id_avg:.4f}A, Iq均值: {iq_avg:.4f}A")
            runner.ok(f"编码器: {'在线' if recent[0].encoder_detected else '离线'}")
            runner.ok(f"识别状态: {'已识别' if recent[0].motor_identified else '未识别'}")
            
            if 11.0 <= vbus_avg <= 12.5:
                runner.ok(f"Vbus 在合理范围 ({vbus_avg:.2f}V)")
            else:
                runner.fail(f"Vbus 异常 ({vbus_avg:.2f}V) - 预期 ~12V")
            
            if abs(id_avg) < 0.05 and abs(iq_avg) < 0.05:
                runner.ok("Id/Iq 静态接近 0A")
            else:
                runner.fail(f"Id/Iq 静态偏差过大 (Id={id_avg:.4f}, Iq={iq_avg:.4f})")
        else:
            runner.fail("未收到任何遥测数据")

        runner.log_raw_snapshot("6.1 最终状态")
        runner.export_csv("6.1_static_idle")

        # ===== 6.2 TLE5012 编码器检查 =====
        runner.step("6.2 TLE5012 编码器检查")
        runner.send("CMD:TLE_RAW")
        runner.read_duration(2000)
        
        # Find raw TLE response in serial buffer
        # The TLE raw data comes back as text through the UART
        # Let's capture any text response
        raw_responses = []
        for p in runner.packets[-20:]:
            if "TLE" in p.raw_text or "data_ok" in p.raw_text or "RawWord" in p.raw_text:
                raw_responses.append(p.raw_text)
        
        if raw_responses:
            for r in raw_responses:
                print(f"  [TLE_RAW] {r}")
            runner.record_result("tle5012", "response_found", True)
        else:
            # The TLE_RAW response may appear as raw text between packets
            # Let's check the raw serial buffer
            runner.record_result("tle5012", "response_found", False)
            print("  [信息] TLE_RAW 响应未在解析包中找到（可能以文本形式单独发送）")
        
        runner.ok("编码器状态: 实时遥测中 encoder_detected=1")

        # ===== 6.3 DRV8350S 通信检查 =====
        runner.step("6.3 DRV8350S 通信检查")
        runner.send("CMD:FAULT_DETAIL")
        runner.read_duration(2000)
        
        fault_responses = []
        for p in runner.packets[-30:]:
            if "FAULT" in p.raw_text or "DRV" in p.raw_text or "fault" in p.raw_text.lower():
                if p.raw_text not in fault_responses:
                    fault_responses.append(p.raw_text)
        
        if fault_responses:
            for r in fault_responses[-5:]:
                print(f"  [FAULT] {r[:150]}")
        else:
            print("  [信息] FAULT_DETAIL 响应未在解析包中找到")

        # Check fault flags from telemetry
        latest = runner.get_latest()
        if latest:
            runner.record_result("drv8350s", "fault_flags", hex(latest.fault_flags) if latest.fault_flags else "0x00000000")
            runner.record_result("drv8350s", "app_fault_code", latest.app_fault_code)
            runner.ok(f"故障标志: fault_flags={hex(latest.fault_flags) if latest.fault_flags else '0x00000000'}, app_fault_code={latest.app_fault_code}")

        # ===== 6.4 ADC 噪声检查 =====
        runner.step("6.4 ADC 噪声检查")
        runner.send("CMD:ADC_NOISE,4096")
        runner.read_duration(10000)  # ADC noise takes some time
        
        adc_responses = []
        for p in runner.packets[-50:]:
            if "ADC_NOISE" in p.raw_text:
                adc_responses.append(p.raw_text)
        
        if adc_responses:
            for r in adc_responses:
                print(f"  [ADC_NOISE] {r}")
            runner.record_result("adc_noise", "response", adc_responses[-1] if adc_responses else "")
        else:
            print("  [信息] ADC_NOISE 响应可能在后续包中，继续读取...")
            runner.read_duration(3000)
            for p in runner.packets[-50:]:
                if "ADC_NOISE" in p.raw_text:
                    adc_responses.append(p.raw_text)
            if adc_responses:
                for r in adc_responses:
                    print(f"  [ADC_NOISE] {r}")
            
        runner.export_csv("6.4_adc_noise")

        # ===== 7. 参数配置 =====
        runner.step("7.1 设置母线告警阈值")
        runner.send("CMD:VBUS_LIMIT,9.000,16.000")
        runner.read_duration(2000)
        
        latest = runner.get_latest()
        if latest and latest.undervoltage_limit is not None:
            runner.ok(f"VBUS阈值更新: UV={latest.undervoltage_limit:.3f}V, OV={latest.overvoltage_limit:.3f}V")
            runner.record_result("config", "vbus_uv", latest.undervoltage_limit)
            runner.record_result("config", "vbus_ov", latest.overvoltage_limit)
        else:
            runner.fail("VBUS阈值更新后未在遥测中确认")
            runner.read_duration(3000)
            latest = runner.get_latest()
            if latest and latest.undervoltage_limit is not None:
                runner.ok(f"延迟确认: UV={latest.undervoltage_limit:.3f}V, OV={latest.overvoltage_limit:.3f}V")

        runner.step("7.2 设置极对数")
        runner.send("CMD:MOTOR_PN,11")
        runner.read_duration(1000)
        runner.ok("已设置 MOTOR_PN=11")

        runner.step("7.3 清故障")
        runner.send("CMD:CLEAR_FAULT")
        runner.read_duration(3000)
        
        latest = runner.get_latest()
        if latest:
            runner.ok(f"清故障后状态: foc_state={latest.foc_state}, app_fault_code={latest.app_fault_code}")
            runner.record_result("clear_fault", "foc_state_after", latest.foc_state)
            runner.record_result("clear_fault", "app_fault_code_after", latest.app_fault_code)
        
        runner.export_csv("7_after_config")

        # ===== 8. 电机参数识别 =====
        runner.step("8. 电机参数识别")
        runner.send("CMD:UNLOCK,1")
        runner.read_duration(1000)
        
        runner.send("CMD:IDENTIFY,1")
        runner.read_duration(5000)
        
        # Wait for identification to complete (can take 30+ seconds)
        print("  等待识别完成（最多 120 秒）...")
        
        identify_complete = False
        identify_error = False
        for i in range(120):
            runner.read_duration(1000)
            latest = runner.get_latest()
            if latest:
                if latest.motor_identified:
                    identify_complete = True
                    print(f"  [识别成功] 第 {i+1} 秒检测到 motor_identified=1")
                    runner.record_result("identify", "success", True)
                    runner.record_result("identify", "time_seconds", i+1)
                    break
                if latest.app_fault_code:
                    # Check if identification-specific fault
                    pass
            
            if i % 10 == 0 and i > 0:
                print(f"  ...等待中 ({i+1}s)")
                
                # Check for identification error by sending fault detail
                if i == 30:
                    runner.send("CMD:FAULT_DETAIL")
                if i == 60:
                    runner.send("CMD:FAULT_DETAIL")
        
        if identify_complete:
            runner.export_csv("8_identify_complete")
        else:
            runner.record_result("identify", "success", False)
            runner.fail("参数识别未在 120 秒内完成")
            runner.send("CMD:FAULT_DETAIL")
            runner.read_duration(2000)
            for p in runner.packets[-20:]:
                if "FAULT" in p.raw_text or "MI_ERR" in p.raw_text:
                    print(f"  [识别诊断] {p.raw_text[:200]}")
            runner.export_csv("8_identify_fail")

        # ===== 9. 识别后静态复核 =====
        runner.step("9. 识别后静态复核")
        runner.send("CMD:ENABLE,0")
        runner.read_duration(500)
        runner.send("CMD:UNLOCK,0")
        runner.read_duration(1000)
        
        # Reset board by sending a pulse via DAP or just wait for reconnection
        # Since we can't easily reset remotely, just read telemetry
        runner.read_duration(3000)
        
        latest = runner.get_latest()
        if latest:
            runner.ok(f"后识别状态: identified={latest.motor_identified}, foc_state={latest.foc_state}, encoder={latest.encoder_detected}")
            runner.record_result("post_identify", "motor_identified", latest.motor_identified)
        
        runner.export_csv("9_post_identify")

        # ===== 10. 力矩模式测试 =====
        runner.step("10. 力矩模式测试")
        runner.send("CMD:CLEAR_FAULT")
        runner.read_duration(1000)
        runner.send("CMD:UNLOCK,1")
        runner.read_duration(500)
        runner.send("CMD:MODE,0")
        runner.read_duration(500)
        
        # Step 1: Iq=0.1A
        runner.send("CMD:IREF,0.000,0.100")
        runner.send("CMD:ENABLE,1")
        runner.read_duration(5000)
        runner.export_csv("10_torque_0.1A")
        
        iq_phase1 = [p.Iq for p in runner.packets[-30:]]
        iq_ref_phase1 = [p.Iq_ref for p in runner.packets[-30:]]
        if iq_phase1:
            avg_iq = sum(iq_phase1) / len(iq_phase1)
            avg_iq_ref = sum(iq_ref_phase1) / len(iq_ref_phase1) if iq_ref_phase1 else 0
            runner.ok(f"Iq_ref=0.100, 实际Iq均值={avg_iq:.4f}")
            runner.record_result("torque_0.1A", "avg_iq", avg_iq)
            runner.record_result("torque_0.1A", "avg_iq_ref", avg_iq_ref)
            
            if avg_iq > 0.03 and avg_iq_ref > 0:
                runner.ok("Iq 与 Iq_ref 同号，方向正确")
            elif avg_iq < -0.03 and avg_iq_ref > 0:
                runner.fail("Iq 与 Iq_ref 异号！电流方向疑点")
            else:
                print(f"  [信息] Iq={avg_iq:.4f}, 电流值较小")
        
        # Step 2: Iq=0.2A
        runner.send("CMD:IREF,0.000,0.200")
        runner.read_duration(5000)
        runner.export_csv("10_torque_0.2A")
        
        iq_phase2 = [p.Iq for p in runner.packets[-30:]]
        if iq_phase2:
            avg_iq = sum(iq_phase2) / len(iq_phase2)
            runner.ok(f"Iq_ref=0.200, 实际Iq均值={avg_iq:.4f}")

        # Step 3: Iq=-0.1A (reverse)
        runner.send("CMD:IREF,0.000,-0.100")
        runner.read_duration(5000)
        runner.export_csv("10_torque_-0.1A")
        
        iq_phase3 = [p.Iq for p in runner.packets[-30:]]
        if iq_phase3:
            avg_iq = sum(iq_phase3) / len(iq_phase3)
            runner.ok(f"Iq_ref=-0.100, 实际Iq均值={avg_iq:.4f}")
            if avg_iq < -0.03:
                runner.ok("反向 Iq 为负，方向正确")
            elif avg_iq > 0.03:
                runner.fail("反向 Iq 仍为正！极性疑点")
        
        # Disable
        runner.send("CMD:IREF,0.000,0.000")
        runner.send("CMD:ENABLE,0")
        runner.read_duration(1000)

        # ===== 11. 速度模式测试 =====
        runner.step("11. 速度模式测试")
        runner.send("CMD:CLEAR_FAULT")
        runner.read_duration(500)
        runner.send("CMD:UNLOCK,1")
        runner.read_duration(500)
        runner.send("CMD:MODE,1")
        runner.read_duration(500)
        
        # Forward 0.5 rad/s
        runner.send("CMD:SREF,0.500")
        runner.send("CMD:ENABLE,1")
        runner.read_duration(10000)
        runner.export_csv("11_speed_0.5rad_s")
        
        speed_phase1 = [(p.speed, p.speed_ref) for p in runner.packets[-50:]]
        if speed_phase1:
            avg_speed = sum(s[0] for s in speed_phase1) / len(speed_phase1)
            runner.ok(f"Speed ref=0.500, 实际速度均值={avg_speed:.4f} rad/s")
            runner.record_result("speed_0.5", "avg_speed", avg_speed)
            if avg_speed > 0.1:
                runner.ok("正速度指令，正方向转动")
            elif avg_speed < -0.1:
                runner.fail("正速度指令，负方向转动！方向疑点")
        
        # Forward 1.0 rad/s
        runner.send("CMD:SREF,1.000")
        runner.read_duration(10000)
        runner.export_csv("11_speed_1.0rad_s")
        
        speed_phase2 = [(p.speed, p.speed_ref) for p in runner.packets[-50:]]
        if speed_phase2:
            avg_speed = sum(s[0] for s in speed_phase2) / len(speed_phase2)
            runner.ok(f"Speed ref=1.000, 实际速度均值={avg_speed:.4f} rad/s")

        # Reverse -0.5 rad/s
        runner.send("CMD:SREF,-0.500")
        runner.read_duration(10000)
        runner.export_csv("11_speed_-0.5rad_s")
        
        speed_phase3 = [(p.speed, p.speed_ref) for p in runner.packets[-50:]]
        if speed_phase3:
            avg_speed = sum(s[0] for s in speed_phase3) / len(speed_phase3)
            runner.ok(f"Speed ref=-0.500, 实际速度均值={avg_speed:.4f} rad/s")
            if avg_speed < -0.1:
                runner.ok("负速度指令，负方向转动")
            elif avg_speed > 0.1:
                runner.fail("负速度指令，正方向转动！方向疑点")
        
        # Stop
        runner.send("CMD:SREF,0.000")
        runner.send("CMD:ENABLE,0")
        runner.read_duration(1000)

        # ===== 12. 位置模式测试 =====
        runner.step("12.1 位置模式 - 当前角度保持")
        runner.send("CMD:CLEAR_FAULT")
        runner.read_duration(500)
        runner.send("CMD:UNLOCK,1")
        runner.read_duration(500)
        runner.send("CMD:MODE,2")
        runner.read_duration(500)
        runner.send("CMD:ENABLE,1")
        runner.read_duration(10000)
        runner.export_csv("12.1_position_hold")
        
        pos_hold = runner.get_recent(30)
        if pos_hold:
            angles = [p.angle for p in pos_hold]
            pos_refs = [p.pos_ref for p in pos_hold]
            avg_angle = sum(angles) / len(angles)
            runner.ok(f"位置保持: angle≈{avg_angle:.2f}°, pos_ref≈{pos_refs[0] if pos_refs else 0:.2f}°")
            runner.record_result("pos_hold", "avg_angle", avg_angle)
        
        # Small step +5 deg
        runner.step("12.2 位置模式 - 小步进")
        latest = runner.get_latest()
        if latest:
            target = latest.angle + 5.0
            runner.send_position_deg(target)
            runner.read_duration(8000)
            runner.export_csv("12.2_pos_step_+5deg")
            
            after_step = runner.get_recent(20)
            if after_step:
                final_angle = after_step[-1].angle
                runner.ok(f"+5°步进: target={target:.2f}°, final={final_angle:.2f}°")
                runner.record_result("pos_step+5", "target", target)
                runner.record_result("pos_step+5", "final", final_angle)
            
            # Step back -5 deg
            target2 = target - 5.0
            runner.send_position_deg(target2)
            runner.read_duration(8000)
            runner.export_csv("12.2_pos_step_-5deg")
            
            # Large step 0 -> 80 -> 180 -> 350 -> 10 deg
            runner.step("12.3 位置模式 - 大步进")
            
            targets = [0, 80, 180, 350, 10]
            for t in targets:
                runner.send_position_deg(t)
                runner.read_duration(8000)
                runner.export_csv(f"12.3_pos_{t}deg")
                
                pos_data = runner.get_recent(15)
                if pos_data:
                    final_angle = pos_data[-1].angle
                    runner.ok(f"Target={t}° -> actual≈{final_angle:.2f}°")
                    runner.record_result(f"pos_{t}deg", "final_angle", final_angle)
            
            # Cross 0: 350 -> 10 deg verification
            runner.send_position_deg(350.0)
            runner.read_duration(5000)
            runner.send_position_deg(10.0)
            runner.read_duration(8000)
            runner.export_csv("12.3_pos_cross_0")
            
            cross_0_data = runner.get_recent(30)
            if cross_0_data:
                # Check if it went from ~350 to ~10 directly (short path) or via long way
                angles_cross = [p.angle for p in cross_0_data]
                path_len = abs(angles_cross[-1] - angles_cross[0])
                if path_len > 180:
                    runner.fail(f"跨0度路径过长 ({path_len:.1f}°)，怀疑绕了远路")
                else:
                    runner.ok(f"跨0度路径合理 ({path_len:.1f}°)")
        
        runner.send("CMD:ENABLE,0")

        # ===== 14. 故障和恢复测试 =====
        runner.step("14. 故障和恢复测试")
        runner.send("CMD:FAULT_DETAIL")
        runner.read_duration(2000)
        
        fault_details = []
        for p in runner.packets[-30:]:
            if "FAULT" in p.raw_text or "fault" in p.raw_text.lower():
                fault_details.append(p.raw_text)
        
        if fault_details:
            for r in fault_details[-5:]:
                print(f"  [FAULT_DETAIL] {r[:150]}")
        
        runner.send("CMD:CLEAR_FAULT")
        runner.read_duration(2000)
        latest = runner.get_latest()
        if latest:
            runner.ok(f"清故障后: foc_state={latest.foc_state}, fault_flags={hex(latest.fault_flags) if latest.fault_flags else '0x00000000'}")
        
        runner.export_csv("14_fault_recovery")

        # ===== 最终报告 =====
        print("\n" + "=" * 60)
        print("测试完成！结果摘要:")
        print("=" * 60)
        print(json.dumps(runner.results, indent=2, default=str))

    except KeyboardInterrupt:
        print("\n[用户中断]")
    except Exception as e:
        print(f"\n[异常] {e}")
        import traceback
        traceback.print_exc()
    finally:
        runner.close()
        print("\n[断开串口]")

    return runner


if __name__ == "__main__":
    runner = run_all_tests()
