import unittest
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent


class TestBuildSystemConsistency(unittest.TestCase):
    def test_tle5012_uses_pa15_as_software_nss(self):
        main_h = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
        gpio_c = (ROOT / "Core" / "Src" / "gpio.c").read_text(encoding="utf-8")
        tle_h = (ROOT / "MDK-ARM" / "code" / "tle5012.h").read_text(encoding="utf-8")
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("TLE5012_NSS_Pin", main_h)
        self.assertIn("TLE5012_NSS_GPIO_Port", main_h)
        self.assertIn("HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);", gpio_c)
        self.assertIn("#define TLE5012_CS_PORT      TLE5012_NSS_GPIO_Port", tle_h)
        self.assertIn("#define TLE5012_CS_PIN       TLE5012_NSS_Pin", tle_h)
        self.assertIn("static void TLE5012_AssertCS(void)", tle_c)
        self.assertIn("static void TLE5012_ReleaseCS(void)", tle_c)
        self.assertIn("HAL_GPIO_WritePin(TLE5012_CS_PORT, TLE5012_CS_PIN, GPIO_PIN_RESET);", tle_c)
        self.assertIn("HAL_GPIO_WritePin(TLE5012_CS_PORT, TLE5012_CS_PIN, GPIO_PIN_SET);", tle_c)
        self.assertIn("TLE5012_ReleaseCS();", tle_c)
        self.assertIn("TLE5012_AssertCS();", tle_c)
        self.assertIn("void TLE5012_HandleTransferError(void);", tle_h)
        self.assertIn("void TLE5012_HandleTransferError(void)", tle_c)
        self.assertIn("TLE5012_HandleTransferError();", it_c)

    def test_debug_boot_path_uses_hse_and_gates_fdcan_init(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        fdcan_h = (ROOT / "Core" / "Inc" / "fdcan.h").read_text(encoding="utf-8")
        fdcan_c = (ROOT / "Core" / "Src" / "fdcan.c").read_text(encoding="utf-8")
        ioc = (ROOT / "24V FOC Controller.ioc").read_text(encoding="utf-8")

        self.assertIn("RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;", main_c)
        self.assertIn("RCC_OscInitStruct.HSEState = RCC_HSE_ON;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 5;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 192;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;", main_c)
        self.assertIn("RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSE", ioc)
        self.assertIn("RCC.DIVM1=5", ioc)
        self.assertIn("RCC.DIVN1=192", ioc)
        self.assertIn("#define FOC_DEBUG_DISABLE_FDCAN_INIT 1U", fdcan_h)
        self.assertIn("#if FOC_DEBUG_DISABLE_FDCAN_INIT", fdcan_c)
        self.assertIn("return;", fdcan_c)

    def test_adc_low_side_sampling_timing_contract(self):
        adc_c = (ROOT / "Core" / "Src" / "adc.c").read_text(encoding="utf-8")
        tim_c = (ROOT / "Core" / "Src" / "tim.c").read_text(encoding="utf-8")
        dma_c = (ROOT / "Core" / "Src" / "dma.c").read_text(encoding="utf-8")
        msp_c = (ROOT / "Core" / "Src" / "stm32h7xx_hal_msp.c").read_text(encoding="utf-8")

        self.assertIn("hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;", adc_c)
        self.assertIn("sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;", adc_c)
        self.assertIn("sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;", adc_c)
        self.assertIn("sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;", tim_c)
        self.assertIn("HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)", tim_c)
        self.assertIn("HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);", msp_c)
        self.assertIn("HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);", dma_c)
        self.assertIn("HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);", tim_c)
        self.assertIn("HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);", dma_c)
        self.assertIn("HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);", dma_c)
        self.assertIn("HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 0);", dma_c)

    def test_adc_sampling_tracks_frame_freshness_by_control_cycle(self):
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")
        adc_c = (ROOT / "MDK-ARM" / "code" / "adc_sampling.c").read_text(encoding="utf-8")

        self.assertIn("volatile uint32_t frameSequence;", adc_h)
        self.assertIn("volatile uint32_t lastCommittedCycle;", adc_h)
        self.assertIn("volatile uint32_t frameAgeCycles;", adc_h)
        self.assertIn("volatile uint32_t sampleMissCount;", adc_h)
        self.assertIn("volatile uint32_t invalidWindowCount;", adc_h)
        self.assertIn("void ADC_Sampling_BeginControlCycle(void);", adc_h)
        self.assertIn("void ADC_Sampling_EndControlCycle(void);", adc_h)
        self.assertIn("uint8_t ADC_Sampling_TryConsumeLatest(void);", adc_h)
        self.assertIn("static volatile uint8_t s_controlWindowOpen = 1U;", adc_c)
        self.assertIn("void ADC_Sampling_BeginControlCycle(void)", adc_c)
        self.assertIn("void ADC_Sampling_EndControlCycle(void)", adc_c)
        self.assertIn("uint8_t ADC_Sampling_TryConsumeLatest(void)", adc_c)
        self.assertIn("void ADC_Sampling_ResetTimingState(void);", adc_h)
        self.assertIn("void ADC_Sampling_ResetTimingState(void)", adc_c)
        self.assertIn("s_adcData.sampleMissCount = 0U;", adc_c)

    def test_foc_loop_faults_after_repeated_adc_sample_misses(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("FOC_FAULT_ADC_SAMPLING", foc_h)
        self.assertIn("#define FOC_ADC_SAMPLE_MISS_FAULT_THRESHOLD", foc_h)
        self.assertIn("ADC_Sampling_BeginControlCycle();", foc_c)
        self.assertIn("ADC_Sampling_TryConsumeLatest()", foc_c)
        self.assertIn("FOC_App_RequestDisableFromISR(handle, FOC_FAULT_ADC_SAMPLING);", foc_c)
        self.assertIn("ADC_Sampling_EndControlCycle();", foc_c)
        self.assertIn("ADC_Sampling_ResetTimingState();", foc_c)

    def test_uart_upload_includes_adc_sampling_diagnostics(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("adcTriggerSource", uart_h)
        self.assertIn("adcCurrentSampleTimeCycles", uart_h)
        self.assertIn("adcVbusSampleTimeCycles", uart_h)
        self.assertIn("adcFrameSequence", uart_h)
        self.assertIn("adcFrameAgeCycles", uart_h)
        self.assertIn("adcSampleMissCount", uart_h)
        self.assertIn("adcInvalidWindowCount", uart_h)
        self.assertIn("adcRawCurrentA", uart_h)
        self.assertIn("adcRawCurrentB", uart_h)
        self.assertIn("adcRawCurrentC", uart_h)
        self.assertIn("adcCurrentA", uart_h)
        self.assertIn("adcCurrentB", uart_h)
        self.assertIn("adcCurrentC", uart_h)
        self.assertIn("adcVbus", uart_h)
        self.assertIn("ADC_SAMPLING_TRIGGER_SOURCE_TEXT", uart_c)
        self.assertIn("[ADC Sampling]", uart_c)
        self.assertIn("packet->adcFrameSequence", uart_c)
        self.assertIn("packet->adcRawCurrentA", uart_c)
        self.assertIn("packet->adcCurrentA", uart_c)

    def test_build_ps1_wrapper_tolerates_native_stderr_with_zero_exit(self):
        script = (ROOT / "build.ps1").read_text(encoding="utf-8")
        match = re.search(
            r"function Invoke-AndCheck\(\[string\]\$exe, \[string\[\]\]\$toolArgs\)\s*\{[\s\S]*?^}",
            script,
            re.MULTILINE,
        )
        self.assertIsNotNone(match, "build.ps1 should define Invoke-AndCheck")

        harness = "\n".join(
            [
                '$ErrorActionPreference = "Stop"',
                match.group(0),
                '$result = Invoke-AndCheck "cmd.exe" @("/c", "echo native-warning 1>&2 & exit /b 0")',
                'Write-Output "OK=$($result.Ok)"',
            ]
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            script_path = Path(temp_dir) / "invoke_and_check_harness.ps1"
            script_path.write_text(harness, encoding="utf-8")
            run = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(script_path),
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )

        self.assertEqual(
            run.returncode,
            0,
            "Invoke-AndCheck should not abort when a native command writes to stderr but exits 0",
        )
        self.assertIn("OK=True", run.stdout)

    def test_host_gui_packaging_wrapper_handles_missing_native_error_variable(self):
        script = (ROOT / "build_host_gui_app.ps1").read_text(encoding="utf-8")
        match = re.search(
            r"function Invoke-AndCheck\(\[string\]\$exe, \[string\[\]\]\$toolArgs\)\s*\{[\s\S]*?^}",
            script,
            re.MULTILINE,
        )
        self.assertIsNotNone(match, "build_host_gui_app.ps1 should define Invoke-AndCheck")

        harness = "\n".join(
            [
                '$ErrorActionPreference = "Stop"',
                "Set-StrictMode -Version Latest",
                "Remove-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue",
                match.group(0),
                '$result = Invoke-AndCheck "cmd.exe" @("/c", "echo packaged-warning 1>&2 & exit /b 0")',
                'Write-Output "OK=$($result.Ok)"',
            ]
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            script_path = Path(temp_dir) / "build_host_gui_invoke_wrapper_harness.ps1"
            script_path.write_text(harness, encoding="utf-8")
            run = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(script_path),
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )

        self.assertEqual(
            run.returncode,
            0,
            "Host GUI Invoke-AndCheck should tolerate shells where PSNativeCommandUseErrorActionPreference is undefined",
        )
        self.assertIn("OK=True", run.stdout)

    def test_local_demo_buttons_gpio_and_unlock_contract(self):
        main_h = (ROOT / "Core" / "Inc" / "main.h").read_text(encoding="utf-8")
        gpio_c = (ROOT / "Core" / "Src" / "gpio.c").read_text(encoding="utf-8")
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        demo_module = ROOT / "MDK-ARM" / "code" / "demo_button_control.c"

        self.assertIn("MOD1_Pin", main_h)
        self.assertIn("MOD1_GPIO_Port", main_h)
        self.assertIn("MOD2_Pin", main_h)
        self.assertIn("MOD2_GPIO_Port", main_h)
        self.assertIn("GPIO_PIN_12|GPIO_PIN_13", gpio_c)
        self.assertIn("GPIO_MODE_INPUT", gpio_c)
        self.assertTrue(demo_module.exists())
        self.assertIn("power_unlocked", foc_h)
        self.assertIn('CMD:UNLOCK,%ld', it_c)

    def test_local_demo_button_mode_behavior_contract(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        demo_c = (ROOT / "MDK-ARM" / "code" / "demo_button_control.c").read_text(encoding="utf-8")

        self.assertIn("FOC_MODE_SPEED", demo_c)
        self.assertIn("FOC_MODE_TORQUE", demo_c)
        self.assertIn("FOC_App_Enable", demo_c)
        self.assertIn("FOC_App_Disable", demo_c)
        self.assertIn("FOC_App_StartIdentify", demo_c)
        self.assertIn("FOC_App_StopIdentify", demo_c)
        self.assertIn("FOC_App_ResetMotionState", demo_c)
        self.assertRegex(demo_c, r"0\.1745[0-9]*f")
        self.assertRegex(demo_c, r"2\.094[0-9]*f")
        self.assertIn("theta_mech", demo_c)
        self.assertIn("DemoButtonControl_Init(&g_foc_app);", main_c)
        self.assertIn("DemoButtonControl_Service();", main_c)

    def test_local_demo_zero_spring_contract(self):
        demo_c = (ROOT / "MDK-ARM" / "code" / "demo_button_control.c").read_text(encoding="utf-8")

        self.assertIn("FOC_PI", demo_c)
        self.assertRegex(demo_c, r"1\.5f")
        self.assertRegex(demo_c, r"2\.09439[0-9]*f")
        self.assertIn("FOC_App_SetCurrentRef", demo_c)

    def test_zero_spring_waits_for_fresh_angle_sample_after_enable(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        demo_c = (ROOT / "MDK-ARM" / "code" / "demo_button_control.c").read_text(encoding="utf-8")

        self.assertIn("theta_sample_seq", foc_h)
        self.assertIn("handle->theta_sample_seq++;", foc_c)
        self.assertIn("s_zero_spring_wait_active", demo_c)
        self.assertIn("s_zero_spring_wait_seq", demo_c)
        self.assertIn("s_app->theta_sample_seq", demo_c)
        self.assertIn("s_app->theta_sample_seq == s_zero_spring_wait_seq", demo_c)

    def test_zero_spring_uses_identify_aligned_mechanical_zero(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        demo_c = (ROOT / "MDK-ARM" / "code" / "demo_button_control.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")

        self.assertIn("theta_mech_zero", motor_h)
        self.assertIn("handle->param->theta_mech_zero = theta_mech;", motor_c)
        self.assertIn("s_app->motor_param.theta_mech_zero", demo_c)
        self.assertIn("s_app->motor_param.theta_mech_zero - s_app->theta_mech", demo_c)
        self.assertIn("识别/对齐", readme)

    def test_host_gui_docs_and_entry_exist(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        architecture = (ROOT / "Project_Architecture.md").read_text(encoding="utf-8")
        gui_entry = ROOT / "HostComputer" / "gui_app.py"
        gui_logic = ROOT / "HostComputer" / "gui_logic.py"
        main_window = ROOT / "HostComputer" / "main_window.py"

        self.assertTrue(gui_entry.exists())
        self.assertTrue(gui_logic.exists())
        self.assertTrue(main_window.exists())
        self.assertIn("python -m HostComputer.gui_app", readme)
        self.assertIn("HostComputer/main_window.py", readme)
        self.assertIn("Advanced Control", readme)
        self.assertIn("PI Parameters", readme)
        self.assertIn("Identify", readme)
        self.assertIn("HostMainWindow", architecture)
        self.assertIn("gui_logic.py", architecture)
        self.assertIn("Debug Panel", architecture)

    def test_host_gui_packaging_contract(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        package_script = ROOT / "build_host_gui_app.ps1"
        spec_file = ROOT / "HostComputer" / "host_gui_app.spec"

        self.assertTrue(package_script.exists())
        self.assertTrue(spec_file.exists())

        script = package_script.read_text(encoding="utf-8")
        spec = spec_file.read_text(encoding="utf-8")

        self.assertIn("HostComputer/requirements.txt", script)
        self.assertIn("PyInstaller", script)
        self.assertIn("HostComputer/host_gui_app.spec", script)
        self.assertIn("24V_FOC_Host", script)

        self.assertIn("name='24V_FOC_Host'", spec)
        self.assertIn("console=False", spec)
        self.assertIn("pyqtgraph", spec)
        self.assertIn("PROJECT_ROOT = Path(SPECPATH).parent", spec)
        self.assertIn("host_gui_launcher.py", spec)

        self.assertIn("build_host_gui_app.ps1", readme)
        self.assertIn("24V_FOC_Host.exe", readme)
        self.assertIn("PyInstaller", readme)


if __name__ == "__main__":
    unittest.main()
