import unittest
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent


class TestBuildSystemConsistency(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
