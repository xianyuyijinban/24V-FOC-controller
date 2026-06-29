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

    def test_tle5012_uses_three_wire_staged_transfer_with_dummy_clocked_response(self):
        spi_c = (ROOT / "Core" / "Src" / "spi.c").read_text(encoding="utf-8")
        tle_h = (ROOT / "MDK-ARM" / "code" / "tle5012.h").read_text(encoding="utf-8")
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("hspi3.Init.Direction = SPI_DIRECTION_2LINES;", spi_c)
        self.assertIn("void TLE5012_HandleTxComplete(void);", tle_h)
        self.assertIn("static uint16_t tle5012_rx_dummy_buf[2];", tle_c)
        self.assertIn("HAL_SPI_Transmit_DMA(&hspi3,", tle_c)
        self.assertNotIn("HAL_SPI_Receive_DMA(&hspi3,", tle_c)
        self.assertIn("HAL_SPI_TransmitReceive_DMA(&hspi3,", tle_c)
        self.assertNotIn("SPI_1LINE_TX(&hspi3);", tle_c)
        self.assertNotIn("SPI_1LINE_RX(&hspi3);", tle_c)
        self.assertIn("static void TLE5012_ConfigCommandPhasePins(void)", tle_c)
        self.assertIn("static void TLE5012_ConfigResponsePhasePins(void)", tle_c)
        self.assertIn("GPIOC->MODER", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_INPUT", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_AF", tle_c)
        self.assertNotIn("HAL_GPIO_Init(GPIOC, &gpio_init);", tle_c)
        self.assertNotIn("GPIO_InitTypeDef gpio_init", tle_c)
        self.assertIn("TLE5012_ConfigCommandPhasePins();", tle_c)
        self.assertIn("TLE5012_ConfigResponsePhasePins();", tle_c)
        self.assertIn("TLE5012_GPIO_PUPDR_PULLUP", tle_c)
        self.assertIn("TLE5012_ConfigDataPullups();", tle_c)
        self.assertIn("GPIOC->PUPDR", tle_c)
        self.assertIn("static void TLE5012_TwrDelay", tle_c)
        self.assertIn("crc_words[0] = TLE5012_READ_CMD;", tle_c)
        self.assertIn("crc_words[1] = raw_data;", tle_c)
        self.assertIn("TLE5012_CalculateCRC8(crc_words, 2U);", tle_c)
        self.assertIn("void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)", it_c)
        self.assertIn("TLE5012_HandleTxComplete();", it_c)
        self.assertIn("if (hspi == &hspi3) {", it_c)
        self.assertIn("TLE5012_ProcessData(tle5012_rx_buf);", it_c)

    def test_tle5012_encoder_board_uses_shared_pc11_pc12_data_net(self):
        tle_h = (ROOT / "MDK-ARM" / "code" / "tle5012.h").read_text(encoding="utf-8")
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        architecture = (ROOT / "Project_Architecture.md").read_text(encoding="utf-8")

        self.assertIn("Netlist_Schematic2_2026-04-24", tle_c)
        self.assertIn("CN2.5/CN2.6 share the encoder DATA net", tle_h)
        self.assertIn("#define TLE5012_DATA_RX_PIN            11U", tle_c)
        self.assertIn("#define TLE5012_DATA_TX_PIN            12U", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_RX_PIN)", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_AF(TLE5012_DATA_TX_PIN)", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_AF(TLE5012_DATA_RX_PIN)", tle_c)
        self.assertIn("TLE5012_GPIO_MODER_INPUT(TLE5012_DATA_TX_PIN)", tle_c)
        self.assertIn("| SPI3_MISO / DATA | PC11 | TLE5012 shared DATA receive phase", readme)
        self.assertIn("| SPI3_MOSI / DATA | PC12 | TLE5012 shared DATA command phase", readme)
        self.assertIn("CN2.5/CN2.6", architecture)

    def test_tle5012_gpio_diag_mode_takes_over_shared_data_lines_safely(self):
        tle_h = (ROOT / "MDK-ARM" / "code" / "tle5012.h").read_text(encoding="utf-8")
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("typedef struct {\n    uint8_t active;", tle_h)
        self.assertIn("extern volatile TLE5012_GpioDiagState_t tle5012_gpio_diag;", tle_h)
        self.assertIn("uint16_t raw_word;", tle_h)
        self.assertIn("uint16_t safety_word;", tle_h)
        self.assertIn("uint8_t received_crc;", tle_h)
        self.assertIn("uint8_t calculated_crc;", tle_h)
        self.assertIn("uint8_t data_ok;", tle_h)
        self.assertIn("tle5012_sensor.raw_word = raw_data;", tle_c)
        self.assertIn("tle5012_sensor.safety_word = safety_word;", tle_c)
        self.assertIn("tle5012_sensor.received_crc = received_crc;", tle_c)
        self.assertIn("tle5012_sensor.calculated_crc = calculated_crc;", tle_c)
        self.assertIn("tle5012_sensor.data_ok = safety_ok;", tle_c)
        self.assertIn("void TLE5012_GpioDiagStart(void);", tle_h)
        self.assertIn("void TLE5012_GpioDiagStop(void);", tle_h)
        self.assertIn("void TLE5012_GpioDiagService(void);", tle_h)
        self.assertIn("uint8_t TLE5012_IsGpioDiagActive(void);", tle_h)
        self.assertIn("static void TLE5012_ConfigGpioDiagPins(void)", tle_c)
        self.assertIn("(void)HAL_SPI_DMAStop(&hspi3);", tle_c)
        self.assertIn("if (TLE5012_IsGpioDiagActive()) {\n        return;\n    }", tle_c)
        self.assertIn("GPIOC->MODER", tle_c)
        self.assertIn("GPIOA->BSRR", tle_c)
        self.assertIn("GPIOC->BSRR", tle_c)
        self.assertIn("tle5012_gpio_diag.data_in", tle_c)
        self.assertIn("TLE5012_GpioDiagService();", it_c)
        self.assertIn('sscanf(cmd, "CMD:TLE_GPIO_DIAG,%ld"', it_c)
        self.assertIn("TLE5012_GpioDiagStart();", it_c)
        self.assertIn("TLE5012_GpioDiagStop();", it_c)
        self.assertIn('strcmp(cmd, "CMD:TLE_RAW") == 0', it_c)
        self.assertIn("char response[256];", it_c)
        self.assertIn('"TLE_RAW,raw=0x%04X,safety=0x%04X', it_c)
        self.assertIn("angle_cdeg", it_c)
        self.assertIn("angle=%lu.%02lu", it_c)
        tle_raw_block = re.search(
            r'if \(strcmp\(cmd, "CMD:TLE_RAW"\) == 0\) \{(.*?)UART_CommandSendText\(response\);',
            it_c,
            re.S,
        )
        self.assertIsNotNone(tle_raw_block)
        self.assertNotIn("%.2f", tle_raw_block.group(1))

    def test_tle5012_data_validity_does_not_treat_sticky_reset_or_system_bits_as_offline(self):
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")

        self.assertIn("#define TLE5012_SAFETY_DATA_VALID_MASK", tle_c)
        self.assertIn(
            "TLE5012_SAFETY_INTERFACE_OK_MASK | TLE5012_SAFETY_ANGLE_OK_MASK",
            tle_c,
        )
        self.assertIn(
            "(safety_word & TLE5012_SAFETY_DATA_VALID_MASK) == TLE5012_SAFETY_DATA_VALID_MASK",
            tle_c,
        )
        safety_block = re.search(
            r"safety_ok\s*=\s*\((.*?)\)\s*\?\s*1U\s*:\s*0U;",
            tle_c,
            re.S,
        )
        self.assertIsNotNone(safety_block)
        self.assertNotIn("TLE5012_SAFETY_RESET_OK_MASK", safety_block.group(1))
        self.assertNotIn("TLE5012_SAFETY_SYSTEM_OK_MASK", safety_block.group(1))

    def test_encoder_fault_requires_threshold_and_auto_recovers_when_online(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("FOC_ENCODER_FAULT_MISS_THRESHOLD", foc_h)
        self.assertIn("static uint8_t FOC_App_IsEncoderFaultActive(void)", foc_c)
        self.assertIn("TLE5012_GetCRCErrorCount() >= FOC_ENCODER_FAULT_MISS_THRESHOLD", foc_c)
        self.assertIn("FOC_App_IsEncoderFaultActive()", foc_c)
        self.assertIn("handle->fault_code == FOC_FAULT_ENCODER", foc_c)
        self.assertIn("TLE5012_IsDataValid()", foc_c)
        self.assertIn("handle->fault_code = FOC_FAULT_NONE;", foc_c)
        self.assertNotIn("require_encoder && !TLE5012_IsDataValid()", foc_c)
        self.assertIn("FOC_App_IsEncoderReadyForPowerStage(fault)", foc_c)
        self.assertIn("if (fault != FOC_FAULT_NONE)", foc_c)

    def test_adc_current_polarity_inverts_low_side_frontend_for_stable_q_feedback(self):
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")

        self.assertIn("#define ADC_CURRENT_POLARITY (-1.0f)", adc_h)
        self.assertIn("return ADC_CURRENT_POLARITY * ((float)adc_centered) * ADC_VREF / ADC_MAX * K_CURRENT;", adc_h)

    def test_motion_reset_clears_stale_foc_current_and_svpwm_state(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        reset_match = re.search(
            r"void\s+FOC_App_ResetMotionState\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_SetCurrentRef",
            foc_c,
        )
        self.assertIsNotNone(reset_match)
        reset_body = reset_match.group(1)

        for token in (
            "handle->foc.Iabc.a = 0.0f;",
            "handle->foc.Iabc.b = 0.0f;",
            "handle->foc.Iabc.c = 0.0f;",
            "handle->foc.IalphaBeta.alpha = 0.0f;",
            "handle->foc.IalphaBeta.beta = 0.0f;",
            "handle->foc.Idq.d = 0.0f;",
            "handle->foc.Idq.q = 0.0f;",
            "FOC_SVPWM_Generate(&handle->foc.ValphaBeta, handle->foc.Vbus, &handle->foc.svpwm);",
        ):
            self.assertIn(token, reset_body)

    def test_refresh_telemetry_preserves_reconstructed_loop_current_while_running(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        refresh_match = re.search(
            r"void\s+FOC_App_RefreshTelemetry\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_ResetMotionState",
            foc_c,
        )
        self.assertIsNotNone(refresh_match)
        refresh_body = refresh_match.group(1)

        raw_current_assign = refresh_body.find("handle->Ia = adc->currentA;")
        running_guard = refresh_body.find(
            "if ((handle->state != FOC_STATE_RUNNING) &&\n"
            "        (handle->state != FOC_STATE_PARAM_IDENTIFY))"
        )
        self.assertNotEqual(raw_current_assign, -1)
        self.assertNotEqual(
            running_guard,
            -1,
            "Main-loop telemetry may refresh raw ADC current only outside ISR-owned running states.",
        )
        self.assertLess(running_guard, raw_current_assign)

    def test_fault_upload_includes_motor_identification_error_detail(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("uint8_t  identifyState;", uart_h)
        self.assertIn("uint8_t  identifyError;", uart_h)
        self.assertIn("float    identifyRsCurrentTarget;", uart_h)
        self.assertIn("float    identifyRsLastVavg;", uart_h)
        self.assertIn("float    identifyRsLastIavg;", uart_h)
        self.assertIn("float    identifyRsLastVecRs;", uart_h)
        self.assertIn("packet->identifyState = (uint8_t)g_foc_app.mi_handle.state;", uart_c)
        self.assertIn("packet->identifyError = (uint8_t)g_foc_app.mi_handle.error_code;", uart_c)
        self.assertIn("packet->identifyRsCurrentTarget = g_foc_app.mi_handle.rs_current_target;", uart_c)
        self.assertIn("packet->identifyRsLastVecRs = g_foc_app.mi_handle.rs_last_vec_rs;", uart_c)
        self.assertIn("packet->identifyRsPositive = g_foc_app.mi_handle.Rs_positive;", uart_c)
        self.assertIn("packet->identifyRsNegative = g_foc_app.mi_handle.Rs_negative;", uart_c)
        self.assertIn("RsDiag: Vd_avg=%s V | Id_avg=%s A", uart_c)
        self.assertIn("Rs_vec=%s Ohm", uart_c)
        self.assertIn("Rs+=%s Ohm | Rs-=%s Ohm", uart_c)
        self.assertIn("ParamDiag: invalid=0x%08lX", uart_c)
        self.assertIn("packet->paramInvalidFlags = Param_GetInvalidFlags(&g_foc_app.motor_param);", uart_c)
        self.assertIn("PARAM_INVALID_RS", uart_c)
        self.assertIn("PARAM_RS_MAX_OHM", uart_c)
        self.assertIn("PARAM_INVALID_LD", uart_c)
        self.assertIn("PARAM_INVALID_LQ", uart_c)
        self.assertIn("DrvUart_IdentifyErrorToString", uart_c)
        self.assertIn("[Motor Identification]", uart_c)
        self.assertIn("packet->focState == (uint8_t)FOC_STATE_PARAM_IDENTIFY", uart_c)
        self.assertIn("MI_ERR_CURRENT_TOO_LOW", uart_c)
        self.assertIn(r"\350\257\206\345\210\253\347\224\265\346\265\201\350\277\207\344\275\216", uart_c)
        self.assertIn("uint16_t encoderRawWord;", uart_h)
        self.assertIn("uint16_t encoderSafetyWord;", uart_h)
        self.assertIn("uint8_t  encoderReceivedCrc;", uart_h)
        self.assertIn("uint8_t  encoderCalculatedCrc;", uart_h)
        self.assertIn("uint8_t  encoderCrcErrorCount;", uart_h)
        self.assertIn("uint8_t  encoderGpioDiagActive;", uart_h)
        self.assertIn("packet->encoderRawWord = tle5012_sensor.raw_word;", uart_c)
        self.assertIn("packet->encoderSafetyWord = tle5012_sensor.safety_word;", uart_c)
        self.assertIn("packet->encoderReceivedCrc = tle5012_sensor.received_crc;", uart_c)
        self.assertIn("packet->encoderCalculatedCrc = tle5012_sensor.calculated_crc;", uart_c)
        self.assertIn("packet->encoderCrcErrorCount = TLE5012_GetCRCErrorCount();", uart_c)
        self.assertIn("packet->encoderGpioDiagActive = TLE5012_IsGpioDiagActive();", uart_c)
        self.assertIn("RawWord:  0x%04X | SafetyWord: 0x%04X", uart_c)
        self.assertIn("CRC Rx/Calc: 0x%02X / 0x%02X | BadFrames: %u | GpioDiag: %s", uart_c)
        self.assertIn("float    thetaMech;", uart_h)
        self.assertIn("float    thetaElec;", uart_h)
        self.assertIn("float    paramThetaOffset;", uart_h)
        self.assertIn("float    paramThetaMechZero;", uart_h)
        self.assertIn("packet->thetaMech = g_foc_app.theta_mech;", uart_c)
        self.assertIn("packet->thetaElec = g_foc_app.theta_elec;", uart_c)
        self.assertIn("packet->paramThetaOffset = g_foc_app.motor_param.theta_offset;", uart_c)
        self.assertIn("packet->paramThetaMechZero = g_foc_app.motor_param.theta_mech_zero;", uart_c)
        self.assertIn("ThetaDiag: mech=", uart_c)
        self.assertIn("CurrentDQ: Id=", uart_c)
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn('strcmp(cmd, "CMD:FAULT_DETAIL") == 0', it_c)
        self.assertNotIn("FAULT_DETAIL,NO_ACTIVE_FAULT", it_c)
        self.assertIn("DrvUart_UploadImmediate();", it_c)
        self.assertIn("CMD:FAULT_DETAIL", readme)

    def test_rs_identification_uses_d_axis_locked_rotor_method(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        # New d-axis locked-rotor macros
        self.assertRegex(motor_h, r"#define\s+MI_RS_LOCK_CURRENT_INITIAL\s+0\.15f\b")
        self.assertRegex(motor_h, r"#define\s+MI_RS_LOCK_CURRENT_STEP\s+0\.1f\b")
        self.assertRegex(motor_h, r"#define\s+MI_RS_LOCK_CURRENT_MAX\s+1\.0f\b")
        self.assertRegex(motor_h, r"#define\s+MI_RS_LOCK_DURATION\s+150\b")
        self.assertRegex(motor_h, r"#define\s+MI_RS_SAMPLE_DURATION\s+50\b")
        # Updated safety macros
        self.assertRegex(motor_h, r"#define\s+MI_RS_CURRENT_THRESH\s+0\.15f\b")
        self.assertRegex(motor_h, r"#define\s+MI_RS_CURRENT_MAX\s+1\.2f\b")
        # Old open-loop macros must be removed
        self.assertNotIn("MI_RS_OPENLOOP_VOLTAGE_INIT", motor_h)
        self.assertNotIn("MI_RS_OPENLOOP_VOLTAGE_STEP", motor_h)
        self.assertNotIn("MI_RS_OPENLOOP_VOLTAGE_MAX", motor_h)
        self.assertNotIn("MI_RS_TEST_DURATION", motor_h)
        self.assertNotIn("MI_RS_SETTLE_TIME", motor_h)
        self.assertNotIn("MI_RS_VDQ_MAX_V", motor_h)
        self.assertNotIn("MI_RS_CURRENT_RAMP_A_PER_MS", motor_h)
        self.assertNotIn("MI_RS_PREALIGN_CURRENT", motor_h)
        # Old handle fields must be removed
        self.assertNotIn("rs_voltage_target", motor_h)
        self.assertNotIn("rs_valid_samples", motor_h)
        self.assertNotIn("rs_low_side_valid_count", motor_h)
        self.assertNotIn("rs_current_applied", motor_h)
        self.assertNotIn("rs_last_vq_avg", motor_h)
        self.assertNotIn("rs_last_vi_avg", motor_h)
        self.assertNotIn("rs_last_ii_avg", motor_h)
        self.assertNotIn("rs_last_ia", motor_h)
        self.assertNotIn("rs_last_ib", motor_h)
        self.assertNotIn("rs_last_ic", motor_h)
        self.assertNotIn("rs_last_ialpha", motor_h)
        self.assertNotIn("rs_last_ibeta", motor_h)
        # Kept handle fields
        self.assertIn("rs_current_target", motor_h)
        self.assertIn("rs_last_v_avg", motor_h)
        self.assertIn("rs_last_i_avg", motor_h)
        self.assertIn("rs_last_i_mag_avg", motor_h)
        self.assertIn("rs_last_vec_rs", motor_h)
        self.assertIn("rs_last_samples", motor_h)
        # New method uses closed-loop d-axis current injection via FOC
        self.assertIn("FOC_SetAngle(handle->foc, 0.0f);", motor_c)
        self.assertIn("FOC_SetCurrentReference(handle->foc,", motor_c)
        # Uses Vdq.d (PI controller output) and Idq.d (measured) for Rs calculation
        self.assertIn("handle->foc->Vdq.d", motor_c)
        self.assertIn("handle->foc->Idq.d", motor_c)
        self.assertIn("fabsf(Vd_avg / Id_avg)", motor_c)
        # Timing: lock duration + sample duration
        self.assertIn("MI_RS_LOCK_DURATION + MI_RS_SAMPLE_DURATION", motor_c)
        self.assertIn("MI_RS_LOCK_DURATION", motor_c)
        # Polarity reversal for offset cancellation
        self.assertIn("handle->Rs_positive = Rs_val;", motor_c)
        self.assertIn("handle->Rs_negative = Rs_val;", motor_c)
        self.assertIn("handle->polarity", motor_c)
        # Current-based retry (not voltage-based)
        self.assertIn("MI_RS_LOCK_CURRENT_STEP", motor_c)
        self.assertIn("MI_RS_LOCK_CURRENT_MAX", motor_c)
        # Converged check
        self.assertIn("MI_RsConverged", motor_c)
        self.assertIn("MI_RsUseSinglePolarityFallback", motor_c)
        # foc_app.c removes RS from identify_direct_svpwm
        identify_direct_block = foc_c.find("identify_direct_svpwm = 1U;")
        rs_in_direct_block = foc_c.find("MI_STATE_RS_IDENTIFY", identify_direct_block, identify_direct_block + 300)
        self.assertEqual(rs_in_direct_block, -1,
            "RS must NOT appear in the identify_direct_svpwm condition in foc_app.c")
        # RS now goes through FOC_Run (not direct SVPWM)
        self.assertIn("FOC_Run(&handle->foc);", foc_c)
        # Old open-loop SVPWM pattern should not appear in RS path
        self.assertNotIn("MI_RS_OPENLOOP_VOLTAGE", motor_c)
        self.assertNotIn("rs_voltage_target", motor_c)

    def test_pn_verification_uses_configured_pole_pairs_and_only_detects_direction(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("MI_PN_ALIGN_CURRENT", motor_h)
        self.assertIn("MI_PN_ALIGN_DURATION", motor_h)
        self.assertIn("MI_PN_TEST_CURRENT_INITIAL", motor_h)
        self.assertIn("MI_PN_TEST_CURRENT_STEP", motor_h)
        self.assertIn("MI_PN_TEST_CURRENT_MAX", motor_h)
        self.assertIn("pn_current_target", motor_h)
        self.assertIn("pn_last_delta_mech", motor_h)
        self.assertIn("pn_last_delta_elec", motor_h)
        self.assertIn("pn_last_calc", motor_h)
        self.assertIn("MI_PN_MIN_EXPECTED_TRAVEL_RATIO", motor_h)
        self.assertIn("MI_PN_MIN_MECH_DELTA_RAD", motor_h)
        self.assertIn("MI_PnRetryWithHigherCurrent", motor_c)
        self.assertIn("FOC_SetCurrentReference(handle->foc, MI_PN_ALIGN_CURRENT, 0.0f);", motor_c)
        self.assertIn("current_target = MI_GetPnTestCurrent(handle);", motor_c)
        self.assertIn("FOC_SetCurrentReference(handle->foc, current_target, 0.0f);", motor_c)
        self.assertNotIn("FOC_SetCurrentReference(handle->foc, 0.0f, current_target);", motor_c)
        self.assertIn("handle->pn_current_target + MI_PN_TEST_CURRENT_STEP", motor_c)
        self.assertIn("configured_pn = handle->param->Pn;", motor_c)
        self.assertIn("expected_delta = fabsf(delta_elec) / (float)configured_pn;", motor_c)
        self.assertIn("MI_PN_MIN_EXPECTED_TRAVEL_RATIO", motor_c)
        self.assertIn("handle->pn_last_calc = fabsf(delta_elec / delta_mech);", motor_c)
        self.assertNotIn("handle->param->Pn = (uint8_t)(pn_calc + 0.5f);", motor_c)
        self.assertRegex(
            motor_c,
            r"handle->param->encoder_dir\s*=\s*\(\(delta_elec \* delta_mech\) >= 0\.0f\) \? 1 : -1;"
            r"[\s\S]*?return MI_ERR_NONE;"
            r"[\s\S]*?if\s*\(\s*MI_PnRetryWithHigherCurrent\(handle\)\s*\)\s*\{"
            r"[\s\S]*?return MI_ERR_IN_PROGRESS;"
            r"[\s\S]*?return MI_ERR_PN_NOT_CONVERGED;",
            "Configured-Pn direction probing should retry before reporting that the motor did not move enough.",
        )
        self.assertIn("void FOC_App_SetPolePairs(FOC_AppHandle_t *handle, uint8_t pole_pairs);", foc_h)
        self.assertIn("void FOC_App_SetPolePairs(FOC_AppHandle_t *handle, uint8_t pole_pairs)", foc_c)
        self.assertIn("handle->motor_param.valid_flag = 0U;", foc_c)
        self.assertIn('sscanf(cmd, "CMD:MOTOR_PN,%ld"', it_c)
        self.assertIn("FOC_App_SetPolePairs(&g_foc_app, (uint8_t)int_arg);", it_c)
        self.assertIn("identifyPnCurrentTarget", uart_h)
        self.assertIn("identifyPnDeltaMech", uart_h)
        self.assertIn("identifyPnDeltaElec", uart_h)
        self.assertIn("identifyPnCalc", uart_h)
        self.assertIn("PnDiag: target=", uart_c)
        self.assertIn("packet->identifyPnCurrentTarget = g_foc_app.mi_handle.pn_current_target;", uart_c)
        self.assertIn('case MI_STATE_PN_IDENTIFY:   return "PN_VERIFY";', uart_c)
        self.assertNotIn('return "PN_IDENTIFY";', uart_c)
        self.assertIn("packet->identifyState = (uint8_t)g_foc_app.mi_handle.state;", uart_c)
        self.assertIn("packet->identifyError = (uint8_t)g_foc_app.mi_handle.error_code;", uart_c)
        self.assertIn("packet->identifyState,\n               packet->identifyError", uart_c)

    def test_pn_verification_defaults_to_bench_friendly_weak_mode(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")

        self.assertRegex(motor_h, r"#define\s+MI_PN_STRICT_VERIFY\s+0\b")
        self.assertIn("if (fabsf(delta_mech) >= min_delta && fabsf(delta_elec) > 0.5f)", motor_c)
        self.assertIn("#if MI_PN_STRICT_VERIFY", motor_c)
        self.assertIn("handle->pn_observed_dir = (delta_mech >= 0.0f) ? 1 : -1;", motor_c)
        self.assertNotIn(
            "handle->pn_observed_dir = (handle->param->encoder_dir >= 0) ? 1 : -1;",
            motor_c,
        )
        self.assertRegex(
            motor_c,
            r"#if MI_PN_STRICT_VERIFY[\s\S]*?return MI_ERR_PN_NOT_CONVERGED;"
            r"[\s\S]*?#else[\s\S]*?return MI_ERR_NONE;[\s\S]*?#endif",
            "PN verify should keep a strict hard-fail path while defaulting to a bench-friendly weak pass path.",
        )

    def test_ls_identification_exposes_runtime_diagnostics_and_fallback_reason(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("ls_last_v_rms", motor_h)
        self.assertIn("ls_last_i_rms", motor_h)
        self.assertIn("ls_last_z", motor_h)
        self.assertIn("ls_last_xl", motor_h)
        self.assertIn("ls_last_l", motor_h)
        self.assertIn("ls_used_fallback", motor_h)
        self.assertIn("identifyLsVrms", uart_h)
        self.assertIn("identifyLsIrms", uart_h)
        self.assertIn("identifyLsZ", uart_h)
        self.assertIn("identifyLsXl", uart_h)
        self.assertIn("identifyLsL", uart_h)
        self.assertIn("identifyLsUsedFallback", uart_h)
        self.assertIn("packet->identifyLsVrms = g_foc_app.mi_handle.ls_last_v_rms;", uart_c)
        self.assertIn("packet->identifyLsUsedFallback = g_foc_app.mi_handle.ls_used_fallback;", uart_c)
        self.assertIn("LsDiag: Vrms=", uart_c)
        self.assertRegex(
            motor_c,
            r"MI_IdentifyLs[\s\S]*?handle->ls_last_v_rms\s*=\s*Vrms;"
            r"[\s\S]*?handle->ls_last_i_rms\s*=\s*Irms;"
            r"[\s\S]*?handle->ls_last_z\s*=\s*Z;"
            r"[\s\S]*?handle->ls_last_xl\s*=\s*XL;"
            r"[\s\S]*?handle->ls_last_l\s*=\s*L;",
            "Ls identification should retain the computed RMS and impedance chain for bench diagnosis.",
        )
        self.assertRegex(
            motor_c,
            r"static\s+MI_ErrorCode_t\s+MI_UseLsFallback\s*\([\s\S]*?handle->ls_used_fallback\s*=\s*1U;"
            r"[\s\S]*?return\s+MI_ERR_NONE;",
            "Ls fallback should mark that default inductance was used so fault detail can distinguish fallback from a true Ls solve.",
        )

    def test_pn_verification_uses_bench_safe_drag_settings(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        pn_start = motor_c.index("MI_ErrorCode_t MI_IdentifyPn(MI_Handle_t *handle)")
        pn_end = motor_c.index("MI_ErrorCode_t MI_IdentifyJ(MI_Handle_t *handle)", pn_start)
        pn_section = motor_c[pn_start:pn_end]

        self.assertRegex(motor_h, r"#define\s+MI_PN_ALIGN_CURRENT\s+0\.1f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_ALIGN_DURATION\s+100\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_TEST_CURRENT_INITIAL\s+0\.25f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_TEST_CURRENT_STEP\s+0\.05f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_TEST_CURRENT_MAX\s+0\.5f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_STEP_ELEC_DEG\s+30\.0f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_NUDGE_ELEC_DEG\s+20\.0f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_NUDGE_SETTLE_MS\s+40\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_STEP_SETTLE_MS\s+120\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_STEP_COUNT\s+18\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_TEST_VOLTAGE_RATIO\s+0\.15f\b")
        self.assertRegex(motor_h, r"#define\s+MI_PN_TEST_VOLTAGE_MAX_V\s+1\.8f\b")
        self.assertNotRegex(motor_h, r"#define\s+MI_PN_ELEC_FREQ_HZ\b")
        self.assertIn("static float MI_ClampAbsVoltage(float voltage, float limit)", motor_c)
        self.assertIn("handle->foc->ValphaBeta.alpha", pn_section)
        self.assertIn("handle->foc->ValphaBeta.beta", pn_section)
        self.assertIn("FOC_SVPWM_Generate(&handle->foc->ValphaBeta, handle->foc->Vbus, &handle->foc->svpwm);", pn_section)
        self.assertIn("voltage_mag = MI_ClampAbsVoltage(voltage_mag, MI_PN_TEST_VOLTAGE_MAX_V);", pn_section)
        self.assertIn("case 2: /* 应用下一步进电角 */", pn_section)
        self.assertIn("case 3: /* 应用反向解卡脉冲 */", pn_section)
        self.assertIn("case 4: /* 等待反向解卡稳定 */", pn_section)
        self.assertIn("case 5: /* 应用正向越峰脉冲 */", pn_section)
        self.assertIn("case 6: /* 等待正向越峰稳定 */", pn_section)
        self.assertIn("case 7: /* 回到主目标步进角 */", pn_section)
        self.assertIn("case 8: /* 等待主步进稳定 */", pn_section)
        self.assertIn("case 9: /* 采样机械角并累计位移 */", pn_section)
        self.assertIn("theta_elec_step = MI_PN_STEP_ELEC_DEG * MI_DEG2RAD", pn_section)
        self.assertIn("theta_elec_nudge = MI_PN_NUDGE_ELEC_DEG * MI_DEG2RAD", pn_section)
        self.assertIn("current_target / MI_PN_TEST_CURRENT_INITIAL", pn_section)
        self.assertNotIn("omega_elec = 2.0f * FOC_PI * MI_PN_ELEC_FREQ_HZ", pn_section)
        self.assertNotIn("FOC_SetCurrentReference(handle->foc, current_target, 0.0f);", pn_section)

    def test_fault_detail_buffer_has_room_for_identification_diagnostics(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        match = re.search(r"#define\s+DRV_UART_BUF_SIZE\s+(\d+)", uart_h)
        self.assertIsNotNone(match)
        self.assertGreaterEqual(
            int(match.group(1)),
            3072,
            "Fault detail text with RsDiag/PnDiag/Chinese hints must not overflow the UART buffer.",
        )
        self.assertIn("FAULT_DETAIL_FORMAT_ERROR", uart_c)
        self.assertIn("s_stats.txErrors++;", uart_c)
        self.assertNotIn("DRV_UART_FAULT_DETAIL_REPEAT_MS", uart_c)
        self.assertNotIn("s_lastFaultDetailRepeatTime", uart_c)
        self.assertIn("s_lastIdentifyState", uart_c)
        self.assertIn("s_lastIdentifyError", uart_c)
        self.assertRegex(
            uart_c,
            r"isNewFault\s*=\s*faultActive[\s\S]*?currentIdentifyState\s*!=\s*s_lastIdentifyState"
            r"[\s\S]*?currentIdentifyError\s*!=\s*s_lastIdentifyError",
            "Fault details should be resent only when the root-cause signature changes, not on a timer.",
        )

    def test_pi_integral_limit_is_scaled_by_ki_to_reach_voltage_limit(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_core.c").read_text(encoding="utf-8")

        self.assertIn("static float FOC_PI_IntegralLimit", foc_c)
        self.assertIn("fabsf(Ki) > FOC_EPSILON", foc_c)
        self.assertIn("(fabsf(output_max) * 0.9f) / fabsf(Ki)", foc_c)
        self.assertIn("pi->integral_max = FOC_PI_IntegralLimit(output_max, Ki);", foc_c)
        self.assertIn("foc->pi_d.integral_max = FOC_PI_IntegralLimit(Vmax, foc->pi_d.Ki);", foc_c)
        self.assertIn("foc->pi_q.integral_max = FOC_PI_IntegralLimit(Vmax, foc->pi_q.Ki);", foc_c)

    def test_clarke_transform_uses_all_three_measured_phase_currents(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_core.c").read_text(encoding="utf-8")

        self.assertIn("abc->c", foc_c)
        self.assertIn("(2.0f * abc->a - abc->b - abc->c) / 3.0f", foc_c)
        self.assertIn("(abc->b - abc->c) / FOC_SQRT3", foc_c)
        self.assertNotIn(
            "alphabeta->alpha = abc->a;\n    alphabeta->beta = (abc->a + 2.0f * abc->b) / FOC_SQRT3;",
            foc_c,
            "Low-side sampling can report a large C-phase current while A/B are small; Clarke must not discard C.",
        )

    def test_ls_identification_falls_back_to_default_inductance(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")

        self.assertIn("MI_LS_FALLBACK_DEFAULT", motor_h)
        self.assertRegex(
            motor_c,
            r"static\s+MI_ErrorCode_t\s+MI_UseLsFallback\s*\([\s\S]*?param->Ld\s*=\s*MI_LS_FALLBACK_DEFAULT;"
            r"[\s\S]*?param->Lq\s*=\s*MI_LS_FALLBACK_DEFAULT;"
            r"[\s\S]*?return\s+MI_ERR_NONE;",
            "Ls fallback must provide a safe default and continue identification.",
        )
        self.assertRegex(
            motor_c,
            r"MI_IdentifyLs[\s\S]*?return\s+MI_UseLsFallback\(handle\);",
            "Ls non-convergence should not force Param Invalid on this low-side sampling bench.",
        )

    def test_ls_identification_cannot_write_param_invalid_inductance(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        param_c = (ROOT / "MDK-ARM" / "code" / "param_storage.c").read_text(encoding="utf-8")
        param_h = (ROOT / "MDK-ARM" / "code" / "param_storage.h").read_text(encoding="utf-8")

        self.assertIn("#define MI_LS_VALID_MAX_H", motor_h)
        self.assertIn("L <= MI_LS_VALID_MAX_H", motor_c)
        self.assertNotIn("L < 0.1f", motor_c)
        self.assertIn("param->Ld <= 0 || param->Ld > 0.01f", param_c)
        self.assertIn("#define PARAM_INVALID_LD", param_h)
        self.assertIn("uint32_t Param_GetInvalidFlags", param_h)
        self.assertIn("flags |= PARAM_INVALID_LD;", param_c)
        self.assertIn("return (Param_GetInvalidFlags(param) == 0U) ? 1U : 0U;", param_c)

    def test_ls_identification_uses_stronger_injection_for_high_resistance_bench(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")

        self.assertRegex(
            motor_h,
            r"#define\s+MI_LS_INJ_AMPLITUDE\s+0\.2f\b",
            "Ls identification should use a stronger injected voltage on the 24V high-Rs bench so Irms does not collapse into noise.",
        )
        self.assertIn("float Vamp = handle->foc->Vbus * MI_LS_INJ_AMPLITUDE;", motor_c)
        self.assertRegex(motor_h, r"#define\s+MI_LS_INJ_VOLTAGE_MAX_V\s+2\.4f\b")
        self.assertIn("Vamp = MI_ClampAbsVoltage(Vamp, MI_LS_INJ_VOLTAGE_MAX_V);", motor_c)

    def test_param_validation_accepts_high_resistance_gimbal_motor(self):
        param_h = (ROOT / "MDK-ARM" / "code" / "param_storage.h").read_text(encoding="utf-8")
        param_c = (ROOT / "MDK-ARM" / "code" / "param_storage.c").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("#define PARAM_RS_MAX_OHM       30.0f", param_h)
        self.assertIn("param->Rs > PARAM_RS_MAX_OHM", param_c)
        self.assertIn("0 < Rs <= %s Ohm", uart_c)
        self.assertIn("paramRsMaxText", uart_c)
        self.assertNotIn("Rs > 10.0f", param_c)
        self.assertNotIn("0 < Rs <= 10 Ohm", uart_c)

    def test_default_motor_params_match_12v_74kv_11pp_motor(self):
        param_c = (ROOT / "MDK-ARM" / "code" / "param_storage.c").read_text(encoding="utf-8")

        self.assertIn("param->Rs = 8.8f;", param_c)
        self.assertIn("param->Ke = 0.129f;", param_c)
        self.assertIn("60/(2pi*74KV)", param_c)
        self.assertIn("param->Pn = 11;", param_c)
        self.assertIn("param->valid_flag = 0xFFFFFFFF;", param_c)

    def test_runtime_forces_user_supplied_motor_constants_without_reidentification(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("static void FOC_App_ApplyKnownMotorConstants", foc_c)
        self.assertIn("handle->motor_param.Rs = 8.8f;", foc_c)
        self.assertIn("handle->motor_param.Ke = 0.129f;", foc_c)
        self.assertIn("handle->motor_param.Pn = 11U;", foc_c)
        self.assertIn("handle->motor_param.valid_flag = 0xFFFFFFFF;", foc_c)
        self.assertIn("FOC_App_ApplyKnownMotorConstants(handle);", foc_c)

    def test_completed_identification_with_invalid_params_enters_param_fault(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        complete_match = re.search(
            r"else if \(MI_IsComplete\(&handle->mi_handle\)\) \{[\s\S]*?handle->enable_identify = 0U;",
            foc_c,
        )
        self.assertIsNotNone(complete_match)
        complete_block = complete_match.group(0)
        self.assertIn("FOC_App_UpdateIdentifyState(handle);", complete_block)
        self.assertIn("if (!handle->motor_identified)", complete_block)
        self.assertIn("FOC_App_EnterFault(handle, FOC_FAULT_PARAM_INVALID);", complete_block)
        self.assertIn("return;", complete_block)

    def test_position_mode_enable_seeds_hold_target(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("position_ref_user_set", foc_h)
        self.assertIn("FOC_POSITION_SPEED_LIMIT_RAD_PER_S", foc_h)

        enable_match = re.search(
            r"void\s+FOC_App_Enable\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{[\s\S]*?handle->state\s*=\s*FOC_STATE_RUNNING;",
            foc_c,
        )
        self.assertIsNotNone(enable_match)
        enable_body = enable_match.group(0)
        self.assertRegex(
            enable_body,
            r"control_mode\s*==\s*FOC_MODE_POSITION[\s\S]*?position_ref_user_set\s*==\s*0U[\s\S]*?FOC_App_RefreshEncoderFeedback\(handle\);[\s\S]*?pos_ref\s*=\s*FOC_App_PositionSensorToControlFrame\(handle,\s*handle->theta_mech\)",
            "Position mode must hold current angle on enable only when no user target was sent.",
        )
        self.assertRegex(
            enable_body,
            r"control_mode\s*==\s*FOC_MODE_POSITION[\s\S]*?speed_ref\s*=\s*0\.0f",
            "Position mode enable should start from hold, not from a stale speed command.",
        )

        mode_match = re.search(
            r"void\s+FOC_App_SetControlMode\s*\(FOC_AppHandle_t\s+\*handle,\s*FOC_ControlMode_t\s+mode\)\s*\{[\s\S]*?\n\}",
            foc_c,
        )
        self.assertIsNotNone(mode_match)
        mode_body = mode_match.group(0)
        self.assertRegex(
            mode_body,
            r"mode\s*==\s*FOC_MODE_POSITION[\s\S]*?enable_pwm\s*==\s*0U[\s\S]*?FOC_App_RefreshEncoderFeedback\(handle\);[\s\S]*?pos_ref\s*=\s*FOC_App_PositionSensorToControlFrame\(handle,\s*handle->theta_mech\)",
            "Selecting position mode while disabled should prepare a zero-error hold target.",
        )
        self.assertRegex(
            mode_body,
            r"mode\s*==\s*FOC_MODE_POSITION[\s\S]*?position_ref_user_set\s*=\s*0U",
            "Mode switch hold target must remain distinguishable from an explicit user target.",
        )

        set_pos_match = re.search(
            r"void\s+FOC_App_SetPositionRef\s*\(FOC_AppHandle_t\s+\*handle,\s*float\s+pos_ref\)\s*\{[\s\S]*?\n\}",
            foc_c,
        )
        self.assertIsNotNone(set_pos_match)
        self.assertIn("handle->position_ref_user_set = 1U;", set_pos_match.group(0))

        self.assertRegex(
            foc_c,
            r"FOC_PositionPD_Init\(&handle->pos_pd,[\s\S]*?FOC_POSITION_SPEED_LIMIT_RAD_PER_S,[\s\S]*?-FOC_POSITION_SPEED_LIMIT_RAD_PER_S",
            "Position loop speed output should be limited for bench-safe position reset.",
        )
        self.assertNotIn(
            'return (strncmp(line, "CMD:PREF,", 9U) == 0) ? 1U : 0U;',
            it_c,
            "PREF must not jump ahead of MODE,2, or MODE can reseed hold and overwrite the user target.",
        )

    def test_compact_uart_reports_position_control_references(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        parser_py = (ROOT / "HostComputer" / "data_parser.py").read_text(encoding="utf-8")
        gui_py = (ROOT / "HostComputer" / "gui_logic.py").read_text(encoding="utf-8")

        for field in ("controlMode", "speed_ref", "pos_ref"):
            self.assertIn(field, uart_h)
            self.assertIn(f"packet->{field}", uart_c)

        for token in ("packet->Id_ref", "packet->Iq_ref", "packet->Vd", "packet->Vq"):
            self.assertIn(token, uart_c)
        self.assertIn("packet->Id_ref = g_foc_app.foc.Id_ref;", uart_c)
        self.assertIn("packet->Iq_ref = g_foc_app.foc.Iq_ref;", uart_c)
        self.assertIn("packet->identifyState,", uart_c)
        self.assertIn("packet->identifyError,", uart_c)
        for token in ("packet->adcCurrentA", "packet->adcCurrentB", "packet->adcCurrentC"):
            self.assertIn(token, uart_c)

        self.assertIn("control_mode: Optional[int] = None", parser_py)
        self.assertIn("Ia: float = 0.0", parser_py)
        self.assertIn("Ib: float = 0.0", parser_py)
        self.assertIn("Ic: float = 0.0", parser_py)
        self.assertIn("speed_ref: float = 0.0", parser_py)
        self.assertIn("pos_ref: float = 0.0", parser_py)
        self.assertIn("packet.control_mode", parser_py)
        self.assertIn("packet.speed_ref", parser_py)
        self.assertIn("packet.pos_ref", parser_py)
        self.assertIn("packet.identify_state = identify_state", parser_py)
        self.assertIn("packet.identify_error = identify_error", parser_py)
        self.assertIn("packet.Ia", gui_py)
        self.assertIn("packet.Ib", gui_py)
        self.assertIn("packet.Ic", gui_py)
        self.assertIn("if packet.control_mode is not None:", gui_py)
        self.assertIn("state.selected_mode = int(packet.control_mode)", gui_py)
        self.assertIn('"position_ref"', gui_py)

    def test_motor_identification_applies_encoder_direction(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        param_c = (ROOT / "MDK-ARM" / "code" / "param_storage.c").read_text(encoding="utf-8")
        param_h = (ROOT / "MDK-ARM" / "code" / "param_storage.h").read_text(encoding="utf-8")

        self.assertIn("int8_t encoder_dir;", motor_h)
        self.assertIn("handle->param->encoder_dir", motor_c)
        self.assertIn("delta_elec * delta_mech", motor_c)
        self.assertIn("theta_offset = FOC_AngleNormalize(-theta_mech * pole_pairs * encoder_dir)", motor_c)
        self.assertNotIn("theta_offset = FOC_AngleNormalize((-theta_mech * pole_pairs * encoder_dir) - (0.5f * FOC_PI))", motor_c)
        self.assertIn("encoder_dir = (handle->motor_param.encoder_dir < 0) ? -1.0f : 1.0f", foc_c)
        self.assertIn("theta_mech * encoder_dir * pole_pairs", foc_c)
        self.assertIn("speed_elec = handle->speed_mech * handle->motor_param.Pn * encoder_dir", foc_c)
        self.assertIn("iq_cmd = iq_ref_mech;", foc_c)
        self.assertNotIn("iq_cmd = iq_ref_mech * encoder_dir", foc_c)
        self.assertNotIn("iq_cmd = -iq_ref_mech * encoder_dir", foc_c)
        self.assertIn("FOC_App_SetCurrentRef(handle, 0.0f, iq_cmd);", foc_c)
        self.assertIn("param->encoder_dir = 1;", param_c)
        self.assertIn("param->encoder_dir != 1", param_c)
        self.assertIn("PARAM_VERSION           0x00010004", param_h)

    def test_param_load_migrates_legacy_encoder_alignment_offset(self):
        param_h = (ROOT / "MDK-ARM" / "code" / "param_storage.h").read_text(encoding="utf-8")
        param_c = (ROOT / "MDK-ARM" / "code" / "param_storage.c").read_text(encoding="utf-8")

        self.assertIn("#define PARAM_VERSION_PRE_ALIGN_D_AXIS   0x00010003", param_h)
        self.assertIn("#define PARAM_VERSION           0x00010004", param_h)
        self.assertIn("if (package->header.version == PARAM_VERSION_PRE_ALIGN_D_AXIS)", param_c)
        self.assertIn("param->theta_offset = FOC_AngleNormalize(param->theta_offset + (0.5f * FOC_PI));", param_c)
        self.assertIn("param->valid_flag = 0xFFFFFFFF;", param_c)
        self.assertIn("package->header.version != PARAM_VERSION", param_c)

    def test_motor_identification_uses_single_direction_weak_motion_verify(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("MI_VERIFY_CURRENT       1.0f", motor_h)
        self.assertIn("MI_VERIFY_MECH_FREQ_HZ  0.03f", motor_h)
        self.assertIn("MI_VERIFY_MIN_MECH_RAD  0.30f", motor_h)
        self.assertIn("MI_VERIFY_DIR_LOCK_RAD  0.15f", motor_h)
        self.assertIn("MI_VERIFY_REVERSE_FAULT_RAD 0.30f", motor_h)
        self.assertIn("MI_VERIFY_PHASE_TIMEOUT_MS 60000", motor_h)
        self.assertIn("MI_VERIFY_NO_MOTION_TIMEOUT_MS 5000", motor_h)
        self.assertIn("MI_VERIFY_NO_MOTION_MIN_RAD 0.03f", motor_h)
        self.assertIn("MI_STATE_MOTION_VERIFY", motor_h)
        self.assertIn("MI_VerifyMotion", motor_c)
        self.assertIn("verify_phase", motor_h)
        self.assertIn("verify_locked_dir", motor_h)
        self.assertIn("verify_expected_dir", motor_h)
        self.assertIn("motion_verify_weak", motor_h)
        self.assertIn("verify_reverse_fault", motor_h)
        self.assertIn('case MI_STATE_MOTION_VERIFY: return "MOTION_VERIFY";', uart_c)
        # Single-direction verify: phase is only 0 or 1 (no phase 2)
        self.assertNotIn("verify_phase == 2U", motor_c)
        # ENCODER_ALIGN → MOTION_VERIFY → COMPLETE chain
        self.assertRegex(
            motor_c,
            r"case\s+MI_STATE_ENCODER_ALIGN:[\s\S]*?MI_EnterState\(handle,\s*MI_STATE_MOTION_VERIFY\);",
        )
        self.assertRegex(
            motor_c,
            r"case\s+MI_STATE_MOTION_VERIFY:[\s\S]*?err\s*=\s*MI_VerifyMotion\(handle\);"
            r"[\s\S]*?MI_EnterState\(handle,\s*MI_STATE_COMPLETE\);",
        )
        # valid_flag set on verify success
        self.assertRegex(
            motor_c,
            r"if\s*\(\s*err\s*==\s*MI_ERR_NONE\s*\)\s*\{[\s\S]*?"
            r"handle->param->valid_flag\s*=\s*0xFFFFFFFF;",
        )
        # Strong pass: accum >= MI_VERIFY_MIN_MECH_RAD AND direction matches
        self.assertIn("MI_VERIFY_MIN_MECH_RAD", motor_c)
        self.assertRegex(motor_c, r"verify_locked_dir\s*==\s*handle->verify_expected_dir")
        # Weak pass: timeout with direction match (rev_fault is diagnostic-only, d-axis drag inherently oscillates)
        self.assertRegex(motor_c, r"motion_verify_weak\s*=\s*1U;")
        self.assertNotRegex(motor_c, r"!handle->verify_reverse_fault[\s\S]*?motion_verify_weak")
        # Reverse fault flag
        self.assertRegex(motor_c, r"verify_reverse_fault\s*=\s*1U;")
        # Vbus/Rs safety current clamp
        self.assertIn("vcur > vlim", motor_c)
        # Fix 1: no-motion returns MI_ERR_TIMEOUT, not MI_ERR_NONE (must not save invalid params)
        self.assertRegex(motor_c, r"motion_verify_weak\s*=\s*1U;[\s\S]*?return MI_ERR_TIMEOUT;")
        # Fix 3: MI_ShutdownOutput helper exists and is used in all exit paths
        self.assertIn("MI_ShutdownOutput", motor_c)

    def test_motor_identification_reports_phase_direction_diagnostics(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("MI_ERR_PHASE_SEQUENCE", motor_h)
        self.assertIn("int8_t pn_observed_dir;", motor_h)
        self.assertIn("int8_t verify_locked_dir;", motor_h)
        self.assertIn("handle->pn_observed_dir", motor_c)
        self.assertIn("identifyPnObservedDir", uart_h)
        self.assertIn("identifyVerifyLockedDir", uart_h)
        self.assertIn("DirDiag:", uart_c)
        self.assertIn("status=", uart_c)
        self.assertIn("rev_fault=", uart_c)
        self.assertIn("motion_verify_status", uart_c)
        self.assertIn("locked_dir", uart_c)
    def test_uart_upload_source_is_armcc_safe_ascii(self):
        uart_path = ROOT / "MDK-ARM" / "code" / "uart_upload.c"
        data = uart_path.read_bytes()
        self.assertTrue(
            all(byte < 0x80 for byte in data),
            "uart_upload.c must stay ASCII-only; use octal UTF-8 escapes for Chinese UART text so Keil/ARMCC5 does not corrupt string literals.",
        )

    def test_single_direction_verify_locks_direction_and_checks_against_expected(self):
        motor_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        motor_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")

        self.assertIn("MI_VERIFY_DIR_LOCK_RAD", motor_h)
        self.assertIn("int8_t verify_locked_dir;", motor_h)
        self.assertIn("verify_expected_dir", motor_h)
        self.assertIn("verify_raw_accum", motor_h)
        self.assertIn("verify_reverse_fault", motor_h)
        # Direction locked from raw mechanical motion
        self.assertRegex(motor_c, r"verify_locked_dir\s*=\s*\(handle->verify_raw_accum >= 0\.0f\)")
        self.assertRegex(motor_c, r"fabsf\(handle->verify_raw_accum\)\s*>=\s*MI_VERIFY_DIR_LOCK_RAD")
        # After lock: mismatch → return PHASE_SEQUENCE (no auto-correct — EncoderAlign already computed theta_offset with old encoder_dir)
        self.assertRegex(motor_c, r"verify_locked_dir\s*!=\s*handle->verify_expected_dir[\s\S]*?return MI_ERR_PHASE_SEQUENCE;")
        self.assertNotIn("handle->param->encoder_dir = handle->verify_locked_dir;", motor_c)
        # Expected from pn_observed_dir or +1
        self.assertRegex(motor_c, r"verify_expected_dir\s*=\s*\(handle->pn_observed_dir != 0\)")
        # Reverse fault is diagnostic-only (doesn't block weak pass — d-axis drag inherently oscillates)
        self.assertRegex(motor_c, r"verify_reverse_fault\s*=\s*1U;")
        self.assertRegex(motor_c, r"verify_locked_dir\s*!=\s*0[\s\S]*?motion_verify_weak\s*=\s*1U;")

    def test_loaded_motor_params_update_loop_after_default_foc_init(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        init_match = re.search(
            r"void\s+FOC_App_Init\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{[\s\S]*?\n\}",
            foc_c,
        )
        self.assertIsNotNone(init_match)
        init_body = init_match.group(0)
        self.assertLess(
            init_body.index("FOC_Init(&handle->foc"),
            init_body.index("if (handle->motor_identified)"),
            "Default FOC_Init must happen before applying loaded motor parameters.",
        )
        self.assertLess(
            init_body.index("FOC_App_UpdateLoopParams(handle);"),
            init_body.index("handle->state = FOC_STATE_READY;"),
            "Loaded/identified params must be applied before READY.",
        )

    def test_drv8350_uses_single_frame_reads_and_keeps_diagnostics_powered(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        drv_c = (ROOT / "MDK-ARM" / "code" / "drv8350s.c").read_text(encoding="utf-8")

        drv_en_high = main_c.find("HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);")
        configure = main_c.find("DRV8350S_Configure(&drv8350s, &config)")
        coast = main_c.find("DRV8350S_DisableGateDrivers(&drv8350s)")

        self.assertNotEqual(drv_en_high, -1, "main.c should power DRV8350S before configuration")
        self.assertNotEqual(configure, -1, "main.c should still configure DRV8350S")
        self.assertNotEqual(coast, -1, "main.c should still force gate drivers off after configuration")
        self.assertLess(drv_en_high, configure)
        self.assertLess(configure, coast)
        self.assertIn("config.vdsLvl = 0x09;", main_c)
        self.assertIn("handle->txBuf[0] = DRV8350S_BuildReadFrame(regAddr);", drv_c)
        self.assertIn("uint16_t response = DRV8350S_ParseResponse(handle->rxBuf[0]);", drv_c)
        self.assertNotIn("handle->txBuf[1] = 0x0000", drv_c)
        self.assertNotIn("handle->rxBuf[1]", drv_c)
        self.assertNotIn("txFrame = 0x0000U;", drv_c)
        self.assertNotIn("HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);", foc_c)
        self.assertIn("#define DRV8350S_NSCS_HIGH_MIN_NS", drv_c)
        self.assertIn("SystemCoreClock", drv_c)
        self.assertIn("while (cycles-- > 0U)", drv_c)
        self.assertNotIn("static void DRV8350S_FrameSpacingDelay(void)\n{\n    __NOP();", drv_c)

    def test_drv8350_distinguishes_comm_fault_from_real_driver_fault_bits(self):
        drv_h = (ROOT / "MDK-ARM" / "code" / "drv8350s.h").read_text(encoding="utf-8")
        drv_c = (ROOT / "MDK-ARM" / "code" / "drv8350s.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("#define DRV8350S_COMM_FAULT_BIT", drv_h)
        self.assertIn("volatile uint8_t  commFaultActive;", drv_h)
        self.assertIn("volatile uint8_t  commValidated;", drv_h)
        self.assertIn("volatile uint16_t lastRxFrame;", drv_h)
        self.assertIn("void DRV8350S_UpdateFaultState(DRV8350S_Handle_t* handle);", drv_h)
        self.assertIn("static uint8_t DRV8350S_IsReadbackInvalid", drv_c)
        self.assertIn("rawFrame == 0xFFFFU", drv_c)
        self.assertIn("faults |= DRV8350S_COMM_FAULT_BIT;", drv_c)
        self.assertIn("handle->runtime.commFaultActive = 1U;", drv_c)
        self.assertIn("handle->runtime.commValidated = 1U;", drv_c)
        self.assertIn("handle->runtime.lastRxFrame = handle->rxBuf[0];", drv_c)
        self.assertIn("void DRV8350S_UpdateFaultState(DRV8350S_Handle_t* handle)", drv_c)
        self.assertIn("DRV8350S_UpdateFaultState(&drv8350s);", it_c)
        self.assertNotIn(
            "drv_fault_active = (((fs1 & 0x07FFU) != 0U) || ((fs2 & 0x00FFU) != 0U)) ? 1U : 0U;",
            it_c,
        )

    def test_drv_hard_faults_force_pwm_shutdown_before_uart_fault_upload(self):
        drv_h = (ROOT / "MDK-ARM" / "code" / "drv8350s.h").read_text(encoding="utf-8")
        drv_c = (ROOT / "MDK-ARM" / "code" / "drv8350s.c").read_text(encoding="utf-8")
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("#define DRV8350S_HARD_SHUTDOWN_FAULT_MASK", drv_h)
        self.assertIn("DRV8350S_VDS_OCP_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_HA_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_LA_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_HB_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_LB_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_HC_BIT", drv_h)
        self.assertIn("DRV8350S_VDS_LC_BIT", drv_h)
        self.assertIn("DRV8350S_GDF_BIT", drv_h)
        self.assertIn("DRV8350S_UVLO_BIT", drv_h)
        self.assertIn("DRV8350S_OTSD_BIT", drv_h)
        self.assertIn("DRV8350S_GDUV_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_HA_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_LA_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_HB_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_LB_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_HC_BIT", drv_h)
        self.assertIn("DRV8350S_VGS_LC_BIT", drv_h)
        self.assertIn("uint8_t DRV8350S_ShouldHardShutdown(uint32_t faultFlags);", drv_h)
        self.assertIn("uint8_t DRV8350S_ShouldHardShutdown(uint32_t faultFlags)", drv_c)
        self.assertIn("faultFlags & DRV8350S_HARD_SHUTDOWN_FAULT_MASK", drv_c)

        hard_mask_match = re.search(
            r"#define DRV8350S_HARD_SHUTDOWN_FAULT_MASK([\s\S]*?)/\* Raw register bit masks",
            drv_h,
        )
        self.assertIsNotNone(hard_mask_match, "hard shutdown mask should be defined in drv8350s.h")
        hard_mask_text = hard_mask_match.group(1)
        self.assertIn("DRV8350S_UVLO_BIT", hard_mask_text)
        self.assertIn("DRV8350S_GDUV_BIT", hard_mask_text)
        self.assertNotIn("DRV8350S_OTW_BIT", hard_mask_text)

        self.assertIn(
            "void FOC_App_RequestFaultShutdownFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault);",
            foc_h,
        )
        self.assertIn("void FOC_App_RequestFaultShutdownFromISR(FOC_AppHandle_t *handle, FOC_FaultCode_t fault)", foc_c)
        self.assertIn("handle->pending_disable = 1U;", foc_c)
        self.assertIn("FOC_App_EnterFault(handle, fault);", foc_c)
        self.assertIn("DRV8350S_ShouldHardShutdown(drv8350s.runtime.faultFlags)", it_c)
        self.assertIn("FOC_App_RequestFaultShutdownFromISR(&g_foc_app, FOC_FAULT_DRV8350S);", it_c)
        spi_callback_start = it_c.index("void HAL_SPI_TxRxCpltCallback")
        spi_callback_end = it_c.index("void HAL_SPI_TxCpltCallback", spi_callback_start)
        spi_callback_section = it_c[spi_callback_start:spi_callback_end]
        self.assertNotIn("DrvUart_UploadImmediate();", spi_callback_section)

    def test_closed_loop_current_reference_is_limited_by_runtime_current_threshold(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("FOC_CURRENT_REF_LIMIT_RATIO", foc_h)
        self.assertIn("FOC_CURRENT_REF_VOLTAGE_RATIO", foc_h)
        self.assertIn("FOC_CURRENT_REF_VOLTAGE_MARGIN", foc_h)
        self.assertIn("#define FOC_CURRENT_REF_VOLTAGE_MARGIN    1.25f", foc_h)
        self.assertIn("static float FOC_App_GetCurrentRefLimit", foc_c)
        self.assertIn("handle->protection.overcurrent_limit_a * FOC_CURRENT_REF_LIMIT_RATIO", foc_c)
        self.assertIn("handle->Vbus * FOC_CURRENT_REF_VOLTAGE_RATIO", foc_c)
        self.assertIn("handle->motor_param.Rs", foc_c)
        self.assertIn("FOC_CURRENT_REF_VOLTAGE_MARGIN", foc_c)
        self.assertIn("if (voltage_limit < limit)", foc_c)

        set_current_match = re.search(
            r"void\s+FOC_App_SetCurrentRef\s*\(FOC_AppHandle_t\s+\*handle,\s*float\s+Id_ref,\s*float\s+Iq_ref\)\s*\{([\s\S]*?)\n\}",
            foc_c,
        )
        self.assertIsNotNone(set_current_match)
        set_current_body = set_current_match.group(1)
        self.assertIn("current_limit = FOC_App_GetCurrentRefLimit(handle);", set_current_body)
        self.assertIn("Id_ref = FOC_Saturate(Id_ref, current_limit, -current_limit);", set_current_body)
        self.assertIn("Iq_ref = FOC_Saturate(Iq_ref, current_limit, -current_limit);", set_current_body)

        speed_loop_match = re.search(
            r"void\s+FOC_App_SpeedLoop\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_PositionLoop",
            foc_c,
        )
        self.assertIsNotNone(speed_loop_match)
        self.assertNotIn("FOC_PI_Update(&handle->pi_speed, speed_error);", speed_loop_match.group(1))
        self.assertIn(
            "FOC_Saturate(FOC_PI_Update(&handle->pi_speed, speed_error)",
            speed_loop_match.group(1),
        )

    def test_speed_loop_uses_bench_safe_bandwidth_and_q_axis_sign(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        speed_loop_match = re.search(
            r"void\s+FOC_App_SpeedLoop\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_PositionLoop",
            foc_c,
        )
        update_params_match = re.search(
            r"static\s+void\s+FOC_App_UpdateLoopParams\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)static\s+void\s+FOC_PositionPD_Init",
            foc_c,
        )
        self.assertIsNotNone(speed_loop_match)
        self.assertIsNotNone(update_params_match)
        self.assertIn("float speed_mech_user = handle->speed_mech * encoder_dir;", speed_loop_match.group(1))
        self.assertIn("speed_feedback = speed_mech_user;", speed_loop_match.group(1))
        self.assertIn("speed_error = speed_ref_temp - speed_feedback;", speed_loop_match.group(1))
        self.assertNotIn("speed_feedback = (handle->control_mode == FOC_MODE_SPEED) ? speed_mech_user : handle->speed_mech;", speed_loop_match.group(1))
        self.assertIn("iq_cmd = iq_ref_mech;", speed_loop_match.group(1))
        self.assertNotIn("iq_cmd = iq_ref_mech * encoder_dir", speed_loop_match.group(1))
        self.assertNotIn("iq_cmd = -iq_ref_mech * encoder_dir", speed_loop_match.group(1))
        self.assertIn("float Kp_s = 0.30f;", update_params_match.group(1))
        self.assertIn("float Ki_s = 0.0f;", update_params_match.group(1))
        self.assertNotIn("float speed_bw", update_params_match.group(1))
        self.assertNotIn("mp->J * ws * ws / Kt", update_params_match.group(1))

    def test_current_loop_defaults_are_12v_low_side_bench_safe(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        foc_core_c = (ROOT / "MDK-ARM" / "code" / "foc_core.c").read_text(encoding="utf-8")

        update_params_match = re.search(
            r"static\s+void\s+FOC_App_UpdateLoopParams\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)static\s+void\s+FOC_PositionPD_Init",
            foc_c,
        )
        self.assertIsNotNone(update_params_match)
        update_body = update_params_match.group(1)

        self.assertIn("#define FOC_CURRENT_LOOP_KP_12V_BENCH       0.30f", foc_h)
        self.assertIn("#define FOC_CURRENT_LOOP_KI_12V_BENCH       0.0f", foc_h)
        self.assertIn("float Kp_i = FOC_CURRENT_LOOP_KP_12V_BENCH;", update_body)
        self.assertIn("float Ki_i = FOC_CURRENT_LOOP_KI_12V_BENCH;", update_body)
        self.assertNotIn("current_bw = 2000.0f", update_body)
        self.assertNotIn("mp->Ld * 2.0f * FOC_PI * current_bw", update_body)
        self.assertNotIn("mp->Rs * 2.0f * FOC_PI * current_bw", update_body)
        self.assertNotIn("if (Ki_i < 0.001f) Ki_i = 0.001f;", update_body)
        self.assertIn("FOC_ZERO_CURRENT_REF_EPS", foc_core_c)
        self.assertIn("FOC_ZERO_CURRENT_FEEDBACK_EPS", foc_core_c)
        self.assertIn("foc->pi_d.integral = 0.0f;", foc_core_c)
        self.assertIn("foc->Vdq.d = 0.0f;", foc_core_c)

    def test_current_loop_uses_motor_resistance_feedforward_for_high_rs_motor(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_core.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_core.c").read_text(encoding="utf-8")
        app_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("float current_resistance_ohm;", foc_h)
        self.assertIn("void FOC_SetCurrentResistance(FOC_Handle_t *foc, float resistance_ohm);", foc_h)
        self.assertIn("void FOC_SetCurrentResistance(FOC_Handle_t *foc, float resistance_ohm)", foc_c)
        self.assertIn("foc->current_resistance_ohm = resistance_ohm;", foc_c)
        self.assertIn("vd_cmd = (foc->current_resistance_ohm * foc->Id_ref) + FOC_PI_Update(&foc->pi_d, error_d);", foc_c)
        self.assertIn("vq_cmd = (foc->current_resistance_ohm * foc->Iq_ref) + FOC_PI_Update(&foc->pi_q, error_q);", foc_c)
        self.assertLess(
            foc_c.index("foc->Vdq.d = 0.0f;"),
            foc_c.index("vd_cmd = (foc->current_resistance_ohm * foc->Id_ref)"),
            "Zero-current neutral path must run before Rs feedforward so idle still produces exactly 0V.",
        )
        self.assertIn("FOC_SetCurrentResistance(&handle->foc, handle->motor_param.Rs);", app_c)

    def test_uart_current_pi_ki_uses_continuous_units_from_host(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        pi_start = it_c.index('if (UART_CommandParseFloat2(cmd, "CMD:PI_CURRENT,", &f1, &f2))')
        pi_end = it_c.index('if (UART_CommandParseFloat2(cmd, "CMD:PI_SPEED,", &f1, &f2))', pi_start)
        pi_section = it_c[pi_start:pi_end]

        self.assertIn("float current_ki_discrete = f2 / (float)FOC_CONTROL_FREQ;", pi_section)
        self.assertIn("FOC_PI_Init(&g_foc_app.foc.pi_d, f1, current_ki_discrete", pi_section)
        self.assertIn("FOC_PI_Init(&g_foc_app.foc.pi_q, f1, current_ki_discrete", pi_section)
        self.assertNotIn("FOC_PI_Init(&g_foc_app.foc.pi_d, f1, f2", pi_section)
        self.assertNotIn("FOC_PI_Init(&g_foc_app.foc.pi_q, f1, f2", pi_section)

    def test_speed_feedback_and_position_steps_are_bench_safe(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("#define FOC_SPEED_LPF_CUTOFF_HZ 20.0f", foc_h)
        self.assertNotIn("#define FOC_SPEED_LPF_CUTOFF_HZ 200.0f", foc_h)
        self.assertIn("#define FOC_SPEED_EST_ACCEL_LIMIT_RAD_PER_S2 60.0f", foc_h)
        self.assertIn("#define FOC_SPEED_STATIC_FRICTION_POS_COMP_A 0.03f", foc_h)
        self.assertIn("#define FOC_SPEED_STATIC_FRICTION_NEG_COMP_A 0.03f", foc_h)
        self.assertIn("#define FOC_SPEED_POSITIVE_IQ_LIMIT_A 0.25f", foc_h)
        self.assertIn("#define FOC_SPEED_NEGATIVE_IQ_LIMIT_A 0.25f", foc_h)
        self.assertIn("#define FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A 0.25f", foc_h)
        self.assertIn("#define FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A 0.25f", foc_h)
        self.assertNotIn("#define FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A 0.22f", foc_h)
        self.assertNotIn("#define FOC_POSITION_IQ_LIMIT_A", foc_h)
        self.assertIn("#define FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 1.0f", foc_h)
        self.assertNotIn("#define FOC_SPEED_STATIC_FRICTION_COMP_A", foc_h)
        self.assertNotIn("#define FOC_SPEED_STATIC_FRICTION_POS_COMP_A 0.07f", foc_h)
        self.assertNotIn("#define FOC_SPEED_STATIC_FRICTION_NEG_COMP_A 0.05f", foc_h)
        self.assertIn("#define FOC_POSITION_USER_POSITIVE_STATIC_FRICTION_COMP_A 0.03f", foc_h)
        self.assertIn("#define FOC_POSITION_USER_NEGATIVE_STATIC_FRICTION_COMP_A 0.03f", foc_h)
        self.assertNotIn("#define FOC_POSITION_STATIC_FRICTION_COMP_A", foc_h)
        self.assertIn("#define FOC_POSITION_PD_KP_DEFAULT 4.0f", foc_h)
        self.assertIn("#define FOC_POSITION_PD_KD_DEFAULT 0.12f", foc_h)
        self.assertIn("#define FOC_POSITION_PD_KP_SCALE 1.0f", foc_h)
        self.assertIn("#define FOC_POSITION_PD_KP_MIN 4.0f", foc_h)
        self.assertIn("#define FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S 0.05f", foc_h)
        self.assertIn("#define FOC_SPEED_STATIC_FRICTION_ACTIVE_RAD_PER_S 0.20f", foc_h)
        self.assertIn("#define FOC_POSITION_STATIC_FRICTION_ENTER_RAD 0.052f", foc_h)
        self.assertIn("#define FOC_POSITION_STATIC_FRICTION_EXIT_RAD 0.017f", foc_h)
        self.assertIn("uint8_t position_friction_active;", foc_h)
        self.assertNotIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           0.50f", foc_h)
        self.assertNotIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           0.80f", foc_h)
        self.assertNotIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           0.30f", foc_h)
        self.assertNotIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           0.20f", foc_h)
        self.assertNotIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           5.0f", foc_h)
        self.assertIn("#define FOC_POSITION_SPEED_LIMIT_RAD_PER_S           1.0f", foc_h)
        self.assertIn("FOC_SPEED_LPF_CUTOFF_HZ", foc_c)
        self.assertIn("FOC_SPEED_EST_ACCEL_LIMIT_RAD_PER_S2", foc_c)
        self.assertIn("speed_step = FOC_SPEED_EST_ACCEL_LIMIT_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ;", foc_c)
        self.assertIn("speed_next = handle->speed_mech + alpha * (speed_raw - handle->speed_mech);", foc_c)
        self.assertIn("FOC_Saturate(speed_next - handle->speed_mech, speed_step, -speed_step)", foc_c)
        self.assertIn("FOC_SPEED_STATIC_FRICTION_POS_COMP_A", foc_c)
        self.assertIn("FOC_SPEED_STATIC_FRICTION_NEG_COMP_A", foc_c)
        self.assertIn("fabsf(speed_ref_temp) > FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S", foc_c)
        self.assertIn("fabsf(speed_error) > FOC_SPEED_STATIC_FRICTION_ERROR_RAD_PER_S", foc_c)
        self.assertIn("fabsf(speed_feedback) < FOC_SPEED_STATIC_FRICTION_ACTIVE_RAD_PER_S", foc_c)
        self.assertIn("(speed_ref_temp * speed_error) > 0.0f", foc_c)
        self.assertNotIn("float friction_user_dir = friction_dir * encoder_dir;", foc_c)
        self.assertIn("friction_comp = (friction_dir > 0.0f) ?\n                                FOC_POSITION_USER_POSITIVE_STATIC_FRICTION_COMP_A :\n                                FOC_POSITION_USER_NEGATIVE_STATIC_FRICTION_COMP_A;", foc_c)
        self.assertIn("friction_comp = (friction_dir > 0.0f) ?\n                                FOC_SPEED_STATIC_FRICTION_POS_COMP_A :\n                                FOC_SPEED_STATIC_FRICTION_NEG_COMP_A;", foc_c)
        self.assertIn("float iq_limit_pos = FOC_App_GetCurrentRefLimit(handle);", foc_c)
        self.assertIn("if (handle->control_mode == FOC_MODE_SPEED) {", foc_c)
        self.assertIn("if (iq_limit_pos > FOC_SPEED_POSITIVE_IQ_LIMIT_A)", foc_c)
        self.assertIn("iq_limit_pos = FOC_SPEED_POSITIVE_IQ_LIMIT_A;", foc_c)
        self.assertIn("if ((-iq_limit_neg) > FOC_SPEED_NEGATIVE_IQ_LIMIT_A)", foc_c)
        self.assertIn("iq_limit_neg = -FOC_SPEED_NEGATIVE_IQ_LIMIT_A;", foc_c)
        self.assertIn("} else if (handle->control_mode == FOC_MODE_POSITION) {", foc_c)
        self.assertIn("iq_limit_pos = FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A;", foc_c)
        self.assertIn("iq_limit_neg = -FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A;", foc_c)
        self.assertNotIn("iq_limit_pos = FOC_POSITION_USER_NEGATIVE_IQ_LIMIT_A;", foc_c)
        self.assertNotIn("iq_limit_neg = -FOC_POSITION_USER_POSITIVE_IQ_LIMIT_A;", foc_c)
        self.assertIn("iq_ref_mech = FOC_Saturate(iq_ref_mech, iq_limit_pos, iq_limit_neg);", foc_c)
        self.assertIn("FOC_App_ClampSpeedPiIntegral(&handle->pi_speed, iq_limit_pos, iq_limit_neg);", foc_c)
        self.assertIn("speed_ref_temp = handle->speed_ref_ramped;", foc_c)
        self.assertIn("FOC_SPEED_REF_RAMP_RATE_RAD_PER_S2 / (float)FOC_SPEED_LOOP_FREQ", foc_c)
        self.assertIn("handle->speed_ref_ramped += FOC_Saturate(handle->speed_ref - handle->speed_ref_ramped", foc_c)
        self.assertNotIn("speed_ref_temp = handle->speed_ref;  /* 位置环更新的速度给定 */", foc_c)
        self.assertIn("float pos_error_for_friction = FOC_AngleNormalize(handle->pos_ref - theta_mech_control);", foc_c)
        self.assertIn("if (fabsf(pos_error_for_friction) > FOC_POSITION_STATIC_FRICTION_ENTER_RAD)", foc_c)
        self.assertIn("handle->position_friction_active = 1U;", foc_c)
        self.assertIn("else if (fabsf(pos_error_for_friction) < FOC_POSITION_STATIC_FRICTION_EXIT_RAD)", foc_c)
        self.assertIn("handle->position_friction_active = 0U;", foc_c)
        self.assertIn("if (handle->position_friction_active != 0U) {", foc_c)
        self.assertIn("friction_dir = pos_error_for_friction;", foc_c)
        self.assertNotIn("friction_dir = speed_ref_temp;\n            }\n        }\n\n        if (friction_dir != 0.0f)", foc_c)
        self.assertIn("friction_delta = (friction_dir > 0.0f) ? friction_comp : -friction_comp;", foc_c)
        self.assertIn("iq_ref_mech += friction_delta;", foc_c)
        self.assertNotIn("iq_ref_mech += (speed_error > 0.0f) ? FOC_SPEED_STATIC_FRICTION_COMP_A : -FOC_SPEED_STATIC_FRICTION_COMP_A;", foc_c)
        self.assertIn("FOC_POSITION_SPEED_LIMIT_RAD_PER_S", foc_c)
        self.assertIn("FOC_POSITION_PD_KP_DEFAULT", foc_c)
        self.assertIn("Kp_p = sqrtf(Kp_s) * FOC_POSITION_PD_KP_SCALE;", foc_c)
        self.assertIn("if (Kp_p < FOC_POSITION_PD_KP_MIN) Kp_p = FOC_POSITION_PD_KP_MIN;", foc_c)
        self.assertNotIn("Kp_p = sqrtf(Kp_s) * 8.0f;", foc_c)

    def test_speed_estimator_state_resets_with_motion_state(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        speed_loop_match = re.search(
            r"void\s+FOC_App_SpeedLoop\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_PositionLoop",
            foc_c,
        )
        reset_motion_match = re.search(
            r"void\s+FOC_App_ResetMotionState\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_SetCurrentRef",
            foc_c,
        )
        enable_match = re.search(
            r"void\s+FOC_App_Enable\s*\(FOC_AppHandle_t\s+\*handle\)\s*\{([\s\S]*?)void\s+FOC_App_Disable",
            foc_c,
        )
        self.assertIsNotNone(speed_loop_match)
        self.assertIsNotNone(reset_motion_match)
        self.assertIsNotNone(enable_match)

        self.assertIn("float speed_theta_prev;", foc_h)
        self.assertIn("float speed_ref_ramped;", foc_h)
        self.assertIn("uint8_t speed_loop_ready;", foc_h)
        self.assertNotIn("static float theta_prev", speed_loop_match.group(1))
        self.assertNotIn("static uint8_t speed_loop_ready", speed_loop_match.group(1))
        self.assertIn("handle->speed_loop_ready = 0U;", reset_motion_match.group(1))
        self.assertIn("handle->speed_ref_ramped = 0.0f;", reset_motion_match.group(1))
        self.assertIn("handle->speed_theta_prev = handle->theta_mech;", reset_motion_match.group(1))
        self.assertIn("handle->speed_loop_ready = 0U;", enable_match.group(1))
        self.assertIn("handle->speed_ref_ramped = 0.0f;", enable_match.group(1))
        self.assertIn("handle->speed_theta_prev = handle->theta_mech;", enable_match.group(1))

        set_pos_match = re.search(
            r"void\s+FOC_App_SetPositionRef\s*\(FOC_AppHandle_t\s+\*handle,\s*float\s+pos_ref\)\s*\{([\s\S]*?)\n\}",
            foc_c,
        )
        self.assertIsNotNone(set_pos_match)
        self.assertIn("handle->speed_ref_ramped = 0.0f;", set_pos_match.group(1))
        self.assertIn("handle->pi_speed.integral = 0.0f;", set_pos_match.group(1))

    def test_spi_bringup_uses_bench_safe_prescaler(self):
        spi_c = (ROOT / "Core" / "Src" / "spi.c").read_text(encoding="utf-8")

        self.assertEqual(
            spi_c.count("SPI_BAUDRATEPRESCALER_128"),
            2,
            "SPI1 and SPI3 should both use the reduced bench prescaler",
        )
    def test_startup_waits_for_external_spi_peripherals_to_settle(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        settle = main_c.find("HAL_Delay(FOC_EXT_SPI_POWERUP_DELAY_MS);")
        tle_init = main_c.find("TLE5012_Init();")
        drv_init = main_c.find("DRV8350S_Init(&drv8350s, &hspi1, &htim1,")
        drv_en_high = main_c.find("HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);")
        drv_en_wait = main_c.find("HAL_Delay(2);")

        self.assertIn("#define FOC_EXT_SPI_POWERUP_DELAY_MS 20U", main_c)
        self.assertNotEqual(settle, -1, "main.c should wait for external SPI peripherals to power up")
        self.assertNotEqual(tle_init, -1)
        self.assertNotEqual(drv_init, -1)
        self.assertNotEqual(drv_en_high, -1)
        self.assertNotEqual(drv_en_wait, -1, "DRV_EN should be followed by the longer settle delay")
        self.assertLess(settle, tle_init)
        self.assertLess(settle, drv_init)
        self.assertLess(drv_en_high, drv_en_wait)

    def test_drv_startup_fault_keeps_uart_alive_for_fault_upload(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")

        self.assertIn("static void Main_RecordDrvStartupCommFault(void);", main_c)
        self.assertIn("drv8350s.runtime.faultFlags |= DRV8350S_COMM_FAULT_BIT;", main_c)
        self.assertIn("g_foc_app.fault_code = FOC_FAULT_DRV8350S;", main_c)
        self.assertIn("g_foc_app.state = FOC_STATE_FAULT;", main_c)
        self.assertEqual(main_c.count("Main_RecordDrvStartupCommFault();"), 2)
        self.assertIsNone(
            re.search(r"DRV8350S_Init\([\s\S]*?\)\s*!=\s*0\)\s*\{\s*Error_Handler\(\);", main_c),
            "DRV startup faults must no longer hard-stop the firmware before UART fault upload starts.",
        )
        self.assertIsNone(
            re.search(r"DRV8350S_Configure\([\s\S]*?\)\s*!=\s*0\)\s*\{\s*Error_Handler\(\);", main_c),
            "DRV configure faults must be reported over UART instead of killing the main loop.",
        )
        self.assertIn("保留 `UART1` 和主循环继续运行", readme)

    def test_post_uart_startup_failures_do_not_hard_stop_before_uart_diagnostics(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        uart_init_idx = main_c.index("DrvUart_Init(&huart1, &drv8350s);")
        while_idx = main_c.index("while (1)", uart_init_idx)
        post_uart_section = main_c[uart_init_idx:while_idx]

        self.assertNotIn(
            "Error_Handler();",
            post_uart_section,
            "Any startup failure after DrvUart_Init currently silences UART diagnostics by trapping in Error_Handler.",
        )
        self.assertIn("Main_RecordStartupFault(", post_uart_section)

    def test_stall_open_loop_enable_defaults_to_real_trial_spin_contract(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        host_window = (ROOT / "HostComputer" / "main_window.py").read_text(encoding="utf-8")

        self.assertIn("#define FOC_STALL_OPEN_LOOP_DEFAULT_SPEED_RAD_PER_S", foc_h)
        self.assertIn("handle->control_mode = FOC_MODE_SPEED;", foc_c)
        self.assertIn("FOC_STALL_OPEN_LOOP_DEFAULT_SPEED_RAD_PER_S", foc_c)
        self.assertIn("FOC_STALL_OPEN_LOOP_DEFAULT_IQ_A", foc_c)
        self.assertIn("fabsf(handle->speed_ref) < 1e-3f", foc_c)
        self.assertIn("fabsf(handle->Iq_ref) < 1e-3f", foc_c)
        self.assertIn("commands.append(CommandBuilder.set_mode(1))", host_window)

    def test_debug_boot_path_uses_hse_primary_with_hsi_fallback_and_gates_fdcan_init(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        fdcan_h = (ROOT / "Core" / "Inc" / "fdcan.h").read_text(encoding="utf-8")
        fdcan_c = (ROOT / "Core" / "Src" / "fdcan.c").read_text(encoding="utf-8")
        ioc = (ROOT / "24V FOC Controller.ioc").read_text(encoding="utf-8")

        self.assertIn("static uint8_t s_system_clock_fallback_to_hsi = 0U;", main_c)
        self.assertIn("RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;", main_c)
        self.assertIn("RCC_OscInitStruct.HSEState = RCC_HSE_ON;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 5;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 192;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;", main_c)
        self.assertIn("RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;", main_c)
        self.assertIn("RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 4;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 60;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;", main_c)
        self.assertIn("if (SystemClock_TryConfigureHse() != HAL_OK)", main_c)
        self.assertIn("s_system_clock_fallback_to_hsi = 1U;", main_c)
        self.assertIn("HAL_RCC_DeInit();", main_c)
        self.assertIn("if (SystemClock_TryConfigureHsi() != HAL_OK)", main_c)
        self.assertIn("RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSE", ioc)
        self.assertIn("RCC.DIVM1=5", ioc)
        self.assertIn("RCC.DIVN1=192", ioc)
        self.assertIn("#define FOC_DEBUG_DISABLE_FDCAN_INIT 1U", fdcan_h)
        self.assertIn("#if FOC_DEBUG_DISABLE_FDCAN_INIT", fdcan_c)
        self.assertIn("return;", fdcan_c)

    def test_system_clock_voltage_scaling_waits_are_bounded_before_uart_boot(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        self.assertIn("#define FOC_PWR_VOSRDY_TIMEOUT_MS 10U", main_c)
        self.assertIn("static HAL_StatusTypeDef SystemClock_WaitForVosReady(uint32_t timeoutMs);", main_c)
        self.assertIn("static HAL_StatusTypeDef SystemClock_WaitForVosReady(uint32_t timeoutMs)", main_c)
        self.assertNotIn("while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}", main_c)
        self.assertIn("if (SystemClock_WaitForVosReady(FOC_PWR_VOSRDY_TIMEOUT_MS) != HAL_OK)", main_c)

    def test_uart_is_available_before_risky_system_clock_config(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        hal_init = main_c.index("HAL_Init();")
        early_gpio = main_c.index("MX_GPIO_Init();", hal_init)
        early_dma = main_c.index("MX_DMA_Init();", hal_init)
        early_uart = main_c.index("MX_USART1_UART_Init();", hal_init)
        early_boot = main_c.index('Main_BootUartSend("BOOT,EARLY_UART\\r\\n"', hal_init)
        clock_config = main_c.index("SystemClock_Config();", hal_init)
        clock_ready = main_c.index('Main_BootUartSend("BOOT,CLOCK_READY\\r\\n"', clock_config)
        uart_reinit = main_c.index("MX_USART1_UART_Init();", clock_config)

        self.assertLess(early_gpio, clock_config)
        self.assertLess(early_dma, clock_config)
        self.assertLess(early_uart, clock_config)
        self.assertLess(early_boot, clock_config)
        self.assertLess(clock_config, clock_ready)
        self.assertLess(clock_ready, uart_reinit)

    def test_adc_triggers_on_oc4_compare_edge_not_update_edge(self):
        adc_c = (ROOT / "Core" / "Src" / "adc.c").read_text(encoding="utf-8")
        tim_c = (ROOT / "Core" / "Src" / "tim.c").read_text(encoding="utf-8")
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")

        self.assertIn("hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;", adc_c)
        self.assertIn("hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;", adc_c)
        self.assertIn("sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;", tim_c)
        self.assertIn("sConfigOC.OCMode = TIM_OCMODE_PWM1;", tim_c)
        self.assertIn("sConfigOC.Pulse = TIM1_ADC_TRIGGER_PULSE;", tim_c)
        self.assertIn("TIM1_TRGO2_OC4REF_FALLING", adc_h)
        self.assertNotIn("ADC_EXTERNALTRIGCONVEDGE_RISING", adc_c)
    def test_adc_low_side_sampling_timing_contract(self):
        adc_c = (ROOT / "Core" / "Src" / "adc.c").read_text(encoding="utf-8")
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        tim_c = (ROOT / "Core" / "Src" / "tim.c").read_text(encoding="utf-8")
        dma_c = (ROOT / "Core" / "Src" / "dma.c").read_text(encoding="utf-8")
        msp_c = (ROOT / "Core" / "Src" / "stm32h7xx_hal_msp.c").read_text(encoding="utf-8")

        self.assertIn("hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;", adc_c)
        self.assertIn("hadc1.Init.NbrOfConversion = 4;", adc_c)
        self.assertIn("sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;", adc_c)
        self.assertIn("sConfig.Rank = ADC_REGULAR_RANK_4;", adc_c)
        self.assertIn("sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;", adc_c)
        self.assertIn("HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, 4)", main_c)
        self.assertIn("sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;", tim_c)
        self.assertIn("#define TIM1_ADC_TRIGGER_PULSE 45U", tim_c)
        self.assertIn("sConfigOC.Pulse = TIM1_ADC_TRIGGER_PULSE;", tim_c)
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

    def test_adc_noise_diagnostic_captures_raw_statistics_without_pwm(self):
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")
        adc_c = (ROOT / "MDK-ARM" / "code" / "adc_sampling.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("ADC_NOISE_MAX_SAMPLES", adc_h)
        self.assertIn("ADC_Sampling_NoiseChannelStats_t", adc_h)
        self.assertIn("uint16_t min;", adc_h)
        self.assertIn("uint16_t max;", adc_h)
        self.assertIn("uint16_t mean;", adc_h)
        self.assertIn("uint16_t peakToPeak;", adc_h)
        self.assertIn("uint16_t stddev;", adc_h)
        self.assertIn("ADC_Sampling_NoiseStats_t", adc_h)
        self.assertIn("ADC_Sampling_CaptureNoiseStats(uint16_t requestedSamples, ADC_Sampling_NoiseStats_t *stats);", adc_h)

        self.assertIn("static uint16_t ADC_Sampling_IntegerSqrt", adc_c)
        self.assertIn("uint16_t ADC_Sampling_CaptureNoiseStats", adc_c)
        self.assertIn("ADC_NOISE_MAX_SAMPLES", adc_c)
        self.assertIn("ADC_CH_CURRENT_A", adc_c)
        self.assertIn("ADC_CH_CURRENT_B", adc_c)
        self.assertIn("ADC_CH_CURRENT_C", adc_c)
        self.assertIn("ADC_CH_VBUS", adc_c)
        self.assertIn("sumSq", adc_c)
        self.assertIn("HAL_GetTick()", adc_c)

        self.assertIn("CMD:ADC_NOISE,%ld", it_c)
        self.assertIn("UART_CommandCanRunAdcNoiseTest()", it_c)
        self.assertIn("ADC_NOISE,BUSY", it_c)
        self.assertIn("ADC_NOISE,START", it_c)
        self.assertIn("ADC_NOISE,OK", it_c)
        self.assertIn("UART_CommandServiceAdcNoise()", it_c)
        self.assertIn("static uint8_t s_adcNoiseActive", it_c)
        self.assertIn("s_adcNoiseLastFrameSequence", it_c)
        self.assertIn("ADC_Sampling_GetData()", it_c)
        self.assertIn(",%s:min=%u,max=%u,mean=%u,pp=%u,std=%u", it_c)
        self.assertIn('UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "A"', it_c)
        self.assertIn('UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "B"', it_c)
        self.assertIn('UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "C"', it_c)
        self.assertIn('UART_CommandAppendAdcNoiseChannel(response, sizeof(response), "VBUS"', it_c)
        self.assertIn("std=", it_c)

    def test_adc_noise_stddev_uses_raw_sum_variance_without_rounded_mean_error(self):
        adc_c = (ROOT / "MDK-ARM" / "code" / "adc_sampling.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        for source in (adc_c, it_c):
            self.assertIn("varianceNumerator", source)
            self.assertIn("((uint64_t)accum->sum * (uint64_t)accum->sum)", source)
            self.assertNotIn("meanSq = mean * mean;", source)
            self.assertNotIn("avgSq > meanSq", source)

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

    def test_foc_loop_reconstructs_current_from_two_valid_low_side_windows(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        core_h = (ROOT / "MDK-ARM" / "code" / "foc_core.h").read_text(encoding="utf-8")
        core_c = (ROOT / "MDK-ARM" / "code" / "foc_core.c").read_text(encoding="utf-8")

        tim1_start = foc_c.index("void FOC_App_TIM1_IRQHandler")
        tim1_end = foc_c.index("static void FOC_App_RequestDisableFromISR", tim1_start)
        tim1_section = foc_c[tim1_start:tim1_end]

        self.assertIn("uint32_t adc_invalid_low_side_count;", foc_h)
        self.assertIn("uint32_t adc_valid_low_side_count;", foc_h)
        self.assertIn("uint32_t adc_forced_low_side_count;", foc_h)
        self.assertIn("uint32_t adc_invalid_low_side_streak;", foc_h)
        self.assertIn("#define FOC_LOW_SIDE_ZERO_WINDOW_FORCE_INTERVAL 32U", foc_h)
        self.assertIn("uint8_t current_feedback_valid = 0U;", tim1_section)
        self.assertIn("uint8_t low_side_valid_count = 0U;", tim1_section)
        self.assertIn("float current_a;", tim1_section)
        self.assertIn("float current_b;", tim1_section)
        self.assertIn("float current_c;", tim1_section)
        self.assertIn("low_side_valid_count += (adc->lowSideValidA != 0U) ? 1U : 0U;", tim1_section)
        self.assertIn("low_side_valid_count += (adc->lowSideValidB != 0U) ? 1U : 0U;", tim1_section)
        self.assertIn("low_side_valid_count += (adc->lowSideValidC != 0U) ? 1U : 0U;", tim1_section)
        self.assertIn("current_feedback_valid = (low_side_valid_count >= 2U) ? 1U : 0U;", tim1_section)
        self.assertIn("if ((handle->state == FOC_STATE_PARAM_IDENTIFY) &&\n        handle->enable_identify &&\n        (handle->mi_handle.state == MI_STATE_LS_IDENTIFY ||\n         handle->mi_handle.state == MI_STATE_RS_IDENTIFY ||\n         handle->mi_handle.state == MI_STATE_ENCODER_ALIGN ||\n         handle->mi_handle.state == MI_STATE_MOTION_VERIFY)) {\n        current_feedback_valid = 1U;", tim1_section)
        self.assertIn("} else if (low_side_valid_count == 0U) {", tim1_section)
        self.assertIn("handle->adc_invalid_low_side_streak++;", tim1_section)
        self.assertIn("handle->adc_invalid_low_side_streak >= FOC_LOW_SIDE_ZERO_WINDOW_FORCE_INTERVAL", tim1_section)
        self.assertIn("handle->adc_forced_low_side_count++;", tim1_section)
        self.assertIn("} else {\n        handle->adc_invalid_low_side_streak = 0U;\n    }", tim1_section)
        self.assertIn("if (!((handle->state == FOC_STATE_PARAM_IDENTIFY) &&\n          handle->enable_identify &&\n          (handle->mi_handle.state == MI_STATE_LS_IDENTIFY ||\n           handle->mi_handle.state == MI_STATE_RS_IDENTIFY ||\n           handle->mi_handle.state == MI_STATE_ENCODER_ALIGN ||\n           handle->mi_handle.state == MI_STATE_MOTION_VERIFY)) &&\n        (low_side_valid_count == 2U)) {", tim1_section)
        self.assertIn("current_c = -(current_a + current_b);", tim1_section)
        self.assertIn("current_b = -(current_a + current_c);", tim1_section)
        self.assertIn("current_a = -(current_b + current_c);", tim1_section)
        self.assertIn("handle->adc_invalid_low_side_count++;", tim1_section)
        self.assertIn("handle->adc_valid_low_side_count++;", tim1_section)
        self.assertIn("if (!current_feedback_valid) {", tim1_section)
        self.assertIn("FOC_RegenerateVoltageVector(&handle->foc);", tim1_section)
        self.assertIn("if (handle->state == FOC_STATE_PARAM_IDENTIFY && !identify_direct_svpwm)", tim1_section)
        self.assertIn("handle->foc.Vdq.d = 0.0f;", tim1_section)
        self.assertIn("handle->foc.Vdq.q = 0.0f;", tim1_section)
        self.assertIn("handle->foc.ValphaBeta.alpha = 0.0f;", tim1_section)
        self.assertIn("handle->foc.ValphaBeta.beta = 0.0f;", tim1_section)
        self.assertIn("FOC_SVPWM_Generate(&handle->foc.ValphaBeta, handle->foc.Vbus, &handle->foc.svpwm)", tim1_section)
        self.assertIn("} else {\n        /* 执行FOC计算 */\n        FOC_Run(&handle->foc);\n    }", tim1_section)
        self.assertIn("FOC_App_ShouldBootstrapNeutralPwm(handle, adc)", tim1_section)
        self.assertIn("static uint8_t FOC_App_ShouldBootstrapNeutralPwm", foc_c)
        bootstrap_start = foc_c.index("static uint8_t FOC_App_ShouldBootstrapNeutralPwm(const FOC_AppHandle_t *handle, const ADC_Sampling_t *adc)\n{")
        bootstrap_end = foc_c.index("/**\n * @brief 转速计算和速度环控制", bootstrap_start)
        bootstrap_section = foc_c[bootstrap_start:bootstrap_end]
        self.assertIn("handle->state != FOC_STATE_RUNNING", bootstrap_section)
        self.assertIn("adc->lowSideValidA != 0U", bootstrap_section)
        self.assertIn("adc->pwmCompareA != adc->pwmCompareB", bootstrap_section)
        self.assertIn("adc->pwmCompareB != adc->pwmCompareC", bootstrap_section)
        self.assertIn("handle->foc.svpwm.sector != 0U", bootstrap_section)
        self.assertIn("fabsf(handle->foc.Vdq.d) > FOC_NEUTRAL_BOOTSTRAP_VOLTAGE_EPS", bootstrap_section)
        self.assertIn("fabsf(handle->foc.Id_ref) <= FOC_NEUTRAL_BOOTSTRAP_CURRENT_EPS", bootstrap_section)
        self.assertIn("fabsf(handle->foc.Iq_ref) <= FOC_NEUTRAL_BOOTSTRAP_CURRENT_EPS", bootstrap_section)
        self.assertIn("void FOC_RegenerateVoltageVector(FOC_Handle_t *foc);", core_h)
        self.assertIn("void FOC_RegenerateVoltageVector(FOC_Handle_t *foc)", core_c)
        regen_start = core_c.index("void FOC_RegenerateVoltageVector")
        regen_end = core_c.index("void FOC_Run", regen_start)
        regen_section = core_c[regen_start:regen_end]
        self.assertIn("FOC_Inverse_Park_Transform(&foc->Vdq", regen_section)
        self.assertIn("FOC_SVPWM_Generate(&foc->ValphaBeta", regen_section)
        self.assertNotIn("FOC_PI_Update", regen_section)

    def test_fault_detail_reports_speed_loop_diagnostics(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        for field in (
            "speed_loop_ref_diag",
            "speed_loop_mech_diag",
            "speed_loop_error_diag",
            "speed_loop_iq_mech_diag",
            "speed_loop_friction_diag",
            "speed_loop_iq_cmd_diag",
        ):
            self.assertIn(f"float {field};", foc_h)

        speed_loop_start = foc_c.index("void FOC_App_SpeedLoop")
        speed_loop_end = foc_c.index("void FOC_App_PositionLoop", speed_loop_start)
        speed_loop_section = foc_c[speed_loop_start:speed_loop_end]
        self.assertIn("handle->speed_loop_ref_diag = speed_ref_temp;", speed_loop_section)
        self.assertIn("handle->speed_loop_mech_diag = speed_feedback;", speed_loop_section)
        self.assertIn("handle->speed_loop_error_diag = speed_error;", speed_loop_section)
        self.assertIn("handle->speed_loop_iq_mech_diag = iq_ref_mech;", speed_loop_section)
        self.assertIn("handle->speed_loop_friction_diag = friction_delta;", speed_loop_section)
        self.assertIn("handle->speed_loop_iq_cmd_diag = iq_cmd;", speed_loop_section)

        for field in (
            "speedLoopRef",
            "speedLoopMech",
            "speedLoopError",
            "speedLoopIqMech",
            "speedLoopFriction",
            "speedLoopIqCmd",
        ):
            self.assertIn(field, uart_h)

        self.assertIn("packet->speedLoopRef = g_foc_app.speed_loop_ref_diag;", uart_c)
        self.assertIn("packet->speedLoopIqCmd = g_foc_app.speed_loop_iq_cmd_diag;", uart_c)
        self.assertIn("  SpeedLoopDiag: ref=%s rad/s | mech=%s rad/s | err=%s rad/s | iq_mech=%s A | friction=%s A | iq_cmd=%s A\\r\\n", uart_c)

    def test_fault_detail_reports_position_loop_diagnostics(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        for field in (
            "position_loop_error_diag",
            "position_loop_pd_out_diag",
            "position_loop_pd_sat_diag",
            "position_loop_speed_ramp_sat_diag",
            "position_loop_iq_pos_sat_diag",
            "position_loop_iq_neg_sat_diag",
        ):
            self.assertIn(field, foc_h)

        position_loop_start = foc_c.index("void FOC_App_PositionLoop")
        position_loop_end = foc_c.index("void FOC_App_ParamIdentifyLoop", position_loop_start)
        position_loop_section = foc_c[position_loop_start:position_loop_end]
        self.assertIn("handle->position_loop_error_diag = pos_error;", position_loop_section)
        self.assertIn("handle->position_loop_pd_out_diag = pos_pd_out;", position_loop_section)
        self.assertIn("handle->position_loop_pd_sat_diag =", position_loop_section)

        speed_loop_start = foc_c.index("void FOC_App_SpeedLoop")
        speed_loop_end = foc_c.index("void FOC_App_PositionLoop", speed_loop_start)
        speed_loop_section = foc_c[speed_loop_start:speed_loop_end]
        self.assertIn("handle->position_loop_speed_ramp_sat_diag =", speed_loop_section)
        self.assertIn("handle->position_loop_iq_pos_sat_diag =", speed_loop_section)
        self.assertIn("handle->position_loop_iq_neg_sat_diag =", speed_loop_section)

        for field in (
            "positionLoopError",
            "positionLoopPdOut",
            "positionLoopPdSat",
            "positionLoopRampSat",
            "positionLoopIqPosSat",
            "positionLoopIqNegSat",
        ):
            self.assertIn(field, uart_h)

        self.assertIn("packet->positionLoopError = g_foc_app.position_loop_error_diag;", uart_c)
        self.assertIn("packet->positionLoopIqNegSat = g_foc_app.position_loop_iq_neg_sat_diag;", uart_c)
        self.assertIn("  PositionLoopDiag: err=%s rad | pd_out=%s rad/s | ramp_ref=%s rad/s | speed_err=%s rad/s | iq_mech=%s A | friction=%s A | iq_cmd=%s A | sat=pd:%u ramp:%u iq+:%u iq-:%u\\r\\n", uart_c)

    def test_fault_detail_reports_power_stage_runtime_state(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("uint8_t  pwmEnabled;", uart_h)
        self.assertIn("uint8_t  tim1MoeEnabled;", uart_h)
        self.assertIn('#include "tim.h"', uart_c)
        self.assertIn("packet->pwmEnabled = g_foc_app.enable_pwm;", uart_c)
        self.assertIn("packet->tim1MoeEnabled = ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;", uart_c)
        self.assertIn("  Power:   pwm=%u moe=%u mode=%u\\r\\n", uart_c)

    def test_uart_iref_keeps_user_torque_direction_in_same_frame_as_speed(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        iref_start = it_c.index('if (UART_CommandParseFloat2(cmd, "CMD:IREF,", &f1, &f2))')
        iref_end = it_c.index('if (UART_CommandParseFloat1(cmd, "CMD:SREF,", &f1))', iref_start)
        iref_section = it_c[iref_start:iref_end]

        self.assertIn("FOC_App_SetCurrentRef(&g_foc_app, f1, f2);", iref_section)
        self.assertNotIn("torque_dir", iref_section)
        self.assertNotIn("f2 * torque_dir", iref_section)

    def test_uart_normal_speed_reports_user_frame_not_raw_encoder_frame(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("float encoder_dir;", uart_c)
        self.assertIn("encoder_dir = (g_foc_app.motor_param.encoder_dir < 0) ? -1.0f : 1.0f;", uart_c)
        self.assertIn("packet->speed = g_foc_app.speed_mech * encoder_dir;", uart_c)
        self.assertNotIn("packet->speed = g_foc_app.speed_mech;", uart_c)

    def test_uart_pref_maps_user_position_direction_through_encoder_dir(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        pref_start = it_c.index('if (UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1))')
        pref_end = it_c.index('if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1)', pref_start)
        pref_section = it_c[pref_start:pref_end]

        self.assertNotIn("position_dir", pref_section)
        self.assertNotIn("pos_mapped", pref_section)
        self.assertIn("FOC_App_SetPositionRef(&g_foc_app, f1);", pref_section)

    def test_uart_pref_records_position_command_diagnostics(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        for field in (
            "position_pref_cmd_count_diag",
            "position_pref_raw_diag",
            "position_pref_mapped_diag",
            "position_pref_before_diag",
            "position_pref_after_diag",
            "position_pref_user_set_diag",
        ):
            self.assertIn(field, foc_h)

        pref_start = it_c.index('if (UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1))')
        pref_end = it_c.index('if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1)', pref_start)
        pref_section = it_c[pref_start:pref_end]
        self.assertIn("float pos_before = g_foc_app.pos_ref;", pref_section)
        self.assertIn("g_foc_app.position_pref_cmd_count_diag++;", pref_section)
        self.assertIn("g_foc_app.position_pref_raw_diag = f1;", pref_section)
        self.assertIn("g_foc_app.position_pref_mapped_diag = g_foc_app.pos_ref;", pref_section)
        self.assertIn("g_foc_app.position_pref_before_diag = pos_before;", pref_section)
        self.assertIn("g_foc_app.position_pref_after_diag = g_foc_app.pos_ref;", pref_section)
        self.assertIn("g_foc_app.position_pref_user_set_diag = g_foc_app.position_ref_user_set;", pref_section)

        for field in (
            "positionPrefCmdCount",
            "positionPrefRaw",
            "positionPrefMapped",
            "positionPrefBefore",
            "positionPrefAfter",
            "positionPrefUserSet",
        ):
            self.assertIn(field, uart_h)

        self.assertIn("packet->positionPrefCmdCount = g_foc_app.position_pref_cmd_count_diag;", uart_c)
        self.assertIn("packet->positionPrefAfter = g_foc_app.position_pref_after_diag;", uart_c)
        self.assertIn("  PrefDiag: count=%lu | raw=%s rad | mapped=%s rad | before=%s rad | after=%s rad | user_set=%u\\r\\n", uart_c)

    def test_position_pref_uses_sensor_angle_contract(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        set_pos_match = re.search(
            r"void\s+FOC_App_SetPositionRef\s*\(FOC_AppHandle_t\s+\*handle,\s*float\s+pos_ref\)\s*\{[\s\S]*?\n\}",
            foc_c,
        )
        self.assertIsNotNone(set_pos_match)
        set_pos_body = set_pos_match.group(0)
        self.assertIn("handle->pos_ref = FOC_App_PositionSensorToControlFrame(handle, pos_ref);", set_pos_body)
        self.assertIn(
            "float FOC_App_PositionSensorToControlFrame(const FOC_AppHandle_t *handle, float pos_ref_sensor)",
            foc_c,
        )
        self.assertIn(
            "return FOC_AngleNormalize(pos_ref_sensor * encoder_dir);",
            foc_c,
        )
        self.assertIn(
            "float FOC_App_PositionControlToSensorFrame(const FOC_AppHandle_t *handle, float pos_ref_control)",
            foc_c,
        )
        self.assertIn("if (pos_ref_sensor < 0.0f)", foc_c)
        self.assertIn("pos_ref_sensor += 2.0f * FOC_PI;", foc_c)

        pref_start = it_c.index('if (UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1))')
        pref_end = it_c.index('if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1)', pref_start)
        pref_section = it_c[pref_start:pref_end]
        self.assertIn("g_foc_app.position_pref_raw_diag = f1;", pref_section)
        self.assertIn("g_foc_app.position_pref_mapped_diag = g_foc_app.pos_ref;", pref_section)
        self.assertNotIn("g_foc_app.position_pref_mapped_diag = f1;", pref_section)

        self.assertIn(
            "packet->pos_ref = FOC_App_PositionControlToSensorFrame(&g_foc_app, g_foc_app.pos_ref);",
            uart_c,
        )

    def test_uart_pref_refreshes_position_loop_diagnostics_immediately(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        pref_start = it_c.index('if (UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1))')
        pref_end = it_c.index('if (sscanf(cmd, "CMD:IDENTIFY,%ld", &int_arg) == 1)', pref_start)
        pref_section = it_c[pref_start:pref_end]

        self.assertIn("FOC_App_SetPositionRef(&g_foc_app, f1);", pref_section)
        self.assertIn("FOC_App_PositionLoop(&g_foc_app);", pref_section)
        self.assertLess(
            pref_section.index("FOC_App_SetPositionRef(&g_foc_app, f1);"),
            pref_section.index("FOC_App_PositionLoop(&g_foc_app);"),
        )
        self.assertIn("g_foc_app.position_pref_after_diag = g_foc_app.pos_ref;", pref_section)
        self.assertLess(
            pref_section.index("FOC_App_PositionLoop(&g_foc_app);"),
            pref_section.index("g_foc_app.position_pref_after_diag = g_foc_app.pos_ref;"),
        )

    def test_uart_pref_commands_keep_fifo_order_with_mode_switch(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("static uint8_t UART_CommandIsPriority(const char *line)", it_c)
        self.assertIn("static void UART_CommandQueuePushPriority(const char *line)", it_c)
        priority_match = re.search(
            r"static\s+uint8_t\s+UART_CommandIsPriority\s*\(const\s+char\s+\*line\)\s*\{([\s\S]*?)\n\}",
            it_c,
        )
        self.assertIsNotNone(priority_match)
        priority_body = priority_match.group(1)
        self.assertIn("(void)line;", priority_body)
        self.assertIn("return 0U;", priority_body)
        self.assertNotIn("CMD:PREF", priority_body)
        self.assertNotIn("UART_CommandQueuePushPriority(line);", priority_body)

    def test_position_loop_uses_explicit_pd_damping_contract(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("typedef struct {", foc_h)
        self.assertIn("float kp;", foc_h)
        self.assertIn("float kd;", foc_h)
        self.assertIn("} FOC_PositionPD_t;", foc_h)
        self.assertIn("FOC_PositionPD_t pos_pd;", foc_h)
        self.assertIn("FOC_PositionPD_Init(&handle->pos_pd", foc_c)
        self.assertIn("FOC_PositionPD_Update(&handle->pos_pd", foc_c)
        self.assertIn("handle->speed_mech", foc_c)
        self.assertIn("FOC_AngleNormalize(pos_error)", foc_c)
        position_loop_start = foc_c.index("void FOC_App_PositionLoop")
        position_loop_end = foc_c.index("void FOC_App_ParamIdentifyLoop", position_loop_start)
        position_loop_section = foc_c[position_loop_start:position_loop_end]
        self.assertIn("float speed_mech_user_pos = handle->speed_mech * encoder_dir_f;", position_loop_section)
        self.assertIn("FOC_PositionPD_Update(&handle->pos_pd, pos_error, speed_mech_user_pos)", position_loop_section)
        self.assertNotIn("FOC_PositionPD_Update(&handle->pos_pd, pos_error, handle->speed_mech)", position_loop_section)
        self.assertNotIn("FOC_PI_Update(&handle->pi_pos", foc_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PD_POS,", &f1, &f2)', it_c)
        self.assertNotIn('CMD:PI_POS,%f,%f', it_c)

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
        self.assertIn("adcValidLowSideCount", uart_h)
        self.assertIn("adcInvalidLowSideCount", uart_h)
        self.assertIn("adcForcedLowSideCount", uart_h)
        self.assertIn("adcRawCurrentA", uart_h)
        self.assertIn("adcRawCurrentB", uart_h)
        self.assertIn("adcRawCurrentC", uart_h)
        self.assertIn("adcRawVbus", uart_h)
        self.assertIn("adcCurrentA", uart_h)
        self.assertIn("adcCurrentB", uart_h)
        self.assertIn("adcCurrentC", uart_h)
        self.assertIn("adcVbus", uart_h)
        self.assertIn("ADC_SAMPLING_TRIGGER_SOURCE_TEXT", uart_c)
        self.assertIn("[ADC Sampling]", uart_c)
        self.assertIn("packet->adcFrameSequence", uart_c)
        self.assertIn("packet->adcValidLowSideCount = g_foc_app.adc_valid_low_side_count;", uart_c)
        self.assertIn("packet->adcInvalidLowSideCount = g_foc_app.adc_invalid_low_side_count;", uart_c)
        self.assertIn("packet->adcForcedLowSideCount = g_foc_app.adc_forced_low_side_count;", uart_c)
        self.assertIn("packet->adcRawCurrentA", uart_c)
        self.assertIn("packet->adcRawVbus", uart_c)
        self.assertIn("packet->adcCurrentA", uart_c)
        self.assertIn("  Gate:    valid_low_side=%lu invalid_low_side=%lu forced_zero_win=%lu", uart_c)

    def test_uart_upload_reports_drv_comm_fault_separately(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("drvCommFaultActive", uart_h)
        self.assertIn("drvCommValidated", uart_h)
        self.assertIn("drvLastRxFrame", uart_h)
        self.assertIn("packet->drvCommFaultActive = s_drvHandle->runtime.commFaultActive;", uart_c)
        self.assertIn("packet->drvCommValidated = s_drvHandle->runtime.commValidated;", uart_c)
        self.assertIn("packet->drvLastRxFrame = s_drvHandle->runtime.lastRxFrame;", uart_c)
        self.assertIn("[DRV8350S Communication]", uart_c)
        self.assertIn("Readback INVALID", uart_c)
        self.assertIn("DRV8350S_COMM_FAULT_BIT", uart_c)

    def test_unidentified_enable_requires_explicit_stall_mode_authorization(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("uint8_t motor_identified;", foc_h)
        self.assertIn("uint8_t stall_mode_armed;", foc_h)
        self.assertIn("CMD:STALL_MODE,%ld", it_c)
        self.assertIn("g_foc_app.stall_mode_armed = 1U;", it_c)
        self.assertIn("g_foc_app.stall_mode_armed = 0U;", it_c)
        self.assertIn("handle->stall_mode_armed = 0U;", foc_c)
        self.assertIn("requires_stall_mode", foc_c)
        self.assertIn("encoder_detected = TLE5012_IsDataValid();", foc_c)
        self.assertIn("(!encoder_detected)", foc_c)
        self.assertIn("if (!handle->stall_mode_armed)", foc_c)
        self.assertIn("handle->control_mode = FOC_MODE_SPEED;", foc_c)
        self.assertNotIn("handle->control_mode = FOC_MODE_TORQUE;", foc_c)

    def test_stall_mode_uses_open_loop_spin_path_instead_of_fixed_angle_hold(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        tim1_start = foc_c.index("void FOC_App_TIM1_IRQHandler")
        tim1_end = foc_c.index("static void FOC_App_RequestDisableFromISR", tim1_start)
        tim1_section = foc_c[tim1_start:tim1_end]

        self.assertIn("uint8_t stall_open_loop_active;", foc_h)
        self.assertIn("float stall_theta_elec;", foc_h)
        self.assertIn("stall_open_loop_active", tim1_section)
        self.assertIn("stall_theta_elec", tim1_section)
        self.assertNotIn(
            "angle_deg = 0.0f;",
            tim1_section,
            "Authorized stall mode should perform open-loop spin, not fixed-angle torque hold.",
        )

    def test_uart_upload_reports_identify_and_stall_mode_state(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("motorIdentified", uart_h)
        self.assertIn("stallModeArmed", uart_h)
        self.assertIn("encoderDetected", uart_h)
        self.assertIn("packet->motorIdentified = g_foc_app.motor_identified;", uart_c)
        self.assertIn("packet->stallModeArmed = g_foc_app.stall_mode_armed;", uart_c)
        self.assertIn("packet->encoderDetected = TLE5012_IsDataValid();", uart_c)
        self.assertIn("Detected:", uart_c)
        self.assertIn("Identified:", uart_c)
        self.assertIn("StallMode:", uart_c)

    def test_usart1_uses_swap_on_current_board(self):
        usart_c = (ROOT / "Core" / "Src" / "usart.c").read_text(encoding="utf-8")
        uart_smoke_c = (ROOT / "BenchTests" / "UART_Smoke_Test" / "main.c").read_text(encoding="utf-8")
        uart_loopback_c = (ROOT / "BenchTests" / "UART_Loopback_Test" / "main.c").read_text(encoding="utf-8")

        for content in (usart_c, uart_smoke_c, uart_loopback_c):
            self.assertIn("AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;", content)
            self.assertIn("AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;", content)
            self.assertNotIn("AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;", content)

    def test_uart_normal_formatter_avoids_float_printf_in_normal_path(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        start = uart_c.rfind("static int16_t DrvUart_FormatNormal")
        end = uart_c.find("static int16_t DrvUart_FormatFault", start)
        self.assertNotEqual(start, -1)
        self.assertNotEqual(end, -1)
        normal_section = uart_c[start:end]

        self.assertIn("static void DrvUart_FormatFixed", uart_c)
        self.assertIn("DrvUart_FormatFixed(angleText", normal_section)
        self.assertIn("DrvUart_FormatFixed(speedText", normal_section)
        self.assertNotIn("%f", normal_section)

    def test_uart_upload_uses_blocking_tx_path_for_host_stream(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        start = uart_c.rfind("static bool DrvUart_StartSend")
        end = uart_c.index("static int16_t DrvUart_Append", start)
        send_section = uart_c[start:end]

        self.assertIn("HAL_UART_Transmit(s_huart, s_txBuf, len, 100)", send_section)
        self.assertNotIn("HAL_UART_Transmit_IT", send_section)
        self.assertNotIn("HAL_UART_Transmit_DMA", send_section)
        self.assertIn("HAL_UART_AbortTransmit(s_huart)", uart_c)

    def test_protection_thresholds_are_runtime_configurable(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("typedef struct {", foc_h)
        self.assertIn("FOC_ProtectionConfig_t", foc_h)
        self.assertIn("overcurrent_limit_a", foc_h)
        self.assertIn("overvoltage_limit_v", foc_h)
        self.assertIn("undervoltage_limit_v", foc_h)
        self.assertIn("FOC_DEFAULT_OVERCURRENT_LIMIT_A", foc_h)
        self.assertIn("FOC_DEFAULT_OVERVOLTAGE_LIMIT_V", foc_h)
        self.assertIn("FOC_DEFAULT_UNDERVOLTAGE_LIMIT_V", foc_h)
        self.assertIn("#define FOC_DEFAULT_OVERVOLTAGE_LIMIT_V   16.0f", foc_h)
        self.assertIn("#define FOC_DEFAULT_UNDERVOLTAGE_LIMIT_V  9.0f", foc_h)
        self.assertNotIn("#define FOC_DEFAULT_UNDERVOLTAGE_LIMIT_V  18.0f", foc_h)
        self.assertIn("FOC_ProtectionConfig_t protection;", foc_h)
        self.assertIn("void FOC_App_SetVoltageThresholds(FOC_AppHandle_t *handle, float undervoltage, float overvoltage);", foc_h)
        self.assertIn("handle->protection.overcurrent_limit_a", foc_c)
        self.assertIn("handle->protection.overvoltage_limit_v", foc_c)
        self.assertIn("handle->protection.undervoltage_limit_v", foc_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:VBUS_LIMIT,", &f1, &f2)', it_c)
        self.assertIn("FOC_App_SetVoltageThresholds(&g_foc_app, f1, f2);", it_c)
        self.assertNotIn("g_foc_app.Vbus >= FOC_UNDERVOLTAGE_THRESH", it_c)
        self.assertNotIn("g_foc_app.Vbus <= FOC_OVERVOLTAGE_THRESH", it_c)

    def test_uart_float_commands_use_bounded_parser_instead_of_scanf_float(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("static uint8_t UART_CommandParseFloatToken", it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:VBUS_LIMIT,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:IREF,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat1(cmd, "CMD:SREF,", &f1)', it_c)
        self.assertIn('UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PI_CURRENT,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PI_SPEED,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PD_POS,", &f1, &f2)', it_c)
        self.assertNotIn('%f', it_c[it_c.index("static void UART_CommandExecute"):it_c.index("void UART_Command_ProcessPending")])

    def test_host_command_builder_covers_firmware_debug_commands(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")
        parser_py = (ROOT / "HostComputer" / "data_parser.py").read_text(encoding="utf-8")

        command_contracts = {
            "CMD:FAULT_DETAIL": "def fault_detail(",
            "CMD:ADC_NOISE,%ld": "def adc_noise_test(",
            "CMD:ADC_PHASE_SCAN,%ld": "def adc_phase_scan(",
            "CMD:ADC_SECTOR_SCAN,%ld": "def adc_sector_scan(",
            "CMD:TLE_GPIO_DIAG,%ld": "def tle_gpio_diag(",
            "CMD:TLE_RAW": "def tle_raw(",
            'UART_CommandParseFloat2(cmd, "CMD:PI_SPEED,", &f1, &f2)': "def set_speed_pi(",
            'UART_CommandParseFloat2(cmd, "CMD:PD_POS,", &f1, &f2)': "def set_position_pd(",
        }
        for firmware_cmd, host_builder in command_contracts.items():
            self.assertIn(firmware_cmd, it_c)
            self.assertIn(host_builder, parser_py)

        self.assertNotIn("CMD:PI_POS", parser_py)

    def test_keil_project_uses_standard_scanf_without_invalid_linker_misc(self):
        uvprojx = (ROOT / "MDK-ARM" / "24V FOC Controller.uvprojx").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:VBUS_LIMIT,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:IREF,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat1(cmd, "CMD:SREF,", &f1)', it_c)
        self.assertIn('UART_CommandParseFloat1(cmd, "CMD:PREF,", &f1)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PI_CURRENT,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PI_SPEED,", &f1, &f2)', it_c)
        self.assertIn('UART_CommandParseFloat2(cmd, "CMD:PD_POS,", &f1, &f2)', it_c)
        self.assertIn("sscanf(cmd,", it_c)
        self.assertNotIn("_sscanf(", it_c)
        self.assertNotIn(
            "--scanf_support=",
            uvprojx,
            "Current Keil ARMCC5 linker rejects --scanf_support in misc options with L3900U, so the project must rely on standard sscanf() instead of an invalid linker flag.",
        )

    def test_foc_firmware_has_enough_stack_for_isr_and_fault_formatting(self):
        startup = (ROOT / "MDK-ARM" / "startup_stm32h743xx.s").read_text(encoding="utf-8")
        ld_script = (ROOT / "STM32H743VITX_FLASH.ld").read_text(encoding="utf-8")

        self.assertIn("Stack_Size\t\tEQU     0x2000", startup)
        self.assertIn("_Min_Stack_Size = 0x2000", ld_script)

    def test_precheck_refreshes_live_telemetry_before_enable_and_clear_fault(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("void FOC_App_RefreshTelemetry(FOC_AppHandle_t *handle);", foc_h)
        self.assertIn("void FOC_App_RefreshTelemetry(FOC_AppHandle_t *handle)", foc_c)
        self.assertIn("ADC_Sampling_GetData();", foc_c)
        self.assertIn("handle->Vbus = adc->vbus;", foc_c)
        self.assertIn("handle->Ia = adc->currentA;", foc_c)
        self.assertIn("handle->Ib = adc->currentB;", foc_c)
        self.assertIn("handle->Ic = adc->currentC;", foc_c)
        self.assertIn("FOC_App_RefreshTelemetry(handle);", foc_c)
        self.assertIn("static uint8_t FOC_App_PrecheckPowerStage", foc_c)
        self.assertIn("FOC_App_PrecheckPowerStage(handle, &fault)", foc_c)
        self.assertIn("FOC_App_RefreshTelemetry(&g_foc_app);", it_c)
        self.assertNotIn("handle->Vbus = 24.0f;", foc_c)

    def test_clear_fault_clears_deferred_isr_disable_before_reenable(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        clear_start = it_c.index('if (strcmp(cmd, "CMD:CLEAR_FAULT") == 0) {')
        clear_end = it_c.index('if (UART_CommandParseFloat2(cmd, "CMD:PI_CURRENT,", &f1, &f2))', clear_start)
        clear_section = it_c[clear_start:clear_end]

        self.assertIn("g_foc_app.pending_disable = 0U;", clear_section)
        self.assertLess(
            clear_section.index("g_foc_app.pending_disable = 0U;"),
            clear_section.index("FOC_App_ResetMotionState(&g_foc_app);"),
            "CLEAR_FAULT must cancel any deferred ISR shutdown before the next ENABLE can be armed.",
        )

    def test_vbus_limit_update_re_evaluates_voltage_fault_without_manual_clear(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("static void UART_ReevaluateVoltageFaultAfterThresholdUpdate(void)", it_c)
        self.assertIn("FOC_FAULT_UNDERVOLTAGE", it_c)
        self.assertIn("FOC_FAULT_OVERVOLTAGE", it_c)
        self.assertIn("FOC_App_SetVoltageThresholds(&g_foc_app, f1, f2);", it_c)
        self.assertIn("UART_ReevaluateVoltageFaultAfterThresholdUpdate();", it_c)
        self.assertIn("FOC_App_RefreshTelemetry(&g_foc_app);", it_c)
        self.assertIn("FOC_App_ResetMotionState(&g_foc_app);", it_c)

    def test_voltage_protection_splits_warning_from_severe_trip_and_auto_recovers(self):
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("#define FOC_WARNING_VBUS_UNDERVOLTAGE_BIT", foc_h)
        self.assertIn("#define FOC_WARNING_VBUS_OVERVOLTAGE_BIT", foc_h)
        self.assertIn("#define FOC_VOLTAGE_SEVERE_TRIP_MARGIN_V", foc_h)
        self.assertIn("#define FOC_VOLTAGE_FAULT_RECOVER_HYSTERESIS_V", foc_h)
        self.assertIn("uint32_t warning_flags;", foc_h)
        self.assertIn("uint32_t FOC_App_GetVoltageWarningFlags(const FOC_AppHandle_t *handle);", foc_h)
        self.assertIn("uint8_t FOC_App_GetVoltageTripFault(const FOC_AppHandle_t *handle, FOC_FaultCode_t *fault);", foc_h)
        self.assertIn("uint8_t FOC_App_IsVoltageFaultRecovered(const FOC_AppHandle_t *handle, FOC_FaultCode_t fault);", foc_h)
        self.assertIn("handle->warning_flags = FOC_App_GetVoltageWarningFlags(handle);", foc_c)
        self.assertIn("uint32_t FOC_App_GetVoltageWarningFlags(const FOC_AppHandle_t *handle)", foc_c)
        self.assertIn("uint8_t FOC_App_GetVoltageTripFault(const FOC_AppHandle_t *handle, FOC_FaultCode_t *fault)", foc_c)
        self.assertIn("uint8_t FOC_App_IsVoltageFaultRecovered(const FOC_AppHandle_t *handle, FOC_FaultCode_t fault)", foc_c)
        self.assertIn("FOC_App_GetVoltageTripFault(handle, &voltageFault)", foc_c)
        self.assertIn("FOC_App_IsVoltageFaultRecovered(handle, handle->fault_code)", foc_c)
        self.assertNotIn(
            "else if (handle->Vbus > handle->protection.overvoltage_limit_v) {\n                FOC_App_Disable(handle);\n                FOC_App_EnterFault(handle, FOC_FAULT_OVERVOLTAGE);",
            foc_c,
        )
        self.assertNotIn(
            "else if (handle->Vbus < handle->protection.undervoltage_limit_v) {\n                FOC_App_Disable(handle);\n                FOC_App_EnterFault(handle, FOC_FAULT_UNDERVOLTAGE);",
            foc_c,
        )
        self.assertIn("FOC_App_GetVoltageTripFault(&g_foc_app, &voltageFault)", it_c)
        self.assertIn("g_foc_app.warning_flags = FOC_App_GetVoltageWarningFlags(&g_foc_app);", it_c)

    def test_uart_and_host_surface_voltage_warning_state_separately_from_faults(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        parser_py = (ROOT / "HostComputer" / "data_parser.py").read_text(encoding="utf-8")
        gui_py = (ROOT / "HostComputer" / "gui_logic.py").read_text(encoding="utf-8")

        self.assertIn("appWarningFlags", uart_h)
        self.assertIn("packet->appWarningFlags = g_foc_app.warning_flags;", uart_c)
        self.assertIn("app_warning_flags", parser_py)
        self.assertIn("APP_WARNING_LABELS", gui_py)
        self.assertIn("application_warning_text", gui_py)
        self.assertIn("packet.app_warning_flags", gui_py)
        self.assertIn("告警激活", gui_py)

    def test_uart_smoke_test_project_contract(self):
        bench_dir = ROOT / "BenchTests" / "UART_Smoke_Test"
        main_c_path = bench_dir / "main.c"
        build_ps1_path = bench_dir / "build.ps1"

        self.assertTrue(main_c_path.exists(), "UART smoke bench main.c should exist")
        self.assertTrue(build_ps1_path.exists(), "UART smoke bench build.ps1 should exist")

        main_c = main_c_path.read_text(encoding="utf-8")
        build_ps1 = build_ps1_path.read_text(encoding="utf-8")

        self.assertIn("GPIO_PIN_14|GPIO_PIN_15", main_c)
        self.assertIn("GPIO_AF4_USART1", main_c)
        self.assertIn("huart1.Instance = USART1", main_c)
        self.assertIn("huart1.Init.BaudRate = 230400", main_c)
        self.assertIn("UART_SMOKE SWAP HSE OK", main_c)
        self.assertIn("stm32h7xx_hal_uart.c", build_ps1)
        self.assertIn("stm32h7xx_hal_uart_ex.c", build_ps1)

    def test_uart_loopback_test_project_contract(self):
        bench_dir = ROOT / "BenchTests" / "UART_Loopback_Test"
        main_c_path = bench_dir / "main.c"
        build_ps1_path = bench_dir / "build.ps1"

        self.assertTrue(main_c_path.exists(), "UART loopback bench main.c should exist")
        self.assertTrue(build_ps1_path.exists(), "UART loopback bench build.ps1 should exist")

        main_c = main_c_path.read_text(encoding="utf-8")
        build_ps1 = build_ps1_path.read_text(encoding="utf-8")

        self.assertIn("GPIO_PIN_14|GPIO_PIN_15", main_c)
        self.assertIn("GPIO_AF4_USART1", main_c)
        self.assertIn("huart1.Init.BaudRate = 230400", main_c)
        self.assertIn("HAL_UART_Transmit", main_c)
        self.assertIn("HAL_UART_Receive", main_c)
        self.assertIn("0x55U", main_c)
        self.assertIn("s_loopbackLastTx", main_c)
        self.assertIn("s_loopbackLastRx", main_c)
        self.assertIn("s_loopbackLastIsrBeforeTx", main_c)
        self.assertIn("s_loopbackLastIsrAfterRx", main_c)
        self.assertIn("stm32h7xx_hal_uart.c", build_ps1)
        self.assertIn("stm32h7xx_hal_uart_ex.c", build_ps1)

    def test_motor_identification_requires_valid_encoder_feedback(self):
        mi_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        mi_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("MI_ERR_ENCODER_INVALID", mi_h)
        self.assertIn('case MI_ERR_ENCODER_INVALID:    return "Encoder Invalid";', mi_c)
        self.assertIn("#define MI_ENCODER_INVALID_CONSECUTIVE_LIMIT", mi_h)
        self.assertIn("uint8_t encoder_invalid_count;", mi_h)
        self.assertIn("static MI_ErrorCode_t MI_RequireValidEncoder(MI_Handle_t *handle)", mi_c)
        self.assertGreaterEqual(mi_c.count("MI_RequireValidEncoder(handle)"), 3)
        self.assertIn("handle->encoder_invalid_count = 0U;", mi_c)
        self.assertIn("handle->state_start_time = HAL_GetTick();\n    handle->encoder_invalid_count = 0U;", mi_c)
        self.assertIn("handle->encoder_invalid_count++", mi_c)
        self.assertIn("MI_ENCODER_INVALID_CONSECUTIVE_LIMIT", mi_c)
        self.assertIn("return MI_ERR_IN_PROGRESS;", mi_c)
        self.assertIn("if (encoder_status != MI_ERR_NONE) {", mi_c)
        self.assertIn("return encoder_status;", mi_c)
        self.assertIn("mi_error == MI_ERR_ENCODER_INVALID", foc_c)
        self.assertIn("handle->fault_code = FOC_FAULT_ENCODER;", foc_c)

    def test_tle5012_preserves_reset_watchdog_status_through_uart_fault_upload(self):
        tle_h = (ROOT / "MDK-ARM" / "code" / "tle5012.h").read_text(encoding="utf-8")
        tle_c = (ROOT / "MDK-ARM" / "code" / "tle5012.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("uint8_t status;", tle_h)
        self.assertIn("uint8_t reset_fault;", tle_h)
        self.assertIn("#define TLE5012_SAFETY_RESET_OK_MASK", tle_c)
        self.assertIn("tle5012_sensor.status = (uint8_t)(safety_word >> 8);", tle_c)
        self.assertIn("tle5012_sensor.reset_fault", tle_c)
        self.assertIn("TLE5012_SAFETY_RESET_OK_MASK", tle_c)
        self.assertIn("encoderSafetyStatus", uart_h)
        self.assertIn("encoderResetFault", uart_h)
        self.assertIn("packet->encoderSafetyStatus = tle5012_sensor.status;", uart_c)
        self.assertIn("packet->encoderResetFault = tle5012_sensor.reset_fault;", uart_c)
        self.assertIn("Reset:", uart_c)
        self.assertIn("Safety:", uart_c)

    def test_startup_primes_tim1_oc4_and_drv_bias_before_adc_zero_calibration(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        base_start = main_c.find("HAL_TIM_Base_Start(&htim1)")
        oc4_start = main_c.find("HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4)")
        adc_cal = main_c.find("ADC_Sampling_Calibrate(200)")
        drv_en_high = main_c.find("HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);")
        drv_configure = main_c.find("DRV8350S_Configure(&drv8350s, &config)")
        clear_uif = main_c.find("__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE)")
        irq_enable = main_c.find("__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE)")

        self.assertNotEqual(base_start, -1, "TIM1 base should be started before ADC zero calibration")
        self.assertNotEqual(oc4_start, -1, "TIM1 CH4/OC4REF should be started before ADC zero calibration")
        self.assertNotEqual(adc_cal, -1, "main.c should still perform ADC zero calibration")
        self.assertNotEqual(drv_en_high, -1, "DRV_EN should be asserted before ADC zero calibration")
        self.assertNotEqual(drv_configure, -1, "DRV8350S should be configured before ADC zero calibration")
        self.assertNotEqual(clear_uif, -1, "main.c should clear any pending TIM1 update flag before enabling interrupts")
        self.assertNotEqual(irq_enable, -1, "TIM1 update interrupt must still be enabled for control loop")
        self.assertLess(base_start, adc_cal)
        self.assertLess(oc4_start, adc_cal)
        self.assertLess(drv_en_high, adc_cal)
        self.assertLess(drv_configure, adc_cal)
        self.assertLess(base_start, irq_enable)

    def test_enable_and_identify_prime_neutral_pwm_before_starting_outputs(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("static void FOC_App_PrimeNeutralPwm", foc_c)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, neutral);", foc_c)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, neutral);", foc_c)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, neutral);", foc_c)

        enable_start = foc_c.index("void FOC_App_Enable(FOC_AppHandle_t *handle)")
        enable_end = foc_c.index("void FOC_App_Disable(FOC_AppHandle_t *handle)", enable_start)
        enable_section = foc_c[enable_start:enable_end]
        self.assertLess(
            enable_section.index("FOC_App_PrimeNeutralPwm();"),
            enable_section.index("HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);"),
            "Enable path must preload neutral duty before PWM outputs become active.",
        )

        identify_start = foc_c.index("void FOC_App_StartIdentify(FOC_AppHandle_t *handle)")
        identify_end = foc_c.index("void FOC_App_StopIdentify(FOC_AppHandle_t *handle)", identify_start)
        identify_section = foc_c[identify_start:identify_end]
        self.assertLess(
            identify_section.index("FOC_App_PrimeNeutralPwm();"),
            identify_section.index("HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);"),
            "Identify path must preload neutral duty before PWM outputs become active.",
        )

    def test_identify_does_not_rewrite_drv_ocp_threshold_runtime(self):
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")
        foc_h = (ROOT / "MDK-ARM" / "code" / "foc_app.h").read_text(encoding="utf-8")

        self.assertNotIn("FOC_DRV_IDENTIFY_VDS_LEVEL", foc_h)
        self.assertNotIn("static void FOC_App_SetDrvVdsLevel(uint8_t vds_level)", foc_c)
        self.assertNotIn("DRV8350S_REG_OCP_CTRL", foc_c[foc_c.index("void FOC_App_StartIdentify(FOC_AppHandle_t *handle)"):foc_c.index("void FOC_App_StopIdentify(FOC_AppHandle_t *handle)")])
        self.assertNotIn("DRV8350S_REG_OCP_CTRL", foc_c[foc_c.index("void FOC_App_StopIdentify(FOC_AppHandle_t *handle)"):foc_c.index("uint8_t FOC_App_IsIdentifyComplete(FOC_AppHandle_t *handle)")])

    def test_uart_fault_packet_buffer_covers_worst_case_fault_dump(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")

        match = re.search(r"#define\s+DRV_UART_BUF_SIZE\s+(\d+)", uart_h)
        self.assertIsNotNone(match, "uart_upload.h should define DRV_UART_BUF_SIZE")
        buf_size = int(match.group(1))

        fs1 = 0x07FF
        vs2 = 0x07FF
        lines = []
        add = lines.append

        add("\r\n========== !!! FAULT DETECTED !!! ==========\r\n")
        add(f"Time: {3000} ms\r\n\r\n")
        add("[TLE5012 Encoder]\r\n")
        add(f"  Angle:  {0.0:7.2f} deg\r\n")
        add(f"  Raw:    {0:5d} (0x{0:04X})\r\n")
        add("  CRC:    ERROR!\r\n\r\n")
        add("[DRV8350S Fault Details]\r\n")
        add(f"  FAULT1: 0x{fs1:04X} | VGS2: 0x{vs2:04X}\r\n\r\n")

        if fs1 & (1 << 10):
            add("  [FAULT] General Fault\r\n")
        if fs1 & (1 << 9):
            add("  [CRIT]  VDS Overcurrent!\r\n")
        if fs1 & (1 << 8):
            add("  [CRIT]  Gate Drive Fault!\r\n")
        if fs1 & (1 << 7):
            add("  [CRIT]  Undervoltage Lockout!\r\n")
        if fs1 & (1 << 6):
            add("  [CRIT]  Overtemperature Shutdown!\r\n")
        if fs1 & 0x003F:
            add("\r\n  VDS OCP Phase:\r\n")
            if fs1 & (1 << 5):
                add("    - A High-Side\r\n")
            if fs1 & (1 << 4):
                add("    - A Low-Side\r\n")
            if fs1 & (1 << 3):
                add("    - B High-Side\r\n")
            if fs1 & (1 << 2):
                add("    - B Low-Side\r\n")
            if fs1 & (1 << 1):
                add("    - C High-Side\r\n")
            if fs1 & (1 << 0):
                add("    - C Low-Side\r\n")
        if vs2 & (1 << 7):
            add("  [WARN]  Overtemperature Warning\r\n")
        if vs2 & (1 << 6):
            add("  [WARN]  Gate Drive UVLO\r\n")
        if vs2 & 0x003F:
            add("\r\n  VGS Fault Phase:\r\n")
            if vs2 & (1 << 5):
                add("    - A High-Side\r\n")
            if vs2 & (1 << 4):
                add("    - A Low-Side\r\n")
            if vs2 & (1 << 3):
                add("    - B High-Side\r\n")
            if vs2 & (1 << 2):
                add("    - B Low-Side\r\n")
            if vs2 & (1 << 1):
                add("    - C High-Side\r\n")
            if vs2 & (1 << 0):
                add("    - C Low-Side\r\n")

        add("\r\n[ADC Sampling]\r\n")
        add("  Trigger: TIM1_TRGO2_OC4REF\r\n")
        add(f"  Samp:    Iabc={32.5:4.1f} cyc | Vbus={16.5:4.1f} cyc\r\n")
        add(f"  Frame:   seq={0} age={0} miss={0} invalid={0}\r\n")
        add(f"  Raw:     A={0:4d} B={0:4d} C={0:4d}\r\n")
        add(f"  Curr:    Ia={0.0:7.3f} Ib={0.0:7.3f} Ic={0.0:7.3f} | Vbus={24.0:7.2f} V\r\n")
        add("\r\n>>> ACTION REQUIRED <<<\r\n")
        add("  1. Disable PWM immediately\r\n")
        add("  2. Check power supply\r\n")
        add("  3. Verify MOSFETs status\r\n")
        add("=============================================\r\n")

        worst_case_len = len("".join(lines).encode("utf-8"))
        self.assertGreaterEqual(
            buf_size,
            worst_case_len,
            f"DRV_UART_BUF_SIZE={buf_size} must cover worst-case fault dump length {worst_case_len}",
        )

    def test_uart_runtime_dq_telemetry_reports_control_loop_and_adc_dq_separately(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("float    adcId;", uart_h)
        self.assertIn("float    adcIq;", uart_h)
        self.assertIn("float    loopCurrentA;", uart_h)
        self.assertIn("int16_t  adcDeltaA;", uart_h)
        self.assertIn("FOC_ABC_t adcTelemetryIabc", uart_c)
        self.assertIn("FOC_Clarke_Transform(&adcTelemetryIabc, &adcTelemetryAlphaBeta);", uart_c)
        self.assertIn(
            "FOC_Park_Transform(&adcTelemetryAlphaBeta, g_foc_app.foc.sin_theta, g_foc_app.foc.cos_theta, &adcTelemetryIdq);",
            uart_c,
        )
        self.assertIn("packet->Id = g_foc_app.foc.Idq.d;", uart_c)
        self.assertIn("packet->Iq = g_foc_app.foc.Idq.q;", uart_c)
        self.assertIn("packet->adcId = adcTelemetryIdq.d;", uart_c)
        self.assertIn("packet->adcIq = adcTelemetryIdq.q;", uart_c)
        self.assertIn("packet->loopCurrentA = g_foc_app.Ia;", uart_c)
        self.assertIn("packet->adcDeltaA = (int16_t)((int32_t)adc->rawCurrentA - (int32_t)adc->offsetA);", uart_c)
        self.assertIn("  AdcDQ:   Id=%s A | Iq=%s A", uart_c)
        self.assertIn("  LoopABC: Ia=%ld mA Ib=%ld mA Ic=%ld mA", uart_c)
        self.assertIn("  Delta:   A=%+d B=%+d C=%+d LSB", uart_c)

    def test_normal_uart_telemetry_uploads_at_50hz_for_current_plots(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")

        self.assertIn("#define DRV_UPLOAD_INTERVAL_MS      20", uart_h)
        self.assertIn("normal telemetry interval is 20ms / 50Hz", readme)

    def test_uart_uploads_high_rate_phase_current_frame_for_sine_plots(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")

        self.assertIn("#define DRV_PHASE_CURRENT_UPLOAD_INTERVAL_MS 5", uart_h)
        self.assertIn("static uint32_t s_lastPhaseCurrentUploadTime = 0;", uart_c)
        self.assertIn("DrvUart_FormatPhaseCurrent", uart_c)
        self.assertIn('"C,%lu,%s,%s,%s\\n"', uart_c)
        self.assertIn("(currentTime - s_lastPhaseCurrentUploadTime) >= DRV_PHASE_CURRENT_UPLOAD_INTERVAL_MS", uart_c)
        self.assertIn("phase current telemetry interval is 5ms / 200Hz", readme)

    def test_uart_fault_detail_reports_adc_phase_offsets(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("adcCalibStatus", uart_h)
        self.assertIn("adcOffsetA", uart_h)
        self.assertIn("adcOffsetB", uart_h)
        self.assertIn("adcOffsetC", uart_h)
        self.assertIn("packet->adcCalibStatus = (uint8_t)adc->calibStatus;", uart_c)
        self.assertIn("packet->adcOffsetA = adc->offsetA;", uart_c)
        self.assertIn("packet->adcOffsetB = adc->offsetB;", uart_c)
        self.assertIn("packet->adcOffsetC = adc->offsetC;", uart_c)
        self.assertIn("  Offset:  A=%4d B=%4d C=%4d\\r\\n", uart_c)
        self.assertIn("  Raw:     A=%4u B=%4u C=%4u\\r\\n", uart_c)
        self.assertIn("  Curr:    Ia=%ld mA Ib=%ld mA Ic=%ld mA\\r\\n", uart_c)
        self.assertIn("  Samp:    Iabc=32.5 cyc | Vbus=16.5 cyc\\r\\n", uart_c)

    def test_uart_fault_detail_reports_pwm_sampling_window_diagnostics(self):
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")
        adc_c = (ROOT / "MDK-ARM" / "code" / "adc_sampling.c").read_text(encoding="utf-8")

        self.assertIn("uint8_t  svpwmSector;", uart_h)
        self.assertIn("float    svpwmTa;", uart_h)
        self.assertIn("float    svpwmTb;", uart_h)
        self.assertIn("float    svpwmTc;", uart_h)
        self.assertIn("uint16_t adcPwmPeriod;", uart_h)
        self.assertIn("uint16_t adcPwmCompareA;", uart_h)
        self.assertIn("uint16_t adcPwmCompareB;", uart_h)
        self.assertIn("uint16_t adcPwmCompareC;", uart_h)
        self.assertIn("uint16_t adcTriggerCompare;", uart_h)
        self.assertIn("uint16_t adcTimerCount;", uart_h)
        self.assertIn("uint8_t  adcTimerCountingDown;", uart_h)
        self.assertIn("uint8_t  adcLowSideValidA;", uart_h)
        self.assertIn("uint8_t  adcLowSideValidB;", uart_h)
        self.assertIn("uint8_t  adcLowSideValidC;", uart_h)

        self.assertIn("volatile uint16_t pwmPeriod;", adc_h)
        self.assertIn("volatile uint16_t pwmCompareA;", adc_h)
        self.assertIn("volatile uint16_t pwmCompareB;", adc_h)
        self.assertIn("volatile uint16_t pwmCompareC;", adc_h)
        self.assertIn("volatile uint16_t triggerCompare;", adc_h)
        self.assertIn("volatile uint16_t timerCount;", adc_h)
        self.assertIn("volatile uint8_t timerCountingDown;", adc_h)
        self.assertIn("volatile uint8_t lowSideValidA;", adc_h)
        self.assertIn("volatile uint8_t lowSideValidB;", adc_h)
        self.assertIn("volatile uint8_t lowSideValidC;", adc_h)

        self.assertIn('#include "tim.h"', adc_c)
        self.assertIn("s_adcData.pwmPeriod = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);", adc_c)
        self.assertIn("s_adcData.pwmCompareA = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);", adc_c)
        self.assertIn("s_adcData.pwmCompareB = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);", adc_c)
        self.assertIn("s_adcData.pwmCompareC = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);", adc_c)
        self.assertIn("s_adcData.triggerCompare = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);", adc_c)
        self.assertIn("s_adcData.timerCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);", adc_c)
        self.assertIn("s_adcData.timerCountingDown = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1) != RESET) ? 1U : 0U;", adc_c)
        self.assertIn("s_adcData.lowSideValidA = (s_adcData.pwmCompareA <= s_adcData.triggerCompare) ? 1U : 0U;", adc_c)
        self.assertIn("s_adcData.lowSideValidB = (s_adcData.pwmCompareB <= s_adcData.triggerCompare) ? 1U : 0U;", adc_c)
        self.assertIn("s_adcData.lowSideValidC = (s_adcData.pwmCompareC <= s_adcData.triggerCompare) ? 1U : 0U;", adc_c)

        self.assertIn("packet->svpwmSector = g_foc_app.foc.svpwm.sector;", uart_c)
        self.assertIn("packet->svpwmTa = g_foc_app.foc.svpwm.Ta;", uart_c)
        self.assertIn("packet->svpwmTb = g_foc_app.foc.svpwm.Tb;", uart_c)
        self.assertIn("packet->svpwmTc = g_foc_app.foc.svpwm.Tc;", uart_c)
        self.assertIn("packet->adcPwmPeriod = adc->pwmPeriod;", uart_c)
        self.assertIn("packet->adcPwmCompareA = adc->pwmCompareA;", uart_c)
        self.assertIn("packet->adcPwmCompareB = adc->pwmCompareB;", uart_c)
        self.assertIn("packet->adcPwmCompareC = adc->pwmCompareC;", uart_c)
        self.assertIn("packet->adcTriggerCompare = adc->triggerCompare;", uart_c)
        self.assertIn("packet->adcTimerCount = adc->timerCount;", uart_c)
        self.assertIn("packet->adcTimerCountingDown = adc->timerCountingDown;", uart_c)
        self.assertIn("packet->adcLowSideValidA = adc->lowSideValidA;", uart_c)
        self.assertIn("packet->adcLowSideValidB = adc->lowSideValidB;", uart_c)
        self.assertIn("packet->adcLowSideValidC = adc->lowSideValidC;", uart_c)
        self.assertIn("  PWM:     ARR=%u | CCR A/B/C=%u/%u/%u | TRIG=%u | CNT=%u | DIR=%s\\r\\n", uart_c)
        self.assertIn("  Win:     low_side_valid A=%u B=%u C=%u | sector=%u | Ta/Tb/Tc=%s/%s/%s\\r\\n", uart_c)

    def test_compact_uart_frames_append_adc_calibration_status_and_offsets(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn('%s,%s,%u,%d,%d,%d\\n"', uart_c)
        self.assertIn("(unsigned)packet->adcCalibStatus", uart_c)
        self.assertIn("(int)packet->adcOffsetA", uart_c)
        self.assertIn("(int)packet->adcOffsetB", uart_c)
        self.assertIn("(int)packet->adcOffsetC", uart_c)

    def test_startup_fails_closed_when_adc_zero_calibration_times_out(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        self.assertIn("if (ADC_Sampling_Calibrate(200) != 0)", main_c)
        self.assertIn("Main_RecordStartupFault(DRV8350S_FW_ADC_CAL_FAULT_BIT, FOC_FAULT_ADC_SAMPLING);", main_c)
        self.assertIn("startupControlReady = 0U;", main_c)

    def test_uart_fault_formatter_avoids_float_printf_in_fault_path(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        fault_start = uart_c.index("static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)\n{")
        fault_end = uart_c.index("static bool DrvUart_StartSend", fault_start)
        fault_section = uart_c[fault_start:fault_end]

        self.assertNotIn("%7.2f", fault_section)
        self.assertNotIn("%7.3f", fault_section)
        self.assertNotIn("%4.1f", fault_section)

    def test_fault_detail_command_uses_blocking_verbose_snapshot(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn('if (strcmp(cmd, "CMD:FAULT_DETAIL") == 0) {', it_c)
        self.assertIn("DrvUart_UploadImmediate();", it_c)
        self.assertNotIn("FAULT_DETAIL,BUSY", it_c)

    def test_adc_phase_scan_command_sweeps_trigger_without_scope(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("#define UART_ADC_PHASE_SCAN_MIN_TRIGGER 2U", it_c)
        self.assertIn("#define UART_ADC_PHASE_SCAN_MAX_TRIGGER 46U", it_c)
        self.assertIn("#define UART_ADC_PHASE_SCAN_TRIGGER_STEP 2U", it_c)
        self.assertIn("static void UART_CommandHandleAdcPhaseScan(uint16_t requestedSamples)", it_c)
        self.assertIn("UART_CommandCanRunAdcPhaseScan()", it_c)
        self.assertIn("if (sscanf(cmd, \"CMD:ADC_PHASE_SCAN,%ld\", &int_arg) == 1)", it_c)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger);", it_c)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, originalTrigger);", it_c)
        self.assertIn('"ADC_PHASE_SCAN,START,n=%u,start=%u,end=%u,step=%u\\r\\n"', it_c)
        self.assertIn('"ADC_PHASE_SCAN,POINT,trig=%u,n=%u,A:min=%u,max=%u,mean=%u,B:min=%u,max=%u,mean=%u,C:min=%u,max=%u,mean=%u,win=%u/%u/%u\\r\\n"', it_c)
        self.assertIn('"ADC_PHASE_SCAN,DONE,restore=%u\\r\\n"', it_c)

    def test_adc_sector_scan_command_buckets_runtime_samples_without_moving_trigger(self):
        it_c = (ROOT / "Core" / "Src" / "stm32h7xx_it.c").read_text(encoding="utf-8")

        self.assertIn("#define UART_ADC_SECTOR_SCAN_SECTORS 6U", it_c)
        self.assertIn("#define UART_ADC_SECTOR_SCAN_MIN_PHASES 3U", it_c)
        self.assertIn("typedef struct {\n    UART_AdcNoiseAccum_t accA;", it_c)
        self.assertIn("static void UART_CommandHandleAdcSectorScan(uint16_t requestedSamples)", it_c)
        self.assertIn("UART_CommandCanRunAdcPhaseScan()", it_c)
        self.assertIn("if (sscanf(cmd, \"CMD:ADC_SECTOR_SCAN,%ld\", &int_arg) == 1)", it_c)
        self.assertIn("UART_CommandAdcSectorMinPhase(adc->pwmCompareA, adc->pwmCompareB, adc->pwmCompareC)", it_c)
        self.assertIn('"ADC_SECTOR_SCAN,START,n=%u\\r\\n"', it_c)
        self.assertIn('"ADC_SECTOR_SCAN,BUCKET,sector=%u,min=%c,count=%u,A:min=%u,max=%u,mean=%u,B:min=%u,max=%u,mean=%u,C:min=%u,max=%u,mean=%u,CCR:%u/%u/%u,win:%u/%u/%u\\r\\n"', it_c)
        self.assertIn('"ADC_SECTOR_SCAN,DONE,captured=%u\\r\\n"', it_c)
        sector_start = it_c.index("static void UART_CommandHandleAdcSectorScan")
        sector_end = it_c.index("static uint16_t UART_CommandCapturePhaseScanPoint", sector_start)
        sector_section = it_c[sector_start:sector_end]
        self.assertNotIn("__HAL_TIM_SET_COMPARE", sector_section)

    def test_uart_normal_path_uses_compact_frames_at_230400(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        uart_h = (ROOT / "MDK-ARM" / "code" / "uart_upload.h").read_text(encoding="utf-8")
        usart_c = (ROOT / "Core" / "Src" / "usart.c").read_text(encoding="utf-8")
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        fault_start = uart_c.rfind("static int16_t DrvUart_FormatFault(")
        fault_end = uart_c.index("static bool DrvUart_StartSend", fault_start)
        fault_section = uart_c[fault_start:fault_end]

        self.assertIn("huart1.Init.BaudRate = 230400", usart_c)
        self.assertIn("undervoltageLimit", uart_h)
        self.assertIn("overvoltageLimit", uart_h)
        self.assertIn("appFaultCode", uart_h)
        self.assertIn('APPEND_FMT("N,', uart_c)
        self.assertNotIn("========== FOC Controller Status ==========", uart_c[uart_c.index("static int16_t DrvUart_FormatNormal"):uart_c.index("static int16_t DrvUart_FormatFault")])
        self.assertIn('APPEND_FMT("F,', uart_c)
        self.assertIn("========== !!! FAULT DETECTED !!! ==========", uart_c)
        self.assertIn("AngleRaw", fault_section)
        self.assertIn("AppFault:", fault_section)
        self.assertIn("RawVbus", fault_section)
        self.assertIn("ADC Pin:", fault_section)
        self.assertIn("Vbus:", fault_section)
        self.assertIn("230400 bps", readme)
        self.assertIn("N,timestamp_ms", readme)
        self.assertIn("F,timestamp_ms", readme)
        self.assertIn("app_fault_code", readme)
        self.assertIn("undervoltage_limit_v", readme)
        self.assertIn("overvoltage_limit_v", readme)

    def test_uart_fault_reporting_combines_drv_and_application_fault_sources(self):
        drv_h = (ROOT / "MDK-ARM" / "code" / "drv8350s.h").read_text(encoding="utf-8")
        drv_c = (ROOT / "MDK-ARM" / "code" / "drv8350s.c").read_text(encoding="utf-8")
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("volatile uint32_t latchedFaultFlags;", drv_h)
        self.assertIn("volatile uint16_t latchedFaultStatus1;", drv_h)
        self.assertIn("volatile uint16_t latchedVgsStatus2;", drv_h)
        self.assertIn("handle->runtime.latchedFaultFlags == 0U", drv_c)
        self.assertIn("handle->runtime.latchedFaultFlags = faults;", drv_c)
        self.assertIn("packet->latchedFaultFlags = s_drvHandle->runtime.latchedFaultFlags;", uart_c)
        self.assertIn("packet->latchedFaultStatus1 = s_drvHandle->runtime.latchedFaultStatus1;", uart_c)
        self.assertIn("packet->latchedVgsStatus2 = s_drvHandle->runtime.latchedVgsStatus2;", uart_c)
        self.assertIn("packet->faultFlags | packet->latchedFaultFlags", uart_c)
        self.assertIn("drv8350s.runtime.latchedFaultFlags = 0U;\n            drv8350s.runtime.latchedFaultStatus1 = 0U;\n            drv8350s.runtime.latchedVgsStatus2 = 0U;\n            HAL_GPIO_WritePin", foc_c)
        self.assertIn("packet->appFaultCode = (uint8_t)g_foc_app.fault_code;", uart_c)
        self.assertIn("packet->isFaultActive = (((packet->faultFlags | packet->latchedFaultFlags) != 0U) ||", uart_c)
        self.assertIn("s_drvHandle->runtime.latchedFaultFlags != 0U", uart_c)
        self.assertIn("currentAppFaultCode = (uint8_t)g_foc_app.fault_code;", uart_c)
        self.assertIn("currentAppFaultCode != s_lastAppFaultCode", uart_c)
        self.assertIn("g_foc_app.state == FOC_STATE_FAULT", uart_c)
        self.assertIn("FOC_App_GetFaultString", uart_c)
        self.assertIn("FOC_App_GetStateString", uart_c)

    def test_adc_voltage_conversion_uses_4096_full_scale_contract(self):
        adc_h = (ROOT / "MDK-ARM" / "code" / "adc_sampling.h").read_text(encoding="utf-8")

        self.assertIn("#define ADC_MAX             4096.0f", adc_h)

    def test_uart_fault_summary_avoids_float_printf_in_fault_first_path(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        start = uart_c.index("static int16_t DrvUart_FormatFaultSummary(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)\n{")
        end = uart_c.index("static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)", start)
        self.assertNotEqual(start, -1)
        self.assertNotEqual(end, -1)
        summary_section = uart_c[start:end]

        self.assertIn("DrvUart_FormatFixed", summary_section)
        self.assertNotIn("%.2f", summary_section)
        self.assertNotIn("%f", summary_section)

    def test_fault_path_uses_summary_plus_chunked_detail_report(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        self.assertIn("static uint8_t s_faultDetailBuf[DRV_UART_BUF_SIZE];", uart_c)
        self.assertIn("static uint16_t s_faultDetailLen = 0U;", uart_c)
        self.assertIn("static uint16_t s_faultDetailOffset = 0U;", uart_c)
        self.assertIn("#define DRV_UART_FAULT_DETAIL_CHUNK_SIZE", uart_c)
        self.assertIn("DrvUart_FormatFaultSummary(&packet, s_txBuf, DRV_UART_BUF_SIZE)", uart_c)
        self.assertIn("DrvUart_QueueFaultDetail(&packet, currentTime);", uart_c)
        self.assertIn("DrvUart_FormatFault(packet, s_faultDetailBuf, DRV_UART_BUF_SIZE)", uart_c)
        self.assertIn("memcpy(s_txBuf, &s_faultDetailBuf[s_faultDetailOffset], chunkLen);", uart_c)
        self.assertIn("s_faultDetailOffset += chunkLen;", uart_c)
        self.assertIn("s_faultDetailOffset >= s_faultDetailLen", uart_c)

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

    def test_keil_project_includes_demo_button_module(self):
        uvprojx = (ROOT / "MDK-ARM" / "24V FOC Controller.uvprojx").read_text(encoding="utf-8")

        self.assertIn("<FileName>demo_button_control.c</FileName>", uvprojx)
        self.assertIn(r"<FilePath>.\code\demo_button_control.c</FilePath>", uvprojx)

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
        self.assertIn("Loop Parameters", readme)
        self.assertIn("Position Loop PD", readme)
        self.assertIn("Identify", readme)
        self.assertIn("HostMainWindow", architecture)
        self.assertIn("gui_logic.py", architecture)
        self.assertIn("Debug Panel", architecture)
        self.assertIn("Loop Parameters", architecture)
        self.assertIn("Position Loop PD", architecture)

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
        self.assertIn("dist_rebuilt", script)
        self.assertIn("Remove-WorkspaceDirectoryIfEmpty", script)
        self.assertIn("Legacy cleanup", script)
        self.assertIn("Test-PythonModulesAvailable", script)
        self.assertIn("Dependencies already installed; skipping pip install.", script)
        self.assertIn("--disable-pip-version-check", script)

        self.assertIn("name='24V_FOC_Host'", spec)
        self.assertIn("console=False", spec)
        self.assertIn("pyqtgraph", spec)
        self.assertIn("PROJECT_ROOT = Path(SPECPATH).parent", spec)
        self.assertIn("host_gui_launcher.py", spec)

        self.assertIn("build_host_gui_app.ps1", readme)
        self.assertIn("24V_FOC_Host.exe", readme)
        self.assertIn("PyInstaller", readme)
        self.assertIn("dist_rebuilt/24V_FOC_Host", readme)
        self.assertIn("skip `pip install`", readme)

    def test_standalone_hse_led_test_project_contract(self):
        test_dir = ROOT / "BenchTests" / "HSE_LED_Test"
        main_c = test_dir / "main.c"
        build_ps1 = test_dir / "build.ps1"

        self.assertTrue(main_c.exists())
        self.assertTrue(build_ps1.exists())

        main_text = main_c.read_text(encoding="utf-8")
        build_text = build_ps1.read_text(encoding="utf-8")

        self.assertIn("GPIO_PIN_8|GPIO_PIN_9", main_text)
        self.assertIn("RCC_OSCILLATORTYPE_HSE", main_text)
        self.assertIn("RCC_HSE_ON", main_text)
        self.assertIn("RCC_PLLSOURCE_HSE", main_text)
        self.assertIn("static void LedTest_SuccessLoop(void)", main_text)
        self.assertIn("static void LedTest_FailureLoop(void)", main_text)
        self.assertIn("HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);", main_text)
        self.assertIn("HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);", main_text)
        self.assertIn("SystemClock_Config()", main_text)
        self.assertIn("startup_stm32h743xx.s", build_text)
        self.assertIn("STM32H743VITX_FLASH.ld", build_text)
        self.assertIn("hse_led_test.elf", build_text)

    def test_bench_scripts_do_not_mislabel_ready_as_closed_loop(self):
        bench_v2 = (ROOT / "bench_test_v2.py").read_text(encoding="utf-8")
        bench_runner = (ROOT / "bench_test_runner.py").read_text(encoding="utf-8")

        self.assertIn("FOC_STATE_READY = 3", bench_v2)
        self.assertIn("FOC_STATE_RUNNING = 4", bench_v2)
        self.assertIn("p.foc_state == FOC_STATE_RUNNING", bench_v2)
        self.assertNotIn("p.foc_state == 3", bench_v2)
        self.assertNotIn("CLOSED_LOOP", bench_v2)
        self.assertIn("FOC_STATE_RUNNING = 4", bench_runner)
        self.assertIn("foc_state == FOC_STATE_RUNNING", bench_runner)

    def test_bench_position_commands_convert_degrees_to_radians(self):
        bench_v2 = (ROOT / "bench_test_v2.py").read_text(encoding="utf-8")
        bench_runner = (ROOT / "bench_test_runner.py").read_text(encoding="utf-8")

        self.assertIn("def send_position_deg", bench_v2)
        self.assertIn("math.radians", bench_v2)
        self.assertNotIn("CMD:PREF,{t:.3f}", bench_v2)
        self.assertNotIn("CMD:PREF,{target:.3f}", bench_v2)
        self.assertIn("math.degrees(p.pos_ref)", bench_v2)
        self.assertIn("math.degrees(pk.pos_ref)", bench_runner)

    def test_closed_bench_script_captures_full_fault_detail_block_per_command(self):
        closed_script = (ROOT / "_closed.py").read_text(encoding="utf-8")

        self.assertIn("def capture_fault_detail_block", closed_script)
        self.assertIn("flush_input(ser)", closed_script)
        self.assertIn('line.startswith(wanted_prefixes)', closed_script)
        self.assertIn('"ThetaDiag:"', closed_script)
        self.assertIn('"CurrentDQ:"', closed_script)
        self.assertIn('"ParamDiag:"', closed_script)
        self.assertIn('log.write(f"IQ_CMD={iq:+.3f}: " + capture_fault_detail_block(ser) + "\\n")', closed_script)
        self.assertNotIn("if 'CurrentDQ' in l:log.write('Iq_ref=%+.1f: '", closed_script)


if __name__ == "__main__":
    unittest.main()









