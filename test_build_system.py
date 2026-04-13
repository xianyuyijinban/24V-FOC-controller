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
        self.assertIn("static void TLE5012_TwrDelay", tle_c)
        self.assertIn("crc_words[0] = TLE5012_READ_CMD;", tle_c)
        self.assertIn("crc_words[1] = raw_data;", tle_c)
        self.assertIn("TLE5012_CalculateCRC8(crc_words, 2U);", tle_c)
        self.assertIn("void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)", it_c)
        self.assertIn("TLE5012_HandleTxComplete();", it_c)
        self.assertIn("if (hspi == &hspi3) {", it_c)
        self.assertIn("TLE5012_ProcessData(tle5012_rx_buf);", it_c)

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
        self.assertIn("config.vdsLvl = 0x01;", main_c)
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

    def test_debug_boot_path_uses_hsi_and_gates_fdcan_init(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        fdcan_h = (ROOT / "Core" / "Inc" / "fdcan.h").read_text(encoding="utf-8")
        fdcan_c = (ROOT / "Core" / "Src" / "fdcan.c").read_text(encoding="utf-8")
        ioc = (ROOT / "24V FOC Controller.ioc").read_text(encoding="utf-8")

        self.assertIn("RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;", main_c)
        self.assertIn("RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;", main_c)
        self.assertIn("RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 4;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 60;", main_c)
        self.assertIn("RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;", main_c)
        self.assertIn("RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSI", ioc)
        self.assertIn("RCC.DIVM1=4", ioc)
        self.assertIn("RCC.DIVN1=60", ioc)
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
        self.assertIn("FOC_ProtectionConfig_t protection;", foc_h)
        self.assertIn("void FOC_App_SetVoltageThresholds(FOC_AppHandle_t *handle, float undervoltage, float overvoltage);", foc_h)
        self.assertIn("handle->protection.overcurrent_limit_a", foc_c)
        self.assertIn("handle->protection.overvoltage_limit_v", foc_c)
        self.assertIn("handle->protection.undervoltage_limit_v", foc_c)
        self.assertIn("CMD:VBUS_LIMIT,%f,%f", it_c)
        self.assertIn("FOC_App_SetVoltageThresholds(&g_foc_app, f1, f2);", it_c)
        self.assertNotIn("g_foc_app.Vbus >= FOC_UNDERVOLTAGE_THRESH", it_c)
        self.assertNotIn("g_foc_app.Vbus <= FOC_OVERVOLTAGE_THRESH", it_c)

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

    def test_motor_identification_requires_valid_encoder_feedback(self):
        mi_h = (ROOT / "MDK-ARM" / "code" / "motor_identify.h").read_text(encoding="utf-8")
        mi_c = (ROOT / "MDK-ARM" / "code" / "motor_identify.c").read_text(encoding="utf-8")
        foc_c = (ROOT / "MDK-ARM" / "code" / "foc_app.c").read_text(encoding="utf-8")

        self.assertIn("MI_ERR_ENCODER_INVALID", mi_h)
        self.assertIn('case MI_ERR_ENCODER_INVALID:    return "Encoder Invalid";', mi_c)
        self.assertIn("static MI_ErrorCode_t MI_RequireValidEncoder(void)", mi_c)
        self.assertGreaterEqual(mi_c.count("MI_RequireValidEncoder()"), 3)
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

    def test_startup_primes_tim1_oc4_before_adc_zero_calibration(self):
        main_c = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")

        base_start = main_c.find("HAL_TIM_Base_Start(&htim1)")
        oc4_start = main_c.find("HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4)")
        adc_cal = main_c.find("ADC_Sampling_Calibrate(200)")
        clear_uif = main_c.find("__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE)")
        irq_enable = main_c.find("__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE)")

        self.assertNotEqual(base_start, -1, "TIM1 base should be started before ADC zero calibration")
        self.assertNotEqual(oc4_start, -1, "TIM1 CH4/OC4REF should be started before ADC zero calibration")
        self.assertNotEqual(adc_cal, -1, "main.c should still perform ADC zero calibration")
        self.assertNotEqual(clear_uif, -1, "main.c should clear any pending TIM1 update flag before enabling interrupts")
        self.assertNotEqual(irq_enable, -1, "TIM1 update interrupt must still be enabled for control loop")
        self.assertLess(base_start, adc_cal)
        self.assertLess(oc4_start, adc_cal)
        self.assertLess(base_start, irq_enable)

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

    def test_uart_fault_formatter_avoids_float_printf_in_fault_path(self):
        uart_c = (ROOT / "MDK-ARM" / "code" / "uart_upload.c").read_text(encoding="utf-8")

        fault_start = uart_c.index("static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)\n{")
        fault_end = uart_c.index("static bool DrvUart_StartSend", fault_start)
        fault_section = uart_c[fault_start:fault_end]

        self.assertNotIn("%7.2f", fault_section)
        self.assertNotIn("%7.3f", fault_section)
        self.assertNotIn("%4.1f", fault_section)
        self.assertIn("AngleRaw", fault_section)
        self.assertIn("Vbus:", fault_section)

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


if __name__ == "__main__":
    unittest.main()
