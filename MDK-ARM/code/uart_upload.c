/**
 * @file    uart_upload.c
 * @brief   DRV8350S parameter and fault telemetry upload over UART
 */

#include "uart_upload.h"
#include "foc_app.h"
#include "param_storage.h"
#include "tle5012.h"
#include "tim.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef* s_huart = NULL;
static DRV8350S_Handle_t* s_drvHandle = NULL;

/* ── Phase 2: UART TX Ring Buffer (IT-based, no DMA) ────────── */
#define UART_TX_RING_SIZE       1024U
#define UART_TX_P0_RESERVE       128U
#define UART_TX_P1_RESERVE       512U

typedef enum {
    UART_PRIO_P0 = 0,  /* command echo, STOP, fault */
    UART_PRIO_P1 = 1,  /* DIAG single output */
    UART_PRIO_P2 = 2,  /* periodic telemetry */
} UartTxPrio;

static uint8_t s_txBuf[DRV_UART_BUF_SIZE];
static uint8_t s_faultDetailBuf[DRV_UART_BUF_SIZE];
static volatile bool s_txITActive = false;
static uint8_t  s_txRing[UART_TX_RING_SIZE];
static volatile uint16_t s_txRingHead = 0U;
static volatile uint16_t s_txRingTail = 0U;
static volatile uint32_t s_txDropCount[3] = {0, 0, 0};
static volatile bool s_enabled = true;
static uint32_t s_uploadInterval = DRV_UPLOAD_INTERVAL_MS;
static uint32_t s_lastUploadTime = 0;
static uint32_t s_lastPhaseCurrentUploadTime = 0;
static uint16_t s_faultDetailLen = 0U;
static uint16_t s_faultDetailOffset = 0U;

static DrvUart_FaultRecord_t s_faultHistory[DRV_FAULT_HISTORY_SIZE];
static uint8_t s_faultHistoryHead = 0;
static uint8_t s_faultHistoryCount = 0;

static DrvUart_Statistics_t s_stats = {0};
static DrvUart_DataPacket_t s_lastFault = {0};

static uint32_t s_lastFaultFlags = 0;  /* Used to detect new fault roots */
static uint8_t s_lastAppFaultCode = 0U;





static uint8_t s_lastIdentifyState = 0U;
static uint8_t s_lastIdentifyError = 0U;
static uint8_t s_lastFaultActive = 0U;
#define DRV_UART_FAULT_DETAIL_CHUNK_SIZE 64U

/* Private function prototypes ----------------------------------------------*/
static void DrvUart_CollectData(DrvUart_DataPacket_t* packet, uint8_t type);
static int16_t DrvUart_FormatNormal(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static int16_t DrvUart_FormatPhaseCurrent(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static int16_t DrvUart_FormatFaultSummary(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize);
static void DrvUart_QueueFaultDetail(const DrvUart_DataPacket_t* packet, uint32_t currentTime);
static void DrvUart_AddFaultHistory(const DrvUart_DataPacket_t* packet);
static bool DrvUart_StartSend(uint16_t len);
static int16_t DrvUart_Append(uint8_t* buf, uint16_t bufSize, int16_t len, const char* fmt, ...);
void DrvUart_FormatFixed(char* dst, uint16_t dstSize, float value, uint8_t decimals);
static const char* DrvUart_IdentifyErrorToString(uint8_t error);
static const char* DrvUart_IdentifyStateToString(uint8_t state);

/* Function implementation ---------------------------------------------------*/

/**
 * @brief Initialize UART upload module
 */
HAL_StatusTypeDef DrvUart_Init(UART_HandleTypeDef* huart, DRV8350S_Handle_t* drvHandle)
{
    if (huart == NULL || drvHandle == NULL) {
        return HAL_ERROR;
    }
    
    s_huart = huart;
    s_drvHandle = drvHandle;
    s_txITActive = false;
    s_txRingHead = 0U;
    s_txRingTail = 0U;
    s_txDropCount[0] = 0U; s_txDropCount[1] = 0U; s_txDropCount[2] = 0U;
    s_enabled = true;
    s_lastUploadTime = 0;
    s_lastPhaseCurrentUploadTime = 0;
    s_uploadInterval = DRV_UPLOAD_INTERVAL_MS;
    
    memset(s_txBuf, 0, sizeof(s_txBuf));
    memset(s_faultDetailBuf, 0, sizeof(s_faultDetailBuf));
    memset(s_faultHistory, 0, sizeof(s_faultHistory));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_lastFault, 0, sizeof(s_lastFault));
    
    s_faultHistoryHead = 0;
    s_faultHistoryCount = 0;
    s_lastFaultFlags = 0;
    s_lastAppFaultCode = 0U;
    s_lastIdentifyState = 0U;
    s_lastIdentifyError = 0U;
    s_lastFaultActive = 0U;
    s_faultDetailLen = 0U;
    s_faultDetailOffset = 0U;
    
    return HAL_OK;
}

/**
 * @brief Deinitialize UART upload module
 */
void DrvUart_DeInit(void)
{
    if (s_huart != NULL && s_txITActive) {
        (void)HAL_UART_AbortTransmit(s_huart);
    }
    
    s_huart = NULL;
    s_drvHandle = NULL;
    s_txITActive = false;
    s_enabled = false;
    s_faultDetailLen = 0U;
    s_faultDetailOffset = 0U;
}

/* External FOC application data */
extern FOC_AppHandle_t g_foc_app;

/**
 * @brief Collect DRV8350S, TLE5012, FOC and ADC data
 */
static void DrvUart_CollectData(DrvUart_DataPacket_t* packet, uint8_t type)
{
    ADC_Sampling_t* adc;
    FOC_ABC_t adcTelemetryIabc;
    FOC_AlphaBeta_t adcTelemetryAlphaBeta;
    FOC_DQ_t adcTelemetryIdq;
    float encoder_dir;

    if (packet == NULL || s_drvHandle == NULL) {
        return;
    }

    adc = ADC_Sampling_GetData();
    encoder_dir = (g_foc_app.motor_param.encoder_dir < 0) ? -1.0f : 1.0f;
    adcTelemetryIabc.a = adc->currentA;
    adcTelemetryIabc.b = adc->currentB;
    adcTelemetryIabc.c = adc->currentC;
    FOC_Clarke_Transform(&adcTelemetryIabc, &adcTelemetryAlphaBeta);
    FOC_Park_Transform(&adcTelemetryAlphaBeta, g_foc_app.foc.sin_theta, g_foc_app.foc.cos_theta, &adcTelemetryIdq);

    packet->timestamp = HAL_GetTick();
    packet->packetType = type;

    /* Encoder data — apply user mechanical zero offset so HOME sets displayed angle to 0 */
    {
        float theta_user = FOC_AngleNormalize(g_foc_app.theta_mech - g_foc_app.motor_param.mech_zero_offset);
        if (theta_user < 0.0f) {
            theta_user += 2.0f * FOC_PI;
        }
        packet->angle = theta_user * 180.0f / FOC_PI;
    }
    packet->rawAngle = tle5012_sensor.raw_angle;
    packet->encoderRawWord = tle5012_sensor.raw_word;
    packet->encoderSafetyWord = tle5012_sensor.safety_word;
    packet->crcError = tle5012_sensor.crc_error;
    packet->encoderReceivedCrc = tle5012_sensor.received_crc;
    packet->encoderCalculatedCrc = tle5012_sensor.calculated_crc;
    packet->encoderCrcErrorCount = TLE5012_GetCRCErrorCount();
    packet->encoderGpioDiagActive = TLE5012_IsGpioDiagActive();
    packet->encoderSafetyStatus = tle5012_sensor.status;
    packet->encoderResetFault = tle5012_sensor.reset_fault;
    packet->encoderDetected = TLE5012_IsDataValid();

    /* DRV8350S data */
    packet->faultStatus1 = s_drvHandle->runtime.regFaultStatus1;
    packet->vgsStatus2 = s_drvHandle->runtime.regVgsStatus2;
    packet->driverCtrl = s_drvHandle->runtime.regDriverCtrl;
    packet->ocpCtrl = s_drvHandle->runtime.regOcpCtrl;
    packet->faultFlags = s_drvHandle->runtime.faultFlags;
    packet->latchedFaultFlags = s_drvHandle->runtime.latchedFaultFlags;
    packet->latchedFaultStatus1 = s_drvHandle->runtime.latchedFaultStatus1;
    packet->latchedVgsStatus2 = s_drvHandle->runtime.latchedVgsStatus2;
    packet->drvCommFaultActive = s_drvHandle->runtime.commFaultActive;
    packet->drvCommValidated = s_drvHandle->runtime.commValidated;
    packet->drvLastRxFrame = s_drvHandle->runtime.lastRxFrame;

    /* FOC application data */
    packet->Id = g_foc_app.foc.Idq.d;
    packet->Iq = g_foc_app.foc.Idq.q;
    packet->adcId = adcTelemetryIdq.d;
    packet->adcIq = adcTelemetryIdq.q;
    packet->Vd = g_foc_app.foc.Vdq.d;
    packet->Vq = g_foc_app.foc.Vdq.q;
    packet->speed = g_foc_app.speed_mech * encoder_dir;
    packet->thetaMech = g_foc_app.theta_mech;
    packet->thetaElec = g_foc_app.theta_elec;
    packet->Id_ref = g_foc_app.foc.Id_ref;
    packet->Iq_ref = g_foc_app.foc.Iq_ref;
    packet->speed_ref = g_foc_app.speed_ref;
    packet->pos_ref = FOC_App_PositionControlToSensorFrame(&g_foc_app, g_foc_app.pos_ref);
    packet->speedLoopRef = g_foc_app.speed_loop_ref_diag;
    packet->speedLoopMech = g_foc_app.speed_loop_mech_diag;
    packet->speedLoopError = g_foc_app.speed_loop_error_diag;
    packet->speedLoopIqMech = g_foc_app.speed_loop_iq_mech_diag;
    packet->speedLoopFriction = g_foc_app.speed_loop_friction_diag;
    packet->speedLoopIqCmd = g_foc_app.speed_loop_iq_cmd_diag;
    packet->speedLoopPIq = g_foc_app.speed_loop_p_iq;
    packet->speedLoopIIq = g_foc_app.speed_loop_i_iq;
    packet->speedLoopIState = g_foc_app.speed_i_state;
    packet->positionLoopError = g_foc_app.position_loop_error_diag;
    packet->positionLoopPdOut = g_foc_app.position_loop_pd_out_diag;
    packet->positionLoopPdSat = g_foc_app.position_loop_pd_sat_diag;
    packet->positionLoopRampSat = g_foc_app.position_loop_speed_ramp_sat_diag;
    packet->positionLoopIqPosSat = g_foc_app.position_loop_iq_pos_sat_diag;
    packet->positionLoopIqNegSat = g_foc_app.position_loop_iq_neg_sat_diag;
    packet->trajActive = g_foc_app.traj_active_diag;
    packet->trajCmd = g_foc_app.traj_cmd_diag;
    packet->positionPrefCmdCount = g_foc_app.position_pref_cmd_count_diag;
    packet->positionPrefRaw = g_foc_app.position_pref_raw_diag;
    packet->positionPrefMapped = g_foc_app.position_pref_mapped_diag;
    packet->positionPrefBefore = g_foc_app.position_pref_before_diag;
    packet->positionPrefAfter = g_foc_app.position_pref_after_diag;
    packet->positionPrefUserSet = g_foc_app.position_pref_user_set_diag;

    /* FFDiag 前馈诊断 */
    packet->ffBemfVd = g_foc_app.ff_diag.bemf_vd;
    packet->ffBemfVq = g_foc_app.ff_diag.bemf_vq;
    packet->ffInertiaIq = g_foc_app.ff_diag.inertia_iq;
    packet->ffFrictionIq = g_foc_app.ff_diag.friction_iq;
    packet->ffCoggingIq = g_foc_app.ff_diag.cogging_iq;
    packet->ffObserverIq = g_foc_app.ff_diag.observer_iq;
    packet->ffTotalIq = g_foc_app.ff_diag.ff_total_iq;
    packet->ffBemfEnabled = g_foc_app.ff_diag.bemf_enabled;
    packet->ffInertiaBlocked = g_foc_app.ff_diag.inertia_blocked;
    packet->ffFrictionEnabled = g_foc_app.ff_diag.friction_enabled;
    packet->ffCoggingEnabled = g_foc_app.ff_diag.cogging_enabled;
    packet->ffObserverEnabled = g_foc_app.ff_diag.observer_enabled;
    packet->ffEncDirBlocked = g_foc_app.ff_diag.ff_enc_dir_blocked;

    packet->undervoltageLimit = g_foc_app.protection.undervoltage_limit_v;
    packet->overvoltageLimit = g_foc_app.protection.overvoltage_limit_v;
    packet->focState = (uint8_t)g_foc_app.state;
    packet->controlMode = (uint8_t)g_foc_app.control_mode;
    packet->pwmEnabled = g_foc_app.enable_pwm;
    packet->tim1MoeEnabled = ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    packet->appWarningFlags = g_foc_app.warning_flags;
    packet->appFaultCode = (uint8_t)g_foc_app.fault_code;
    packet->motorIdentified = g_foc_app.motor_identified;
    packet->stallModeArmed = g_foc_app.stall_mode_armed;
    packet->stallOpenLoopActive = g_foc_app.stall_open_loop_active;
    packet->identifyState = (uint8_t)g_foc_app.mi_handle.state;
    packet->identifyError = (uint8_t)g_foc_app.mi_handle.error_code;
    packet->identifyRsCurrentTarget = g_foc_app.mi_handle.rs_current_target;
    packet->identifyRsLastVavg = g_foc_app.mi_handle.rs_last_v_avg;
    packet->identifyRsLastIavg = g_foc_app.mi_handle.rs_last_i_avg;
    packet->identifyRsLastImagAvg = g_foc_app.mi_handle.rs_last_i_mag_avg;
    packet->identifyRsLastVecRs = g_foc_app.mi_handle.rs_last_vec_rs;
    packet->identifyRsLastSamples = g_foc_app.mi_handle.rs_last_samples;
    packet->identifyRsPositive = g_foc_app.mi_handle.Rs_positive;
    packet->identifyRsNegative = g_foc_app.mi_handle.Rs_negative;
    packet->identifyLsVrms = g_foc_app.mi_handle.ls_last_v_rms;
    packet->identifyLsIrms = g_foc_app.mi_handle.ls_last_i_rms;
    packet->identifyLsZ = g_foc_app.mi_handle.ls_last_z;
    packet->identifyLsXl = g_foc_app.mi_handle.ls_last_xl;
    packet->identifyLsL = g_foc_app.mi_handle.ls_last_l;
    packet->identifyLsUsedFallback = g_foc_app.mi_handle.ls_used_fallback;
    packet->identifyPnCurrentTarget = g_foc_app.mi_handle.pn_current_target;
    packet->identifyPnDeltaMech = g_foc_app.mi_handle.pn_last_delta_mech;
    packet->identifyPnDeltaElec = g_foc_app.mi_handle.pn_last_delta_elec;
    packet->identifyPnCalc = g_foc_app.mi_handle.pn_last_calc;
    packet->identifyPnObservedDir = g_foc_app.mi_handle.pn_observed_dir;
    packet->identifyVerifyLockedDir = g_foc_app.mi_handle.verify_locked_dir;
    packet->identifyVerifyPhase = g_foc_app.mi_handle.verify_phase;
    packet->identifyVerifyAccum = g_foc_app.mi_handle.verify_theta_accum;
    packet->paramInvalidFlags = Param_GetInvalidFlags(&g_foc_app.motor_param);
    packet->paramValidFlag = g_foc_app.motor_param.valid_flag;
    packet->paramRs = g_foc_app.motor_param.Rs;
    packet->paramLd = g_foc_app.motor_param.Ld;
    packet->paramLq = g_foc_app.motor_param.Lq;
    packet->paramKe = g_foc_app.motor_param.Ke;
    packet->paramPn = g_foc_app.motor_param.Pn;
    packet->paramEncoderDir = g_foc_app.motor_param.encoder_dir;
    packet->paramThetaOffset = g_foc_app.motor_param.theta_offset;
    packet->paramThetaMechZero = g_foc_app.motor_param.mech_zero_offset;
    packet->paramJ = g_foc_app.motor_param.J;
    packet->svpwmSector = g_foc_app.foc.svpwm.sector;
    packet->svpwmTa = g_foc_app.foc.svpwm.Ta;
    packet->svpwmTb = g_foc_app.foc.svpwm.Tb;
    packet->svpwmTc = g_foc_app.foc.svpwm.Tc;
    packet->isFaultActive = (((packet->faultFlags | packet->latchedFaultFlags) != 0U) ||
                             (packet->appFaultCode != (uint8_t)FOC_FAULT_NONE) ||
                             (packet->focState == (uint8_t)FOC_STATE_FAULT)) ? 1U : 0U;

    (void)snprintf(packet->adcTriggerSource,
                   sizeof(packet->adcTriggerSource),
                   "%s",
                   ADC_SAMPLING_TRIGGER_SOURCE_TEXT);
    packet->adcCurrentSampleTimeCycles = ADC_SAMPLING_CURRENT_SAMPLE_TIME_CYCLES;
    packet->adcVbusSampleTimeCycles = ADC_SAMPLING_VBUS_SAMPLE_TIME_CYCLES;
    packet->adcFrameSequence = adc->frameSequence;
    packet->adcFrameAgeCycles = adc->frameAgeCycles;
    packet->adcSampleMissCount = adc->sampleMissCount;
    packet->adcInvalidWindowCount = adc->invalidWindowCount;
    packet->adcValidLowSideCount = g_foc_app.adc_valid_low_side_count;
    packet->adcInvalidLowSideCount = g_foc_app.adc_invalid_low_side_count;
    packet->adcForcedLowSideCount = g_foc_app.adc_forced_low_side_count;
    packet->adcRawCurrentA = adc->rawCurrentA;
    packet->adcRawCurrentB = adc->rawCurrentB;
    packet->adcRawCurrentC = adc->rawCurrentC;
    packet->adcRawVbus = adc->rawVbus;
    packet->adcPwmPeriod = adc->pwmPeriod;
    packet->adcPwmCompareA = adc->pwmCompareA;
    packet->adcPwmCompareB = adc->pwmCompareB;
    packet->adcPwmCompareC = adc->pwmCompareC;
    packet->adcTriggerCompare = adc->triggerCompare;
    packet->adcTimerCount = adc->timerCount;
    packet->adcTimerCountingDown = adc->timerCountingDown;
    packet->adcLowSideValidA = adc->lowSideValidA;
    packet->adcLowSideValidB = adc->lowSideValidB;
    packet->adcLowSideValidC = adc->lowSideValidC;
    packet->adcCalibStatus = (uint8_t)adc->calibStatus;
    packet->adcOffsetA = adc->offsetA;
    packet->adcOffsetB = adc->offsetB;
    packet->adcOffsetC = adc->offsetC;
    packet->adcCurrentA = adc->currentA;
    packet->adcCurrentB = adc->currentB;
    packet->adcCurrentC = adc->currentC;
    packet->loopCurrentA = g_foc_app.Ia;
    packet->loopCurrentB = g_foc_app.Ib;
    packet->loopCurrentC = g_foc_app.Ic;
    packet->adcDeltaA = (int16_t)((int32_t)adc->rawCurrentA - (int32_t)adc->offsetA);
    packet->adcDeltaB = (int16_t)((int32_t)adc->rawCurrentB - (int32_t)adc->offsetB);
    packet->adcDeltaC = (int16_t)((int32_t)adc->rawCurrentC - (int32_t)adc->offsetC);
    packet->adcVbus = adc->vbus;
}

/**
 * @brief Format normal data as a compact single-line frame
 */
static int16_t DrvUart_FormatNormal(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
    char angleText[20];
    char idText[20];
    char iqText[20];
    char speedText[20];
    char vbusText[20];
    char idRefText[20];
    char iqRefText[20];
    char speedRefText[20];
    char posRefText[20];
    char vdText[20];
    char vqText[20];
    char iaText[20];
    char ibText[20];
    char icText[20];
    char uvText[20];
    char ovText[20];
#define APPEND_FMT(...) \
    do { \
        len = DrvUart_Append(buf, bufSize, len, __VA_ARGS__); \
        if (len < 0) { \
            return -1; \
        } \
    } while (0)
    
    if (packet == NULL || buf == NULL || bufSize < 96) {
        return -1;
    }

    DrvUart_FormatFixed(angleText, sizeof(angleText), packet->angle, 2U);
    DrvUart_FormatFixed(idText, sizeof(idText), packet->Id, 3U);
    DrvUart_FormatFixed(iqText, sizeof(iqText), packet->Iq, 3U);
    DrvUart_FormatFixed(speedText, sizeof(speedText), packet->speed, 2U);
    DrvUart_FormatFixed(vbusText, sizeof(vbusText), packet->adcVbus, 2U);
    DrvUart_FormatFixed(idRefText, sizeof(idRefText), packet->Id_ref, 3U);
    DrvUart_FormatFixed(iqRefText, sizeof(iqRefText), packet->Iq_ref, 3U);
    DrvUart_FormatFixed(speedRefText, sizeof(speedRefText), packet->speed_ref, 3U);
    DrvUart_FormatFixed(posRefText, sizeof(posRefText), packet->pos_ref, 3U);
    DrvUart_FormatFixed(vdText, sizeof(vdText), packet->Vd, 3U);
    DrvUart_FormatFixed(vqText, sizeof(vqText), packet->Vq, 3U);
    DrvUart_FormatFixed(iaText, sizeof(iaText), packet->adcCurrentA, 3U);
    DrvUart_FormatFixed(ibText, sizeof(ibText), packet->adcCurrentB, 3U);
    DrvUart_FormatFixed(icText, sizeof(icText), packet->adcCurrentC, 3U);
    DrvUart_FormatFixed(uvText, sizeof(uvText), packet->undervoltageLimit, 2U);
    DrvUart_FormatFixed(ovText, sizeof(ovText), packet->overvoltageLimit, 2U);
    APPEND_FMT("N,%lu,%u,%s,%s,%s,%s,%s,0x%08lX,%u,%u,%u,%u,0x%08lX,%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u,%s,%s,%u,%d,%d,%d\n",
               packet->timestamp,
               packet->focState,
               angleText,
               speedText,
               idText,
               iqText,
               vbusText,
               (unsigned long)packet->faultFlags,
               packet->encoderDetected ? 1U : 0U,
               packet->motorIdentified ? 1U : 0U,
               packet->stallModeArmed ? 1U : 0U,
               packet->stallOpenLoopActive ? 1U : 0U,
               (unsigned long)packet->appWarningFlags,
               packet->appFaultCode,
               packet->controlMode,
               idRefText,
               speedRefText,
               posRefText,
               iqRefText,
               vdText,
               vqText,
               iaText,
               ibText,
               icText,
               packet->identifyState,
               packet->identifyError,
               uvText,
               ovText,
               (unsigned)packet->adcCalibStatus,
               (int)packet->adcOffsetA,
               (int)packet->adcOffsetB,
               (int)packet->adcOffsetC);

#undef APPEND_FMT
    
    return (len > 0 && len < bufSize) ? len : -1;
}

static int16_t DrvUart_FormatPhaseCurrent(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
    char iaText[20];
    char ibText[20];
    char icText[20];

    if (packet == NULL || buf == NULL || bufSize < 48) {
        return -1;
    }

    DrvUart_FormatFixed(iaText, sizeof(iaText), packet->adcCurrentA, 3U);
    DrvUart_FormatFixed(ibText, sizeof(ibText), packet->adcCurrentB, 3U);
    DrvUart_FormatFixed(icText, sizeof(icText), packet->adcCurrentC, 3U);
    len = DrvUart_Append(buf,
                         bufSize,
                         len,
                         "C,%lu,%s,%s,%s\n",
                         packet->timestamp,
                         iaText,
                         ibText,
                         icText);

    return (len > 0 && len < bufSize) ? len : -1;
}

static int16_t DrvUart_FormatFaultSummary(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
    char vbusText[20];
    char uvText[20];
    char ovText[20];
#define APPEND_FMT(...) \
    do { \
        len = DrvUart_Append(buf, bufSize, len, __VA_ARGS__); \
        if (len < 0) { \
            return -1; \
        } \
    } while (0)

    if (packet == NULL || buf == NULL || bufSize < 64) {
        return -1;
    }

    DrvUart_FormatFixed(vbusText, sizeof(vbusText), packet->adcVbus, 2U);
    DrvUart_FormatFixed(uvText, sizeof(uvText), packet->undervoltageLimit, 2U);
    DrvUart_FormatFixed(ovText, sizeof(ovText), packet->overvoltageLimit, 2U);

    APPEND_FMT("F,%lu,%u,0x%08lX,%u,%u,%u,0x%04X,0x%04X,0x%04X,0x%08lX,%u,%u,%u,%s,%s,%s,%u,%d,%d,%d\n",
               packet->timestamp,
               packet->focState,
               (unsigned long)packet->faultFlags,
               packet->drvCommFaultActive ? 1U : 0U,
               packet->encoderDetected ? 1U : 0U,
               packet->stallOpenLoopActive ? 1U : 0U,
               packet->faultStatus1,
               packet->vgsStatus2,
               packet->drvLastRxFrame,
               (unsigned long)packet->appWarningFlags,
               packet->appFaultCode,
               packet->identifyState,
               packet->identifyError,
               vbusText,
               uvText,
               ovText,
               (unsigned)packet->adcCalibStatus,
               (int)packet->adcOffsetA,
               (int)packet->adcOffsetB,
               (int)packet->adcOffsetC);

#undef APPEND_FMT

    return (len > 0 && len < bufSize) ? len : -1;
}

/**
 * @brief Format detailed fault data
 */
static int16_t DrvUart_FormatFault(const DrvUart_DataPacket_t* packet, uint8_t* buf, uint16_t bufSize)
{
    int16_t len = 0;
    uint16_t fs1, vs2;
    int32_t ia_mA, ib_mA, ic_mA;
    int32_t loopIa_mA, loopIb_mA, loopIc_mA;
    uint32_t vbus_mV;
    uint32_t adcPin_mV;
    char rsVavgText[20];
    char rsIavgText[20];
    char rsImagAvgText[20];
    char rsVecRsText[20];
    char rsPositiveText[20];
    char rsNegativeText[20];
    char pnTargetText[20];
    char pnDeltaMechText[20];
    char pnDeltaElecText[20];
    char pnCalcText[20];
    char verifyAccumText[20];
    char paramRsText[20];
    char paramLdText[20];
    char paramLqText[20];
    char paramKeText[20];
    char paramJText[20];
    char thetaMechText[20];
    char thetaElecText[20];
    char thetaOffsetText[20];
    char thetaZeroText[20];
    char idText[20];
    char iqText[20];
    char adcIdText[20];
    char adcIqText[20];
    char idRefText[20];
    char iqRefText[20];
    char vdText[20];
    char vqText[20];
    char speedLoopRefText[20];
    char speedLoopMechText[20];
    char speedLoopErrorText[20];
    char speedLoopIqMechText[20];
    char speedLoopFrictionText[20];
    char speedLoopIqCmdText[20];
    char speedLoopPIqText[20];
    char speedLoopIIqText[20];
    char positionLoopErrorText[20];
    char positionLoopPdOutText[20];
    char trajCmdText[20];
    char positionPrefRawText[20];
    char positionPrefMappedText[20];
    char positionPrefBeforeText[20];
    char positionPrefAfterText[20];
    char svpwmTaText[20];
    char svpwmTbText[20];
    char svpwmTcText[20];
    char paramRsMaxText[20];
    char lsVrmsText[20];
    char lsIrmsText[20];
    char lsZText[20];
    char lsXlText[20];
    char lsLText[20];
    uint8_t showIdentifyDetail;
    uint8_t activeFault;
    const char *timerDirText;
#define APPEND_FMT(...) \
    do { \
        len = DrvUart_Append(buf, bufSize, len, __VA_ARGS__); \
        if (len < 0) { \
            return -1; \
        } \
    } while (0)
    
    if (packet == NULL || buf == NULL || bufSize < 700) {
        return -1;
    }
    
    fs1 = packet->faultStatus1;
    vs2 = packet->vgsStatus2;
    ia_mA = (int32_t)(packet->adcCurrentA * 1000.0f);
    ib_mA = (int32_t)(packet->adcCurrentB * 1000.0f);
    ic_mA = (int32_t)(packet->adcCurrentC * 1000.0f);
    loopIa_mA = (int32_t)(packet->loopCurrentA * 1000.0f);
    loopIb_mA = (int32_t)(packet->loopCurrentB * 1000.0f);
    loopIc_mA = (int32_t)(packet->loopCurrentC * 1000.0f);
    vbus_mV = (uint32_t)(packet->adcVbus * 1000.0f);
    adcPin_mV = (((uint32_t)packet->adcRawVbus * 3300U) + 2048U) / 4096U;
    DrvUart_FormatFixed(rsVavgText, sizeof(rsVavgText), packet->identifyRsLastVavg, 3U);
    DrvUart_FormatFixed(rsIavgText, sizeof(rsIavgText), packet->identifyRsLastIavg, 3U);
    DrvUart_FormatFixed(rsImagAvgText, sizeof(rsImagAvgText), packet->identifyRsLastImagAvg, 3U);
    DrvUart_FormatFixed(rsVecRsText, sizeof(rsVecRsText), packet->identifyRsLastVecRs, 3U);
    DrvUart_FormatFixed(rsPositiveText, sizeof(rsPositiveText), packet->identifyRsPositive, 3U);
    DrvUart_FormatFixed(rsNegativeText, sizeof(rsNegativeText), packet->identifyRsNegative, 3U);
    DrvUart_FormatFixed(lsVrmsText, sizeof(lsVrmsText), packet->identifyLsVrms, 3U);
    DrvUart_FormatFixed(lsIrmsText, sizeof(lsIrmsText), packet->identifyLsIrms, 3U);
    DrvUart_FormatFixed(lsZText, sizeof(lsZText), packet->identifyLsZ, 3U);
    DrvUart_FormatFixed(lsXlText, sizeof(lsXlText), packet->identifyLsXl, 3U);
    DrvUart_FormatFixed(lsLText, sizeof(lsLText), packet->identifyLsL, 6U);
    DrvUart_FormatFixed(pnTargetText, sizeof(pnTargetText), packet->identifyPnCurrentTarget, 3U);
    DrvUart_FormatFixed(pnDeltaMechText, sizeof(pnDeltaMechText), packet->identifyPnDeltaMech, 3U);
    DrvUart_FormatFixed(pnDeltaElecText, sizeof(pnDeltaElecText), packet->identifyPnDeltaElec, 3U);
    DrvUart_FormatFixed(pnCalcText, sizeof(pnCalcText), packet->identifyPnCalc, 3U);
    DrvUart_FormatFixed(verifyAccumText, sizeof(verifyAccumText), packet->identifyVerifyAccum, 3U);
    DrvUart_FormatFixed(paramRsText, sizeof(paramRsText), packet->paramRs, 3U);
    DrvUart_FormatFixed(paramLdText, sizeof(paramLdText), packet->paramLd, 6U);
    DrvUart_FormatFixed(paramLqText, sizeof(paramLqText), packet->paramLq, 6U);
    DrvUart_FormatFixed(paramKeText, sizeof(paramKeText), packet->paramKe, 6U);
    DrvUart_FormatFixed(paramJText, sizeof(paramJText), packet->paramJ, 6U);
    DrvUart_FormatFixed(thetaMechText, sizeof(thetaMechText), packet->thetaMech, 3U);
    DrvUart_FormatFixed(thetaElecText, sizeof(thetaElecText), packet->thetaElec, 3U);
    DrvUart_FormatFixed(thetaOffsetText, sizeof(thetaOffsetText), packet->paramThetaOffset, 3U);
    DrvUart_FormatFixed(thetaZeroText, sizeof(thetaZeroText), packet->paramThetaMechZero, 3U);
    DrvUart_FormatFixed(idText, sizeof(idText), packet->Id, 3U);
    DrvUart_FormatFixed(iqText, sizeof(iqText), packet->Iq, 3U);
    DrvUart_FormatFixed(adcIdText, sizeof(adcIdText), packet->adcId, 3U);
    DrvUart_FormatFixed(adcIqText, sizeof(adcIqText), packet->adcIq, 3U);
    DrvUart_FormatFixed(idRefText, sizeof(idRefText), packet->Id_ref, 3U);
    DrvUart_FormatFixed(iqRefText, sizeof(iqRefText), packet->Iq_ref, 3U);
    DrvUart_FormatFixed(vdText, sizeof(vdText), packet->Vd, 3U);
    DrvUart_FormatFixed(vqText, sizeof(vqText), packet->Vq, 3U);
    DrvUart_FormatFixed(speedLoopRefText, sizeof(speedLoopRefText), packet->speedLoopRef, 3U);
    DrvUart_FormatFixed(speedLoopMechText, sizeof(speedLoopMechText), packet->speedLoopMech, 3U);
    DrvUart_FormatFixed(speedLoopErrorText, sizeof(speedLoopErrorText), packet->speedLoopError, 3U);
    DrvUart_FormatFixed(speedLoopIqMechText, sizeof(speedLoopIqMechText), packet->speedLoopIqMech, 3U);
    DrvUart_FormatFixed(speedLoopFrictionText, sizeof(speedLoopFrictionText), packet->speedLoopFriction, 3U);
    DrvUart_FormatFixed(speedLoopIqCmdText, sizeof(speedLoopIqCmdText), packet->speedLoopIqCmd, 3U);
    DrvUart_FormatFixed(speedLoopPIqText, sizeof(speedLoopPIqText), packet->speedLoopPIq, 3U);
    DrvUart_FormatFixed(speedLoopIIqText, sizeof(speedLoopIIqText), packet->speedLoopIIq, 3U);
    DrvUart_FormatFixed(positionLoopErrorText, sizeof(positionLoopErrorText), packet->positionLoopError, 3U);
    DrvUart_FormatFixed(positionLoopPdOutText, sizeof(positionLoopPdOutText), packet->positionLoopPdOut, 3U);
    DrvUart_FormatFixed(trajCmdText, sizeof(trajCmdText), packet->trajCmd, 3U);
    DrvUart_FormatFixed(positionPrefRawText, sizeof(positionPrefRawText), packet->positionPrefRaw, 3U);
    DrvUart_FormatFixed(positionPrefMappedText, sizeof(positionPrefMappedText), packet->positionPrefMapped, 3U);
    DrvUart_FormatFixed(positionPrefBeforeText, sizeof(positionPrefBeforeText), packet->positionPrefBefore, 3U);
    DrvUart_FormatFixed(positionPrefAfterText, sizeof(positionPrefAfterText), packet->positionPrefAfter, 3U);
    DrvUart_FormatFixed(svpwmTaText, sizeof(svpwmTaText), packet->svpwmTa, 3U);
    DrvUart_FormatFixed(svpwmTbText, sizeof(svpwmTbText), packet->svpwmTb, 3U);
    DrvUart_FormatFixed(svpwmTcText, sizeof(svpwmTcText), packet->svpwmTc, 3U);
    DrvUart_FormatFixed(paramRsMaxText, sizeof(paramRsMaxText), PARAM_RS_MAX_OHM, 1U);
    showIdentifyDetail = ((packet->focState == (uint8_t)FOC_STATE_PARAM_IDENTIFY) ||
                          (packet->appFaultCode == (uint8_t)FOC_FAULT_PARAM_INVALID) ||
                          (packet->identifyState != (uint8_t)MI_STATE_IDLE) ||
                          (packet->identifyError != (uint8_t)MI_ERR_NONE)) ? 1U : 0U;
    activeFault = ((packet->focState == (uint8_t)FOC_STATE_FAULT) ||
                   (packet->appFaultCode != (uint8_t)FOC_FAULT_NONE) ||
                   (packet->faultFlags != 0U) ||
                   (fs1 != 0U) ||
                   (vs2 != 0U) ||
                   (packet->identifyError != (uint8_t)MI_ERR_NONE)) ? 1U : 0U;
    timerDirText = (packet->adcTimerCountingDown != 0U) ? "down" : "up";
    
    /* Diagnostic title */
    APPEND_FMT(activeFault ?
               "\r\n========== !!! FAULT DETECTED !!! ==========\r\n" :
               "\r\n========== FOC Diagnostic Snapshot ==========\r\n");
    
    /* Timestamp */
    APPEND_FMT("Time: %lu ms\r\n\r\n", packet->timestamp);
    
    /* Avoid float printf on the fault path so the root cause is uploaded quickly. */
    APPEND_FMT("[TLE5012 Encoder]\r\n");
    APPEND_FMT("  Detected: %s\r\n", packet->encoderDetected ? "YES" : "NO");
    APPEND_FMT("  AngleRaw: %5u (0x%04X)\r\n", packet->rawAngle, packet->rawAngle);
    APPEND_FMT("  RawWord:  0x%04X | SafetyWord: 0x%04X\r\n",
               packet->encoderRawWord,
               packet->encoderSafetyWord);
    APPEND_FMT("  CRC:      %s\r\n", packet->crcError ? "ERROR!" : "OK");
    APPEND_FMT("  CRC Rx/Calc: 0x%02X / 0x%02X | BadFrames: %u | GpioDiag: %s\r\n",
               packet->encoderReceivedCrc,
               packet->encoderCalculatedCrc,
               packet->encoderCrcErrorCount,
               packet->encoderGpioDiagActive ? "ON" : "OFF");
    APPEND_FMT("  Safety:   0x%02X\r\n", packet->encoderSafetyStatus);
    APPEND_FMT("  Reset:    %s\r\n\r\n", packet->encoderResetFault ? "FAULT!" : "OK");
    
    APPEND_FMT("[DRV8350S Communication]\r\n");
    APPEND_FMT("  Comm:   %s\r\n",
               packet->drvCommFaultActive ? "Readback INVALID" :
               (packet->drvCommValidated ? "Validated" : "Checking"));
    APPEND_FMT("  RawSPI: 0x%04X\r\n", packet->drvLastRxFrame);
    APPEND_FMT("  CTRL:   0x%04X | OCP: 0x%04X\r\n\r\n", packet->driverCtrl, packet->ocpCtrl);
    APPEND_FMT("  Identified: %s\r\n", packet->motorIdentified ? "YES" : "NO");
    APPEND_FMT("  StallMode:  %s\r\n\r\n", packet->stallModeArmed ? "ARMED" : "OFF");
    APPEND_FMT("  StallOpenLoop: %s\r\n\r\n", packet->stallOpenLoopActive ? "ACTIVE" : "OFF");
    APPEND_FMT("[FOC Application]\r\n");
    APPEND_FMT("  State:    %s\r\n", FOC_App_GetStateString((FOC_AppState_t)packet->focState));
    APPEND_FMT("  Power:   pwm=%u moe=%u mode=%u\r\n",
               packet->pwmEnabled,
               packet->tim1MoeEnabled,
               packet->controlMode);
    if (packet->appWarningFlags != 0U) {
        APPEND_FMT("  Warning:  0x%08lX\r\n", (unsigned long)packet->appWarningFlags);
        if (packet->appWarningFlags & FOC_WARNING_VBUS_UNDERVOLTAGE_BIT) {
            APPEND_FMT("  [WARN]  Bus voltage low (non-trip)\r\n");
        }
        if (packet->appWarningFlags & FOC_WARNING_VBUS_OVERVOLTAGE_BIT) {
            APPEND_FMT("  [WARN]  Bus voltage high (non-trip)\r\n");
        }
    }
    APPEND_FMT("  AppFault: %u (%s)\r\n\r\n",
               packet->appFaultCode,
               FOC_App_GetFaultString((FOC_FaultCode_t)packet->appFaultCode));
    APPEND_FMT("  ThetaDiag: mech=%s rad | elec=%s rad | offset=%s rad | zero=%s rad | Pn=%u | enc_dir=%d\r\n",
               thetaMechText,
               thetaElecText,
               thetaOffsetText,
               thetaZeroText,
               packet->paramPn,
               (int)packet->paramEncoderDir);
    APPEND_FMT("  CurrentDQ: Id=%s A | Iq=%s A | Id_ref=%s A | Iq_ref=%s A | Vd=%s V | Vq=%s V\r\n\r\n",
               idText,
               iqText,
               idRefText,
               iqRefText,
               vdText,
               vqText);
    APPEND_FMT("  AdcDQ:   Id=%s A | Iq=%s A\r\n",
               adcIdText,
               adcIqText);
    APPEND_FMT("  LoopABC: Ia=%ld mA Ib=%ld mA Ic=%ld mA\r\n\r\n",
               (long)loopIa_mA,
               (long)loopIb_mA,
               (long)loopIc_mA);
    APPEND_FMT("  SpeedLoopDiag: ref=%s rad/s | mech=%s rad/s | err=%s rad/s | iq_mech=%s A | friction=%s A | iq_cmd=%s A | p_iq=%s A | i_iq=%s A | i_state=%u\r\n\r\n",
               speedLoopRefText,
               speedLoopMechText,
               speedLoopErrorText,
               speedLoopIqMechText,
               speedLoopFrictionText,
               speedLoopIqCmdText,
               speedLoopPIqText,
               speedLoopIIqText,
               packet->speedLoopIState);
    APPEND_FMT("  PositionLoopDiag: err=%s rad | pd_out=%s rad/s | ramp_ref=%s rad/s | speed_err=%s rad/s | iq_mech=%s A | friction=%s A | iq_cmd=%s A | sat=pd:%u ramp:%u iq+:%u iq-:%u\r\n\r\n",
               positionLoopErrorText,
               positionLoopPdOutText,
               speedLoopRefText,
               speedLoopErrorText,
               speedLoopIqMechText,
               speedLoopFrictionText,
               speedLoopIqCmdText,
               packet->positionLoopPdSat,
               packet->positionLoopRampSat,
               packet->positionLoopIqPosSat,
               packet->positionLoopIqNegSat);
    APPEND_FMT("  TrajDiag: active=%u | cmd=%s rad/s | pd=%s rad/s | cruise=%.3f | hold=%.3f\r\n\r\n",
               packet->trajActive,
               trajCmdText,
               positionLoopPdOutText,
               (double)g_foc_app.position_cruise_speed_radps,
               (double)FOC_POSITION_CRUISE_HOLD_THRESHOLD_RAD);
    APPEND_FMT("  PrefDiag: count=%lu | raw=%s rad | mapped=%s rad | before=%s rad | after=%s rad | user_set=%u\r\n\r\n",
               (unsigned long)packet->positionPrefCmdCount,
               positionPrefRawText,
               positionPrefMappedText,
               positionPrefBeforeText,
               positionPrefAfterText,
               packet->positionPrefUserSet);

    /* FFDiag: 前馈诊断输出 */
    {
        char ffBemfVdText[20], ffBemfVqText[20];
        char ffInertiaIqText[20], ffFrictionIqText[20], ffCoggingIqText[20];
        char ffObserverIqText[20], ffTotalIqText[20];
        DrvUart_FormatFixed(ffBemfVdText, sizeof(ffBemfVdText), packet->ffBemfVd, 3U);
        DrvUart_FormatFixed(ffBemfVqText, sizeof(ffBemfVqText), packet->ffBemfVq, 3U);
        DrvUart_FormatFixed(ffInertiaIqText, sizeof(ffInertiaIqText), packet->ffInertiaIq, 3U);
        DrvUart_FormatFixed(ffFrictionIqText, sizeof(ffFrictionIqText), packet->ffFrictionIq, 3U);
        DrvUart_FormatFixed(ffCoggingIqText, sizeof(ffCoggingIqText), packet->ffCoggingIq, 3U);
        DrvUart_FormatFixed(ffObserverIqText, sizeof(ffObserverIqText), packet->ffObserverIq, 3U);
        DrvUart_FormatFixed(ffTotalIqText, sizeof(ffTotalIqText), packet->ffTotalIq, 3U);
        APPEND_FMT("  FFDiag: total=%s A | bemf(Vd=%s Vq=%s en=%u) | inertia(Iq=%s blk=%u) | friction(Iq=%s en=%u) | cogging(Iq=%s en=%u) | observer(Iq=%s en=%u) | enc_dir_blk=%u\r\n\r\n",
                   ffTotalIqText,
                   ffBemfVdText, ffBemfVqText, packet->ffBemfEnabled,
                   ffInertiaIqText, packet->ffInertiaBlocked,
                   ffFrictionIqText, packet->ffFrictionEnabled,
                   ffCoggingIqText, packet->ffCoggingEnabled,
                   ffObserverIqText, packet->ffObserverEnabled,
                   packet->ffEncDirBlocked);
    }

    /* CurrentLoopDiag: 电流环电压分解诊断 */
    {
        FOC_Handle_t *foc = &g_foc_app.foc;
        char vdRsText[20], vqRsText[20];
        char vdPiText[20], vqPiText[20];
        char vdBemfText[20], vqBemfText[20];
        char vdCmdText[20], vqCmdText[20];
        char vMagText[20], satRatioText[20];

        DrvUart_FormatFixed(vdRsText, sizeof(vdRsText), foc->diag_vd_rs_ff, 4U);
        DrvUart_FormatFixed(vqRsText, sizeof(vqRsText), foc->diag_vq_rs_ff, 4U);
        DrvUart_FormatFixed(vdPiText, sizeof(vdPiText), foc->diag_vd_pi, 4U);
        DrvUart_FormatFixed(vqPiText, sizeof(vqPiText), foc->diag_vq_pi, 4U);
        DrvUart_FormatFixed(vdBemfText, sizeof(vdBemfText), foc->diag_vd_bemf, 4U);
        DrvUart_FormatFixed(vqBemfText, sizeof(vqBemfText), foc->diag_vq_bemf, 4U);
        DrvUart_FormatFixed(vdCmdText, sizeof(vdCmdText), foc->diag_vd_cmd, 4U);
        DrvUart_FormatFixed(vqCmdText, sizeof(vqCmdText), foc->diag_vq_cmd, 4U);
        DrvUart_FormatFixed(vMagText, sizeof(vMagText), foc->diag_v_mag, 4U);
        DrvUart_FormatFixed(satRatioText, sizeof(satRatioText), foc->diag_sat_ratio, 4U);

        APPEND_FMT("  CurrentLoopDiag: RsFF(Vd=%s Vq=%s) | PI(Vd=%s Vq=%s) | "
                   "BEMF(Vd=%s Vq=%s) | PreSat(Vd=%s Vq=%s mag=%s sat=%s)\r\n",
                   vdRsText, vqRsText,
                   vdPiText, vqPiText,
                   vdBemfText, vqBemfText,
                   vdCmdText, vqCmdText,
                   vMagText, satRatioText);

        {
            float ke_used = (foc->bemf_Ke_temp > 0.0f) ? foc->bemf_Ke_temp : foc->bemf_Ke;
            char keText[20], omegaText[20];
            DrvUart_FormatFixed(keText, sizeof(keText), ke_used, 6U);
            DrvUart_FormatFixed(omegaText, sizeof(omegaText), foc->omega_elec_radps, 3U);
            APPEND_FMT("  BEMF Ctrl: user=%u hw=%u blocked=%u | Ke_used=%s | omega_e=%s rad/s\r\n\r\n",
                       foc->bemf_user_enable, foc->bemf_enabled, foc->bemf_blocked,
                       keText, omegaText);
        }
    }

    /* MotionCfg: V5 runtime motion parameters */
    {
        char motionSpeedText[20], motionAccelText[20], motionCruiseText[20];
        DrvUart_FormatFixed(motionSpeedText, sizeof(motionSpeedText),
            g_foc_app.position_speed_limit_radps, 3U);
        DrvUart_FormatFixed(motionAccelText, sizeof(motionAccelText),
            g_foc_app.position_accel_limit_radps2, 3U);
        DrvUart_FormatFixed(motionCruiseText, sizeof(motionCruiseText),
            g_foc_app.position_cruise_speed_radps, 3U);
        APPEND_FMT("  MotionCfg: speed=%s rad/s | accel=%s rad/s^2 | cruise=%s rad/s\r\n\r\n",
                   motionSpeedText, motionAccelText, motionCruiseText);
    }

    if (showIdentifyDetail != 0U) {
        APPEND_FMT("[Motor Identification]\r\n");
        APPEND_FMT("  State:  %u (%s)\r\n",
                   packet->identifyState,
                   DrvUart_IdentifyStateToString(packet->identifyState));
        APPEND_FMT("  Error:  %u (%s)\r\n",
                   packet->identifyError,
                   DrvUart_IdentifyErrorToString(packet->identifyError));
        APPEND_FMT("  RsDiag: Vd_avg=%s V | Id_avg=%s A | I_mag_avg=%s A | Rs_vec=%s Ohm | samples=%lu | Rs+=%s Ohm | Rs-=%s Ohm\r\n",
                   rsVavgText,
                   rsIavgText,
                   rsImagAvgText,
                   rsVecRsText,
                   (unsigned long)packet->identifyRsLastSamples,
                   rsPositiveText,
                   rsNegativeText);
        APPEND_FMT("  LsDiag: Vrms=%s V | Irms=%s A | Z=%s Ohm | Xl=%s Ohm | L=%s H | fallback=%u\r\n",
                   lsVrmsText,
                   lsIrmsText,
                   lsZText,
                   lsXlText,
                   lsLText,
                   (unsigned)packet->identifyLsUsedFallback);
        APPEND_FMT("  PnDiag: target=%s A | dMech=%s rad | dElec=%s rad | PnCalc=%s\r\n",
                   pnTargetText,
                   pnDeltaMechText,
                   pnDeltaElecText,
                   pnCalcText);
        APPEND_FMT("  DirDiag: enc_dir=%d | pn_dir=%d | verify_phase=%u | locked_dir=%d | verify_accum=%s rad | status=%u | rev_fault=%u\r\n",
                   (int)g_foc_app.motor_param.encoder_dir,
                   (int)packet->identifyPnObservedDir,
                   packet->identifyVerifyPhase,
                   (int)packet->identifyVerifyLockedDir,
                   verifyAccumText,
                   g_foc_app.mi_handle.motion_verify_status,
                   g_foc_app.mi_handle.verify_reverse_fault);
        APPEND_FMT("  ParamDiag: invalid=0x%08lX | valid=0x%08lX | Rs=%s Ohm | Ld=%s H | Lq=%s H | Ke=%s | Pn=%u | enc_dir=%d | J=%s\r\n",
                   (unsigned long)packet->paramInvalidFlags,
                   (unsigned long)packet->paramValidFlag,
                   paramRsText,
                   paramLdText,
                   paramLqText,
                   paramKeText,
                   packet->paramPn,
                   (int)packet->paramEncoderDir,
                   paramJText);
        if (packet->paramInvalidFlags & PARAM_INVALID_VALID_FLAG) {
            APPEND_FMT("  [PARAM] valid_flag is not programmed.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_RS) {
            APPEND_FMT("  [PARAM] Rs out of range: expected 0 < Rs <= %s Ohm.\r\n", paramRsMaxText);
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_LD) {
            APPEND_FMT("  [PARAM] Ld out of range: expected 0 < Ld <= 0.01 H.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_LQ) {
            APPEND_FMT("  [PARAM] Lq out of range: expected 0 < Lq <= 0.01 H.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_KE) {
            APPEND_FMT("  [PARAM] Ke out of range: expected 0 <= Ke <= 1.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_PN) {
            APPEND_FMT("  [PARAM] Pn out of range: expected 1..50.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_ENCODER_DIR) {
            APPEND_FMT("  [PARAM] encoder_dir out of range: expected +1 or -1.\r\n");
        }
        if (packet->paramInvalidFlags & PARAM_INVALID_J) {
            APPEND_FMT("  [PARAM] J out of range: expected 0 <= J <= 1.\r\n");
        }
        if (packet->identifyError == (uint8_t)MI_ERR_CURRENT_TOO_LOW) {
            APPEND_FMT("  [FAIL]  \350\257\206\345\210\253\347\224\265\346\265\201\350\277\207\344\275\216\357\274\232\346\243\200\346\237\245\351\251\261\345\212\250\346\230\257\345\220\246\347\234\237\346\255\243\344\275\277\350\203\275\343\200\201\347\233\270\347\272\277\346\230\257\345\220\246\346\216\245\345\245\275\343\200\201\350\257\206\345\210\253\347\224\265\345\216\213/\347\224\265\346\265\201\346\230\257\345\220\246\350\277\207\345\260\217\343\200\202\r\n");
        }
        if (packet->identifyError == (uint8_t)MI_ERR_PN_NOT_CONVERGED) {
            APPEND_FMT("  [FAIL]  \346\236\201\345\257\271\346\225\260\350\257\206\345\210\253\346\234\252\346\224\266\346\225\233\357\274\232\346\243\200\346\237\245\347\274\226\347\240\201\345\231\250\346\226\271\345\220\221\343\200\201\347\233\270\345\272\217\345\222\214\347\224\265\346\234\272\346\230\257\345\220\246\347\234\237\345\256\236\350\275\254\345\212\250\343\200\202\r\n");
        }
        if (packet->identifyError == (uint8_t)MI_ERR_TIMEOUT) {
            APPEND_FMT("  [FAIL]  \350\257\206\345\210\253\350\266\205\346\227\266\357\274\232\346\243\200\346\237\245\347\224\265\346\234\272\346\230\257\345\220\246\345\215\241\344\275\217\343\200\201\344\276\233\347\224\265\345\222\214PWM\350\276\223\345\207\272\346\230\257\345\220\246\346\255\243\345\270\270\343\200\202\r\n");
        }
        if (packet->identifyError == (uint8_t)MI_ERR_PHASE_SEQUENCE) {
            APPEND_FMT("  [FAIL]  \347\233\270\345\272\217\346\210\226\347\274\226\347\240\201\345\231\250\346\226\271\345\220\221\345\274\202\345\270\270\357\274\232\346\243\200\346\237\245UVW\347\233\270\347\272\277\351\241\272\345\272\217\343\200\201\347\224\265\346\265\201\351\207\207\346\240\267\347\233\270\345\272\217\345\222\214\347\274\226\347\240\201\345\231\250\345\256\211\350\243\205\346\226\271\345\220\221\343\200\202\r\n");
        }
        APPEND_FMT("\r\n");
    }

    if (packet->faultFlags & (DRV8350S_FW_UART_RX_FAULT_BIT |
                              DRV8350S_FW_ADC_CAL_FAULT_BIT |
                              DRV8350S_FW_TIM_TRIG_FAULT_BIT |
                              DRV8350S_FW_ADC_DMA_FAULT_BIT)) {
        APPEND_FMT("[Startup Bring-up]\r\n");
        if (packet->faultFlags & DRV8350S_FW_UART_RX_FAULT_BIT) {
            APPEND_FMT("  [BOOT]  USART1 RX DMA/IDLE chain failed to start.\r\n");
        }
        if (packet->faultFlags & DRV8350S_FW_ADC_CAL_FAULT_BIT) {
            APPEND_FMT("  [BOOT]  ADC calibration or sampling init failed.\r\n");
        }
        if (packet->faultFlags & DRV8350S_FW_TIM_TRIG_FAULT_BIT) {
            APPEND_FMT("  [BOOT]  TIM1 base/OC4 ADC trigger failed.\r\n");
        }
        if (packet->faultFlags & DRV8350S_FW_ADC_DMA_FAULT_BIT) {
            APPEND_FMT("  [BOOT]  ADC DMA start failed.\r\n");
        }
        APPEND_FMT("\r\n");
    }

    if (packet->faultFlags & DRV8350S_COMM_FAULT_BIT) {
        APPEND_FMT("  [COMM]  SPI readback invalid; DRV SDO may be Hi-Z or bus timing is still wrong.\r\n");
        APPEND_FMT("  FAULT1: 0x%04X | VGS2: 0x%04X\r\n\r\n", fs1, vs2);
    } else {
        /* DRV8350S fault details */
        APPEND_FMT("[DRV8350S Fault Details]\r\n");
        APPEND_FMT("  FAULT1: 0x%04X | VGS2: 0x%04X\r\n\r\n", fs1, vs2);
        if (packet->latchedFaultFlags != 0U) {
            APPEND_FMT("  Latched: flags=0x%08lX | FAULT1=0x%04X | VGS2=0x%04X\r\n\r\n",
                       (unsigned long)packet->latchedFaultFlags,
                       packet->latchedFaultStatus1,
                       packet->latchedVgsStatus2);
        }
        
        /* FAULT_STATUS_1 fault bits */
        if (fs1 & (1U << 10))
            APPEND_FMT("  [FAULT] General Fault\r\n");
        if (fs1 & (1U << 9))
            APPEND_FMT("  [CRIT]  VDS Overcurrent!\r\n");
        if (fs1 & (1U << 8))
            APPEND_FMT("  [CRIT]  Gate Drive Fault!\r\n");
        if (fs1 & (1U << 7))
            APPEND_FMT("  [CRIT]  Undervoltage Lockout!\r\n");
        if (fs1 & (1U << 6))
            APPEND_FMT("  [CRIT]  Overtemperature Shutdown!\r\n");
        
        /* VDS overcurrent phase details */
        if (fs1 & 0x003F) {
            APPEND_FMT("\r\n  VDS OCP Phase:\r\n");
            if (fs1 & (1U << 5)) APPEND_FMT("    - A High-Side\r\n");
            if (fs1 & (1U << 4)) APPEND_FMT("    - A Low-Side\r\n");
            if (fs1 & (1U << 3)) APPEND_FMT("    - B High-Side\r\n");
            if (fs1 & (1U << 2)) APPEND_FMT("    - B Low-Side\r\n");
            if (fs1 & (1U << 1)) APPEND_FMT("    - C High-Side\r\n");
            if (fs1 & (1U << 0)) APPEND_FMT("    - C Low-Side\r\n");
        }
        
        /* VGS_STATUS_2 fault bits */
        /* Note: DRV8350S does NOT have CSA, Bit 10-8 are Reserved */
        if (vs2 & (1U << 7))
            APPEND_FMT("  [WARN]  Overtemperature Warning\r\n");
        if (vs2 & (1U << 6))
            APPEND_FMT("  [WARN]  Gate Drive UVLO\r\n");
        
        /* VGS fault phase details */
        if (vs2 & 0x003F) {
            APPEND_FMT("\r\n  VGS Fault Phase:\r\n");
            if (vs2 & (1U << 5)) APPEND_FMT("    - A High-Side\r\n");
            if (vs2 & (1U << 4)) APPEND_FMT("    - A Low-Side\r\n");
            if (vs2 & (1U << 3)) APPEND_FMT("    - B High-Side\r\n");
            if (vs2 & (1U << 2)) APPEND_FMT("    - B Low-Side\r\n");
            if (vs2 & (1U << 1)) APPEND_FMT("    - C High-Side\r\n");
            if (vs2 & (1U << 0)) APPEND_FMT("    - C Low-Side\r\n");
        }
    }

    APPEND_FMT("\r\n[ADC Sampling]\r\n");
    APPEND_FMT("  Trigger: %s\r\n", packet->adcTriggerSource);
    APPEND_FMT("  Samp:    Iabc=32.5 cyc | Vbus=16.5 cyc\r\n");
    APPEND_FMT("  Frame:   seq=%lu age=%lu miss=%lu invalid=%lu\r\n",
               packet->adcFrameSequence,
               packet->adcFrameAgeCycles,
               packet->adcSampleMissCount,
               packet->adcInvalidWindowCount);
    APPEND_FMT("  Gate:    valid_low_side=%lu invalid_low_side=%lu forced_zero_win=%lu\r\n",
               packet->adcValidLowSideCount,
               packet->adcInvalidLowSideCount,
               packet->adcForcedLowSideCount);
    APPEND_FMT("  Raw:     A=%4u B=%4u C=%4u\r\n",
               packet->adcRawCurrentA,
               packet->adcRawCurrentB,
               packet->adcRawCurrentC);
    APPEND_FMT("  Offset:  A=%4d B=%4d C=%4d\r\n",
               (int)packet->adcOffsetA,
               (int)packet->adcOffsetB,
               (int)packet->adcOffsetC);
    APPEND_FMT("  Delta:   A=%+d B=%+d C=%+d LSB\r\n",
               (int)packet->adcDeltaA,
               (int)packet->adcDeltaB,
               (int)packet->adcDeltaC);
    APPEND_FMT("  PWM:     ARR=%u | CCR A/B/C=%u/%u/%u | TRIG=%u | CNT=%u | DIR=%s\r\n",
               (unsigned)packet->adcPwmPeriod,
               (unsigned)packet->adcPwmCompareA,
               (unsigned)packet->adcPwmCompareB,
               (unsigned)packet->adcPwmCompareC,
               (unsigned)packet->adcTriggerCompare,
               (unsigned)packet->adcTimerCount,
               timerDirText);
    APPEND_FMT("  Win:     low_side_valid A=%u B=%u C=%u | sector=%u | Ta/Tb/Tc=%s/%s/%s\r\n",
               (unsigned)packet->adcLowSideValidA,
               (unsigned)packet->adcLowSideValidB,
               (unsigned)packet->adcLowSideValidC,
               (unsigned)packet->svpwmSector,
               svpwmTaText,
               svpwmTbText,
               svpwmTcText);
    APPEND_FMT("  RawVbus: %4u\r\n", packet->adcRawVbus);
    APPEND_FMT("  Curr:    Ia=%ld mA Ib=%ld mA Ic=%ld mA\r\n",
               (long)ia_mA,
               (long)ib_mA,
               (long)ic_mA);
    APPEND_FMT("  ADC Pin: %lu mV\r\n", (unsigned long)adcPin_mV);
    APPEND_FMT("  Vbus:    %lu mV\r\n", (unsigned long)vbus_mV);
    
    if (activeFault != 0U) {
        /* Suggested actions */
        APPEND_FMT("\r\n>>> ACTION REQUIRED <<<\r\n");
        APPEND_FMT("  1. Disable PWM immediately\r\n");
        APPEND_FMT("  2. Check power supply\r\n");
        APPEND_FMT("  3. Verify MOSFETs status\r\n");
    } else {
        APPEND_FMT("\r\n>>> DIAGNOSTIC SNAPSHOT <<<\r\n");
        APPEND_FMT("  No active shutdown fault detected.\r\n");
    }
    
    APPEND_FMT("=============================================\r\n");

#undef APPEND_FMT
    
    return (len > 0 && len < bufSize) ? len : -1;
}

static void DrvUart_QueueFaultDetail(const DrvUart_DataPacket_t* packet, uint32_t currentTime)
{
    int16_t len;

    (void)currentTime;

    if (packet == NULL) {
        return;
    }

    len = DrvUart_FormatFault(packet, s_faultDetailBuf, DRV_UART_BUF_SIZE);
    if (len > 0) {
        s_faultDetailLen = (uint16_t)len;
        s_faultDetailOffset = 0U;
    } else {
        s_stats.txErrors++;
        s_faultDetailLen = (uint16_t)snprintf((char*)s_faultDetailBuf,
                                               DRV_UART_BUF_SIZE,
                                               "FAULT_DETAIL_FORMAT_ERROR,len=%d,buf=%u\r\n",
                                               (int)len,
                                               (unsigned)DRV_UART_BUF_SIZE);
        s_faultDetailOffset = 0U;
    }
}

/* Free space in ring buffer */
static uint16_t UartTx_FreeSpace(void)
{
    if (s_txRingHead >= s_txRingTail) {
        return (uint16_t)(UART_TX_RING_SIZE - (s_txRingHead - s_txRingTail) - 1U);
    }
    return (uint16_t)(s_txRingTail - s_txRingHead - 1U);
}

/* Enqueue data into ring buffer with priority-based admission */
static bool UartTx_Enqueue(const uint8_t *data, uint16_t len, UartTxPrio prio)
{
    uint16_t free_space;
    uint16_t i;

    if (data == NULL || len == 0U || len >= UART_TX_RING_SIZE) {
        return false;
    }

    __disable_irq();
    free_space = UartTx_FreeSpace();

    /* Admission control */
    if (prio == UART_PRIO_P2 && free_space < (UART_TX_P1_RESERVE + len)) {
        s_txDropCount[2]++;
        __enable_irq();
        return false;  /* drop telemetry under backpressure */
    }
    if (prio == UART_PRIO_P1 && free_space < (UART_TX_P0_RESERVE + len)) {
        s_txDropCount[1]++;
        __enable_irq();
        return false;  /* drop DIAG only when nearly full */
    }
    /* P0 always accepted — if ring full, we spin briefly (should never happen) */

    for (i = 0U; i < len; i++) {
        s_txRing[s_txRingHead] = data[i];
        s_txRingHead = (uint16_t)((s_txRingHead + 1U) % UART_TX_RING_SIZE);
    }
    __enable_irq();

    return true;
}

/* Start IT transmission from ring buffer. Call with IRQ disabled or from ISR. */
static void UartTx_StartIT(void)
{
    uint16_t avail;
    uint16_t chunk;
    uint16_t i;

    if (s_txITActive || s_huart == NULL) {
        return;
    }

    if (s_txRingHead == s_txRingTail) {
        return;  /* empty */
    }

    /* Calculate contiguous readable chunk from tail */
    if (s_txRingHead > s_txRingTail) {
        avail = (uint16_t)(s_txRingHead - s_txRingTail);
    } else {
        avail = (uint16_t)(UART_TX_RING_SIZE - s_txRingTail);
    }

    if (avail == 0U) {
        return;
    }

    /* Copy to flat TX buffer for HAL IT (HAL expects contiguous buffer) */
    chunk = (avail > DRV_UART_BUF_SIZE) ? (uint16_t)DRV_UART_BUF_SIZE : avail;
    for (i = 0U; i < chunk; i++) {
        s_txBuf[i] = s_txRing[s_txRingTail];
        s_txRingTail = (uint16_t)((s_txRingTail + 1U) % UART_TX_RING_SIZE);
    }

    s_txITActive = true;
    if (HAL_UART_Transmit_IT(s_huart, s_txBuf, chunk) != HAL_OK) {
        s_txITActive = false;
        s_stats.txErrors++;
    }
}

/* Called from main loop to kick IT if data is waiting */
static void UartTx_Service(void)
{
    if (!s_txITActive && s_txRingHead != s_txRingTail) {
        __disable_irq();
        UartTx_StartIT();
        __enable_irq();
    }
}

/**
 * @brief Send data via ring buffer with priority (replaces blocking send)
 */
static bool DrvUart_StartSend(uint16_t len)
{
    if (s_huart == NULL || len == 0U || len > DRV_UART_BUF_SIZE) {
        return false;
    }

    if (!UartTx_Enqueue(s_txBuf, len, UART_PRIO_P2)) {
        return false;
    }

    /* Kick IT from main context */
    if (!s_txITActive) {
        __disable_irq();
        UartTx_StartIT();
        __enable_irq();
    }
    return true;
}

/* Send with explicit priority (for command echo, DIAG, etc.) */
static bool DrvUart_StartSendPrio(uint16_t len, UartTxPrio prio)
{
    if (s_huart == NULL || len == 0U || len > DRV_UART_BUF_SIZE) {
        return false;
    }

    if (!UartTx_Enqueue(s_txBuf, len, prio)) {
        return false;
    }

    if (!s_txITActive) {
        __disable_irq();
        UartTx_StartIT();
        __enable_irq();
    }
    return true;
}

static int16_t DrvUart_Append(uint8_t* buf, uint16_t bufSize, int16_t len, const char* fmt, ...)
{
    int written;
    va_list args;

    if ((buf == NULL) || (fmt == NULL) || (len < 0) || ((uint16_t)len >= bufSize)) {
        return -1;
    }

    va_start(args, fmt);
    written = vsnprintf((char*)buf + len, (size_t)(bufSize - (uint16_t)len), fmt, args);
    va_end(args);

    if ((written < 0) || (written >= (int)(bufSize - (uint16_t)len))) {
        return -1;
    }

    return (int16_t)(len + written);
}

void DrvUart_FormatFixed(char* dst, uint16_t dstSize, float value, uint8_t decimals)
{
    uint32_t scale = 1U;
    uint32_t whole;
    uint32_t frac;
    uint8_t i;
    uint8_t negative;
    float absValue;

    if ((dst == NULL) || (dstSize == 0U)) {
        return;
    }

    for (i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }

    negative = (value < 0.0f) ? 1U : 0U;
    absValue = negative ? -value : value;
    whole = (uint32_t)absValue;
    frac = (uint32_t)(((absValue - (float)whole) * (float)scale) + 0.5f);
    if (frac >= scale) {
        whole += 1U;
        frac -= scale;
    }

    if (decimals == 0U) {
        (void)snprintf(dst, dstSize, negative ? "-%lu" : "%lu", (unsigned long)whole);
    } else {
        (void)snprintf(dst,
                       dstSize,
                       negative ? "-%lu.%0*lu" : "%lu.%0*lu",
                       (unsigned long)whole,
                       (int)decimals,
                       (unsigned long)frac);
    }
}

static const char* DrvUart_IdentifyStateToString(uint8_t state)
{
    switch ((MI_State_t)state) {
        case MI_STATE_IDLE:          return "IDLE";
        case MI_STATE_PN_IDENTIFY:   return "PN_VERIFY";
        case MI_STATE_RS_IDENTIFY:   return "RS_IDENTIFY";
        case MI_STATE_LS_IDENTIFY:   return "LS_IDENTIFY";
        case MI_STATE_KE_IDENTIFY:   return "KE_IDENTIFY";
        case MI_STATE_J_IDENTIFY:    return "J_IDENTIFY";
        case MI_STATE_ENCODER_ALIGN: return "ENCODER_ALIGN";
        case MI_STATE_MOTION_VERIFY: return "MOTION_VERIFY";
        case MI_STATE_COMPLETE:      return "COMPLETE";
        case MI_STATE_ERROR:         return "ERROR";
        default:                     return "UNKNOWN";
    }
}

static const char* DrvUart_IdentifyErrorToString(uint8_t error)
{
    switch ((MI_ErrorCode_t)error) {
        case MI_ERR_NONE:             return "MI_ERR_NONE / \346\227\240\351\224\231\350\257\257";
        case MI_ERR_IN_PROGRESS:      return "MI_ERR_IN_PROGRESS / \350\257\206\345\210\253\344\270\255";
        case MI_ERR_MOTOR_MOVING:     return "MI_ERR_MOTOR_MOVING / \347\224\265\346\234\272\347\247\273\345\212\250";
        case MI_ERR_RS_NOT_CONVERGED: return "MI_ERR_RS_NOT_CONVERGED / Rs\346\234\252\346\224\266\346\225\233";
        case MI_ERR_LS_NOT_CONVERGED: return "MI_ERR_LS_NOT_CONVERGED / Ls\346\234\252\346\224\266\346\225\233";
        case MI_ERR_KE_NOT_CONVERGED: return "MI_ERR_KE_NOT_CONVERGED / Ke\346\234\252\346\224\266\346\225\233";
        case MI_ERR_PN_NOT_CONVERGED: return "MI_ERR_PN_VERIFY_FAILED / \346\236\201\345\257\271\346\225\260\351\252\214\350\257\201\345\244\261\350\264\245";
        case MI_ERR_J_NOT_CONVERGED:  return "MI_ERR_J_NOT_CONVERGED / J\346\234\252\346\224\266\346\225\233";
        case MI_ERR_CURRENT_TOO_LOW:  return "MI_ERR_CURRENT_TOO_LOW / \350\257\206\345\210\253\347\224\265\346\265\201\350\277\207\344\275\216";
        case MI_ERR_CURRENT_TOO_HIGH: return "MI_ERR_CURRENT_TOO_HIGH / \350\257\206\345\210\253\347\224\265\346\265\201\350\277\207\351\253\230";
        case MI_ERR_ENCODER_INVALID:  return "MI_ERR_ENCODER_INVALID / \347\274\226\347\240\201\345\231\250\346\227\240\346\225\210";
        case MI_ERR_TIMEOUT:          return "MI_ERR_TIMEOUT / \350\257\206\345\210\253\350\266\205\346\227\266";
        case MI_ERR_PHASE_SEQUENCE:   return "MI_ERR_PHASE_SEQUENCE / \347\233\270\345\272\217\346\210\226\347\274\226\347\240\201\345\231\250\346\226\271\345\220\221\345\274\202\345\270\270";
        default:                      return "MI_ERR_UNKNOWN / \346\234\252\347\237\245\351\224\231\350\257\257";
    }
}

/**
 * @brief Add fault to history
 */
static void DrvUart_AddFaultHistory(const DrvUart_DataPacket_t* packet)
{
    if (packet == NULL) {
        return;
    }
    
    memcpy(&s_faultHistory[s_faultHistoryHead].data, packet, sizeof(DrvUart_DataPacket_t));
    s_faultHistory[s_faultHistoryHead].valid = 1;
    
    s_faultHistoryHead = (s_faultHistoryHead + 1) % DRV_FAULT_HISTORY_SIZE;
    
    if (s_faultHistoryCount < DRV_FAULT_HISTORY_SIZE) {
        s_faultHistoryCount++;
    }
    
    /* Save the latest fault snapshot */
    memcpy(&s_lastFault, packet, sizeof(DrvUart_DataPacket_t));
}

/**
 * @brief Main UART telemetry service
 */
void DrvUart_Process(void)
{
    uint32_t currentTime;
    DrvUart_DataPacket_t packet;
    int16_t len;
    bool isNewFault;
    bool faultActive;
    uint16_t chunkLen;
    uint8_t currentAppFaultCode;
    uint8_t currentIdentifyState;
    uint8_t currentIdentifyError;
    
    if (!s_enabled || s_huart == NULL || s_drvHandle == NULL) {
        return;
    }
    
    currentTime = HAL_GetTick();

    if (s_faultDetailOffset < s_faultDetailLen) {
        chunkLen = (uint16_t)(s_faultDetailLen - s_faultDetailOffset);
        if (chunkLen > DRV_UART_FAULT_DETAIL_CHUNK_SIZE) {
            chunkLen = DRV_UART_FAULT_DETAIL_CHUNK_SIZE;
        }

        memcpy(s_txBuf, &s_faultDetailBuf[s_faultDetailOffset], chunkLen);
        if (DrvUart_StartSendPrio(chunkLen, UART_PRIO_P0)) {
            s_faultDetailOffset += chunkLen;
            s_stats.totalUploads++;
            s_stats.lastUploadTime = currentTime;
            s_lastUploadTime = currentTime;
            if (s_faultDetailOffset >= s_faultDetailLen) {
                s_faultDetailLen = 0U;
                s_faultDetailOffset = 0U;
            }
        }
        return;
    }
    
    currentAppFaultCode = (uint8_t)g_foc_app.fault_code;
    currentIdentifyState = (uint8_t)g_foc_app.mi_handle.state;
    currentIdentifyError = (uint8_t)g_foc_app.mi_handle.error_code;
    faultActive = ((s_drvHandle->runtime.faultFlags != 0U) ||
                   (s_drvHandle->runtime.latchedFaultFlags != 0U) ||
                   (s_drvHandle->runtime.isFaultActive != 0U) ||
                   (currentAppFaultCode != (uint8_t)FOC_FAULT_NONE) ||
                   (g_foc_app.state == FOC_STATE_FAULT));

    /* Detect new fault roots; do not periodically spam detailed logs. */
    isNewFault = faultActive &&
                 (((s_drvHandle->runtime.faultFlags | s_drvHandle->runtime.latchedFaultFlags) != s_lastFaultFlags) ||
                  (currentAppFaultCode != s_lastAppFaultCode) ||
                  (currentIdentifyState != s_lastIdentifyState) ||
                  (currentIdentifyError != s_lastIdentifyError) ||
                  (s_lastFaultActive == 0U));
    
    if (isNewFault) {
        /* New fault root, upload immediately. */
        DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
        
        len = DrvUart_FormatFaultSummary(&packet, s_txBuf, DRV_UART_BUF_SIZE);
        if (len > 0 && DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P0)) {
            DrvUart_AddFaultHistory(&packet);
            s_stats.faultUploads++;
            s_stats.totalUploads++;
            s_stats.lastUploadTime = currentTime;
            s_lastUploadTime = currentTime;
            DrvUart_QueueFaultDetail(&packet, currentTime);
            /* Update only after the first summary enters the send path. */
            s_lastFaultFlags = s_drvHandle->runtime.faultFlags | s_drvHandle->runtime.latchedFaultFlags;
            s_lastAppFaultCode = currentAppFaultCode;
            s_lastIdentifyState = currentIdentifyState;
            s_lastIdentifyError = currentIdentifyError;
            s_lastFaultActive = 1U;
        }
    }
    else if (faultActive) {
        /* Active faults only resend short summaries; details are root-change or CMD:FAULT_DETAIL driven. */
        if ((currentTime - s_stats.lastUploadTime) >= 500 && s_faultDetailOffset >= s_faultDetailLen) {
            DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
            len = DrvUart_FormatFaultSummary(&packet, s_txBuf, DRV_UART_BUF_SIZE);
            if (len > 0 && DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P0)) {
                s_stats.totalUploads++;
                s_stats.lastUploadTime = currentTime;
                s_lastUploadTime = currentTime;
            }
        }
    }

    if (s_faultDetailLen == 0U && (currentTime - s_lastPhaseCurrentUploadTime) >= DRV_PHASE_CURRENT_UPLOAD_INTERVAL_MS) {
        DrvUart_CollectData(&packet, DRV_PKT_TYPE_NORMAL);
        len = DrvUart_FormatPhaseCurrent(&packet, s_txBuf, DRV_UART_BUF_SIZE);
        if (len > 0) {
            if (DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P2)) {
                s_stats.totalUploads++;
                s_lastPhaseCurrentUploadTime = currentTime;
            }
        }
    }

    if (s_faultDetailLen == 0U && (currentTime - s_lastUploadTime) >= s_uploadInterval) {
        /* Runtime telemetry continues in fault state so the GUI can still plot angle/current. */
        DrvUart_CollectData(&packet, DRV_PKT_TYPE_NORMAL);

        len = DrvUart_FormatNormal(&packet, s_txBuf, DRV_UART_BUF_SIZE);
        if (len > 0) {
            if (DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P2)) {
                s_stats.totalUploads++;
                s_stats.lastUploadTime = currentTime;
                s_lastUploadTime = currentTime;
            }
        }
    }

    /* Clear fault-root history after recovery. */
    if (s_lastFaultActive != 0U && !faultActive) {
        s_lastFaultFlags = 0U;
        s_lastAppFaultCode = 0U;
        s_lastIdentifyState = 0U;
        s_lastIdentifyError = 0U;
        s_lastFaultActive = 0U;
        s_faultDetailLen = 0U;
        s_faultDetailOffset = 0U;
    }

    /* Service ring buffer: kick IT if data waiting and IT idle */
    UartTx_Service();
}

/**
 * @brief Upload current data immediately in blocking mode
 */
void DrvUart_UploadImmediate(void)
{
    DrvUart_DataPacket_t packet;
    int16_t len;
    uint32_t waitStart;
    
    if (s_huart == NULL || s_drvHandle == NULL) {
        return;
    }
    
    /* Wait for the current transfer with a timeout. */
    waitStart = HAL_GetTick();
    while (s_txITActive) {
        if ((HAL_GetTick() - waitStart) > 100U) {
            s_stats.txErrors++;
            return;
        }
    }
    
    DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
    len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
    
    if (len > 0) {
        DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P1);
    }
}

/**
 * @brief Force a detailed fault upload
 */
bool DrvUart_UploadFault(void)
{
    DrvUart_DataPacket_t packet;
    int16_t len;
    
    if (s_huart == NULL || s_drvHandle == NULL) {
        return false;
    }
    
    DrvUart_CollectData(&packet, DRV_PKT_TYPE_FAULT);
    DrvUart_AddFaultHistory(&packet);
    
    len = DrvUart_FormatFault(&packet, s_txBuf, DRV_UART_BUF_SIZE);
    if (len > 0) {
        return DrvUart_StartSend(len);
    }
    
    return false;
}

/**
 * @brief Upload J/B identification diagnostic compact line
 */
bool DrvUart_UploadJDiag(void)
{
    extern FOC_AppHandle_t g_foc_app;
    int16_t len;
    char jText[20], bText[20], tcText[20];

    if (s_huart == NULL || s_drvHandle == NULL) {
        return false;
    }

    if (s_txITActive) {
        return false;
    }

    DrvUart_FormatFixed(jText, sizeof(jText), g_foc_app.motor_param.J, 8U);
    DrvUart_FormatFixed(bText, sizeof(bText), g_foc_app.motor_param.B, 8U);
    DrvUart_FormatFixed(tcText, sizeof(tcText), g_foc_app.motor_param.Tc, 6U);

#if FOC_FF_ENABLE_COGGING
    {
        char cogMinText[16], cogMaxText[16];
        uint16_t cog_nonzero = 0U;
        float cogMin = 0.0f, cogMax = 0.0f;
        uint16_t i;
        if (g_foc_app.cogging_lut.valid && g_foc_app.cogging_lut.valid_size > 0U) {
            cogMin = cogMax = g_foc_app.cogging_lut.table[0];
            for (i = 1U; i < g_foc_app.cogging_lut.valid_size; i++) {
                if (fabsf(g_foc_app.cogging_lut.table[i]) > 1e-6f) cog_nonzero++;
                if (g_foc_app.cogging_lut.table[i] < cogMin) cogMin = g_foc_app.cogging_lut.table[i];
                if (g_foc_app.cogging_lut.table[i] > cogMax) cogMax = g_foc_app.cogging_lut.table[i];
            }
        }
        char cogGainText[16], cogPhaseText[16];
        DrvUart_FormatFixed(cogMinText, sizeof(cogMinText), cogMin, 4U);
        DrvUart_FormatFixed(cogMaxText, sizeof(cogMaxText), cogMax, 4U);
        DrvUart_FormatFixed(cogGainText, sizeof(cogGainText),
                            g_foc_app.cogging_lut.gain, 3U);
        DrvUart_FormatFixed(cogPhaseText, sizeof(cogPhaseText),
                            g_foc_app.cogging_lut.phase_offset_rad * 180.0f / FOC_PI, 1U);
        len = snprintf((char*)s_txBuf, DRV_UART_BUF_SIZE,
                       "JDIAG,v6,J=%s,B=%s,Tc=%s,enc=%d,valid=0x%08lX,cog_valid=%u,cog_size=%u,cog_bins=%u,cog_save=%u,cog_gain=%s,cog_phase=%s,cog_min=%s,cog_max=%s\r\n",
                       jText, bText, tcText,
                       (int)g_foc_app.motor_param.encoder_dir,
                       (unsigned long)g_foc_app.motor_param.valid_flag,
                       (unsigned)g_foc_app.cogging_lut.valid,
                       (unsigned)g_foc_app.cogging_lut.valid_size,
                       (unsigned)cog_nonzero,
                       (unsigned)g_foc_app.cogging_lut.save_attempted,
                       cogGainText, cogPhaseText,
                       cogMinText, cogMaxText);
    }
#else
    len = snprintf((char*)s_txBuf, DRV_UART_BUF_SIZE,
                   "JDIAG,J=%s,B=%s,Tc=%s,enc=%d,valid=0x%08lX\r\n",
                   jText, bText, tcText,
                   (int)g_foc_app.motor_param.encoder_dir,
                   (unsigned long)g_foc_app.motor_param.valid_flag);
#endif

    if (len > 0 && len < DRV_UART_BUF_SIZE) {
        return DrvUart_StartSend((uint16_t)len);
    }

    return false;
}

/**
 * @brief Query current P0 cogging configuration
 */
void DrvUart_QueryCogCfg(void)
{
    extern FOC_AppHandle_t g_foc_app;
    char gainText[16], phaseText[16];
    int16_t len;

    if (s_huart == NULL || s_txITActive) {
        return;
    }

    DrvUart_FormatFixed(gainText, sizeof(gainText),
                        g_foc_app.cogging_lut.gain, 3U);
    DrvUart_FormatFixed(phaseText, sizeof(phaseText),
                        g_foc_app.cogging_lut.phase_offset_rad * 180.0f / FOC_PI, 1U);

    len = (int16_t)snprintf((char*)s_txBuf, DRV_UART_BUF_SIZE,
                            "COG_CFG,gain=%s,phase_deg=%s\r\n",
                            gainText, phaseText);
    if (len > 0 && len < DRV_UART_BUF_SIZE) {
        DrvUart_StartSend((uint16_t)len);
    }
}

/**
 * @brief UART transmit complete callback
 */
void DrvUart_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == s_huart) {
        s_txITActive = false;
        s_stats.isTxBusy = 0;
        /* Dequeue next chunk from ring buffer */
        UartTx_StartIT();
    }
}

/**
 * @brief Send text with P0 priority (command echo — never dropped)
 */
void DrvUart_SendTextP0(const char *text)
{
    size_t len;
    if (text == NULL || s_huart == NULL) return;
    len = strlen(text);
    if (len == 0 || len > DRV_UART_BUF_SIZE) return;
    memcpy(s_txBuf, text, len);
    DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P0);
}

/**
 * @brief Send text with P1 priority (DIAG output)
 */
void DrvUart_SendTextP1(const char *text)
{
    size_t len;
    if (text == NULL || s_huart == NULL) return;
    len = strlen(text);
    if (len == 0 || len > DRV_UART_BUF_SIZE) return;
    memcpy(s_txBuf, text, len);
    DrvUart_StartSendPrio((uint16_t)len, UART_PRIO_P1);
}

/**
 * @brief Enable or disable upload
 */
void DrvUart_SetEnable(bool enable)
{
    s_enabled = enable;
}

/**
 * @brief Get current upload interval in ms
 */
uint32_t DrvUart_GetInterval(void)
{
    return s_uploadInterval;
}

/**
 * @brief Check if telemetry is enabled
 */
bool DrvUart_IsEnabled(void)
{
    return s_enabled;
}

/**
 * @brief Set upload interval
 */
void DrvUart_SetInterval(uint32_t intervalMs)
{
    if (intervalMs >= 10 && intervalMs <= 10000) {
        s_uploadInterval = intervalMs;
    }
}

/**
 * @brief Get upload statistics
 */
void DrvUart_GetStatistics(DrvUart_Statistics_t* stats)
{
    if (stats != NULL) {
        memcpy(stats, (void*)&s_stats, sizeof(DrvUart_Statistics_t));
    }
}

/**
 * @brief Clear fault history
 */
void DrvUart_ClearFaultHistory(void)
{
    s_faultHistoryHead = 0;
    s_faultHistoryCount = 0;
    memset(s_faultHistory, 0, sizeof(s_faultHistory));
}

/**
 * @brief Get fault history count
 */
uint8_t DrvUart_GetFaultHistoryCount(void)
{
    return s_faultHistoryCount;
}

/**
 * @brief Check whether a fault is active
 */
bool DrvUart_HasActiveFault(void)
{
    if (s_drvHandle == NULL) {
        return false;
    }
    return ((s_drvHandle->runtime.isFaultActive != 0U) ||
            (s_drvHandle->runtime.faultFlags != 0U) ||
            (s_drvHandle->runtime.latchedFaultFlags != 0U) ||
            (g_foc_app.fault_code != FOC_FAULT_NONE) ||
            (g_foc_app.state == FOC_STATE_FAULT));
}

/**
 * @brief Get latest fault snapshot
 */
void DrvUart_GetLastFault(DrvUart_DataPacket_t* packet)
{
    if (packet != NULL && s_lastFault.timestamp != 0) {
        memcpy(packet, &s_lastFault, sizeof(DrvUart_DataPacket_t));
    }
}
