/**
 * @file    can_protocol.c
 * @brief   CAN v1.0 protocol implementation for FDCAN1
 *
 * Protocol reference: docs/CAN_PROTOCOL.md v1.0.
 */

#include "can_protocol.h"
#include "fdcan.h"
#include "foc_app.h"
#include "head.h"
#include <string.h>

extern FOC_AppHandle_t g_foc_app;
extern DRV8350S_Handle_t drv8350s;
extern TLE5012_Data_t tle5012_sensor;

#define CAN_RX_RING_DEPTH       16U
#define CAN_TX_QUEUE_DEPTH      64U
#define CAN_TUNNEL_MAX_LEN      255U
#define CAN_TUNNEL_BUFFER_SIZE  256U
#define CAN_TUNNEL_TIMEOUT_MS   300U
#define CAN_TELEMETRY_PERIOD_MS 20U

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} CanRxFrame_t;

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} CanTxFrame_t;

static uint8_t s_can_node_id = CAN_NODE_ID_DEFAULT;
static uint8_t s_can_enabled = 0U;
static uint8_t s_can_self_test_ok = 0U;
static uint8_t s_can_bus_was_off = 0U;

static CanRxFrame_t s_rxRing[CAN_RX_RING_DEPTH];
static volatile uint8_t s_rxHead = 0U;
static volatile uint8_t s_rxTail = 0U;

static CanTxFrame_t s_txQueue[CAN_TX_QUEUE_DEPTH];
static uint8_t s_txHead = 0U;
static uint8_t s_txTail = 0U;

static uint8_t s_heartbeat_armed = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_heartbeat_timeout_ms = CAN_HEARTBEAT_TIMEOUT_DEFAULT;

static uint8_t s_tunnelBuf[CAN_TUNNEL_BUFFER_SIZE];
static uint16_t s_tunnelLen = 0U;
static uint16_t s_tunnelExpected = 0U;
static uint8_t s_tunnelSeq = 0U;
static uint8_t s_tunnelActive = 0U;
static uint32_t s_tunnelLastTick = 0U;

static uint32_t s_lastTelemetryTick = 0U;
static uint8_t s_bootupPending = 1U;
static uint8_t s_faultSentCode = 0xFFU;

static uint32_t s_selfTestFrameId = 0U;
static uint8_t  s_selfTestFrameData[8] = {0};
static uint8_t  s_selfTestFrameLen = 0U;

static uint8_t Can_IsNodeTarget(uint32_t msg_id, uint8_t allow_broadcast)
{
    uint8_t node = (uint8_t)(msg_id & 0x3FU);

    if (node == s_can_node_id) {
        return 1U;
    }
    return (allow_broadcast != 0U) && (node == 0U);
}

static uint32_t Can_BytesToDlc(uint8_t len)
{
    if (len <= 8U) {
        return (uint32_t)len;
    }
    if (len <= 12U) {
        return FDCAN_DLC_BYTES_12;
    }
    if (len <= 16U) {
        return FDCAN_DLC_BYTES_16;
    }
    if (len <= 20U) {
        return FDCAN_DLC_BYTES_20;
    }
    if (len <= 24U) {
        return FDCAN_DLC_BYTES_24;
    }
    if (len <= 32U) {
        return FDCAN_DLC_BYTES_32;
    }
    if (len <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

static uint8_t CanProtocol_SendRaw(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;

    if (!s_can_enabled || (data == NULL) || (len > 8U)) {
        return 0U;
    }

    memset(&txHeader, 0, sizeof(txHeader));
    txHeader.Identifier = msg_id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = Can_BytesToDlc(len);
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) == HAL_OK) ? 1U : 0U;
}

static uint8_t Can_TxQueuePush(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    uint8_t next;

    if (len > 8U) {
        return 0U;
    }

    next = (uint8_t)((s_txHead + 1U) % CAN_TX_QUEUE_DEPTH);
    if (next == s_txTail) {
        return 0U;
    }

    s_txQueue[s_txHead].id = msg_id;
    s_txQueue[s_txHead].len = len;
    if (data != NULL) {
        memcpy(s_txQueue[s_txHead].data, data, len);
    } else {
        memset(s_txQueue[s_txHead].data, 0, sizeof(s_txQueue[s_txHead].data));
    }
    s_txHead = next;
    return 1U;
}

static void CanProtocol_ProcessTxQueue(void)
{
    while ((s_txHead != s_txTail) &&
           (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)) {
        (void)CanProtocol_SendRaw(s_txQueue[s_txTail].id,
                                  s_txQueue[s_txTail].data,
                                  s_txQueue[s_txTail].len);
        s_txTail = (uint8_t)((s_txTail + 1U) % CAN_TX_QUEUE_DEPTH);
    }
}

static void CanProtocol_SendFrame(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    if (!s_can_enabled || (len > 8U)) {
        return;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) {
        CanProtocol_SendRaw(msg_id, data, len);
    }
}

static void CanProtocol_SendTunnelPayload(const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8];
    uint16_t pos = 0U;
    uint8_t seq = 0U;
    uint32_t resp_id = CAN_GROUP_TUNNEL_RESP | s_can_node_id;

    if (payload == NULL) {
        return;
    }
    if (len > CAN_TUNNEL_MAX_LEN) {
        len = CAN_TUNNEL_MAX_LEN;
    }

    if (len <= 7U) {
        memset(frame, 0, sizeof(frame));
        frame[0] = (uint8_t)(0x00U | len);
        if (len > 0U) {
            memcpy(&frame[1], payload, len);
        }
        (void)Can_TxQueuePush(resp_id, frame, (uint8_t)(1U + len));
        return;
    }

    memset(frame, 0, sizeof(frame));
    frame[0] = 0x40U;
    frame[1] = (uint8_t)len;
    memcpy(&frame[2], payload, 6U);
    (void)Can_TxQueuePush(resp_id, frame, 8U);
    pos = 6U;

    while (pos < len) {
        uint16_t remaining = (uint16_t)(len - pos);
        uint16_t maxChunk = 7U;
        uint8_t chunk = (uint8_t)((remaining < maxChunk) ? remaining : maxChunk);

        memset(frame, 0, sizeof(frame));
        frame[0] = (uint8_t)(0x80U | seq);
        memcpy(&frame[1], &payload[pos], chunk);
        (void)Can_TxQueuePush(resp_id, frame, (uint8_t)(1U + chunk));
        pos += chunk;
        seq++;
    }
}

void CanProtocol_SendTunnelText(const char *text)
{
    uint16_t len = 0U;

    if (text == NULL) {
        return;
    }

    while ((len < CAN_TUNNEL_MAX_LEN) && (text[len] != '\0')) {
        len++;
    }
    while ((len > 0U) && ((text[len - 1U] == '\r') || (text[len - 1U] == '\n'))) {
        len--;
    }
    CanProtocol_SendTunnelPayload((const uint8_t *)text, len);
}

static void CanProtocol_TunnelReset(void)
{
    s_tunnelActive = 0U;
    s_tunnelLen = 0U;
    s_tunnelExpected = 0U;
    s_tunnelSeq = 0U;
    s_tunnelLastTick = 0U;
}

static void CanProtocol_TunnelInject(void)
{
    if ((s_tunnelLen == 0U) || (s_tunnelLen > CAN_TUNNEL_MAX_LEN)) {
        CanProtocol_TunnelReset();
        return;
    }

    s_tunnelBuf[s_tunnelLen] = '\0';
    UART_CommandQueuePushFromCan((const char *)s_tunnelBuf);
    CanProtocol_TunnelReset();
}

static void CanProtocol_TunnelFrame(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    uint8_t first;
    uint8_t frameLen;

    if ((data == NULL) || (len < 1U)) {
        return;
    }
    if (!Can_IsNodeTarget(msg_id, 0U)) {
        return;
    }

    first = data[0];
    if ((first & 0xC0U) == 0x00U) {
        frameLen = (uint8_t)(first & 0x3FU);
        if ((frameLen >= 1U) && (frameLen <= 7U) && (len >= (uint8_t)(frameLen + 1U))) {
            CanProtocol_TunnelReset();
            memcpy(s_tunnelBuf, &data[1], frameLen);
            s_tunnelLen = frameLen;
            CanProtocol_TunnelInject();
        }
        return;
    }

    if (first == 0x40U) {
        if ((len >= 2U) && (data[1] >= 8U)) {
            CanProtocol_TunnelReset();
            s_tunnelExpected = data[1];
            s_tunnelLen = (uint16_t)((len > 7U) ? (len - 2U) : 0U);
            if (s_tunnelLen > 6U) {
                s_tunnelLen = 6U;
            }
            memcpy(s_tunnelBuf, &data[2], s_tunnelLen);
            s_tunnelActive = 1U;
            s_tunnelLastTick = HAL_GetTick();
            if (s_tunnelLen >= s_tunnelExpected) {
                CanProtocol_TunnelInject();
            }
        }
        return;
    }

    if ((first & 0x80U) != 0U) {
        uint8_t seq = (uint8_t)(first & 0x7FU);
        uint8_t chunk;

        if (!s_tunnelActive || (seq != s_tunnelSeq)) {
            CanProtocol_TunnelReset();
            return;
        }

        chunk = (uint8_t)((len > 1U) ? (len - 1U) : 0U);
        if (chunk > 7U) {
            chunk = 7U;
        }
        if ((uint16_t)(s_tunnelLen + chunk) > s_tunnelExpected) {
            chunk = (uint8_t)(s_tunnelExpected - s_tunnelLen);
        }
        if (chunk > 0U) {
            memcpy(&s_tunnelBuf[s_tunnelLen], &data[1], chunk);
            s_tunnelLen = (uint16_t)(s_tunnelLen + chunk);
        }
        s_tunnelSeq++;
        s_tunnelLastTick = HAL_GetTick();
        if (s_tunnelLen >= s_tunnelExpected) {
            CanProtocol_TunnelInject();
        }
        return;
    }

    CanProtocol_TunnelReset();
}

static void CanProtocol_FastStop(void)
{
    FOC_App_StopIdentify(&g_foc_app);
    FOC_App_SetSpeedRef(&g_foc_app, 0.0f);
    FOC_App_Disable(&g_foc_app);
}

static void CanProtocol_HandleFastCtrl(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    float value;

    if ((data == NULL) || (len < 1U)) {
        return;
    }
    if (!Can_IsNodeTarget(msg_id, 1U)) {
        return;
    }

    switch (data[0]) {
    case 0x01U: /* STOP */
        CanProtocol_FastStop();
        break;

    case 0x02U: /* UNLOCK */
        if (len >= 2U) {
            if (data[1] != 0U) {
                g_foc_app.power_unlocked = 1U;
            } else {
                g_foc_app.power_unlocked = 0U;
                g_foc_app.stall_mode_armed = 0U;
                FOC_App_StopIdentify(&g_foc_app);
                FOC_App_Disable(&g_foc_app);
            }
        }
        break;

    case 0x03U: /* ENABLE */
        if (len >= 2U) {
            if (data[1] == 0U) {
                FOC_App_Disable(&g_foc_app);
            } else if (g_foc_app.power_unlocked != 0U) {
                if ((g_foc_app.app_mode != APP_MODE_SCROLL_WHEEL) ||
                    WheelInput_IsSessionActive()) {
                    FOC_App_Enable(&g_foc_app);
                }
            }
        }
        break;

    case 0x04U: /* SET_MODE */
        if ((len >= 2U) && (data[1] <= (uint8_t)FOC_MODE_POSITION)) {
            FOC_App_SetRawControlMode(&g_foc_app, (FOC_ControlMode_t)data[1]);
        }
        break;

    case 0x10U: /* SET_POS */
        if (len >= 5U) {
            memcpy(&value, &data[1], 4U);
            FOC_App_SetPositionRef(&g_foc_app, value);
        }
        break;

    case 0x11U: /* SET_SPEED */
        if (len >= 5U) {
            memcpy(&value, &data[1], 4U);
            FOC_App_SetSpeedRef(&g_foc_app, value);
        }
        break;

    case 0x12U: /* SET_TORQUE */
        if (len >= 5U) {
            memcpy(&value, &data[1], 4U);
            FOC_App_SetCurrentRef(&g_foc_app, 0.0f, value);
        }
        break;

    case 0x20U: /* CLEAR_FAULT */
        UART_CommandExecuteMuted("CMD:CLEAR_FAULT");
        break;

    default:
        break;
    }
}

static void CanProtocol_HandleNmt(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    uint16_t timeoutMs;

    if ((data == NULL) || (len < 1U)) {
        return;
    }
    if (!Can_IsNodeTarget(msg_id, 1U)) {
        return;
    }

    switch (data[0]) {
    case 0x01U: /* HEARTBEAT */
        if (len >= 4U) {
            timeoutMs = (uint16_t)(data[2] | ((uint16_t)data[3] << 8U));
            s_last_heartbeat_ms = HAL_GetTick();
            if (timeoutMs == 0xFFFFU) {
                s_heartbeat_armed = 0U;
            } else {
                s_heartbeat_armed = 1U;
                s_heartbeat_timeout_ms = (timeoutMs == 0U) ?
                                         CAN_HEARTBEAT_TIMEOUT_DEFAULT :
                                         (uint32_t)timeoutMs;
            }
        }
        break;

    case 0x02U: /* ESTOP */
        CanProtocol_FastStop();
        break;

    case 0x03U: /* SET_NODE_ID, v1.1: ignore */
    default:
        break;
    }
}

static void CanProtocol_SendStateFast(void)
{
    uint8_t data[8];
    int16_t iq_mA;
    uint16_t vbus_x10;

    memset(data, 0, sizeof(data));
    data[0] = 0x01U;
    data[1] = (uint8_t)g_foc_app.state;
    data[2] = (uint8_t)g_foc_app.fault_code;
    data[3] = (uint8_t)g_foc_app.control_mode;
    data[4] = (uint8_t)g_foc_app.app_mode;

    {
        float iq_a = g_foc_app.foc.Idq.q * 1000.0f;

        if (iq_a > 32767.0f) {
            iq_a = 32767.0f;
        } else if (iq_a < -32768.0f) {
            iq_a = -32768.0f;
        }
        iq_mA = (int16_t)iq_a;
    }
    data[5] = (uint8_t)(iq_mA & 0xFFU);
    data[6] = (uint8_t)(((uint16_t)iq_mA >> 8U) & 0xFFU);

    vbus_x10 = (uint16_t)(g_foc_app.Vbus * 10.0f);
    if (vbus_x10 > 255U) {
        vbus_x10 = 255U;
    }
    data[7] = (uint8_t)vbus_x10;

    CanProtocol_SendFrame(CAN_GROUP_TELEMETRY | s_can_node_id, data, 8U);
}

static void CanProtocol_SendBootup(void)
{
    uint8_t data[8];
    const char *hash = FOC_GIT_HASH;
    uint8_t i;

    memset(data, 0, sizeof(data));
    data[0] = 0x10U;
    data[1] = CAN_PROTOCOL_VERSION;
    data[2] = s_can_node_id;
    for (i = 0U; i < 4U; i++) {
        data[3U + i] = (hash[i] != '\0') ? (uint8_t)hash[i] : 0U;
    }
    data[7] = (uint8_t)g_foc_app.state;
    CanProtocol_SendFrame(CAN_GROUP_TELEMETRY | s_can_node_id, data, 8U);
    s_bootupPending = 0U;
}

static void CanProtocol_SendFaultEvent(void)
{
    uint8_t data[8];
    uint16_t fs1 = drv8350s.runtime.regFaultStatus1;
    uint16_t vgs2 = drv8350s.runtime.regVgsStatus2;

    memset(data, 0, sizeof(data));
    data[0] = 0x01U;
    data[1] = (uint8_t)g_foc_app.fault_code;
    data[2] = (uint8_t)g_foc_app.state;
    data[3] = (uint8_t)(fs1 & 0xFFU);
    data[4] = (uint8_t)((fs1 >> 8U) & 0xFFU);
    data[5] = (uint8_t)(vgs2 & 0xFFU);
    data[6] = (uint8_t)((vgs2 >> 8U) & 0xFFU);
    data[7] = tle5012_sensor.status;

    CanProtocol_SendFrame(CAN_GROUP_FAULT | s_can_node_id, data, 8U);
}

static uint8_t CanProtocol_StartHw(uint32_t mode)
{
    FDCAN_FilterTypeDef filter;

    if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK) {
        return 0U;
    }

    hfdcan1.Init.Mode = mode;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        return 0U;
    }

    memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        return 0U;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return 0U;
    }

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    (void)HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                         FDCAN_INTERRUPT_LINE0);
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK) {
        return 0U;
    }

    return 1U;
}

void CanProtocol_Init(uint8_t node_id)
{
    uint8_t loopbackData[8];
    uint32_t deadline;

    s_can_node_id = ((node_id > 0U) && (node_id <= CAN_NODE_ID_MAX)) ?
                    node_id : CAN_NODE_ID_DEFAULT;
    s_heartbeat_armed = 0U;
    s_heartbeat_timeout_ms = CAN_HEARTBEAT_TIMEOUT_DEFAULT;
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_txHead = 0U;
    s_txTail = 0U;
    s_bootupPending = 1U;
    s_faultSentCode = 0xFFU;
    CanProtocol_TunnelReset();

    s_can_enabled = 0U;
    if (!CanProtocol_StartHw(FDCAN_MODE_INTERNAL_LOOPBACK)) {
        s_can_self_test_ok = 0U;
        return;
    }

    s_can_enabled = 1U;
    memset(loopbackData, 0, sizeof(loopbackData));
    loopbackData[0] = 0x01U;
    s_selfTestFrameId = CAN_GROUP_FAST_CTRL | s_can_node_id;
    s_selfTestFrameLen = 1U;
    s_selfTestFrameData[0] = loopbackData[0];
    CanProtocol_SendRaw(s_selfTestFrameId, s_selfTestFrameData, s_selfTestFrameLen);

    deadline = HAL_GetTick() + 100U;
    while ((s_rxHead == s_rxTail) && (HAL_GetTick() < deadline)) {
        /* Wait for the internal loopback frame. */
    }
    s_can_self_test_ok = (s_rxHead != s_rxTail) ? 1U : 0U;
    s_rxHead = 0U;
    s_rxTail = 0U;

    s_can_enabled = 0U;
    if (!CanProtocol_StartHw(FDCAN_MODE_NORMAL)) {
        s_can_enabled = 0U;
        return;
    }
    s_can_enabled = 1U;
    s_lastTelemetryTick = HAL_GetTick();
}

uint8_t CanProtocol_SelfTestOk(void)
{
    /* 上电内部回环自检结果: 1=回环帧收到(收发通路OK), 0=失败 */
    return s_can_self_test_ok;
}

void CanProtocol_SetNodeId(uint8_t node_id)
{
    if ((node_id > 0U) && (node_id <= CAN_NODE_ID_MAX)) {
        s_can_node_id = node_id;
    }
}

uint8_t CanProtocol_GetNodeId(void)
{
    return s_can_node_id;
}

uint8_t CanProtocol_IsHeartbeatOk(void)
{
    return s_heartbeat_armed;
}

void CanProtocol_OnRx(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    uint8_t next;

    if (!s_can_enabled || (data == NULL) || (len > 8U)) {
        return;
    }

    next = (uint8_t)((s_rxHead + 1U) % CAN_RX_RING_DEPTH);
    if (next == s_rxTail) {
        s_rxTail = (uint8_t)((s_rxTail + 1U) % CAN_RX_RING_DEPTH);
    }
    s_rxRing[s_rxHead].id = msg_id;
    s_rxRing[s_rxHead].len = len;
    memcpy(s_rxRing[s_rxHead].data, data, len);
    s_rxHead = next;
}

void CanProtocol_Process(void)
{
    FDCAN_ProtocolStatusTypeDef status;
    uint32_t now;

    if (!s_can_enabled) {
        return;
    }

    /* Simple bus-off self-recovery per protocol section 2. */
    memset(&status, 0, sizeof(status));
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) == HAL_OK) {
        if (status.BusOff != 0U) {
            if (!s_can_bus_was_off) {
                s_can_bus_was_off = 1U;
            }
            (void)HAL_FDCAN_Stop(&hfdcan1);
            if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
                (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                                     0U);
                s_can_bus_was_off = 0U;
            }
        }
    }

    CanProtocol_ProcessTxQueue();

    /* Drain RX ring in main-loop context (interrupt only enqueues). */
    while (s_rxTail != s_rxHead) {
        uint32_t group = s_rxRing[s_rxTail].id & 0x700U;

        if (group == CAN_GROUP_NMT) {
            CanProtocol_HandleNmt(s_rxRing[s_rxTail].id,
                                  s_rxRing[s_rxTail].data,
                                  s_rxRing[s_rxTail].len);
        } else if (group == CAN_GROUP_FAST_CTRL) {
            CanProtocol_HandleFastCtrl(s_rxRing[s_rxTail].id,
                                       s_rxRing[s_rxTail].data,
                                       s_rxRing[s_rxTail].len);
        } else if (group == CAN_GROUP_TUNNEL_REQ) {
            CanProtocol_TunnelFrame(s_rxRing[s_rxTail].id,
                                    s_rxRing[s_rxTail].data,
                                    s_rxRing[s_rxTail].len);
        }
        s_rxTail = (uint8_t)((s_rxTail + 1U) % CAN_RX_RING_DEPTH);
    }

    /* Tunnel reassembly timeout. */
    if (s_tunnelActive &&
        ((HAL_GetTick() - s_tunnelLastTick) > CAN_TUNNEL_TIMEOUT_MS)) {
        CanProtocol_TunnelReset();
    }

    /* Heartbeat guard: not armed means UART remains fully independent. */
    if (s_heartbeat_armed &&
        ((HAL_GetTick() - s_last_heartbeat_ms) > s_heartbeat_timeout_ms)) {
        s_heartbeat_armed = 0U;
        CanProtocol_FastStop();
    }

    now = HAL_GetTick();
    if ((now - s_lastTelemetryTick) >= CAN_TELEMETRY_PERIOD_MS) {
        s_lastTelemetryTick = now;
        CanProtocol_SendStateFast();
    }

    if (s_bootupPending != 0U) {
        CanProtocol_SendBootup();
    }

    if (g_foc_app.state == FOC_STATE_FAULT) {
        if (g_foc_app.fault_code != s_faultSentCode) {
            s_faultSentCode = g_foc_app.fault_code;
            CanProtocol_SendFaultEvent();
        }
    } else {
        s_faultSentCode = 0xFFU;
    }
}
