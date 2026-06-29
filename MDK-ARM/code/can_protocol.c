/**
 * @file    can_protocol.c
 * @brief   Phase 6: CAN multi-node protocol implementation
 *
 * Minimal CAN protocol layer. Builds and links, but FDCAN hardware
 * init is currently disabled (FOC_DEBUG_DISABLE_FDCAN_INIT=1 in fdcan.h).
 * To activate: fix crystal path, enable MX_FDCAN1_Init(), call from main.
 */

#include "can_protocol.h"
#include "foc_app.h"
#include <string.h>

extern FOC_AppHandle_t g_foc_app;

static uint8_t  s_can_node_id = CAN_NODE_ID_DEFAULT;
static uint32_t s_last_heartbeat_ms = 0U;
static uint8_t  s_heartbeat_ok = 0U;
static uint8_t  s_can_enabled = 0U;

/* Forward decl — will call FDCAN send when hardware is enabled */
static void CanProtocol_Send(uint32_t msg_id, const uint8_t *data, uint8_t len);

void CanProtocol_Init(uint8_t node_id)
{
    s_can_node_id = (node_id > 0U && node_id <= CAN_NODE_ID_MAX)
                    ? node_id : CAN_NODE_ID_DEFAULT;
    s_last_heartbeat_ms = 0U;
    s_heartbeat_ok = 0U;
    s_can_enabled = 1U;
}

void CanProtocol_SetNodeId(uint8_t node_id)
{
    if (node_id > 0U && node_id <= CAN_NODE_ID_MAX) {
        s_can_node_id = node_id;
    }
}

uint8_t CanProtocol_GetNodeId(void)
{
    return s_can_node_id;
}

uint8_t CanProtocol_IsHeartbeatOk(void)
{
    return s_heartbeat_ok;
}

/* ── Message dispatch ──────────────────────────────────────── */

void CanProtocol_OnRx(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    uint8_t target_node;
    float val;

    if (!s_can_enabled || data == NULL) return;

    target_node = (uint8_t)(msg_id & 0x3FU);
    if (target_node != s_can_node_id && target_node != 0U) return;  /* not for us */

    switch (msg_id & 0x7F0U) {
    case CAN_MSG_HEARTBEAT:
        s_last_heartbeat_ms = HAL_GetTick();
        s_heartbeat_ok = 1U;
        break;

    case CAN_MSG_SET_POS:
        if (len >= 4U) {
            memcpy(&val, data, 4U);
            FOC_App_SetPositionRef(&g_foc_app, val);
        }
        break;

    case CAN_MSG_SET_SPEED:
        if (len >= 4U) {
            memcpy(&val, data, 4U);
            FOC_App_SetSpeedRef(&g_foc_app, val);
        }
        break;

    case CAN_MSG_GET_STATE: {
        CanStateResp_t resp;
        resp.state        = (uint8_t)g_foc_app.state;
        resp.fault_code   = (uint8_t)g_foc_app.fault_code;
        resp.control_mode = (uint8_t)g_foc_app.control_mode;
        resp.app_mode     = (uint8_t)g_foc_app.app_mode;
        resp.Vbus         = g_foc_app.Vbus;
        CanProtocol_Send(CAN_MSG_STATE_RESP | s_can_node_id,
                         (const uint8_t *)&resp, sizeof(resp));
        break;
    }

    case CAN_MSG_CLEAR_FAULT:
        /* CLEAR_FAULT handled via UART command queue — CAN triggers equivalent */
        g_foc_app.fault_code = FOC_FAULT_NONE;
        if (g_foc_app.state == FOC_STATE_FAULT) {
            g_foc_app.state = g_foc_app.motor_identified
                            ? FOC_STATE_READY : FOC_STATE_IDLE;
        }
        break;

    case CAN_MSG_STOP:
        __disable_irq();
        FOC_App_SetSpeedRef(&g_foc_app, 0.0f);
        FOC_App_Disable(&g_foc_app);
        __enable_irq();
        break;
    }
}

/* ── Periodic processing ───────────────────────────────────── */

void CanProtocol_Process(void)
{
    if (!s_can_enabled) return;

    /* Heartbeat timeout check */
    if (s_heartbeat_ok &&
        (HAL_GetTick() - s_last_heartbeat_ms) > CAN_HEARTBEAT_TIMEOUT) {
        s_heartbeat_ok = 0U;
        /* Auto-STOP: master heartbeat lost */
        __disable_irq();
        FOC_App_SetSpeedRef(&g_foc_app, 0.0f);
        FOC_App_Disable(&g_foc_app);
        __enable_irq();
    }

    /* Send heartbeat (master-side responsibility — slave echoes if needed) */
}

/* ── Placeholder send (activate when FDCAN enabled) ────────── */

static void CanProtocol_Send(uint32_t msg_id, const uint8_t *data, uint8_t len)
{
    (void)msg_id;
    (void)data;
    (void)len;
    /* TODO: call FDCAN send when MX_FDCAN1_Init() is enabled.
     *   FDCAN_TxHeaderTypeDef txHeader;
     *   txHeader.Identifier = msg_id;
     *   txHeader.IdType = FDCAN_STANDARD_ID;
     *   ...
     *   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
     */
}
