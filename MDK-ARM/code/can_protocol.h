/**
 * @file    can_protocol.h
 * @brief   Phase 6: CAN multi-node protocol (minimal, for future FDCAN enable)
 *
 * Hardware path: STM32H743 FDCAN1 (currently disabled at init — FOC_DEBUG_DISABLE_FDCAN_INIT=1)
 * When crystal path is repaired, enable MX_FDCAN1_Init() to activate.
 */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_NODE_ID_DEFAULT      1U
#define CAN_NODE_ID_MAX         64U
#define CAN_HEARTBEAT_MS       200U    /* heartbeat interval */
#define CAN_HEARTBEAT_TIMEOUT  600U    /* auto-STOP if no heartbeat from master */

/* CAN message IDs (11-bit) */
#define CAN_MSG_HEARTBEAT      0x700U  /* + node_id */
#define CAN_MSG_SET_POS        0x200U  /* + node_id, data: float pos_deg */
#define CAN_MSG_SET_SPEED      0x300U  /* + node_id, data: float speed_radps */
#define CAN_MSG_GET_STATE      0x400U  /* + node_id, request */
#define CAN_MSG_STATE_RESP     0x500U  /* + node_id, response */
#define CAN_MSG_CLEAR_FAULT    0x600U  /* + node_id */
#define CAN_MSG_STOP           0x100U  /* + node_id, emergency stop */

/* State response data (8 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  state;           /* FOC_AppState_t */
    uint8_t  fault_code;      /* FOC_FaultCode_t */
    uint8_t  control_mode;    /* FOC_ControlMode_t */
    uint8_t  app_mode;        /* AppMode_t */
    float    Vbus;            /* bus voltage */
} CanStateResp_t;

/* ── API ──────────────────────────────────────────────────── */

void CanProtocol_Init(uint8_t node_id);
void CanProtocol_SetNodeId(uint8_t node_id);
uint8_t CanProtocol_GetNodeId(void);

/* Called from main loop at ~10Hz */
void CanProtocol_Process(void);

/* Called when CAN message received (from FDCAN RX IRQ) */
void CanProtocol_OnRx(uint32_t msg_id, const uint8_t *data, uint8_t len);

/* Heartbeat lost → auto STOP guard */
uint8_t CanProtocol_IsHeartbeatOk(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
