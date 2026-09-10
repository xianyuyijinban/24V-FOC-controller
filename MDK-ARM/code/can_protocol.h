/**
 * @file    can_protocol.h
 * @brief   CAN v1.0 protocol: NMT, FAST_CTRL, telemetry, tunnel, fault
 *
 * Protocol reference: docs/CAN_PROTOCOL.md v1.0.
 */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_NODE_ID_DEFAULT      1U
#define CAN_NODE_ID_MAX         63U
#define CAN_PROTOCOL_VERSION   0x01U
#define CAN_HEARTBEAT_TIMEOUT_DEFAULT 600U

/* ID group bases (11-bit standard frame, low 6 bits = node_id). */
#define CAN_GROUP_NMT          0x000U
#define CAN_GROUP_FAST_CTRL    0x100U
#define CAN_GROUP_TELEMETRY    0x200U
#define CAN_GROUP_TUNNEL_REQ   0x300U
#define CAN_GROUP_TUNNEL_RESP  0x400U
#define CAN_GROUP_FAULT        0x500U

/* ── API ──────────────────────────────────────────────────── */

void CanProtocol_Init(uint8_t node_id);
void CanProtocol_SetNodeId(uint8_t node_id);
uint8_t CanProtocol_GetNodeId(void);

/* Power-up internal loopback self-test result: 1 = TX/RX path OK. */
uint8_t CanProtocol_SelfTestOk(void);

/* Called from main loop at ~10Hz */
void CanProtocol_Process(void);

/* Called when CAN message received (from FDCAN RX IRQ) */
void CanProtocol_OnRx(uint32_t msg_id, const uint8_t *data, uint8_t len);

/* Heartbeat guard state: 0 = not armed, 1 = armed/ok. */
uint8_t CanProtocol_IsHeartbeatOk(void);

/* Send a tunneled UART text response (truncated to 255 bytes). */
void CanProtocol_SendTunnelText(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
