/**
 * @file    current_stream.h
 * @brief   2kHz binary phase current stream — ring buffer + binary frame packing
 * @note    V1.1: 1000000 baud baseline, 25-byte frames, CRC-8
 *
 * ISR (TIM1 20kHz) writes samples into a lock-free ring buffer.
 * Main loop drains the ring and sends frames via UART TX ring (P1 priority).
 * At 2kHz: 25B × 2000 = 50 kB/s, ~50% of 1000000 baud budget.
 */

#ifndef __CURRENT_STREAM_H
#define __CURRENT_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Ring buffer sizing ────────────────────────────────────────── */
#define CUR_STREAM_RING_SIZE  64U   /* 2kHz → 32ms buffer, ISR-safe */

/* ── Binary frame constants ─────────────────────────────────────── */
#define CUR_STREAM_FRAME_LEN  25U   /* sync(2) + type(1) + len(1) + payload(20) + crc(1) */
#define CUR_STREAM_PAYLOAD_LEN 20U
#define CUR_STREAM_SYNC0      0xA5U
#define CUR_STREAM_SYNC1      0x5AU
#define CUR_STREAM_TYPE       'C'

/* ── Mode ───────────────────────────────────────────────────────── */
typedef enum {
    CUR_STREAM_OFF   = 0,
    CUR_STREAM_ASCII = 1,
    CUR_STREAM_BIN   = 2
} CurStreamMode_t;

/* ── Sample (21 bytes raw, packed to 20-byte payload) ──────────── */
typedef struct {
    uint16_t seq;       /* monotonic sample counter */
    uint32_t tick_ms;   /* HAL_GetTick() at capture */
    int16_t  ia_mA;     /* phase A current, mA */
    int16_t  ib_mA;     /* phase B current, mA */
    int16_t  ic_mA;     /* phase C current, mA */
    int16_t  id_mA;     /* D-axis current, mA */
    int16_t  iq_mA;     /* Q-axis current, mA */
    uint16_t vbus_mV;   /* bus voltage, mV */
    uint16_t flags;     /* reserved, 0 for now */
} CurStreamSample_t;

/* ── API ────────────────────────────────────────────────────────── */

void CurStream_Init(void);
void CurStream_SetMode(CurStreamMode_t mode, uint16_t rate_hz);
CurStreamMode_t CurStream_GetMode(void);
uint16_t CurStream_GetRate(void);

/* Called from TIM1 ISR — writes sample to ring, never blocks */
void CurStream_PushSample(const CurStreamSample_t *s);

/* Called from main loop — drains ring, sends frames (P1 priority) */
void CurStream_Process(void);

/* ── Shared binary frame builder ─────────────────────────────────── */
/**
 * @brief Build a binary envelope frame: A5 5A | type | payload_len | payload | CRC-8
 * @param type         Frame type byte ('C' = current, 'W' = wheel event, etc.)
 * @param payload      Payload bytes to pack
 * @param payload_len  Number of payload bytes
 * @param buf_out      Output buffer, must be at least (payload_len + 5) bytes
 * @return Total frame length written to buf_out (payload_len + 5)
 */
uint8_t CurStream_BuildFrame(uint8_t type, const uint8_t *payload,
                             uint8_t payload_len, uint8_t *buf_out);

/* CRC-8 (poly 0x07) — exposed for shared use by wheel_input, etc. */
uint8_t CurStream_CRC8(const uint8_t *data, uint16_t len);

/* Statistics */
uint32_t CurStream_GetSentCount(void);
uint32_t CurStream_GetDropCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_STREAM_H */
