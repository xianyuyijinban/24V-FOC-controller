/**
 * @file    current_stream.c
 * @brief   2kHz binary phase current stream — ISR-push with decimation, main-loop drain
 * @note    V1.1: 1000000 baud, P1 priority, CRC-8 poly 0x07
 *
 * Architecture:
 *  - TIM1 ISR (~20kHz center-aligned, overflow+underflow) → CurStream_PushSample()
 *    → accumulator decimation → ring buffer
 *  - main loop (~200Hz) → CurStream_Process() → burst drain → DrvUart_SendBytesP1()
 *
 * Rate control is in the ISR via accumulator decimation (20000 Hz base).
 * Main loop drains as fast as possible (burst cap prevents TX ring overload).
 * At 2000Hz target: 20000 / 10 = 2000Hz push → 200Hz main loop × 10 burst ≈ 2000 fps out.
 */

#include "current_stream.h"
#include "uart_upload.h"
#include <string.h>
#include <stdio.h>

/* ── ISR decimation base ────────────────────────────────────────── */
#define CUR_STREAM_ISR_HZ  20000U  /* TIM1 center-aligned: update IRQ fires on both overflow & underflow */

/* ── CRC-8 lookup table (poly 0x07, init 0x00, no reflect) ──────── */
static const uint8_t s_crc8_table[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

static uint8_t CurStream_CRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    uint16_t i;
    for (i = 0U; i < len; i++) {
        crc = s_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ── Ring buffer ─────────────────────────────────────────────────── */
static CurStreamSample_t s_ring[CUR_STREAM_RING_SIZE];
static volatile uint8_t  s_write_idx = 0U;  /* ISR writes */
static uint8_t           s_read_idx  = 0U;  /* main loop reads */

/* ── State ──────────────────────────────────────────────────────── */
static CurStreamMode_t     s_mode            = CUR_STREAM_OFF;
static uint16_t            s_rate_hz         = 0U;
static uint32_t            s_sent            = 0U;
static uint32_t            s_drop            = 0U;
static volatile uint16_t   s_push_seq        = 0U;  /* ISR-owned monotonic seq */
static volatile uint32_t   s_decim_acc       = 0U;  /* ISR decimation accumulator */
static uint32_t            s_saved_telem_ms  = 0U;  /* saved N-frame interval before BIN 2000 */

/* ── Local frame buffer ──────────────────────────────────────────── */
#define FRAME_BUF_SIZE  CUR_STREAM_FRAME_LEN  /* 25 */

/* ──────────────────────────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────────────────────────── */

void CurStream_Init(void)
{
    s_mode            = CUR_STREAM_OFF;
    s_rate_hz         = 0U;
    s_write_idx       = 0U;
    s_read_idx        = 0U;
    s_sent            = 0U;
    s_drop            = 0U;
    s_push_seq        = 0U;
    s_decim_acc       = 0U;
    s_saved_telem_ms  = 0U;
}

void CurStream_SetMode(CurStreamMode_t mode, uint16_t rate_hz)
{
    if (mode == CUR_STREAM_OFF) {
        s_mode    = CUR_STREAM_OFF;
        s_rate_hz = 0U;
        DrvUart_SetLegacyPhaseCurrentEnable(true);
        /* Restore N-frame telemetry rate saved on BIN 2000 entry */
        if (s_saved_telem_ms != 0U) {
            DrvUart_SetInterval(s_saved_telem_ms);
            s_saved_telem_ms = 0U;
        }
    } else if (mode == CUR_STREAM_BIN || mode == CUR_STREAM_ASCII) {
        if (rate_hz == 0U || rate_hz > 5000U) {
            return;
        }
        s_mode      = mode;
        s_rate_hz   = rate_hz;
        s_sent      = 0U;
        s_drop      = 0U;
        s_push_seq  = 0U;
        s_decim_acc = 0U;
        if (mode == CUR_STREAM_BIN) {
            DrvUart_SetLegacyPhaseCurrentEnable(false);
            if (rate_hz >= 2000U) {
                /* BIN 2000: auto-reduce N-frame telemetry to 10Hz */
                if (s_saved_telem_ms == 0U) {
                    s_saved_telem_ms = DrvUart_GetInterval();
                }
                DrvUart_SetInterval(100U);
            } else {
                /* BIN 1000: restore saved N-frame rate if coming down from 2000 */
                if (s_saved_telem_ms != 0U) {
                    DrvUart_SetInterval(s_saved_telem_ms);
                    s_saved_telem_ms = 0U;
                }
            }
        }
    }
}

CurStreamMode_t CurStream_GetMode(void)
{
    return s_mode;
}

uint16_t CurStream_GetRate(void)
{
    return s_rate_hz;
}

/**
 * @brief Push a sample into the ring buffer (TIM1 ISR, 20kHz).
 * @note  Uses accumulator decimation to achieve target rate_hz.
 *        E.g. rate=2000Hz → pushes every 10th ISR call.
 *        Lock-free: only the ISR writes s_write_idx.
 */
void CurStream_PushSample(const CurStreamSample_t *s)
{
    if (s_mode == CUR_STREAM_OFF || s == NULL || s_rate_hz == 0U) {
        return;
    }

    /* Accumulator decimation: push when acc >= ISR_HZ */
    s_decim_acc += s_rate_hz;
    if (s_decim_acc < CUR_STREAM_ISR_HZ) {
        return;
    }
    s_decim_acc -= CUR_STREAM_ISR_HZ;

    /* Compute next write position */
    uint8_t next = s_write_idx + 1U;
    if (next >= CUR_STREAM_RING_SIZE) {
        next = 0U;
    }

    /* Ring full? Drop oldest (increment read_idx to skip one) */
    if (next == s_read_idx) {
        s_drop++;
        s_read_idx++;
        if (s_read_idx >= CUR_STREAM_RING_SIZE) {
            s_read_idx = 0U;
        }
    }

    s_ring[s_write_idx] = *s;
    s_ring[s_write_idx].seq = s_push_seq++;
    s_write_idx = next;
}

/**
 * @brief Drain the ring buffer (main loop, ~200Hz).
 * @note  Burst-drain up to 32 frames per call. Backpressure is enforced
 *        by DrvUart_SendBytesP1 returning false when the TX ring P1
 *        admission rejects (free < 128+len). Command TX (P0) always wins.
 */
void CurStream_Process(void)
{
    uint8_t buf[FRAME_BUF_SIZE];
    uint8_t burst;
    uint8_t crc;

    if (s_mode == CUR_STREAM_OFF || s_rate_hz == 0U) {
        return;
    }

    burst = 0U;
    while (s_read_idx != s_write_idx && burst < 32U) {
        if (s_mode == CUR_STREAM_BIN) {
            CurStreamSample_t *samp = &s_ring[s_read_idx];

            /* Pack 25-byte binary frame */
            buf[0]  = CUR_STREAM_SYNC0;
            buf[1]  = CUR_STREAM_SYNC1;
            buf[2]  = CUR_STREAM_TYPE;
            buf[3]  = CUR_STREAM_PAYLOAD_LEN;
            buf[4]  = (uint8_t)(samp->seq & 0xFFU);
            buf[5]  = (uint8_t)((samp->seq >> 8) & 0xFFU);
            buf[6]  = (uint8_t)(samp->tick_ms & 0xFFU);
            buf[7]  = (uint8_t)((samp->tick_ms >> 8) & 0xFFU);
            buf[8]  = (uint8_t)((samp->tick_ms >> 16) & 0xFFU);
            buf[9]  = (uint8_t)((samp->tick_ms >> 24) & 0xFFU);
            buf[10] = (uint8_t)(samp->ia_mA & 0xFFU);
            buf[11] = (uint8_t)((samp->ia_mA >> 8) & 0xFFU);
            buf[12] = (uint8_t)(samp->ib_mA & 0xFFU);
            buf[13] = (uint8_t)((samp->ib_mA >> 8) & 0xFFU);
            buf[14] = (uint8_t)(samp->ic_mA & 0xFFU);
            buf[15] = (uint8_t)((samp->ic_mA >> 8) & 0xFFU);
            buf[16] = (uint8_t)(samp->id_mA & 0xFFU);
            buf[17] = (uint8_t)((samp->id_mA >> 8) & 0xFFU);
            buf[18] = (uint8_t)(samp->iq_mA & 0xFFU);
            buf[19] = (uint8_t)((samp->iq_mA >> 8) & 0xFFU);
            buf[20] = (uint8_t)(samp->vbus_mV & 0xFFU);
            buf[21] = (uint8_t)((samp->vbus_mV >> 8) & 0xFFU);
            buf[22] = (uint8_t)(samp->flags & 0xFFU);
            buf[23] = (uint8_t)((samp->flags >> 8) & 0xFFU);
            crc     = CurStream_CRC8(buf, 24U);
            buf[24] = crc;

            if (!DrvUart_SendBytesP1(buf, FRAME_BUF_SIZE)) {
                break;  /* TX ring full — stop draining, try next cycle */
            }
            s_sent++;
            burst++;
        } else {
            /* ASCII mode */
            char text[80];
            CurStreamSample_t *samp = &s_ring[s_read_idx];
            int n = snprintf(text, sizeof(text),
                     "C,%lu,%d,%d,%d,%d,%d\r\n",
                     (unsigned long)samp->tick_ms,
                     (int)samp->ia_mA, (int)samp->ib_mA, (int)samp->ic_mA,
                     (int)samp->id_mA, (int)samp->iq_mA);
            if (n > 0 && n < (int)sizeof(text)) {
                text[n] = '\0';
                DrvUart_SendTextP1(text);
                s_sent++;
                burst++;
            }
        }

        s_read_idx++;
        if (s_read_idx >= CUR_STREAM_RING_SIZE) {
            s_read_idx = 0U;
        }
    }
}

uint32_t CurStream_GetSentCount(void)
{
    return s_sent;
}

uint32_t CurStream_GetDropCount(void)
{
    return s_drop;
}
