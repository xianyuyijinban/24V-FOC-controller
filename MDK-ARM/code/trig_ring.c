/**
 * @file    trig_ring.c
 * @brief   TRIG 故障触发 ring buffer 实现 — 见 trig_ring.h
 */

#include "trig_ring.h"
#include <string.h>

static TrigFrame_t s_ring[TRIG_RING_SIZE];      /* 28KB, 默认 ZI 域 */
static volatile TrigState_t s_state = TRIG_STATE_IDLE;
static volatile TrigSource_t s_source = TRIG_SRC_NONE;
static volatile uint32_t s_write_idx = 0U;      /* 滚动写指针 (IDLE 态推进) */
static volatile uint32_t s_trig_idx = 0U;       /* 触发帧的 ring 索引 (触发即锁存) */
static volatile uint32_t s_trig_tick = 0U;      /* 触发帧 tick (锁存) */
static volatile uint32_t s_post_count = 0U;     /* post 段已采帧数 */
static volatile uint32_t s_tick_20k = 0U;       /* 独立 20kHz 时基 (全态递增) */

void TrigRing_Init(void)
{
    s_state = TRIG_STATE_IDLE;
    s_source = TRIG_SRC_NONE;
    s_write_idx = 0U;
    s_trig_idx = 0U;
    s_trig_tick = 0U;
    s_post_count = 0U;
    s_tick_20k = 0U;
}

void TrigRing_TickTick(void)
{
    s_tick_20k++;
}

uint32_t TrigRing_GetTick(void)
{
    return s_tick_20k;
}

uint16_t TrigRing_CRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t b;
    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0U; b < 8U; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

void TrigRing_Sample(float id, float iq, float vd, float vq,
                     float theta_elec, float iq_ref, uint32_t tick_20k)
{
    TrigFrame_t f;
    f.id = id;
    f.iq = iq;
    f.vd = vd;
    f.vq = vq;
    f.theta_elec = theta_elec;
    f.iq_ref = iq_ref;
    f.tick_20k = tick_20k;

    if (s_state == TRIG_STATE_FROZEN) {
        return;                     /* 冻结防覆写: 验尸窗保持 */
    }

    /* 单写者无锁: 只有 ISR 写 s_ring/s_write_idx/post 计数;
     * 主循环读侧只在 FROZEN 态 (此时 ISR 不再写) — 无竞争。 */
    s_ring[s_write_idx] = f;

    if (s_state == TRIG_STATE_IDLE) {
        s_write_idx = (s_write_idx + 1U) & (TRIG_RING_SIZE - 1U);
    } else {
        /* POST 态: 继续推进写指针 (触发帧后一帧起), post 计数对应
         * (s_write_idx-1) 帧; 计满 256 转 FROZEN (写指针停在冻结点) */
        s_write_idx = (s_write_idx + 1U) & (TRIG_RING_SIZE - 1U);
        s_post_count++;
        if (s_post_count >= TRIG_POST_FRAMES) {
            s_state = TRIG_STATE_FROZEN;
        }
    }
}

uint8_t TrigRing_Trigger(TrigSource_t src)
{
    if (s_state != TRIG_STATE_IDLE) {
        return 0U;
    }
    /* 触发帧 = 刚写入的那帧 (s_write_idx 已推进到下一格 → 触发帧在
     * (s_write_idx-1)&1023)。锁存后 post 从下一拍起算。 */
    s_trig_idx = (s_write_idx - 1U) & (TRIG_RING_SIZE - 1U);
    s_trig_tick = s_ring[s_trig_idx].tick_20k;
    s_post_count = 0U;
    s_source = src;
    s_state = TRIG_STATE_POST;
    return 1U;
}

TrigState_t TrigRing_GetState(void)       { return s_state; }
TrigSource_t TrigRing_GetSource(void)     { return s_source; }
uint32_t TrigRing_GetTrigTick(void)       { return s_trig_tick; }
uint32_t TrigRing_GetPostCount(void)      { return s_post_count; }

uint16_t TrigRing_Pull(uint16_t off, uint16_t len, uint8_t *out_buf)
{
    uint16_t i;
    uint16_t byte_off;
    uint16_t crc;
    uint32_t ring_idx;

    if (out_buf == NULL || len == 0U ||
        (uint32_t)off + (uint32_t)len > TRIG_WINDOW_FRAMES) {
        return 0U;
    }
    if (s_state != TRIG_STATE_FROZEN) {
        return 0U;
    }

    /* 帧序 → ring 索引: 帧 0 = 触发前 768 帧 (pre 段头) */
    ring_idx = (s_trig_idx + (uint32_t)off + TRIG_RING_SIZE - TRIG_PRE_FRAMES) &
               (TRIG_RING_SIZE - 1U);
    byte_off = 0U;
    for (i = 0U; i < len; i++) {
        (void)memcpy(&out_buf[byte_off], &s_ring[ring_idx], TRIG_FRAME_SIZE);
        byte_off = (uint16_t)(byte_off + TRIG_FRAME_SIZE);
        ring_idx = (ring_idx + 1U) & (TRIG_RING_SIZE - 1U);
    }
    crc = TrigRing_CRC16(out_buf, (uint16_t)(len * TRIG_FRAME_SIZE));
    out_buf[byte_off] = (uint8_t)(crc & 0xFFU);
    out_buf[byte_off + 1U] = (uint8_t)(crc >> 8);
    return (uint16_t)(byte_off + 2U);
}

void TrigRing_Clear(void)
{
    s_state = TRIG_STATE_IDLE;
    s_source = TRIG_SRC_NONE;
    s_post_count = 0U;
    /* s_write_idx 不清: IDLE 态继续滚动, pre 段历史保持最新 */
}
