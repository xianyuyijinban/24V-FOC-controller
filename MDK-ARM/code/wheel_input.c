/**
 * @file    wheel_input.c
 * @brief   Scroll wheel input — continuous angle unwrapping, detent state machine
 *          with hysteresis, W-frame binary event output, session keepalive.
 *
 * Architecture:
 *  - WheelInput_Update()  — called from SpeedLoop haptic path (2 kHz)
 *  - WheelInput_Process() — called from main loop (~200 Hz), drains pending events
 *  - W frames use the shared CurStream_BuildFrame() envelope, sent at P0 priority
 */

#include "wheel_input.h"
#include "current_stream.h"    /* CurStream_BuildFrame() */
#include "uart_upload.h"       /* DrvUart_SendBytesP0() */
#include "foc_core.h"          /* FOC_PI, FOC_AngleNormalize, FOC_Saturate */
#include "stm32h7xx_hal.h"     /* HAL_GetTick() */
#include <string.h>
#include <math.h>              /* roundf, fabsf */

/* ── Global state ─────────────────────────────────────────────────── */
WheelInputState_t g_wheel;

/* ── Helpers ──────────────────────────────────────────────────────── */

/** Wrap delta to [-PI, PI] for continuous angle tracking */
static float wrap_delta(float delta)
{
    while (delta > FOC_PI)  delta -= 2.0f * FOC_PI;
    while (delta < -FOC_PI) delta += 2.0f * FOC_PI;
    return delta;
}

/* ──────────────────────────────────────────────────────────────────
 *  Public API
 * ────────────────────────────────────────────────────────────────── */

void WheelInput_Init(void)
{
    memset(&g_wheel, 0, sizeof(g_wheel));
    /* Default config — will be overwritten by WHEEL:CFG or foc_app defaults */
    g_wheel.cfg.count     = 24.0f;
    g_wheel.cfg.strength  = 6.0f;
    g_wheel.cfg.width_rad = 0.16f;
    g_wheel.cfg.damping   = 0.10f;
    g_wheel.cfg.limit_A   = 0.30f;
    g_wheel.first_update  = 1U;
}

void WheelInput_ResetState(void)
{
    g_wheel.position_steps     = 0;
    g_wheel.unwrapped_pos_rad  = 0.0f;
    g_wheel.prev_raw_pos_rad   = 0.0f;
    g_wheel.last_detent_center = 0.0f;
    g_wheel.current_detent_idx = 0;
    g_wheel.pending_delta      = 0;
    g_wheel.delta_this_frame   = 0;
    g_wheel.seq                = 0;
    g_wheel.flags              = 0;
    g_wheel.speed_mrad_s       = 0;
    g_wheel.first_update       = 1U;
    g_wheel.session_active     = 0U;
    g_wheel.session_id         = 0U;
    g_wheel.last_keepalive_ms  = 0U;
}

/**
 * @brief Feed a new position sample for detent quantization.
 *
 * Continuous unwrapping: each call computes the delta from the previous
 * raw position, wraps it to [-PI, PI], and accumulates into unwrapped_pos_rad.
 * This allows infinite forward/reverse rotation without 0/360° wrap artifacts.
 *
 * Detent state machine with hysteresis:
 *  - detent_spacing = 2π / count
 *  - On first call: snap to nearest detent center as starting point
 *  - Hysteresis threshold: 0.55 × spacing (must cross majority of detent to switch)
 *  - On crossing: increment/decrement delta_steps, advance detent center
 *  - Multi-detent: handle fast spins that cross 2+ detents in one sample
 */
void WheelInput_Update(float control_pos_rad, float speed_rad_s,
                       uint8_t motor_running, uint8_t encoder_valid)
{
    float spacing, raw;
    float error;
    int8_t  direction;

    if (g_wheel.cfg.count < 1.0f) {
        return;  /* invalid config */
    }

    spacing = (2.0f * FOC_PI) / g_wheel.cfg.count;
    raw     = control_pos_rad;

    /* ── Continuous unwrapping ── */
    if (g_wheel.first_update) {
        g_wheel.unwrapped_pos_rad  = raw;
        g_wheel.prev_raw_pos_rad   = raw;
        /* Snap to nearest detent center as starting point */
        g_wheel.last_detent_center = roundf(raw / spacing) * spacing;
        g_wheel.current_detent_idx = (int32_t)roundf(raw / spacing);
        g_wheel.first_update       = 0U;
    } else {
        float delta = raw - g_wheel.prev_raw_pos_rad;
        delta = wrap_delta(delta);
        g_wheel.unwrapped_pos_rad += delta;
        g_wheel.prev_raw_pos_rad   = raw;
    }

    /* ── Speed ── */
    g_wheel.speed_mrad_s = speed_rad_s * 1000.0f;

    /* ── Detent state machine with hysteresis ── */
    error = g_wheel.unwrapped_pos_rad - g_wheel.last_detent_center;

    /* Multi-detent crossing with hysteresis:
     * Cross to next detent when error exceeds 0.55 × spacing (more than half).
     * Continue in a loop to handle fast spins that jump 2+ detents at once. */
    while (fabsf(error) >= WHEEL_DETENT_HYSTERESIS * spacing) {
        direction = (error > 0.0f) ? 1 : -1;
        g_wheel.current_detent_idx += direction;
        g_wheel.last_detent_center += (float)direction * spacing;
        if (g_wheel.session_active) {
            g_wheel.delta_this_frame += direction;
        }

        error = g_wheel.unwrapped_pos_rad - g_wheel.last_detent_center;
    }

    /* ── Build flags ── */
    g_wheel.flags = 0;
    if (motor_running)  g_wheel.flags |= WHEEL_FLAG_MOTOR_RUNNING;
    if (encoder_valid)  g_wheel.flags |= WHEEL_FLAG_ENCODER_VALID;
    if (g_wheel.session_active) g_wheel.flags |= WHEEL_FLAG_SESSION_ACTIVE;
}

/**
 * @brief Send pending wheel events (called from main loop).
 *
 * Clamps per-frame delta to ±WHEEL_DELTA_MAX_PER_FRAME.
 * On TX backpressure, leaves remainder in pending_delta for next cycle.
 *
 * @return 0 = normal, 1 = session just expired (caller must STOP motor)
 */
uint8_t WheelInput_Process(void)
{
    uint8_t  frame_buf[WHEEL_EVENT_FRAME_LEN];
    uint8_t  payload[WHEEL_EVENT_PAYLOAD_LEN];
    int16_t  to_send;
    int16_t  clamped;
    uint32_t now_ms;

    /* ── Session timeout check ── */
    /* W frames are owned input events and require a live bridge session. */
    if (!g_wheel.session_active) {
        __disable_irq();
        g_wheel.pending_delta = 0;
        g_wheel.delta_this_frame = 0;
        __enable_irq();
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - g_wheel.last_keepalive_ms) > WHEEL_SESSION_TIMEOUT_MS) {
        WheelInput_SessionEnd();
        return 1U;
    }

    /* ── Combine pending + new delta (critical section: ISR writes delta_this_frame) ── */
    __disable_irq();
    to_send = g_wheel.pending_delta + g_wheel.delta_this_frame;
    g_wheel.pending_delta    = 0;
    g_wheel.delta_this_frame = 0;
    __enable_irq();

    if (to_send == 0) {
        return 0U;  /* nothing to send */
    }

    /* ── Clamp per-frame delta ── */
    if (to_send > WHEEL_DELTA_MAX_PER_FRAME) {
        g_wheel.pending_delta = (int16_t)(to_send - WHEEL_DELTA_MAX_PER_FRAME);
        clamped = WHEEL_DELTA_MAX_PER_FRAME;
    } else if (to_send < -WHEEL_DELTA_MAX_PER_FRAME) {
        g_wheel.pending_delta = (int16_t)(to_send + WHEEL_DELTA_MAX_PER_FRAME);
        clamped = (int16_t)(-WHEEL_DELTA_MAX_PER_FRAME);
    } else {
        clamped = to_send;
    }

    if (g_wheel.pending_delta != 0) {
        g_wheel.events_coalesced++;
        g_wheel.flags |= WHEEL_FLAG_DELTA_COALESCED;
    }

    /* ── Update cumulative position ── */
    g_wheel.position_steps += clamped;

    /* ── Pack W-frame payload (16 bytes, little-endian) ── */
    now_ms = HAL_GetTick();
    payload[0]  = (uint8_t)(g_wheel.seq & 0xFFU);
    payload[1]  = (uint8_t)((g_wheel.seq >> 8) & 0xFFU);
    payload[2]  = (uint8_t)(now_ms & 0xFFU);
    payload[3]  = (uint8_t)((now_ms >> 8) & 0xFFU);
    payload[4]  = (uint8_t)((now_ms >> 16) & 0xFFU);
    payload[5]  = (uint8_t)((now_ms >> 24) & 0xFFU);
    payload[6]  = (uint8_t)(clamped & 0xFFU);
    payload[7]  = (uint8_t)((clamped >> 8) & 0xFFU);
    payload[8]  = (uint8_t)(g_wheel.position_steps & 0xFFU);
    payload[9]  = (uint8_t)((g_wheel.position_steps >> 8) & 0xFFU);
    payload[10] = (uint8_t)((g_wheel.position_steps >> 16) & 0xFFU);
    payload[11] = (uint8_t)((g_wheel.position_steps >> 24) & 0xFFU);
    payload[12] = (uint8_t)((int16_t)(g_wheel.speed_mrad_s) & 0xFFU);
    payload[13] = (uint8_t)(((int16_t)(g_wheel.speed_mrad_s) >> 8) & 0xFFU);
    payload[14] = (uint8_t)(g_wheel.flags & 0xFFU);
    payload[15] = (uint8_t)((g_wheel.flags >> 8) & 0xFFU);

    /* ── Build and send frame (P0 priority) ── */
    (void)CurStream_BuildFrame(WHEEL_EVENT_TYPE, payload,
                               WHEEL_EVENT_PAYLOAD_LEN, frame_buf);

    if (DrvUart_SendBytesP0(frame_buf, WHEEL_EVENT_FRAME_LEN)) {
        g_wheel.events_sent++;
        g_wheel.total_delta += (uint32_t)((clamped > 0) ? clamped : -clamped);
        g_wheel.seq++;
    } else {
        /* TX failed — push delta back into pending for retry */
        g_wheel.pending_delta = (int16_t)(g_wheel.pending_delta + clamped);
        g_wheel.position_steps -= clamped;  /* undo position advance */
        g_wheel.events_dropped++;
    }
    return 0U;
}

/* ──────────────────────────────────────────────────────────────────
 *  Session
 * ────────────────────────────────────────────────────────────────── */

void WheelInput_SessionStart(uint32_t id)
{
    uint32_t now_ms = HAL_GetTick();

    __disable_irq();
    g_wheel.position_steps     = 0;
    g_wheel.pending_delta      = 0;
    g_wheel.delta_this_frame   = 0;
    g_wheel.seq                = 0U;
    g_wheel.first_update       = 1U;
    g_wheel.session_active    = 1U;
    g_wheel.session_id        = id;
    g_wheel.last_keepalive_ms = now_ms;
    __enable_irq();
}

void WheelInput_SessionKeepalive(uint32_t id)
{
    if (g_wheel.session_active && g_wheel.session_id == id) {
        g_wheel.last_keepalive_ms = HAL_GetTick();
    }
}

void WheelInput_SessionEnd(void)
{
    g_wheel.session_active    = 0U;
    g_wheel.session_id        = 0U;
    g_wheel.last_keepalive_ms = 0U;
    g_wheel.pending_delta     = 0;
    g_wheel.delta_this_frame  = 0;
    g_wheel.flags             = 0;
}

uint8_t WheelInput_IsSessionActive(void)
{
    return g_wheel.session_active;
}

uint8_t WheelInput_CheckSessionTimeout(void)
{
    uint32_t now_ms;

    if (!g_wheel.session_active) {
        return 0U;  /* no session to time out */
    }

    now_ms = HAL_GetTick();
    if ((now_ms - g_wheel.last_keepalive_ms) > WHEEL_SESSION_TIMEOUT_MS) {
        WheelInput_SessionEnd();
        return 1U;  /* expired — caller must STOP motor */
    }
    return 0U;
}

/* ──────────────────────────────────────────────────────────────────
 *  Config
 * ────────────────────────────────────────────────────────────────── */

void WheelInput_ApplyConfig(const WheelConfig_t *cfg)
{
    if (cfg == NULL) return;

    if (cfg->count < 1.0f) {
        g_wheel.cfg.count = 1.0f;
    } else {
        g_wheel.cfg.count = cfg->count;
    }

    g_wheel.cfg.strength  = (cfg->strength  < 0.0f) ? 0.0f  : cfg->strength;
    g_wheel.cfg.width_rad = (cfg->width_rad < 0.01f)? 0.01f : cfg->width_rad;
    g_wheel.cfg.damping   = (cfg->damping   < 0.0f) ? 0.0f  : cfg->damping;

    if (cfg->limit_A < 0.01f) {
        g_wheel.cfg.limit_A = 0.01f;
    } else if (cfg->limit_A > 1.0f) {
        g_wheel.cfg.limit_A = 1.0f;
    } else {
        g_wheel.cfg.limit_A = cfg->limit_A;
    }

    g_wheel.cfg_dirty = 1U;

    /* Reset the detent tracking since count may have changed */
    g_wheel.first_update = 1U;
}

void WheelInput_GetConfig(WheelConfig_t *cfg)
{
    if (cfg == NULL) return;
    cfg->count     = g_wheel.cfg.count;
    cfg->strength  = g_wheel.cfg.strength;
    cfg->width_rad = g_wheel.cfg.width_rad;
    cfg->damping   = g_wheel.cfg.damping;
    cfg->limit_A   = g_wheel.cfg.limit_A;
}

/* ──────────────────────────────────────────────────────────────────
 *  Statistics
 * ────────────────────────────────────────────────────────────────── */

void WheelInput_GetStats(uint32_t *sent, uint32_t *dropped,
                         uint32_t *coalesced, uint32_t *total_delta)
{
    if (sent)       *sent       = g_wheel.events_sent;
    if (dropped)    *dropped    = g_wheel.events_dropped;
    if (coalesced)  *coalesced  = g_wheel.events_coalesced;
    if (total_delta)*total_delta = g_wheel.total_delta;
}
