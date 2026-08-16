/**
 * @file    wheel_input.h
 * @brief   Scroll wheel input module — detent quantization, event generation,
 *          session keepalive, and W-frame binary output.
 * @note    APP_MODE_SCROLL_WHEEL shares detent force-computation with DETENT
 *          (same haptic-torque injection path), but owns independent config
 *          and wheel-specific event state via WHEEL:CFG.
 */

#ifndef __WHEEL_INPUT_H
#define __WHEEL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Binary frame constants ──────────────────────────────────────── */
#define WHEEL_EVENT_PAYLOAD_LEN   16U
#define WHEEL_EVENT_FRAME_LEN     21U   /* sync(2)+type(1)+len(1)+payload(16)+crc(1) */
#define WHEEL_EVENT_TYPE          'W'

/* ── Session timing ──────────────────────────────────────────────── */
#define WHEEL_SESSION_TIMEOUT_MS     1000U
#define WHEEL_KEEPALIVE_INTERVAL_MS   250U

/* ── Detent hysteresis ───────────────────────────────────────────── */
#define WHEEL_DETENT_HYSTERESIS  0.55f  /* fraction of detent_spacing to cross */

/* ── Event clamping ──────────────────────────────────────────────── */
#define WHEEL_DELTA_MAX_PER_FRAME  8    /* max |delta_steps| per W frame */

/* ── W-frame flags ───────────────────────────────────────────────── */
#define WHEEL_FLAG_MOTOR_RUNNING    (1U << 0)
#define WHEEL_FLAG_ENCODER_VALID    (1U << 1)
#define WHEEL_FLAG_DELTA_COALESCED  (1U << 2)
#define WHEEL_FLAG_SESSION_ACTIVE   (1U << 3)

/* ── Config ──────────────────────────────────────────────────────── */
typedef struct {
    float    count;             /* detents per revolution (e.g. 24) */
    float    strength;          /* A/rad */
    float    width_rad;         /* rad */
    float    damping;           /* A/(rad/s) */
    float    limit_A;           /* A */
} WheelConfig_t;

/* ── State ───────────────────────────────────────────────────────── */
typedef struct {
    /* ── Event quantization ── */
    int32_t  position_steps;      /* cumulative detent position (unbounded int) */
    float    unwrapped_pos_rad;   /* continuous unwrapped control-frame angle */
    float    prev_raw_pos_rad;    /* last raw position for unwrapping delta */
    float    last_detent_center;  /* center of current detent (unwrapped) */
    int32_t  current_detent_idx;  /* which detent we're in (int, unbounded) */
    int16_t  pending_delta;       /* delta not yet sent (coalesced by TX backpressure) */
    int16_t  delta_this_frame;    /* delta to send in WheelInput_Process() */

    /* ── Sequence ── */
    uint16_t seq;                 /* monotonic W-frame sequence number */
    uint16_t flags;               /* WHEEL_FLAG_* for next frame */
    float    speed_mrad_s;        /* speed in mrad/s for next frame */

    /* ── Session ── */
    uint32_t session_id;
    uint32_t last_keepalive_ms;
    uint8_t  session_active;

    /* ── Config (independent of DETENT) ── */
    WheelConfig_t cfg;
    uint8_t  cfg_dirty;           /* 1 = config updated, needs re-sync to foc_app */

    /* ── Statistics ── */
    uint32_t events_sent;
    uint32_t events_dropped;
    uint32_t events_coalesced;    /* count of times pending_delta absorbed overflow */
    uint32_t total_delta;         /* sum of |delta| for all events sent */

    /* ── First-frame gate ── */
    uint8_t  first_update;        /* 1 = need to seed prev_raw and last_detent */
} WheelInputState_t;

/* Global wheel input state */
extern WheelInputState_t g_wheel;

/* ── API ─────────────────────────────────────────────────────────── */

void WheelInput_Init(void);
void WheelInput_ResetState(void);

/**
 * @brief Feed a new position sample (called from SpeedLoop haptic path, 2kHz).
 * @param control_pos_rad  Control-frame mechanical position (unwrapped by caller via delta)
 *                          — actually raw, WheelInput unwraps internally
 * @param speed_rad_s      Mechanical speed, rad/s
 * @param motor_running    1 if PWM enabled and motor is active
 * @param encoder_valid    1 if encoder data is currently valid
 */
void WheelInput_Update(float control_pos_rad, float speed_rad_s,
                       uint8_t motor_running, uint8_t encoder_valid);

/**
 * @brief Send pending wheel events (called from main loop).
 * @note  Clamps per-frame delta to ±WHEEL_DELTA_MAX_PER_FRAME.
 *        On TX backpressure, coalesces remainder into pending_delta.
 *        Also checks session timeout internally.
 * @return 0 = normal, 1 = session just expired (caller must STOP motor)
 */
uint8_t WheelInput_Process(void);

/* ── Session ── */
void WheelInput_SessionStart(uint32_t id);
void WheelInput_SessionKeepalive(uint32_t id);
void WheelInput_SessionEnd(void);
uint8_t WheelInput_IsSessionActive(void);

/**
 * @brief Check session timeout and take action if expired.
 * @return 1 if session is still valid (or no session), 0 if expired and action taken.
 * @note  Called from WheelInput_Process(). On timeout: clears events,
 *        triggers STOP via FOC_App_Disable / mode reset.
 */
uint8_t WheelInput_CheckSessionTimeout(void);

/* ── Config ── */
void WheelInput_ApplyConfig(const WheelConfig_t *cfg);
void WheelInput_GetConfig(WheelConfig_t *cfg);

/* ── Statistics ── */
void WheelInput_GetStats(uint32_t *sent, uint32_t *dropped,
                         uint32_t *coalesced, uint32_t *total_delta);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_INPUT_H */
