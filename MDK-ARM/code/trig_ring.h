/**
 * @file    trig_ring.h
 * @brief   TRIG 故障触发 ring buffer — 20kHz 电流环原速采样 + 触发冻结 + 分块拉取
 * @note    2026-09-09 ③协议增量 (docs/plans/2026-09-09-ai-link-taskcards-deepseek.md)。
 *          任务卡口径 "2kHz" 实为电流环 20kHz (FOC_CONTROL_FREQ=20000, TIM1 下溢沿);
 *          验尸窗 = pre 768 + 触发帧 1 + post 256 = 1025 帧 @20kHz (51.25ms)。
 *
 * 写入: 20kHz ISR 内 O(1) 单写者无锁 (写指针自进, 读侧只在触发冻结后访问)。
 * 触发: fault 闩锁 / state 掉出 RUNNING (主循环监视) / CMD:TRIG,NOW (手动合成)。
 * 触发后 post 段 (256 帧) 计满即冻结 (防覆写 pre 段历史)。
 * 拉取: CMD:TRIG,PULL,off,len 分块上行 (≤32 帧/块), 每块 CRC16 (CCITT-FALSE,
 *       poly 0x1021 init 0xFFFF); 拉取期间 PDBBIN 暂停 (it.c 门控), 拉完恢复。
 *
 * RAM: 2048 × 28B = 56KB (DTCM 128KB 内; 1024 槽时 post 末帧恰好覆写 pre 首帧
 *      — trig_idx+256 ≡ trig_idx-768 (mod 1024), 212303 台架实证)。
 */

#ifndef __TRIG_RING_H
#define __TRIG_RING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 2048 槽: 验尸窗 pre 768 + 触发帧 1 + post 256 = 1025 > 1024 — 原 1024 槽
 * 时 post 末帧恰好覆写 pre 首帧 (trig_idx+256 ≡ trig_idx-768 mod 1024,
 * 212303 台架实证), 取下一 2 的幂 2048 */
#define TRIG_RING_SIZE      2048U   /* 帧数 (2 的幂, 索引 &2047) */
#define TRIG_PRE_FRAMES     768U    /* 触发点前保留帧 (75%) */
#define TRIG_POST_FRAMES    256U    /* 触发点后采集帧 (25%), 计满冻结 */
#define TRIG_FRAME_SIZE     28U     /* 7×float32 */
#define TRIG_WINDOW_FRAMES  (TRIG_PRE_FRAMES + 1U + TRIG_POST_FRAMES)  /* 1025 帧验尸窗 */
#define TRIG_PULL_MAX_BYTES  896U   /* 单块数据上限 32 帧 × 28B (TX ring 1024B 预算:
                                     * 头 ~17B + 896B + CRC 2B < 1024B) */
#define TRIG_PULL_MAX_FRAMES 32U    /* 单块帧数上限 (与上式同步) */

/* CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, 无反射无异或输出) */
uint16_t TrigRing_CRC16(const uint8_t *data, uint32_t len);

/* 帧布局 (28B 小端 packed, float 直拷 — Cortex-M7 小端): */
typedef struct {
    float id;           /* D 轴电流反馈 A (foc.Idq.d) */
    float iq;           /* Q 轴电流反馈 A (foc.Idq.q) */
    float vd;           /* D 轴电压指令 V (foc.Vdq.d) */
    float vq;           /* Q 轴电压指令 V (foc.Vdq.q) */
    float theta_elec;   /* 电角度 rad (foc.theta_elec) */
    float iq_ref;       /* Q 轴电流指令 A (handle->Iq_ref) */
    uint32_t tick_20k;  /* 电流环节拍 (handle->control_count, t=tick/20000) */
} TrigFrame_t;

typedef enum {
    TRIG_STATE_IDLE = 0,     /* 环形滚动, 未触发 */
    TRIG_STATE_POST,         /* 已触发, post 段采集中 */
    TRIG_STATE_FROZEN,       /* post 计满, 冻结待拉取 */
} TrigState_t;

typedef enum {
    TRIG_SRC_NONE = 0,
    TRIG_SRC_FAULT,          /* fault 闩锁 / state 掉出 RUNNING */
    TRIG_SRC_MANUAL,         /* CMD:TRIG,NOW */
} TrigSource_t;

/* ── API ─────────────────────────────────────────────────────── */

void TrigRing_Init(void);

/* 20kHz 电流环采样点调用 (TIM1 ISR, exit_cycle 前): O(1) 写 ring;
 * IDLE 态滚动覆写; POST 态推进 post 计数, 计满转 FROZEN; FROZEN 态丢弃。 */
void TrigRing_Sample(float id, float iq, float vd, float vq,
                     float theta_elec, float iq_ref, uint32_t tick_20k);

/* 触发 (主循环 fault 监视或 CMD:TRIG,NOW): IDLE → POST。
 * 返回 1 = 接受 (本拍起算 post), 0 = 忽略 (非 IDLE 态)。 */
uint8_t TrigRing_Trigger(TrigSource_t src);

/* 状态查询 (CMD:TRIG,STAT?): 0=IDLE 1=POST(采集中) 2=FROZEN(待拉取) */
TrigState_t TrigRing_GetState(void);
TrigSource_t TrigRing_GetSource(void);
uint32_t TrigRing_GetTrigTick(void);    /* 触发帧 tick_20k (触发即锁存) */
uint32_t TrigRing_GetPostCount(void);   /* 已采 post 帧数 (FROZEN=TRIG_POST_FRAMES) */

/* 分块拉取 (CMD:TRIG,PULL,off,len): off/len 按帧序号 (0..1023, 触发帧 = 帧 768)。
 * 输出块 = len×28B 数据 + CRC16 (2B, LE) 尾缀; 返回块总字节数 (写进 out_buf,
 * 需 ≥ TRIG_PULL_MAX_BYTES + 2), 0 = 参数非法或非 FROZEN 态。
 * 环序: 帧 i 的 ring 索引 = (trig_idx + i - TRIG_PRE_FRAMES) & (SIZE-1),
 *       即帧 0 = 触发前 768 帧, 帧 767 = 触发前一帧, 帧 768 = 触发帧。 */
uint16_t TrigRing_Pull(uint16_t off, uint16_t len, uint8_t *out_buf);

/* 独立 20kHz 时基 (IDLE 态也递增 — control_count 在非 RUNNING 拍被跳过,
 * 无法作验尸时基); ISR 每拍调一次, Sample 用 GetTick() 取值。 */
void TrigRing_TickTick(void);
uint32_t TrigRing_GetTick(void);

/* 复位 (CMD:TRIG,CLR): 任意态 → IDLE, 重新滚动可再触发 */
void TrigRing_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRIG_RING_H */
