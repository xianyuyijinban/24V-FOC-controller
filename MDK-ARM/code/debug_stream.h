/**
 * @file    debug_stream.h
 * @brief   PDBBIN 二进制调试流 — 文本 PDB 的二进制孪生, 带 seq + CRC8
 * @note    2026-08-30 任务卡 T2: 20kHz 时代高速段 PDB 丢窗不可定位,
 *          二进制帧带 seq(丢帧定位) + CRC8(校验), 与文本 PDB 同门控 200Hz。
 *
 * 帧格式 (复用 current_stream 骨架, CRC8 poly 0x07):
 *   A5 5A | type | payload_len | payload(37B) | CRC8
 *   type = 0x20 (DBG_TYPE_PDB2); payload 小端 packed, 见 PdbBinPayload_t。
 *
 * 语义: seq/tick 在发射点赋值 (主循环 drain 时补会比实际晚, 丢帧定位失真)。
 */

#ifndef __DEBUG_STREAM_H
#define __DEBUG_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 帧常量 */
#define DBG_TYPE_PDB2           0x20U   /* 0x20 起第一个空闲 type ('C'/'W' 已占) */
#define DBG_TYPE_PDB2V2         0x21U   /* v2: v1 37B 尾部追加 3×float (2026-09-09 ①) */
#define DBG_PDB_FRAME_LEN       42U     /* sync(2)+type(1)+len(1)+payload(37)+crc(1) */
#define DBG_PDB_PAYLOAD_LEN     37U
#define DBG_PDBV2_FRAME_LEN     54U     /* sync(2)+type(1)+len(1)+payload(49)+crc(1) */
#define DBG_PDBV2_PAYLOAD_LEN   49U

/* Payload 定点序 packed (小端): 1+4+4+7×4 = 37B */
typedef struct {
    uint8_t  seq;       /* 发射点递增, 丢帧定位核心 */
    uint32_t tick_2khz; /* 硬件时基 tick (t=tick/2000) */
    uint32_t flags;     /* 次 8 位=state (bits 15:8), 低 8 位=fault_code (2026-09-04);
                         * bit16=pos_aw_esc_active 僵持逃逸态 (2026-09-06) */
    float    pos_err_rad;   /* 位置误差 rad (control 帧, =文本 PDB p[1]) */
    float    iq_cmd;        /* FF 前位置环指令 A (= p[7]) */
    float    ff_total;      /* FF 层总注入 A (= p[9]) */
    float    theta_user_rad;/* 用户帧机械角 rad (= p[10]) */
    float    iq_act;        /* 实测 q 电流 A */
    float    v_mech_rad_s;  /* 实测机械速度 rad/s */
    float    pos_ref_rad;   /* 位置给定 rad (control 帧) */
} PdbBinPayload_t;

/* v2 payload: v1 37B 原样前缀 + 尾部追加 3×float = 49B (2026-09-09 ①)
 * 追加字段口径 (取数点 2026-09-09 核实):
 *   ff_coulomb   — 库仑分量实际生效值: Stribeck smooth 后 × 方向锁存
 *                  (foc_app.c Coulomb 段 coulomb*smooth, 粘滞/观测器项不含)
 *   ff_cogging   — COG LUT 输出分量 (LUT 关闭时照实输出 0) = ff_diag.cogging_iq
 *   pos_integral — 位置环积分通道饱和后实际生效输出 (ki_out 口径,
 *                  AW1 回拉/ESC 放门作用后) = pos_ki_out_prev */
typedef struct {
    PdbBinPayload_t v1;         /* 前 37B: 逐比特与 v1 一致 */
    float    ff_coulomb;        /* 库仑分量 A (粘滞/观测器项不含) */
    float    ff_cogging;        /* COG LUT 分量 A */
    float    pos_integral;      /* 位置环积分生效输出 A (ki_out 口径) */
} PdbBinV2Payload_t;

/* ── API ─────────────────────────────────────────────────────── */

void DebugStream_Init(void);

/* 200Hz 发射点调用 (主循环, 与文本 PDB 同一 gate); 内部自递增 seq,
 * 组帧 -> DrvUart_SendBytesP1 (P1 准入). */
void DebugStream_PushPdb(uint32_t tick_2khz, const PdbBinPayload_t *p);

/* v2 发射点调用: payload 49B (v1 前缀逐比特一致 + 3 追加字段). */
void DebugStream_PushPdbV2(uint32_t tick_2khz, const PdbBinV2Payload_t *p);

/* 当前 PDBBIN 版本 (0=关 1=v1 2=v2), CMD:PDBBIN,? 上报用. */
uint8_t DebugStream_GetVer(void);

/* 设发射帧版本 (1=v1 2=v2); CMD:PDBBIN,1|2 设置, 越界忽略. */
void DebugStream_SetVer(uint8_t ver);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_STREAM_H */
