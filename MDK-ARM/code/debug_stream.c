/**
 * @file    debug_stream.c
 * @brief   PDBBIN 二进制调试流实现 — 见 debug_stream.h (v1 37B / v2 49B, 2026-09-09 ①)
 */

#include "debug_stream.h"
#include "current_stream.h"   /* CurStream_BuildFrame / CurStream_CRC8 复用 (任务卡: 不复制) */
#include "uart_upload.h"
#include <string.h>

static uint8_t s_dbg_seq = 0U;      /* 发射点递增 (u8 回绕, 主机按 mod 256 判 gap) */
static uint8_t s_pdbbin_ver = 1U;   /* 发射帧版本: 1=v1 (37B), 2=v2 (49B); 0=关由 it.c 门控 */
static uint8_t s_dbg_frame[DBG_PDBV2_FRAME_LEN];

void DebugStream_Init(void)
{
    s_dbg_seq = 0U;
    s_pdbbin_ver = 1U;
}

void DebugStream_SetVer(uint8_t ver)
{
    if ((ver >= 1U) && (ver <= 2U)) {
        s_pdbbin_ver = ver;
    }
}

uint8_t DebugStream_GetVer(void)
{
    return s_pdbbin_ver;
}

/* v1/v2 共用打包: 前 37B 两版逐比特一致; v2 再追加 3×float.
 * seq 在此统一自递增 (v1/v2 同一 seq 空间, 主机 per-type 独立追踪). */
static void DebugStream_Pack(uint8_t type, uint32_t tick_2khz,
                             const PdbBinPayload_t *p,
                             const float *extra, uint8_t extra_cnt)
{
    uint8_t payload[DBG_PDBV2_PAYLOAD_LEN];
    uint8_t payload_len = DBG_PDB_PAYLOAD_LEN;
    int16_t off;

    if (p == NULL) {
        return;
    }

    payload[0] = s_dbg_seq++;

    /* 小端 packed: u32/f32 均为小端 (ARM Cortex-M7), memcpy 直拷 */
    memcpy(&payload[1], &tick_2khz, 4U);
    memcpy(&payload[5], &p->flags, 4U);  /* flags: 高 8 位=state, 低 8 位=fault_code (2026-09-04) */
    off = 9;
    memcpy(&payload[off], &p->pos_err_rad,  4U); off += 4;
    memcpy(&payload[off], &p->iq_cmd,       4U); off += 4;
    memcpy(&payload[off], &p->ff_total,     4U); off += 4;
    memcpy(&payload[off], &p->theta_user_rad, 4U); off += 4;
    memcpy(&payload[off], &p->iq_act,       4U); off += 4;
    memcpy(&payload[off], &p->v_mech_rad_s, 4U); off += 4;
    memcpy(&payload[off], &p->pos_ref_rad,  4U); off += 4;

    if ((extra != NULL) && (extra_cnt != 0U)) {
        uint8_t i;
        for (i = 0U; i < extra_cnt; i++) {
            memcpy(&payload[off], &extra[i], 4U);
            off = (int16_t)(off + 4);
        }
        payload_len = DBG_PDBV2_PAYLOAD_LEN;
    }

    {
        uint8_t frame_len = CurStream_BuildFrame(type, payload,
                                                 payload_len, s_dbg_frame);
        (void)DrvUart_SendBytesP1(s_dbg_frame, frame_len);
    }
}

void DebugStream_PushPdb(uint32_t tick_2khz, const PdbBinPayload_t *p)
{
    DebugStream_Pack(DBG_TYPE_PDB2, tick_2khz, p, NULL, 0U);
}

void DebugStream_PushPdbV2(uint32_t tick_2khz, const PdbBinV2Payload_t *p)
{
    float extra[3];
    if (p == NULL) {
        return;
    }
    extra[0] = p->ff_coulomb;
    extra[1] = p->ff_cogging;
    extra[2] = p->pos_integral;
    DebugStream_Pack(DBG_TYPE_PDB2V2, tick_2khz, &p->v1, extra, 3U);
}
