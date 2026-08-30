/**
 * @file    debug_stream.c
 * @brief   PDBBIN 二进制调试流实现 — 见 debug_stream.h
 */

#include "debug_stream.h"
#include "current_stream.h"   /* CurStream_BuildFrame / CurStream_CRC8 复用 (任务卡: 不复制) */
#include "uart_upload.h"
#include <string.h>

static uint8_t s_dbg_seq = 0U;      /* 发射点递增 (u8 回绕, 主机按 mod 256 判 gap) */
static uint8_t s_dbg_frame[DBG_PDB_FRAME_LEN];

void DebugStream_Init(void)
{
    s_dbg_seq = 0U;
}

void DebugStream_PushPdb(uint32_t tick_2khz, const PdbBinPayload_t *p)
{
    uint8_t payload[DBG_PDB_PAYLOAD_LEN];
    int16_t off;
    uint8_t frame_len;

    if (p == NULL) {
        return;
    }

    payload[0] = s_dbg_seq++;

    /* 小端 packed: u32/f32 均为小端 (ARM Cortex-M7), memcpy 直拷 */
    memcpy(&payload[1], &tick_2khz, 4U);
    payload[5] = 0U; payload[6] = 0U; payload[7] = 0U; payload[8] = 0U;  /* flags=0 */
    off = 9;
    memcpy(&payload[off], &p->pos_err_rad,  4U); off += 4;
    memcpy(&payload[off], &p->iq_cmd,       4U); off += 4;
    memcpy(&payload[off], &p->ff_total,     4U); off += 4;
    memcpy(&payload[off], &p->theta_user_rad, 4U); off += 4;
    memcpy(&payload[off], &p->iq_act,       4U); off += 4;
    memcpy(&payload[off], &p->v_mech_rad_s, 4U); off += 4;
    memcpy(&payload[off], &p->pos_ref_rad,  4U); off += 4;

    frame_len = CurStream_BuildFrame(DBG_TYPE_PDB2, payload,
                                     DBG_PDB_PAYLOAD_LEN, s_dbg_frame);
    (void)DrvUart_SendBytesP1(s_dbg_frame, frame_len);
}
