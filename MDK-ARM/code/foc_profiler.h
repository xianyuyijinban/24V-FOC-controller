#ifndef FOC_PROFILER_H
#define FOC_PROFILER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include <stdint.h>

#define FOC_PROFILER_TIM1_BUDGET_US 50U
#define FOC_PROFILER_FOC_BUDGET_US  100U

#ifndef LOOP_PROF_EN
#define LOOP_PROF_EN 1
#endif

typedef enum {
    FOC_PROBE_FOC_RUN = 0,
    FOC_PROBE_CURRENT_PATH,
    FOC_PROBE_SPEED_LOOP,
    FOC_PROBE_POSITION_LOOP,
    FOC_PROBE_TIM1_ISR,
    FOC_PROBE_IRQ_PERIOD,
    FOC_PROBE_MAIN_LOOP,       /* 主循环整体 (while 1 体) */
    FOC_PROBE_CMD_PREF,        /* PREF 命令分支单独 */
    FOC_PROBE_POSDBG,          /* PDBBIN/PDB 发射服务 */
#if LOOP_PROF_EN
    FOC_PROBE_SEG_CMD,         /* 主循环分段: UART_Command_ProcessPending */
    FOC_PROBE_SEG_PREF,        /* 分段: PREF 分支 (净耗时, ISR 税校正) */
    FOC_PROBE_SEG_PDB,         /* 分段: PDBBIN 发射 (UART_CommandServicePosdbg, 税校正) */
    FOC_PROBE_SEG_NFRAME,      /* 分段: DrvUart_上传服务 (含 N 帧) (税校正) */
    FOC_PROBE_SEG_OTHER,       /* 分段: 主循环其余 (FOC_App_MainLoop/DRV/CurStream) */
#endif
    FOC_PROBE_COUNT
} FOC_ProfilerProbe_t;

typedef struct {
    uint32_t count;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint64_t sum_cycles;
    uint32_t overrun_count;
} FOC_ProfilerStat_t;

typedef struct {
    uint8_t enabled;
    uint32_t cpu_hz;
    uint32_t probe_overhead_cycles;
    uint32_t budget_cycles[FOC_PROBE_COUNT];
    FOC_ProfilerStat_t stats[FOC_PROBE_COUNT];
} FOC_ProfilerSnapshot_t;

void FOC_Profiler_Init(void);
void FOC_Profiler_Clear(void);
void FOC_Profiler_RecordCycles(FOC_ProfilerProbe_t probe, uint32_t raw_cycles);
void FOC_Profiler_RecordIrqEntry(uint32_t entry_cycle);
void FOC_Profiler_GetSnapshot(FOC_ProfilerSnapshot_t *snapshot);
const char *FOC_Profiler_ProbeName(FOC_ProfilerProbe_t probe);

static inline uint32_t FOC_Profiler_Begin(void)
{
    return DWT->CYCCNT;
}

static inline void FOC_Profiler_End(FOC_ProfilerProbe_t probe, uint32_t start_cycle)
{
    uint32_t elapsed = DWT->CYCCNT - start_cycle;
    FOC_Profiler_RecordCycles(probe, elapsed);
}

#if LOOP_PROF_EN
/* ISR 税校正: TIM1_UP ISR 把自身耗时累入 64 位账本; 主循环分段结算时
 * 净耗时 = 墙钟差 − 段内 ISR 累计差值。分段可嵌套 (SEG_CMD ⊃ SEG_PREF/SEG_PDB)。 */
typedef struct {
    uint32_t wall;       /* 段墙钟起点 (CYCCNT) */
    uint64_t isr;        /* 段 ISR 账本起点 */
} FOC_LoopSegHandle_t;

typedef struct {
    uint32_t iter_count;    /* 迭代次数 */
    uint64_t iter_sum_cyc;  /* 迭代墙钟间隔和 */
    uint32_t iter_min_cyc;  /* 迭代墙钟间隔 min/max */
    uint32_t iter_max_cyc;
    uint64_t isr_sum_cyc;   /* 每迭代 ISR 税 (TIM1 抢占) 累计 */
    uint32_t isr_max_cyc;   /* 每迭代 ISR 税 max */
    uint32_t tail_isr;      /* 当前迭代 ISR 税 (与迭代间隔同时段) */
} FOC_LoopProfIter_t;

void FOC_Profiler_RecordTim1IsrCycles(uint32_t isr_cycles);
void FOC_Profiler_LoopBegin(void);
void FOC_Profiler_LoopEnd(void);
FOC_LoopProfIter_t FOC_Profiler_LoopIterStats(void);
/* 分段: SegBegin 快照墙钟+ISR 起点; SegEnd 结算净耗时 (墙钟−ISR 差) 记入探针并返回 */
void FOC_Profiler_SegBegin(FOC_LoopSegHandle_t *h);
uint32_t FOC_Profiler_SegEnd(FOC_LoopSegHandle_t *h, FOC_ProfilerProbe_t probe);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FOC_PROFILER_H */
