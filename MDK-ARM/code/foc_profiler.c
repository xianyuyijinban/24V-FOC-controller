#include "foc_profiler.h"

#include <limits.h>
#include <string.h>

#define FOC_DWT_UNLOCK_KEY 0xC5ACCE55UL
#define FOC_PROFILER_OVERHEAD_SAMPLES 32U

static volatile uint8_t s_enabled;
static uint32_t s_cpuHz;
static uint32_t s_probeOverheadCycles;
static uint32_t s_lastIrqEntryCycle;
static uint32_t s_budgetCycles[FOC_PROBE_COUNT];
static volatile FOC_ProfilerStat_t s_stats[FOC_PROBE_COUNT];

#if LOOP_PROF_EN
static volatile uint64_t s_tim1IsrAccum;
static uint32_t s_iterCount;
static uint64_t s_iterSumCyc;
static uint32_t s_iterMinCyc;
static uint32_t s_iterMaxCyc;
static uint64_t s_isrSumCyc;
static uint32_t s_isrMaxCyc;
static uint32_t s_loopPrevWall;
static uint64_t s_loopPrevIsr;
#endif

static uint32_t FOC_Profiler_UsToCycles(uint32_t us)
{
    uint64_t cycles = ((uint64_t)s_cpuHz * (uint64_t)us) / 1000000ULL;
    return (cycles > UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;
}

static void FOC_Profiler_ResetStats(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)FOC_PROBE_COUNT; ++i) {
        s_stats[i].count = 0U;
        s_stats[i].min_cycles = UINT32_MAX;
        s_stats[i].max_cycles = 0U;
        s_stats[i].sum_cycles = 0ULL;
        s_stats[i].overrun_count = 0U;
    }
    s_lastIrqEntryCycle = 0U;
#if LOOP_PROF_EN
    s_iterCount = 0U;
    s_iterSumCyc = 0ULL;
    s_iterMinCyc = 0U;
    s_iterMaxCyc = 0U;
    s_isrSumCyc = 0ULL;
    s_isrMaxCyc = 0U;
#endif
}

static void FOC_Profiler_RecordAdjusted(FOC_ProfilerProbe_t probe, uint32_t cycles)
{
    volatile FOC_ProfilerStat_t *stat;
    uint32_t budget;

    if ((s_enabled == 0U) || ((uint32_t)probe >= (uint32_t)FOC_PROBE_COUNT)) {
        return;
    }

    stat = &s_stats[(uint32_t)probe];
    stat->count++;
    if (cycles < stat->min_cycles) {
        stat->min_cycles = cycles;
    }
    if (cycles > stat->max_cycles) {
        stat->max_cycles = cycles;
    }
    stat->sum_cycles += (uint64_t)cycles;

    budget = s_budgetCycles[(uint32_t)probe];
    if ((budget != 0U) && (cycles >= budget)) {
        stat->overrun_count++;
    }
}

void FOC_Profiler_Init(void)
{
    uint32_t i;
    uint32_t min_overhead = UINT32_MAX;

    s_enabled = 0U;
    s_cpuHz = SystemCoreClock;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = FOC_DWT_UNLOCK_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        FOC_Profiler_ResetStats();
        return;
    }

    s_enabled = 1U;
    for (i = 0U; i < FOC_PROFILER_OVERHEAD_SAMPLES; ++i) {
        uint32_t start = DWT->CYCCNT;
        uint32_t elapsed = DWT->CYCCNT - start;
        if (elapsed < min_overhead) {
            min_overhead = elapsed;
        }
    }
    s_probeOverheadCycles = (min_overhead == UINT32_MAX) ? 0U : min_overhead;

    (void)memset(s_budgetCycles, 0, sizeof(s_budgetCycles));
    s_budgetCycles[FOC_PROBE_FOC_RUN] = FOC_Profiler_UsToCycles(FOC_PROFILER_FOC_BUDGET_US);
    s_budgetCycles[FOC_PROBE_CURRENT_PATH] = FOC_Profiler_UsToCycles(FOC_PROFILER_TIM1_BUDGET_US);
    s_budgetCycles[FOC_PROBE_SPEED_LOOP] = FOC_Profiler_UsToCycles(FOC_PROFILER_TIM1_BUDGET_US);
    s_budgetCycles[FOC_PROBE_POSITION_LOOP] = FOC_Profiler_UsToCycles(FOC_PROFILER_TIM1_BUDGET_US);
    s_budgetCycles[FOC_PROBE_TIM1_ISR] = FOC_Profiler_UsToCycles(FOC_PROFILER_TIM1_BUDGET_US);
    FOC_Profiler_ResetStats();
}

void FOC_Profiler_Clear(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    FOC_Profiler_ResetStats();
    __DMB();
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

void FOC_Profiler_RecordCycles(FOC_ProfilerProbe_t probe, uint32_t raw_cycles)
{
    uint32_t adjusted = (raw_cycles > s_probeOverheadCycles) ?
                        (raw_cycles - s_probeOverheadCycles) : 0U;
    FOC_Profiler_RecordAdjusted(probe, adjusted);
}

void FOC_Profiler_RecordIrqEntry(uint32_t entry_cycle)
{
    uint32_t previous = s_lastIrqEntryCycle;

    s_lastIrqEntryCycle = entry_cycle;
    if (previous != 0U) {
        FOC_Profiler_RecordAdjusted(FOC_PROBE_IRQ_PERIOD, entry_cycle - previous);
    }
}

void FOC_Profiler_GetSnapshot(FOC_ProfilerSnapshot_t *snapshot)
{
    uint32_t primask;
    uint32_t i;

    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->enabled = s_enabled;
    snapshot->cpu_hz = s_cpuHz;
    snapshot->probe_overhead_cycles = s_probeOverheadCycles;
    for (i = 0U; i < (uint32_t)FOC_PROBE_COUNT; ++i) {
        snapshot->budget_cycles[i] = s_budgetCycles[i];
        snapshot->stats[i].count = s_stats[i].count;
        snapshot->stats[i].min_cycles = s_stats[i].min_cycles;
        snapshot->stats[i].max_cycles = s_stats[i].max_cycles;
        snapshot->stats[i].sum_cycles = s_stats[i].sum_cycles;
        snapshot->stats[i].overrun_count = s_stats[i].overrun_count;
    }
    __DMB();
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

const char *FOC_Profiler_ProbeName(FOC_ProfilerProbe_t probe)
{
    static const char *const names[FOC_PROBE_COUNT] = {
        "FOC_RUN",
        "CURRENT_PATH",
        "SPEED_LOOP",
        "POSITION_LOOP",
        "TIM1_ISR",
        "IRQ_PERIOD",
        "MAIN_LOOP",
        "CMD_PREF",
        "POSDBG",
#if LOOP_PROF_EN
        "SEG_CMD",
        "SEG_PREF",
        "SEG_PDB",
        "SEG_NFRAME",
        "SEG_OTHER",
#endif
    };

    if ((uint32_t)probe >= (uint32_t)FOC_PROBE_COUNT) {
        return "UNKNOWN";
    }
    return names[(uint32_t)probe];
}

#if LOOP_PROF_EN
/* ── 主循环分段探针: ISR 税校正 ──
 * 主循环是最低优先级, 每段起止间必被 TIM1_UP (prio=1, 50µs 周期, ~10µs 每次)
 * 周期性抢占。段净耗时 = CYCCNT 差 − ΔISR_ADD(段起止间 TIM1_ISR 计费累计)。
 * ISR 账本: TIM1_UP handler 末尾把本次 ISR 墙钟耗时累入 64 位 s_tim1IsrAccum。
 * 分段句柄: SegBegin 在禁中断下快照 (wall, isr) 对; SegEnd 同样在禁中断下取
 * (wall_now, isr_now) 对, net = Δwall − Δisr。句柄独立可嵌套 (SEG_CMD ⊃ SEG_PREF/SEG_PDB)。
 */
void FOC_Profiler_RecordTim1IsrCycles(uint32_t isr_cycles)
{
    s_tim1IsrAccum += (uint64_t)isr_cycles;
}

void FOC_Profiler_SegBegin(FOC_LoopSegHandle_t *h)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    h->wall = DWT->CYCCNT;
    h->isr = s_tim1IsrAccum;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

uint32_t FOC_Profiler_SegEnd(FOC_LoopSegHandle_t *h, FOC_ProfilerProbe_t probe)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t wall_now;
    uint64_t isr_now;
    uint64_t isr_delta;
    uint32_t seg_isr;
    uint32_t seg_wall;
    uint32_t net;

    __disable_irq();
    wall_now = DWT->CYCCNT;
    isr_now = s_tim1IsrAccum;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }

    seg_wall = (uint32_t)(wall_now - h->wall);
    isr_delta = isr_now - h->isr;
    seg_isr = (isr_delta > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)isr_delta;
    net = (seg_wall >= seg_isr) ? (seg_wall - seg_isr) : 0U;
    FOC_Profiler_RecordCycles(probe, net);
    return net;
}

void FOC_Profiler_LoopBegin(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t wall_now;
    uint64_t isr_now;

    __disable_irq();
    wall_now = DWT->CYCCNT;
    isr_now = s_tim1IsrAccum;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }

    /* 迭代间隔统计: 与上一迭代 LoopBegin 的 (wall, isr) 端点差 */
    if (s_loopPrevWall != 0U) {
        uint32_t iter = wall_now - s_loopPrevWall;
        uint32_t isr_d = (uint32_t)(((isr_now - s_loopPrevIsr) > (uint64_t)UINT32_MAX) ?
                                    UINT32_MAX : (isr_now - s_loopPrevIsr));
        s_iterCount++;
        s_iterSumCyc += (uint64_t)iter;
        if (s_iterMinCyc == 0U || iter < s_iterMinCyc) {
            s_iterMinCyc = iter;
        }
        if (iter > s_iterMaxCyc) {
            s_iterMaxCyc = iter;
        }
        s_isrSumCyc += (uint64_t)isr_d;
        if (isr_d > s_isrMaxCyc) {
            s_isrMaxCyc = isr_d;
        }
    }
    s_loopPrevWall = wall_now;
    s_loopPrevIsr = isr_now;
}

void FOC_Profiler_LoopEnd(void)
{
    /* 迭代记账已在 LoopBegin 完成 (端点是上次 LoopBegin) */
}

FOC_LoopProfIter_t FOC_Profiler_LoopIterStats(void)
{
    FOC_LoopProfIter_t r;
    r.iter_count = s_iterCount;
    r.iter_sum_cyc = s_iterSumCyc;
    r.iter_min_cyc = s_iterMinCyc;
    r.iter_max_cyc = s_iterMaxCyc;
    r.isr_sum_cyc = s_isrSumCyc;
    r.isr_max_cyc = s_isrMaxCyc;
    return r;
}
#endif
