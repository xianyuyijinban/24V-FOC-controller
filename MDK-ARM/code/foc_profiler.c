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
        "IRQ_PERIOD"
    };

    if ((uint32_t)probe >= (uint32_t)FOC_PROBE_COUNT) {
        return "UNKNOWN";
    }
    return names[(uint32_t)probe];
}
