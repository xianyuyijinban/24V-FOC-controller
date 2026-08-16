#ifndef FOC_PROFILER_H
#define FOC_PROFILER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include <stdint.h>

#define FOC_PROFILER_TIM1_BUDGET_US 50U
#define FOC_PROFILER_FOC_BUDGET_US  100U

typedef enum {
    FOC_PROBE_FOC_RUN = 0,
    FOC_PROBE_CURRENT_PATH,
    FOC_PROBE_SPEED_LOOP,
    FOC_PROBE_POSITION_LOOP,
    FOC_PROBE_TIM1_ISR,
    FOC_PROBE_IRQ_PERIOD,
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

#ifdef __cplusplus
}
#endif

#endif /* FOC_PROFILER_H */
