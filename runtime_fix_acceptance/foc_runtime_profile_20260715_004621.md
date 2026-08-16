# FOC Runtime Profile

- Generated: 2026-07-15T00:46:46
- Overall deadline result: **PASS**
- TIM1 hard budget: 50.000 us

## TIM1 ISR Summary

| Scenario | Repeat | Avg us | Max us | Avg CPU % | Max CPU % | Remaining us | Overrun | Status | Outlier |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| READY_IDLE | 1 | 9.301 | 16.010 | 18.6 | 32.0 | 33.990 | 0 | GREEN |  |
| SPEED_BIN1000 | 1 | 16.735 | 26.940 | 33.5 | 53.9 | 23.060 | 0 | GREEN |  |

## All Probes

| Scenario | Repeat | Probe | Count | Rate Hz | Min us | Avg us | Max us | Budget us | Overrun |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| READY_IDLE | 1 | FOC_RUN | 0 | 0.0 | 0.000 | 0.000 | 0.000 | 100.000 | 0 |
| READY_IDLE | 1 | CURRENT_PATH | 200128 | 20012.8 | 1.167 | 1.240 | 1.371 | 50.000 | 0 |
| READY_IDLE | 1 | SPEED_LOOP | 20013 | 2001.3 | 0.413 | 0.457 | 1.704 | 50.000 | 0 |
| READY_IDLE | 1 | POSITION_LOOP | 2002 | 200.2 | 0.208 | 0.208 | 0.208 | 50.000 | 0 |
| READY_IDLE | 1 | TIM1_ISR | 200128 | 20012.8 | 7.977 | 9.301 | 16.010 | 50.000 | 0 |
| READY_IDLE | 1 | IRQ_PERIOD | 200127 | 20012.7 | 34.362 | 49.996 | 65.617 | 0.000 | 0 |
| SPEED_BIN1000 | 1 | FOC_RUN | 100080 | 10008.0 | 3.485 | 3.495 | 3.498 | 100.000 | 0 |
| SPEED_BIN1000 | 1 | CURRENT_PATH | 200160 | 20016.0 | 4.979 | 7.979 | 11.313 | 50.000 | 0 |
| SPEED_BIN1000 | 1 | SPEED_LOOP | 20016 | 2001.6 | 7.404 | 7.506 | 8.771 | 50.000 | 0 |
| SPEED_BIN1000 | 1 | POSITION_LOOP | 2001 | 200.1 | 0.292 | 0.292 | 0.292 | 50.000 | 0 |
| SPEED_BIN1000 | 1 | TIM1_ISR | 200160 | 20016.0 | 7.631 | 16.735 | 26.940 | 50.000 | 0 |
| SPEED_BIN1000 | 1 | IRQ_PERIOD | 200159 | 20015.9 | 34.108 | 49.996 | 65.896 | 0.000 | 0 |

## Communication And Fault Checks

| Scenario | Repeat | UART errors | P0 drops | P1 drops | P2 drops | Fault code | Binary frames | Binary CRC errors |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| READY_IDLE | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| SPEED_BIN1000 | 1 | 0 | 0 | 0 | 0 | 0 | 12015 | 0 |
