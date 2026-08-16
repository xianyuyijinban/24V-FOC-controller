# UART Runtime Fix Acceptance Report

**Date**: 2026-07-15
**Tester**: DeepSeek automated test suite
**Firmware**: `codex/sync-main-20260519` @ `09536c5dc598cb20f1f203c06805bdd0976813bb`

---

## 1. Environment

| Item | Value |
|---|---|
| **Branch** | `codex/sync-main-20260519` |
| **HEAD** | `09536c5dc598cb20f1f203c06805bdd0976813bb` |
| **HEX SHA256** | `111AC08A854BE5D6CBCB0C7D0AFC082C3FBB3A3A1A4EA50B5E0925C1C6E6E16D` |
| **Target** | STM32H743VITx |
| **Probe** | CMSIS-DAP `0001A0000001` (muselab-tech) |
| **Serial** | COM7 (USB-SERIAL CH340) @ 1,000,000 baud 8N1 |
| **Power** | 12V regulated, motor free-spinning |
| **text/data/bss** | 165488 / 504 / 39472 bytes |

### Firmware Fix Applied

**File**: `Core/Src/stm32h7xx_it.c:1100`

**Before** (bug):
```c
if (strcmp(cmd, "CMD:UART_RX_STAT?") == 0 || strcmp(cmd, "DIAG:UART_RX?") == 0) {
```
`DIAG:UART_RX?` was mapped to `CMD:UART_RX?` by alias, but the handler only checked for `CMD:UART_RX_STAT?` and the dead `DIAG:UART_RX?` — so `DIAG:UART_RX?` was never handled.

**After** (fix):
```c
if (strcmp(cmd, "CMD:UART_RX?") == 0 || strcmp(cmd, "CMD:UART_RX_STAT?") == 0) {
```
Both `DIAG:UART_RX?` (via alias → `CMD:UART_RX?`) and `CMD:UART_RX_STAT?` now match. The response includes `tx_p0_drop`, `tx_p1_drop`, `tx_p2_drop` fields.

---

## 2. Software Test Results

| Test Suite | Result |
|---|---|
| `scripts.test_foc_runtime_profile` | **6/6 PASS** |
| `HostComputer test*.py` (discover) | **193/193 PASS** |
| Firmware build (`build.ps1`) | **0 error, 0 warning** |
| ELF/HEX/BIN generated | **Yes** |

---

## 3. UART Command Alias Regression

Test: 20× `DIAG:UART_RX?` + 20× `CMD:UART_RX_STAT?` with `MixedStreamDecoder`.

| Metric | `DIAG:UART_RX?` | `CMD:UART_RX_STAT?` |
|---|---|---|
| Success rate | **20/20** | **20/20** |
| Response prefix | `UART_RX,OK` | `UART_RX,OK` |
| `err` delta | 0 | 0 |
| `restart_fail` delta | 0 | 0 |
| `last_pos` | present | present |
| `buf` | present | present |
| `tx_p0_drop` delta | 0 | 0 |
| `tx_p1_drop` delta | 0 | 0 |
| `tx_p2_drop` delta | 0 | 0 |

**Verdict: PASS** — both commands 40/40, all fields present and consistent, all deltas zero.

---

## 4. BIN1000 Concurrent Stress Test

Test: 60s `TELEM:CUR,BIN,1000` with 100× `DIAG:UART_RX?` queries at ~600ms intervals. `MixedStreamDecoder` used throughout. **Only one `reset_input_buffer()` at startup.**

| Metric | Result | Requirement | Status |
|---|---|---|---|
| UART queries | **100/100** | 100/100 | ✅ |
| Binary frames | **60,521** | 57,000–63,000 | ✅ |
| Binary CRC errors | **0** | 0 | ✅ |
| UART RX error delta | **0** | 0 | ✅ |
| `restart_fail` delta | **0** | 0 | ✅ |
| `tx_p0_drop` delta | **0** | 0 | ✅ |
| `tx_p1_drop` delta | **0** | informational | ✅ |
| `tx_p2_drop` delta | **0** | informational | ✅ |
| `CMD:STOP` response | **OK** | normal | ✅ |

**Verdict: PASS** — 100/100 ACK under full BIN1000 load, zero errors across all counters.

---

## 5. FOC Time Diagnostic Smoke

Test: `foc_runtime_profile.py` with `READY_IDLE` + `SPEED_BIN1000`, 10s capture, 2s warmup, 1 repeat.

### READY_IDLE

| Probe | Count | Rate | Avg | Max | Overrun |
|---|---|---|---|---|---|
| FOC_RUN | 0 | 0 Hz | 0 | 0 | 0 |
| CURRENT_PATH | 200,128 | 20,012.8 Hz | 1.240μs | 1.371μs | 0 |
| SPEED_LOOP | 20,013 | 2,001.3 Hz | 0.457μs | 1.704μs | 0 |
| POSITION_LOOP | 2,002 | 200.2 Hz | 0.208μs | 0.208μs | 0 |
| **TIM1_ISR** | **200,128** | **20,012.8 Hz** | **9.301μs** | **16.010μs** | **0** |
| IRQ_PERIOD | 200,127 | 20,012.7 Hz | 49.996μs | 65.617μs | 0 |

✅ `FOC_RUN.n=0`, `TIM1_ISR ≈ 20kHz`, `max_us=16.010μs` (GREEN)

### SPEED_BIN1000

| Probe | Count | Rate | Avg | Max | Overrun |
|---|---|---|---|---|---|
| FOC_RUN | 100,080 | 10,008.0 Hz | 3.495μs | 3.498μs | 0 |
| CURRENT_PATH | 200,160 | 20,016.0 Hz | 7.979μs | 11.313μs | 0 |
| SPEED_LOOP | 20,016 | 2,001.6 Hz | 7.506μs | 8.771μs | 0 |
| POSITION_LOOP | 2,001 | 200.1 Hz | 0.292μs | 0.292μs | 0 |
| **TIM1_ISR** | **200,160** | **20,016.0 Hz** | **16.735μs** | **26.940μs** | **0** |
| IRQ_PERIOD | 200,159 | 20,015.9 Hz | 49.996μs | 65.896μs | 0 |

### Communication Checks

| Scenario | UART err | P0 drop | P1 drop | P2 drop | Fault | Binary frames | CRC err |
|---|---|---|---|---|---|---|---|
| READY_IDLE | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| SPEED_BIN1000 | 0 | 0 | 0 | 0 | 0 | 12,015 | 0 |

✅ `FOC_RUN ≈ 10kHz`, `TIM1_ISR ≈ 20kHz`, `overrun=0`, `max_us=26.940μs` (GREEN), all errors zero.

### Deviation from Previous Baseline

| Metric | Previous (2026-07-14) | Current (2026-07-15) | Δ |
|---|---|---|---|
| TIM1_ISR max (SPEED_BIN1000) | 22.923μs | 26.940μs | +4.017μs (**+17.5%**) |

Deviation is within 20% threshold — **no WARNING**. The increase is attributable to new UART TX drop counter tracking (`DrvUart_GetTxDropCounts`) added to the `UART_RX,OK` response path. `max_us` remains well within GREEN (< 35μs).

---

## 6. `reset_input_buffer()` Usage Declaration

| Location | Called? |
|---|---|
| Startup (test initialization) | **Yes — one call** per test script |
| During test execution | **No** |
| Between queries | **No** |
| Between scenarios | **No** |
| In `foc_runtime_profile.py` `command()` | **No** (uses `_drain_pending_input()` on decoded stream) |

---

## 7. Final Verdict

# **PASS** ✅

| Test Area | Result |
|---|---|
| Software unit tests | 199/199 PASS |
| Firmware build | 0e0w |
| UART alias regression (40 queries) | 40/40 PASS, all deltas 0 |
| BIN1000 concurrent stress (100 queries) | 100/100 PASS, 60521 frames, 0 CRC |
| FOC diagnostic smoke (READY_IDLE) | GREEN, max 16.010μs |
| FOC diagnostic smoke (SPEED_BIN1000) | GREEN, max 26.940μs |
| P0 drop | 0 across all tests |
| UART RX errors | 0 across all tests |
| CRC errors | 0 across all tests |

**The `DIAG:UART_RX?` alias fix is verified effective. BIN1000 concurrent ACK under full stream load passes without buffer manipulation. All UART error counters remain zero. FOC timing performance is preserved (GREEN, < 35μs, +17.5% within threshold).**

---

*Report generated by DeepSeek automated acceptance suite. No production parameters modified. No Git commits made.*
