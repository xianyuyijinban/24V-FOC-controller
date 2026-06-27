# 12V FOC Controller Platform — V1 Overview

**Version**: `v1.1.0-12V_STANDARD_PLATFORM_COMPLETE`
**HEAD**: `8dbf1d5`
**Baseline**: `12V_STANDARD`

---

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | STM32H743VIT6 (480MHz Cortex-M7, FPv5-D16) |
| Gate Driver | DRV8350S (SPI, 6x PWM) |
| Encoder | TLE5012B (15-bit SPI) |
| Power | 12V DC (10-16V threshold) |
| PWM | 20kHz, ARR=11999 |
| UART | COM9 @ 230400 baud |

---

## Control Baseline (Golden)

```text
PI_CURRENT  = 0.50 / 0
PI_SPEED    = 0.25 / 0.001 gated  (speed mode only; position mode P-only)
RS_FF       = DQ / 0.20, adaptive OFF
BEMF        = OFF
COG         = 0.25 / +60deg
```

| Parameter | Value |
|-----------|-------|
| Rated speed | ±0.5 rad/s |
| Verified limit | ±1.0 rad/s (no-load steady state) |
| Control floor | ±0.05 rad/s (2-3s settle) |
| Speed tracking | >95% (SREF=±0.5) |
| Zero-return Vq | <30mV typical |

---

## UART Command System

7 command groups. Legacy `CMD:` prefix fully supported.

| Group | Commands |
|-------|----------|
| **SYS** | FW_INFO?, CLEAR_FAULT, CMDS? |
| **CTRL** | UNLOCK, ENABLE, MODE, IREF, SREF, PREF, STOP, APP_MODE |
| **GAIN** | PI_CURRENT, PI_SPEED, PD_POS |
| **MOTION** | MOTION_CFG, MOTION_CFG_RESET |
| **FF** | COG, BEMF, KE_TEMP, RS_MODE, RS_SCALE, RS_ADAPTIVE, RS_SIGN |
| **CAL** | STATUS?, STOP, ALL, HOME, SAVE, ENC/MOTOR/JB/COG (V1 stub) |
| **DIAG** | FAULT_DETAIL, JDIAG, PWM_DIAG, TLE_RAW, BLACKBOX |
| **TELEM** | ON, OFF, RATE, RATE? |
| **JOINT** | LIMIT, LIMIT_OFF |
| **GIMBAL** | RAMP |
| **SPRING** | CFG |
| **DETENT** | CFG |
| **CAN** | NODE, HEARTBEAT? |

Response format: `<CMD>,OK,...` / `<CMD>,FAIL,<reason>`

Failure reasons: `parse | range | state | busy | fault | unsupported`

---

## Telemetry

- Ring buffer + HAL UART IT (non-blocking, no DMA)
- Configurable rate: `TELEM:RATE,0..100` (Hz)
- 3-level priority: P0 (cmd/fault) > P1 (DIAG) > P2 (telemetry)
- NUL bytes: 0% (Phase 2b fix verified)

---

## Application Modes

| Mode | Underlying | Key Behavior |
|------|-----------|--------------|
| **RAW** | user-controlled | Full backward compatibility |
| **JOINT_POS** | POSITION | Soft limits, PREF clipping |
| **GIMBAL_SPEED** | SPEED | Configurable SREF ramp |
| **HOLD** | POSITION | Lock current position |
| **SPRING_DAMPER** | POSITION | Iq=K*pos_err−D*speed |
| **DETENT** | POSITION | Snap to nearest virtual detent |

---

## Calibration

- `CAL:ALL` — full identification with precheck
- `CAL:STOP` — abort, preserve old params
- `CAL:SAVE` — persist to Flash
- Progress: `CAL,STEP,<name>,<percent>,<status>`
- Busy protection: rejects CTRL commands during CAL

---

## Black Box

- 100 samples @ 50Hz = 2 seconds pre-fault history
- 21 fields per sample (Vbus, speed, Id/Iq, Vd/Vq, DRV registers, etc.)
- Auto-freeze on fault (>10 pre-fault samples required)
- `DIAG:BLACKBOX,DUMP` — full CSV export

---

## CAN

- Node ID config (1-64)
- Heartbeat timeout → auto STOP
- Message types: SET_POS, SET_SPEED, GET_STATE, CLEAR_FAULT, STOP
- FDCAN hardware init pending crystal path repair
- Protocol layer ready for integration

---

## Known Limitations

| Item | Severity | Notes |
|------|----------|-------|
| CAL:ENC/MOTOR/JB/COG stub | P2 | Use CAL:ALL for full calibration |
| CAN FDCAN hardware disabled | P2 | Crystal path repair needed |
| 1/50 startup frame truncation | P3 | Initialization window, no functional impact |
| SPRING_DAMPER/DETENT hardware验收 | P2 | Code implemented,手感 test pending |

---

## Safety Boundaries

- MOTION_CFG speed limit: 1.0 rad/s (SREF clamped)
- All APP_MODE forces clamped to ±0.30A
- STOP always available (all modes, during CAL, during telemetry)
- CLEAR_FAULT does not clear black box
- Failed CAL preserves old Flash parameters
- Heartbeat loss → auto STOP (CAN mode)

---

## Build

```bash
powershell -ExecutionPolicy Bypass -File build.ps1
```

Toolchain: `arm-none-eabi-gcc 10.3` (Chocolatey)
Flash: `pyocd flash -t stm32h743vitx build/gcc/24V_FOC_Controller.hex`

---

## Version History

| Version | Tag | Key Milestone |
|---------|-----|---------------|
| v1.1.0 | `v1.1.0-12V_STANDARD_PLATFORM_COMPLETE` | All 6 phases complete |
| v1.0.0 | `v1.0.0-12V_STANDARD_PLATFORM` | Platform V1 freeze |
