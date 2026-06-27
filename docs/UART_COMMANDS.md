# UART Command Reference — 12V FOC Controller v1.0.0

**Baseline**: `12V_STANDARD` | **Baud**: 230400 | **Protocol**: `CMD:\r\n` or `<GROUP>:<CMD>\r\n`

All new commands follow the grouped prefix convention. Legacy `CMD:` prefix commands remain fully functional.

## Response Format

```
<CMD>,OK,<params...>
<CMD>,FAIL,<reason>
```

Failure reasons: `parse` | `range` | `state` | `busy` | `fault` | `unsupported`

---

## SYS — System

| Command | Params | Description |
|---------|--------|-------------|
| `SYS:FW_INFO?` | — | Firmware version, baseline, git hash |
| `SYS:CLEAR_FAULT` | — | Clear latched faults, re-check DRV/encoder/Vbus |
| `SYS:CMDS?` | — | List all command groups |

**Legacy**: `CMD:FW_INFO?`, `CMD:CLEAR_FAULT`

---

## CTRL — Control

| Command | Params | Description |
|---------|--------|-------------|
| `CTRL:UNLOCK,N` | `0` or `1` | Unlock/lock power stage |
| `CTRL:ENABLE,N` | `0` or `1` | Enable/disable PWM output |
| `CTRL:MODE,N` | `0`=TORQUE, `1`=SPEED, `2`=POSITION | Set control mode |
| `CTRL:IREF,Id,Iq` | `float,float` A | Set current reference (torque mode) |
| `CTRL:SREF,speed` | `float` rad/s | Set speed reference (clamped to MOTION speed_limit) |
| `CTRL:PREF,pos` | `float` rad | Set position reference (position mode) |
| `CTRL:STOP` | — | Zero speed ref + disable PWM |

**Legacy**: `CMD:UNLOCK,N`, `CMD:ENABLE,N`, `CMD:MODE,N`, `CMD:IREF,Id,Iq`, `CMD:SREF,speed`, `CMD:PREF,pos`

---

## GAIN — Control Gains

| Command | Params | Description |
|---------|--------|-------------|
| `GAIN:PI_CURRENT,Kp,Ki` | `float,float` | Current loop PI (Kp>0, Ki≥0) |
| `GAIN:PI_SPEED,Kp,Ki` | `float,float` | Speed loop PI (Kp>0, Ki≥0) |
| `GAIN:PD_POS,Kp,Kd` | `float,float` | Position loop PD (Kp>0, Kd≥0) |

**Legacy**: `CMD:PI_CURRENT,Kp,Ki`, `CMD:PI_SPEED,Kp,Ki`, `CMD:PD_POS,Kp,Kd`

---

## MOTION — Motion Profile

| Command | Params | Description |
|---------|--------|-------------|
| `MOTION:MOTION_CFG?` | — | Query speed_limit, accel, cruise |
| `MOTION:MOTION_CFG,s,a,c` | `float,float,float` | Set speed_limit(rad/s), accel(rad/s²), cruise(rad/s) |
| `MOTION:MOTION_CFG,RESET` | — | Reset to 12V defaults (1.0 / 2.0 / 0.3) |

**12V defaults**: speed_limit=1.0, accel=2.0, cruise=0.3

**Legacy**: `CMD:MOTION_CFG?`, `CMD:MOTION_CFG,s,a,c`, `CMD:MOTION_CFG_RESET`

---

## FF — Feedforward

| Command | Params | Description |
|---------|--------|-------------|
| `FF:COG?` | — | Query cogging gain & phase |
| `FF:COG,gain,phase_deg` | `float,float` | Set cogging gain(0..1), phase(deg) |
| `FF:COG_PHASE,rad` | `float` | Set cogging phase offset (rad) |
| `FF:BEMF?` | — | Query BEMF state |
| `FF:BEMF,0\|1` | `int` | Disable/enable BEMF feedforward |
| `FF:KE_TEMP,Ke` | `float` | Set BEMF Ke temperature coefficient |
| `FF:RS_MODE?` | — | Query RsFF mode |
| `FF:RS_MODE,0\|1\|2` | `int` | Set RsFF mode (0=OFF, 1=DQ, 2=ABC) |
| `FF:RS_SCALE,0..1` | `float` | Set RsFF gain scale |
| `FF:RS_ADAPTIVE?` | — | Query adaptive RsFF state |
| `FF:RS_ADAPTIVE,0\|1` | `int` | Disable/enable adaptive RsFF |
| `FF:RS_SIGN?` | — | Query RsFF sign protection |
| `FF:RS_SIGN,0\|1` | `int` | Disable/enable RsFF sign protection |

**Legacy**: `CMD:COG_CFG?`, `CMD:COG_CFG,g,p`, `CMD:COG_PHASE,r`, `CMD:BEMF_CFG?`, `CMD:BEMF_CFG,N`, `CMD:KE_TEMP,v`, `CMD:RS_FF_MODE?`, `CMD:RS_FF_MODE,N`, `CMD:RS_FF_SCALE,v`, `CMD:RS_FF_ADAPTIVE?`, `CMD:RS_FF_ADAPTIVE,N`, `CMD:RS_FF_SIGN_PROTECT?`, `CMD:RS_FF_SIGN_PROTECT,N`

---

## CAL — Calibration

| Command | Params | Description |
|---------|--------|-------------|
| `CAL:IDENTIFY,0\|1` | `int` | Start/stop motor parameter identification |
| `CAL:ENCODER_DIR,1\|-1` | `int` | Set encoder direction (PWM must be OFF) |
| `CAL:MOTOR_PN,1..50` | `int` | Set motor pole pairs |
| `CAL:HOME` | — | Set current position as mechanical zero |
| `CAL:CLEAR_HOME` | — | Clear mechanical zero offset |
| `CAL:ADC_ZERO,N` | `int` | Sample N ADC readings, report offsets (PWM OFF) |

**Debug only**: `CMD:ADC_NOISE,N`, `CMD:ADC_PHASE_SCAN,N`, `CMD:ADC_SECTOR_SCAN,N`

**Legacy**: `CMD:IDENTIFY,N`, `CMD:ENCODER_DIR,N`, `CMD:MOTOR_PN,N`, `CMD:HOME`, `CMD:CLEAR_HOME`, `CMD:ADC_ZERO,N`

---

## DIAG — Diagnostics

| Command | Params | Description |
|---------|--------|-------------|
| `DIAG:FAULT_DETAIL` | — | Full fault diagnostic snapshot |
| `DIAG:JDIAG` | — | Inertia identification diagnostic |
| `DIAG:PWM_DIAG` | — | Real-time PWM/timing/current diagnostic |
| `DIAG:TLE_RAW` | — | TLE5012B raw sensor readout |
| `DIAG:TLE_GPIO,0\|1` | `int` | Start/stop TLE5012 GPIO diagnostic (5s) |

**Legacy**: `CMD:FAULT_DETAIL`, `CMD:JDIAG`, `CMD:PWM_DIAG`, `CMD:TLE_RAW`, `CMD:TLE_GPIO_DIAG,N`

---

## 12V Standard Baseline

```text
PI_CURRENT  = 0.50 / 0
PI_SPEED    = 0.25 / 0.001 gated
RS_FF       = DQ / 0.20, adaptive OFF
BEMF        = OFF
COG         = 0.25 / +60deg

MOTION_CFG: speed_limit=1.0, accel=2.0, cruise=0.3
VBUS_LIMIT: 10-18V (12V bench)
```

Verify baseline: `SYS:FW_INFO?` → `baseline=12V_STANDARD`
