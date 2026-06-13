# Hardware Test Scripts

Serial test scripts for COM9 @ 230400. Run from repo root or desktop.

## Quick Diagnostics

| Script | Purpose |
|---|---|
| `phase_b_torque_test.py` | CLEAR_FAULT → UNLOCK → torque ramp (0.05-1.0A), verify angle response |
| `phase_c_home_pref.py` | CMD:HOME → PREF=0/±5°/±20° position closed-loop validation |
| `phase_diag_speed.py` | Speed mode test (±1/±3/±5 rad/s), detect commutation issues |
| `identify_monitor.py` | Start IDENTIFY, monitor progress with state names, capture final params |
| `phase_bcd_full.py` | Post-brake-release B→C→D one-shot (torque check → position → feedforward) |

## Usage

```bash
cd E:\24V_FOC_Controller_sync_20260519
python tests/hardware/phase_b_torque_test.py
```

All scripts assume:
- COM9, 230400 baud
- Firmware already flashed
- 24V power connected
- Motor free to rotate

## Phase Sequence

After firmware flash + power cycle:
1. `identify_monitor.py` — identify motor parameters
2. `phase_b_torque_test.py` — verify torque direction
3. `phase_c_home_pref.py` — position closed-loop validation
4. `phase_bcd_full.py` — full feedforward A/B test (future)
