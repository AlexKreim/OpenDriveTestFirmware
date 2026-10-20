# Project context for Claude

Custom BLDC ESC: STM32G474RET firmware (STM32CubeIDE project, this repo root = folder with the `.ioc`) and a
Python/PySide6 PC program in `esc_GUI/`. The full user guide is in the README (setup, commands, parameters, troubleshooting).

## Working rules (from the owner)
- The code in this repo is the **last known working version**. Change it in small, separate steps and say what each step touches.
  A previous attempt (auto-tune, adjustable speed filter, automatic re-zero at start, parameter version 6) broke open-loop and
  low-speed FOC on hardware and was reverted completely. Do not reintroduce these without being asked.
- The owner tests on hardware; you cannot. Say clearly what was only compiled / simulated.
- Keep answers short; the owner has a limited token budget. No long test runs unless asked.
- Firmware defaults should stay the ODrive M8325s values (speed Kp 0.04, Ki 0.2) unless told otherwise.
- "Write to flash" only works while the motor is stopped (keep it that way).

## Hardware
- STM32G474 @170 MHz, USB CDC console, 3x L6387E gate drivers, low-side shunts 1 mOhm + internal op-amps (PGA x16),
  AS5047P encoder on SPI3 (FOC uses SPI; the ABI count reads 0, not configured), driver NTC 10k beta 3380, bus cap only 47 uF,
  low-side FETs BSC123N08NS3G run warm. Pin map: `Core/Src/esc_hw.c` header comment. Scaling: `Core/Inc/esc_config.h`.
- The hardware ADC trigger (TIM1 CC4 -> injected) does not start on this board; the firmware falls back to a software start in
  `TIM1_CC_IRQHandler` (center-aligned mode 1: CC4 flag only while counting down -> once per period; never filter on DIR).

## Firmware (Core/Src, Core/Inc, files esc_*)
- `esc_motor.c`: 20 kHz control interrupt, state machine, open loop, current sampling, overcurrent (4 samples), FOC glue,
  late-sample guard (`SAMPLE_MARGIN_CNT`), sum-of-currents spike rejection, interrupt priorities (control 0, USB 3) set at start.
- `esc_foc.c`: hardware-independent FOC: dq PI current loops, tracking observer (80 Hz) for speed, speed PI, position profile + PID,
  calibration (R, L, encoder direction, pole pairs). Outer loops use the measured time between ticks.
- `esc_app.c`: commands, telemetry `T,` (22 fields), watchdog (signed time difference, fixed race), `status` (stop reason,
  reset cause, ISR time, late/rejected samples), SPI speed unwrapped every 2 ms.
- `esc_params.c`: parameter table, flash image version 5 (144-byte struct, static asserts). Changing the struct needs a version
  bump plus GUI `EXPECTED_PARAMS_VERSION` and the test harness.
- Build with **-O2** (at -O0 the control interrupt is too slow). Linker script FLASH length 508K (last 2 KB = parameters).
- PC simulation: `bash tests/run_foc_sim.sh` (gcc, PMSM model of the ODrive motor) if the `tests` folder is present.

## GUI (esc_GUI)
- `main.py` (tabs Speed/Torque/Position/Tuning/Motor & Advanced), `link.py` (serial thread, OK/ERR matching, events),
  `protocol.py` (parameter specs, telemetry fields, `EXPECTED_PARAMS_VERSION = 5`), `presets.py` + `presets/*.json`.
- Tests without Qt or hardware: `python tests/test_link.py`, `python tests/test_gui_smoke.py` (need gcc; the fake board compiles
  the real `esc_params.c`, path configured in `tests/fake_esc.py`). One start/stop test is timing-sensitive and can flake.

## Motors in use
- DJI 2212 920KV (Phantom 1): pp 7, R about 120-150 mOhm line-line, L about 40-54 uH line-line (the calibration's L reads ~5x low:
  set `ll` by hand), speed Kp 0.01 / Ki 0.3 worked well. Low-speed (60 rpm) FOC still sticks/jumps once per electrical turn (open issue).
- ODrive M8325s 100KV: pp 20, 48 mOhm / 19.8 uH line-line, Kt 0.083.
- An older "BLDC Schwarz" motor has shorted turns (burnt winding) - do not use for tuning.

## Lessons from debugging
- +-100 A single-phase "overcurrents" were bad samples (late software ADC start), not real current.
- Speed jitter/offset came from skipped periods with a fixed time step (fixed by measuring dt).
- Random idle stops were the watchdog race (fixed). Use `status` -> last stop / last MCU reset / uptime.
