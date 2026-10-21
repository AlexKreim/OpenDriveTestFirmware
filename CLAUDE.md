# CLAUDE.md – project context

Custom BLDC ESC: firmware for an STM32G474 board (this repo root is the STM32CubeIDE project: `.ioc`, `Core/`, `Drivers/`, ...)
plus the Python/PySide6 PC program "ESC Control" in `GUI/`. The user guide (setup, commands, parameters, troubleshooting)
is `README.md` (same file in `GUI/`). Keep both copies identical when you edit it.

---------------------------------------------------------------------------------------------------------------------------

## 1. Working rules (from the owner)

- **This repo is the last version that works on hardware.** Make small, separate changes; say exactly what each one touches,
  and keep the fallback/previous behaviour where possible so a failed change is easy to undo. Commit per change.
- **You cannot test on hardware; the owner can.** Always state what was only compiled, simulated, or not checked at all.
  Never claim a hardware result.
- The owner has a **limited token budget**: short answers, no long test runs or broad refactors unless asked.
- The owner holds an EE degree: technical explanations are welcome, but keep them concise.
- Firmware defaults stay the **ODrive M8325s values** (speed Kp 0.04 A/rpm, Ki 0.2 A/(rpm s)) unless told otherwise.
  Motor-specific values live in GUI presets.
- "Write to flash" (`save`) only works while the motor is stopped. Keep it that way.
- Deliver code changes as edits in this repo (no zips). Update README and this file when behaviour changes.

## 2. History and what to be careful with

- **Diagnoses made before 2026-10-20 may be confounded.** For a long time a motor ("BLDC Schwarz") with a **shorted phase /
  burnt winding** was connected. Shorted turns give low/unequal inductance, unbalanced phases, current spikes and torque
  ripple. Several conclusions from that period (current-sample spikes, the "hardware ADC trigger does not work" finding,
  the low inductance measured by the calibration) may have been caused by the motor, not the firmware.
  Re-check on the healthy DJI 2212 motor before building on them.
- **Gain auto-tune re-implemented 2026-10-20 (untested on hardware):** `tune [bw]`, `foc_tune_*` in `esc_foc.c`, isolated:
  a torque-mode FOC run whose torque command comes from `foc_tune_step` (one-shot `tune_arm` -> `tune_request` in `esc_motor.c`,
  cleared on start/stop/fault/finish). No parameter struct change; writes only spdkp/spdki/poskp in RAM.
- **Reverted, do not reintroduce unless asked:** the first gain auto-tune (`autotune`), adjustable speed-observer bandwidth parameter
  (`obsbw`), automatic current re-zero during precharge, parameter version 6, sending the torque set point at start.
  After these, open loop and low-speed FOC misbehaved on hardware; the cause was never isolated.
- **Change on 2026-10-20 (untested on hardware):** current sampling moved to the **hardware ADC trigger TIM1 TRGO2 = OC4REF**,
  see section 4. Ask the owner for `status` ("current trigger: ...") and `diag` output after flashing.

## 3. Hardware

- MCU STM32G474RET, 170 MHz (HSE 8 MHz, PLL M2 N85 R2), USB FS CDC (virtual COM port), SWD.
- Gate drivers 3x L6387E, TIM1 complementary outputs, dead time 500 ns. FETs: high side BSC030N08NS5, low side
  BSC123N08NS3G (weaker, runs warm). Bus capacitance only 47 uF. No hardware overcurrent break.
- Current sensing: low-side shunts 1 mOhm per phase, internal op-amps OPAMP1/2/3 in PGA x16 (CubeMX), outputs PA2/PA6/PB1
  -> ADC1_IN3 / ADC2_IN3 / ADC3_IN1. Scaling `CS_V_PER_A = 0.852 * 16 * 0.001`, `CS_SIGN -1`, about 59 mA per ADC count.
  Idle reading about 1350 counts.
- Bus voltage PA0 (divider 23), CC6905 bus current (PC2/PB12), phase voltage dividers PA4/PC4/PB11 (PB10 low = on),
  driver NTC PC1 (10k, beta 3380), motor NTC PC0 (often not fitted, reads -999).
- AS5047P: SPI3 PC10/11/12, nCS PB9, 14-bit angle (used by FOC, read inside the control interrupt as one pipelined frame).
  ABI on TIM3 (PC6/PC7, index PD2) read 0 counts so far; diagnostics and the open-loop speed readout only.
  On the current board the AS5047P reports MAGL=1, AGC 255 (field too weak: magnet too far / weak), 2026-10-20.
- LED RGB on PC13/14/15 (active low). Full pin map: header comment of `Core/Src/esc_hw.c`.

## 4. Firmware

Files (all `Core/Src/esc_*.c`, `Core/Inc/esc_*.h`; CubeMX code only calls `esc_early_init`, `esc_app_init`, `esc_app_loop`
in `main.c` and `esc_usb_rx` in `usbd_cdc_if.c`):

| File | Content |
|---|---|
| `esc_config.h` | Clock, PWM (20 kHz, center-aligned, ARR 4250), dead time, scaling, limits, all `DEF_*` defaults, `FOC_DMAX` 0.90 |
| `esc_hw.c` | Register-level setup: TIM1, ADC1/2/3 (injected = phase currents, regular = slow channels), op-amps, SPI3, TIM3, GPIO, AS5047P read/pipeline/reset |
| `esc_motor.c` | 20 kHz control interrupt, state machine (IDLE, PRECHARGE, ALIGN, RUN, FAULT, CALIB), open loop (V/f, sine or 6-step), current conversion, overcurrent (4 samples > `ilim`), FOC glue, FOC log for `focdump`, scope buffer (1000 x 7 floats at 2 kHz / div, filled in `foc_tick` only when armed) |
| `esc_foc.c` | Hardware-independent FOC (also compiled on the PC): Clarke/Park, dq PI with decoupling and anti-windup, SVPWM (min/max injection), encoder tracking (jump gating + relock), 2 kHz outer loops (tracking observer 80 Hz, speed PI, position profile + PID, overspeed), motor calibration state machine |
| `esc_app.c` | USB text console, commands, telemetry, temperatures/overtemp, host watchdog, `status`/`diag`/`test`, boot sequence (ADC trigger selection, offsets, flash load) |
| `esc_faultlog.c` | Fault log: last 8 faults / watchdog stops / abnormal resets, RAM + own flash page 0x0807E000, written by the main loop only while stopped; `faultlog [clear]` |
| `esc_params.c`, `esc_flash.c` | Parameter table (name, type, range), defaults, flash image in the last 2 KB (0x0807F800), CRC, version 5 |

Key mechanisms:
- **Current sampling.** ADC1/2/3 injected, one channel each, 12.5-cycle sampling, triggered at OC4REF rising edge
  (CCR4 = ARR - 100, PWM mode 2, counting up, about 0.6 us before the peak, all low sides on because duty <= 0.9).
  Boot (`esc_app_init`) tries JEXTSEL **8 = TIM1_TRGO2** (TIM1_CR2.MMS2 = 0111, OC4REF), then **1 = TIM1_CC4**; a source is
  accepted only if 1024 samples arrive in about 51 ms and >= 95 % of the ADC interrupts start within 600 counts of the peak
  (`motor_trigger_timing_ok`). Otherwise fallback: TIM1 CC4 interrupt starts the ADCs by software (`TIM1_CC_IRQHandler`;
  center-aligned mode 1 sets CC4IF only while counting down, so it runs once per period; never filter on DIR).
  The JEXTSEL numbers come from RM0440 (injected trigger tables, same for ADC1/2 and ADC3/4/5); confirm there if in doubt.
- Software-fallback protections (keep): late-sample guard (`SAMPLE_MARGIN_CNT`), sum-of-currents spike rejection,
  control interrupt priority 0 and USB 3 (set at every start).
- **FOC conventions:** alpha axis = phase A, amplitude-invariant Clarke, d axis = magnet axis. Position/speed in the SENSOR
  frame; `enc_dir` (+-1, 0 = uncalibrated) maps sensor to electrical angle: theta_e = enc_dir * ((enc * pp) & 0x3FFF) + offset.
  Positive torque turns towards rising sensor angle. Encoder frame is one period old: Park uses theta + w*Ts, inverse Park
  theta + 2*w*Ts. Alignment (d current `ialign`) at every FOC start.
- Outer loops use the measured time between ticks (`foc_in_t.dt` from the DWT cycle counter), so skipped periods do not
  distort the speed.
- Current loop gains auto: Kp = 2 pi `curbw` Ls, Ki = 2 pi `curbw` Rs (Ls = `ll`/2, Rs = `rl`/2).
- Flash wear guard (`flash_guard` in `esc_flash.c`): every page erase is rate limited (params / CAN config 0.5 s, 100 / 50
  per boot; fault log 10 s, 50 per boot) -> `FLASH_RES_LIMIT` / `ERR,flash_limit`. New flash writers must call it.
- Main-loop IWDG (2 s, `wdg_start`/`wdg_kick` in `esc_app.c`), started at the end of `esc_app_init`, kicked each loop pass and in
  the focdump/faultlog loops. Any new blocking code longer than ~1 s must kick it.
- USB send stuck (`usb_tx_watch` in `esc_app.c`): CDC TxState busy 500 ms while running -> stop; 3 s while stopped -> USBD_Stop/Start.
- Host watchdog: GUI sends `ping` every 200 ms and `wd 3000`; uses a signed time difference (an unsigned one caused random
  stops). `status` shows last stop reason, last MCU reset cause (RCC CSR flags), uptime, current-trigger mode, rejected/late
  samples, longest control-interrupt time (must be well below 50 us).
- Build: **-O2** (at -O0 the control interrupt is too slow), hard-float, linker script FLASH LENGTH 504K (fault log page 0x0807E000, parameters 0x0807F800).
- **Changing `esc_params_t`** needs a `PARAMS_VERSION` bump, the static asserts in `esc_params.c`, the GUI's
  `EXPECTED_PARAMS_VERSION` in `GUI/protocol.py`, and a decision about old flash images (saved settings and
  calibration are lost otherwise). The current loader also accepts version-6 images from the reverted build.

Protocol: newline-terminated text. Replies `OK` / `ERR,<reason>`; `P,<name>,<value>`; `N,<preset>`;
`V,ESC,<fw>,<paramversion>,<count>`; telemetry `T,` with 22 fields (see `GUI/protocol.py`); `CAL,ok,<R_ll mOhm>,<L_ll uH>,<dir>,<pp>`
or `CAL,fail,<reason>`; `TUNE,ok,<J gcm2>,<Tf mNm>,<spdkp>,<spdki>,<poskp>,<bw>` or `TUNE,fail,<reason>`; `focdump` lines `F,<i>,ia,ib,ic,id,iq,vd,vq,theta_e_deg,id_ref,iq_ref,enc_raw,enc_ok`; `scope dump` lines
`S,count,<n>,<us>`, `S,<i>,rpm,rpm_sp,iq,iq_ref,id,vq,pos`, `S,end,<n>`; `faultlog` lines `L,count,<n>,boot,<b>`, `L,<i>,...`; `adchist` lines `H,...`.
2026-10-21: focdump currents cluster on 32-count steps (codes ending in binary 11111, errors up to ~1.9 A per phase while switching);
`adchist` added to confirm; `adcsmp` (RAM only, `hw_adc_set_inj_smp`) changes the sampling time to test it.
Result 10-21: bin 31 is 45-60 % at every sampling time, odd codes other than 15/31 almost never occur, and codes just above a
32-step (e.g. C above 0x51F) never occur, also at idle (`raw` C 1310-1311). So it is static: each SAR bit decision biased low
(missing codes), not switching noise. CALFACT was fine (56-62, `adccal` no change). **Cause found 2026-10-21:** VREF+/VDDA fed
through a 600 Ohm@100 MHz ferrite bead with the 1 uF + 100 nF on the supply side, none at the pins. Owner shorted the bead
(caps now at the pins): bins become smooth, odd codes back, bin 31 ~0-4 %. Hardware rule: caps must sit at the VREF+/VDDA pins.
Everything measured before (offsets, `zero`, calibration R/L, auto-tune, low-speed sticking) must be re-checked after this. `help` printed in several uprintf calls (buffer 255 chars).
Floats are printed with `%ld` on scaled integers (newlib-nano has no `%lld` / float printf).

## 5. GUI (`GUI/`)

- `main.py` (tabs Speed / Torque / Position / Tuning / Motor & Advanced; presets bar; plots; CSV; step-response helper;
  calibration dialog that stores results in the current preset and saves), `link.py` (serial thread, command/ack matching,
  events), `protocol.py` (parameter specs: name, label, type, range, step, decimals, unit, tooltip; telemetry fields),
  `presets.py` + `presets/*.json` (user presets in `%APPDATA%\ESC_Control\presets`), `preset_dialog.py`, `stepmetrics.py`.
- Bottom of the window: log + command line (`on_console_send`, any firmware command, history in an editable combo box).
  Tuning tab also has the auto-tune button and the scope; Motor & Advanced has Status / Fault log / Self test buttons.
- GUI telemetry timeout (`send_ping`): no telemetry 1 s while running -> `stop` and no more pings until telemetry returns.
- `WheelGuard` (app-wide event filter in `main()`): the mouse wheel changes spin/combo/slider/dial only when focused; otherwise
  it scrolls the page (scroll bars are excluded from the guard).
- Crash/hang log `%APPDATA%\ESC_Control\crash.log` (`install_crash_log`): exceptions, hard crashes (faulthandler) and GUI-thread
  hangs > 5 s with all thread stacks. Ask the owner for it after a GUI freeze.
- Torque mode starts at 0 by design (the set point is reset at start/stop).
- Run: `run.bat` (creates `.venv`). Exe: `build_exe.bat` (PyInstaller).

## 6. Tests (PC only, no hardware)

- FOC simulation: `esc_firmware/tests` from the owner's package (`run_foc_sim.sh`, `foc_sim.c`); expects `../Core` relative
  to the tests folder, so place it as `tests/` in the repo root. `bash tests/run_foc_sim.sh` (gcc). Models the ODrive motor;
  checks align, torque, speed, calibration, overspeed, position, auto-tune. `GLITCH=0.05` adds bad encoder frames.
- GUI: `python GUI/tests/test_link.py`, `python GUI/tests/test_gui_smoke.py` (gcc needed; a fake board compiles the
  real `Core/Src/esc_params.c`, found automatically or via `ESC_FW_CORE`). Qt is stubbed. One start/stop test can flake on timing.
- The motor/ADC/interrupt code (`esc_motor.c`, `esc_hw.c`, `esc_app.c`) is not covered by any test; only compile it.

## 7. Motors

| Motor | pp | R line-line | L line-line | Kv | Notes |
|---|---|---|---|---|---|
| DJI 2212 920KV (Phantom 1), current test motor | 7 | ~120-150 mOhm | ~40-54 uH (profiler) | 920 | Speed Kp 0.01 / Ki 0.3 worked; auto-tune J 53 g cm^2; sticks once per mechanical rev at low speed (section 8) |
| ODrive M8325s 100KV | 20 | 48 mOhm | 19.8 uH | 100 | Kt 0.083, defaults tuned for it |
| "BLDC Schwarz" | 7 | - | - | - | **Shorted phase, do not use** |

The calibration's inductance (5 kHz voltage dither) read about 5x below the ST profiler value; it was measured on the damaged
motor, so re-check on the 2212 before trusting or fixing the method.

## 8. Board results 2026-10-20 (healthy DJI 2212, build with auto-tune/fault log/scope)

- `test`: CC6905 and motor NTC not fitted (their checks are meaningless; CC6905 may read "OK" by chance). AS5047P: MAGL=1,
  AGC 255 = magnet field too weak. `raw` idle A/B/C 1338/1341/1310, spread 1-6 counts.
- `status`: current trigger **hardware (TRGO2)** works, 0 rejected/late samples, control interrupt max 18 us.
- ABI works now: SPI-ABI offset constant ~14.6 deg (fine, no index used), traces correlate.
- Calibration (2nd try, 1st failed): R_ll 365.9 mOhm (expected ~120-150, maybe wiring/FETs), L_ll 20.6 uH (profiler not run yet).
- Auto-tune: imax 10 A -> overspeed fault, 1 A -> motor did not move (0.5 A < breakaway), **5 A worked**: J 53.2 g cm^2,
  friction 2.36 mNm, at 5 Hz spdkp 0.00195, spdki 0.01529, poskp 1.309 (stored in preset 'BLDCPhantom').
  Idea not done yet: separate test current `tune [bw] [A]` so imax need not change.
- Scope 700 rpm (`GUI/scope.csv` on the desktop PC, gitignored): Iq ~0.13 A, speed ripple 13 rpm 1x/rev + 9 rpm 2x/rev with little Iq reaction -> mostly
  encoder angle error (~1 deg mech), consistent with the weak magnet. Not the low-speed problem.
- **Low-speed sticking** (scope `GUI/60rpm.csv`, desktop PC only): rotor always sticks at the SAME mechanical angle (~48 deg sensor), Iq ramps to
  ~0.8-0.9 A, breaks free, overshoots to 150-400 rpm, does exactly one revolution, sticks there again. Once per mechanical rev
  -> most likely mechanical (rubbing / bearing / magnet holder) rather than cogging or current offsets. Torque mode 0.3 A starts
  only from 1-2 positions. Next: turn by hand unpowered and feel for a tight spot; check magnet mount.
- **Crashes**: twice the GUI froze (after saving a scope CSV; once at 5000 rpm), the motor kept running, reconnect failed until a
  power cycle. Suspected: USB CDC send stuck while the GUI still pinged. Added (untested): main-loop IWDG, USB-send-stuck stop +
  USB reconnect, GUI telemetry timeout. Ask the owner for fault log, `status` (last stop, last MCU reset) after the next one.
- Fault log showed `L,count,0,boot,1` (never written) even after the overspeed; pending owner test: `set otdrv 20` -> overtemp
  in idle -> Refresh fault log -> header `flash,ok|fail` -> `set otdrv 80`, `clear`.

## 9. Open issues / ideas

1. Crash / USB root cause (see section 8). 2026-10-21 lead: `wrap360`/`wrap180`/`wrap_pi` in `esc_foc.c` were subtract loops;
   `pos_deg = wrap360(pos_cnt * deg)` runs every 20 kHz cycle and pos_cnt grows to 256 turns before it is shifted, so the
   interrupt time grew with every turn (SPI speed in the GUI drifted to 4x while FOC speed was right, then USB was lost).
   Fixed with constant-time wraps (untested on hardware). Check `status` interrupt time after > 1 min of running.
2. Low-speed sticking at one mechanical angle (section 8); the calibration's R and L on the 2212.
3. Weak AS5047P magnet field (MAGL).
4. Owner wants cogging/offset compensation (Iq feed-forward vs angle) soon, not yet. On hold: bus-current/regen limit,
   bring-up wizard, offset sanity check at start, field weakening.
5. Hardware: more bus capacitance, low-side FET heating, eventually a 40 A capable board.
6. Dev PC has no host gcc (FOC sim and GUI tests not run); `pacman -S mingw-w64-ucrt-x86_64-gcc` in MSYS2. GUI venv: `GUI/.venv`.
   Only `GUI/README.md` exists (no root copy).
