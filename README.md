# Custom BLDC ESC – firmware and PC control program

A field-oriented motor controller for a custom STM32G474 board with an AS5047P magnetic encoder, plus a Windows
program (Python / Qt) to set it up, tune it and drive it.

The project comes in two folders, and this README is the same in both:

| Folder | What it is |
|---|---|
| `esc_firmware` | Source files for an STM32CubeIDE project (you generate the CubeMX skeleton and add these files) |
| `esc_gui` | The PC program "ESC Control" (Windows; also runs on Linux/macOS with Python) |

> **Status: experimental.** The control code has been checked in a PC simulation and run on one prototype board with
> small, unloaded motors. Treat every new board and every new motor as a bring-up: current-limited bench supply,
> unloaded motor, low current limits, and a hand near the supply switch.

---

## Contents

1. [What it can do](#1-what-it-can-do)
2. [Hardware](#2-hardware)
3. [Building the firmware](#3-building-the-firmware)
4. [Installing the PC program](#4-installing-the-pc-program)
5. [First power-up](#5-first-power-up-bring-up-checklist)
6. [Setting up a motor](#6-setting-up-a-motor)
7. [Calibration](#7-calibration)
8. [Operating modes](#8-operating-modes)
9. [Tuning by hand and reducing jitter](#9-tuning-by-hand-and-reducing-jitter)
10. [Protection, faults and diagnostics](#10-protection-faults-and-diagnostics)
11. [Troubleshooting](#11-troubleshooting)
12. [Parameter reference](#12-parameter-reference)
13. [Command / protocol reference](#13-command--protocol-reference)
14. [Tests for developers](#14-tests-for-developers)
15. [Known limitations](#15-known-limitations)
16. [CAN bus (ODrive CANSimple)](#16-can-bus-odrive-cansimple)

---

## 1. What it can do

* **Open-loop speed control** (V/f, sinusoidal or 6-step). It needs no encoder; you choose an rpm and the voltages are derived from the motor constants.
* **FOC (field-oriented control)** with the AS5047P encoder:
  * **Torque mode**: q-axis current set point in A or Nm.
  * **Speed mode**: speed PI loop.
  * **Position mode**: position loop with a trapezoidal motion profile (speed and acceleration limits), shortest-way moves, holding.
* **Motor calibration**: measures resistance, inductance, encoder direction and pole pairs.
* **Protections**: software overcurrent, bus over/undervoltage, overspeed, encoder failure, driver/motor overtemperature (NTC) and a host watchdog.
* **PC program**: motor presets, live plots, CSV recording, step-response helper, all parameters with tooltips, and settings saved in the board's flash.

## 2. Hardware

* **MCU**: STM32G474RET (170 MHz Cortex-M4F), USB FS (virtual COM port).
* **Gate drivers**: 3 × L6387E (high/low inputs driven by TIM1 complementary outputs, 500 ns dead time).
* **Current sensing**: low-side shunts (1 mΩ) on every phase, amplified by the internal op-amps (PGA ×16). Sampled when all low sides are on.
* **Bus current**: CC6905 Hall sensor. **Bus voltage**: 22k/1k divider (max 62 V in software).
* **Encoder**: AS5047P on SPI3 (14-bit angle, used by FOC) and ABI to TIM3 (used for the open-loop speed readout).
* **NTCs**: driver NTC (10k, β 3380) and an optional motor NTC.
* **LED**: RGB, active low.

### Pin map

| Function | Pins |
|---|---|
| High-side inputs A/B/C | PA8 / PA9 / PA10 (TIM1_CH1/2/3, AF6) |
| Low-side inputs A/B/C | PB13 / PB14 / PB15 (TIM1_CH1N/2N AF6, CH3N **AF4**) |
| Op-amp 1 (phase A) | PA1 +, PA3 −, out PA2 → ADC1_IN3 |
| Op-amp 2 (phase B) | PA7 +, PA5 −, out PA6 → ADC2_IN3 |
| Op-amp 3 (phase C) | PB0 +, PB2 −, out PB1 → ADC3_IN1 |
| Bus voltage | PA0 (ADC1_IN1) |
| CC6905 Vref / Vout | PB12 (ADC1_IN11) / PC2 (ADC1_IN8) |
| Phase voltage dividers A/B/C | PA4 / PC4 / PB11 (ADC2_IN17/5/14), PB10 = divider enable (low = on) |
| Driver NTC / motor NTC | PC1 (ADC2_IN7) / PC0 (ADC2_IN6) |
| Encoder ABI | PC6 / PC7 (TIM3 CH1/CH2), PD2 index (TIM3_ETR) |
| Encoder SPI | PC10 SCK, PC11 MISO, PC12 MOSI (SPI3, AF6), PB9 nCS |
| LED R/G/B | PC13 / PC14 / PC15 (active low) |

If your board differs, change `esc_hw.c` (pins) and `esc_config.h` (scaling: shunt, gain, dividers, NTCs).

### Hardware notes from the prototype

* The bus capacitance (47 µF) is small. Add bulk capacitance close to the bridge for anything above a few amps.
* The low-side FETs (BSC123N08NS3G) are weaker than the high-side ones and run warm.
* Protection is software only; there is no hardware overcurrent break.

## 3. Building the firmware

### 3.1 Create the CubeMX project

Configure only the items below. Everything else is set up in code.

1. New STM32 project in **STM32CubeIDE**, MCU **STM32G474RETx** (or your package; it needs PD2).
2. **SYS**: Debug = Serial Wire. **RCC**: HSE = Crystal/Ceramic Resonator.
3. **Clock tree**: HSE 8 MHz → PLL (source HSE, /M 2, ×N 85, /R 2) → SYSCLK **170 MHz**, AHB/APB1/APB2 /1. USB clock = HSI48.
   (If the clock is wrong the firmware refuses to run and blinks red fast.)
4. **Connectivity → USB**: Device (FS). **Middleware → USB_DEVICE**: Class = CDC (Virtual Port Com).
5. **Analog → OPAMP1, OPAMP2, OPAMP3**:
   * Mode = PGA, non-inverting input = VINP0 (IO0).
   * Inverting input **not connected**: PA3/PA5/PB2 are plain ground pins on this board.
   * Gain 16, power mode high speed.
   * Internal output to the ADC disabled, so the output stays on the pin (PA2/PA6/PB1).
   * `CS_PGA_GAIN` in `esc_config.h` must match the gain (16).
6. Do **not** enable any other peripheral or pin. Generate code.

### 3.2 Add the source files

Copy `Core/Inc/esc_*.h` and `Core/Src/esc_*.c` into the generated project's `Core/Inc` and `Core/Src`, then refresh the project
(F5). Then add these hooks to the generated files:

**`Core/Src/main.c`**

```c
/* USER CODE BEGIN Includes */
#include "esc_app.h"
/* USER CODE END Includes */

int main(void)
{
  /* USER CODE BEGIN 1 */
  esc_early_init();          /* gate driver inputs low before anything else */
  /* USER CODE END 1 */
  ...
  /* USER CODE BEGIN 2 */
  esc_app_init();
  /* USER CODE END 2 */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    esc_app_loop();
  }
}
```

**`USB_DEVICE/App/usbd_cdc_if.c`**

```c
/* USER CODE BEGIN INCLUDE */
#include "esc_app.h"
/* USER CODE END INCLUDE */
...
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  esc_usb_rx(Buf, *Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
```

### 3.3 Reserve the parameter and fault-log flash pages

The settings live in the last 2 KB of flash (0x0807F800), the fault log at 0x0807E000. In the linker script
`STM32G474RETX_FLASH.ld` set `FLASH ... LENGTH` to **`488K`** (done in this repo; the CAN settings use 0x0807D000, the anti-cogging map 0x0807A000-0x0807CFFF). Re-flashing from CubeIDE keeps the saved settings; a full chip erase removes them.
`HAL_FLASH_MODULE_ENABLED` must be on in `stm32g4xx_hal_conf.h` (it is by default).

### 3.4 Compiler settings (important)

* **Optimization `-O2`**: Project → Properties → C/C++ Build → Settings → Tool Settings → MCU/MPU GCC Compiler → Optimization →
  *Optimize more (-O2)*. Choose *[All configurations]* first. Then Project → Clean and build.
  The Debug default `-O0` makes the 20 kHz control interrupt too slow, which shows as rough running, skipped samples and watchdog stops.
* Hard-float ABI and `-lm` are CubeIDE defaults for the G4; keep them.

### 3.5 Flash it

Build, then flash with the ST-Link (Run or Debug in CubeIDE). At boot the LED lights white briefly, then blinks green slowly.

### 3.6 Values to check in `esc_config.h`

* `CS_SHUNT_OHM`, `CS_PGA_GAIN`, `CS_ATTEN`, `CS_SIGN`: current-sense scaling. Check against a known current.
* `CC6905_V_PER_A`: sensitivity of your CC6905 variant.
* `ENC_CPR`: AS5047P ABI counts per revolution (×4). This is a placeholder; FOC does not use it, it uses SPI.
* NTC constants (`NTC_*_R25`, `NTC_*_BETA`, `NTC_*_RFIX`).
* Defaults (`DEF_*`) are the values used after `factory` or an empty flash.

## 4. Installing the PC program

1. Install **Python 3.10 or newer** from python.org (tick "Add python.exe to PATH").
2. Double-click **`run.bat`** in `esc_gui`. The first start creates a virtual environment and installs `PySide6`, `pyqtgraph`,
   `numpy` and `pyserial`; later starts are immediate. On Linux/macOS: `pip install -r requirements.txt`, then `python main.py`.
3. Optional stand-alone `.exe`: run `build_exe.bat`. The result is in `dist\ESC_Control\` (copy the whole folder).

Plug in the board (powered from its DC input), choose the COM port (STM32 ports are listed first) and press **Connect**.
The firmware and GUI versions must match (parameter version 5). The GUI tells you if they don't.

## 5. First power-up (bring-up checklist)

Use a current-limited bench supply (start at 1–2 A), 12–24 V, motor unloaded.

1. **Connect** the GUI. The status line shows IDLE and the bus voltage.
2. In a terminal or the GUI command box, run **`test`**: it checks the clock, ADCs, op-amps, encoder SPI and NTCs and prints an OK/FAIL list.
3. Run **`zero`** with the motor stopped and no current flowing. It measures the current-sensor offsets and stores them.
4. Press **Raw ADC check** (Motor & Advanced tab) or type `raw`. Each phase should read about 1350 counts at idle, with small spread.
5. Run **`status`**. "current trigger" should show *hardware (TIM1 TRGO2 = OC4REF)*. At boot the firmware tries the hardware
   triggers TIM1 TRGO2, then TIM1 CC4, and accepts one only if the samples arrive once per PWM period right at the PWM peak.
   If neither works it falls back to *software (TIM1 CC4 interrupt)*, which also runs but is sensitive to interrupt latency.
   The control-interrupt time is shown after a run.
6. Select or create a motor preset (section 6) and run it in **open loop** at low speed first (Speed tab, *Open loop*, e.g. 300 rpm).
7. Then calibrate (section 7) and try **torque mode** at a small current (±0.3 A), then speed and position mode.

LED: white = boot, slow green = idle, blue = running, fast red = fault, red/yellow flashing = no ADC triggers at boot,
fast red only = wrong system clock.

## 6. Setting up a motor

Presets hold the motor constants and, after calibration, the measured values. One is bundled (ODrive M8325s 100KV).
Use the top bar:

* **Use this motor**: sends the preset to the board, makes it the current preset and writes it to flash (motor stopped).
* **New... / Edit... / Delete**: manage presets. Your own presets are JSON files in `%APPDATA%\ESC_Control\presets`.
* **Save board settings as new preset...**: stores whatever the board has now.

**Values you need for a new motor**

| Value | Where it comes from |
|---|---|
| Pole pairs | Count the rotor magnets and divide by 2 (12N14P → 7). The calibration confirms it. |
| Kv | Datasheet (rpm per volt). From a back-EMF constant Ke in Vrms/krpm (line-line): Kv ≈ 1000 / (Ke × √2). |
| R line-line | Datasheet ×2 if given phase-to-neutral; the calibration measures it. |
| L line-line | Datasheet ×2 if phase-to-neutral. Use an LCR meter or a datasheet value if the calibration result looks too low (see section 7). |
| Kt | ≈ 8.27 / Kv (Nm/A). Only used for the Nm display. |
| Speed limit, trip current | Conservative bench values first. |

The New/Edit form has a "phase-to-neutral" tick box that doubles R and L for you.

## 7. Calibration

In the **Tuning** tab. It needs a motor that turns freely; it moves the shaft. Do it once per motor and wiring.

**Calibrate motor** holds the rotor, measures **resistance** (two currents, so the dead time cancels) and **inductance** (voltage
dither), then turns two electrical revolutions in each direction to find the **encoder direction** and the **pole pairs**.
It takes about 8 seconds. The results are written to the board's flash and into the current preset, so selecting that motor
later needs no new calibration. FOC modes refuse to start until a motor has been calibrated (`encdir` ≠ 0).

* The calibration current is `ical` (default 2 A). Raise it if the motor sticks, lower it for tiny motors.
* **Inductance**: the dither method measures at 5 kHz and can read low on some motors. If the result is far below the
  datasheet or LCR value, type the correct `ll` by hand. The current loop gain scales with it.
* Calibrate again after changing motor, phase wiring or the encoder magnet.

## 8. Operating modes

| Mode | Tab | What you set | Notes |
|---|---|---|---|
| Open loop | Speed (*Open loop*) | rpm | Precharge → align → ramp. *Automatic parameters* derive boost/align/voltage limit from R, Kv and *Drive current*. |
| Speed (FOC) | Speed (*Closed loop*) | rpm | PI loop on the encoder speed, ramp `accel`. |
| Torque (FOC) | Torque | A or Nm | Always starts at 0: press START, then enter the torque. An unloaded motor accelerates until `rpmmax`, where torque is faded out. |
| Position (FOC) | Position | angle 0–360° or dial | Shortest way, profile limits `posvmax`/`posacc`; *Hold current position* holds where the shaft is. |

**START** runs it. **STOP**, **Esc** or the red button at the top coasts the motor. Every FOC start begins with a short alignment
(`ialign`, `alignms`): keep the shaft free during it. Speed and set points can change live; direction and pole pairs only while stopped.

**Write settings to flash** saves all parameters (only while stopped; a `*` marks unsaved changes). *Load saved* re-reads
flash, *Factory defaults* loads the built-in values into RAM.

The plots show command, SPI speed (main loop, smoothed) and FOC speed (the controller's estimate), currents, voltages and temperatures.
**Record CSV...** logs every telemetry sample; **Export buffer...** saves the last two minutes.

## 9. Tuning by hand and reducing jitter

**Gain auto-tune** (Tuning tab, *Auto-tune speed / position gains*, or `tune [bw_hz]`). Needs the motor calibration. In torque
mode the motor is accelerated with half the FOC current limit (`imax`) to half the speed limit (at most 1500 rpm, and at most a
back-EMF of 40 % of the bus voltage), then braked with the same current down to 10 % of that speed, then switched off. The speed
between 20 % and 80 % is fitted with a straight line in both phases: inertia J = 2·Kt·I / (a1 − a2), friction Tf = −J·(a1 + a2)/2,
with Kt = 8.27 / Kv. For the speed-loop bandwidth `bw` (default 5 Hz, 1–20 Hz, ω = 2π·bw):
`spdkp` = J·ω/Kt (in A/rpm), `spdki` = `spdkp`·ω/4, `poskp` = ω/24 (position bandwidth ω/4). Result line
`TUNE,ok,<J g·cm²>,<Tf mNm>,<spdkp>,<spdki>,<poskp>,<bw>` or `TUNE,fail,<reason>`. The firmware only changes the three gains in
RAM; the GUI stores them in the current preset and writes them to flash. Run it with the real load, free to rotate. Stop, a fault
or the host watchdog end the test like a normal run. *Not tested on hardware yet.*

**Anti-cogging** (Tuning tab, *Measure cogging map*, or `cog run [s_per_turn]`). Needs the calibration; run it after the auto-tune,
unloaded. In position mode the set point creeps one mechanical turn forward and back (default 60 s each way, 10-600 s, plus
10° lead-in/out). The position loop's current reference is averaged in 4096 sensor-angle bins (0.088° each); forward and backward
means are averaged so friction cancels, the mean is removed and the map is lightly smoothed. The map is enabled and written to its
own flash area (0x0807A000), then used as Iq feed-forward in **speed and position mode** (not torque mode), interpolated from the
encoder angle every control cycle. Result `COG,ok,<peak-peak mA>,<rms mA>,<friction mNm>,<flash result>` or `COG,fail,<reason>`
(`not_following`: position error > 20°; `coverage`: < 90 % of the bins seen). `cog` shows the state, `cog on|off` switches it
(RAM), `cog save` stores the map with the on/off state, `cog clear` removes it (RAM, then `cog save`), `cog dump` prints
`G,<bin>,<mA>` (*Save map as CSV...* in the Tuning tab writes it to a file: bin, sensor angle, feed-forward in mA). The map belongs to the motor and its magnet mounting: measure again after changing either. Encoder angle errors
(weak magnet) end up in the map as well. *Not tested on hardware yet.*

The **scope** (Tuning tab, `scope [div]`) records 1000 samples of the FOC loops inside the firmware at 2 kHz / `div`
(0.5 s at div 1, up to 25 s): speed, speed set point, Iq, Iq reference, Id, Vq and position. It starts at the next FOC run after
the alignment, or immediately if FOC is running, and works with the step helper and the auto-tune. The GUI polls `scope state`
and reads the result with `scope dump` (`S,count,<n>,<us per sample>`, `S,<i>,rpm,rpm_sp,iq,iq_ref,id,vq,pos`, `S,end,<n>`; the
lines are sent one per main-loop pass so the protections keep running). *Save CSV...* stores it.

The **step-response helper** (Tuning tab) alternates the set point between two values
and reports rise time, overshoot and settling time.

1. **Current loop**: leave `curauto` on. `curbw` 1000 Hz is a safe default; 500–700 Hz is quieter. A correct `ll` and `rl` matter more than the bandwidth.
2. **Speed loop**:
   * **Jitter / noise** → lower **Speed Kp** in ~30 % steps. Kp turns speed-estimate noise directly into current.
   * **Stutter / stick-slip at low speed** → raise **Speed Ki**. Ki builds torque while the rotor lags behind.
   * Overshoot / oscillation after a step → lower Kp or Ki.
3. **Position loop**: `poskp` (rpm per degree of error) first. `poski`/`poskd` are usually 0 because the speed loop already damps.
   `posvmax`/`posacc` shape the move.

Judge smoothness by ear or by the SPI speed trace; the FOC speed trace shows the raw estimate and looks noisier than the shaft really runs.

Typical values that worked on the prototype:

| Motor | Speed Kp | Speed Ki | Notes |
|---|---|---|---|
| ODrive M8325s 100KV (pp 20) | 0.04 | 0.2 | heavy rotor |
| DJI 2212 920KV (pp 7) | 0.01 | 0.3 | light rotor; these are the firmware defaults |

## 10. Protection, faults and diagnostics

| Fault | Trigger | What to do |
|---|---|---|
| overcurrent | any phase above `ilim` for 4 consecutive samples (200 µs) | Check `focdump` and `status` (last sample above the limit). Spikes on one phase only point at measurement/switching problems. |
| overvoltage / undervoltage | bus outside 9–62 V (undervoltage: 5 samples) | Supply, bus capacitance |
| overtemp | driver NTC above `otdrv` (on by default), motor NTC above `otmot` (off by default); an enabled sensor reading −999 also trips. Checked in every state. | Let it cool, `clear` |
| encoder | 20 consecutive bad SPI frames in FOC | Magnet distance, cable, `clear` (re-initialises SPI) |
| overspeed | FOC speed above 1.5 × `rpmmax` + 50 rpm for 3 ms | Usually a wrong `encdir`/`pp` or a load that drives the motor |

`clear` resets a fault. The GUI shows faults in plain words.

**Fault log** (`faultlog`, panel *Fault log* on the Motor & Advanced tab with Refresh / Clear): the last 8 faults, host-watchdog stops and abnormal MCU
resets (watchdog, software/HardFault, low-power), oldest first, with boot number, uptime, state, mode, bus voltage, phase currents,
|I|, speed, Iq reference and the last sample above the overcurrent level. Lines `L,<i>,<boot>,<uptime ms>,<kind>,<what>,<state>,
<mode>,<vbus>,<ia>,<ib>,<ic>,<imag>,<rpm>,<iq_ref>,<oc_a>`. It is written to its own flash page once the motor is stopped, so it
survives a reset or power cycle (a brown-out itself cannot be logged; see *last MCU reset* in `status`). `faultlog clear` empties it.
The header `L,count,<n>,boot,<b>,flash,<none|ok|fail|pending>` gives the number of entries, the number of this power-on
(entries with the same boot number come from the same power-on) and the result of the last flash write in this power-on.

**Main-loop watchdog** (IWDG, 2 s): if the firmware main loop stops, the MCU resets and the outputs switch off; it shows as *independent watchdog* in `status` and in the fault log. It is frozen while a debugger halts the core.

**USB send stuck**: if the board cannot send for 500 ms while the motor runs (PC program frozen and not reading the port), the motor is stopped (*last stop* and fault log: *USB send stuck*). After 3 s stuck while stopped the board disconnects and reconnects USB, so the COM port comes back without a power cycle.

**Flash wear guard**: a flash page survives about 10 000 erase cycles. Each page may be written at most every 0.5 s (fault log:
every 10 s) and at most 100 times per boot (fault log and CAN settings: 50). Beyond that `save`/`zero` answer `ERR,flash_limit`
and the data stays in RAM until the next reboot; the fault log keeps pending entries in RAM and retries. `status` shows the counts.

**GUI crash log**: exceptions, crashes and freezes of the window longer than 5 s are written with stack traces to `%APPDATA%\ESC_Control\crash.log` (path shown in the log at start). Send it along when the GUI froze.

**Host watchdog**: the GUI pings the board every 200 ms and sets `wd 3000`. If nothing arrives for 3 s the motor is stopped and
`INFO: motor stopped by the host watchdog` is printed. Closing the GUI or disconnecting sends STOP. If the GUI receives no telemetry for 1 s while the motor runs, it sends STOP and stops the pings, so the board's watchdog stops the motor even if that STOP is lost (long blocking commands such as `focdump` while running can trigger this).

**Diagnostics commands**

* `status`: state, fault, bus voltage, currents, mode, |I|, Id, Iq, temperatures, last overcurrent sample,
  **last stop reason**, **last MCU reset cause** (brown-out, watchdog, software reset, reset pin), uptime, current-trigger mode,
  rejected/late current samples, and the longest control-interrupt time (must stay well below 50 µs).
* `focdump`: the last 100 control samples (5 ms) before a FOC fault: ia ib ic id iq vd vq θe id_ref iq_ref enc_raw enc_ok.
* `diag`: op-amp and ADC register summary. `raw`: mean/min/max of the current channels over 0.2 s. `gates`: gate-driver test pattern.
* `enc [ms]`: stream encoder readings.

## 11. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| FOC refuses to start, "needs the encoder calibration" | Run the calibration (Tuning tab). A preset that was never calibrated resets `encdir`. |
| Overcurrent with single-phase spikes of ±100 A in `focdump` | Bad current samples, not real current. Check that the build is `-O2`, check `status` (late/rejected samples), run `zero` again. |
| Rough sound, skipped samples, random watchdog stops | Build with `-O2` (section 3.4). Check the interrupt time in `status`. |
| Speed jitter | Lower Speed Kp in steps, keep Ki high (section 9). |
| Stutters at low rpm | Raise Speed Ki; make sure `ll`, `rl`, Kv are right. |
| Stays at a fixed speed below the target, current at the limit | Gains far too high (oscillation saturates the current), wrong Kv, or the voltage limit is reached. |
| Motor goes back to idle without a fault | `status` → *last stop*: host watchdog (GUI busy / USB) or stop command. If *uptime* is short, the board reset: see *last MCU reset* (brown-out = supply dip → more bus capacitance / supply current). |
| Calibration: `no_current` | Motor wires, supply, current sensors (`raw`). |
| Calibration: `no_motion` / `pole_pairs` | Shaft not free, `ical` too low, magnet not centred, wrong magnet count. |
| Strange behaviour with a motor that once overheated | Shorted turns (low inductance, unbalanced phases, current spikes). Use another motor. |
| GUI fields all show their minimum | The GUI did not get the parameters: reconnect, or type `get`. |
| ABI speed reads 0 in open loop | ABI output of the AS5047P not configured/connected (FOC uses SPI and is unaffected). |

## 12. Parameter reference

Set with `set <name> <value>` or in the GUI; `get [name]` prints them; `save` writes them to flash.

| Name | Meaning | Unit |
|---|---|---|
| pp | Pole pairs (magnets / 2) | |
| kv | Speed constant, rpm per volt of peak line-line back-EMF | rpm/V |
| rl | Line-line resistance (2 × phase) | mΩ |
| ll | Line-line inductance (2 × phase) | µH |
| kt | Torque constant (Nm display only) | Nm/A |
| rpmmax | Speed limit | rpm |
| ilim | Overcurrent trip per phase | A |
| idrive | Open loop: drive current for holding / boost (automatic mode) | A |
| accel | Speed ramp | rpm/s |
| rpm | Target speed | rpm |
| dir | Direction +1 / −1 (stopped only) | |
| six | 6-step commutation in open loop | 0/1 |
| auto | Open loop: derive boost/align/voltage limit automatically | 0/1 |
| boost, maxmod, alignv | Open loop manual values (auto = 0) | V, –, V |
| alignms | Align time (open loop and FOC) | ms |
| curbw | Current loop bandwidth (automatic gains) | Hz |
| curauto | Automatic current gains from R, L, bandwidth | 0/1 |
| curkp, curki | Manual current gains | V/A, V/(A·s) |
| spdkp, spdki | Speed PI (defaults 0.01 / 0.3) | A/rpm, A/(rpm·s) |
| poskp, poski, poskd | Position PID (output: speed set point) | rpm/deg … |
| posvmax, posacc | Position move speed and acceleration | rpm, rpm/s |
| imax | FOC current limit (q axis) | A |
| ialign | FOC alignment current | A |
| ical | Calibration current | A |
| otdrv, otdrven | Driver overtemperature trip and enable | °C, 0/1 |
| otmot, otmoten | Motor overtemperature trip and enable | °C, 0/1 |
| encdir | Encoder direction (0 = not calibrated) | ±1 |

## 13. Command / protocol reference

Plain text lines over the USB virtual COM port (any baud rate, newline-terminated). You can use the **command line at the bottom of the GUI** (any command, replies in the log above it, history in the drop-down / arrow keys, drag the splitter to enlarge the log) or PuTTY / any terminal while the GUI is closed.
Commands answer `OK` or `ERR,<reason>`.

| Command | Purpose |
|---|---|
| `help`, `ver` | Command list; `V,ESC,<fw>,<param version>,<param count>` |
| `status`, `test`, `diag`, `raw`, `gates`, `enc [ms]` | Diagnostics (see section 10) |
| `get [name]`, `set <name> <value>`, `setname <text>` | Parameters (`P,<name>,<value>` lines, `N,<preset name>`) |
| `save`, `load`, `factory` | Flash: write (stopped only), reload, built-in defaults |
| `zero [ram]` | Measure current-sensor offsets (and store them) |
| `mode 0\|1\|2\|3` | Open loop / torque / speed / position |
| `start`, `stop`, `clear` | Run, coast, reset a fault |
| `tq <A>`, `pos <deg>` | Torque set point; position target in absolute multi-turn degrees (360 = one turn, like ODrive, +-36000000 = 100000 turns; float resolution about 0.1 deg at 4000 turns, re-zero with `setpos 0`) |
| `takeover` | The PC takes over a motor that CAN started (keeps the present set points); GUI button *Take over (USB)* |
| `setpos [deg]` | The present position becomes `deg` (like ODrive Set_Absolute_Position, RAM only); without argument: print it |
| `tune [bw_hz]` | Speed/position gain auto-tune (section 9) → `TUNE,ok,...` or `TUNE,fail,<reason>` |
| `cog [run [s]\|on\|off\|save\|clear\|dump]` | Anti-cogging map (section 9) → `COG,ok,<pp mA>,<rms mA>,<friction mNm>,<flash>` or `COG,fail,<reason>` |
| `calibrate` | Motor calibration → `CAL,ok,<R_ll mΩ>,<L_ll µH>,<dir>,<pp>` or `CAL,fail,<reason>` |
| `focdump` | Last 100 FOC samples: `F,<n>,...` |
| `scope [div]`, `scope state\|stop\|dump` | 2 kHz capture of the FOC loops (section 9) |
| `adchist start`, run, `adchist` | Histogram of the lowest 5 bits of the phase-current ADC codes while driven: `H,n,...`, per phase bin 31 / bin 0 / largest bin in %, then 32 counts. Even codes give ~3.1 % per bin |
| `adcsmp [12\|24\|47\|92]` | Test only: phase-current ADC sampling time in cycles (hardware trigger, motor stopped). Not stored, every boot starts with 12.5 |
| `adccal` | Test only: runs the ADC calibration again (motor stopped, not stored); run `zero ram` afterwards. `diag` shows the calibration factors |
| `can`, `canget`, `canset <name> <value>`, `cansave` | CAN settings and state (section 16) |
| `faultlog [clear]` | Fault log (section 10): `L,count,<n>,boot,<b>` then `L,<i>,...` lines |
| `stream <hz>` | Telemetry `T,...` lines (22 fields, see `protocol.py`) |
| `wd <ms>`, `ping` | Host watchdog |
| `led r\|g\|b\|off\|auto`, `bemf 0\|1` | LED test, phase-voltage divider |

## 14. Tests for developers

* **Firmware control code** (`esc_firmware/tests`): `bash tests/run_foc_sim.sh` compiles `esc_foc.c` and `esc_params.c` with gcc
  and runs them against a simulated PMSM (ODrive M8325s): alignment, torque, speed, calibration, overspeed, position and the gain auto-tune.
  `GLITCH=0.05` adds corrupted encoder frames; `hand` simulates turning the shaft by hand; `tune` sweeps position gains.
* **GUI** (`esc_gui/tests`, needs gcc and the `esc_firmware` folder next to `esc_gui`): `python tests/test_link.py` and
  `python tests/test_gui_smoke.py`. They use a simulated board that runs the firmware's real parameter code, and Qt stubs, so no display is needed.

Firmware source overview:

| File | Content |
|---|---|
| `esc_app.c` | USB console, commands, telemetry, temperatures, watchdog, calibration result handling |
| `esc_motor.c` | Control interrupt (20 kHz), state machine, open loop, current sampling, overcurrent, FOC glue |
| `esc_foc.c` | FOC core (hardware independent): current loops, observer, speed/position loops, calibration |
| `esc_hw.c` | Registers: TIM1, ADCs, op-amps, SPI3, TIM3, GPIO |
| `esc_faultlog.c` | Fault log in RAM and its own flash page (0x0807E000) |
| `esc_params.c`, `esc_flash.c` | Parameter table, defaults, flash image (version 5; images saved by the short-lived version 6 are read too) |
| `esc_config.h` | Board constants, scaling, limits, defaults |

## 15. Known limitations

* Current sampling uses the hardware trigger TIM1 TRGO2 (= OC4REF, 100 counts before the PWM peak) on all three injected ADCs.
  Earlier builds used TIM1 CC4 and fell back to a software start on the prototype; that diagnosis was made while a motor with a
  shorted phase was connected, so it may have been wrong. The software fallback is still there and shows in `status` if used.
* The ABI encoder count read 0 on the prototype. The firmware now reads the AS5047P SETTINGS1/2 registers at boot and sets the ABI counts per revolution from them; `test` shows the settings and the TIM3 pins, `enc` prints the measured counts per revolution once the shaft has turned a full revolution, `status` shows the ABI speed next to the SPI speed. ABI stays diagnostics only (open-loop speed readout); FOC uses the SPI angle. If the count and the pins never change while turning, check the A/B/I wiring.
* The inductance measurement of the calibration can read low (see section 7).
* No hardware overcurrent protection; the software trip needs 200 µs.
* Low bus capacitance and warm low-side FETs on the prototype limit the usable current.

## 16. CAN bus (ODrive CANSimple)

**Hardware:** FDCAN2, PB5 = RX, PB6 = TX, TCAN332 transceiver, 120 Ohm termination by solder jumper (close it on the two
nodes at the ends of the bus). Default **250 kbit/s, node ID 0**, like ODrive. Settings: GUI *Motor & Advanced -> CAN bus*,
or `can` (show), `canset <name> <value>`, `cansave`. Names: `enable node kbps timeout loss hb enc iq temp bus torque power err ext`
(rates in ms, 0 = off). They live in their own flash page (0x0807D000), so the parameters and their format are unchanged.
Build without CAN: `ESC_CAN_ENABLE 0` in `esc_config.h`. Last build before CAN: git tag `pre-can`.

**Protocol:** ODrive CANSimple, v0.6 message layout, 11-bit ID = `node_id << 5 | cmd`, little endian, ODrive units
(turns, turns/s, Nm, A, V). Positive = rising encoder angle. Torque in Nm uses Kt = `kt` (or 8.27 / Kv if `kt` is 0).

| cmd | Message | Here |
|---|---|---|
| 0x001 | Heartbeat (TX) | axis error, axis state (1 idle, 3 calibration, 8 closed loop, 9 lockin spin = open loop), procedure result, trajectory done |
| 0x002 | Estop | stops, fault *estop* (like ODrive: Clear_Errors before the next start) |
| 0x003 | Get_Error | active errors (also as disarm reason) |
| 0x006 | Set_Axis_Node_ID | v0.5 style: byte 0 = new node ID, saved (motor stopped) |
| 0x007 | Set_Axis_State | 1 idle (stop), 8 closed loop (start in the Set_Controller_Mode mode), 9 lockin spin (open loop at the Set_Input_Vel speed), 3/4/7 calibration (R, L, encoder, pole pairs; written to flash when done) |
| 0x009 | Get_Encoder_Estimates (TX) | position (turns, multi-turn), velocity (turns/s) |
| 0x00B | Set_Controller_Mode | 1 torque, 2 velocity, 3 position (default 3). Takes effect at the next closed-loop start; input mode is stored only (velocity always ramps with `accel`, position always uses the trapezoid profile) |
| 0x00C | Set_Input_Pos | absolute target (turns); vel/torque feed-forward ignored |
| 0x00D | Set_Input_Vel | turns/s; torque feed-forward ignored |
| 0x00E | Set_Input_Torque | Nm, limited to `imax` |
| 0x00F | Set_Limits | velocity limit -> `rpmmax`, current limit -> `imax` |
| 0x011 / 0x012 | Set_Traj_Vel_Limit / Accel_Limits | -> `posvmax`, `posacc` (deceleration = acceleration) |
| 0x013 | Set_Traj_Inertia | ignored |
| 0x014 / 0x015 / 0x017 / 0x01C / 0x01D | Get_Iq / Temperature / Bus_Voltage_Current / Torques / Powers (TX) | bus current and powers are estimated from Vd·Id + Vq·Iq (no current sensor fitted) |
| 0x016 | Reboot | byte 0: 0 reboot, 1 save configuration (parameters + CAN settings); motor stopped |
| 0x018 | Clear_Errors | clears the fault |
| 0x019 | Set_Absolute_Position | the present position becomes this value (turns) |
| 0x01A / 0x01B | Set_Pos_Gain / Set_Vel_Gains | -> `poskp` (/6), `spdkp`/`spdki` (/(60·Kt)) |
| 0x000 | Get_Version | answered on RTR |

Telemetry is sent cyclically (defaults: heartbeat 100 ms, encoder 10 ms, Iq / temperature / bus / torques 100 ms) and every
Get_ message is also answered to an RTR request (ODrive v0.5 style). **Extra frames** (29-bit, `0x10000000 | node << 8 | index`,
100 ms): 0 phase currents + state + fault, 1 Id/Iq/Vd/Vq, 2 speed and speed command (rpm), 3 Vbus / estimated Ibus /
temperatures, 4 interrupt time, rejected / late samples, control owner, mode. **DBC file** with all messages and value tables:
`can/esc_odrive_node0.dbc`; other node IDs or several nodes in one file: `python can/make_dbc.py 1` / `python can/make_dbc.py 0 1 2 3`.

**USB and CAN together:** telemetry goes out on both at the same time. Control belongs to whoever started the motor
(`status`: *control*, GUI top bar). While CAN controls it, USB set points (`tq`, `pos`, `set rpm`) answer `ERR,not_owner`;
**stop works from both sides always**. *Take over (USB)* (`takeover`) gives the PC control without stopping the motor; CAN set
points are then ignored until the motor stops.

**CAN watchdog:** while CAN controls the motor, every frame addressed to this node resets it. No frame for `timeout`
(default 500 ms, 0 = off) -> *coast* (outputs off) or *brake* (speed / open loop ramp to 0, torque mode brakes with half
`imax`, max 3 s; position mode always coasts), then fault *CAN timeout* (Clear_Errors / `clear` before the next start).
A master must therefore send something (e.g. its set point) at least every `timeout`.

**Differences to a real ODrive (for porting):** control mode changes only at the next closed-loop start; input modes and
feed-forward are not used; no SDO / endpoint access (0x004/0x005), no anticogging, no DFU; position limits / homing not
implemented; one axis per board.

