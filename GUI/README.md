# ESC Control (Windows GUI)

Controls the custom STM32G474 ESC over the USB virtual COM port.

**Stage 1:** open-loop speed control where you only pick an rpm and everything else is derived, motor presets,
live readout and plots (higher update rate while the motor runs, lower when idle), CSV recording/export, and settings
stored in the board's flash with an explicit *Write* button (only works while the motor is stopped).
**Stage 2 (this version, needs firmware 2.0):** field-oriented torque, speed and position control, tunable PID values, motor calibration,
step-response helper, current magnitude / Id / Iq readouts and the overtemperature settings.

## Run it

1. Install Python 3.10 or newer from python.org (tick "Add python.exe to PATH").
2. Double-click `run.bat`. The first start creates a virtual environment and installs the packages in
   `requirements.txt`; later starts are immediate.
3. Plug in the ESC (powered from its DC input), pick the COM port (STM32 ports are listed first) and press **Connect**.

To make a stand-alone `.exe` run `build_exe.bat`; the result is `dist\ESC_Control\ESC_Control.exe` (copy the whole folder).

## Using it

* **Motor presets**: the top box shows the *current preset on the board* (with "(modified)" if you changed values since).
  *Use this motor* sends the selected preset to the board, makes it the board's current preset and writes it to flash
  (motor must be stopped). *New...* opens a form for another motor: name, pole pairs, Kv, R, L, Kt, limits. Tick
  "phase-to-neutral" if the datasheet gives per-phase values, R and L are then doubled. *Edit...* changes a preset,
  *Delete* removes your own ones. *Save board settings as new preset...* stores whatever the board has now.
  Presets are JSON files in `%APPDATA%\ESC_Control\presets`, bundled ones in the `presets` folder next to the program.
  Names: up to 23 plain ASCII characters, no commas.
* **Speed tab**: move the slider or type a number. With *Automatic parameters* on, the firmware derives the boost voltage,
  align voltage and voltage limit from the motor constants and the live bus voltage. *Drive current* is the peak phase
  current used for holding and as boost: raise it if the rotor slips at low speed, lower it for a gentler start.
  The text below shows what the firmware derives and the highest speed the present bus voltage allows.
* **START** runs precharge, align and ramp to the target speed. **STOP** (or the **Esc** key, or the red button at the top)
  coasts the motor. Changing the target speed while running works live; direction, pole pairs and 6-step mode can only be
  changed while stopped.
* **Write settings to flash**: stores every parameter so they survive a reboot (a `*` marks unsaved changes).
  The board refuses it while the motor runs. *Load saved* re-reads the flash copy, *Factory defaults* loads the built-in
  defaults into RAM (write them to flash to keep them).
* **Motor & Advanced tab**: all remaining parameters (limits, manual boost/modulation/align values, 6-step switch).
  *Current sensors*: **Zero** re-measures the sensor offsets (motor stopped, no current) and stores them; **Raw ADC check**
  prints mean/min/max of the three sense channels to the log (idle should be about 1350 counts).
* **Closed loop (FOC)**: needs the firmware 2.0 and the one-time motor calibration (see below).
  * *Speed tab*: choose *Closed loop (FOC)* to run a speed PI loop instead of open loop.
  * *Torque tab*: set torque in Nm or in A (q-axis current, limited by *FOC current limit*). It always starts at 0. An unloaded
    motor accelerates up to the speed limit.
  * *Position tab*: enter a target angle or turn the dial; the shaft moves by the shortest way with the speed and acceleration
    limits and holds. *Hold current position* sets the target to where the shaft is.
* **Tuning tab**: *Calibrate motor* measures R, L, encoder direction and pole pairs (shaft turns a little, keep it free) and stores
  them in the parameters; write them to flash afterwards. The gain boxes (current loop, speed PI, position PID, profile, current limits)
  change the running controller live. *Step-response helper*: alternates the set point between two values (torque, speed or
  position), plots target / reference / measurement and shows rise time, overshoot and settling time of the last step.
  Tip: tune the speed loop first, then the position loop.
* **Overtemperature shutdown** (Motor & Advanced tab): tick *Trip above* and set a temperature for the driver NTC and/or the motor NTC.
  With a limit enabled, a missing sensor (-999) also trips, so leave the motor one unticked if it is not fitted.
* The calibration result (R, L, pole pairs, encoder direction) is written to the board's flash and into the current preset (your own copy if it was a bundled
  one), so selecting that motor again needs no new calibration. A preset that was never calibrated resets the encoder direction: calibrate once for a new motor.
* **Plots**: window length, pause, update rates, **Record CSV...** (writes every telemetry sample while on),
  **Export buffer...** (the last two minutes).

## Safety features

* The GUI sends a keep-alive every 200 ms and enables a **host watchdog** in the firmware (1 s): if the program crashes or
  the cable is pulled while the motor runs, the board stops the motor by itself.
* Closing the program or pressing Disconnect sends STOP first.
* The firmware limits (overcurrent trip, speed limit, bus voltage window) are always active, the GUI cannot disable them.

## Protocol (for your own tools)

Plain text lines at any baud rate. `set <name> <value>`, `get`, `start`, `stop`, `clear`, `save`, `load`, `factory`,
`stream <hz>`, `wd <ms>`, `ping`, `ver`, `setname <text>`, `zero`, `raw`, `mode 0|1|2|3`, `tq <A>`, `pos <deg>`, `calibrate` (answers `OK`, then a `CAL,...` line when finished). Replies: `OK` or `ERR,<reason>`; `P,<name>,<value>` lines after `get`;
`T,...` telemetry lines (see `protocol.py` for the field list). You can type the same commands in PuTTY.

## Tests (optional)

`tests/` contains a simulated board that runs the firmware's real parameter code (compiled with gcc) plus tests for the
link, parameter ranges, derived values, flash image, watchdog, and a GUI smoke test that runs without Qt.
Needs gcc and the firmware folder `esc_firmware` next to this one: `python tests/test_link.py`, `python tests/test_gui_smoke.py`.
