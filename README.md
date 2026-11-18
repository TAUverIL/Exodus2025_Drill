# Drill Arm — Exodus Rover

Firmware and control stack for a vertical drilling arm: a Teensy 4.1 runs the
real-time motion firmware, and a host computer (Mac for bench work, Jetson
Orin + ROS 2 for rover integration) commands it over a plain-text USB serial
protocol.

```
 Mac GUI (tkinter)                Jetson Orin (ROS 2 Humble)
 live graphs + buttons            bridge node + actions/services/topics
        \                                /
         \______ USB serial, 115200 ____/
                 ASCII commands
                      |
                Teensy 4.1 firmware
                 10 µs stepper ISR
                      |
   ┌──────────┬───────┴──────┬────────────┬─────────────┐
 TB6600 +   Cytron MD10C   HX711 +      ToF sensor    top limit
 NEMA17     + drill motor  2 load cells (I2C)         switch
 (Y axis)   (PWM)          (force)      (ground dist)
```

## Hardware

| Part | Role |
|---|---|
| Teensy 4.1 | Runs all real-time control (no OS) |
| TB6600 + NEMA17 17HS9401S | Y-axis lead screw, 400 steps/mm (4 microsteps, 2 mm/rev) |
| Cytron MD10C r3 + DC motor | Drill spindle, PWM at 10 kHz |
| HX711 + 2 combined load cells | Drilling force measurement |
| TWTOF240UI ToF sensor (I2C) | Distance to ground / contact detection |
| Limit switch (top) | Homing reference, Y = 0 |

## Repository layout

| Path | What it is |
|---|---|
| `drill_arm_teensy_usb_serial/` | **Main firmware** (`.ino`) + Mac GUI (`mac_gui/`) + isolated test sketches |
| `ros2_ws/src/drill_arm_interfaces/` | ROS 2 action definitions (Home, MoveY, DrillToDepth) |
| `ros2_ws/src/drill_arm_ros_bridge/` | ROS 2 bridge node + interactive CLI node |
| `project/` | KiCad schematic / PCB |
| `Load_Cells/`, `Elevator_Debugging/`, `Teensy_Code/` | Earlier standalone sketches |
| `ROS2_OVERVIEW.txt`, `*.pdf` | Design notes and prep documents |

## Firmware highlights

- **10 µs timer ISR** generates stepper pulses independently of the main loop —
  sensor reads and serial parsing can never stall motion.
- **Trapezoidal speed profile** for downward moves (accelerate / cruise /
  decelerate using `v² = v₀² + 2as`); constant-speed upward moves.
- **Position-compensated force readings**: a 13-point baseline sweep maps the
  load cell's mechanical drift along Y, then live readings subtract the
  interpolated baseline.
- **`BOOT` sequence**: home → tare (verified, with residual report) → baseline
  calibration, in one command.
- **Layered safety**: force hard-limit (25 kg), ToF ground-contact and
  clearance checks, limit-switch/ToF conflict detection, command watchdog,
  latched fault codes that require explicit `CLEAR_FAULT`.
- Debounced limit switch (sampled every ISR tick), HX711 reads guarded with
  `noInterrupts()` (bit-bang timing vs. ISR), all ISR-shared state `volatile`.

## Serial protocol (115200 baud, newline-terminated)

| Command | Effect |
|---|---|
| `BOOT` | Home → tare → baseline calibration |
| `HOME` | 3-phase homing (approach / backoff / slow re-approach) |
| `SET_HOME` | Force current position = Y 0 |
| `MOVE Y=120 SPEED=2` | Absolute move (mm, mm/s) |
| `MOVE_REL Y=-5 SPEED=1` | Relative move |
| `DRILL PERCENT=60` | Drill motor power (−100…100, ramped) |
| `STOP` / `ESTOP` | Stop motion / emergency stop + disable stepper |
| `STATUS` | One-line `KEY=VALUE` state dump |
| `TARE` / `CLEAR_FAULT` | Zero load cells / clear latched fault |
| `TUNE` | The stepper sings the Imperial March. Yes really. |

Replies: `ACK …`, `ERR …`, `INFO …`, `STATUS …` — trivially parseable.

## Running it

**Mac GUI** (no ROS needed):
```bash
pip3 install pyserial matplotlib
python3 drill_arm_teensy_usb_serial/mac_gui/drill_arm_gui.py
```
Live position/force/drill graphs, all commands as buttons, fault decoding.

**ROS 2 (Jetson, Humble):**
```bash
cd ~/drill_wa01   # workspace containing ros2_ws/src packages
colcon build --packages-select drill_arm_interfaces drill_arm_ros_bridge
source install/setup.bash
ros2 run drill_arm_ros_bridge drill_arm_bridge    # terminal 1
ros2 run drill_arm_ros_bridge drill_arm_cli       # terminal 2
```
The bridge exposes `/drill_arm/status` (topic), `stop/estop/clear_fault/tare/
boot/set_home` (services) and `home / move_y / drill_to_depth` (actions).

## Known limitations (honest list)

- **Open-loop position**: steps are counted, not measured — a stall silently
  desyncs position. Planned fix: linear encoder on the rail.
- Load cell calibration constant is bench-fitted, not traceably calibrated.
- `COMMAND_TIMEOUT_MS` is set long (5 min) for bench testing; production
  should use ~2 s with a heartbeat from the host.
