# Drill Arm Teensy USB Serial Protocol

This sketch keeps the Teensy as firmware, not a ROS node.
The Jetson will run the ROS 2 node and talk to the Teensy over USB Serial.

## USB Serial Settings

```text
Baud: 115200
Line ending: newline
```

On the Jetson, the Teensy will usually appear as:

```text
/dev/ttyACM0
```

## Current Pin Map

```text
Y STEP:       Teensy pin 3
Y DIR:        Teensy pin 4
Y ENABLE:     Teensy pin 2
Top limit:    Teensy pin 5, wired active-low to ground
Drill PWM:    Teensy pin 6
Drill DIR:    Teensy pin 7
HX711 DT:     Teensy pin 8
HX711 SCK:    Teensy pin 27
Load cells:   Two physical load cells combined into one HX711 channel
```

Combined load-cell wiring:

```text
Both green signal wires -> HX711 A+
Both white signal wires -> HX711 A-
Both red excitation wires -> HX711 E+
Both black excitation wires -> HX711 E-
HX711 DT/DOUT -> Teensy pin 8
HX711 SCK/CLK -> Teensy pin 27
```

## Current Mechanical Constants

```text
Motor:          0.9 degree stepper, 400 full steps/rev
Microsteps:     1.0, full-step on A4988 unless MS1/MS2/MS3 are changed
Lead:           2.0 mm/rev
Resolution:     200 steps/mm
Default move:   0.5 mm/s for first powered tests
Homing speed:   0.25 mm/s for first powered tests
Max Y speed:    5 mm/s until lifting is proven stable
Force input:    one HX711 reading two combined physical load cells
Force warn:     20 kg / 20000 g
Force limit:    25 kg / 25000 g
Command timeout: 60 seconds for manual Arduino Serial Monitor testing
```

## Commands To Teensy

```text
HOME
```

Homes the Y axis upward until the top limit switch is pressed. After this, the top position is `Y=0`.

```text
MOVE Y=100 SPEED=5
```

Moves the Y axis to absolute position `100 mm` at `5 mm/s`.
The firmware clamps requested speed to the configured maximum, currently `10 mm/s`.

```text
MOVE_REL Y=-10 SPEED=5
```

Moves the Y axis relative to the current position. Negative moves upward, positive moves downward.

```text
DRILL PERCENT=50
```

Runs the DC drill motor forward at 50 percent PWM.

```text
DRILL PERCENT=-50
```

Runs the DC drill motor reverse at 50 percent PWM.

```text
DRILL PERCENT=0
```

Stops the DC drill motor.

```text
STOP
```

Stops Y motion and the drill motor.

```text
ESTOP
```

Stops everything and disables the stepper driver.

```text
STATUS
```

Prints one machine-readable status line.

```text
POS
```

Prints a shorter position-only line.

```text
CLEAR_FAULT
```

Clears the active fault.

```text
TARE
```

Tares/zeros the connected HX711 load cells if they are ready.

## Replies From Teensy

```text
READY DRILL_ARM_TEENSY_USB_SERIAL
ACK HOME
ACK MOVE
ACK DRILL
ACK STOP
ACK ESTOP
ACK CLEAR_FAULT
ACK TARE
```

Status format:

```text
STATUS Y=0.000 TARGET=0.000 LIMIT_TOP=1 HOMED=1 MOVING=0 HOMING=0 SPEED=0.000 DRILL=0 LOAD_G=0.00 LOAD_READY=1 FORCE_G=0.00 FORCE_WARN=0 FAULT=0 FAULT_ACTIVE=0
```

Error examples:

```text
ERR NOT_HOMED
ERR BAD_COMMAND <command>
ERR COMMS_TIMEOUT
ERR FORCE_LIMIT
```

Fault codes:

```text
0 NONE
1 TOP_LIMIT_WHILE_MOVING_UP
2 NOT_HOMED
3 COMMS_TIMEOUT
4 BAD_COMMAND
5 ESTOP
6 FORCE_LIMIT
7 HOMING_TIMEOUT
```

## Recommended ROS 2 Mapping

ROS 2 actions on Jetson:

```text
/drill_arm/home
/drill_arm/move_y
/drill_arm/drill
```

ROS 2 services on Jetson:

```text
/drill_arm/stop
/drill_arm/estop
/drill_arm/clear_fault
```

ROS 2 topic from Jetson:

```text
/drill_arm/status
```
