#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "HX711.h"

// Tune-playback note: a stepper "sings" at its step rate, so a melody is
// just a sequence of (frequency, duration) pairs. Declared up top because
// the Arduino IDE auto-generates function prototypes before this point,
// and those prototypes need this type to already be visible.
struct Note {
    float freq_hz;            // 0 = rest (silent pause)
    unsigned long duration_ms;
};

// "The Imperial March" (Star Wars) — opening phrase + variation.
const Note TUNE_IMPERIAL_MARCH[] = {
    {392.00, 400}, // G4
    {392.00, 400}, // G4
    {392.00, 400}, // G4
    {311.13, 200}, // Eb4
    {466.16, 200}, // Bb4
    {392.00, 400}, // G4
    {311.13, 200}, // Eb4
    {466.16, 200}, // Bb4
    {392.00, 800}, // G4 (held)
    {0,      150}, // rest
    {587.33, 400}, // D5
    {587.33, 400}, // D5
    {587.33, 400}, // D5
    {622.25, 200}, // Eb5
    {466.16, 200}, // Bb4
    {369.99, 400}, // Gb4
    {311.13, 200}, // Eb4
    {466.16, 200}, // Bb4
    {392.00, 800}, // G4 (held)
};
const int TUNE_IMPERIAL_MARCH_LEN = sizeof(TUNE_IMPERIAL_MARCH) / sizeof(Note);

// "Hatikvah" — opening melodic line (D minor: rising D-E-F-G-A, falling Bb-A-G-F-E-D).
// Best-effort transcription of the well-known contour — adjust note timing if it's off.
const Note TUNE_HATIKVAH[] = {
    {293.66, 350}, // D4
    {329.63, 350}, // E4
    {349.23, 350}, // F4
    {392.00, 350}, // G4
    {440.00, 350}, // A4
    {440.00, 600}, // A4 (held)
    {0,      150}, // rest
    {466.16, 350}, // Bb4
    {440.00, 350}, // A4
    {392.00, 350}, // G4
    {349.23, 350}, // F4
    {329.63, 350}, // E4
    {293.66, 600}, // D4 (held)
};
const int TUNE_HATIKVAH_LEN = sizeof(TUNE_HATIKVAH) / sizeof(Note);

// =========================================================
//  DRILLING ARM CONTROLLER - TEENSY 4.1
//  USB Serial protocol for Jetson / ROS 2 bridge
//
//  Hardware:
//  - Teensy 4.1
//  - TB6600 stepper driver + NEMA17 17HS9401S (1.8 deg, 200 steps/rev, 1.7A)
//    Wiring: all + terminals (PUL+, DIR+, ENA+) -> 3.3V
//            PUL- -> pin 3, DIR- -> pin 4, ENA- -> pin 2
//            All signals are ACTIVE LOW (common anode, optocoupler isolated).
//            Microstepping is set via DIP switches on the TB6600 (not via pins).
//  - Cytron MD10C r3 + one DC drill motor
//  - One HX711 amplifier reading two matched load cells wired together
//  - One top limit switch
// =========================================================

// -----------------------------
// Pin definitions
// -----------------------------
const int Y_STEP_PIN = 3;
const int Y_DIR_PIN = 4;
const int Y_EN_PIN = 2;
const int Y_LIMIT_TOP_PIN = 5;

const int DRILL_PWM_PIN = 6;
const int DRILL_DIR_PIN = 7;

// Do not reuse pins 5, 6, or 7 for HX711:
// 5 is the top limit switch, 6 is drill PWM, 7 is drill DIR.
// Current load-cell wiring:
// - Two physical load cells are combined into one HX711 input.
// - Both green signal wires are tied together to HX711 A+.
// - Both white signal wires are tied together to HX711 A-.
// - Both red excitation wires should go to HX711 E+.
// - Both black excitation wires should go to HX711 E-.
// - HX711 DOUT/DT goes to Teensy pin 8.
// - HX711 SCK/CLK goes to Teensy pin 27.
const int COMBINED_LOADCELL_DATA_PIN = 8;
const int COMBINED_LOADCELL_CLOCK_PIN = 27;

// -----------------------------
// Motion configuration
// IMPORTANT: confirm these with the mechanical team.
// -----------------------------
const float MOTOR_FULL_STEPS_PER_REV = 200.0;  // 1.8 degree NEMA17 (17HS9401S)
const float MICROSTEPS = 4.0;                  // TB6600 DIP switches: S1=OFF S2=OFF S3=ON = 4 microsteps.
const float STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;
const float MM_PER_REV = 2.0;                  // Lead screw/mechanism travel per motor revolution
const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

const float Y_MIN_MM = 0.0;                    // Top/home position
const float Y_MAX_MM = 480.0;                  // Software bottom limit
const float DEFAULT_Y_SPEED_MM_S = 0.75;
const float HOMING_SPEED_MM_S = 8;
const float MIN_Y_SPEED_MM_S = 0.3;            // Ramp start/end speed — not a ceiling on requested speed.
const float Y_ACCEL_MM_S2 = 5.0;
const long HOMING_BACKOFF_STEPS = 800;         // 2 mm at 400 steps/mm.

// Load cell configuration.
// Replace these calibration values with values measured for your real load cells.
// One HX711 reading the combined pair of physical load cells.
const bool USE_COMBINED_LOADCELL = true;
const float COMBINED_LOADCELL_CALIBRATION = 604.825;  // Halved: readings were reading 2x too low at old value
const unsigned long LOAD_CELL_READ_INTERVAL_MS = 100;
const float FORCE_WARNING_G = 20000.0;         // 20 kg warning threshold in STATUS.
const float MAX_FORCE_G = 25000.0;             // 25 kg hard stop threshold.
const float AUTO_TARE_AFTER_HOME_MAX_G = 40.0; // Only auto-tare on HOME if load looks like drift, not a real sample.

const int NUM_BASELINE_POINTS = 13;      // Y = 0, 40, 80 … 480 mm
const float BASELINE_STEP_MM = 40.0;
const unsigned long BASELINE_SETTLE_MS = 600;

// ToF sensor configuration (TWTOF240UI, I2C).
// SDA = Teensy pin 18, SCL = Teensy pin 19. Power: 3.3V only.
const uint8_t TOF_I2C_ADDR = 0x36;
const uint8_t TOF_CMD_DISTANCE = 0x81;
const unsigned long TOF_READ_INTERVAL_MS = 50;   // Sensor updates every 36 ms; poll at 50 ms.
const int TOF_MIN_CLEARANCE_MM = 50;             // Stop downward motion if closer than this to ground.
const int TOF_GROUND_CONTACT_MM = 10;            // Drill has reached the surface.

// Direction convention:
// Top limit switch is at Y = 0.
// Moving down increases Y.
// TB6600 common anode (+ to 3.3V, signal on -): output is inverted through optocoupler.
// LOW on Teensy pin -> optocoupler ON -> TB6600 sees active signal.
// If the motor moves the wrong way after testing, swap DIR_UP and DIR_DOWN.
const int DIR_UP = HIGH;
const int DIR_DOWN = LOW;

// TB6600 common anode: HIGH on ENA- = no current = ENA inactive = driver ENABLED.
//                      LOW on ENA-  = current flows = ENA active = driver DISABLED.
const int STEPPER_ENABLE_ACTIVE = HIGH;
const int STEPPER_ENABLE_DISABLED = LOW;

// -----------------------------
// Runtime state
// Variables used by the timer ISR must be volatile.
// -----------------------------
volatile long y_current_steps = 0;
volatile uint8_t limit_top_debounce = 0;
volatile long y_target_steps = 0;
volatile bool y_moving = false;
volatile bool y_homing = false;
volatile bool y_homed = false;
volatile bool fault_active = false;
volatile int fault_code = 0;
volatile int y_homing_phase = 0;
volatile long y_homing_step_count = 0;
volatile long y_homing_backoff_count = 0;
volatile bool homing_just_completed = false;

volatile unsigned long y_step_interval_us = 1000;
volatile unsigned long y_last_step_us = 0;
volatile float y_current_speed_mm_s = 0.0;
volatile float y_requested_speed_mm_s = DEFAULT_Y_SPEED_MM_S;

float y_speed_mm_s = DEFAULT_Y_SPEED_MM_S;
int drill_speed_percent = 0;
int drill_target_percent = 0;
unsigned long last_drill_ramp_ms = 0;
const int DRILL_RAMP_STEP_PERCENT = 5;
const unsigned long DRILL_RAMP_INTERVAL_MS = 20;
const int MAX_DRILL_PERCENT = 100;

HX711 combined_loadcell;
float combined_loadcell_g = 0.0;
bool combined_loadcell_ready = false;
float total_force_g = 0.0;
bool force_warning_active = false;
unsigned long last_loadcell_read_ms = 0;

float baseline_g[NUM_BASELINE_POINTS];
bool baseline_calibrated = false;
bool baseline_calibrating = false;
bool baseline_pending = false;
bool first_home_done = false;
bool boot_sequence_active = false;   // BOOT command: home -> tare -> baseline calibration

int16_t tof_distance_mm = -1;
uint8_t tof_status_code = 0xFF;
bool tof_valid = false;
unsigned long last_tof_read_ms = 0;

unsigned long last_command_ms = 0;
const unsigned long COMMAND_TIMEOUT_MS = 300000; // 5 min for manual testing; Jetson should use ~2000 ms in production.

IntervalTimer stepper_timer;

// -----------------------------
// Fault codes
// -----------------------------
const int FAULT_NONE = 0;
const int FAULT_TOP_LIMIT_WHILE_MOVING_UP = 1;
const int FAULT_NOT_HOMED = 2;
const int FAULT_COMMS_TIMEOUT = 3;
const int FAULT_BAD_COMMAND = 4;
const int FAULT_ESTOP = 5;
const int FAULT_FORCE_LIMIT = 6;
const int FAULT_HOMING_TIMEOUT = 7;
const int FAULT_TOF_CLEARANCE = 8;
const int FAULT_GROUND_CONTACT = 9;

// -----------------------------
// Helpers
// -----------------------------
float stepsToMm(long steps)
{
    return steps / STEPS_PER_MM;
}

long mmToSteps(float mm)
{
    return lroundf(mm * STEPS_PER_MM);
}

bool topLimitPressed()
{
    if (digitalRead(Y_LIMIT_TOP_PIN) == LOW) {
        if (limit_top_debounce < 50) limit_top_debounce++;
    } else {
        limit_top_debounce = 0;
    }
    return limit_top_debounce >= 50;
}

void setFault(int code)
{
    fault_active = true;
    fault_code = code;
    y_moving = false;
    y_homing = false;
    y_homing_phase = 0;
    y_current_speed_mm_s = 0.0;
}

void clearFault()
{
    fault_active = false;
    fault_code = FAULT_NONE;
}

void enableStepper()
{
    digitalWrite(Y_EN_PIN, STEPPER_ENABLE_ACTIVE);
}

void disableStepper()
{
    digitalWrite(Y_EN_PIN, STEPPER_ENABLE_DISABLED);
}

void updateStepInterval(float speed_mm_s)
{
    y_speed_mm_s = speed_mm_s;
    y_requested_speed_mm_s = speed_mm_s;

    float steps_per_second = speed_mm_s * STEPS_PER_MM;
    if (steps_per_second < 1.0f) {
        steps_per_second = 1.0f;
    }

    y_step_interval_us = (unsigned long)(1000000.0f / steps_per_second);
}

void setCurrentMotionSpeed(float speed_mm_s)
{
    y_current_speed_mm_s = speed_mm_s;

    float steps_per_second = speed_mm_s * STEPS_PER_MM;
    if (steps_per_second < 1.0f) {
        steps_per_second = 1.0f;
    }

    y_step_interval_us = (unsigned long)(1000000.0f / steps_per_second);
}

void updateTrapezoidSpeed(long remaining_steps)
{
    float remaining_mm = fabsf(stepsToMm(remaining_steps));
    float speed = y_current_speed_mm_s;
    float requested_speed = y_requested_speed_mm_s;
    float step_mm = 1.0f / STEPS_PER_MM;
    float stopping_mm = (speed * speed) / (2.0f * Y_ACCEL_MM_S2);

    if (remaining_mm <= step_mm) {
        y_current_speed_mm_s = 0.0f;
        return;
    }

    if (remaining_mm <= stopping_mm + step_mm) {
        speed = sqrtf(fmaxf(0.0f, (speed * speed) - (2.0f * Y_ACCEL_MM_S2 * step_mm)));
    } else {
        speed = sqrtf((speed * speed) + (2.0f * Y_ACCEL_MM_S2 * step_mm));
    }

    speed = constrain(speed, MIN_Y_SPEED_MM_S, requested_speed);
    setCurrentMotionSpeed(speed);
}

void pulseStep()
{
    // TB6600 common anode: pulse LOW to trigger a step (optocoupler activates on LOW).
    // TB6600 minimum pulse width is 2.2 µs; 5 µs used here for safety margin.
    digitalWrite(Y_STEP_PIN, LOW);
    delayNanoseconds(5000);
    digitalWrite(Y_STEP_PIN, HIGH);
}

// -----------------------------
// Timer ISR: generates stepper pulses
// -----------------------------
void driveStepper()
{
    if (fault_active) {
        return;
    }

    // Update debounce every ISR tick regardless of step rate
    topLimitPressed();

    unsigned long now = micros();
    if (now - y_last_step_us < y_step_interval_us) {
        return;
    }

    if (y_homing) {
        y_last_step_us = now;

        if (y_homing_phase == 2) {
            digitalWrite(Y_DIR_PIN, DIR_DOWN);
            pulseStep();
            y_current_steps++;
            y_homing_backoff_count++;

            if (y_homing_backoff_count >= HOMING_BACKOFF_STEPS && !topLimitPressed()) {
                y_homing_phase = 3;
                y_homing_step_count = 0;
                limit_top_debounce = 0;
            } else if (y_homing_backoff_count > HOMING_BACKOFF_STEPS * 5) {
                setFault(FAULT_HOMING_TIMEOUT);
            }

            return;
        }

        digitalWrite(Y_DIR_PIN, DIR_UP);
        pulseStep();
        y_current_steps--;
        y_homing_step_count++;

        if (topLimitPressed()) {
            if (y_homing_phase == 1) {
                y_homing_phase = 2;
                y_homing_backoff_count = 0;
            } else {
                y_homing = false;
                y_moving = false;
                y_homed = true;
                y_homing_phase = 0;
                y_current_steps = 0;
                y_target_steps = 0;
                y_current_speed_mm_s = 0.0;
                homing_just_completed = true;
            }
        }

        return;
    }

    if (!y_moving) {
        return;
    }

    if (y_current_steps == y_target_steps) {
        y_moving = false;
        y_current_speed_mm_s = 0.0;
        return;
    }

    bool moving_up = y_target_steps < y_current_steps;
    if (moving_up && topLimitPressed()) {
        bool unexpected = (y_target_steps != 0);
        y_current_steps = 0;
        y_target_steps = 0;
        y_homed = true;
        y_moving = false;
        y_current_speed_mm_s = 0.0;
        if (unexpected) {
            setFault(FAULT_TOP_LIMIT_WHILE_MOVING_UP);
        }
        return;
    }

    y_last_step_us = now;

    if (moving_up) {
        digitalWrite(Y_DIR_PIN, DIR_UP);
        pulseStep();
        y_current_steps--;
    } else {
        digitalWrite(Y_DIR_PIN, DIR_DOWN);
        pulseStep();
        y_current_steps++;
    }

    long remaining_steps = labs(y_target_steps - y_current_steps);
    if (remaining_steps == 0) {
        y_moving = false;
        y_current_speed_mm_s = 0.0;
    } else if (!moving_up) {
        // Moving up runs at a constant speed (set in moveYAbsolute), no ramp.
        updateTrapezoidSpeed(remaining_steps);
    }
}

// -----------------------------
// Stepper commands
// -----------------------------
void homeY()
{
    clearFault();
    enableStepper();

    if (topLimitPressed()) {
        y_homing_phase = 2;
    } else {
        y_homing_phase = 1;
    }

    y_homing_step_count = 0;
    y_homing_backoff_count = 0;
    setCurrentMotionSpeed(HOMING_SPEED_MM_S);
    updateStepInterval(HOMING_SPEED_MM_S);
    y_moving = false;
    y_homing = true;
    Serial.println("ACK HOME");
}

void markMoveStopped()
{
    y_moving = false;
    y_homing = false;
    y_homing_phase = 0;
    y_current_speed_mm_s = 0.0;
}

void moveYAbsolute(float target_mm, float speed_mm_s)
{
    clearFault();
    target_mm = constrain(target_mm, Y_MIN_MM, Y_MAX_MM);
    updateStepInterval(speed_mm_s);

    y_target_steps = mmToSteps(target_mm);

    if (y_target_steps == y_current_steps) {
        markMoveStopped();
        Serial.println("ACK MOVE");
        return;
    }

    bool moving_up = y_target_steps < y_current_steps;
    if (moving_up) {
        // Lifting against gravity is rough at low ramp-start speeds (torque
        // ripple fights gravity instead of being masked by it). Homing proves
        // the motor handles an instant jump to speed while lifting, so skip
        // the ramp and run at the requested speed the whole way, like homing.
        setCurrentMotionSpeed(speed_mm_s);
    } else {
        setCurrentMotionSpeed(MIN_Y_SPEED_MM_S);
    }
    y_moving = true;
    y_homing = false;
    y_homing_phase = 0;
    enableStepper();

    Serial.println("ACK MOVE");
}

void moveYRelative(float delta_mm, float speed_mm_s)
{
    float current_mm = stepsToMm(y_current_steps);
    moveYAbsolute(current_mm + delta_mm, speed_mm_s);
}

// Silent move used by baseline calibration — no ACK, no fault clear.
void moveYAbsoluteSilent(float target_mm, float speed_mm_s)
{
    if (fault_active) return;
    target_mm = constrain(target_mm, Y_MIN_MM, Y_MAX_MM);
    updateStepInterval(speed_mm_s);
    y_target_steps = mmToSteps(target_mm);
    if (y_target_steps == y_current_steps) {
        markMoveStopped();
        return;
    }
    bool moving_up = y_target_steps < y_current_steps;
    setCurrentMotionSpeed(moving_up ? speed_mm_s : MIN_Y_SPEED_MM_S);
    y_moving = true;
    y_homing = false;
    y_homing_phase = 0;
    enableStepper();
}

float interpolateBaseline(float y_mm)
{
    if (!baseline_calibrated) return 0.0f;
    float idx_f = y_mm / BASELINE_STEP_MM;
    int idx = (int)idx_f;
    idx = constrain(idx, 0, NUM_BASELINE_POINTS - 2);
    float t = idx_f - (float)idx;
    return baseline_g[idx] * (1.0f - t) + baseline_g[idx + 1] * t;
}

// Runs after first HOME: sweeps Y to record the mechanical force offset at each
// calibration point. The arm must be empty. Handles STATUS and STOP/ESTOP
// mid-sweep so the bridge and GUI stay responsive.
void runBaselineCalibration()
{
    baseline_calibrating = true;
    Serial.println("INFO BASELINE_CAL_START");

    for (int i = 0; i < NUM_BASELINE_POINTS; i++) {
        if (fault_active) {
            baseline_calibrating = false;
            Serial.println("ERR BASELINE_CAL_ABORTED_FAULT");
            return;
        }

        float target_mm = i * BASELINE_STEP_MM;
        Serial.print("INFO BASELINE_CAL_MOVING Y=");
        Serial.println(target_mm, 1);

        moveYAbsoluteSilent(target_mm, HOMING_SPEED_MM_S);

        unsigned long move_timeout = millis() + 30000UL;
        while (y_moving && millis() < move_timeout) {
            if (Serial.available()) {
                String cmd = Serial.readStringUntil('\n');
                cmd.trim(); cmd.toUpperCase();
                last_command_ms = millis();
                if (cmd == "ESTOP") { emergencyStop(); baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
                if (cmd == "STOP")  { stopMotion();    baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
                if (cmd == "STATUS" || cmd == "M114") printStatus();
            }
            updateLoadCells();
            updateToF();
            checkToFSafety();
            if (fault_active) { baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_FAULT"); return; }
        }

        unsigned long settle_end = millis() + BASELINE_SETTLE_MS;
        while (millis() < settle_end) {
            if (Serial.available()) {
                String cmd = Serial.readStringUntil('\n');
                cmd.trim(); cmd.toUpperCase();
                last_command_ms = millis();
                if (cmd == "ESTOP") { emergencyStop(); baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
                if (cmd == "STOP")  { stopMotion();    baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
                if (cmd == "STATUS" || cmd == "M114") printStatus();
            }
            updateLoadCells();
        }

        baseline_g[i] = combined_loadcell_g;
        Serial.print("INFO BASELINE_CAL_POINT Y=");
        Serial.print(target_mm, 1);
        Serial.print(" OFFSET_G=");
        Serial.println(baseline_g[i], 2);
    }

    // Return to home
    moveYAbsoluteSilent(0.0f, HOMING_SPEED_MM_S);
    unsigned long home_timeout = millis() + 30000UL;
    while (y_moving && millis() < home_timeout) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim(); cmd.toUpperCase();
            last_command_ms = millis();
            if (cmd == "ESTOP") { emergencyStop(); baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
            if (cmd == "STOP")  { stopMotion();    baseline_calibrating = false; Serial.println("ERR BASELINE_CAL_ABORTED_USER"); return; }
            if (cmd == "STATUS" || cmd == "M114") printStatus();
        }
        updateLoadCells();
    }

    baseline_calibrated = true;
    baseline_calibrating = false;
    Serial.println("INFO BASELINE_CAL_COMPLETE — force readings are now position-compensated");
}

// -----------------------------
// Tune playback — steps the motor at musical note frequencies.
// A stepper "sings" at its step rate; varying the rate plays a melody.
// Direction alternates per note (and near travel limits) so the carriage
// oscillates in place instead of drifting away.
// -----------------------------
bool tuneAbortRequested()
{
    if (!Serial.available()) {
        return false;
    }
    // Only a real STOP/ESTOP should cut the tune short. Anything else
    // (e.g. a bridge's routine STATUS poll) is read and discarded so it
    // doesn't false-trigger an abort, but doesn't get treated as a command.
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    return cmd == "STOP" || cmd == "ESTOP";
}

void playNote(float freq_hz, unsigned long duration_ms, bool &dir_up)
{
    if (freq_hz <= 0) {
        delay(duration_ms);
        return;
    }

    unsigned long half_period_us = (unsigned long)(500000.0f / freq_hz);
    unsigned long end_ms = millis() + duration_ms;

    digitalWrite(Y_DIR_PIN, dir_up ? DIR_UP : DIR_DOWN);

    while (millis() < end_ms) {
        float current_mm = stepsToMm(y_current_steps);
        if (dir_up && current_mm <= 2.0f) {
            dir_up = false;
            digitalWrite(Y_DIR_PIN, DIR_DOWN);
        } else if (!dir_up && current_mm >= Y_MAX_MM - 2.0f) {
            dir_up = true;
            digitalWrite(Y_DIR_PIN, DIR_UP);
        }

        pulseStep();
        if (dir_up) {
            y_current_steps--;
        } else {
            y_current_steps++;
        }

        delayMicroseconds(half_period_us);

        if (tuneAbortRequested()) {
            return;
        }
    }
}

void playTune(const Note *notes, int len)
{
    clearFault();
    enableStepper();
    y_moving = false;
    y_homing = false;

    bool dir_up = true;
    for (int i = 0; i < len; i++) {
        if (tuneAbortRequested()) {
            break;
        }
        playNote(notes[i].freq_hz, notes[i].duration_ms, dir_up);
        delay(20); // brief gap between notes for articulation
    }

    y_current_speed_mm_s = 0.0;
}

// -----------------------------
// DC motor commands
// -----------------------------
void applyDrillPwm(int percent)
{
    if (percent == 0) {
        analogWrite(DRILL_PWM_PIN, 0);
        return;
    }
    digitalWrite(DRILL_DIR_PIN, percent > 0 ? HIGH : LOW);
    analogWrite(DRILL_PWM_PIN, map(abs(percent), 0, 100, 0, 255));
}

void setDrillSpeed(int percent)
{
    drill_target_percent = constrain(percent, -MAX_DRILL_PERCENT, MAX_DRILL_PERCENT);
    Serial.println("ACK DRILL");
}

void stopDrill()
{
    drill_target_percent = 0;
    drill_speed_percent = 0;
    analogWrite(DRILL_PWM_PIN, 0);
}

void updateDrillRamp()
{
    if (drill_speed_percent == drill_target_percent) return;
    if (millis() - last_drill_ramp_ms < DRILL_RAMP_INTERVAL_MS) return;
    last_drill_ramp_ms = millis();

    if (drill_speed_percent < drill_target_percent) {
        drill_speed_percent = min(drill_speed_percent + DRILL_RAMP_STEP_PERCENT, drill_target_percent);
    } else {
        drill_speed_percent = max(drill_speed_percent - DRILL_RAMP_STEP_PERCENT, drill_target_percent);
    }
    applyDrillPwm(drill_speed_percent);
}

void stopMotionNoAck()
{
    y_moving = false;
    y_homing = false;
    y_homing_phase = 0;
    y_current_speed_mm_s = 0.0;
    stopDrill();
}

// Robust tare: waits for HX711 samples (it only produces one every ~100 ms,
// so an instant is_ready() check usually misses) and averages several raw
// reads before setting the new zero offset. Interrupts are disabled only
// during each brief bit-bang readout, not the whole wait.
// Returns true if the offset was actually updated.
bool tareLoadCellsSilent()
{
    if (!USE_COMBINED_LOADCELL) return false;

    const int TARE_SAMPLES = 5;
    const unsigned long TARE_TIMEOUT_MS = 1500;

    long long sum = 0;
    int got = 0;
    unsigned long start = millis();

    while (got < TARE_SAMPLES && millis() - start < TARE_TIMEOUT_MS) {
        noInterrupts();
        bool ready = combined_loadcell.is_ready();
        interrupts();
        if (!ready) {
            delay(5);
            continue;
        }
        noInterrupts();
        long v = combined_loadcell.read();
        interrupts();
        sum += v;
        got++;
    }

    if (got == 0) return false;
    combined_loadcell.set_offset((long)(sum / got));
    return true;
}

void tareLoadCells()
{
    if (tareLoadCellsSilent()) {
        Serial.println("ACK TARE");
    } else {
        Serial.println("ERR TARE_NOT_READY");
    }
}

void updateLoadCells()
{
    if (millis() - last_loadcell_read_ms < LOAD_CELL_READ_INTERVAL_MS) {
        return;
    }

    last_loadcell_read_ms = millis();

    if (USE_COMBINED_LOADCELL) {
        // Disable interrupts during HX711 read — the stepper ISR bit-bangs
        // the step pin and its timing interferes with the HX711 clock line.
        noInterrupts();
        combined_loadcell_ready = combined_loadcell.is_ready();
        if (combined_loadcell_ready) {
            combined_loadcell_g = combined_loadcell.get_units(1);
        }
        interrupts();
    }

    total_force_g = 0.0;
    if (USE_COMBINED_LOADCELL && combined_loadcell_ready) {
        float compensated = combined_loadcell_g - interpolateBaseline(stepsToMm(y_current_steps));
        total_force_g = fabsf(compensated);
    }
    force_warning_active = total_force_g >= FORCE_WARNING_G;
}

void checkForceLimit()
{
    if (fault_active) {
        return;
    }

    if (total_force_g >= MAX_FORCE_G) {
        stopMotionNoAck();
        setFault(FAULT_FORCE_LIMIT);
        Serial.println("ERR FORCE_LIMIT");
    }
}

void checkToFSafety()
{
    if (fault_active) return;

    // Limit switch ON + ToF below threshold = unexpected conflict → full ESTOP.
    // Skip during homing when limit switch contact is intentional.
    if (!y_homing && topLimitPressed() && tof_valid && tof_distance_mm < TOF_MIN_CLEARANCE_MM) {
        emergencyStop();
        Serial.println("ERR LIM_TOF_CONFLICT");
        return;
    }

    if (!tof_valid) return;

    bool moving_down = y_moving && (y_target_steps > y_current_steps);

    // Ground contact: drill has reached the surface → stop Y motion.
    if (moving_down && tof_distance_mm <= TOF_GROUND_CONTACT_MM) {
        stopMotionNoAck();
        setFault(FAULT_GROUND_CONTACT);
        Serial.println("ERR GROUND_CONTACT");
        return;
    }

    // Minimum clearance: too close to ground → stop Y motion.
    if (moving_down && tof_distance_mm < TOF_MIN_CLEARANCE_MM) {
        stopMotionNoAck();
        setFault(FAULT_TOF_CLEARANCE);
        Serial.println("ERR TOF_CLEARANCE");
    }
}

void stopMotion()
{
    stopMotionNoAck();
    Serial.println("ACK STOP");
}

void emergencyStop()
{
    stopMotionNoAck();
    disableStepper();
    setFault(FAULT_ESTOP);
    Serial.println("ACK ESTOP");
}

// -----------------------------
// ToF sensor
// -----------------------------
void updateToF()
{
    if (millis() - last_tof_read_ms < TOF_READ_INTERVAL_MS) return;
    last_tof_read_ms = millis();

    // Trigger a distance measurement.
    Wire.beginTransmission(TOF_I2C_ADDR);
    Wire.write(TOF_CMD_DISTANCE);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) {
        tof_valid = false;
        return;
    }

    // Read back 5 bytes: [cmd, len, dist_hi, dist_lo, status]
    Wire.requestFrom(TOF_I2C_ADDR, (uint8_t)5);
    if (Wire.available() < 5) {
        tof_valid = false;
        return;
    }

    uint8_t buf[5];
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();

    tof_distance_mm  = ((int16_t)buf[2] << 8) | buf[3];
    tof_status_code  = buf[4];
    tof_valid        = (tof_status_code == 0x00);
}

// -----------------------------
// Status
// -----------------------------
void printStatus()
{
    Serial.print("STATUS");
    Serial.print(" Y=");
    Serial.print(stepsToMm(y_current_steps), 3);
    Serial.print(" TARGET=");
    Serial.print(stepsToMm(y_target_steps), 3);
    Serial.print(" LIMIT_TOP=");
    Serial.print(topLimitPressed() ? 1 : 0);
    Serial.print(" HOMED=");
    Serial.print(y_homed ? 1 : 0);
    Serial.print(" MOVING=");
    Serial.print(y_moving ? 1 : 0);
    Serial.print(" HOMING=");
    Serial.print(y_homing ? 1 : 0);
    Serial.print(" SPEED=");
    Serial.print((float)y_current_speed_mm_s, 3);
    Serial.print(" DRILL=");
    Serial.print(drill_speed_percent);
    Serial.print(" LOAD_G=");
    Serial.print(combined_loadcell_g, 2);
    Serial.print(" LOAD_READY=");
    Serial.print(combined_loadcell_ready ? 1 : 0);
    Serial.print(" FORCE_G=");
    Serial.print(total_force_g, 2);
    Serial.print(" FORCE_WARN=");
    Serial.print(force_warning_active ? 1 : 0);
    Serial.print(" TOF_MM=");
    Serial.print(tof_distance_mm);
    Serial.print(" TOF_VALID=");
    Serial.print(tof_valid ? 1 : 0);
    Serial.print(" FAULT=");
    Serial.print(fault_code);
    Serial.print(" FAULT_ACTIVE=");
    Serial.print(fault_active ? 1 : 0);
    Serial.print(" CALIBRATING=");
    Serial.println(baseline_calibrating ? 1 : 0);
}

float readValueAfterKey(const String &s, const String &key, float default_value)
{
    int idx = s.indexOf(key);
    if (idx < 0) {
        return default_value;
    }
    return s.substring(idx + key.length()).toFloat();
}

// -----------------------------
// USB Serial command parser
// -----------------------------
void handleCommand(String command)
{
    command.trim();
    command.toUpperCase();

    if (command.length() == 0) {
        return;
    }

    last_command_ms = millis();

    if (command == "HOME" || command == "G28") {
        homeY();
    } else if (command == "BOOT") {
        // Full startup sequence: home -> tare -> baseline calibration.
        // Tare and calibration are chained in loop() once homing completes.
        boot_sequence_active = true;
        Serial.println("ACK BOOT");
        homeY();
    } else if (command.startsWith("MOVE_REL")) {
        if (command.indexOf("Y=") < 0) {
            setFault(FAULT_BAD_COMMAND);
            Serial.print("ERR BAD_COMMAND ");
            Serial.println(command);
            Serial.println("HINT use: MOVE_REL Y=10 SPEED=1");
        } else {
            float dy = readValueAfterKey(command, "Y=", 0.0);
            float speed = readValueAfterKey(command, "SPEED=", y_speed_mm_s);
            moveYRelative(dy, speed);
        }
    } else if (command.startsWith("MOVE")) {
        if (command.indexOf("Y=") < 0) {
            setFault(FAULT_BAD_COMMAND);
            Serial.print("ERR BAD_COMMAND ");
            Serial.println(command);
            Serial.println("HINT use: MOVE Y=100 SPEED=1");
        } else {
            float y = readValueAfterKey(command, "Y=", stepsToMm(y_current_steps));
            float speed = readValueAfterKey(command, "SPEED=", y_speed_mm_s);
            moveYAbsolute(y, speed);
        }
    } else if (command.startsWith("DRILL")) {
        if (command.indexOf("PERCENT=") < 0) {
            setFault(FAULT_BAD_COMMAND);
            Serial.print("ERR BAD_COMMAND ");
            Serial.println(command);
            Serial.println("HINT use: DRILL PERCENT=50");
        } else {
            int percent = (int)readValueAfterKey(command, "PERCENT=", 0);
            setDrillSpeed(percent);
        }
    } else if (command.startsWith("TUNE")) {
        String tune_name = command.substring(4);
        tune_name.trim();
        if (tune_name == "HATIKVAH") {
            playTune(TUNE_HATIKVAH, TUNE_HATIKVAH_LEN);
        } else {
            playTune(TUNE_IMPERIAL_MARCH, TUNE_IMPERIAL_MARCH_LEN);
        }
        Serial.println("ACK TUNE");
    } else if (command == "STOP") {
        stopMotion();
    } else if (command == "ESTOP") {
        emergencyStop();
    } else if (command == "STATUS" || command == "M114") {
        printStatus();
    } else if (command == "POS") {
        Serial.print("POS Y=");
        Serial.print(stepsToMm(y_current_steps), 3);
        Serial.print(" TARGET=");
        Serial.println(stepsToMm(y_target_steps), 3);
    } else if (command == "SET_HOME") {
        y_current_steps = 0;
        y_target_steps  = 0;
        y_homed         = true;
        y_moving        = false;
        y_homing        = false;
        y_homing_phase  = 0;
        y_current_speed_mm_s = 0.0;
        Serial.println("ACK SET_HOME");
    } else if (command == "CLEAR_FAULT") {
        clearFault();
        Serial.println("ACK CLEAR_FAULT");
    } else if (command == "TARE") {
        tareLoadCells();
    } else {
        stopMotionNoAck();
        setFault(FAULT_BAD_COMMAND);
        Serial.print("ERR BAD_COMMAND ");
        Serial.println(command);
    }
}

// -----------------------------
// Setup / loop
// -----------------------------
void setup()
{
    Serial.begin(115200);

    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_EN_PIN, OUTPUT);
    pinMode(Y_LIMIT_TOP_PIN, INPUT_PULLUP);

    // TB6600 common anode: STEP must idle HIGH so no spurious pulses on startup.
    digitalWrite(Y_STEP_PIN, HIGH);
    digitalWrite(Y_DIR_PIN, DIR_DOWN);

    pinMode(DRILL_PWM_PIN, OUTPUT);
    pinMode(DRILL_DIR_PIN, OUTPUT);
    analogWriteFrequency(DRILL_PWM_PIN, 10000);  // Cytron MD10C r3 accepts 5-20 kHz; Teensy default is ~36 kHz which is too high

    if (USE_COMBINED_LOADCELL) {
        combined_loadcell.begin(COMBINED_LOADCELL_DATA_PIN, COMBINED_LOADCELL_CLOCK_PIN);
        combined_loadcell.set_scale(COMBINED_LOADCELL_CALIBRATION);
    }

    Wire.begin();
    Wire.setClock(400000);

    disableStepper();
    stopDrill();
    updateStepInterval(DEFAULT_Y_SPEED_MM_S);

    last_command_ms = millis();

    stepper_timer.begin(driveStepper, 10);

    Serial.println("READY DRILL_ARM_TEENSY_USB_SERIAL");
}

void loop()
{
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        handleCommand(command);
    }

    updateLoadCells();
    checkForceLimit();
    updateDrillRamp();
    updateToF();
    checkToFSafety();

    if (homing_just_completed) {
        homing_just_completed = false;
        if (boot_sequence_active) {
            // BOOT sequence: tare unconditionally, then always run calibration.
            // Verify the tare actually took (readings near zero) before the
            // sweep starts, so the baseline isn't built on a stale offset.
            boot_sequence_active = false;
            bool tared = tareLoadCellsSilent();
            if (!tared) {
                tared = tareLoadCellsSilent();  // one retry
            }
            float residual = 0.0f;
            if (tared) {
                noInterrupts();
                residual = combined_loadcell.get_units(1);
                interrupts();
                combined_loadcell_g = residual;
            }
            if (tared && fabsf(residual) < 10.0f) {
                Serial.print("INFO BOOT_TARE_DONE RESIDUAL_G=");
                Serial.println(residual, 2);
            } else if (tared) {
                Serial.print("ERR BOOT_TARE_RESIDUAL_HIGH G=");
                Serial.println(residual, 2);
            } else {
                Serial.println("ERR BOOT_TARE_FAILED");
            }
            first_home_done = true;
            baseline_pending = true;
        } else {
            // Only re-zero if the load looks like drift, not a real sample sitting
            // on the cell (e.g. lifted up with the drill after a core sample).
            if (total_force_g < AUTO_TARE_AFTER_HOME_MAX_G) {
                tareLoadCellsSilent();
                Serial.println("INFO AUTO_TARE_AFTER_HOME");
            } else {
                Serial.println("INFO AUTO_TARE_SKIPPED_LOAD_PRESENT");
            }
            if (!first_home_done) {
                first_home_done = true;
                baseline_pending = true;
            }
        }
    }

    // If homing faulted mid-BOOT (e.g. dead limit switch), abandon the sequence.
    if (boot_sequence_active && fault_active) {
        boot_sequence_active = false;
        Serial.println("ERR BOOT_ABORTED_FAULT");
    }

    if (baseline_pending && !fault_active) {
        baseline_pending = false;
        runBaselineCalibration();
    }

    bool active_motion = y_moving || y_homing || drill_speed_percent != 0;
    if (!fault_active && active_motion && millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
        stopMotionNoAck();
        setFault(FAULT_COMMS_TIMEOUT);
        Serial.println("ERR COMMS_TIMEOUT");
    }
}
