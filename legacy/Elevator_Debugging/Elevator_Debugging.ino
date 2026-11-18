#include <Arduino.h>

// ---------------- PINS ----------------
#define Y_STEP_PIN 3
#define Y_DIR_PIN 4
#define Y_EN_PIN 2
#define Y_LIMIT_PIN 5

#define DC_PWM_PIN 6
#define DC_DIR_PIN 7

// ---------------- MOTION ----------------
const float STEPS_PER_REV = 400.0;
const float LEAD_MM = 2.0;
const float STEPS_PER_MM = STEPS_PER_REV / LEAD_MM;

float currentMM = 0;
float targetMM = 0;

float maxSpeed = 2.0;
float accel = 8.0;

float currentSpeed = 0;

unsigned long lastStep = 0;

bool enabled = true;

// =========================================================
// STEP PULSE
// =========================================================
inline void stepPulse()
{
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(5);
}

// =========================================================
// LIMIT SWITCH
// =========================================================
inline bool limitHit()
{
    return digitalRead(Y_LIMIT_PIN) == LOW;
}

// =========================================================
// ENABLE / DISABLE
// =========================================================
void enableMotor()
{
    digitalWrite(Y_EN_PIN, LOW);
    enabled = true;
}

void disableMotor()
{
    digitalWrite(Y_EN_PIN, HIGH);
    enabled = false;
}

// =========================================================
// SAFE HOMING (FIXED: NO STALLING)
// =========================================================
void homeAxis()
{
    Serial.println("HOMING");

    digitalWrite(Y_DIR_PIN, LOW);

    // approach switch
    while (!limitHit())
    {
        stepPulse();
        delayMicroseconds(1800);
    }

    delay(80); // debounce + settle

    // BACK OFF (critical fix)
    digitalWrite(Y_DIR_PIN, HIGH);

    for (int i = 0; i < 250; i++)
    {
        stepPulse();
        delayMicroseconds(1800);
    }

    currentMM = 0;
    targetMM = 0;
    currentSpeed = 0;

    Serial.println("HOME DONE");
}

// =========================================================
// MOVE
// =========================================================
void moveTo(float mm)
{
    targetMM = constrain(mm, 0, 500);
}

// =========================================================
// MOTION ENGINE (UNCHANGED BUT STABLE)
// =========================================================
void updateMotion()
{
    if (!enabled) return;

    float dist = targetMM - currentMM;

    if (fabs(dist) < 0.002)
    {
        currentSpeed = 0;
        return;
    }

    float dir = (dist > 0) ? 1 : -1;

    float stopDist = (currentSpeed * currentSpeed) / (2.0 * accel);
    float targetSpeed = maxSpeed;

    if (fabs(dist) < stopDist)
        targetSpeed = sqrt(2.0 * accel * fabs(dist));

    if (currentSpeed < targetSpeed)
        currentSpeed += accel * 0.001;
    else
        currentSpeed -= accel * 0.001;

    currentSpeed = constrain(currentSpeed, 0, maxSpeed);

    float stepsPerSec = currentSpeed * STEPS_PER_MM;
    if (stepsPerSec < 1) return;

    unsigned long interval = 1000000.0 / stepsPerSec;
    unsigned long now = micros();

    if (now - lastStep >= interval)
    {
        lastStep = now;

        digitalWrite(Y_DIR_PIN, (dir > 0) ? HIGH : LOW);

        stepPulse();

        currentMM += dir * (1.0 / STEPS_PER_MM);

        // safety: bottom limit
        if (limitHit() && dir < 0)
        {
            currentMM = 0;
            targetMM = 0;
            currentSpeed = 0;
        }
    }
}

// =========================================================
// DC MOTOR
// =========================================================
void setDC(int pwm, bool dir)
{
    digitalWrite(DC_DIR_PIN, dir);
    analogWrite(DC_PWM_PIN, constrain(pwm,0,255));
}

void stopDC()
{
    analogWrite(DC_PWM_PIN, 0);
}

// =========================================================
// SERIAL COMMANDS (FIXED RESET)
// =========================================================
void handle(String cmd)
{
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.startsWith("GOTO"))
    {
        moveTo(cmd.substring(4).toFloat());
    }

    else if (cmd.startsWith("MOVE"))
    {
        moveTo(targetMM + cmd.substring(4).toFloat());
    }

    else if (cmd.startsWith("SPEED"))
    {
        maxSpeed = cmd.substring(5).toFloat();
        Serial.println("OK SPEED");
    }

    else if (cmd == "POS")
    {
        Serial.print("POS=");
        Serial.println(currentMM,3);
    }

    else if (cmd == "HOME")
    {
        homeAxis();
    }

    else if (cmd == "STOP")
    {
        disableMotor();
        stopDC();
        currentSpeed = 0;

        Serial.println("STOPPED");
    }

    else if (cmd == "RESET")
    {
        // FIXED RESET: NO RESUME MOTION BUG
        enableMotor();

        currentSpeed = 0;
        targetMM = currentMM;

        Serial.println("RESET SAFE");
    }

    else if (cmd.startsWith("DC"))
    {
        int sp, dir;
        sscanf(cmd.c_str(),"DC %d %d",&sp,&dir);
        setDC(sp,dir);
    }

    else if (cmd == "DCSTOP")
    {
        stopDC();
    }
}

// =========================================================
// SETUP
// =========================================================
void setup()
{
    Serial.begin(115200);

    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_EN_PIN, OUTPUT);
    pinMode(Y_LIMIT_PIN, INPUT_PULLUP);

    pinMode(DC_PWM_PIN, OUTPUT);
    pinMode(DC_DIR_PIN, OUTPUT);

    enableMotor();

    Serial.println("READY");
}

// =========================================================
// LOOP
// =========================================================
void loop()
{
    if (Serial.available())
        handle(Serial.readStringUntil('\n'));

    updateMotion();
}