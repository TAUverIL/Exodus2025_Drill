    // =========================================================
    //  HIGH-PERFORMANCE DUAL STEPPER CONTROLLER - TEENSY 4.1
    // =========================================================

    #include <Arduino.h>

    // ---------------------------------------------------------
    // PIN DEFINITIONS
    // ---------------------------------------------------------
    #define Y_STEP_PIN 3
    #define Y_DIR_PIN 4
    #define Y_EN_PIN 2
    #define Y_LIMIT_PIN 5

    // E axis pins (Adjust these to your physical wiring)
    #define E_STEP_PIN 6
    #define E_DIR_PIN 7
    #define E_EN_PIN 8

    // ---------------------------------------------------------
    // MOTION PARAMETERS
    // ---------------------------------------------------------
    const float STEPS_PER_REV = 400 *4; 
    const float MM_PER_REV = 8;           
    const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

    const float Y_MIN = 0.0;
    const float Y_MAX = 350.0;

    volatile long y_target_steps = 0;
    volatile long y_current_steps = 0;
    volatile bool y_moving = false;
    volatile unsigned long y_step_interval = 1;   
    volatile unsigned long y_last_step = 0;

    volatile bool y_homing = false;
    const float HOMING_SPEED_MMPS = 20.0; 

    volatile bool e_running = false;
    volatile bool e_dir_cw = true;
    volatile float e_speed_rps = 2.0;              
    volatile unsigned long e_step_interval = 1;    
    volatile unsigned long e_last_step = 0;

    float Y_SPEED_MMPS = 50.0;

    // Increased MAX_STEPS_PER_SEC for Teensy 4.1 performance
    const float MAX_STEPS_PER_SEC = 80000.0; 

    // Teensy High-Resolution Timer
    IntervalTimer stepperTimer;

    // ---------------------------------------------------------
    // HIGH-SPEED INTERRUPT HANDLER (100 kHz)
    // ---------------------------------------------------------
    void driveMotors() 
    {
        // Functionality preserved exactly from original logic
        unsigned long now = micros();

        if (y_homing)
        {
            if (now - y_last_step >= y_step_interval)
            {
                y_last_step = now;
                digitalWrite(Y_DIR_PIN, LOW); 
                digitalWrite(Y_STEP_PIN, HIGH);
                digitalWrite(Y_STEP_PIN, LOW);
                y_current_steps--;

                if (digitalRead(Y_LIMIT_PIN) == LOW) 
                {
                    y_homing = false;
                    y_current_steps = 0;
                    y_target_steps = 0;
                    y_moving = false;
                }
            }
        }
        else if (y_moving)
        {
            if (now - y_last_step >= y_step_interval)
            {
                y_last_step = now;
                if (y_current_steps < y_target_steps)
                {
                    digitalWrite(Y_DIR_PIN, HIGH);
                    y_current_steps++;
                }
                else if (y_current_steps > y_target_steps)
                {
                    digitalWrite(Y_DIR_PIN, LOW);
                    y_current_steps--;
                }
                else
                {
                    y_moving = false;
                }
                digitalWrite(Y_STEP_PIN, HIGH);
                digitalWrite(Y_STEP_PIN, LOW);
            }
        }

        if (e_running)
        {
            if (now - e_last_step >= e_step_interval)
            {
                e_last_step = now;
                digitalWrite(E_DIR_PIN, e_dir_cw ? HIGH : LOW);
                digitalWrite(E_STEP_PIN, HIGH);
                digitalWrite(E_STEP_PIN, LOW);
            }
        }
    }

    // ---------------------------------------------------------
    // SETUP
    // ---------------------------------------------------------
    void setup()
    {
        Serial.begin(115200);

        pinMode(Y_STEP_PIN, OUTPUT);
        pinMode(Y_DIR_PIN, OUTPUT);
        pinMode(Y_EN_PIN, OUTPUT);
        pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
        digitalWrite(Y_EN_PIN, LOW);

        pinMode(E_STEP_PIN, OUTPUT);
        pinMode(E_DIR_PIN, OUTPUT);
        pinMode(E_EN_PIN, OUTPUT);
        digitalWrite(E_EN_PIN, LOW);

        // Timing adjustment: 100 kHz (10 microseconds)
        // This allows for significantly smoother pulse generation
        stepperTimer.begin(driveMotors, 10); 

        Serial.println("Teensy 4.1 Optimized Ready.");
    }

    // ---------------------------------------------------------
    // REMAINING FUNCTIONS (Functionality Unchanged)
    // ---------------------------------------------------------

    void moveY(float mm) {
        if (mm < Y_MIN) mm = Y_MIN;
        if (mm > Y_MAX) mm = Y_MAX;
        y_target_steps = mm * STEPS_PER_MM;
        y_moving = true;
        float steps_per_sec = Y_SPEED_MMPS * STEPS_PER_MM;
        if (steps_per_sec < 1.0f) steps_per_sec = 1.0f;
        if (steps_per_sec > MAX_STEPS_PER_SEC) steps_per_sec = MAX_STEPS_PER_SEC;
        y_step_interval = (unsigned long)(1000000.0 / steps_per_sec);
    }

    void moveYRelative(float delta_mm) {
        long target_steps = y_current_steps + (delta_mm * STEPS_PER_MM);
        if (target_steps < Y_MIN * STEPS_PER_MM) target_steps = Y_MIN * STEPS_PER_MM;
        if (target_steps > Y_MAX * STEPS_PER_MM) target_steps = Y_MAX * STEPS_PER_MM;
        y_target_steps = target_steps;
        y_moving = true;
        float steps_per_sec = Y_SPEED_MMPS * STEPS_PER_MM;
        if (steps_per_sec < 1.0f) steps_per_sec = 1.0f;
        if (steps_per_sec > MAX_STEPS_PER_SEC) steps_per_sec = MAX_STEPS_PER_SEC;
        y_step_interval = (unsigned long)(1000000.0 / steps_per_sec);
    }

    void homeY() {
        y_homing = true;
        float steps_per_sec = HOMING_SPEED_MMPS * STEPS_PER_MM;
        y_step_interval = (unsigned long)(1000000.0 / steps_per_sec);
        digitalWrite(Y_EN_PIN, LOW);
    }

    void startE(bool cw) {
        e_dir_cw = cw;
        float steps_per_sec = e_speed_rps * STEPS_PER_REV;
        if (steps_per_sec > MAX_STEPS_PER_SEC) steps_per_sec = MAX_STEPS_PER_SEC;
        e_step_interval = (unsigned long)(1000000.0 / steps_per_sec);
        e_running = true;
    }

    void setESpeed(float rps) {
        if (rps <= 0) return;
        e_speed_rps = rps;
        float steps_per_sec = e_speed_rps * STEPS_PER_REV;
        if (steps_per_sec > MAX_STEPS_PER_SEC) steps_per_sec = MAX_STEPS_PER_SEC;
        e_step_interval = (unsigned long)(1000000.0 / steps_per_sec);
    }

    void stopE() { e_running = false; }

    void disableAllMotors() {
        y_moving = false;
        y_homing = false;
        digitalWrite(Y_EN_PIN, HIGH);
        e_running = false;
        digitalWrite(E_EN_PIN, HIGH);
    }

    void handleCommand(String s) {
        s.trim();
        s.toUpperCase();
        if (s.startsWith("G1")) {
            int idx = s.indexOf('Y');
            if (idx > 0) moveY(s.substring(idx + 1).toFloat());
        } else if (s.startsWith("M20")) {
            int idx = s.indexOf('Y');
            if (idx > 0) moveYRelative(s.substring(idx + 1).toFloat());
        } else if (s.startsWith("G28")) {
            homeY();
        } else if (s.startsWith("E1")) {
            int dir_idx = s.indexOf(' ');
            int s_idx = s.indexOf('S');
            if (dir_idx > 0) {
                String dirStr = s.substring(dir_idx + 1, (s_idx>0? s_idx : s.length()));
                dirStr.trim();
                if (dirStr == "0") startE(true);
                else if (dirStr == "1") startE(false);
            }
            if (s_idx > 0) setESpeed(s.substring(s_idx + 1).toFloat());
        } else if (s.startsWith("E0")) {
            stopE();
        } else if (s.startsWith("G0") && s.indexOf('S') > 0) {
            Y_SPEED_MMPS = s.substring(s.indexOf('S') + 1).toFloat();
        } else if (s.startsWith("M114")) {
            Serial.print("Current Y: ");
            Serial.println(y_current_steps / STEPS_PER_MM);
        } else if (s.startsWith("M118")) {
            disableAllMotors();
        }
    }

    void loop() {
        if (Serial.available()) {
            handleCommand(Serial.readStringUntil('\n'));
        }
    }