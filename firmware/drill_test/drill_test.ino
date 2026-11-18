// Minimal drill motor test — Cytron MD10C r3
// Pin 6 = PWM, Pin 7 = DIR
// Commands over Serial at 115200:
//   DRILL 50    -> run at 50%
//   DRILL 0     -> stop
//   DRILL -50   -> reverse at 50%

const int DRILL_PWM_PIN = 6;
const int DRILL_DIR_PIN = 7;

void applyDrill(int percent)
{
    percent = constrain(percent, -100, 100);
    if (percent == 0) {
        analogWrite(DRILL_PWM_PIN, 0);
        Serial.println("DRILL STOPPED");
        return;
    }
    digitalWrite(DRILL_DIR_PIN, percent > 0 ? HIGH : LOW);
    int pwm = map(abs(percent), 0, 100, 0, 255);
    analogWrite(DRILL_PWM_PIN, pwm);
    Serial.print("DRILL ");
    Serial.print(percent);
    Serial.print("% -> PWM=");
    Serial.println(pwm);
}

void setup()
{
    Serial.begin(115200);
    pinMode(DRILL_PWM_PIN, OUTPUT);
    pinMode(DRILL_DIR_PIN, OUTPUT);
    analogWriteFrequency(DRILL_PWM_PIN, 10000);  // 10 kHz — within Cytron 5-20 kHz range
    analogWrite(DRILL_PWM_PIN, 0);
    Serial.println("READY — send: DRILL <percent>  (e.g. DRILL 50)");
}

void loop()
{
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("DRILL")) {
            String arg = line.substring(5);
            arg.trim();
            if (arg.length() == 0 || (!isDigit(arg[0]) && arg[0] != '-')) {
                Serial.print("ERR bad command: ");
                Serial.println(line);
                Serial.println("HINT use: DRILL 50  or  DRILL -50  or  DRILL 0");
            } else {
                applyDrill(arg.toInt());
            }
        } else {
            Serial.println("Unknown command. Use: DRILL 50");
        }
    }
}
