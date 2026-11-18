// HX711 standalone test
// DT  -> Teensy pin 8
// SCK -> Teensy pin 27

#include "HX711.h"

HX711 scale;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("HX711 test starting...");
    scale.begin(8, 27);

    Serial.print("HX711 is_ready: ");
    Serial.println(scale.is_ready() ? "YES" : "NO");

    if (scale.is_ready()) {
        Serial.println("Taring...");
        scale.tare();
        Serial.println("Tare done. Reading every 500ms.");
    } else {
        Serial.println("HX711 not detected. Check wiring.");
    }
}

void loop() {
    if (scale.is_ready()) {
        long raw = scale.read();
        Serial.print("RAW: ");
        Serial.println(raw);
    } else {
        Serial.println("NOT READY");
    }
    delay(500);
}
