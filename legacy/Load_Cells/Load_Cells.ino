#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);




//const int loadcell1_data = D5;
//const int loadcell1_clock = D6;
const int loadcell2_data = D5;
const int loadcell2_clock = D6;


//HX711 scale1;
HX711 scale2;
float calib = 1209.65;

void setup() {
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); 

  display.setTextSize(3);display.setCursor(0,0); display.print("Ready!");
  display.display();

  //scale1.begin(loadcell1_data, loadcell1_clock);
  //scale1.tare();
 // scale1.set_scale(calib);
  
  scale2.begin(loadcell2_data, loadcell2_clock);
  scale2.tare();
  scale2.set_scale(calib);
}


void loop() {
  if (scale2.is_ready()) {
    //float weight1 = scale1.get_units();
    float weight2 = scale2.get_units();
    //float total = weight1 + weight2;
    display.clearDisplay();
    display.setTextSize(2); display.setCursor(0,0); display.print("Weight: ");
    display.setTextSize(2); display.setCursor(0,20); display.print(weight2);display.print(" g");
    display.display();  
  }  
  else {
    Serial.println("HX711 not found. Check your wiring!");


  }
  delay(2000);

}
