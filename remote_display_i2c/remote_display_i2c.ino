#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

void setup() {
  Serial.begin(38400);
    lcd.begin(16, 2);
    lcd.setRGB(colorR, colorG, colorB);
    lcd.noCursor();
    lcd.print("Starting...");
}

void loop() {
  if (Serial.available() > 0) {
    String serialData = Serial.readStringUntil('\n');
    updateDisplay(serialData);
  }
}

void updateDisplay(String serialData) {
  if (serialData.startsWith("L1>")) {
    lcd.setCursor(0, 0);
    lcd.print(serialData.substring(3)); // Print the content after "Line1:"
  } else if (serialData.startsWith("L2>")) {
    lcd.setCursor(0, 1);
    lcd.print(serialData.substring(3)); // Print the content after "Line2:"
  } else if (serialData.equals("C>")) {
    lcd.clear();
  }
}
