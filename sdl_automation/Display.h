#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

constexpr int LCD_UPDATE_INTERVAL_MS = 300;
unsigned long lcd_last_update = 0;

class Display {
  private:
    bool display_last_loading_dot = false;

  public:
    void write(bool loading, float laser_value, float weight_value);

};

void Display::write(bool loading, float laser_value, float weight_value) {
  float laser_value_threshold = 0.2; // percentage
  float weight_value_threshold = 3; // grams

  unsigned long current_time = millis();
  if (current_time - lcd_last_update > LCD_UPDATE_INTERVAL_MS) {
    Serial1.print("C>");
    Serial1.print("\n");
    Serial1.print("L1>");
    if (loading) {
      String loading_char = display_last_loading_dot == true ? " " : ";";
      Serial1.print(loading_char);
    }
    if (abs(0 - laser_value) < laser_value_threshold) {
      laser_value = 0;
    }
    Serial1.print("Laser: ");
    Serial1.print(laser_value, 1);
    Serial1.print("%\n");
    Serial1.print("L2>");
    if (loading) {
      String loading_char = display_last_loading_dot == true ? " " : ";";
      Serial1.print(loading_char);
      display_last_loading_dot = !display_last_loading_dot;
    }
    Serial1.print("Force: ");
    Serial1.print(weight_value, 1);
    Serial1.print("g\n");
    lcd_last_update = current_time;
    }
}

#endif
