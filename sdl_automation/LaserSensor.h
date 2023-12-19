#ifndef LASER_H
#define LASER_H

#include <Arduino.h>

constexpr int CLAMP_LENGTH = 76;
constexpr int LASER_ANALOG_STARTING_POSITION = 332; // 10-bit analog value, corresponds to the laser measurement at 76mm (CLAMP_LENGTH), when textile has just been placed in the clamp.
constexpr int LASER_MAX_RANGE = 280; // mm
constexpr int LASER_MIN_RANGE = 120; // mm
constexpr int LASER_PIN = 1;

class LaserSensor {
  private:
    float measurement_to_mm(int analog_value);

  public:
    float read();
};

float LaserSensor::read() {
  /*
  Laser range: 120mm to 280mm
  Corresponds: 5V to 0V
  Out of range: 5V / 120mm
  */
  int laser_analog = analogRead(LASER_PIN);
  float mm_difference_from_start = measurement_to_mm(LASER_ANALOG_STARTING_POSITION) - measurement_to_mm(laser_analog);
  float percentage_extension = ((float)mm_difference_from_start + (float)CLAMP_LENGTH) * 100 / (float)CLAMP_LENGTH;
  return percentage_extension - 100.00;
}

float LaserSensor::measurement_to_mm(int analog_value) {
  float measurement_mm = (float)LASER_MAX_RANGE + (float)analog_value * ((float)LASER_MIN_RANGE - (float)LASER_MAX_RANGE) / 1030;
  return measurement_mm;
}

#endif
