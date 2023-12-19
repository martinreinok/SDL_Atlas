#ifndef FORCE_H
#define FORCE_H

#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

constexpr int WEIGHT_CLOCK = 3;
constexpr int WEIGHT_DATA = 2;
constexpr int FORCE_DATA = 2;
constexpr int FORCE_CLOCK = 3;
constexpr int TARE_ADDRESS = 0;
constexpr int CALIBRATION_ADDRESS = 4;
long weight_tare;
float weight_calibration;
float weight_latest_value;

HX711_ADC LoadCell(WEIGHT_DATA, WEIGHT_CLOCK);

class ForceSensor {
  private:

  public:
    float read();
    void initialize();
};

float ForceSensor::read() {
    static boolean newDataReady = 0;
    if (LoadCell.update()) { newDataReady = true; }
    if (newDataReady) {
        weight_latest_value = LoadCell.getData();
        newDataReady = 0;
    }
    return weight_latest_value;
}

void ForceSensor::initialize() {
    LoadCell.begin();
    EEPROM.get(TARE_ADDRESS, weight_tare);
    EEPROM.get(CALIBRATION_ADDRESS, weight_calibration);
    LoadCell.setTareOffset(weight_tare);
    LoadCell.setCalFactor(weight_calibration);
    LoadCell.start(2000, false);
}

#endif
