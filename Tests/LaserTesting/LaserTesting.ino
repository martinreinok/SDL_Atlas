constexpr int LASER_ANALOG_STARTING_POSITION = 330; // 10-bit analog value, corresponds to the laser measurement at 76mm (CLAMP_LENGTH), when textile has just been placed in the clamp.
constexpr long LASER_MAX_RANGE = 280000; // micrometer
constexpr long LASER_MIN_RANGE = 120000; // micrometer

void setup() {
  Serial.begin(38400);
}

void loop() {
  int laser_value = analogRead(A1);
  delay(100);  // delay in between reads for stability
  Serial.print("Analog : ");
  Serial.println(laser_value);
  Serial.println(convert_to_percentage(laser_value));
}

float convert_to_percentage(int analog) {
  int analog_at_starting_position = 330;
  int mm_difference_from_start = convert_laser_measurement_to_mm(analog_at_starting_position) - convert_laser_measurement_to_mm(analog);
  float percentage = (mm_difference_from_start + 76) * 100 / 76;
  return mm_difference_from_start;
}

int convert_laser_measurement_to_mm(int analog_value) {
  long measurement_micrometer = LASER_MAX_RANGE + analog_value * (LASER_MIN_RANGE - LASER_MAX_RANGE) / 1030;
  int measurement_mm = measurement_micrometer / 1000;
  return measurement_mm;
}
