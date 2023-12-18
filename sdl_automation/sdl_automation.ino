#include <HX711_ADC.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// TODO: if laser value is really close to zero +-0.2, make it 0
// TODO: faster move back during auto

// Wiring and pins
constexpr int STEPPER_PULSE_PIN = 10;
constexpr int STEPPER_DIRECTION_PIN = 9;
constexpr int STEPPER_DRIVER_ENABLED = 8;
constexpr int MOTOR_FORWARD_BUTTON = 5;
constexpr int MOTOR_BACKWARD_BUTTON = 6;
constexpr int ROTARY_SWITCH = 7;
constexpr int LASER_PIN = 1;
constexpr int WEIGHT_DATA = 2;
constexpr int WEIGHT_CLOCK = 3;

// Calibration data
constexpr int KNOWN_MASS_FOR_CALIBRATION = 3015; // The 3kg block is not exactly 3kg
constexpr long WEIGHT_CALIBRATION_06092023_1_TARE = 8243794;
constexpr float WEIGHT_CALIBRATION_06092023_1 = 188.21;
constexpr long WEIGHT_CALIBRATION_06092023_2_TARE = 8243857;
constexpr float WEIGHT_CALIBRATION_06092023_2 = 188.28;

// EEPROM addresses for the calibration data
int TARE_ADDRESS = 0;
int CALIBRATION_ADDRESS = 4;
long weight_tare;
float weight_calibration;

// Motor speeds
constexpr int MOTOR_MAX_SPEED = 750;
constexpr int MOTOR_MAX_SPEED_CLOSE = 200;
constexpr int MOTOR_MOVE_BACKWARDS_CONSTANT_SPEED = 50;
constexpr int MOTOR_ACCELERATION = 250;

// Automatic mode limits
constexpr int AUTOMATIC_MOTOR_SLOW_SPEED = 20;
constexpr int AUTOMATIC_MOTOR_NORMAL_SPEED = 50;
constexpr int AUTOMATIC_MOTOR_MAX_SPEED_LASER = 400;
constexpr int AUTOMATIC_MOTOR_MAX_SPEED_WEIGHT = 100;
constexpr int WEIGHT_LIMIT_ABSOLUTE = 20000;
constexpr int WEIGHT_LIMIT_FORWARD = -200;
constexpr int WEIGHT_LIMIT_BACKWARD = 3000;
constexpr int BACKWARD_RANGE_LIMIT_LASER = 110; // This is percentage limit

// SDL Atlas data
constexpr int CLAMP_LENGTH = 76;

// Laser
constexpr int LASER_ANALOG_STARTING_POSITION = 332; // 10-bit analog value, corresponds to the laser measurement at 76mm (CLAMP_LENGTH), when textile has just been placed in the clamp.
constexpr int LASER_MAX_RANGE = 280; // mm
constexpr int LASER_MIN_RANGE = 120; // mm

// Display commands over Serial1
constexpr int LCD_UPDATE_INTERVAL_MS = 300;
unsigned long lcd_last_update = 0;

// Update sensorics
constexpr int SENSOR_UPDATE_INTERVAL_MS = 50;
unsigned long sensors_last_update = 0;

// Motor
AccelStepper stepper(1, STEPPER_PULSE_PIN, STEPPER_DIRECTION_PIN);
HX711_ADC LoadCell(WEIGHT_DATA, WEIGHT_CLOCK);

float laser_latest_value = 0;
float weight_latest_value = 0;
const int BUFFER_SIZE = 20;
float laser_buffer[BUFFER_SIZE];
float weight_buffer[BUFFER_SIZE];
int laser_buffer_index = 0;
int weight_buffer_index = 0;
float previous_laser_value = 0;
float previous_weight_value = 0;
bool display_last_loading_dot = false;

bool AUTOMATIC_MODE = false;
bool CALIBRATION_MODE = false;
bool entering_calibration_mode = false;
unsigned long entering_calibration_mode_timer = 0;
constexpr int CALIBRATION_ENTER_TIME = 5000; // 5 seconds both buttons held down

void setup() {
  // Initialize arrays with initial values
  for (int i = 0; i < BUFFER_SIZE; i++) {
    laser_buffer[i] = 0;
    weight_buffer[i] = 0;
  }

  pinMode(MOTOR_FORWARD_BUTTON, INPUT_PULLUP);
  pinMode(MOTOR_BACKWARD_BUTTON, INPUT_PULLUP);
  pinMode(ROTARY_SWITCH, INPUT_PULLUP);
  pinMode(STEPPER_DRIVER_ENABLED, OUTPUT);
  pinMode(STEPPER_DIRECTION_PIN, OUTPUT);
  digitalWrite(STEPPER_DRIVER_ENABLED, LOW);
  digitalWrite(STEPPER_DIRECTION_PIN, LOW);
  Serial1.begin(38400);
  stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper.setAcceleration(MOTOR_ACCELERATION);
  LoadCell.begin();
  EEPROM.get(TARE_ADDRESS, weight_tare);
  EEPROM.get(CALIBRATION_ADDRESS, weight_calibration);
  LoadCell.setTareOffset(weight_tare);
  LoadCell.setCalFactor(weight_calibration);
  LoadCell.start(2000, false);
}

void loop() {
  byte move_forward = !digitalRead(MOTOR_FORWARD_BUTTON);
  byte move_backward = !digitalRead(MOTOR_BACKWARD_BUTTON);
  AUTOMATIC_MODE = !digitalRead(ROTARY_SWITCH);

  if (!AUTOMATIC_MODE) {
    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    if (move_backward && move_forward) {
      /* Both buttons are pressed simultaneously */
      stop_stepper();

      // Record start time of holding buttons down
      if (!entering_calibration_mode) {
        entering_calibration_mode_timer = millis();
        entering_calibration_mode = true;
      }

      // Check if both buttons are held down for more than 5 seconds
      while (entering_calibration_mode && (millis() - entering_calibration_mode_timer < CALIBRATION_ENTER_TIME)) {
        // Check if buttons are released prematurely
        if (!move_backward || !move_forward) {
          entering_calibration_mode = false; // Reset the flag if either button is released
          break; // Exit the loop
        }
      }
      // Check if the buttons were held down for the required time
      if (entering_calibration_mode) {
        // Enter calibration mode
        calibrate_weight_sensor();
        entering_calibration_mode = false; // Reset the flag
      }

    }
    else if (move_forward) {
      if (laser_latest_value < 4.00) {
        stepper.setMaxSpeed(MOTOR_MAX_SPEED_CLOSE);
      }
      else {
        stepper.setMaxSpeed(MOTOR_MAX_SPEED);
      }
      move_stepper_forward();
    }
    else if (move_backward) {
      move_stepper_backward();
    }
    else {
      stop_stepper();
    }
  }

  else {
    stepper.setMaxSpeed(AUTOMATIC_MOTOR_SLOW_SPEED);
    if (move_backward && move_forward) {
      /* Both buttons are pressed simultaneously 
      This does not work :>)
      */
      stop_stepper();
    }

    else if (move_forward) {
      while(weight_latest_value > (float)WEIGHT_LIMIT_FORWARD && AUTOMATIC_MODE) {
        AUTOMATIC_MODE = !digitalRead(ROTARY_SWITCH);
        if (laser_latest_value > 4.00) {
          stepper.setMaxSpeed(AUTOMATIC_MOTOR_MAX_SPEED_LASER);
        }
        else {
          stepper.setMaxSpeed(AUTOMATIC_MOTOR_NORMAL_SPEED);
        }
        move_stepper_forward();
        read_laser();
        read_load_cell();
        update_lcd(true);
      }
      stop_stepper();
    }

    else if (move_backward) {
      while(weight_latest_value < (float)WEIGHT_LIMIT_BACKWARD && AUTOMATIC_MODE) {
        AUTOMATIC_MODE = !digitalRead(ROTARY_SWITCH);
        if (weight_latest_value < (float)WEIGHT_LIMIT_BACKWARD * 0.6) {
          stepper.setMaxSpeed(AUTOMATIC_MOTOR_MAX_SPEED_WEIGHT);
        }
        else {
          stepper.setMaxSpeed(AUTOMATIC_MOTOR_SLOW_SPEED);
        }
        stepper.moveTo(stepper.currentPosition() - 10000);
        stepper.run();
        read_laser();
        read_load_cell();
        update_lcd(true);
      }
      stop_stepper();

      // We have reached the desired force, so let's maintain it for 5 seconds.
      unsigned long startTime = millis();
      stepper.setMaxSpeed(AUTOMATIC_MOTOR_SLOW_SPEED);
      while (millis() - startTime < 8000 && AUTOMATIC_MODE) {
        AUTOMATIC_MODE = !digitalRead(ROTARY_SWITCH);
        // Adjust the motor position to maintain the desired force.
        if (weight_latest_value < (float)WEIGHT_LIMIT_BACKWARD) {
          stepper.moveTo(stepper.currentPosition() - 10000);
          stepper.run();
        }
        else {
          stop_stepper();
        }
        read_laser();
        read_load_cell();
        update_lcd(true);
      }
    }
}
  
  // Update LCD with sensor values every 0.3 second
  update_lcd(false);
  // Update sensorics every 0.1 second
  read_sensors();
}

void update_lcd(bool loading) {
  float laser_value_threshold = 0.2; // percentage
  float weight_value_threshold = 3; // grams
  unsigned long current_time = millis();
  if (current_time - lcd_last_update > LCD_UPDATE_INTERVAL_MS) {
    float laser_value = get_laser_average_measurement();
    float weight_value = get_weight_average_measurement();
    if (abs(laser_value - previous_laser_value) > laser_value_threshold || abs(weight_value - previous_weight_value) > weight_value_threshold || abs(0 - laser_value) < laser_value_threshold) {
      Serial1.print("C>"); // Clear display
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
      previous_laser_value = laser_value;
      previous_weight_value = weight_value;
    }
  }
}

void read_sensors() {
  unsigned long current_time = millis();
  if (current_time - sensors_last_update > SENSOR_UPDATE_INTERVAL_MS) {
    read_laser();
    read_load_cell();
  }
}

void calibrate_weight_sensor() {
  LoadCell.begin();
  Serial1.print("C>\n");
  Serial1.print("L1>Calibration mode \n");
  Serial1.print("L2>Rmv wght & prs W\n");
  delay(2000);

  // Wait for white button to continue
  byte white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
  while (!white_button) {
    white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
    delay(10);
  }

  Serial1.print("C>\n");
  Serial1.print("L1>Wait.. \n");

  LoadCell.update();
  LoadCell.tareNoDelay();
  delay(1000);
  long tareCalibrationOffset = LoadCell.getTareOffset();
  Serial1.print("C>\n");
  Serial1.print("L1>Tare offset: \n");
  Serial1.print("L2>");
  Serial1.print(tareCalibrationOffset);
  Serial1.print("(W)\n");
  delay(2000);

  // Wait for white button to continue
  white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
  while (!white_button) {
    white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
    delay(10);
  }

  Serial1.print("C>\n");
  Serial1.print("L1>Add 3kg weight \n");
  Serial1.print("L2>Press W\n");
  delay(2000);

  // Wait for white button to continue
  white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
  while (!white_button) {
    white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
    delay(10);
  }

  float known_mass = 3015;
  LoadCell.update();
  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

  Serial1.print("C>\n");
  Serial1.print("L1>New calibration\n");
  Serial1.print("L2>");
  Serial1.print(newCalibrationValue);
  Serial1.print("(W)\n");
  delay(2000);

  // Wait for white button to continue
  white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
  while (!white_button) {
    white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
    delay(10);
  }

  Serial1.print("C>\n");
  Serial1.print("L1>Save value?\n");
  Serial1.print("L2>yes - W | no - B\n");
  delay(2000);

  // Wait for white button to continue
  white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
  byte black_button = !digitalRead(MOTOR_FORWARD_BUTTON);
  while (!white_button || !black_button) {
    white_button = !digitalRead(MOTOR_BACKWARD_BUTTON);
    black_button = !digitalRead(MOTOR_FORWARD_BUTTON);
    delay(10);
  }
  if(white_button) {
      EEPROM.put(TARE_ADDRESS, tareCalibrationOffset);
      EEPROM.put(CALIBRATION_ADDRESS, newCalibrationValue);
      LoadCell.setTareOffset(tareCalibrationOffset);
      LoadCell.setCalFactor(newCalibrationValue);
      return;
    }
  else {
    return;
  }
}

void move_stepper_forward() {
  /*
  Moves stepper forward fast, while accelerating every time the button is pressed.
  This is useful for accurate placement and the max speed is quite high, can move fast back to start.
  */
  stepper.moveTo(stepper.currentPosition() + 10000);
  stepper.run();
}

void move_stepper_backward() {
  /*
  Moves stepper backwards at constant speed as this is where measurement are taken.
  The speed is chosen to be slow.
  */
  stepper.setSpeed(-MOTOR_MOVE_BACKWARDS_CONSTANT_SPEED);
  stepper.runSpeed();
}

void stop_stepper() {
  /*
  Stop motor immediately after letting go of button
  */
  stepper.moveTo(stepper.currentPosition());
  stepper.runSpeedToPosition();
  stepper.setCurrentPosition(0);
  stepper.stop();
}

void read_laser() {
  /*
  Laser range: 120mm to 280mm
  Corresponds: 5V to 0V
  Out of range: 5V / 120mm
  */
  int laser_analog = analogRead(LASER_PIN);
  float mm_difference_from_start = convert_laser_measurement_to_mm(LASER_ANALOG_STARTING_POSITION) - convert_laser_measurement_to_mm(laser_analog);
  float percentage_extension = ((float)mm_difference_from_start + (float)CLAMP_LENGTH) * 100 / (float)CLAMP_LENGTH;
  laser_buffer[laser_buffer_index] = percentage_extension - 100.00;
  laser_buffer_index = (laser_buffer_index + 1) % BUFFER_SIZE;
  laser_latest_value = percentage_extension - 100.00;
}

float convert_laser_measurement_to_mm(int analog_value) {
  float measurement_mm = (float)LASER_MAX_RANGE + (float)analog_value * ((float)LASER_MIN_RANGE - (float)LASER_MAX_RANGE) / 1030;
  return measurement_mm;
}

void read_load_cell() {
  static boolean newDataReady = 0;
  if (LoadCell.update()) { newDataReady = true; }
  if (newDataReady) {
    weight_latest_value = LoadCell.getData();
    newDataReady = 0;
  }
  weight_buffer[weight_buffer_index] = weight_latest_value;
  weight_buffer_index = (weight_buffer_index + 1) % BUFFER_SIZE;
}

float get_laser_average_measurement() {
  float avg_laser = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    avg_laser += laser_buffer[i];
  }
  return avg_laser /= BUFFER_SIZE;
}

float get_weight_average_measurement() {
  float avg_weight = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    avg_weight += weight_buffer[i];
  }
  return avg_weight /= BUFFER_SIZE;
}