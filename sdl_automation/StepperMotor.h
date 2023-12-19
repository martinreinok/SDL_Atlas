#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <AccelStepper.h>
constexpr int STEPPER_PULSE_PIN = 10;
constexpr int STEPPER_DIRECTION_PIN = 9;
constexpr int STEPPER_DRIVER_ENABLED = 8;

AccelStepper Stepper(1, STEPPER_PULSE_PIN, STEPPER_DIRECTION_PIN);

class StepperMotor {
  private:

  public:
    void set_max_speed(int speed);
    void initialize();
    void stop();
    void move_forward(int speed);
    void move_backward(int speed, bool constant_speed);
    int get_max_speed_from_laser(float laser_value);
    /* Motor speeds */
    const int ACCELERATION = 350;
    const int SPEED_MAX = 750;
    const int SPEED_FAST = 400;
    const int SPEED_MEDIUM = 100;
    const int SPEED_SLOW = 50;
    const int SPEED_VERY_SLOW = 20;

  
};

void StepperMotor::initialize() {
    pinMode(STEPPER_DRIVER_ENABLED, OUTPUT);
    pinMode(STEPPER_DIRECTION_PIN, OUTPUT);
    digitalWrite(STEPPER_DRIVER_ENABLED, LOW);
    Stepper.setMaxSpeed(SPEED_MAX);
    Stepper.setAcceleration(ACCELERATION);
}

void StepperMotor::set_max_speed(int speed) {
    Stepper.setMaxSpeed(speed);
}

void StepperMotor::stop() {
    Stepper.moveTo(Stepper.currentPosition());
    Stepper.runSpeedToPosition();
    Stepper.setCurrentPosition(0);
    Stepper.stop();
}

void StepperMotor::move_forward(int speed) {
    /*
    Moves stepper forward fast, while accelerating every time the button is pressed.
    This is useful for accurate placement and the max speed is quite high, can move fast back to start.
    */
    this->set_max_speed(speed);
    Stepper.moveTo(Stepper.currentPosition() + 10000);
    Stepper.run();
}

void StepperMotor::move_backward(int speed, bool constant_speed = false) {
    /*
    Moves stepper forward fast, while accelerating every time the button is pressed.
    This is useful for accurate placement and the max speed is quite high, can move fast back to start.
    */
    if (constant_speed) {
      Stepper.setSpeed(-speed);
      Stepper.runSpeed();
    }
    else {
      this->set_max_speed(speed);
      Stepper.moveTo(Stepper.currentPosition() - 10000);
      Stepper.run();
    }
}

int StepperMotor::get_max_speed_from_laser(float laser_value) {
  if (laser_value < 4.00) {
    return this->SPEED_SLOW;
  }
  else {
    return this->SPEED_MAX;
  }

}

#endif
