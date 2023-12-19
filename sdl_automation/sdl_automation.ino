/*
    SDL Automation project

    This project automates a fabric extensiometer used for quality control of textiles and fabrics.
    Beforehan the extensiometer had to be winded manually, with mechanically attached 3kg weight.
    This project replaces the winding with a motor and the weight with a load cell.
    Additionally a laser is added for more accurate results, however the ruler on the device will remain for double checking.

    The system:
    * Non-captive stepper motor, laser distance sensor, load cell with hx711.
    * PLC: used for safety system, it's relays power all other devices (24V & 5V). The E-stop is connected to the PLC.
    * 24V & 5V PSU: 24V powers Laser, PLC & stepper driver. 5V powers Arduino, load cell & all buttons.
    * Arduino Nano Every: Controls the stepper 

    Created 07.09.2023
    By Martin Reinok

    https://martinreinok.com/

*/

#include "ForceSensor.h"
#include "LaserSensor.h"
#include "StepperMotor.h"
#include "Display.h"

/* Wiring and pins */
constexpr int ROTARY_SWITCH = 7;
constexpr int MOTOR_BACKWARD_BUTTON = 6;
constexpr int MOTOR_FORWARD_BUTTON = 5;

/* Load Cell */
constexpr int WEIGHT_LIMIT_ABSOLUTE = 20000; // This is load cell capacity
constexpr int WEIGHT_LIMIT_FORWARD = -200; // Motor can push maximum of 200 grams into the load cell
constexpr int WEIGHT_LIMIT_BACKWARD = 3000; // Motor must strech the textile to 3kg load

bool AUTOMATIC_MODE = false;
bool CALIBRATION_MODE = false;

LaserSensor Laser;
StepperMotor Motor;
ForceSensor Force;
Display LCD;


void setup() {
  pinMode(MOTOR_FORWARD_BUTTON, INPUT_PULLUP);
  pinMode(MOTOR_BACKWARD_BUTTON, INPUT_PULLUP);
  pinMode(ROTARY_SWITCH, INPUT_PULLUP);
  Serial1.begin(38400);
  Motor.initialize();
  Force.initialize();
}

void loop() {
  byte move_forward = !digitalRead(MOTOR_FORWARD_BUTTON);
  byte move_backward = !digitalRead(MOTOR_BACKWARD_BUTTON);
  AUTOMATIC_MODE = !digitalRead(ROTARY_SWITCH);

  if (AUTOMATIC_MODE) {

    if (move_forward) {
      float force_value = Force.read();
      float laser_value = Laser.read();
      while(force_value > (float)WEIGHT_LIMIT_FORWARD && !digitalRead(ROTARY_SWITCH)) {
        int speed = Motor.get_max_speed_from_laser(laser_value);
        Motor.move_forward(speed);
        LCD.write(true, laser_value, force_value);
      }
      Motor.stop();
      LCD.write(false, laser_value, force_value);
    }

    else if (move_backward) {
      float force_value = Force.read();
      float laser_value = Laser.read();
      while(force_value < (float)WEIGHT_LIMIT_BACKWARD && !digitalRead(ROTARY_SWITCH)) {
        if (force_value < (float)WEIGHT_LIMIT_BACKWARD * 0.6) {
          Motor.move_backward(Motor.SPEED_MEDIUM);
        }
        else {
          Motor.move_backward(Motor.SPEED_SLOW);
        }
        LCD.write(true, laser_value, force_value);
      }
      Motor.stop();
      LCD.write(true, laser_value, force_value);

      /* Maintain force at 3kg */
      int duration = 2000;
      unsigned long startTime = millis();
      Motor.set_max_speed(Motor.SPEED_SLOW);
      while (millis() - startTime < duration && !digitalRead(ROTARY_SWITCH)) {
        float force_value = Force.read();
        float laser_value = Laser.read();
        if (force_value < (float)WEIGHT_LIMIT_BACKWARD) {
          Motor.move_backward(Motor.SPEED_SLOW);
        }
        else {
          Motor.stop();
        }      
        LCD.write(true, laser_value, force_value);
      }
      LCD.write(false, laser_value, force_value);
    }
  }

  else if (!AUTOMATIC_MODE) {

    if (move_forward) {
      float force_value = Force.read();
      float laser_value = Laser.read();
      int speed = Motor.get_max_speed_from_laser(laser_value);
      Motor.move_forward(speed);
      LCD.write(false, laser_value, force_value);
    }

    else if (move_backward) {
      float force_value = Force.read();
      float laser_value = Laser.read();
      Motor.move_backward(Motor.SPEED_MEDIUM);
      LCD.write(false, laser_value, force_value);
    }

    else {
      Motor.stop();
    }
  }
}