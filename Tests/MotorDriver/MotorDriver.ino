#define STEPS_PER_REVOLUTION 200
#define STEPPER_DRIVER_ENABLED 4
#define STEPPER_ANALOG_SPEED A2
#define DIRECTION_PIN 2
#define MOTOR_FORWARD_BUTTON 13
#define MOTOR_BACKWARD_BUTTON 12

void setup() {
  pinMode(MOTOR_FORWARD_BUTTON, INPUT_PULLUP);
  pinMode(MOTOR_BACKWARD_BUTTON, INPUT_PULLUP);
  pinMode(STEPPER_DRIVER_ENABLED, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  Serial.begin(9600);
  digitalWrite(STEPPER_DRIVER_ENABLED, LOW);
  digitalWrite(DIRECTION_PIN, LOW);
}

void loop() {
  byte move_forward = digitalRead(MOTOR_FORWARD_BUTTON);
  byte move_backward = digitalRead(MOTOR_BACKWARD_BUTTON);
  while (move_backward) {
    analogWrite(ledPin, fadeValue);
    move_backward = digitalRead(MOTOR_BACKWARD_BUTTON);
  }
  
}