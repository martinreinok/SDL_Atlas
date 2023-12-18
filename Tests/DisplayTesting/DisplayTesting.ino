void setup() {
  Serial1.begin(38400);
}

void loop() {
  long laser_value = 200;
  long weight_value = -40000;
  Serial1.print("C>");
  Serial1.print("\n");
  
  Serial1.print("L1>");
  Serial1.print("Laser: ");
  Serial1.print(laser_value);
  Serial1.print("\n");

  Serial1.print("L2>");
  Serial1.print("Force: ");
  Serial1.print(weight_value);
  Serial1.print("\n");
  delay(200);
}
