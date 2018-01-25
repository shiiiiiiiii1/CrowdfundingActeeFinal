void setup() {
  Serial.begin(115200);
  Serial.println("Program start !!!");
}

void loop() {
  Serial.println("Serial print in loop");
  delay(1000);
}