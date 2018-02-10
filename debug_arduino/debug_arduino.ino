/* デバッグ内容
  - ArduinoProMiniに付属しているLEDが一度点灯したあと消える。
  - シリアルモニタに「Program start !!!」が一度表示されたあと「Serial print in loop」がずっと表示される。
*/

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(115200);
  Serial.println("Program start !!!");
}

void loop() {
  Serial.println("Serial print in loop");
  delay(1000);
}
