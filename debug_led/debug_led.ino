/* デバッグ内容
  - フルカラーLEDが「白・赤・緑・青」に変化する。
*/

#include <Adafruit_NeoPixel.h>

#define PIN 7                 // フルカラーLEDのPIN番号
#define NUMPIXELS 6           // LEDの個数

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pixels.begin();
  pixels.setBrightness(40);
}

void loop() {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 255, 255, 255);
  }
  pixels.show();
  delay(1000);
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 255, 0, 0);
  }
  pixels.show();
  delay(1000);
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 0, 255, 0);
  }
  pixels.show();
  delay(1000);
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 0, 0, 255);
  }
  pixels.show();
  delay(1000);
}
