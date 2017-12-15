#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include <Adafruit_NeoPixel.h>

#define PIN 7                 // フルカラーLEDのPIN番号
#define NUMPIXELS 6           // LEDの個数
#define INTERRUPT_PIN 2       // use pin 2 on Arduino Uno & most boards

MPU6050 mpu;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

bool dmpReady = false;        // set true if DMP init was successful
uint8_t mpuIntStatus;         // holds actual interrupt status byte from MPU
uint8_t devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;          // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;           // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];       // FIFO storage buffer
Quaternion q;                 // [w, x, y, z]         quaternion container
VectorFloat gravity;          // [x, y, z]            gravity vector
VectorInt16 gyro;             // [x, y, z]            角速度センサの測定値
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pixels.begin();
  pixels.setBrightness(40);
}

void loop() {
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetGyro(&gyro, fifoBuffer);
  Serial.print("yow:"); Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("  pitch:"); Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("  roll:"); Serial.print(ypr[2] * 180 / M_PI);
  Serial.print("  gyroZ:"); Serial.println(gyro.x);

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, 255, 255, 255);
  }
  pixels.show();
  Serial.println("LED Brightness");
}