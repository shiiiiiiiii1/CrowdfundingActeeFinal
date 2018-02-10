/* ライブラリのインクルード */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include <Adafruit_NeoPixel.h>

/* 定数宣言 */
#define PIN 7                 // フルカラーLEDのPIN番号
#define NUMPIXELS 6           // LEDの個数
#define ARRAY_LENGTH 100      // いくつ分のデータを取得するか
#define INTERRUPT_PIN 2       // use pin 2 on Arduino Uno & most boards

/* 加速度・LEDのインスタンス生成 */
MPU6050 mpu;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/* グローバル変数の宣言 */
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
// 色に関する変数
char color_have;              // 持ってる時の色
char color_fly;               // 飛んでる時の色
char color_count;             // 連続で何回同じ色が出せたか
// 回転情報に関する変数
char rotation;                // 回転方向を格納
char rotation_count;          // 回転した回数を格納する変数
// 角度に関する変数
int angle_yaw;
float slope[ARRAY_LENGTH];    // ARRAY_LENGTH 分の平均を出すための配列

bool flying;                  // 今飛んでるかどうかの変数
int loop_count;               // 持っている時に変化させるためのカウントをする変数


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
  // Serial.begin(115200);
  // while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  setupActee();   // 追記セットアップ
}


void loop() {
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  /* ヨーピッチロールを取得 */
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetGyro(&gyro, fifoBuffer);

  int angle_yaw_pre = angle_yaw;
  angle_yaw = (int) (ypr[0] * 180/M_PI);
  float pitch = (ypr[1] * 180/M_PI);
  float roll = (ypr[2] * 180/M_PI);
  for (int i = 1; i < ARRAY_LENGTH; i++) {
    slope[i] = slope[i-1];
  }
  slope[0] = sqrt(pow(pitch, 2) + pow(roll, 2));

  char rotation_state = 0;
  if (gyro.z < 0) {          // 時計回り
    rotation_state = -1;
  } else if (0 < gyro.z) {   // 反時計回り
    rotation_state = 1;
  }
  bool flew = flying;
  flying = stateActee(angle_yaw_pre, angle_yaw, rotation_state);   // 投げたかどうかの測定


  /* ----- 持っている時の処理 ----- */
  if (!flew && !flying) {
    loop_count = changeColor(color_have, loop_count);
  }

  /* ----- 投げた時の処理 ----- */
  if (!flew && flying) {
    float sum = 0;
    for (int i = 0; i < ARRAY_LENGTH; i++) {
      sum += slope[i];
    }
    float ave = sum / ARRAY_LENGTH;
    // 回転速度とブレ具合の測定。「color_fly」に代入。
    int gyro_z = gyro.z;
    char color_fly_pre = color_fly;
    color_fly = colorSelect(abs(gyro_z), ave);
    // 何回連続で投げられたかのカウントで「color_have」に代入。
    color_have = effectSelect(color_fly_pre, color_fly);
    changeColor(color_fly, 0);
    // Serial.print("throw -------------------- GyroZ"); Serial.print(abs(gyro.z)); Serial.print("  /  SlopeAverage:"); Serial.print(ave); Serial.println(" --------------------");
    // Serial.print("having led : "); Serial.print(int(color_have)); Serial.print("\t flying led : "); Serial.println(int(color_fly));
  }

  /* ----- 飛んでいる時の処理 ----- */
  if (flew && flying) {
    changeColor(color_fly, 0);
  }

  /* ----- キャッチした時の処理 ----- */
  mpu.resetFIFO();
  if (flew && !flying) {
    if (0 <= color_have) {
      loop_count = 0;
    } else {
      loop_count = 1;
    }
    loop_count = changeColor(color_have, loop_count);
    // Serial.println("catch -------------------- ");
  }

}


/*
 セットアップ関数
 - 飛んでる判定値の初期化
 - 回転方向の変数を初期化
    - 1：時計回り
    - 0：回ってない
    - -1：反時計回り
 - 一回前の回転方向の変数を初期化
 - 色変化変数の初期化
 - 連続で投げられた回数の変数の初期化
 - 一回前の角度の変数の初期化
 - LED光らせるために渡す変数の初期化
 - 起動色の表示
 @ 引数：なし
 @ 戻り値：なし
*/
void setupActee() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  flying = false;
  rotation = 0;
  color_have = 0;
  color_fly = 0;
  color_count = 0;
  loop_count = 0;
  pixels.begin();
  pixels.setBrightness(40);
  pixels.setPixelColor(0, 255, 0, 0);
  pixels.show();
  delay(100);
  pixels.setPixelColor(1, 255, 255, 0);
  pixels.show();
  delay(100);
  pixels.setPixelColor(2, 0, 255, 0);
  pixels.show();
  delay(100);
  pixels.setPixelColor(3, 0, 255, 255);
  pixels.show();
  delay(100);
  pixels.setPixelColor(4, 0, 0, 255);
  pixels.show();
  delay(100);
  pixels.setPixelColor(5, 255, 0, 255);
  pixels.show();
  delay(1000);
  changeColor(color_have, 0);
}


/*
 フリスビーが飛んだかどうかを判定するための関数
 - 前の角度と今の角度の差を計算
 - 現在角度をbefore変数に格納
 - 一定の値が続いたら回転したと判断
 - flying変数に回転状態を格納
 - どっち回転かを測定
 - rotation変数に回転方向を格納
 - 一定回数同じ方向に回転したら
 @ 引数：一個前の回転角度 現在の回転角度
 @ 戻り値：飛んでたらtrue、飛んでなかったらfalse
*/
bool stateActee (int ang_pre, int ang_now, char state) {
  #define NUM_CONTINUITY 12   // 連続で投げられる回数
  char rotation_pre = rotation;
  int ang_diff = ang_now - ang_pre;
  /* 動いていない場合の処理 */
  if ( (-6 <= ang_diff && ang_diff <= 6) || state == 0) {
    rotation = 0;
    rotation_count = 0;
    return false;
  }
  switch (state) {
    case -1:   // 時計回り
      rotation = 1;
      break;
    case 1:   // 反時計回り
      rotation = -1;
      break;
  }

  /* 前回と同じ方向に回ってない
     - 飛んでる時：前回と違う方向だったので、キャッチした
     - 持ってる時：同じ方向に回ってないので、回転はしてない
  */
  if (rotation_pre != rotation) {
    rotation_count = 0;
    return false;
  }

  /* 前回と同じ方向に回ってる
     - 飛んでる時：まだ飛んでいると判定
     - 持ってる時：回ってはいるので投げられたかどうかの審査をする -> 連続で NUM_CONTINUITY 分投げられたら投げたと判定
  */
  if (flying) {
    return true;
  } else {
    rotation_count ++;
    if (rotation_count == NUM_CONTINUITY) {
      return true;
    }
    return false;
  }
}


/*
 Acteeの色変化のための関数
 @ 引数１：色を指定するための変数(color_type)
    - -6：全色連続
    - -5：青3回連続
    - -4：緑3回連続
    - -3：黄色3回連続
    - -2：オレンジ3回連続
    - -1：赤ピンク3回連続
    - 0 ：スタンダードの色
    - 1 ：評価での赤ピンク
    - 2 ：評価でのオレンジ
    - 3 ：評価での黄色
    - 4 ：評価での緑
    - 5 ：評価での青色
 @ 引数２：何回目か数える変数(loop_count)
 @ 戻り値：今何カウント目か
*/
int changeColor(char color_type, int _loop_count) {
  #define MAX_COUNT 300
  int count_con;
  if (1 <=_loop_count && _loop_count <= MAX_COUNT/6) {
    count_con = 1;
  } else if (MAX_COUNT/6 < _loop_count && _loop_count <= MAX_COUNT/6*2) {
    count_con = 2;
  } else if (MAX_COUNT/6*2 < _loop_count && _loop_count <= MAX_COUNT/6*3) {
    count_con = 3;
  } else if (MAX_COUNT/6*3 < _loop_count && _loop_count <= MAX_COUNT/6*4) {
    count_con = 4;
  } else if (MAX_COUNT/6*4 < _loop_count && _loop_count <= MAX_COUNT/6*5) {
    count_con = 5;
  } else if (MAX_COUNT/6*5 < _loop_count) {
    count_con = 6;
  }
  switch(color_type) {
    case 0:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 100, 100, 100);
      }
      break;
    case 1:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 50, 60);
      }
      break;
    case 2:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 80, 0);
      }
      break;
    case 3:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 251, 211, 28);
      }
      break;
    case 4:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 56, 151, 35);
      }
      break;
    case 5:
      for (int i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 10, 255, 255);
      }
      break;
    case -1:   // ------------------------------ 赤3連続 ------------------------------
      switch(count_con) {
        case 1: case 3: case 5:
          for (int i = 0; i < NUMPIXELS; i++) {
            if (i % 2 == 0) {
              pixels.setPixelColor(i, 255, 50, 60);
            } else {
              pixels.setPixelColor(i, 0, 0, 0);
            }
          }
          break;
        case 2: case 4: case 6:
          for (int i = 0; i < NUMPIXELS; i++) {
            if (i % 2 == 0) {
              pixels.setPixelColor(i, 0, 0, 0);
            } else {
              pixels.setPixelColor(i, 255, 50, 60);
            }
          }
          break;
      }
      break;
    case -2:   // ------------------------------ オレンジ3連続 ------------------------------
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i != int(count_con-1)) {
          pixels.setPixelColor(i, 0, 0, 0);
        } else {
          pixels.setPixelColor(i, 255, 80, 0);
        }
      }
      break;
    case -3:   // ------------------------------ 黄3連続 ------------------------------
      int j;
      switch (count_con) {
        case 1: j = 0; break;
        case 2: j = 2; break;
        case 3: j = 4; break;
        case 4: j = 1; break;
        case 5: j = 3; break;
        case 6: j = 5; break;
      }
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i != j) {
          pixels.setPixelColor(i, 0, 0, 0);
        } else {
          pixels.setPixelColor(i, 255, 255, 0);
        }
      }
      break;
    case -4:   // ------------------------------ 緑3連続 ------------------------------
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i < int(count_con)) {
          pixels.setPixelColor(i, 0, 255, 0);
        } else {
          pixels.setPixelColor(i, 0, 0, 0);
        }
      }
      break;
    case -5:   // ------------------------------ 青3連続 ------------------------------
      for (int i = 0; i < NUMPIXELS; i++) {
        if (i == int(count_con-1)) {
          if (i != 5) {
            pixels.setPixelColor(i, 20, 20, 255);
            pixels.setPixelColor(i+1, 20, 20, 255);
            i++;
          } else {
            pixels.setPixelColor(i, 20, 20, 255);
            pixels.setPixelColor(0, 20, 20, 255);
          }
        } else {
          pixels.setPixelColor(i, 10, 255, 255);
        }
      }
     break;
    case -6:   // ------------------------------ シークレット ------------------------------
      pixels.setPixelColor(0, 255, 0, 0);
      pixels.setPixelColor(1, 255, 255, 0);
      pixels.setPixelColor(2, 0, 255, 0);
      pixels.setPixelColor(3, 0, 255, 255);
      pixels.setPixelColor(4, 0, 0, 255);
      pixels.setPixelColor(5, 255, 0, 255);
     break;
  }
  pixels.show();
  if (_loop_count == 0) {
    return 0;
  } else if (0 < _loop_count && _loop_count < MAX_COUNT) {
    _loop_count++;
  } else {
    _loop_count = 1;
  }
  return _loop_count;
}


/*
 飛んでいる間に変化する色を決定するための関数
 - 回転速度とActeeのブレ具合を[0, 1, 2]の値に変換
 - 変換した値によって判定し、戻り値を決定する
 @ 引数：回転速度　Acteeのブレ
 @ 戻り値：飛んでいる間の色の数字
*/
char colorSelect(float gyro, float slope) {
  char return_val;
  int int_gyro, int_slope;
  float min_gyro = 500;   // gyroの最小値
  float max_gyro = 1000;   // gyroの最大値
  float min_slope = 25;     // slopeの最小値
  float max_slope = 40;     // slopeの最大値
  if (gyro < min_gyro) {
    int_gyro = 0;
  } else if (min_gyro <= gyro && gyro <= max_gyro) {
    int_gyro = 1;
  } else {
    int_gyro = 2;
  }
  if (slope < min_slope) {
    int_slope = 2;
  } else if (min_slope <= slope && slope <= max_slope) {
    int_slope = 1;
  } else {
    int_slope = 0;
  }
  if ((int_gyro==0 && int_slope==0) || (int_gyro==1 && int_slope==0)) {   // ピンク
    return_val = 1;
  }
  if ((int_gyro==0 && int_slope==1) || (int_gyro==2 && int_slope==0)) {   // オレンジ
    return_val = 2;
  }
  if ((int_gyro==0 && int_slope==2) || (int_gyro==1 && int_slope==1)) {   // 黄色
    return_val = 3;
  }
  if ((int_gyro==1 && int_slope==2) || (int_gyro==2 && int_slope==1)) {   // 緑
    return_val = 4;
  }
  if (int_gyro==2 && int_slope==2) {                                      // 青
    return_val = 5;
  }
  return return_val;
}


/*
 何回連続で同じ色を投げられたかを判定するための関数
 - ランダムでシークレットにする
 @ 引数：前回の投げられた色　今回の投げられた色
 @ 戻り値：持っている間に変化する色
*/
char effectSelect(char before, char now) {
  if (before == now) {
    color_count += 1;
  } else {
    color_count = 0;
    if (int(random(30)) == 0) {
      return -6;
    }
  }
  if (color_count == 2) {
    color_count = 0;
    return -1 * now;
  }
  return 0;
}
