#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// EEPROMに書き込むための処理
#include <EEPROM.h>
#include <rom_adv.h>
rom_adv rom(1023);


const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

int off_az, off_gx, off_gy, off_gz;

void ForceHeader() {
  LinesOut = 99;
}

void GetSmoothed() {
  int RawValue[6];
  long Sums[6];
  for (int i = iAx; i <= iGz; i++) {
    Sums[i] = 0;
  }

  for (int i = 1; i <= N; i++) { // get sums
    accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
    for (int j = iAx; j <= iGz; j++)
      Sums[j] = Sums[j] + RawValue[j];
  } // get sums
  for (int i = iAx; i <= iGz; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N ;
  }
} // GetSmoothed

void Initialize() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
} // Initialize

void SetOffsets(int TheOffsets[6]) {
  accelgyro.setXAccelOffset(TheOffsets [iAx]);
  accelgyro.setYAccelOffset(TheOffsets [iAy]);
  accelgyro.setZAccelOffset(TheOffsets [iAz]);
  accelgyro.setXGyroOffset (TheOffsets [iGx]);
  accelgyro.setYGyroOffset (TheOffsets [iGy]);
  accelgyro.setZGyroOffset (TheOffsets [iGz]);
} // SetOffsets

bool offset_HR(int low, int high, int i) {
  if (low == i) {
    return false;
  } else if (high == i) {
    return true;
  } else {
    return false;
  }
}

void ShowProgress() {
  if (LinesOut >= LinesBetweenHeaders) { // show header
    Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  } // show header
  for (int i = iAx; i <= iGz; i++) {
    Serial.print(LBRACKET);
    Serial.print(LowOffset[i]);
    Serial.print(COMMA);
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(COMMA);
    Serial.print(HighValue[i]);
    if (i == iGz) {
      Serial.println(RBRACKET);
    } else {
      Serial.print("]\t");
    }
    bool b;
    b = offset_HR(LowValue[i], HighValue[i], 0);
    switch (i) {
      case iAz:
        b = offset_HR(LowValue[i], HighValue[i], 16384);
        if (b) {
          off_az = HighOffset[i];
        } else {
          off_az = LowOffset[i];
        }
        break;
      case iGx:
        if (b) {
          off_gx = HighOffset[i];
        } else {
          off_gx = LowOffset[i];
        }
        break;
      case iGy:
        if (b) {
          off_gy = HighOffset[i];
        } else {
          off_gy = LowOffset[i];
        }
        break;
      case iGz:
        if (b) {
          off_gz = HighOffset[i];
        } else {
          off_gz = LowOffset[i];
        }
        break;
      default:
        break;
    }
  }
  LinesOut++;
} // ShowProgress

void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  SetOffsets(HighOffset);
  GetSmoothed();
  for (int i = iAx; i <= iGz; i++) {
    HighValue[i] = Smoothed[i]; // needed for ShowProgress
  }

  while (!Done) {
    Done = true;
    SetOffsets(LowOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) { // got low values
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i])      {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } else {
        NextLowOffset[i] = LowOffset[i];
      }
    } // got low values
    ShowProgress();
    for (int i = iAx; i <= iGz; i++) {
      LowOffset[i] = NextLowOffset[i]; // had to wait until ShowProgress done
    }
  } // keep going

  Done = false;
  while (!Done) {
    Done = true;
    SetOffsets(HighOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) { // got high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } else {
        NextHighOffset[i] = HighOffset[i];
      }
    } // got high values
    ShowProgress();
    for (int i = iAx; i <= iGz; i++) {
      HighOffset[i] = NextHighOffset[i]; // had to wait until ShowProgress done
    }
  } // keep going
} // PullBracketOut

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("averaging ");
  Serial.print(N);
  Serial.println(" readings each time");
} // SetAveraging

void setup() {
  boolean StillWorking;
  int NewOffset[6];
  boolean AllBracketsNarrow;

  Initialize();
  for (int i = iAx; i <= iGz; i++) { // set targets and initial guesses
    Target[i] = 0; // must fix for ZAccel
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  } // set targets and initial guesses
  Target[iAz] = 16384;
  SetAveraging(NFast);

  Serial.println("expanding:");
  ForceHeader();
  PullBracketsOut();

  Serial.println("\nclosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } else {
      AllBracketsNarrow = true;  // tentative
    }
    for (int i = iAx; i <= iGz; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } else { // binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } // binary search
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++) { // closing in
      if (Smoothed[i] > Target[i]) { // use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } // use lower half
      else
      { // use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } // use upper half
    } // closing in
    ShowProgress();
  } // still working
  Serial.println("-------------- offset value --------------");
  Serial.println();
  Serial.println("-------------- copying undertext and paste to 'final.ino' --------------");
  Serial.println(); Serial.println();
  Serial.print("  mpu.setXGyroOffset("); Serial.print(off_gx); Serial.println(");");
  Serial.print("  mpu.setYGyroOffset("); Serial.print(off_gy); Serial.println(");");
  Serial.print("  mpu.setZGyroOffset("); Serial.print(off_gz); Serial.println(");");
  Serial.print("  mpu.setZAccelOffset("); Serial.print(off_az); Serial.println(");");
  Serial.println(); Serial.println();
  Serial.println("-------------- done --------------");
} // setup

void loop() {
} // loop
