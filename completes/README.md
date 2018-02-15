# プログラムファイルの説明
## ファイル詳細
- _offset/_offset.ino
- final/final.ino

## _offset/_offset.ino
加速度センサの個体値を識別するためのプログラム

### 使用手順
1. ArduinoIDEを立ち上げプログラムを開く。
2. 「ツール」から「ボード・プロセッサ・シリアルポート」のそれぞれを設定する。
3. プログラムを書き込む。
4. 書き込み終わったらシリアルモニタを立ち上げ、シリアルスピードを「115200」に変更する。
  - この時にシリアルモニタが `Initializing I2C devices...` で止まったらArduinoProMiniのリセットボタンを押す。
5. シリアルモニタに下記のような表示がされたら完了。
  - この時に `copying undertext and paste to 'final.ino'` の部分の下４行をコピーしておく。

```
-------------- offset value --------------

-------------- copying undertext and paste to 'final.ino' --------------


  mpu.setXGyroOffset(164);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(-86);
  mpu.setZAccelOffset(14213);


-------------- done --------------
```

## final/final.ino
Actee最終書き込みのプログラム

### 使用手順
1. 上記の 2 までは手順は一緒。
2. 先ほどのプログラムのシリアルモニタに表示されていた `copying undertext and paste to 'final.ino'` の部分の下４行を `/* paste text */` の下に上書きして貼り付ける。
3. プログラムを書き込む。

