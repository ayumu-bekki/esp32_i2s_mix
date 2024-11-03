# esp32_i2s_mix

## 概要
esp-idf(C++)でI2Sを使ったサウンドミックス再生を行うためのクラス・サンプル
I2S制御用クラスとして I2SSoundMixクラス (i2s_mix.h)が用意されています。
main.ccのOnButton関数にて、プログラム中で生成した440Hz正弦波を再生しています。
WavファイルからPCMデータを取り出しての再生もサンプルとして記載しています。

I2SSoundMixクラスを開始(Run)すると音声処理用のスレッドを作成し、
任意のタイミングでPlay関数で追加されたデータを合成した上で再生します。

## 環境
ESP-IDF v5.2-37

MAX98357 I2S Class-D Mono Ampで動作確認しました。
https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp

## WavファイルをC++向けヘッダーファイルにする

soxコマンドのインストールが必要です。
先ずsoxコマンドでWavのヘッダーを除外しRaw PCMデータとして保存しています。
これは、xxd -iオプションが標準入力では効かないための対応です。
その後、xxd -i でCヘッダファイル化し、sedでC++形式に整形しています。

```bash
WAV_FILE_NAME=test.wav
name=`basename $WAV_FILE_NAME. wav`
sox $file -t raw ${name}.pcm
xxd --include ${name}.pcm | sed 's/unsigned char/constexpr uint8_t/' | sed 's/unsigned int/constexpr size_t/' > ${name}.h
rm ${name}.pcm
```

