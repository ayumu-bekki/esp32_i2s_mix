# esp32_i2s_mix


## WavファイルをC++向けヘッダーファイルにする

```bash
WAV_FILE_NAME=test.wav
name=`basename $WAV_FILE_NAME. wav`
sox $file -t raw ${name}.pcm
xxd --include ${name}.pcm | sed 's/unsigned char/constexpr uint8_t/' | sed 's/unsigned int/constexpr size_t/' > ${name}.h
```

