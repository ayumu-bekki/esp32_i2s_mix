// ESP32 I2S Mix
// (C)2024 bekki.jp
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>
#include <memory>

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
static const char *TAG = "I2S MIX";

#include "bomb.h"
#include "gpio_input_watch_task.h"
#include "i2s_mix.h"

constexpr gpio_num_t I2S_BCLK_PIN_NO = GPIO_NUM_27;  /// BCLK pin number (SCK)
constexpr gpio_num_t I2S_WS_PIN_NO =
    GPIO_NUM_14;  /// LRCLK pin number (LRCLK/FS/SYNC)
constexpr gpio_num_t I2S_DOUT_PIN_NO =
    GPIO_NUM_26;  /// Data Out pin number (SD/SDATA/SDIN/SDOUT/DACDAT/ADCDAT)

constexpr uint32_t I2S_SAMPLE_RATE_HZ =
    44100u;  // Sample Rate(Hz) 44100 = 44.1kHz

/// Sign 440Hz Sample
std::unique_ptr<uint8_t[]> sign_pcm = nullptr;
constexpr size_t sign_pcm_len = I2S_SAMPLE_RATE_HZ;

std::unique_ptr<uint8_t[]> CreateSignWave(uint32_t frame_length) {
  constexpr float sample_rate =
      static_cast<float>(I2S_SAMPLE_RATE_HZ);  /// サンプルレート（Hz）
  constexpr float sign_rate = 440.0f;          /// 正弦波の周波数（Hz）
  constexpr float volume = 0.1f;
  constexpr float PI = 3.141592653f;

  std::unique_ptr<uint8_t[]> pcm_buffer =
      std::make_unique<uint8_t[]>(frame_length * 2);

  for (int frame = 0; frame < frame_length; ++frame) {
    const int16_t data = std::sin(2.0f * PI * frame * sign_rate / sample_rate) *
                         static_cast<float>(INT16_MAX) * volume;
    pcm_buffer[frame * 2] = data & 0xff;
    pcm_buffer[frame * 2 + 1] = (data >> 8) & 0xff;
  }

  return pcm_buffer;
}

/// I2S再生マネージャー
std::shared_ptr<I2SSoundMix> i2s = nullptr;

/// ボタンを押した時
void OnButton() {
  ESP_LOGI(TAG, "On");

  if (i2s) {
    // 指定したPCMデータを再生リストに追加
    // i2s->Play(I2SSoundMix::SoundInfo(bomb_pcm, bomb_pcm_len)); // PCMデータ
    i2s->Play(I2SSoundMix::SoundInfo(sign_pcm.get(),
                                     sign_pcm_len * 2));  // 440Hz正弦波
  }
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "Start");

  // GPIO入力をウォッチ
  constexpr gpio_num_t INPUT_BUTTON_GPIO_NO = GPIO_NUM_13;
  GpioInputWatchTask gpio_watcher_;
  gpio_watcher_.AddMonitor(
      GpioInputWatchTask::GpioInfo(INPUT_BUTTON_GPIO_NO, OnButton, nullptr),
      GpioInputWatchTask::PULL_UP_REGISTOR_ENABLE);
  gpio_watcher_.Start();

  // 440Hz 正弦波 PCMデータ生成
  sign_pcm = CreateSignWave(sign_pcm_len);

  // I2S再生マネージャーを生成し実行開始
  i2s = std::make_shared<I2SSoundMix>(I2S_BCLK_PIN_NO, I2S_WS_PIN_NO,
                                      I2S_DOUT_PIN_NO, I2S_SAMPLE_RATE_HZ);
  i2s->Run();

  while (true) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, 1000 / portTICK_PERIOD_MS);
  }
}

// EOF
