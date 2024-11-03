#ifndef ESP_I2S_MIX_H_
#define ESP_I2S_MIX_H_
// ESP32 I2S Mix
// (C)2024 bekki.jp

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>

#include <cmath>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

/// I2Sサウンドミックス再生用クラス
class I2SSoundMix {
 private:
  static constexpr std::string_view TASK_NAME = "I2S SoundMix";
  static constexpr int32_t CORE_ID = APP_CPU_NUM;

  static constexpr size_t I2S_DMA_SAMPLING = 480;
  static constexpr size_t I2S_DMA_BUFFER_LEN = I2S_DMA_SAMPLING * 2;
  static constexpr float VOLUME_DEFAULT = 0.8f;

 public:
  struct SoundInfo {
   public:
    SoundInfo(const uint8_t *const data, size_t length)
        : cur(0), data(data), length(length) {}

    int cur;
    const uint8_t *data;
    size_t length;
  };

 public:
  I2SSoundMix(gpio_num_t bclk_no, gpio_num_t ws_no, gpio_num_t dout_no,
              uint32_t sample_rate_hz)
      : bclk_no_(bclk_no),
        ws_no_(ws_no),
        dout_no_(dout_no),
        sample_rate_hz_(sample_rate_hz),
        i2s_handle_(nullptr),
        sound_buffer_(nullptr),
        sound_list_(),
        volume_(VOLUME_DEFAULT),
        thread_(nullptr),
        mtx_() {
    sound_buffer_ = std::make_unique<std::uint8_t[]>(I2S_DMA_BUFFER_LEN);
  }

  ~I2SSoundMix() {
    i2s_channel_disable(i2s_handle_);
    i2s_del_channel(i2s_handle_);
  }

 public:
  /// 実行
  void Run() {
    Setup();

    // std::threadで指定したのコアを利用するよう対応
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = std::string(TASK_NAME).c_str();
    cfg.pin_to_core = CORE_ID;
    esp_pthread_set_cfg(&cfg);

    thread_ = std::make_unique<std::thread>([&] {
      while (true) {
        Update();
      }
    });
  }

  /// 再生するサウンド情報を追加
  void Play(SoundInfo sound_info) {
    std::lock_guard<std::mutex> lock(mtx_);
    sound_list_.emplace_back(sound_info);
  }

 private:
  void Setup() {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_AUTO,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear = false,
        .intr_priority = 0,
    };

    i2s_new_channel(&chan_cfg, &i2s_handle_, NULL);

    i2s_std_config_t std_cfg = {
        .clk_cfg =
            {
                .sample_rate_hz = sample_rate_hz_,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            },
        .slot_cfg =
            {
                .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_MONO,
                .slot_mask = I2S_STD_SLOT_LEFT,
                .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
                .ws_pol = false,
                .bit_shift = true,
                .msb_right = true,
            },
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = bclk_no_,
                .ws = ws_no_,
                .dout = dout_no_,
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    i2s_channel_init_std_mode(i2s_handle_, &std_cfg);

    i2s_channel_enable(i2s_handle_);
  }

  /// sound_bufferに再生用データを書き込む
  void Update() {
    std::memset(sound_buffer_.get(), 0, I2S_DMA_BUFFER_LEN);

    {
      std::lock_guard<std::mutex> lock(mtx_);
      for (std::list<SoundInfo>::iterator iter = sound_list_.begin();
           iter != sound_list_.end();) {
        if (PcmDataMix(&(*iter))) {
          ++iter;
        } else {
          // ESP_LOGI(TAG, "Erase cur:%d length:%u", iter->cur, iter->length);
          iter = sound_list_.erase(iter);
        }
      }
    }

    i2s_channel_write(i2s_handle_, sound_buffer_.get(), I2S_DMA_BUFFER_LEN,
                      nullptr, portMAX_DELAY);
  }

  /// サウンドバッファにPCMデータをミックスする
  bool PcmDataMix(SoundInfo *const sound_info) {
    for (size_t frame = 0; frame < I2S_DMA_SAMPLING; ++frame) {
      if (sound_info->length <= sound_info->cur + 1) {
        return false;  // 該当のSoundInfoを破棄する
      }

      int16_t base = sound_buffer_[frame * 2] | sound_buffer_[frame * 2 + 1]
                                                    << 8;

      const int16_t add = (*(sound_info->data + sound_info->cur)) |
                          (*(sound_info->data + sound_info->cur + 1) << 8);

      // データを加算し、INT16の範囲でクリップ(計算はint32_tで行う)
      const int32_t temp =
          std::max(static_cast<int32_t>(INT16_MIN),
                   std::min(static_cast<int32_t>(INT16_MAX),
                            base + static_cast<int32_t>(add * volume_)));

      base = static_cast<int16_t>(temp);

      sound_buffer_[frame * 2] = base & 0xff;
      sound_buffer_[frame * 2 + 1] = (base >> 8) & 0xff;

      sound_info->cur += 2;
    }
    return true;
  }

 private:
  gpio_num_t bclk_no_;  /// BCLK pin number (SCK)
  gpio_num_t ws_no_;    /// Word Select pin number (LRCLK/FS/SYNC)
  gpio_num_t
      dout_no_;  /// Data Out pin number (SD/SDATA/SDIN/SDOUT/DACDAT/ADCDAT)
  uint32_t sample_rate_hz_;                  /// Sample Rate(hz)
  i2s_chan_handle_t i2s_handle_;             /// I2C Handle
  std::unique_ptr<uint8_t[]> sound_buffer_;  /// Sound Buffer
  std::list<SoundInfo> sound_list_;          // Play Sounds List
  float volume_;                             /// Volume
  std::unique_ptr<std::thread> thread_;      /// thread
  mutable std::mutex mtx_;                   /// Mutex
};

#endif  // ESP_I2S_MIX_H_
