#include "i2s_duplex_probe.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#ifdef USE_I2S_DUPLEX_AEC
#include <esp_heap_caps.h>
#include <esp_timer.h>
#endif

namespace esphome::i2s_duplex_probe {

static const char *const TAG = "i2s_duplex_probe";
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr size_t FRAMES_PER_BLOCK = 160;  // 10 ms stereo PCM block
static constexpr size_t BLOCK_COUNT = 200;       // 2 seconds maximum sound
static constexpr std::array<uint16_t, 4> MULTITONE_FREQUENCIES{{300, 700, 1300, 2500}};

void I2SDuplexProbe::dump_config() {
  ESP_LOGCONFIG(TAG, "Experimental I2S full-duplex probe:");
  ESP_LOGCONFIG(TAG, "  DIN: GPIO%d", this->din_pin_);
  ESP_LOGCONFIG(TAG, "  DOUT: GPIO%d", this->dout_pin_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %lu Hz; test requires explicit start", static_cast<unsigned long>(SAMPLE_RATE));
}

void I2SDuplexProbe::start_test() {
  if (this->task_handle_ != nullptr || this->tx_handle_ != nullptr || this->rx_handle_ != nullptr) {
    ESP_LOGW(TAG, "Test already active");
    return;
  }
  if (!this->parent_->try_lock()) {
    ESP_LOGW(TAG, "I2S bus is busy");
    return;
  }
  this->parent_locked_ = true;

  i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(this->parent_->get_port(), I2S_ROLE_MASTER);
  channel_config.dma_desc_num = 4;
  channel_config.dma_frame_num = 256;
  channel_config.auto_clear = true;
  esp_err_t err = i2s_new_channel(&channel_config, &this->tx_handle_, &this->rx_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Paired channel allocation failed: %s", esp_err_to_name(err));
    this->release_channels_();
    return;
  }

  i2s_std_gpio_config_t pins = this->parent_->get_pin_config();
  pins.din = static_cast<gpio_num_t>(this->din_pin_);
  pins.dout = static_cast<gpio_num_t>(this->dout_pin_);
  i2s_std_config_t config = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = pins,
  };
  err = i2s_channel_init_std_mode(this->tx_handle_, &config);
  if (err == ESP_OK)
    err = i2s_channel_init_std_mode(this->rx_handle_, &config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Paired channel initialization failed: %s", esp_err_to_name(err));
    this->release_channels_();
    return;
  }
  const i2s_event_callbacks_t rx_callbacks = {.on_recv_q_ovf = &I2SDuplexProbe::on_rx_overflow_};
  const i2s_event_callbacks_t tx_callbacks = {.on_send_q_ovf = &I2SDuplexProbe::on_tx_overflow_};
  err = i2s_channel_register_event_callback(this->rx_handle_, &rx_callbacks, this);
  if (err == ESP_OK)
    err = i2s_channel_register_event_callback(this->tx_handle_, &tx_callbacks, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Paired channel callback registration failed: %s", esp_err_to_name(err));
    this->release_channels_();
    return;
  }
#ifdef USE_I2S_DUPLEX_AEC
  if (this->aec_enabled_) {
    const size_t internal_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t psram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    this->aec_handle_ = aec_create(SAMPLE_RATE, 4, 1, AEC_MODE_SR_LOW_COST);
    if (this->aec_handle_ == nullptr) {
      ESP_LOGE(TAG, "AEC allocation failed");
      this->release_channels_();
      return;
    }
    this->aec_chunk_size_ = static_cast<size_t>(aec_get_chunksize(this->aec_handle_));
    if (this->aec_chunk_size_ != 512) {
      ESP_LOGE(TAG, "Unexpected AEC frame size: %u", static_cast<unsigned>(this->aec_chunk_size_));
      this->release_channels_();
      return;
    }
    const size_t buffer_size = this->aec_chunk_size_ * sizeof(int16_t);
    this->aec_mic_ = static_cast<int16_t *>(heap_caps_aligned_alloc(16, buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    this->aec_ref_ = static_cast<int16_t *>(heap_caps_aligned_alloc(16, buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    this->aec_out_ = static_cast<int16_t *>(heap_caps_aligned_alloc(16, buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (this->aec_mic_ == nullptr || this->aec_ref_ == nullptr || this->aec_out_ == nullptr) {
      ESP_LOGE(TAG, "AEC frame buffer allocation failed");
      this->release_channels_();
      return;
    }
    this->aec_queue_ = xQueueCreate(4, sizeof(AECFrame));
    if (this->aec_queue_ == nullptr) {
      ESP_LOGE(TAG, "AEC input queue allocation failed");
      this->release_channels_();
      return;
    }
    ESP_LOGI(TAG, "AEC allocated: %u samples per frame; internal RAM delta=%u B, PSRAM delta=%u B",
             static_cast<unsigned>(this->aec_chunk_size_),
             static_cast<unsigned>(internal_before - heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(psram_before - heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    this->aec_done_.store(false);
    if (xTaskCreatePinnedToCore(&I2SDuplexProbe::aec_task_, "i2s_duplex_aec", 12288, this, 4,
                                &this->aec_task_handle_, 0) != pdPASS) {
      ESP_LOGE(TAG, "Could not start AEC worker");
      this->aec_task_handle_ = nullptr;
      this->release_channels_();
      return;
    }
  }
#endif
  err = i2s_channel_enable(this->tx_handle_);
  if (err == ESP_OK) {
    this->tx_enabled_ = true;
    err = i2s_channel_enable(this->rx_handle_);
    if (err == ESP_OK)
      this->rx_enabled_ = true;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Paired channel enable failed: %s", esp_err_to_name(err));
    this->release_channels_();
    return;
  }

  this->received_bytes_ = 0;
  this->receive_errors_ = 0;
  this->send_errors_ = 0;
  this->microphone_energy_ = 0;
  this->mic_frames_ = 0;
  for (size_t channel = 0; channel < 2; ++channel) {
    this->tone_sin_[channel] = 0;
    this->tone_cos_[channel] = 0;
    this->mic_square_[channel] = 0;
    this->mic_peak_[channel] = 0;
    this->mic_near_clip_[channel] = 0;
  }
  this->rx_overflows_.store(0);
  this->tx_overflows_.store(0);
#ifdef USE_I2S_DUPLEX_AEC
  this->aec_frames_ = 0;
  this->aec_max_us_ = 0;
  this->aec_raw_sin_ = 0;
  this->aec_raw_cos_ = 0;
  this->aec_clean_sin_ = 0;
  this->aec_clean_cos_ = 0;
  this->aec_samples_ = 0;
  this->aec_queue_drops_ = 0;
  this->aec_max_queue_ = 0;
  this->aec_total_us_ = 0;
  this->aec_raw_square_ = 0;
  this->aec_clean_square_ = 0;
  for (size_t tone = 0; tone < MULTITONE_FREQUENCIES.size(); ++tone) {
    this->aec_multitone_raw_sin_[tone] = 0;
    this->aec_multitone_raw_cos_[tone] = 0;
    this->aec_multitone_clean_sin_[tone] = 0;
    this->aec_multitone_clean_cos_[tone] = 0;
  }
#endif
  this->completed_.store(false);
  if (xTaskCreatePinnedToCore(&I2SDuplexProbe::run_task_, "i2s_duplex_probe", 8192, this, 5,
                              &this->task_handle_, 1) != pdPASS) {
    ESP_LOGE(TAG, "Could not start probe task");
    this->task_handle_ = nullptr;
    this->release_channels_();
    return;
  }
  ESP_LOGI(TAG, "Started 16 kHz paired RX/TX test");
}

void I2SDuplexProbe::run_task_(void *argument) {
  auto *self = static_cast<I2SDuplexProbe *>(argument);
  std::array<int16_t, FRAMES_PER_BLOCK * 2> output{};
  std::array<int16_t, FRAMES_PER_BLOCK * 2> input{};
  std::array<int16_t, 40> basis_sin{};
  std::array<int16_t, 40> basis_cos{};
  // Every frequency has an integer number of periods per block, so the
  // quiet diagnostic waveform has no discontinuity at block boundaries.
  // The sum of four 31-PCM tones is hard limited to 124 PCM (0.38% FS).
  for (size_t frame = 0; frame < FRAMES_PER_BLOCK; ++frame) {
    const float time = static_cast<float>(frame) / SAMPLE_RATE;
    float value = 500.0f * sinf(2.0f * 3.14159265f * 400.0f * time);
    if (self->multitone_enabled_) {
      value = 31.0f * (sinf(2.0f * 3.14159265f * 300.0f * time) +
                        sinf(2.0f * 3.14159265f * 700.0f * time) +
                        sinf(2.0f * 3.14159265f * 1300.0f * time) +
                        sinf(2.0f * 3.14159265f * 2500.0f * time));
    }
    int16_t sample = static_cast<int16_t>(value);
    output[2 * frame] = sample;
    output[2 * frame + 1] = sample;
  }
  for (size_t frame = 0; frame < basis_sin.size(); ++frame) {
    const float angle = 2.0f * 3.14159265f * static_cast<float>(frame) / basis_sin.size();
    basis_sin[frame] = static_cast<int16_t>(32767.0f * sinf(angle));
    basis_cos[frame] = static_cast<int16_t>(32767.0f * cosf(angle));
  }
#ifdef USE_I2S_DUPLEX_AEC
  size_t aec_fill = 0;
#endif

  for (size_t block = 0; block < BLOCK_COUNT; ++block) {
    size_t written = 0;
    esp_err_t err = i2s_channel_write(self->tx_handle_, output.data(), sizeof(output), &written, 200);
    if (err != ESP_OK || written != sizeof(output)) {
      self->send_errors_++;
      break;
    }

    // TX write is DMA-paced. Drain RX data accumulated during that interval.
    size_t received = 0;
    err = i2s_channel_read(self->rx_handle_, input.data(), sizeof(input), &received, 0);
    if (err == ESP_OK && received > 0) {
      self->received_bytes_ += received;
      for (size_t frame = 0; frame < received / (2 * sizeof(int16_t)); ++frame) {
        const size_t phase = (self->mic_frames_ + frame) % basis_sin.size();
        for (size_t channel = 0; channel < 2; ++channel) {
          const int32_t sample = input[2 * frame + channel];
          const uint32_t magnitude = static_cast<uint32_t>(std::abs(sample));
          self->mic_peak_[channel] = std::max(self->mic_peak_[channel], magnitude);
          if (magnitude >= 30000)
            ++self->mic_near_clip_[channel];
          self->microphone_energy_ += magnitude;
          self->mic_square_[channel] += static_cast<uint64_t>(sample * sample);
          self->tone_sin_[channel] += static_cast<int64_t>(sample) * basis_sin[phase];
          self->tone_cos_[channel] += static_cast<int64_t>(sample) * basis_cos[phase];
        }
#ifdef USE_I2S_DUPLEX_AEC
        if (self->aec_handle_ != nullptr) {
          self->aec_mic_[aec_fill] = input[2 * frame];
          // The diagnostic tone repeats exactly. Scale its reference by the measured
          // acoustic/ADC gain (~6x) so the AEC input and reference have similar levels.
          // A general backend must tap the actual post-volume PCM and calibrate this gain.
          self->aec_ref_[aec_fill] = 6 * output[2 * ((self->mic_frames_ + frame) % FRAMES_PER_BLOCK)];
          ++aec_fill;
          if (aec_fill == self->aec_chunk_size_) {
            AECFrame aec_frame;
            memcpy(aec_frame.mic, self->aec_mic_, sizeof(aec_frame.mic));
            memcpy(aec_frame.ref, self->aec_ref_, sizeof(aec_frame.ref));
            if (xQueueSend(self->aec_queue_, &aec_frame, 0) != pdTRUE)
              ++self->aec_queue_drops_;
            self->aec_max_queue_ = std::max(self->aec_max_queue_,
                                            static_cast<uint32_t>(uxQueueMessagesWaiting(self->aec_queue_)));
            aec_fill = 0;
          }
        }
#endif
      }
      self->mic_frames_ += received / (2 * sizeof(int16_t));
    } else if (err != ESP_ERR_TIMEOUT) {
      self->receive_errors_++;
    }
  }
  self->completed_.store(true);
  vTaskDelete(nullptr);
}

#ifdef USE_I2S_DUPLEX_AEC
void I2SDuplexProbe::aec_task_(void *argument) {
  auto *self = static_cast<I2SDuplexProbe *>(argument);
  AECFrame frame;
  std::array<int16_t, 40> basis_sin{};
  std::array<int16_t, 40> basis_cos{};
  std::array<std::array<int16_t, FRAMES_PER_BLOCK>, 4> multitone_sin{};
  std::array<std::array<int16_t, FRAMES_PER_BLOCK>, 4> multitone_cos{};
  for (size_t phase = 0; phase < basis_sin.size(); ++phase) {
    const float angle = 2.0f * 3.14159265f * static_cast<float>(phase) / basis_sin.size();
    basis_sin[phase] = static_cast<int16_t>(32767.0f * sinf(angle));
    basis_cos[phase] = static_cast<int16_t>(32767.0f * cosf(angle));
  }
  if (self->multitone_enabled_) {
    for (size_t tone = 0; tone < MULTITONE_FREQUENCIES.size(); ++tone) {
      for (size_t phase = 0; phase < FRAMES_PER_BLOCK; ++phase) {
        const float angle = 2.0f * 3.14159265f * MULTITONE_FREQUENCIES[tone] * phase / SAMPLE_RATE;
        multitone_sin[tone][phase] = static_cast<int16_t>(32767.0f * sinf(angle));
        multitone_cos[tone][phase] = static_cast<int16_t>(32767.0f * cosf(angle));
      }
    }
  }
  while (true) {
    if (xQueueReceive(self->aec_queue_, &frame, pdMS_TO_TICKS(50)) == pdTRUE) {
      const int64_t started = esp_timer_get_time();
      aec_process(self->aec_handle_, frame.mic, frame.ref, self->aec_out_);
      const uint32_t elapsed = static_cast<uint32_t>(esp_timer_get_time() - started);
      self->aec_max_us_ = std::max(self->aec_max_us_, elapsed);
      self->aec_total_us_ += elapsed;
      ++self->aec_frames_;
      // Discard the first 15 frames (480 ms) while the adaptive filter converges.
      if (self->aec_frames_ > 15) {
        for (size_t sample = 0; sample < self->aec_chunk_size_; ++sample) {
          const size_t phase = (self->aec_samples_ + sample) % basis_sin.size();
          self->aec_raw_sin_ += static_cast<int64_t>(frame.mic[sample]) * basis_sin[phase];
          self->aec_raw_cos_ += static_cast<int64_t>(frame.mic[sample]) * basis_cos[phase];
          self->aec_clean_sin_ += static_cast<int64_t>(self->aec_out_[sample]) * basis_sin[phase];
          self->aec_clean_cos_ += static_cast<int64_t>(self->aec_out_[sample]) * basis_cos[phase];
          const int32_t raw_sample = frame.mic[sample];
          const int32_t clean_sample = self->aec_out_[sample];
          self->aec_raw_square_ += static_cast<uint64_t>(raw_sample * raw_sample);
          self->aec_clean_square_ += static_cast<uint64_t>(clean_sample * clean_sample);
          if (self->multitone_enabled_) {
            const size_t multitone_phase = (self->aec_samples_ + sample) % FRAMES_PER_BLOCK;
            for (size_t tone = 0; tone < MULTITONE_FREQUENCIES.size(); ++tone) {
              self->aec_multitone_raw_sin_[tone] +=
                  static_cast<int64_t>(raw_sample) * multitone_sin[tone][multitone_phase];
              self->aec_multitone_raw_cos_[tone] +=
                  static_cast<int64_t>(raw_sample) * multitone_cos[tone][multitone_phase];
              self->aec_multitone_clean_sin_[tone] +=
                  static_cast<int64_t>(clean_sample) * multitone_sin[tone][multitone_phase];
              self->aec_multitone_clean_cos_[tone] +=
                  static_cast<int64_t>(clean_sample) * multitone_cos[tone][multitone_phase];
            }
          }
        }
        self->aec_samples_ += self->aec_chunk_size_;
      }
    } else if (self->completed_.load()) {
      break;
    }
  }
  self->aec_done_.store(true);
  vTaskDelete(nullptr);
}
#endif

bool IRAM_ATTR I2SDuplexProbe::on_rx_overflow_(i2s_chan_handle_t, i2s_event_data_t *, void *argument) {
  static_cast<I2SDuplexProbe *>(argument)->rx_overflows_.fetch_add(1);
  return false;
}

bool IRAM_ATTR I2SDuplexProbe::on_tx_overflow_(i2s_chan_handle_t, i2s_event_data_t *, void *argument) {
  static_cast<I2SDuplexProbe *>(argument)->tx_overflows_.fetch_add(1);
  return false;
}

void I2SDuplexProbe::loop() {
  if (!this->completed_.load())
    return;
#ifdef USE_I2S_DUPLEX_AEC
  if (this->aec_handle_ != nullptr && !this->aec_done_.load())
    return;
  this->aec_task_handle_ = nullptr;
#endif
  this->completed_.store(false);
  this->task_handle_ = nullptr;
  ESP_LOGI(TAG, "RX bytes=%lu, RX errors=%lu, TX errors=%lu, RX overflow=%lu, TX overflow=%lu, mic energy=%lu",
           static_cast<unsigned long>(this->received_bytes_), static_cast<unsigned long>(this->receive_errors_),
           static_cast<unsigned long>(this->send_errors_), static_cast<unsigned long>(this->rx_overflows_.load()),
           static_cast<unsigned long>(this->tx_overflows_.load()), static_cast<unsigned long>(this->microphone_energy_));
  for (size_t channel = 0; channel < 2; ++channel) {
    const double sin_sum = static_cast<double>(this->tone_sin_[channel]);
    const double cos_sum = static_cast<double>(this->tone_cos_[channel]);
    const double amplitude = this->mic_frames_ == 0
                                 ? 0.0
                                 : 2.0 * sqrt(sin_sum * sin_sum + cos_sum * cos_sum) /
                                       (static_cast<double>(this->mic_frames_) * 32767.0);
    const double rms = this->mic_frames_ == 0
                           ? 0.0
                           : sqrt(static_cast<double>(this->mic_square_[channel]) / this->mic_frames_);
    ESP_LOGI(TAG, "Mic channel %u: 400 Hz amplitude=%d PCM, RMS=%d PCM, peak=%lu PCM, near_clip=%lu",
             static_cast<unsigned>(channel), static_cast<int>(amplitude), static_cast<int>(rms),
             static_cast<unsigned long>(this->mic_peak_[channel]),
             static_cast<unsigned long>(this->mic_near_clip_[channel]));
  }
#ifdef USE_I2S_DUPLEX_AEC
  if (this->aec_handle_ != nullptr) {
    const auto tone_amplitude = [this](int64_t sin_sum, int64_t cos_sum) -> int {
      if (this->aec_samples_ == 0)
        return 0;
      const double sin_value = static_cast<double>(sin_sum);
      const double cos_value = static_cast<double>(cos_sum);
      return static_cast<int>(2.0 * sqrt(sin_value * sin_value + cos_value * cos_value) /
                              (static_cast<double>(this->aec_samples_) * 32767.0));
    };
    ESP_LOGI(TAG, "AEC frames=%lu, avg=%lu us, max=%lu us, queue max=%lu/4, dropped=%lu, 400 Hz raw=%d PCM, cleaned=%d PCM",
             static_cast<unsigned long>(this->aec_frames_),
             static_cast<unsigned long>(this->aec_frames_ == 0 ? 0 : this->aec_total_us_ / this->aec_frames_),
             static_cast<unsigned long>(this->aec_max_us_), static_cast<unsigned long>(this->aec_max_queue_),
             static_cast<unsigned long>(this->aec_queue_drops_),
             tone_amplitude(this->aec_raw_sin_, this->aec_raw_cos_),
             tone_amplitude(this->aec_clean_sin_, this->aec_clean_cos_));
    if (this->aec_samples_ > 0) {
      ESP_LOGI(TAG, "AEC signal=%s, raw RMS=%d PCM, cleaned RMS=%d PCM",
               this->multitone_enabled_ ? "four-tone" : "400 Hz",
               static_cast<int>(sqrt(static_cast<double>(this->aec_raw_square_) / this->aec_samples_)),
               static_cast<int>(sqrt(static_cast<double>(this->aec_clean_square_) / this->aec_samples_)));
    }
    if (this->multitone_enabled_) {
      for (size_t tone = 0; tone < MULTITONE_FREQUENCIES.size(); ++tone) {
        ESP_LOGI(TAG, "AEC %u Hz: raw=%d PCM, cleaned=%d PCM", MULTITONE_FREQUENCIES[tone],
                 tone_amplitude(this->aec_multitone_raw_sin_[tone], this->aec_multitone_raw_cos_[tone]),
                 tone_amplitude(this->aec_multitone_clean_sin_[tone], this->aec_multitone_clean_cos_[tone]));
      }
    }
  }
#endif
  this->release_channels_();
}

void I2SDuplexProbe::release_channels_() {
#ifdef USE_I2S_DUPLEX_AEC
  if (this->aec_task_handle_ != nullptr) {
    if (!this->aec_done_.load())
      vTaskDelete(this->aec_task_handle_);
    this->aec_task_handle_ = nullptr;
  }
  if (this->aec_queue_ != nullptr) {
    vQueueDelete(this->aec_queue_);
    this->aec_queue_ = nullptr;
  }
  if (this->aec_mic_ != nullptr) {
    heap_caps_free(this->aec_mic_);
    this->aec_mic_ = nullptr;
  }
  if (this->aec_ref_ != nullptr) {
    heap_caps_free(this->aec_ref_);
    this->aec_ref_ = nullptr;
  }
  if (this->aec_out_ != nullptr) {
    heap_caps_free(this->aec_out_);
    this->aec_out_ = nullptr;
  }
  if (this->aec_handle_ != nullptr) {
    aec_destroy(this->aec_handle_);
    this->aec_handle_ = nullptr;
  }
  this->aec_chunk_size_ = 0;
#endif
  if (this->tx_handle_ != nullptr) {
    if (this->tx_enabled_)
      i2s_channel_disable(this->tx_handle_);
    i2s_del_channel(this->tx_handle_);
    this->tx_handle_ = nullptr;
    this->tx_enabled_ = false;
  }
  if (this->rx_handle_ != nullptr) {
    if (this->rx_enabled_)
      i2s_channel_disable(this->rx_handle_);
    i2s_del_channel(this->rx_handle_);
    this->rx_handle_ = nullptr;
    this->rx_enabled_ = false;
  }
  if (this->parent_locked_) {
    this->parent_->unlock();
    this->parent_locked_ = false;
  }
}

}  // namespace esphome::i2s_duplex_probe

#endif  // USE_ESP32
