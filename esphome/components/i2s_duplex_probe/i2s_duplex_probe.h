#pragma once

#ifdef USE_ESP32

#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/core/component.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <driver/i2s_std.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#ifdef USE_I2S_DUPLEX_AEC
#include <esp_aec.h>
#endif

namespace esphome::i2s_duplex_probe {

class I2SDuplexProbe : public Component, public Parented<i2s_audio::I2SAudioComponent> {
 public:
  void set_din_pin(int pin) { this->din_pin_ = pin; }
  void set_dout_pin(int pin) { this->dout_pin_ = pin; }
  void set_multitone_enabled(bool enabled) { this->multitone_enabled_ = enabled; }
#ifdef USE_I2S_DUPLEX_AEC
  void set_aec_enabled(bool enabled) { this->aec_enabled_ = enabled; }
#endif

  void loop() override;
  void dump_config() override;

  // Explicitly invoked by a test button. Never produces sound on boot.
  void start_test();
  bool is_running() const { return this->task_handle_ != nullptr; }

 protected:
  static void run_task_(void *argument);
#ifdef USE_I2S_DUPLEX_AEC
  static void aec_task_(void *argument);
#endif
  static bool IRAM_ATTR on_rx_overflow_(i2s_chan_handle_t channel, i2s_event_data_t *event, void *argument);
  static bool IRAM_ATTR on_tx_overflow_(i2s_chan_handle_t channel, i2s_event_data_t *event, void *argument);
  void release_channels_();

  int din_pin_{I2S_GPIO_UNUSED};
  int dout_pin_{I2S_GPIO_UNUSED};
  bool multitone_enabled_{false};
  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  bool tx_enabled_{false};
  bool rx_enabled_{false};
  TaskHandle_t task_handle_{nullptr};
  std::atomic<bool> completed_{false};
  std::atomic<uint32_t> rx_overflows_{0};
  std::atomic<uint32_t> tx_overflows_{0};
  bool parent_locked_{false};
  uint32_t received_bytes_{0};
  uint32_t receive_errors_{0};
  uint32_t send_errors_{0};
  uint32_t microphone_energy_{0};
  int64_t tone_sin_[2]{0, 0};
  int64_t tone_cos_[2]{0, 0};
  uint64_t mic_square_[2]{0, 0};
  uint32_t mic_peak_[2]{0, 0};
  uint32_t mic_near_clip_[2]{0, 0};
  uint32_t mic_frames_{0};
#ifdef USE_I2S_DUPLEX_AEC
  bool aec_enabled_{false};
  aec_handle_t *aec_handle_{nullptr};
  int16_t *aec_mic_{nullptr};
  int16_t *aec_ref_{nullptr};
  int16_t *aec_out_{nullptr};
  size_t aec_chunk_size_{0};
  uint32_t aec_frames_{0};
  uint32_t aec_max_us_{0};
  int64_t aec_raw_sin_{0};
  int64_t aec_raw_cos_{0};
  int64_t aec_clean_sin_{0};
  int64_t aec_clean_cos_{0};
  uint32_t aec_samples_{0};
  struct alignas(16) AECFrame {
    alignas(16) int16_t mic[512];
    alignas(16) int16_t ref[512];
  };
  QueueHandle_t aec_queue_{nullptr};
  TaskHandle_t aec_task_handle_{nullptr};
  std::atomic<bool> aec_done_{false};
  uint32_t aec_queue_drops_{0};
  uint32_t aec_max_queue_{0};
  uint64_t aec_total_us_{0};
  uint64_t aec_raw_square_{0};
  uint64_t aec_clean_square_{0};
  int64_t aec_multitone_raw_sin_[4]{0, 0, 0, 0};
  int64_t aec_multitone_raw_cos_[4]{0, 0, 0, 0};
  int64_t aec_multitone_clean_sin_[4]{0, 0, 0, 0};
  int64_t aec_multitone_clean_cos_[4]{0, 0, 0, 0};
#endif
};

}  // namespace esphome::i2s_duplex_probe

#endif  // USE_ESP32
