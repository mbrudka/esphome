#pragma once

#ifdef USE_ESP32

#include <atomic>
#include <cstdint>
#include <memory>

#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"

#include <esp_a2dp_api.h>
#include <esp_gap_bt_api.h>

namespace esphome::a2dp_source {

class A2DPSourceSpeaker final : public Component, public speaker::Speaker {
 public:
  void set_address(const char *address);
  void setup() override;
  void loop() override;
  void dump_config() override;

  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }
  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  void start() override;
  void stop() override;
  void finish() override;
  bool has_buffered_data() const override;
  void set_volume(float volume) override;
  void set_mute_state(bool mute_state) override;
  uint32_t callback_count() const { return this->callback_count_.load(); }
  uint32_t pcm_bytes() const { return this->pcm_bytes_.load(); }
  uint32_t underrun_bytes() const { return this->underrun_bytes_.load(); }
  uint32_t underrun_events() const { return this->underrun_events_.load(); }
  uint32_t max_callback_gap_us() const { return this->max_callback_gap_us_.load(); }
  bool connected() const { return this->connected_.load(); }
  bool streaming() const { return this->streaming_.load(); }
  bool profile_ready() const { return this->profile_ready_.load(); }

 protected:
  static void a2dp_event_(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
  static void gap_event_(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
  static int32_t audio_data_(uint8_t *data, int32_t length);
  void process_event_(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

  uint8_t address_[6]{};
  std::unique_ptr<ring_buffer::RingBuffer> pcm_buffer_{};
  bool initialized_{false};
  std::atomic<bool> profile_ready_{false};
  std::atomic<bool> connect_pending_{false};
  std::atomic<bool> disconnect_pending_{false};
  std::atomic<bool> media_check_pending_{false};
  std::atomic<bool> media_start_pending_{false};
  std::atomic<bool> finish_pending_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<bool> requested_{false};
  std::atomic<uint32_t> callback_count_{0};
  std::atomic<uint32_t> underrun_bytes_{0};
  std::atomic<uint32_t> underrun_events_{0};
  std::atomic<uint32_t> pcm_bytes_{0};
  std::atomic<uint32_t> max_callback_gap_us_{0};
  std::atomic<uint16_t> gain_q15_{655};
  std::atomic<bool> muted_{false};
  std::atomic<bool> received_first_data_{false};
  std::atomic<int64_t> previous_callback_us_{0};
  uint32_t next_connect_attempt_ms_{0};
  uint32_t next_media_check_ms_{0};
};

}  // namespace esphome::a2dp_source

#endif  // USE_ESP32
