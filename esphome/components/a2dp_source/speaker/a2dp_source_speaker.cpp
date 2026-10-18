#include "a2dp_source_speaker.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <iterator>

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/core/log.h"

#include <esp_a2dp_legacy_api.h>
#include <esp_bt_main.h>
#include <esp_timer.h>

namespace esphome::a2dp_source {

static const char *const TAG = "a2dp_source";
static A2DPSourceSpeaker *active_speaker = nullptr;
static constexpr size_t PCM_BUFFER_BYTES = 8192;

void A2DPSourceSpeaker::set_address(const char *address) {
  unsigned int octets[6];
  if (std::sscanf(address, "%02x:%02x:%02x:%02x:%02x:%02x", &octets[0], &octets[1], &octets[2], &octets[3],
                  &octets[4], &octets[5]) == 6) {
    for (size_t i = 0; i < 6; i++)
      this->address_[i] = static_cast<uint8_t>(octets[i]);
  }
}

void A2DPSourceSpeaker::setup() {
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 2, 44100);
  this->volume_ = 0.02f;
}

void A2DPSourceSpeaker::loop() {
  if (!esp32_ble::global_ble->is_active() || esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
    this->connected_.store(false);
    this->streaming_.store(false);
    this->profile_ready_.store(false);
    this->connect_pending_ = false;
    this->disconnect_pending_ = false;
    this->media_check_pending_ = false;
    this->media_start_pending_ = false;
    this->initialized_ = false;
    this->next_connect_attempt_ms_ = 0;
    return;
  }

  if (!this->initialized_) {
    active_speaker = this;
    esp_err_t err = esp_bt_gap_register_callback(A2DPSourceSpeaker::gap_event_);
    if (err == ESP_OK) {
      uint8_t iocap = ESP_BT_IO_CAP_NONE;
      err = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    }
    if (err == ESP_OK)
      err = esp_a2d_source_init();
    if (err == ESP_OK)
      err = esp_a2d_register_callback(A2DPSourceSpeaker::a2dp_event_);
    if (err == ESP_OK)
      err = esp_a2d_source_register_data_callback(A2DPSourceSpeaker::audio_data_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "A2DP init failed: %s", esp_err_to_name(err));
      this->mark_failed();
      return;
    }
    this->initialized_ = true;
    ESP_LOGI(TAG, "A2DP source initialized");
  }

  if (!this->profile_ready_.load()) {
    esp_a2d_profile_status_t status{};
    if (esp_a2d_get_profile_status(&status) == ESP_OK && status.a2d_src_inited)
      this->profile_ready_.store(true);
  }
  if (!this->profile_ready_.load())
    return;

  if (this->requested_.load())
    this->state_ = this->streaming_.load() ? speaker::STATE_RUNNING : speaker::STATE_STARTING;

  const uint32_t now = millis();
  if (this->connect_pending_ && static_cast<int32_t>(now - this->next_connect_attempt_ms_) >= 0) {
    ESP_LOGW(TAG, "A2DP connection timed out");
    this->connect_pending_ = false;
    esp_a2d_source_disconnect(this->address_);
    this->next_connect_attempt_ms_ = now + 5000;
  }
  if (this->media_check_pending_ && static_cast<int32_t>(now - this->next_media_check_ms_) >= 0)
    this->media_check_pending_ = false;
  if (this->media_start_pending_ && static_cast<int32_t>(now - this->next_media_check_ms_) >= 0)
    this->media_start_pending_ = false;
  if (this->requested_.load()) {
    if (!this->connected_.load() && !this->connect_pending_ &&
        static_cast<int32_t>(now - this->next_connect_attempt_ms_) >= 0) {
      esp_err_t err = esp_a2d_source_connect(this->address_);
      this->next_connect_attempt_ms_ = now + 15000;
      if (err == ESP_OK)
        this->connect_pending_ = true;
      else
        ESP_LOGW(TAG, "A2DP connect failed: %s", esp_err_to_name(err));
    } else if (this->connected_.load() && !this->streaming_.load() && !this->media_check_pending_ &&
               !this->media_start_pending_ && static_cast<int32_t>(now - this->next_media_check_ms_) >= 0) {
      if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) == ESP_OK)
        this->media_check_pending_ = true;
      this->next_media_check_ms_ = now + 1000;
    }
  } else if (this->connected_.load() && !this->disconnect_pending_) {
    if (this->streaming_.load())
      esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    if (esp_a2d_source_disconnect(this->address_) == ESP_OK)
      this->disconnect_pending_ = true;
  }

  if (this->finish_pending_ && !this->has_buffered_data())
    this->stop();
}

void A2DPSourceSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG, "A2DP Source Speaker:");
  ESP_LOGCONFIG(TAG, "  Connected: %s", YESNO(this->connected_.load()));
  ESP_LOGCONFIG(TAG, "  Streaming: %s", YESNO(this->streaming_.load()));
  ESP_LOGCONFIG(TAG, "  PCM bytes: %" PRIu32, this->pcm_bytes_.load());
  ESP_LOGCONFIG(TAG, "  Underrun bytes: %" PRIu32, this->underrun_bytes_.load());
}

void A2DPSourceSpeaker::set_volume(float volume) {
  this->volume_ = std::clamp(volume, 0.0f, 1.0f);
  this->gain_q15_.store(static_cast<uint16_t>(this->volume_ * 32768.0f));
}

void A2DPSourceSpeaker::set_mute_state(bool mute_state) {
  this->mute_state_ = mute_state;
  this->muted_.store(mute_state);
}

void A2DPSourceSpeaker::start() {
  if (std::all_of(std::begin(this->address_), std::end(this->address_), [](uint8_t octet) { return octet == 0; })) {
    ESP_LOGW(TAG, "Speaker address is not configured");
    this->state_ = speaker::STATE_STOPPED;
    return;
  }
  if (!this->pcm_buffer_) {
    this->pcm_buffer_ = ring_buffer::RingBuffer::create(PCM_BUFFER_BYTES);
    if (!this->pcm_buffer_) {
      ESP_LOGE(TAG, "Unable to allocate PCM buffer");
      return;
    }
  }
  this->pcm_buffer_->reset();
  this->finish_pending_ = false;
  this->received_first_data_.store(false);
  this->previous_callback_us_ = 0;
  this->requested_.store(true);
  this->state_ = speaker::STATE_STARTING;
}

void A2DPSourceSpeaker::stop() {
  this->requested_.store(false);
  this->finish_pending_ = false;
  this->state_ = speaker::STATE_STOPPED;
}

void A2DPSourceSpeaker::finish() { this->finish_pending_ = true; }

bool A2DPSourceSpeaker::has_buffered_data() const {
  return this->pcm_buffer_ && this->pcm_buffer_->available() != 0;
}

size_t A2DPSourceSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  if (!this->requested_.load())
    this->start();
  if (!this->pcm_buffer_)
    return 0;
  size_t written = this->pcm_buffer_->write_without_replacement(data, length, ticks_to_wait);
  if (written)
    this->received_first_data_.store(true);
  return written;
}

void A2DPSourceSpeaker::a2dp_event_(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  if (active_speaker)
    active_speaker->process_event_(event, param);
}

void A2DPSourceSpeaker::gap_event_(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  if (!active_speaker)
    return;
  auto *speaker = active_speaker;
  if (event == ESP_BT_GAP_AUTH_CMPL_EVT) {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
      ESP_LOGI(TAG, "Bluetooth Classic speaker authenticated");
    else
      ESP_LOGW(TAG, "Bluetooth Classic authentication failed: %d", param->auth_cmpl.stat);
  } else if (event == ESP_BT_GAP_CFM_REQ_EVT) {
    bool expected = std::memcmp(param->cfm_req.bda, speaker->address_, 6) == 0;
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, expected);
  } else if (event == ESP_BT_GAP_PIN_REQ_EVT) {
    bool expected = std::memcmp(param->pin_req.bda, speaker->address_, 6) == 0;
    esp_bt_pin_code_t pin{};
    if (expected) {
      pin[0] = '0';
      pin[1] = '0';
      pin[2] = '0';
      pin[3] = '0';
    }
    esp_bt_gap_pin_reply(param->pin_req.bda, expected, expected ? 4 : 0, pin);
  }
}

void A2DPSourceSpeaker::process_event_(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  if (event == ESP_A2D_PROF_STATE_EVT) {
    this->profile_ready_.store(param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS);
  } else if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    bool connected = param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED;
    this->connected_.store(connected);
    this->connect_pending_ = false;
    this->disconnect_pending_ = false;
    if (!connected) {
      this->streaming_.store(false);
      this->media_check_pending_ = false;
      this->media_start_pending_ = false;
    }
  } else if (event == ESP_A2D_AUDIO_STATE_EVT) {
    bool streaming = param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED;
    this->streaming_.store(streaming);
  } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
    if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
      this->media_check_pending_ = false;
      if (param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS && this->requested_.load()) {
        if (esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START) == ESP_OK)
          this->media_start_pending_ = true;
      }
    } else if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START) {
      this->media_start_pending_ = false;
    }
  }
}

int32_t A2DPSourceSpeaker::audio_data_(uint8_t *data, int32_t length) {
  if (!active_speaker || !data || length <= 0)
    return 0;
  auto *speaker = active_speaker;
  speaker->callback_count_.fetch_add(1);
  const int64_t now_us = esp_timer_get_time();
  if (speaker->previous_callback_us_ > 0) {
    const int64_t gap = now_us - speaker->previous_callback_us_;
    if (gap > 0 && gap < UINT32_MAX) {
      uint32_t previous_max = speaker->max_callback_gap_us_.load();
      while (static_cast<uint32_t>(gap) > previous_max &&
             !speaker->max_callback_gap_us_.compare_exchange_weak(previous_max, static_cast<uint32_t>(gap))) {
      }
    }
  }
  speaker->previous_callback_us_ = now_us;
  size_t bytes = speaker->requested_.load() && speaker->pcm_buffer_ ? speaker->pcm_buffer_->read(data, length) : 0;
  if (bytes < static_cast<size_t>(length)) {
    std::memset(data + bytes, 0, length - bytes);
    // The ESP-IDF callback can continue while playback is finishing and after
    // it has stopped. Those intentional silence frames are not underruns.
    if (speaker->requested_.load() && !speaker->finish_pending_.load() && speaker->received_first_data_.load()) {
      speaker->underrun_bytes_.fetch_add(length - bytes);
      speaker->underrun_events_.fetch_add(1);
    }
  }
  speaker->pcm_bytes_.fetch_add(bytes);
  if (bytes != 0)
    speaker->audio_output_callback_(bytes / 4, now_us);
  const int32_t gain_q15 = speaker->muted_.load() ? 0 : speaker->gain_q15_.load();
  auto *samples = reinterpret_cast<int16_t *>(data);
  for (int32_t i = 0; i < length / 2; i++)
    samples[i] = static_cast<int16_t>((static_cast<int32_t>(samples[i]) * gain_q15) >> 15);
  return length;
}

}  // namespace esphome::a2dp_source

#endif  // USE_ESP32
