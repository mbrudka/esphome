#include "a2dp_wav_media_source.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <esp_http_client.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::a2dp_wav {

static const char *const TAG = "a2dp_wav";
static constexpr size_t TASK_STACK = 4096;
static constexpr uint32_t HTTP_TIMEOUT_MS = 3000;

static uint16_t le16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
static uint32_t le32(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

void A2DPWavMediaSource::setup() { this->disable_loop(); }
void A2DPWavMediaSource::dump_config() {
  ESP_LOGCONFIG(TAG, "A2DP HTTP PCM WAV source (44.1 kHz stereo, 22.05 kHz stereo or mono, 16-bit)");
}
bool A2DPWavMediaSource::can_handle(const std::string &uri) const {
  return uri.starts_with("http://") || uri.starts_with("https://");
}

bool A2DPWavMediaSource::play_uri(const std::string &uri) {
  if (!this->can_handle(uri) || !this->has_listener() || this->task_active_.load() ||
      this->get_state() != media_source::MediaSourceState::IDLE)
    return false;
  this->uri_ = uri;
  this->stop_ = false;
  this->result_ = 0;
  this->input_bytes_ = 0;
  this->output_bytes_ = 0;
  this->max_http_read_us_ = 0;
  this->task_active_ = true;
  // Wi-Fi and Bluetooth controller tasks are pinned to core 0 on the ESP32.
  if (xTaskCreatePinnedToCore(&A2DPWavMediaSource::task_entry_, "a2dp_wav", TASK_STACK, this, 2, nullptr, 1) !=
      pdPASS) {
    this->task_active_ = false;
    ESP_LOGE(TAG, "Cannot create WAV reader task");
    return false;
  }
  this->set_state_(media_source::MediaSourceState::PLAYING);
  this->enable_loop();
  return true;
}

void A2DPWavMediaSource::handle_command(media_source::MediaSourceCommand command) {
  switch (command) {
    case media_source::MediaSourceCommand::STOP:
      this->stop_ = true;
      break;
    case media_source::MediaSourceCommand::PAUSE:
      // The A2DP sink must disconnect whenever audio is not playing.
      // Treat pause as stop; a later play command starts the track again.
      this->stop_ = true;
      break;
    default:
      break;
  }
}

void A2DPWavMediaSource::loop() {
  if (this->task_active_.load())
    return;
  const int result = this->result_.load();
  if (result == 0)
    return;
  ESP_LOGI(TAG, "WAV result=%d input=%" PRIu32 " output=%" PRIu32 " max HTTP read=%" PRIu32 " us", result,
           this->input_bytes_.load(), this->output_bytes_.load(), this->max_http_read_us_.load());
  if (result < 0)
    this->set_state_(media_source::MediaSourceState::ERROR);
  // The orchestrator finishes the A2DP speaker on IDLE, including after a read error.
  this->set_state_(media_source::MediaSourceState::IDLE);
  this->disable_loop();
}

void A2DPWavMediaSource::task_entry_(void *arg) {
  auto *self = static_cast<A2DPWavMediaSource *>(arg);
  self->run_();
  self->task_active_ = false;
  vTaskDelete(nullptr);
}

void A2DPWavMediaSource::run_() {
  bool ok = false;
  esp_http_client_config_t config = {};
  config.url = this->uri_.c_str();
  config.timeout_ms = HTTP_TIMEOUT_MS;
  config.buffer_size = 1024;
  config.buffer_size_tx = 512;
  config.disable_auto_redirect = false;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGE(TAG, "HTTP client allocation failed");
    this->result_ = -1;
    return;
  }

  do {
    if (esp_http_client_open(client, 0) != ESP_OK || this->stop_)
      break;
    esp_http_client_fetch_headers(client);
    if (esp_http_client_get_status_code(client) != 200) {
      ESP_LOGE(TAG, "HTTP response %d", esp_http_client_get_status_code(client));
      break;
    }

    // Read exactly the requested bytes, including through short TCP reads.
    // Keep the final partial read: live ffmpeg WAV responses use an unknown
    // data length and may end between our 256-byte read boundaries.
    size_t last_read = 0;
    auto read_exact = [this, client, &last_read](uint8_t *dst, size_t size) -> bool {
      size_t offset = 0;
      while (offset < size && !this->stop_) {
        const int n = esp_http_client_read(client, reinterpret_cast<char *>(dst + offset), size - offset);
        if (n > 0) {
          offset += n;
        } else if (n < 0 || esp_http_client_is_complete_data_received(client)) {
          break;
        } else {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
      }
      last_read = offset;
      return offset == size;
    };
    auto skip = [&read_exact](uint32_t size) -> bool {
      uint8_t trash[128];
      while (size) {
        const size_t n = std::min<size_t>(size, sizeof(trash));
        if (!read_exact(trash, n))
          return false;
        size -= n;
      }
      return true;
    };

    uint8_t riff[12] = {};
    if (!read_exact(riff, sizeof(riff)) || memcmp(riff, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
      ESP_LOGE(TAG, "Response is not RIFF/WAVE: %02x %02x %02x %02x %02x %02x %02x %02x", riff[0], riff[1], riff[2],
               riff[3], riff[8], riff[9], riff[10], riff[11]);
      break;
    }
    bool format_ok = false;
    bool mono_22050 = false;
    bool stereo_22050 = false;
    uint32_t data_size = 0;
    for (int chunk = 0; chunk < 32 && !this->stop_; chunk++) {
      uint8_t header[8];
      if (!read_exact(header, sizeof(header)))
        break;
      const uint32_t size = le32(header + 4);
      if (memcmp(header, "fmt ", 4) == 0) {
        uint8_t fmt[16];
        if (size < sizeof(fmt) || !read_exact(fmt, sizeof(fmt)) || !skip(size - sizeof(fmt) + (size & 1)))
          break;
        const bool stereo_44100 =
            le16(fmt) == 1 && le16(fmt + 2) == 2 && le32(fmt + 4) == 44100 && le16(fmt + 14) == 16;
        mono_22050 = le16(fmt) == 1 && le16(fmt + 2) == 1 && le32(fmt + 4) == 22050 && le16(fmt + 14) == 16;
        stereo_22050 = le16(fmt) == 1 && le16(fmt + 2) == 2 && le32(fmt + 4) == 22050 && le16(fmt + 14) == 16;
        format_ok = stereo_44100 || stereo_22050 || mono_22050;
        if (!format_ok) {
          ESP_LOGE(TAG, "Only PCM16 stereo 44100 Hz, stereo 22050 Hz or mono 22050 Hz WAV is supported");
          break;
        }
      } else if (memcmp(header, "data", 4) == 0 && format_ok) {
        data_size = size;
        break;
      } else if (!skip(size + (size & 1))) {
        break;
      }
    }
    if (!format_ok || data_size == 0) {
      ESP_LOGE(TAG, "WAV data chunk missing");
      break;
    }
    const bool streaming_size = data_size == UINT32_MAX;

    const audio::AudioStreamInfo info(16, 2, 44100);
    uint8_t pcm[1024];
    uint64_t total = 0;
    uint32_t stalled_ms = 0;
    while (!this->stop_ && data_size) {
      const size_t block_size = mono_22050 ? sizeof(pcm) / 4 : stereo_22050 ? sizeof(pcm) / 2 : sizeof(pcm);
      size_t want = std::min<size_t>(block_size, data_size);
      const int64_t read_start = esp_timer_get_time();
      if (!read_exact(pcm, want)) {
        if (streaming_size && esp_http_client_is_complete_data_received(client)) {
          want = last_read;
          data_size = 0;
          if (want == 0)
            break;
        } else {
          break;
        }
      }
      const uint32_t read_us = static_cast<uint32_t>(esp_timer_get_time() - read_start);
      uint32_t previous_max = this->max_http_read_us_.load();
      while (read_us > previous_max && !this->max_http_read_us_.compare_exchange_weak(previous_max, read_us)) {
      }
      this->input_bytes_.fetch_add(want);
      if ((want & (mono_22050 ? 1 : 3)) != 0) {
        ESP_LOGE(TAG, "WAV PCM length is not frame aligned");
        break;
      }
      size_t output_size = want;
      if (mono_22050) {
        // Expand in place from the end: each mono sample becomes two identical
        // stereo frames. No second audio buffer or resampling task is needed.
        for (int i = static_cast<int>(want / 2) - 1; i >= 0; i--) {
          const uint8_t lo = pcm[2 * i];
          const uint8_t hi = pcm[2 * i + 1];
          for (int j = 0; j < 4; j++) {
            pcm[8 * i + 2 * j] = lo;
            pcm[8 * i + 2 * j + 1] = hi;
          }
        }
        output_size = want * 4;
      } else if (stereo_22050) {
        // Duplicate each stereo frame in place while preserving left and right.
        for (int i = static_cast<int>(want / 4) - 1; i >= 0; i--) {
          const size_t input = 4 * i;
          const size_t output = 8 * i;
          std::memmove(pcm + output, pcm + input, 4);
          std::memcpy(pcm + output + 4, pcm + output, 4);
        }
        output_size = want * 2;
      }
      size_t written = 0;
      while (written < output_size && !this->stop_) {
        const size_t n = this->write_output(pcm + written, output_size - written, 50, info);
        if (n == 0) {
          stalled_ms += 50;
          if (stalled_ms >= 10000) {
            ESP_LOGE(TAG, "A2DP output stalled for 10 seconds");
            break;
          }
        } else {
          written += n;
          total += n;
          this->output_bytes_.fetch_add(n);
          stalled_ms = 0;
        }
      }
      if (written != output_size)
        break;
      if (!streaming_size)
        data_size -= want;
    }
    if (data_size == 0 && !this->stop_) {
      ESP_LOGI(TAG, "WAV PCM delivered: %llu bytes", static_cast<unsigned long long>(total));
      ok = true;
    }
  } while (false);

  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  this->result_ = (ok || this->stop_) ? 1 : -1;
}

}  // namespace esphome::a2dp_wav

#endif  // USE_ESP32
