#include "i2s_duplex_probe.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#include <array>
#include <cmath>

namespace esphome::i2s_duplex_probe {

static const char *const TAG = "i2s_duplex_probe";
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr size_t FRAMES_PER_BLOCK = 160;  // 10 ms stereo PCM block
static constexpr size_t BLOCK_COUNT = 200;       // 2 seconds maximum sound

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
  this->rx_overflows_.store(0);
  this->tx_overflows_.store(0);
  this->completed_.store(false);
  if (xTaskCreatePinnedToCore(&I2SDuplexProbe::run_task_, "i2s_duplex_probe", 4096, this, 5,
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
  // 400 Hz is exactly four periods per block, avoiding a discontinuity at block boundaries.
  // The waveform is hard limited to 3% of full-scale PCM to keep the diagnostic quiet.
  for (size_t frame = 0; frame < FRAMES_PER_BLOCK; ++frame) {
    int16_t sample = static_cast<int16_t>(983.0f * sinf(2.0f * 3.14159265f * 400.0f * frame / SAMPLE_RATE));
    output[2 * frame] = sample;
    output[2 * frame + 1] = sample;
  }

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
      for (size_t i = 0; i < received / sizeof(int16_t); ++i) {
        self->microphone_energy_ += static_cast<uint32_t>(std::abs(static_cast<int32_t>(input[i])));
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      self->receive_errors_++;
    }
  }
  self->completed_.store(true);
  vTaskDelete(nullptr);
}

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
  this->completed_.store(false);
  this->task_handle_ = nullptr;
  ESP_LOGI(TAG, "RX bytes=%lu, RX errors=%lu, TX errors=%lu, RX overflow=%lu, TX overflow=%lu, mic energy=%lu",
           static_cast<unsigned long>(this->received_bytes_), static_cast<unsigned long>(this->receive_errors_),
           static_cast<unsigned long>(this->send_errors_), static_cast<unsigned long>(this->rx_overflows_.load()),
           static_cast<unsigned long>(this->tx_overflows_.load()), static_cast<unsigned long>(this->microphone_energy_));
  this->release_channels_();
}

void I2SDuplexProbe::release_channels_() {
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
