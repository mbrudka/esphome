#pragma once

#ifdef USE_ESP32

#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/core/component.h"

#include <atomic>
#include <cstdint>
#include <driver/i2s_std.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::i2s_duplex_probe {

class I2SDuplexProbe : public Component, public Parented<i2s_audio::I2SAudioComponent> {
 public:
  void set_din_pin(int pin) { this->din_pin_ = pin; }
  void set_dout_pin(int pin) { this->dout_pin_ = pin; }

  void loop() override;
  void dump_config() override;

  // Explicitly invoked by a test button. Never produces sound on boot.
  void start_test();

 protected:
  static void run_task_(void *argument);
  static bool IRAM_ATTR on_rx_overflow_(i2s_chan_handle_t channel, i2s_event_data_t *event, void *argument);
  static bool IRAM_ATTR on_tx_overflow_(i2s_chan_handle_t channel, i2s_event_data_t *event, void *argument);
  void release_channels_();

  int din_pin_{I2S_GPIO_UNUSED};
  int dout_pin_{I2S_GPIO_UNUSED};
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
};

}  // namespace esphome::i2s_duplex_probe

#endif  // USE_ESP32
