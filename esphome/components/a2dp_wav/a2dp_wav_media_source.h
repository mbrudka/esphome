#pragma once

#include "esphome/components/media_source/media_source.h"
#include "esphome/core/component.h"

#include <atomic>
#include <string>

namespace esphome::a2dp_wav {

class A2DPWavMediaSource final : public Component, public media_source::MediaSource {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  bool can_handle(const std::string &uri) const override;
  bool play_uri(const std::string &uri) override;
  void handle_command(media_source::MediaSourceCommand command) override;

 protected:
  static void task_entry_(void *arg);
  void run_();

  std::string uri_;
  std::atomic<bool> stop_{false};
  std::atomic<int> result_{0};  // 0 running, 1 complete/stopped, -1 failed
  std::atomic<bool> task_active_{false};
  std::atomic<uint32_t> input_bytes_{0};
  std::atomic<uint32_t> output_bytes_{0};
  std::atomic<uint32_t> max_http_read_us_{0};
};

}  // namespace esphome::a2dp_wav
