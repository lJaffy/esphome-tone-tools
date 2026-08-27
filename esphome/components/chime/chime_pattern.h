#pragma once
// chime/chime_pattern.h — pure pattern state machine, no I/O.
// Port of struct Chime in chime.h:28 / chime.cpp:603-813 with
// platform side-effects (ESP_LOG, binary_sensor, millis) removed.

#include "chime_types.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace esphome::chime::core {

class ChimePattern {
 public:
  explicit ChimePattern(const ChimeConfig &cfg);
  ChimePattern() = default;

  // Config (readable for diagnostics / dump_config)
  const ChimeConfig &config() const { return cfg_; }
  void set_config(const ChimeConfig &cfg);

  // Frequency mapping – populated by ChimeEngine::build_frequency_map()
  std::vector<std::vector<uint32_t>> chord_filter_indices;

  // State machine
  bool pattern_active = false;
  uint8_t match_index = 0;
  uint32_t pattern_start_ms = 0;
  bool need_falling_edge = false;
  uint32_t release_until_ms = 0;
  bool detected_latched = false;

  void reset_pattern();
  void reset_all() {
    pattern_active = false;
    match_index = 0;
    need_falling_edge = false;
  }

  // Returns true if chord present; peak_db set to max bin dB in the chord.
  bool chord_present(const float *spectrum_db, const float *noise_floor, const float *local_bg,
                     const float *onset_contrast, uint8_t step, float &peak_db) const;

  // Evaluate one tick. now_ms is the audio time of the tick (like ChimeDetector.run nowMs).
  // Emits 0..n Events via the callback (replaces ESP_LOG + latch side-effects).
  // Caller handles max-duration timeout and release deadlines separately if desired,
  // but this method handles step timing, falling edge, and detection latch.
  void evaluate_pattern(uint32_t now_ms, const float *spectrum_db, const float *noise_floor,
                        const float *local_bg, const float *onset_contrast,
                        uint32_t chime_index, uint32_t tick_index,
                        const std::function<void(const Event &)> &emit);

  // For diagnostics: per-bin gate snapshot (mirrors ChimeComponent::log_chime_diagnostics_)
  struct BinGate {
    float db = -300;
    float eff = -300;
    float floor = 0;
    bool thr_ok = false;
    float local_bg = -300;
    float prom = 0;
    bool prom_ok = false;
    float onset = 0;
    bool onset_ok = false;
  };
  BinGate bin_gates(const float *spectrum_db, const float *noise_floor, bool noise_floor_ready,
                    const float *local_bg, const float *onset_contrast, uint32_t filter_idx) const;

 private:
  ChimeConfig cfg_;
  void latch_detection_(uint32_t now_ms, uint32_t elapsed_ms, uint32_t chime_index, uint32_t tick_index,
                        const std::function<void(const Event &)> &emit);
};

}  // namespace esphome::chime::core
