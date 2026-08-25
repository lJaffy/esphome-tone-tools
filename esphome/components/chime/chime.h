#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <vector>

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome::chime {

/// Sentinel value indicating "no time constraint" for a pattern step.
static constexpr uint32_t NO_TIME = 0xFFFFFFFF;

// ──────────────────────────────────────────────
//  Chime – one pattern definition + binary sensor
// ──────────────────────────────────────────────
struct Chime {
  // ── Config ──
  /// Each inner vector is one "step" in the sequence (a chord of 1+ frequencies).
  std::vector<std::vector<float>> pattern_chords_;
  /// Timestamp for each chord in milliseconds from pattern start.
  /// NO_TIME (0xFFFFFFFF) means "no time constraint" for that step.
  std::vector<uint32_t> pattern_times_ms_;
  uint32_t min_duration_ms_{2000};
  uint32_t max_duration_ms_{5000};
  float threshold_db_{-50.0f};
  uint32_t tail_grace_ms_{2000};
  /// Derived: max_duration_ms_ + 2000.
  uint32_t release_time_ms_{7000};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  // ── Frequency mapping (populated during setup) ──
  std::vector<std::vector<uint32_t>> chord_filter_indices_;

  // ── State machine ──
  bool pattern_active_{false};
  uint8_t match_index_{0};
  uint32_t pattern_start_ms_{0};
  bool need_falling_edge_{false};
  uint32_t release_until_ms_{0};
  bool detected_latched_{false};

  // ── Methods ──
  void set_pattern(const std::vector<std::vector<float>> &chords) { this->pattern_chords_ = chords; }
  void set_pattern_times(const std::vector<uint32_t> &times_ms) { this->pattern_times_ms_ = times_ms; }
  void set_min_duration_ms(uint32_t ms) { this->min_duration_ms_ = ms; }
  void set_max_duration_ms(uint32_t ms) {
    this->max_duration_ms_ = ms;
    this->release_time_ms_ = ms + 2000;
  }
  void set_threshold_db(float db) { this->threshold_db_ = db; }
  void set_tail_grace_ms(uint32_t ms) { this->tail_grace_ms_ = ms; }
  void set_detected_sensor(binary_sensor::BinarySensor *s) { this->detected_sensor_ = s; }

  bool chord_present_(const float *spectrum_db, uint8_t step, float &peak_db) const;
  void log_chord_(uint8_t step) const;

  void evaluate_pattern_(const float *spectrum_db);
  void latch_detection_(uint32_t elapsed_ms);
  void reset_pattern_();
  void reset_all_() {
    this->pattern_active_ = false;
    this->match_index_ = 0;
    this->need_falling_edge_ = false;
  }
};

// ──────────────────────────────────────────────
//  Component
// ──────────────────────────────────────────────
class ChimeComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // ── Configuration setters ──
  void set_microphone_source(microphone::MicrophoneSource *mic) { this->microphone_source_ = mic; }
  void set_window_size(uint16_t n) { this->window_size_ = n; }
  void set_tick_interval(uint32_t ms) { this->tick_interval_ms_ = ms; }

  uint8_t add_chime() {
    this->chimes_.emplace_back();
    return static_cast<uint8_t>(this->chimes_.size() - 1);
  }

  Chime &chime(uint8_t i) { return this->chimes_[i]; }

  // ── Automation actions ──
  void start();
  void stop();

 protected:
  bool start_();
  void stop_();
  void process_frame_(const int16_t *samples);
  void emit_tick_();
  void build_frequency_map_();

  // ── Configuration (shared) ──
  microphone::MicrophoneSource *microphone_source_{nullptr};
  uint16_t window_size_{1024};
  uint32_t tick_interval_ms_{100};

  // ── Chimes ──
  std::vector<Chime> chimes_;

  // ── Audio pipeline (shared) ──
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  int16_t *frame_buf_{nullptr};
  uint32_t frame_buf_offset_{0};

  // ── Goertzel DSP state ──
  float *window_{nullptr};
  float *accum_{nullptr};
  float *g_v1_{nullptr};
  float *g_v2_{nullptr};
  float *g_c2_{nullptr};
  float *spectrum_db_{nullptr};
  std::vector<float> global_freqs_;
  uint32_t total_filters_{0};
  float sample_rate_hz_{0.0f};
  bool dsp_ready_{false};

  // ── Tick assembly ──
  uint32_t frame_count_{0};
  uint32_t tick_sample_count_{0};
};

// ── Automation actions ──
template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<ChimeComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<ChimeComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::chime

#endif