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

namespace esphome::tone_sequence {

// ──────────────────────────────────────────────
//  Per-detector state
// ──────────────────────────────────────────────
struct Detector {
  // ── Config (set from to_code) ──
  /// Each inner vector is one "step" in the sequence (a chord of 1+ frequencies).
  std::vector<std::vector<float>> pattern_chords_;
  /// Timestamp for each chord in milliseconds from pattern start (t_0 = 0).
  std::vector<uint32_t> pattern_times_ms_;
  uint32_t min_duration_ms_{2000};
  uint32_t max_duration_ms_{5000};
  float threshold_db_{-50.0f};
  /// Grace period (ms) after the last chord's timestamp for its detection window.
  uint32_t tail_grace_ms_{2000};
  /// Derived automatically: max_duration_ms_ + 2000.
  uint32_t release_time_ms_{7000};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  // ── Frequency mapping (populated during setup) ──
  /// chord_filter_indices_[step] = list of global Goertzel indices for each freq in that chord
  std::vector<std::vector<uint32_t>> chord_filter_indices_;

  // ── Pattern state machine ──
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

  /// Returns true if every frequency in chord[step] is above threshold.
  /// Sets peak_db to the strongest component.
  bool chord_present_(const float *spectrum_db, uint8_t step, float &peak_db) const;

  /// Log a human-readable description of a chord (e.g. "[440.0, 6000.0] Hz").
  void log_chord_(uint8_t step) const;

  // State-machine methods
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
class ToneSequenceComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // ── Configuration setters (called from to_code) ──
  void set_microphone_source(microphone::MicrophoneSource *mic) { this->microphone_source_ = mic; }
  void set_window_size(uint16_t n) { this->window_size_ = n; }
  void set_tick_interval(uint32_t ms) { this->tick_interval_ms_ = ms; }

  /// Adds an empty detector; returns its index.
  uint8_t add_detector() {
    this->detectors_.emplace_back();
    return static_cast<uint8_t>(this->detectors_.size() - 1);
  }

  /// Access a detector by index (for to_code property setting).
  Detector &detector(uint8_t i) { return this->detectors_[i]; }

  // ── Automation actions ──
  void start();
  void stop();

 protected:
  bool start_();
  void stop_();

  /// Runs the Goertzel IIR for each global filter over one N-sample frame.
  void process_frame_(const int16_t *samples);

  /// Averages accum[] into dBFS spectrum, then evaluates every detector.
  void emit_tick_();

  /// Build the union of all frequencies and map each detector's chords
  /// to indices in the global filter array.
  void build_frequency_map_();

  // ── Configuration (shared) ──
  microphone::MicrophoneSource *microphone_source_{nullptr};
  uint16_t window_size_{1024};
  uint32_t tick_interval_ms_{100};

  // ── Detectors ──
  std::vector<Detector> detectors_;

  // ── Audio pipeline (shared) ──
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  int16_t *frame_buf_{nullptr};
  uint32_t frame_buf_offset_{0};

  // ── Goertzel DSP state (shared, sized for the union of all frequencies) ──
  float *window_{nullptr};
  float *accum_{nullptr};
  float *g_v1_{nullptr};
  float *g_v2_{nullptr};
  float *g_c2_{nullptr};
  float *spectrum_db_{nullptr};

  // Global frequency list (all unique tones across all detectors)
  std::vector<float> global_freqs_;
  uint32_t total_filters_{0};

  float sample_rate_hz_{0.0f};
  bool dsp_ready_{false};

  // ── Tick assembly ──
  uint32_t frame_count_{0};
  uint32_t tick_sample_count_{0};
};

// ── Automation actions ──
template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<ToneSequenceComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<ToneSequenceComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::tone_sequence

#endif