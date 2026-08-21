#pragma once

#ifdef USE_ESP32

#include <array>
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
  uint32_t min_duration_ms_{1500};
  uint32_t max_duration_ms_{3000};
  float threshold_db_{-50.0f};
  /// Margin above the adaptive noise floor a tone must exceed to turn on.
  float noise_margin_db_{6.0f};
  /// Schmitt-trigger width: once on, a chord stays present until its level drops
  /// below (on-threshold − hysteresis).
  float hysteresis_db_{5.0f};
  /// A note present longer than this at one step is "stuck" and ignored for
  /// advance purposes until the chord falls below its off-threshold.
  uint32_t max_note_duration_ms_{1000};
  /// A chord only counts as present after this many consecutive ticks above threshold.
  uint8_t min_consecutive_ticks_{2};
  /// Derived automatically: max_duration_ms_ + 2000.
  uint32_t release_time_ms_{5000};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  // ── Frequency mapping (populated during setup) ──
  /// chord_filter_indices_[step][i] = {f-Δ, f, f+Δ} global Goertzel indices for the i-th freq in that chord
  std::vector<std::vector<std::array<uint32_t, 3>>> chord_filter_indices_;

  // ── Hysteresis state (sized to pattern_chords_ in set_pattern) ──
  /// Per-step flag: is this step's chord currently in the hysteresis "on" state?
  std::vector<bool> chord_active_;

  // ── Consecutive-tick confirmation (sized to pattern_chords_ in set_pattern) ──
  /// Per-step count of consecutive ticks the chord has been present.
  std::vector<uint8_t> chord_tick_count_;

  // ── Pattern state machine ──
  bool pattern_active_{false};
  uint8_t match_index_{0};
  uint32_t pattern_start_ms_{0};
  bool need_falling_edge_{false};
  uint32_t release_until_ms_{0};
  bool detected_latched_{false};
  /// IDLE ignores chord-1 matches until this time (set on latch).
  uint32_t refractory_until_ms_{0};

  // ── Per-step timing guard rails (sized in set_pattern) ──
  /// Wall-clock time the current note for this step started; 0 = no note running.
  std::vector<uint32_t> note_start_ms_;
  /// Set when a step's note was rejected as stuck; held until the chord drops
  /// below its off-threshold (re-arm), never cleared by a plain counter reset.
  std::vector<uint8_t> step_stuck_;

  // ── Methods ──
  void set_pattern(const std::vector<std::vector<float>> &chords) {
    this->pattern_chords_ = chords;
    const size_t nsteps = this->pattern_chords_.size();
    this->chord_active_.assign(nsteps, false);
    this->chord_tick_count_.assign(nsteps, 0);
    this->note_start_ms_.assign(nsteps, 0);
    this->step_stuck_.assign(nsteps, 0);
  }
  void set_min_duration_ms(uint32_t ms) { this->min_duration_ms_ = ms; }
  void set_max_duration_ms(uint32_t ms) {
    this->max_duration_ms_ = ms;
    this->release_time_ms_ = ms + 2000;
  }
  void set_threshold_db(float db) { this->threshold_db_ = db; }
  void set_min_consecutive_ticks(uint8_t n) { this->min_consecutive_ticks_ = n; }
  void set_noise_margin_db(float db) { this->noise_margin_db_ = db; }
  void set_hysteresis_db(float db) { this->hysteresis_db_ = db; }
  void set_max_note_duration_ms(uint32_t ms) { this->max_note_duration_ms_ = ms; }
  void set_detected_sensor(binary_sensor::BinarySensor *s) { this->detected_sensor_ = s; }

  /// Hysteresis-stabilised presence test for chord[step]: turns on when every
  /// frequency is above its effective on-threshold (max of threshold_db_ and
  /// noise_floor_[bin] + noise_margin_db_), stays on while every frequency is
  /// above on-threshold − hysteresis_db_. Updates chord_active_[step].
  /// Sets peak_db to the strongest component (unthresholded).
  bool chord_present_(const float *spectrum_db, const float *noise_floor_db, uint8_t step, float &peak_db);

  /// Confirmation counter for step s: increments if the (hysteresis-stabilised)
  /// chord is present this tick, resets to 0 otherwise. Returns true when the
  /// counter reaches min_consecutive_ticks_.
  bool confirm_step_(const float *spectrum_db, const float *noise_floor_db, uint8_t step, float &peak_db);

  /// Log a human-readable description of a chord (e.g. "[440.0, 6000.0] Hz").
  void log_chord_(uint8_t step) const;

  // State-machine methods
  void evaluate_pattern_(const float *spectrum_db, const float *noise_floor_db);
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
  /// Slow exponential-min ambient floor per global bin (shared across detectors).
  float *noise_floor_{nullptr};
  /// Adaptive-floor decay time-constant in seconds (recompute step per tick).
  float floor_decay_s_{30.0f};
  void set_floor_decay_s(float s) { this->floor_decay_s_ = s; }

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
