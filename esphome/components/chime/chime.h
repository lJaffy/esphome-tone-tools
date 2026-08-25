#pragma once

#ifdef USE_ESP32

#include
#include

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
// Chime – one pattern definition + binary sensor
// ──────────────────────────────────────────────
struct Chime {
  // ── Config ──
  /// Each inner vector is one "step" in the sequence (a chord of 1+ frequencies).
  std::vector<std::vector> pattern_chords_;
  /// Timestamp for each chord in milliseconds from pattern start.
  /// NO_TIME (0xFFFFFFFF) means "no time constraint" for that step.
  std::vector pattern_times_ms_;
  uint32_t min_duration_ms_{2000};
  uint32_t max_duration_ms_{5000};
  /// Hard lower bound on level (dB). The adaptive noise floor can only raise the
  /// effective threshold above this value, never below it.
  float threshold_db_{-50.0f};
  /// How far above the local adaptive noise floor a bin must sit to pass (dB).
  /// 6 dB = 2x power, 10 dB = 10x power.
  float snr_margin_db_{8.0f};
  uint32_t tail_grace_ms_{2000};
  /// Derived: max_duration_ms_ + 2000.
  uint32_t release_time_ms_{7000};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  // ── Frequency mapping (populated during setup) ──
  std::vector<std::vector> chord_filter_indices_;

  // ── State machine ──
  bool pattern_active_{false};
  uint8_t match_index_{0};
  uint32_t pattern_start_ms_{0};
  bool need_falling_edge_{false};
  uint32_t release_until_ms_{0};
  bool detected_latched_{false};

  // ── Methods ──
  void set_pattern(const std::vector<std::vector> &chords) { this->pattern_chords_ = chords; }
  void set_pattern_times(const std::vector &times_ms) { this->pattern_times_ms_ = times_ms; }
  void set_min_duration_ms(uint32_t ms) { this->min_duration_ms_ = ms; }
  void set_max_duration_ms(uint32_t ms) {
    this->max_duration_ms_ = ms;
    this->release_time_ms_ = ms + 2000;
  }
  void set_threshold_db(float db) { this->threshold_db_ = db; }
  void set_snr_margin_db(float db) { this->snr_margin_db_ = db; }
  void set_tail_grace_ms(uint32_t ms) { this->tail_grace_ms_ = ms; }
  void set_detected_sensor(binary_sensor::BinarySensor *s) { this->detected_sensor_ = s; }

  bool chord_present_(const float *spectrum_db, const float *noise_floor, uint8_t step, float &peak_db) const;
  void log_chord_(uint8_t step) const;

  void evaluate_pattern_(const float *spectrum_db, const float *noise_floor);
  void latch_detection_(uint32_t elapsed_ms);
  void reset_pattern_();
  void reset_all_() {
    this->pattern_active_ = false;
    this->match_index_ = 0;
    this->need_falling_edge_ = false;
  }
};

// ──────────────────────────────────────────────
// Component
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
  void set_noise_floor_alpha_down(float a) { this->noise_floor_alpha_down_ = a; }
  void set_noise_floor_alpha_up(float a) { this->noise_floor_alpha_up_ = a; }

  uint8_t add_chime() {
    this->chimes_.emplace_back();
    return static_cast(this->chimes_.size() - 1);
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
  /// True if any active chime pattern currently occupies this global filter bin,
  /// so the noise floor for that bin is left untouched while the chime sounds.
  bool bin_is_busy_(uint32_t filter_idx) const;

  // ── Configuration (shared) ──
  microphone::MicrophoneSource *microphone_source_{nullptr};
  uint16_t window_size_{1024};
  uint32_t tick_interval_ms_{100};

  // ── Chimes ──
  std::vector chimes_;

  // ── Audio pipeline (shared) ──
  std::unique_ptraudio::RingBufferAudioSource audio_source_;
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
  std::vector global_freqs_;
  uint32_t total_filters_{0};
  float sample_rate_hz_{0.0f};
  bool dsp_ready_{false};

  // ── Adaptive noise floor (one entry per global filter) ──
  float *noise_floor_{nullptr};
  float *noise_floor_init_{nullptr};
  bool noise_floor_ready_{false};
  float noise_floor_alpha_down_{0.05f};
  float noise_floor_alpha_up_{0.005f};

  // ── Tick assembly ──
  uint32_t frame_count_{0};
  uint32_t tick_sample_count_{0};
};

// ── Automation actions ──
template<typename... Ts> class StartAction : public Action<Ts...>, public Parented {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::chime

#endif
