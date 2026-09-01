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
#include "esphome/core/gpio.h"

#include "chime_engine.h"
#include "chime_types.h"
#include "../frequency/dsp/sample_source.h"

#if defined(USE_ESP32)
#if __has_include(<esp_adc/adc_continuous.h>)
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <hal/adc_types.h>
#else
// IDF <5.0 fallback – driver/adc.h still provides needed types
#include <driver/adc.h>
#include <esp_adc_cal.h>
#endif
#endif

namespace esphome::chime {

// Re-export core constants for compat with existing cpp/tests
static constexpr uint32_t NO_TIME = core::NO_TIME;
static constexpr uint32_t ONSET_LOOKBACK = core::ONSET_LOOKBACK;

// ──────────────────────────────────────────────
//  Chime – one pattern definition + binary sensor (ESP glue)
//  Thin wrapper around core::ChimeConfig + binary_sensor.
//  The actual state machine lives in core::ChimePattern inside ChimeEngine.
// ──────────────────────────────────────────────
struct Chime {
  // ── Config (mirrors original fields for dump_config / setters) ──
  std::vector<std::vector<float>> pattern_chords_;
  std::vector<uint32_t> pattern_times_ms_;
  uint32_t min_duration_ms_{2000};
  uint32_t max_duration_ms_{5000};
  float threshold_db_{-50.0f};
  float snr_margin_db_{8.0f};
  float prominence_db_{0.0f};
  float onset_contrast_db_{8.0f};
  uint32_t tail_grace_ms_{2000};
  uint32_t release_time_ms_{7000};
  binary_sensor::BinarySensor *detected_sensor_{nullptr};

  // For diagnostics / compat: filled from engine after build_frequency_map
  std::vector<std::vector<uint32_t>> chord_filter_indices_;

  // State mirrors (read from engine for loop/diagnostics)
  bool pattern_active_{false};
  uint8_t match_index_{0};
  uint32_t pattern_start_ms_{0};
  bool need_falling_edge_{false};
  uint32_t release_until_ms_{0};
  bool detected_latched_{false};

  void set_pattern(const std::vector<std::vector<float>> &chords) {
    pattern_chords_ = chords;
  }
  void set_pattern_times(const std::vector<uint32_t> &times_ms) {
    pattern_times_ms_ = times_ms;
  }
  void set_min_duration_ms(uint32_t ms) { min_duration_ms_ = ms; }
  void set_max_duration_ms(uint32_t ms) {
    max_duration_ms_ = ms;
    release_time_ms_ = ms + 2000;
  }
  void set_threshold_db(float db) { threshold_db_ = db; }
  void set_snr_margin_db(float db) { snr_margin_db_ = db; }
  void set_prominence_db(float db) { prominence_db_ = db; }
  void set_onset_contrast_db(float db) { onset_contrast_db_ = db; }
  void set_tail_grace_ms(uint32_t ms) { tail_grace_ms_ = ms; }
  void set_detected_sensor(binary_sensor::BinarySensor *s) {
    detected_sensor_ = s;
  }

  // Compat helpers – delegate to engine's pattern via ChimeComponent sync
  bool chord_present_(const float *spectrum_db, const float *noise_floor,
                      const float *local_bg, const float *onset_contrast,
                      uint8_t step, float &peak_db) const;
  void log_chord_(uint8_t step) const;
  void evaluate_pattern_(const float *spectrum_db, const float *noise_floor,
                         const float *local_bg, const float *onset_contrast);
  void latch_detection_(uint32_t elapsed_ms);
  void reset_pattern_();
  void reset_all_() {
    pattern_active_ = false;
    match_index_ = 0;
    need_falling_edge_ = false;
  }

  core::ChimeConfig to_core_config(const std::string &name) const;
  void sync_from_core(const core::ChimePattern &p);
  void sync_to_core(core::ChimePattern &p) const;
};

// ──────────────────────────────────────────────
//  Component – ESP glue + core::ChimeEngine
// ──────────────────────────────────────────────
class ChimeComponent : public Component {
public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override {
    return setup_priority::AFTER_CONNECTION;
  }

  void set_microphone_source(microphone::MicrophoneSource *mic) {
    microphone_source_ = mic;
    has_mic_ = (mic != nullptr);
  }
  // ADC source – exclusive with microphone, for mechanically coupled pipe/piezo
  void set_adc_pin(uint8_t pin) {
    adc_pin_ = pin;
    has_adc_ = true;
  }
  void set_adc_attenuation(adc_atten_t atten) { adc_atten_ = atten; }
  void set_adc_sample_rate(uint32_t sr) { adc_sample_rate_ = sr; }
  void set_adc_gain(uint32_t g) { adc_gain_ = g; }
  void set_passive(bool p) { passive_ = p; }

  void set_window_size(uint16_t n) { window_size_ = n; }
  void set_tick_interval(uint32_t ms) { tick_interval_ms_ = ms; }
  void set_noise_floor_alpha_down(float a) { noise_floor_alpha_down_ = a; }
  void set_noise_floor_alpha_up(float a) { noise_floor_alpha_up_ = a; }
  void set_guard_separation_hz(float hz) { guard_separation_hz_ = hz; }

  bool has_microphone() const { return has_mic_; }
  bool has_adc() const { return has_adc_; }

  uint8_t add_chime() {
    chimes_.emplace_back();
    return static_cast<uint8_t>(chimes_.size() - 1);
  }
  Chime &chime(uint8_t i) { return chimes_[i]; }

  void start();
  void stop();

protected:
  bool start_();
  void stop_();
  // ADC helpers
  bool start_adc_();
  void stop_adc_();
  bool read_adc_into_engine_();
  void update_adc_hpf_();
  void build_frequency_map_();
  void compute_local_background_();
  bool bin_is_busy_(uint32_t filter_idx) const;
  void log_chime_diagnostics_(size_t chime_idx, const Chime &c);
  void handle_tick_events_(const std::vector<core::Event> &evs);

  // Old DSP helpers now delegated to engine (kept for compat if called
  // externally)
  void process_frame_(const int16_t *samples);
  void emit_tick_();

  // ── Configuration (shared) ──
  microphone::MicrophoneSource *microphone_source_{nullptr};
  bool has_mic_{false};
  // ADC source
  bool has_adc_{false};
  uint8_t adc_pin_{255};
  adc_atten_t adc_atten_{ADC_ATTEN_DB_11};
  uint32_t adc_sample_rate_{16000};
  uint32_t adc_gain_{1};
  bool passive_{false};

  uint16_t window_size_{1024};
  uint32_t tick_interval_ms_{100};
  float guard_separation_hz_{150.0f};
  float noise_floor_alpha_down_{0.05f};
  float noise_floor_alpha_up_{0.005f};

  std::vector<Chime> chimes_;

  // ── Audio pipeline (microphone path) — legacy members kept for compat, now delegated to mic_adapter_
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  int16_t *frame_buf_{nullptr};
  uint32_t frame_buf_offset_{0};

  // ── ADC pipeline — legacy members kept for compat, now delegated to adc_adapter_
#ifdef USE_ESP32
  adc_continuous_handle_t adc_handle_{nullptr};
  adc_cali_handle_t adc_cali_handle_{nullptr};
  uint8_t *adc_raw_buf_{nullptr};
  size_t adc_raw_buf_len_{0};
  static constexpr size_t ADC_RAW_BUF_BYTES = 2048;
  // DC / bias tracking for piezo-on-pipe (midpoint drifts with temperature)
  int32_t adc_midpoint_{2048};
  float adc_hpf_x_prev_{0.0f};
  float adc_hpf_y_prev_{0.0f};
  float adc_hpf_alpha_{0.995f};
#endif
  bool adc_running_{false};

  // ── Shared sample-source adapters (full abstraction, used by new code paths) ──
  dsp::MicrophoneSampleSource mic_adapter_;
  dsp::AdcSampleSource adc_adapter_;
  int16_t *adapter_read_buf_{nullptr};
  static constexpr size_t kAdapterReadSamples = 1024;

  // ── Core engine (platform-agnostic DSP) ──
  core::ChimeEngine engine_;
  bool engine_built_{false};

  // ── Tick assembly / diagnostics ──
  float sample_rate_hz_{0.0f};
  bool dsp_ready_{false};
  uint32_t debug_tick_count_{0};
  // Compat mirrors for dump_config / diagnostics (proxied from engine)
  std::vector<float> global_freqs_;
  uint32_t total_filters_{0};
  float *window_{nullptr}; // alias to engine window for compat (not owned)
  float *spectrum_db_{nullptr};
  float *noise_floor_{nullptr};
  bool noise_floor_ready_{false};
};

template <typename... Ts>
class StartAction : public Action<Ts...>, public Parented<ChimeComponent> {
public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template <typename... Ts>
class StopAction : public Action<Ts...>, public Parented<ChimeComponent> {
public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

} // namespace esphome::chime

#endif
