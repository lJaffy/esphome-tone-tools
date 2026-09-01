#pragma once
// frequency/frequency_engine.h — pure DSP engine for dominant-frequency detection.
// No ESPHome dependencies. Used by frequency Component glue and (optionally) WASM.
// Mirrors esphome::chime::core::ChimeEngine but for continuous band peak-picking.

#include <cstdint>
#include <vector>

namespace esphome::frequency::core {

static constexpr uint32_t MAX_FFT_FRAMES = 32;

struct FrequencyConfig {
  uint16_t window_size = 1024;
  float min_frequency_hz = 100.0f;
  float max_frequency_hz = 12000.0f;
  float peak_threshold_db = -50.0f;
  uint32_t measurement_duration_ms = 1000;
};

struct FrequencyResult {
  bool valid = false;          // true if peak_db >= threshold and band valid
  float frequency_hz = 0.0f;   // refined (parabolic) frequency, NAN if not valid
  float peak_db = -300.0f;     // dB of strongest bin
  uint32_t bin = 0;            // absolute DFT bin k_star
  float raw_bin_hz = 0.0f;     // bin center before refinement
};

class FrequencyEngine {
 public:
  explicit FrequencyEngine(const FrequencyConfig &cfg = FrequencyConfig{});
  ~FrequencyEngine();

  void set_config(const FrequencyConfig &cfg);
  const FrequencyConfig &config() const { return cfg_; }

  void set_window_size(uint16_t n);
  void set_band(float min_hz, float max_hz);
  void set_threshold_db(float db) { cfg_.peak_threshold_db = db; }
  void set_measurement_duration_ms(uint32_t ms) { cfg_.measurement_duration_ms = ms; }

  // Must be called after config and before first tick; also called on set_sample_rate.
  void set_sample_rate(float fs);
  float sample_rate_hz() const { return sample_rate_hz_; }

  // Streaming API (ESP path): feed PCM and emit when window full.
  void reset_streaming_state();
  void feed_pcm(const int16_t *samples, size_t count);
  // Returns true if a window was emitted; out_result populated. sample_count_before_emit
  // is samples accumulated for diagnostics (like frequency loop's samples_in_window).
  bool try_emit_window(FrequencyResult &out_result);

  // Offline batch API: process mono float [-1,1] in one call (for WASM/tests).
  // Returns vector of window results (one per measurement window).
  std::vector<FrequencyResult> run_offline(const float *mono, size_t num_samples, float fs);

  // Introspection
  bool band_valid() const { return band_valid_; }
  uint32_t k_min() const { return k_min_; }
  uint32_t k_max() const { return k_max_; }
  uint32_t num_bins() const { return num_bins_; }
  const std::vector<float> &window() const { return window_; }
  const std::vector<float> &accum() const { return accum_; }

 private:
  FrequencyConfig cfg_;
  float sample_rate_hz_{0.0f};
  bool band_valid_{false};
  uint32_t k_min_{0};
  uint32_t k_max_{0};
  uint32_t num_bins_{0};

  // DSP buffers (allocated for max N/2, num_bins_ entries used)
  std::vector<float> window_;
  std::vector<float> accum_;
  std::vector<float> v1_, v2_, c2_;

  // Streaming state
  std::vector<int16_t> frame_buf_;
  uint32_t frame_buf_offset_{0};
  uint32_t frame_count_{0};
  uint32_t window_sample_count_{0};

  void ensure_buffers_();
  void update_band_();
  bool process_goertzel_frame_(const int16_t *samples);
  FrequencyResult emit_window_();
};

}  // namespace esphome::frequency::core
