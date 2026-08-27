#pragma once
// chime/chime_engine.h — pure DSP engine (Goertzel + tick orchestration).
// No ESPHome dependencies. Used by both ESP glue and WASM bindings.

#include "chime_pattern.h"
#include "chime_types.h"

#include <cstdint>
#include <vector>

namespace esphome::chime::core {

class ChimeEngine {
 public:
  explicit ChimeEngine(const DetectorConfig &cfg = DetectorConfig{});
  ~ChimeEngine();

  // Configuration
  void set_detector_config(const DetectorConfig &cfg);
  const DetectorConfig &detector_config() const { return det_cfg_; }

  void add_chime(const ChimeConfig &cfg);
  void clear_chimes();
  size_t num_chimes() const { return chimes_.size(); }
  ChimePattern &chime(size_t i) { return chimes_[i]; }
  const ChimePattern &chime(size_t i) const { return chimes_[i]; }

  // Must be called after chimes are added and before first tick.
  // Also recomputed on set_sample_rate.
  void build_frequency_map();

  void set_sample_rate(float fs);
  float sample_rate_hz() const { return sample_rate_hz_; }

  // Streaming API (ESP path): feed PCM and produce tick results.
  // Call set_sample_rate() first. feed_pcm accumulates int16 samples;
  // each full window is Goertzel-processed. Use try_emit_tick() to
  // see if a tick is ready (frameCount >= framesPerTick etc.).
  void reset_streaming_state();
  void feed_pcm(const int16_t *samples, size_t count);
  // Returns true if a tick was emitted; out_events/out_tick populated.
  // now_ms is the audio/millis time of this tick (for pattern state machine).
  bool try_emit_tick(uint32_t now_ms, std::vector<Event> &out_events, TickSnapshot *out_tick = nullptr);

  // Offline batch API (WASM path): process mono float [-1,1] in one call,
  // mirroring js/chime.js ChimeDetector.run().
  RunResult run_offline(const float *mono, size_t num_samples, float fs);

  // Introspection for diagnostics / glue
  const std::vector<float> &global_freqs() const { return global_freqs_; }
  const std::vector<float> &effective_freqs() const { return effective_freqs_; }
  uint32_t total_filters() const { return total_filters_; }
  const std::vector<float> &spectrum_db() const { return spectrum_db_; }
  const std::vector<float> &noise_floor() const { return noise_floor_; }
  bool noise_floor_ready() const { return noise_floor_ready_; }
  const std::vector<float> &local_bg() const { return local_bg_db_; }
  const std::vector<float> &onset_contrast() const { return onset_contrast_; }

  // Direct buffer access for ESP diagnostics (compat with old API)
  // These allocate internally; exposed for glue to handle onset history reset etc.
  void reset_onset_history();

 private:
  DetectorConfig det_cfg_;
  std::vector<ChimePattern> chimes_;

  // Frequency map
  std::vector<float> global_freqs_;
  std::vector<float> effective_freqs_;  // clamped to [20, nyquist-20]
  uint32_t total_filters_ = 0;

  // DSP state (vectors, allocated in build_frequency_map / set_sample_rate)
  std::vector<float> window_;
  std::vector<float> accum_;
  std::vector<float> g_v1_, g_v2_, g_c2_;
  std::vector<float> spectrum_db_;
  std::vector<float> noise_floor_, noise_floor_init_;
  bool noise_floor_ready_ = false;
  std::vector<float> local_bg_db_;
  std::vector<float> onset_history_;  // nf * ONSET_LOOKBACK
  std::vector<uint8_t> onset_count_;
  std::vector<float> onset_contrast_;
  uint32_t onset_ring_pos_ = 0;
  float sample_rate_hz_ = 0.0f;
  bool dsp_ready_ = false;

  // Streaming tick assembly
  std::vector<int16_t> frame_buf_;
  uint32_t frame_buf_offset_ = 0;
  uint32_t frame_count_ = 0;
  uint32_t tick_sample_count_ = 0;
  uint32_t debug_tick_count_ = 0;

  // Helpers
  void ensure_buffers_();
  void process_frame_(const int16_t *samples);
  void compute_local_background_();
  bool bin_is_busy_(uint32_t filter_idx) const;
  uint32_t frames_per_tick_() const;
  float tick_audio_ms_() const;
};

}  // namespace esphome::chime::core
