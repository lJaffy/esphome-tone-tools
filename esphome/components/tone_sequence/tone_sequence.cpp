#include "tone_sequence.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace esphome::tone_sequence {

static const char *const TAG = "tone_sequence";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t MAX_FRAMES_PER_TICK = 16;

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────

void ToneSequenceComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tone Sequence Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Detectors: %lu\n"
                "  Total Goertzel filters: %lu (3 bins per target frequency, Δ = max(5 Hz, 1%% of f))",
                this->window_size_, this->tick_interval_ms_, (unsigned long) this->detectors_.size(),
                (unsigned long) this->total_filters_);

  for (size_t d = 0; d < this->detectors_.size(); ++d) {
    auto &det = this->detectors_[d];
    ESP_LOGCONFIG(TAG, "    Detector[%lu]:", (unsigned long) d);
    ESP_LOGCONFIG(TAG, "      Min Duration: %" PRIu32 " ms", det.min_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Max Duration: %" PRIu32 " ms", det.max_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Threshold: %.1f dB", det.threshold_db_);
    ESP_LOGCONFIG(TAG, "      Min Consecutive Ticks: %u", (unsigned) det.min_consecutive_ticks_);
    ESP_LOGCONFIG(TAG, "      Noise Margin: %.1f dB (above adaptive floor)", det.noise_margin_db_);
    ESP_LOGCONFIG(TAG, "      Hysteresis: %.1f dB", det.hysteresis_db_);
    ESP_LOGCONFIG(TAG, "      Max Note Duration: %" PRIu32 " ms", det.max_note_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Release Time: %" PRIu32 " ms (auto)", det.release_time_ms_);
    ESP_LOGCONFIG(TAG, "      Chords (%lu steps):", (unsigned long) det.pattern_chords_.size());
    for (size_t s = 0; s < det.pattern_chords_.size(); ++s) {
      ESP_LOGCONFIG(TAG, "        [%u]:", (unsigned) s);
      const auto &chord = det.pattern_chords_[s];
      for (size_t f = 0; f < chord.size(); ++f) {
        ESP_LOGCONFIG(TAG, "          %.1f Hz", chord[f]);
      }
    }
    if (det.detected_sensor_ != nullptr) {
      LOG_BINARY_SENSOR("      ", "Detected:", det.detected_sensor_);
    }
  }
}

// ──────────────────────────────────────────────
//  Setup – one-time allocation
// ──────────────────────────────────────────────

void ToneSequenceComponent::setup() {
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto rb = this->ring_buffer_.lock();
    if (rb != nullptr) {
      rb->write((void *) data.data(), data.size());
    }
  });

  if (this->detectors_.empty()) {
    ESP_LOGE(TAG, "No detectors configured – nothing to do");
    return;
  }

  // ── Build the global frequency union ──
  this->build_frequency_map_();

  ESP_LOGI(TAG, "Global Goertzel filters: %lu (3 bins per target frequency)", (unsigned long) this->total_filters_);

  // ── Hann window (N floats) ──
  const uint32_t n = this->window_size_;
  float *window = static_cast<float *>(malloc(n * sizeof(float)));
  if (window == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate window buffer (%" PRIu32 " bytes)", n * sizeof(float));
    return;
  }
  for (uint32_t i = 0; i < n; ++i) {
    window[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) i / (float) (n - 1)));
  }
  this->window_ = window;

  // ── Goertzel buffers (one slot per global filter) ──
  const uint32_t nf = this->total_filters_;
  float *accum = static_cast<float *>(malloc(nf * sizeof(float)));
  float *v1 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *v2 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *c2 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *spec = static_cast<float *>(malloc(nf * sizeof(float)));
  float *floor = static_cast<float *>(malloc(nf * sizeof(float)));
  if (accum == nullptr || v1 == nullptr || v2 == nullptr || c2 == nullptr || spec == nullptr || floor == nullptr) {
    free(accum);
    free(v1);
    free(v2);
    free(c2);
    free(spec);
    free(floor);
    free(window);
    this->window_ = nullptr;
    ESP_LOGE(TAG, "Failed to allocate Goertzel buffers");
    return;
  }

  this->accum_ = accum;
  this->g_v1_ = v1;
  this->g_v2_ = v2;
  this->g_c2_ = c2;
  this->spectrum_db_ = spec;
  this->noise_floor_ = floor;
  memset(accum, 0, nf * sizeof(float));
  // Floor starts very low so it never masks the hard threshold at boot.
  for (uint32_t i = 0; i < nf; ++i)
    floor[i] = -80.0f;

  this->sample_rate_hz_ = 0.0f;
  this->dsp_ready_ = true;

  if (!this->microphone_source_->is_passive()) {
    this->microphone_source_->start();
  }
}

// ──────────────────────────────────────────────
//  Build global frequency union & per-detector index mapping
// ──────────────────────────────────────────────

void ToneSequenceComponent::build_frequency_map_() {
  // Collect all unique chord frequencies across all detectors, expanding each
  // nominal into a ±Δ triplet: Δ = max(5 Hz, 1% of f). The triplet (f-Δ, f, f+Δ)
  // is deduplicated against the expanded list with the same 0.5 Hz epsilon, so
  // expanded frequencies from different nominals that coincide share one filter.
  std::vector<float> all_freqs;

  auto add_unique = [&all_freqs](float f) {
    for (auto &existing : all_freqs) {
      if (std::fabs(existing - f) < 0.5f)
        return;  // close enough
    }
    all_freqs.push_back(f);
  };

  auto add_triplet = [&add_unique](float f) {
    const float delta = std::max(5.0f, 0.01f * f);
    add_unique(f - delta);
    add_unique(f);
    add_unique(f + delta);
  };

  for (auto &det : this->detectors_) {
    for (const auto &chord : det.pattern_chords_) {
      for (auto f : chord)
        add_triplet(f);
    }
  }

  this->global_freqs_ = all_freqs;
  this->total_filters_ = all_freqs.size();

  // For each detector, map its chord frequencies to triplets of global indices.
  auto find_global_idx = [&all_freqs](float f) -> uint32_t {
    for (uint32_t i = 0; i < all_freqs.size(); ++i) {
      if (std::fabs(all_freqs[i] - f) < 0.5f)
        return i;
    }
    return 0;  // shouldn't happen
  };

  for (auto &det : this->detectors_) {
    det.chord_filter_indices_.clear();
    for (const auto &chord : det.pattern_chords_) {
      std::vector<std::array<uint32_t, 3>> indices;
      for (auto f : chord) {
        const float delta = std::max(5.0f, 0.01f * f);
        indices.push_back({find_global_idx(f - delta), find_global_idx(f), find_global_idx(f + delta)});
      }
      det.chord_filter_indices_.push_back(indices);
    }
  }
}

// ──────────────────────────────────────────────
//  Main loop
// ──────────────────────────────────────────────

void ToneSequenceComponent::loop() {
  if (!this->dsp_ready_ || this->window_ == nullptr || this->accum_ == nullptr || this->g_v1_ == nullptr ||
      this->g_v2_ == nullptr || this->g_c2_ == nullptr) {
    return;
  }

  // ── Check if any detector has a sensor (for mic-not-running warning) ──
  bool any_sensor = false;
  for (auto &det : this->detectors_) {
    if (det.detected_sensor_ != nullptr) {
      any_sensor = true;
      break;
    }
  }

  if (this->microphone_source_->is_running() && !this->status_has_error()) {
    if (this->start_()) {
      this->status_clear_warning();
    } else {
      ESP_LOGW(TAG, "Buffer allocation failed");
      return;
    }
  } else {
    if (!this->status_has_warning()) {
      this->status_set_warning(LOG_STR("Microphone is not running"));
    }
    this->stop_();
    if (any_sensor) {
      for (auto &det : this->detectors_) {
        if (det.detected_sensor_ != nullptr)
          det.detected_sensor_->publish_state(false);
      }
    }
    return;
  }

  if (this->status_has_error()) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();

  // First loop with a valid sample rate: compute Goertzel coefficients
  if (this->sample_rate_hz_ == 0.0f) {
    this->sample_rate_hz_ = static_cast<float>(stream_info.get_sample_rate());
    const uint32_t n = this->window_size_;
    const float nyquist = this->sample_rate_hz_ / 2.0f;

    // DC-blocking high-pass coefficients (applied once per sample at buffer ingress)
    const float rc_samples = this->sample_rate_hz_ / (2.0f * (float) M_PI * this->hpf_corner_hz_);
    this->hpf_alpha_ = rc_samples / (rc_samples + 1.0f);
    this->hpf_x_prev_ = 0.0f;
    this->hpf_y_prev_ = 0.0f;

    for (uint32_t t = 0; t < this->total_filters_; ++t) {
      float freq = this->global_freqs_[t];
      if (freq < 20.0f)
        freq = 20.0f;
      if (freq > nyquist - 20.0f)
        freq = nyquist - 20.0f;
      const float k = (freq * static_cast<float>(n)) / this->sample_rate_hz_;
      this->g_c2_[t] = 2.0f * cosf(2.0f * (float) M_PI * k / static_cast<float>(n));
    }

    ESP_LOGI(TAG, "Goertzel init: %lu filters, N=%" PRIu16 ", fs=%" PRIu32 " Hz", (unsigned long) this->total_filters_,
             this->window_size_, (uint32_t) stream_info.get_sample_rate());
  }

  const uint32_t samples_in_tick = stream_info.ms_to_samples(this->tick_interval_ms_);

  // ── Stage samples into the circular buffer (1.5 N) and run Goertzel with a
  //    N/2 hop (50 % overlap). The DC-blocker IIR runs once per sample at
  //    ingress, so overlapped frames each see the correctly-filtered value. ──
  const uint32_t n = this->window_size_;
  const uint32_t buf_cap = (3u * n) / 2u;
  const uint32_t hop = n / 2u;

  while (this->frame_count_ < MAX_FRAMES_PER_TICK && this->tick_sample_count_ < samples_in_tick) {
    // Process every complete frame available in the circular buffer.
    if (this->frame_samples_avail_ >= n) {
      // Extract the N-sample span at frame_read_pos_ into the linear scratch
      // (at most two memcpy when the span wraps the buffer end).
      const uint32_t first = std::min(n, buf_cap - this->frame_read_pos_);
      std::memcpy(this->frame_scratch_, this->frame_buf_ + this->frame_read_pos_, stream_info.samples_to_bytes(first));
      if (first < n) {
        std::memcpy(this->frame_scratch_ + first, this->frame_buf_, stream_info.samples_to_bytes(n - first));
      }
      this->process_frame_(this->frame_scratch_);
      this->frame_read_pos_ += hop;
      if (this->frame_read_pos_ >= buf_cap)
        this->frame_read_pos_ -= buf_cap;
      this->frame_samples_avail_ -= hop;
      this->frame_count_++;
      // With a N/2 hop, each processed frame advances real audio time by N/2.
      this->tick_sample_count_ += hop;
    }

    this->audio_source_->fill(0, false);
    const uint32_t available = stream_info.bytes_to_samples(this->audio_source_->available());
    if (available == 0)
      break;

    const int16_t *data = reinterpret_cast<const int16_t *>(this->audio_source_->mutable_data());
    const uint32_t free_space = buf_cap - this->frame_samples_avail_;
    const uint32_t take = std::min(available, free_space);
    if (take > 0) {
      // First-order DC-blocking high-pass: y[n] = a·(y[n-1] + x[n] − x[n-1])
      for (uint32_t i = 0; i < take; ++i) {
        const float x = static_cast<float>(data[i]);
        float y = this->hpf_alpha_ * (this->hpf_y_prev_ + x - this->hpf_x_prev_);
        this->hpf_x_prev_ = x;
        this->hpf_y_prev_ = y;
        const uint32_t idx = (this->frame_write_pos_ + i) % buf_cap;
        this->frame_buf_[idx] = static_cast<int16_t>(y < -32768.0f ? -32768.0f : (y > 32767.0f ? 32767.0f : y));
      }
      this->frame_write_pos_ = (this->frame_write_pos_ + take) % buf_cap;
      this->frame_samples_avail_ += take;
    }
    this->audio_source_->consume(stream_info.samples_to_bytes(take));
  }

  // ── Emit when the tick window is full or frame cap is hit ──
  if (this->frame_count_ > 0 &&
      (this->tick_sample_count_ >= samples_in_tick || this->frame_count_ >= MAX_FRAMES_PER_TICK)) {
    this->emit_tick_();
  }

  // ── Per-detector: release & max-duration deadline ──
  for (auto &det : this->detectors_) {
    // Release latched detection
    if (det.detected_latched_ && millis() >= det.release_until_ms_) {
      det.detected_latched_ = false;
      if (det.detected_sensor_ != nullptr)
        det.detected_sensor_->publish_state(false);
      ESP_LOGD(TAG, "Detector[%lu] released after %" PRIu32 " ms hold",
               (unsigned long) &det - (unsigned long) &this->detectors_[0], det.release_time_ms_);
    }

    // Max-duration deadline: if the pattern is still incomplete after `max`,
    // clear the cache to make way for a new detection.
    if (det.pattern_active_) {
      const uint32_t elapsed = millis() - det.pattern_start_ms_;
      if (elapsed > det.max_duration_ms_) {
        ESP_LOGD(TAG, "Detector[%lu] pattern timed out after %" PRIu32 " ms (max %" PRIu32 " ms, matched %u/%lu)",
                 (unsigned long) &det - (unsigned long) &this->detectors_[0], (unsigned long) elapsed,
                 (unsigned long) det.max_duration_ms_, det.match_index_, (unsigned long) det.pattern_chords_.size());
        det.reset_pattern_();
        if (!det.detected_latched_ && det.detected_sensor_ != nullptr)
          det.detected_sensor_->publish_state(false);
      }
    }
  }
}

// ──────────────────────────────────────────────
//  Goertzel frame processing
// ──────────────────────────────────────────────

void ToneSequenceComponent::process_frame_(const int16_t *samples) {
  const uint32_t n = this->window_size_;
  const uint32_t nf = this->total_filters_;

  for (uint32_t t = 0; t < nf; ++t) {
    this->g_v1_[t] = 0.0f;
    this->g_v2_[t] = 0.0f;
  }

  for (uint32_t i = 0; i < n; ++i) {
    const float x = (static_cast<float>(samples[i]) / 32768.0f) * this->window_[i];
    for (uint32_t t = 0; t < nf; ++t) {
      const float v = this->g_c2_[t] * this->g_v1_[t] - this->g_v2_[t] + x;
      this->g_v2_[t] = this->g_v1_[t];
      this->g_v1_[t] = v;
    }
  }

  for (uint32_t t = 0; t < nf; ++t) {
    const float v1 = this->g_v1_[t];
    const float v2 = this->g_v2_[t];
    this->accum_[t] += v1 * v1 + v2 * v2 - this->g_c2_[t] * v1 * v2;
  }
}

// ──────────────────────────────────────────────
//  Tick emission – shared spectrum, per-detector evaluation
// ──────────────────────────────────────────────

void ToneSequenceComponent::emit_tick_() {
  const uint32_t n = this->window_size_;
  const float ref = static_cast<float>(n);

  // Average over frames in this tick, then convert to dBFS
  if (this->frame_count_ > 0) {
    const float inv = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t t = 0; t < this->total_filters_; ++t) {
      this->accum_[t] *= inv;
      const float p = this->accum_[t];
      this->spectrum_db_[t] = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
    }
  }

  // Update the adaptive ambient floor per bin: drops instantly with the
  // spectrum, rises at most decay_step_db (exponential, ~floor_decay_s time constant).
  const float tick_period_s = static_cast<float>(this->tick_interval_ms_) / 1000.0f;
  const float decay_step_db = 10.0f * log10f(1.0f + tick_period_s / this->floor_decay_s_);
  for (uint32_t t = 0; t < this->total_filters_; ++t) {
    this->noise_floor_[t] = std::min(this->spectrum_db_[t], this->noise_floor_[t] + decay_step_db);
  }

  // Evaluate each detector against the shared spectrum (floor-aware hysteresis)
  for (auto &det : this->detectors_) {
    det.evaluate_pattern_(this->spectrum_db_, this->noise_floor_);
  }

  // Reset for next tick
  memset(this->accum_, 0, this->total_filters_ * sizeof(float));
  this->frame_count_ = 0;
  this->tick_sample_count_ = 0;
}

// ──────────────────────────────────────────────
//  Per-detector chord detection
// ──────────────────────────────────────────────

bool Detector::chord_present_(const float *spectrum_db, const float *noise_floor_db, uint8_t step, float &peak_db) {
  const auto &indices = this->chord_filter_indices_[step];
  peak_db = -300.0f;

  // Per-frequency levels: max over the three Goertzel bins (f-Δ, f, f+Δ).
  const uint8_t nfreq = static_cast<uint8_t>(indices.size());
  float levels[8];   // chord length is schema-bounded to 8
  float on_th[8];    // effective on-threshold per frequency

  for (uint8_t i = 0; i < nfreq; ++i) {
    const auto &triplet = indices[i];
    levels[i] = std::max({spectrum_db[triplet[0]], spectrum_db[triplet[1]], spectrum_db[triplet[2]]});
    if (levels[i] > peak_db)
      peak_db = levels[i];

    // Effective on-threshold: the user's absolute threshold is a hard lower
    // bound; above that, the tone must clear the adaptive floor by margin.
    const float best_floor = std::max({noise_floor_db[triplet[0]], noise_floor_db[triplet[1]],
                                       noise_floor_db[triplet[2]]});
    on_th[i] = std::max(this->threshold_db_, best_floor + this->noise_margin_db_);
  }

  const bool was_active = this->chord_active_[step];
  bool result;
  if (was_active) {
    // Staying on: every frequency must remain above its off-threshold.
    result = true;
    for (uint8_t i = 0; i < nfreq; ++i) {
      if (levels[i] < on_th[i] - this->hysteresis_db_) {
        result = false;
        break;
      }
    }
  } else {
    // Turning on: every frequency must cross its on-threshold.
    result = true;
    for (uint8_t i = 0; i < nfreq; ++i) {
      if (levels[i] < on_th[i]) {
        result = false;
        break;
      }
    }
  }

  this->chord_active_[step] = result;
  return result;
}

bool Detector::confirm_step_(const float *spectrum_db, const float *noise_floor_db, uint8_t step, float &peak_db) {
  if (this->chord_present_(spectrum_db, noise_floor_db, step, peak_db)) {
    if (this->note_start_ms_[step] == 0) {
      this->note_start_ms_[step] = millis();
    }
    this->chord_tick_count_[step] = (this->chord_tick_count_[step] < 250u) ? this->chord_tick_count_[step] + 1 : 250;

    // Stuck-tone guard: a note confirmed but still present after
    // max_note_duration_ms_ is not a chime note. Mark the step stuck until
    // its falling edge — do not advance and do not reset the pattern.
    if (!this->step_stuck_[step] && this->chord_tick_count_[step] >= this->min_consecutive_ticks_ &&
        millis() - this->note_start_ms_[step] > this->max_note_duration_ms_) {
      this->step_stuck_[step] = 1;
      this->chord_tick_count_[step] = 0;
      ESP_LOGD(TAG, "Det step %u stuck tone (>%" PRIu32 " ms) – ignored until falling edge", (unsigned) step,
               (unsigned long) this->max_note_duration_ms_);
    }
    return !this->step_stuck_[step] && this->chord_tick_count_[step] >= this->min_consecutive_ticks_;
  }

  // Falling edge for this step → re-arm.
  this->note_start_ms_[step] = 0;
  this->step_stuck_[step] = 0;
  this->chord_tick_count_[step] = 0;
  peak_db = -300.0f;
  return false;
}

void Detector::log_chord_(uint8_t step) const {
  const auto &chord = this->pattern_chords_[step];
  std::string parts;
  for (size_t i = 0; i < chord.size(); ++i) {
    if (i > 0)
      parts += ", ";
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f", chord[i]);
    parts += buf;
  }
  ESP_LOGD(TAG, "  chord[%u] = [%s] Hz", (unsigned) step, parts.c_str());
}

// ──────────────────────────────────────────────
//  Per-detector pattern state machine
// ──────────────────────────────────────────────

void Detector::evaluate_pattern_(const float *spectrum_db, const float *noise_floor_db) {
  const uint32_t num_steps = static_cast<uint32_t>(this->pattern_chords_.size());

  // IDLE – waiting for first chord (confirmed over min_consecutive_ticks_ ticks)
  if (!this->pattern_active_) {
    // Refractory: ignore chord-1 matches briefly after a latch so the tail of
    // the same chime event does not start a second pattern.
    if (millis() < this->refractory_until_ms_) {
      return;
    }

    float peak = -300.0f;
    if (this->confirm_step_(spectrum_db, noise_floor_db, 0, peak)) {
      this->chord_tick_count_[0] = 0;
      this->pattern_active_ = true;
      this->match_index_ = 1;
      this->pattern_start_ms_ = millis();
      this->need_falling_edge_ = true;
      ESP_LOGI(TAG, "Det pattern started: chord 1/%lu, peak %.1f dB", (unsigned long) num_steps, peak);
      this->log_chord_(0);
    }
    return;
  }

  // WAITING FOR FALLING EDGE (previous chord must cease)
  if (this->need_falling_edge_) {
    const uint8_t prev_idx = static_cast<uint8_t>(this->match_index_ - 1);
    float prev_peak = -300.0f;
    const bool prev_still_present = this->chord_present_(spectrum_db, noise_floor_db, prev_idx, prev_peak);

    if (prev_still_present) {
      return;  // previous chord still sounding – wait
    }

    this->need_falling_edge_ = false;
    ESP_LOGD(TAG, "Det falling edge after chord %u/%lu", (unsigned) (prev_idx + 1), (unsigned long) num_steps);
  }

  // MATCHING – look for the next chord
  if (this->match_index_ >= num_steps) {
    const uint32_t elapsed = millis() - this->pattern_start_ms_;
    if (elapsed >= this->min_duration_ms_) {
      this->latch_detection_(elapsed);
    } else {
      ESP_LOGD(TAG, "Det pattern discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms",
               (unsigned long) elapsed, (unsigned long) this->min_duration_ms_);
      this->reset_pattern_();
    }
    return;
  }

  float peak = -300.0f;
  if (this->confirm_step_(spectrum_db, noise_floor_db, this->match_index_, peak)) {
    this->chord_tick_count_[this->match_index_] = 0;
    ESP_LOGD(TAG, "Det chord %lu/%lu matched, peak %.1f dB", (unsigned long) (this->match_index_ + 1),
             (unsigned long) num_steps, peak);
    this->log_chord_(this->match_index_);
    this->match_index_++;
    this->need_falling_edge_ = true;

    if (this->match_index_ >= num_steps) {
      const uint32_t elapsed = millis() - this->pattern_start_ms_;
      if (elapsed >= this->min_duration_ms_) {
        this->latch_detection_(elapsed);
      } else {
        ESP_LOGD(TAG, "Det pattern discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms",
                 (unsigned long) elapsed, (unsigned long) this->min_duration_ms_);
        this->reset_pattern_();
      }
    }
  }
}

void Detector::latch_detection_(uint32_t elapsed_ms) {
  ESP_LOGI(TAG, "Det PATTERN DETECTED in %" PRIu32 " ms (%lu chord steps)", (unsigned long) elapsed_ms,
           (unsigned long) this->pattern_chords_.size());

  this->detected_latched_ = true;
  this->release_until_ms_ = millis() + this->release_time_ms_;

  if (this->detected_sensor_ != nullptr) {
    this->detected_sensor_->publish_state(true);
  }

  // Refractory window: the tail of the chime that just completed must not
  // immediately start a second pattern. One full note at the maximum allowed
  // duration covers any realistic overlap without being so long it blocks a
  // genuine quick re-trigger.
  this->refractory_until_ms_ = millis() + this->max_note_duration_ms_;

  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
  std::fill(this->chord_tick_count_.begin(), this->chord_tick_count_.end(), 0);
  std::fill(this->note_start_ms_.begin(), this->note_start_ms_.end(), 0u);
  std::fill(this->step_stuck_.begin(), this->step_stuck_.end(), 0u);
  std::fill(this->chord_active_.begin(), this->chord_active_.end(), false);
}

void Detector::reset_pattern_() {
  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
  std::fill(this->chord_tick_count_.begin(), this->chord_tick_count_.end(), 0);
  std::fill(this->note_start_ms_.begin(), this->note_start_ms_.end(), 0u);
  std::fill(this->step_stuck_.begin(), this->step_stuck_.end(), 0u);
  std::fill(this->chord_active_.begin(), this->chord_active_.end(), false);
}

// ──────────────────────────────────────────────
//  Public start/stop
// ──────────────────────────────────────────────

void ToneSequenceComponent::start() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot start microphone in passive mode");
    return;
  }
  this->microphone_source_->start();
}

void ToneSequenceComponent::stop() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot stop microphone in passive mode");
    return;
  }
  this->microphone_source_->stop();
}

// ──────────────────────────────────────────────
//  Internal buffer management
// ──────────────────────────────────────────────

bool ToneSequenceComponent::start_() {
  if (this->audio_source_ != nullptr) {
    return true;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();
  const size_t bpf = stream_info.frames_to_bytes(1);

  this->ring_buffer_.reset();
  const size_t rb_size = (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bpf) * bpf;
  std::shared_ptr<ring_buffer::RingBuffer> rb = ring_buffer::RingBuffer::create(rb_size);
  if (rb == nullptr) {
    this->status_momentary_error("ring_buffer", 15000);
    return false;
  }

  this->audio_source_ = audio::RingBufferAudioSource::create(rb, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS),
                                                             static_cast<uint8_t>(bpf));
  if (this->audio_source_ == nullptr) {
    this->status_momentary_error("audio_source", 15000);
    return false;
  }
  this->ring_buffer_ = rb;

  const uint32_t buf_cap = (3u * this->window_size_) / 2u;
  this->frame_buf_ = static_cast<int16_t *>(malloc(buf_cap * sizeof(int16_t)));
  if (this->frame_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    this->audio_source_.reset();
    this->ring_buffer_.reset();
    this->status_momentary_error("frame_buf", 15000);
    return false;
  }
  this->frame_scratch_ = static_cast<int16_t *>(malloc(this->window_size_ * sizeof(int16_t)));
  if (this->frame_scratch_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate frame scratch buffer");
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
    this->audio_source_.reset();
    this->ring_buffer_.reset();
    this->status_momentary_error("frame_scratch", 15000);
    return false;
  }
  this->frame_write_pos_ = 0;
  this->frame_read_pos_ = 0;
  this->frame_samples_avail_ = 0;

  this->status_clear_error();
  return true;
}

void ToneSequenceComponent::stop_() {
  this->audio_source_.reset();
  if (this->frame_buf_ != nullptr) {
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
  }
  if (this->frame_scratch_ != nullptr) {
    free(this->frame_scratch_);
    this->frame_scratch_ = nullptr;
  }
  this->frame_write_pos_ = 0;
  this->frame_read_pos_ = 0;
  this->frame_samples_avail_ = 0;
  this->hpf_x_prev_ = 0.0f;
  this->hpf_y_prev_ = 0.0f;
}

}  // namespace esphome::tone_sequence

#endif
