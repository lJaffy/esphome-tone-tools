#include "tone_sequence.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <set>

namespace esphome::tone_sequence {

static const char *const TAG = "tone_sequence";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t MAX_FRAMES_PER_TICK = 16;
static const uint32_t DIAG_INTERVAL_MS = 5000;

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────

void ToneSequenceComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tone Sequence Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Detectors: %lu\n"
                "  Total Goertzel filters: %lu",
                this->window_size_, this->tick_interval_ms_, (unsigned long) this->detectors_.size(),
                (unsigned long) this->total_filters_);

  for (size_t d = 0; d < this->detectors_.size(); ++d) {
    auto &det = this->detectors_[d];
    ESP_LOGCONFIG(TAG, "    Detector[%lu]:", (unsigned long) d);
    ESP_LOGCONFIG(TAG, "      Min Duration: %" PRIu32 " ms", det.min_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Max Duration: %" PRIu32 " ms", det.max_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Tolerance: ±%.1f Hz", det.tolerance_hz_);
    ESP_LOGCONFIG(TAG, "      Threshold: %.1f dB", det.threshold_db_);
    ESP_LOGCONFIG(TAG, "      Dominance: %.1f dB", det.dominance_db_);
    ESP_LOGCONFIG(TAG, "      Guard Offset: %" PRIu16 " Hz", det.guard_offset_hz_);
    ESP_LOGCONFIG(TAG, "      Release Time: %" PRIu32 " ms", det.release_time_ms_);
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

  // ── Build each detector's guard frequencies (one pair per unique freq in all chords) ──
  for (auto &det : this->detectors_) {
    det.guard_freqs_.clear();
    for (const auto &chord : det.pattern_chords_) {
      for (float f : chord) {
        // Skip if this frequency already has guards (dedup across chords)
        bool seen = false;
        for (const auto &other_chord : det.pattern_chords_) {
          for (float existing : other_chord) {
            if (existing == f) {
              seen = true;
              break;
            }
            if (seen)
              break;
          }
          if (seen)
            break;
        }
        if (seen)
          continue;

        const float lo = f - det.guard_offset_hz_;
        const float hi = f + det.guard_offset_hz_;
        if (lo > 20.0f)
          det.guard_freqs_.push_back(lo);
        if (hi < 20000.0f)
          det.guard_freqs_.push_back(hi);
      }
    }
  }

  // ── Build the global frequency union ──
  this->build_frequency_map_();

  ESP_LOGI(TAG, "Global Goertzel filters: %lu", (unsigned long) this->total_filters_);

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
  if (accum == nullptr || v1 == nullptr || v2 == nullptr || c2 == nullptr) {
    free(accum);
    free(v1);
    free(v2);
    free(c2);
    free(window);
    this->window_ = nullptr;
    ESP_LOGE(TAG, "Failed to allocate Goertzel buffers");
    return;
  }

  this->accum_ = accum;
  this->g_v1_ = v1;
  this->g_v2_ = v2;
  this->g_c2_ = c2;
  memset(accum, 0, nf * sizeof(float));

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
  // Collect all unique frequencies (all chord tones + guards) across all detectors.
  std::vector<float> all_freqs;

  auto add_unique = [&all_freqs](float f) {
    for (auto &existing : all_freqs) {
      if (std::fabs(existing - f) < 0.5f)
        return;  // close enough
    }
    all_freqs.push_back(f);
  };

  for (auto &det : this->detectors_) {
    for (const auto &chord : det.pattern_chords_) {
      for (auto f : chord)
        add_unique(f);
    }
    for (auto f : det.guard_freqs_)
      add_unique(f);
  }

  this->global_freqs_ = all_freqs;
  this->total_filters_ = all_freqs.size();

  // For each detector, map its chord frequencies and guards to global indices.
  auto find_global_idx = [&all_freqs](float f) -> uint32_t {
    for (uint32_t i = 0; i < all_freqs.size(); ++i) {
      if (std::fabs(all_freqs[i] - f) < 0.5f)
        return i;
    }
    return 0;  // shouldn't happen
  };

  for (auto &det : this->detectors_) {
    det.chord_filter_indices_.clear();
    det.guard_filter_indices_.clear();

    // Per-chord mapping
    for (const auto &chord : det.pattern_chords_) {
      std::vector<uint32_t> indices;
      for (auto f : chord) {
        indices.push_back(find_global_idx(f));
      }
      det.chord_filter_indices_.push_back(indices);
    }

    // Guards
    for (auto f : det.guard_freqs_) {
      det.guard_filter_indices_.push_back(find_global_idx(f));
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

  // ── Stage samples and run Goertzel on each complete frame ──
  while (this->frame_count_ < MAX_FRAMES_PER_TICK && this->tick_sample_count_ < samples_in_tick) {
    if (this->frame_buf_offset_ >= this->window_size_) {
      this->process_frame_(this->frame_buf_);
      this->frame_buf_offset_ = 0;
      this->frame_count_++;
      this->tick_sample_count_ += this->window_size_;
    }

    this->audio_source_->fill(0, false);
    const uint32_t available = stream_info.bytes_to_samples(this->audio_source_->available());
    if (available == 0)
      break;

    const int16_t *data = reinterpret_cast<const int16_t *>(this->audio_source_->mutable_data());
    const uint32_t need = this->window_size_ - this->frame_buf_offset_;
    const uint32_t take = std::min(available, need);
    std::memcpy(this->frame_buf_ + this->frame_buf_offset_, data, stream_info.samples_to_bytes(take));
    this->audio_source_->consume(stream_info.samples_to_bytes(take));
    this->frame_buf_offset_ += take;
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

  // Average over frames in this tick
  if (this->frame_count_ > 0) {
    const float inv = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t t = 0; t < this->total_filters_; ++t) {
      this->accum_[t] *= inv;
    }
  }

  // Convert to dBFS spectrum (shared, read-only for detectors)
  float *spectrum_db = static_cast<float *>(malloc(this->total_filters_ * sizeof(float)));
  if (spectrum_db == nullptr) {
    memset(this->accum_, 0, this->total_filters_ * sizeof(float));
    this->frame_count_ = 0;
    this->tick_sample_count_ = 0;
    return;
  }

  for (uint32_t t = 0; t < this->total_filters_; ++t) {
    const float p = this->accum_[t];
    spectrum_db[t] = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
  }

  // Evaluate each detector against the shared spectrum
  for (auto &det : this->detectors_) {
    // Compute guard-band average for THIS detector
    float guard_sum = 0.0f;
    uint32_t guard_count = 0;
    for (auto gi : det.guard_filter_indices_) {
      const float db = spectrum_db[gi];
      if (db > -300.0f) {
        guard_sum += db;
        guard_count++;
      }
    }
    const float guard_avg_db = (guard_count > 0) ? guard_sum / static_cast<float>(guard_count) : -300.0f;

    // Feed the per-detector state machine (it inspects the spectrum directly)
    det.evaluate_pattern_(spectrum_db, this->total_filters_, guard_avg_db);
  }

  free(spectrum_db);

  // Reset for next tick
  memset(this->accum_, 0, this->total_filters_ * sizeof(float));
  this->frame_count_ = 0;
  this->tick_sample_count_ = 0;
}

// ──────────────────────────────────────────────
//  Per-detector chord detection helpers
// ──────────────────────────────────────────────

bool Detector::chord_present_(const float *spectrum_db, uint8_t step, float &peak_db, float guard_avg_db) const {
  const auto &indices = this->chord_filter_indices_[step];
  const auto &chord = this->pattern_chords_[step];
  peak_db = -300.0f;

  // Every frequency in the chord must be above threshold AND within tolerance
  // of the nominal frequency (the Goertzel bin is exact, but we still guard
  // against a filter landing in a noisy region).
  for (size_t i = 0; i < indices.size(); ++i) {
    const float db = spectrum_db[indices[i]];
    if (db < this->threshold_db_) {
      return false;  // one component missing → chord not present
    }
    if (db > peak_db)
      peak_db = db;
  }

  // Dominance: the chord peak must be at least dominance_db_ above guard average
  if ((peak_db - guard_avg_db) < this->dominance_db_) {
    return false;
  }

  return true;
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

void Detector::evaluate_pattern_(const float *spectrum_db, uint32_t num_global_filters, float guard_avg_db) {
  const uint32_t num_steps = static_cast<uint32_t>(this->pattern_chords_.size());

  // IDLE – waiting for first chord
  if (!this->pattern_active_) {
    float peak = -300.0f;
    if (this->chord_present_(spectrum_db, 0, peak, guard_avg_db)) {
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
    const bool prev_still_present = this->chord_present_(spectrum_db, prev_idx, prev_peak, guard_avg_db);

    if (prev_still_present) {
      return;  // previous chord still sounding – wait
    }

    this->need_falling_edge_ = false;
    ESP_LOGD(TAG, "Det falling edge after chord %u/%lu", (unsigned) (prev_idx + 1), (unsigned long) num_steps);
  }

  // MATCHING – look for the next chord
  if (this->match_index_ >= num_steps) {
    // All chords matched – handled in the matching block below (span check).
    // This line is unreachable in normal flow because we check after increment,
    // but kept for safety.
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
  if (this->chord_present_(spectrum_db, this->match_index_, peak, guard_avg_db)) {
    ESP_LOGD(TAG, "Det chord %lu/%lu matched, peak %.1f dB", (unsigned long) (this->match_index_ + 1),
             (unsigned long) num_steps, peak);
    this->log_chord_(this->match_index_);
    this->match_index_++;
    this->need_falling_edge_ = true;

    if (this->match_index_ >= num_steps) {
      // Full sequence matched – check the time span
      const uint32_t elapsed = millis() - this->pattern_start_ms_;
      if (elapsed >= this->min_duration_ms_) {
        this->latch_detection_(elapsed);
      } else {
        // Completed faster than min → too fast, discount
        ESP_LOGD(TAG, "Det pattern discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms",
                 (unsigned long) elapsed, (unsigned long) this->min_duration_ms_);
        this->reset_pattern_();
      }
    }
  }
  // If chord not present, just wait (deadline enforced in loop())
}

void Detector::latch_detection_(uint32_t elapsed_ms) {
  ESP_LOGI(TAG, "Det PATTERN DETECTED in %" PRIu32 " ms (%lu chord steps)", (unsigned long) elapsed_ms,
           (unsigned long) this->pattern_chords_.size());

  this->detected_latched_ = true;
  this->release_until_ms_ = millis() + this->release_time_ms_;

  if (this->detected_sensor_ != nullptr) {
    this->detected_sensor_->publish_state(true);
  }

  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
}

void Detector::reset_pattern_() {
  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
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

  this->frame_buf_ = static_cast<int16_t *>(malloc((this->window_size_ + 1) * sizeof(int16_t)));
  if (this->frame_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    this->audio_source_.reset();
    this->ring_buffer_.reset();
    this->status_momentary_error("frame_buf", 15000);
    return false;
  }

  this->status_clear_error();
  return true;
}

void ToneSequenceComponent::stop_() {
  this->audio_source_.reset();
  if (this->frame_buf_ != nullptr) {
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
  }
  this->frame_buf_offset_ = 0;
}

}  // namespace esphome::tone_sequence

#endif
