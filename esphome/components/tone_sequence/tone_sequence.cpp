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
static const uint32_t DIAG_INTERVAL_MS = 5000;

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────

void ToneSequenceComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tone Sequence Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Pattern Duration: %" PRIu32 " ms\n"
                "  Tolerance: ±%.1f Hz\n"
                "  Threshold: %.1f dB\n"
                "  Dominance: %.1f dB\n"
                "  Guard Offset: %" PRIu16 " Hz\n"
                "  Min Match Span: %" PRIu32 " ms\n"
                "  Release Time: %" PRIu32 " ms\n"
                "  Tones (%lu):",
                this->window_size_, this->tick_interval_ms_, this->pattern_duration_ms_, this->tolerance_hz_,
                this->threshold_db_, this->dominance_db_, this->guard_offset_hz_, this->min_match_span_ms_,
                this->release_time_ms_, (unsigned long) this->pattern_tones_.size());
  for (size_t i = 0; i < this->pattern_tones_.size(); ++i) {
    ESP_LOGCONFIG(TAG, "    [%u] = %.1f Hz", (unsigned) i, this->pattern_tones_[i]);
  }
  if (this->detected_sensor_ != nullptr) {
    LOG_BINARY_SENSOR("  ", "Detected:", this->detected_sensor_);
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

  const uint32_t n = this->window_size_;
  this->num_tones_ = this->pattern_tones_.size();
  if (this->num_tones_ == 0) {
    ESP_LOGE(TAG, "No tones configured – nothing to do");
    return;
  }

  // ── Build the guard-band frequency list ──
  // For each *unique* pattern tone, add ±guard_offset_hz references.
  this->guard_freqs_.clear();
  for (uint32_t i = 0; i < this->num_tones_; ++i) {
    const float f = this->pattern_tones_[i];
    // Skip if this exact frequency was already added
    bool seen = false;
    for (uint32_t j = 0; j < i; ++j) {
      if (this->pattern_tones_[j] == f) {
        seen = true;
        break;
      }
    }
    if (seen)
      continue;

    const float lo = f - this->guard_offset_hz_;
    const float hi = f + this->guard_offset_hz_;
    if (lo > 20.0f) {
      this->guard_freqs_.push_back(lo);
    }
    if (hi < 20000.0f) {
      this->guard_freqs_.push_back(hi);
    }
  }
  this->num_guards_ = this->guard_freqs_.size();
  this->total_filters_ = this->num_tones_ + this->num_guards_;

  ESP_LOGI(TAG, "Guard-band filters: %lu (offset ±%" PRIu16 " Hz)", (unsigned long) this->num_guards_,
           this->guard_offset_hz_);
  for (size_t g = 0; g < this->guard_freqs_.size(); ++g) {
    ESP_LOGD(TAG, "  guard[%lu] = %.1f Hz", (unsigned long) g, this->guard_freqs_[g]);
  }

  // ── Hann window (N floats) ──
  float *window = static_cast<float *>(malloc(n * sizeof(float)));
  if (window == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate window buffer (%" PRIu32 " bytes)", n * sizeof(float));
    return;
  }
  for (uint32_t i = 0; i < n; ++i) {
    window[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) i / (float) (n - 1)));
  }
  this->window_ = window;

  // ── Goertzel buffers (one slot per filter: tones + guards) ──
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
//  Main loop
// ──────────────────────────────────────────────

void ToneSequenceComponent::loop() {
  if (this->detected_sensor_ == nullptr) {
    return;
  }

  if (!this->dsp_ready_ || this->window_ == nullptr || this->accum_ == nullptr || this->g_v1_ == nullptr ||
      this->g_v2_ == nullptr || this->g_c2_ == nullptr) {
    return;
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
    if (this->detected_sensor_ != nullptr) {
      this->detected_sensor_->publish_state(false);
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

    // Pattern-tone coefficients (indices 0 .. num_tones_-1)
    for (uint32_t t = 0; t < this->num_tones_; ++t) {
      float freq = this->pattern_tones_[t];
      if (freq < 20.0f)
        freq = 20.0f;
      if (freq > nyquist - 20.0f)
        freq = nyquist - 20.0f;
      const float k = (freq * static_cast<float>(n)) / this->sample_rate_hz_;
      this->g_c2_[t] = 2.0f * cosf(2.0f * (float) M_PI * k / static_cast<float>(n));
    }

    // Guard-band coefficients (indices num_tones_ .. total_filters_-1)
    for (uint32_t g = 0; g < this->num_guards_; ++g) {
      float freq = this->guard_freqs_[g];
      if (freq < 20.0f)
        freq = 20.0f;
      if (freq > nyquist - 20.0f)
        freq = nyquist - 20.0f;
      const float k = (freq * static_cast<float>(n)) / this->sample_rate_hz_;
      this->g_c2_[this->num_tones_ + g] = 2.0f * cosf(2.0f * (float) M_PI * k / static_cast<float>(n));
    }

    ESP_LOGI(TAG, "Goertzel init: %lu tones + %lu guards, N=%" PRIu16 ", fs=%" PRIu32 " Hz",
             (unsigned long) this->num_tones_, (unsigned long) this->num_guards_, this->window_size_,
             (uint32_t) stream_info.get_sample_rate());
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

  // ── Release the latched detection after the hold period expires ──
  if (this->detected_latched_ && millis() >= this->release_until_ms_) {
    this->detected_latched_ = false;
    if (this->detected_sensor_ != nullptr) {
      this->detected_sensor_->publish_state(false);
    }
    ESP_LOGD(TAG, "Detection released after %" PRIu32 " ms hold", this->release_time_ms_);
  }

  // ── Pattern deadline check (runs every loop, not just on tick) ──
  if (this->pattern_active_) {
    const uint32_t elapsed = millis() - this->pattern_start_ms_;
    if (elapsed > this->pattern_duration_ms_) {
      ESP_LOGD(TAG, "Pattern timed out after %" PRIu32 " ms (matched %u/%lu)", (unsigned long) elapsed,
               this->match_index_, (unsigned long) this->num_tones_);
      this->reset_pattern_();
    }
  }
}

// ──────────────────────────────────────────────
//  Goertzel frame processing
// ──────────────────────────────────────────────

void ToneSequenceComponent::process_frame_(const int16_t *samples) {
  const uint32_t n = this->window_size_;
  const uint32_t nf = this->total_filters_;

  // Reset IIR state
  for (uint32_t t = 0; t < nf; ++t) {
    this->g_v1_[t] = 0.0f;
    this->g_v2_[t] = 0.0f;
  }

  // IIR recursion: v[n] = c2·v[n-1] - v[n-2] + x[n]
  for (uint32_t i = 0; i < n; ++i) {
    const float x = (static_cast<float>(samples[i]) / 32768.0f) * this->window_[i];
    for (uint32_t t = 0; t < nf; ++t) {
      const float v = this->g_c2_[t] * this->g_v1_[t] - this->g_v2_[t] + x;
      this->g_v2_[t] = this->g_v1_[t];
      this->g_v1_[t] = v;
    }
  }

  // Magnitude-squared: |X[k]|² = v1² + v2² - c2·v1·v2
  for (uint32_t t = 0; t < nf; ++t) {
    const float v1 = this->g_v1_[t];
    const float v2 = this->g_v2_[t];
    this->accum_[t] += v1 * v1 + v2 * v2 - this->g_c2_[t] * v1 * v2;
  }
}

// ──────────────────────────────────────────────
//  Tick emission & pattern evaluation
// ──────────────────────────────────────────────

void ToneSequenceComponent::emit_tick_() {
  const uint32_t n = this->window_size_;
  const float ref = static_cast<float>(n);

  // Average over the frames in this tick
  if (this->frame_count_ > 0) {
    const float inv = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t t = 0; t < this->total_filters_; ++t) {
      this->accum_[t] *= inv;
    }
  }

  // ── Find the strongest pattern tone ──
  float peak_db = -300.0f;
  uint32_t peak_idx = 0;
  for (uint32_t t = 0; t < this->num_tones_; ++t) {
    const float p = this->accum_[t];
    const float db = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
    if (db > peak_db) {
      peak_db = db;
      peak_idx = t;
    }
  }

  // ── Compute guard-band average ──
  float guard_sum = 0.0f;
  uint32_t guard_count = 0;
  for (uint32_t g = 0; g < this->num_guards_; ++g) {
    const float p = this->accum_[this->num_tones_ + g];
    if (p > 0.0f) {
      guard_sum += 10.0f * log10f(p / ref);
      guard_count++;
    }
  }
  const float guard_avg_db = (guard_count > 0) ? guard_sum / static_cast<float>(guard_count) : -300.0f;

  // ── Dominance test ──
  if (peak_db >= this->threshold_db_ && (peak_db - guard_avg_db) < this->dominance_db_) {
    ESP_LOGD(TAG, "Tick rejected: peak %.1f dB but only %.1f dB above guard avg (%.1f dB)", peak_db,
             peak_db - guard_avg_db, guard_avg_db);
    peak_db = -300.0f;
  }

  float dominant_hz = (peak_db > -300.0f) ? this->pattern_tones_[peak_idx] : 0.0f;

  // Feed the state machine
  this->evaluate_pattern_(dominant_hz, peak_db);

  // Reset for next tick
  memset(this->accum_, 0, this->total_filters_ * sizeof(float));
  this->frame_count_ = 0;
  this->tick_sample_count_ = 0;
}

// ──────────────────────────────────────────────
//  Pattern state machine
// ──────────────────────────────────────────────

void ToneSequenceComponent::evaluate_pattern_(float dominant_hz, float peak_db) {
  // ═══════════════════════════════════════════════════════════
  //  STATE: IDLE – waiting for the first tone in the sequence
  // ═══════════════════════════════════════════════════════════
  if (!this->pattern_active_) {
    if (peak_db >= this->threshold_db_ && std::fabs(dominant_hz - this->pattern_tones_[0]) <= this->tolerance_hz_) {
      this->pattern_active_ = true;
      this->match_index_ = 1;
      this->pattern_start_ms_ = millis();
      this->need_falling_edge_ = true;  // must drop out before tone 1 can match
      ESP_LOGI(TAG, "Pattern started: tone 1/%lu matched (%.1f Hz, %.1f dB)", (unsigned long) this->num_tones_,
               dominant_hz, peak_db);
    }
    return;
  }

  // ═══════════════════════════════════════════════════════════
  //  STATE: SPAN WAIT – all tones matched, waiting for min_match_span
  // ═══════════════════════════════════════════════════════════
  if (this->match_index_ >= this->num_tones_) {
    const uint32_t elapsed = millis() - this->pattern_start_ms_;
    if (elapsed >= this->min_match_span_ms_) {
      this->latch_detection_(elapsed);
    }
    // Note: we do NOT require the tone to still be present here.
    // The pattern has been fully validated (all tones + falling edges).
    // We're just waiting to confirm the total span meets the minimum.
    return;
  }

  // ═══════════════════════════════════════════════════════════
  //  STATE: WAITING FOR FALLING EDGE
  //  The previously-matched tone must drop out (silence or transition
  //  to a different frequency) before the next tone can register.
  // ═══════════════════════════════════════════════════════════
  if (this->need_falling_edge_) {
    // What frequency did we just match?
    const uint8_t prev_idx = static_cast<uint8_t>(this->match_index_ - 1);
    const float prev_freq = this->pattern_tones_[prev_idx];

    // Is the previously-matched tone still the dominant one?
    const bool prev_still_present =
        (peak_db >= this->threshold_db_) && (std::fabs(dominant_hz - prev_freq) <= this->tolerance_hz_);

    if (prev_still_present) {
      // Tone has not dropped yet – keep waiting. A constant drone
      // will be stuck here until pattern_duration times out.
      return;
    }

    // Falling edge detected! The tone either went silent or
    // transitioned to a different frequency.
    this->need_falling_edge_ = false;
    ESP_LOGD(TAG, "Falling edge detected after tone %u/%lu (now: %.1f dB, dominant %.1f Hz)", (unsigned) (prev_idx + 1),
             (unsigned long) this->num_tones_, peak_db, dominant_hz);

    // Don't return – fall through to check if the current tick
    // already contains the next expected tone (e.g. 500→1000 transition
    // with no silent gap in between).
  }

  // ═══════════════════════════════════════════════════════════
  //  STATE: MATCHING – waiting for the next tone (falling edge confirmed)
  // ═══════════════════════════════════════════════════════════
  if (peak_db < this->threshold_db_) {
    // Silence – simply wait for the next tone to appear
    return;
  }

  const float expected = this->pattern_tones_[this->match_index_];
  if (std::fabs(dominant_hz - expected) <= this->tolerance_hz_) {
    ESP_LOGD(TAG, "Tone %lu/%lu matched (%.1f Hz, %.1f dB)", (unsigned long) (this->match_index_ + 1),
             (unsigned long) this->num_tones_, dominant_hz, peak_db);
    this->match_index_++;
    this->need_falling_edge_ = true;  // require falling edge before the NEXT tone

    if (this->match_index_ >= this->num_tones_) {
      // All tones matched. Check if span is already satisfied.
      const uint32_t elapsed = millis() - this->pattern_start_ms_;
      if (elapsed >= this->min_match_span_ms_) {
        this->latch_detection_(elapsed);
      }
      // else: next tick enters the SPAN WAIT state
    }
  }
  // Wrong tone: ignored – do not reset, do not advance.
}

// ──────────────────────────────────────────────
//  Latch / reset helpers
// ──────────────────────────────────────────────

void ToneSequenceComponent::latch_detection_(uint32_t elapsed_ms) {
  ESP_LOGI(TAG, "PATTERN DETECTED in %" PRIu32 " ms (%lu tones)", (unsigned long) elapsed_ms,
           (unsigned long) this->num_tones_);

  this->detected_latched_ = true;
  this->release_until_ms_ = millis() + this->release_time_ms_;

  if (this->detected_sensor_ != nullptr) {
    this->detected_sensor_->publish_state(true);
  }

  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
}

void ToneSequenceComponent::reset_pattern_() {
  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
  if (!this->detected_latched_ && this->detected_sensor_ != nullptr) {
    this->detected_sensor_->publish_state(false);
  }
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
