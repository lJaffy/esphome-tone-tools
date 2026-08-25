#include "chime.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace esphome::chime {

static const char *const TAG = "chime";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static const uint32_t MAX_FRAMES_PER_TICK = 16;

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────

void ChimeComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Chime Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Chimes: %lu\n"
                "  Total Goertzel filters: %lu",
                this->window_size_, this->tick_interval_ms_, (unsigned long) this->chimes_.size(),
                (unsigned long) this->total_filters_);

  for (size_t d = 0; d < this->chimes_.size(); ++d) {
    auto &c = this->chimes_[d];
    ESP_LOGCONFIG(TAG, "    Chime[%lu]:", (unsigned long) d);
    ESP_LOGCONFIG(TAG, "      Min Duration: %" PRIu32 " ms", c.min_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Max Duration: %" PRIu32 " ms", c.max_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Threshold: %.1f dB", c.threshold_db_);
    ESP_LOGCONFIG(TAG, "      Tail Grace: %" PRIu32 " ms", c.tail_grace_ms_);
    ESP_LOGCONFIG(TAG, "      Release Time: %" PRIu32 " ms (auto)", c.release_time_ms_);
    ESP_LOGCONFIG(TAG, "      Pattern (%lu steps):", (unsigned long) c.pattern_chords_.size());
    for (size_t s = 0; s < c.pattern_chords_.size(); ++s) {
      const auto &chord = c.pattern_chords_[s];
      const uint32_t t_ms = (s < c.pattern_times_ms_.size()) ? c.pattern_times_ms_[s] : NO_TIME;
      if (t_ms == NO_TIME) {
        ESP_LOGCONFIG(TAG, "        [%u] @ (any time):", (unsigned) s);
      } else {
        ESP_LOGCONFIG(TAG, "        [%u] @ %lu ms:", (unsigned) s, (unsigned long) t_ms);
      }
      for (size_t f = 0; f < chord.size(); ++f) {
        ESP_LOGCONFIG(TAG, "          %.1f Hz", chord[f]);
      }
    }
    if (c.detected_sensor_ != nullptr) {
      LOG_BINARY_SENSOR("      ", "Sensor:", c.detected_sensor_);
    }
  }
}

// ──────────────────────────────────────────────
//  Setup – one-time allocation
// ──────────────────────────────────────────────

void ChimeComponent::setup() {
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto rb = this->ring_buffer_.lock();
    if (rb != nullptr) {
      rb->write((void *) data.data(), data.size());
    }
  });

  if (this->chimes_.empty()) {
    ESP_LOGE(TAG, "No chimes configured – nothing to do");
    return;
  }

  this->build_frequency_map_();

  ESP_LOGI(TAG, "Global Goertzel filters: %lu", (unsigned long) this->total_filters_);

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

  const uint32_t nf = this->total_filters_;
  float *accum = static_cast<float *>(malloc(nf * sizeof(float)));
  float *v1 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *v2 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *c2 = static_cast<float *>(malloc(nf * sizeof(float)));
  float *spec = static_cast<float *>(malloc(nf * sizeof(float)));
  if (accum == nullptr || v1 == nullptr || v2 == nullptr || c2 == nullptr || spec == nullptr) {
    free(accum);
    free(v1);
    free(v2);
    free(c2);
    free(spec);
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
  memset(accum, 0, nf * sizeof(float));

  this->sample_rate_hz_ = 0.0f;
  this->dsp_ready_ = true;

  if (!this->microphone_source_->is_passive()) {
    this->microphone_source_->start();
  }
}

// ──────────────────────────────────────────────
//  Build global frequency union & per-chime index mapping
// ──────────────────────────────────────────────

void ChimeComponent::build_frequency_map_() {
  std::vector<float> all_freqs;

  auto add_unique = [&all_freqs](float f) {
    for (auto &existing : all_freqs) {
      if (std::fabs(existing - f) < 0.5f)
        return;
    }
    all_freqs.push_back(f);
  };

  for (auto &c : this->chimes_) {
    for (const auto &chord : c.pattern_chords_) {
      for (auto f : chord)
        add_unique(f);
    }
  }

  this->global_freqs_ = all_freqs;
  this->total_filters_ = all_freqs.size();

  auto find_global_idx = [&all_freqs](float f) -> uint32_t {
    for (uint32_t i = 0; i < all_freqs.size(); ++i) {
      if (std::fabs(all_freqs[i] - f) < 0.5f)
        return i;
    }
    return 0;
  };

  for (auto &c : this->chimes_) {
    c.chord_filter_indices_.clear();
    for (const auto &chord : c.pattern_chords_) {
      std::vector<uint32_t> indices;
      for (auto f : chord) {
        indices.push_back(find_global_idx(f));
      }
      c.chord_filter_indices_.push_back(indices);
    }
  }
}

// ──────────────────────────────────────────────
//  Main loop
// ──────────────────────────────────────────────

void ChimeComponent::loop() {
  if (!this->dsp_ready_ || this->window_ == nullptr || this->accum_ == nullptr || this->g_v1_ == nullptr ||
      this->g_v2_ == nullptr || this->g_c2_ == nullptr) {
    return;
  }

  bool any_sensor = false;
  for (auto &c : this->chimes_) {
    if (c.detected_sensor_ != nullptr) {
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
      for (auto &c : this->chimes_) {
        if (c.detected_sensor_ != nullptr)
          c.detected_sensor_->publish_state(false);
      }
    }
    return;
  }

  if (this->status_has_error()) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();

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

  if (this->frame_count_ > 0 &&
      (this->tick_sample_count_ >= samples_in_tick || this->frame_count_ >= MAX_FRAMES_PER_TICK)) {
    this->emit_tick_();
  }

  // ── Per-chime: release & max-duration deadline ──
  for (auto &c : this->chimes_) {
    if (c.detected_latched_ && millis() >= c.release_until_ms_) {
      c.detected_latched_ = false;
      if (c.detected_sensor_ != nullptr)
        c.detected_sensor_->publish_state(false);
      ESP_LOGD(TAG, "Chime[%lu] released after %" PRIu32 " ms hold",
               (unsigned long) &c - (unsigned long) &this->chimes_[0], c.release_time_ms_);
    }

    if (c.pattern_active_) {
      const uint32_t elapsed = millis() - c.pattern_start_ms_;
      if (elapsed > c.max_duration_ms_) {
        ESP_LOGD(TAG, "Chime[%lu] timed out after %" PRIu32 " ms (max %" PRIu32 " ms, matched %u/%lu)",
                 (unsigned long) &c - (unsigned long) &this->chimes_[0], (unsigned long) elapsed,
                 (unsigned long) c.max_duration_ms_, c.match_index_, (unsigned long) c.pattern_chords_.size());
        c.reset_pattern_();
        if (!c.detected_latched_ && c.detected_sensor_ != nullptr)
          c.detected_sensor_->publish_state(false);
      }
    }
  }
}

// ──────────────────────────────────────────────
//  Goertzel frame processing
// ──────────────────────────────────────────────

void ChimeComponent::process_frame_(const int16_t *samples) {
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
//  Tick emission
// ──────────────────────────────────────────────

void ChimeComponent::emit_tick_() {
  const uint32_t n = this->window_size_;
  const float ref = static_cast<float>(n);

  if (this->frame_count_ > 0) {
    const float inv = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t t = 0; t < this->total_filters_; ++t) {
      this->accum_[t] *= inv;
      const float p = this->accum_[t];
      this->spectrum_db_[t] = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
    }
  }

  for (auto &c : this->chimes_) {
    c.evaluate_pattern_(this->spectrum_db_);
  }

  memset(this->accum_, 0, this->total_filters_ * sizeof(float));
  this->frame_count_ = 0;
  this->tick_sample_count_ = 0;
}

// ──────────────────────────────────────────────
//  Per-chime chord detection
// ──────────────────────────────────────────────

bool Chime::chord_present_(const float *spectrum_db, uint8_t step, float &peak_db) const {
  const auto &indices = this->chord_filter_indices_[step];
  peak_db = -300.0f;

  for (size_t i = 0; i < indices.size(); ++i) {
    const float db = spectrum_db[indices[i]];
    if (db < this->threshold_db_) {
      return false;
    }
    if (db > peak_db)
      peak_db = db;
  }

  return true;
}

void Chime::log_chord_(uint8_t step) const {
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
//  Per-chime pattern state machine
// ──────────────────────────────────────────────

void Chime::evaluate_pattern_(const float *spectrum_db) {
  const uint32_t num_steps = static_cast<uint32_t>(this->pattern_chords_.size());
  const uint32_t elapsed = millis() - this->pattern_start_ms_;

  // ── IDLE – waiting for first chord ──
  if (!this->pattern_active_) {
    float peak = -300.0f;
    if (this->chord_present_(spectrum_db, 0, peak)) {
      this->pattern_active_ = true;
      this->match_index_ = 1;
      this->pattern_start_ms_ = millis();
      this->need_falling_edge_ = true;
      ESP_LOGI(TAG, "Chime pattern started: chord 1/%lu, peak %.1f dB", (unsigned long) num_steps, peak);
      this->log_chord_(0);
    }
    return;
  }

  // ── WAITING FOR FALLING EDGE ──
  if (this->need_falling_edge_) {
    const uint8_t prev_idx = static_cast<uint8_t>(this->match_index_ - 1);
    float prev_peak = -300.0f;
    const bool prev_still_present = this->chord_present_(spectrum_db, prev_idx, prev_peak);

    if (prev_still_present) {
      return;
    }

    this->need_falling_edge_ = false;
    ESP_LOGD(TAG, "Falling edge after chord %u/%lu at t=%" PRIu32 " ms",
             (unsigned) (prev_idx + 1), (unsigned long) num_steps, (unsigned) elapsed);
  }

  // ── ALL STEPS MATCHED ──
  if (this->match_index_ >= num_steps) {
    const uint32_t elapsed_now = millis() - this->pattern_start_ms_;
    if (elapsed_now >= this->min_duration_ms_) {
      this->latch_detection_(elapsed_now);
    } else {
      ESP_LOGD(TAG, "Chime discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms",
               (unsigned long) elapsed_now, (unsigned long) this->min_duration_ms_);
      this->reset_pattern_();
    }
    return;
  }

  // ── MATCHING – look for the next chord ──
  const uint32_t chord_idx = this->match_index_;
  const uint32_t t_chord = (chord_idx < this->pattern_times_ms_.size())
                               ? this->pattern_times_ms_[chord_idx]
                               : NO_TIME;

  // Time-boxed lower bound
  if (t_chord != NO_TIME && elapsed < t_chord) {
    return;
  }

  // Time-boxed upper bound
  if (t_chord != NO_TIME) {
    const uint32_t next_idx = chord_idx + 1;
    const uint32_t t_next = (next_idx < num_steps && next_idx < this->pattern_times_ms_.size())
                                ? this->pattern_times_ms_[next_idx]
                                : NO_TIME;
    const uint32_t window_end = (t_next != NO_TIME) ? t_next : (t_chord + this->tail_grace_ms_);

    if (elapsed >= window_end) {
      ESP_LOGW(TAG, "Chime timed out: chord %lu/%lu not detected in [%" PRIu32 ", %" PRIu32 "] ms (elapsed %" PRIu32 " ms)",
               (unsigned long) (chord_idx + 1), (unsigned long) num_steps,
               (unsigned) t_chord, (unsigned) window_end, (unsigned) elapsed);
      this->reset_pattern_();
      return;
    }
  }

  // Check chord presence
  float peak = -300.0f;
  if (this->chord_present_(spectrum_db, chord_idx, peak)) {
    if (t_chord != NO_TIME) {
      ESP_LOGD(TAG, "Chord %lu/%lu matched at t=%" PRIu32 " ms (from %" PRIu32 "), peak %.1f dB",
               (unsigned long) (chord_idx + 1), (unsigned long) num_steps, (unsigned) elapsed,
               (unsigned) t_chord, peak);
    } else {
      ESP_LOGD(TAG, "Chord %lu/%lu matched at t=%" PRIu32 " ms, peak %.1f dB",
               (unsigned long) (chord_idx + 1), (unsigned long) num_steps, (unsigned) elapsed, peak);
    }
    this->log_chord_(chord_idx);
    this->match_index_++;
    this->need_falling_edge_ = true;

    if (this->match_index_ >= num_steps) {
      const uint32_t elapsed_now = millis() - this->pattern_start_ms_;
      if (elapsed_now >= this->min_duration_ms_) {
        this->latch_detection_(elapsed_now);
      } else {
        ESP_LOGD(TAG, "Chime discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms",
                 (unsigned long) elapsed_now, (unsigned long) this->min_duration_ms_);
        this->reset_pattern_();
      }
    }
  }
}

void Chime::latch_detection_(uint32_t elapsed_ms) {
  ESP_LOGI(TAG, "CHIME DETECTED in %" PRIu32 " ms (%lu steps)", (unsigned long) elapsed_ms,
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

void Chime::reset_pattern_() {
  this->pattern_active_ = false;
  this->match_index_ = 0;
  this->need_falling_edge_ = false;
}

// ──────────────────────────────────────────────
//  Public start/stop
// ──────────────────────────────────────────────

void ChimeComponent::start() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot start microphone in passive mode");
    return;
  }
  this->microphone_source_->start();
}

void ChimeComponent::stop() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Cannot stop microphone in passive mode");
    return;
  }
  this->microphone_source_->stop();
}

// ──────────────────────────────────────────────
//  Internal buffer management
// ──────────────────────────────────────────────

bool ChimeComponent::start_() {
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

void ChimeComponent::stop_() {
  this->audio_source_.reset();
  if (this->frame_buf_ != nullptr) {
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
  }
  this->frame_buf_offset_ = 0;
}

}  // namespace esphome::chime

#endif