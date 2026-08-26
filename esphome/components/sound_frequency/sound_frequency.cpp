#include "sound_frequency.h"

#ifdef USE_ESP32

#include <sys/param.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstdint>
#include <cstring>

// Throttle for the periodic loop diagnostics (see loop()). The one-shot events
// (DSP init, band computation, first window emit) are not throttled.
static const uint32_t DIAGNOSTIC_LOG_INTERVAL_MS = 5000;

namespace esphome::sound_frequency {

static const char *const TAG = "sound_frequency";

static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
/// Cap on the number of Goertzel frames averaged into one measurement window (bounds CPU at high sample rates)
static const uint32_t MAX_FFT_FRAMES = 32;

void SoundFrequencyComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Sound Frequency Component:\n"
                "  Measurement Duration: %" PRIu32 " ms\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Min Frequency: %f Hz\n"
                "  Max Frequency: %f Hz\n"
                "  Peak Threshold: %f dB",
                this->measurement_duration_ms_, this->window_size_, this->min_frequency_hz_, this->max_frequency_hz_,
                this->peak_threshold_db_);
  LOG_SENSOR("  ", "Frequency:", this->frequency_sensor_);
  LOG_SENSOR("  ", "Peak Magnitude:", this->peak_magnitude_sensor_);
}

void SoundFrequencyComponent::setup() {
  this->microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto temp_ring_buffer = this->ring_buffer_.lock();
    if (temp_ring_buffer != nullptr) {
      temp_ring_buffer->write((void *) data.data(), data.size());
    }
  });

  const uint32_t n = this->window_size_;

  // Allocate and fill the Hann window. Computed directly – no esp_dsp needed.
  float *window = static_cast<float *>(malloc(n * sizeof(float)));
  if (window == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate window buffer (%" PRIu32 " bytes)", n * sizeof(float));
    return;
  }
  for (uint32_t i = 0; i < n; ++i) {
    window[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * (float) i / (float) (n - 1)));
  }
  this->window_ = window;

  // Allocate Goertzel buffers at worst-case size (N/2 bins). Only the first
  // num_bins_ entries will actually be used once the band is known.
  const uint32_t max_bins = n / 2;

  float *accum = static_cast<float *>(malloc(max_bins * sizeof(float)));
  float *v1 = static_cast<float *>(malloc(max_bins * sizeof(float)));
  float *v2 = static_cast<float *>(malloc(max_bins * sizeof(float)));
  float *c2 = static_cast<float *>(malloc(max_bins * sizeof(float)));
  if (accum == nullptr || v1 == nullptr || v2 == nullptr || c2 == nullptr) {
    free(accum);
    free(v1);
    free(v2);
    free(c2);
    free(window);
    ESP_LOGE(TAG, "Failed to allocate Goertzel buffers");
    return;
  }

  this->accum_ = accum;
  this->goertzel_v1_ = v1;
  this->goertzel_v2_ = v2;
  this->goertzel_c2_ = c2;
  this->num_bins_ = 0;

  memset(this->accum_, 0, max_bins * sizeof(float));

  this->dsp_initialized_ = true;

  if (!this->microphone_source_->is_passive()) {
    // Automatically start the microphone if not in passive mode
    this->microphone_source_->start();
  }
}

void SoundFrequencyComponent::loop() {
  if ((this->frequency_sensor_ == nullptr) && (this->peak_magnitude_sensor_ == nullptr)) {
    return;
  }

  if (!this->dsp_initialized_ || this->window_ == nullptr || this->accum_ == nullptr || this->goertzel_v1_ == nullptr ||
      this->goertzel_v2_ == nullptr || this->goertzel_c2_ == nullptr) {
    return;
  }

  if (this->microphone_source_->is_running() && !this->status_has_error()) {
    if (this->start_()) {
      this->status_clear_warning();
    } else {
      ESP_LOGW(TAG, "Internal buffers failed to allocate");
      return;
    }
  } else {
    if (!this->status_has_warning()) {
      this->status_set_warning(LOG_STR("Microphone is not running, can't compute dominant frequency"));

      this->stop_();

      if (this->frequency_sensor_ != nullptr) {
        this->frequency_sensor_->publish_state(NAN);
      }
      if (this->peak_magnitude_sensor_ != nullptr) {
        this->peak_magnitude_sensor_->publish_state(NAN);
      }

      memset(this->accum_, 0, this->num_bins_ * sizeof(float));
      this->frame_count_ = 0;
      this->window_sample_count_ = 0;
      this->frame_buf_offset_ = 0;
    }

    return;
  }

  if (this->status_has_error()) {
    return;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();

  // The sample rate is only known at runtime, so the in-band bin range and
  // Goertzel coefficients are computed here (once, until valid).
  if (!this->band_valid_) {
    const float fs = static_cast<float>(stream_info.get_sample_rate());
    this->sample_rate_hz_ = fs;

    const uint32_t n = this->window_size_;
    uint32_t k_min = static_cast<uint32_t>(std::ceil((this->min_frequency_hz_ * static_cast<double>(n)) / fs));
    uint32_t k_max = static_cast<uint32_t>((this->max_frequency_hz_ * static_cast<double>(n) / fs));
    if (k_min < 1) {
      k_min = 1;
    }
    if (k_max > n / 2 - 1) {
      k_max = n / 2 - 1;
    }

    this->k_min_ = k_min;
    this->k_max_ = k_max;
    this->band_valid_ = (k_min < k_max);

    if (!this->band_valid_) {
      ESP_LOGW(TAG,
               "Frequency band %.0f-%.0f Hz is outside the analyzable range for a sample rate of %" PRIu32
               " Hz and window size %" PRIu32,
               this->min_frequency_hz_, this->max_frequency_hz_, stream_info.get_sample_rate(), n);
    } else {
      this->num_bins_ = k_max - k_min + 1;

      // Precompute 2*cos(2*pi*k/N) for each target bin (the IIR coefficient)
      for (uint32_t b = 0; b < this->num_bins_; ++b) {
        const uint32_t k = k_min + b;
        const float angle = 2.0f * (float) M_PI * (float) k / (float) n;
        this->goertzel_c2_[b] = 2.0f * cosf(angle);
      }

      ESP_LOGW(TAG, "Goertzel band %.0f-%.0f Hz -> bins %lu-%lu (%lu bins, sample rate %" PRIu32 " Hz, N=%" PRIu32 ")",
               this->min_frequency_hz_, this->max_frequency_hz_, static_cast<unsigned long>(k_min),
               static_cast<unsigned long>(k_max), static_cast<unsigned long>(this->num_bins_),
               stream_info.get_sample_rate(), n);
    }
  }

  const uint32_t samples_in_window = stream_info.ms_to_samples(this->measurement_duration_ms_);

  // Stage samples into frame_buf_ and run Goertzel on each complete N-sample frame.
  while (this->frame_count_ < MAX_FFT_FRAMES && this->window_sample_count_ < samples_in_window) {
    if (this->frame_buf_offset_ >= this->window_size_) {
      this->process_goertzel_frame_(this->frame_buf_);
      this->frame_buf_offset_ = 0;
      this->frame_count_++;
      this->window_sample_count_ += this->window_size_;
    }

    this->audio_source_->fill(0, false);

    const uint32_t samples_available = stream_info.bytes_to_samples(this->audio_source_->available());
    if (samples_available == 0) {
      break;
    }
    const int16_t *data = reinterpret_cast<const int16_t *>(this->audio_source_->mutable_data());

    const uint32_t copies_needed = this->window_size_ - this->frame_buf_offset_;
    const uint32_t copies_allowed = std::min(samples_available, copies_needed);
    std::memcpy(this->frame_buf_ + this->frame_buf_offset_, data, stream_info.samples_to_bytes(copies_allowed));
    this->audio_source_->consume(stream_info.samples_to_bytes(copies_allowed));
    this->frame_buf_offset_ += copies_allowed;
  }

  if (this->frame_count_ > 0) {
    if (this->band_valid_) {
      if (this->window_sample_count_ >= samples_in_window || this->frame_count_ >= MAX_FFT_FRAMES) {
        ESP_LOGW(TAG, "Window emit: %" PRIu32 " frames, %" PRIu32 "/%" PRIu32 " samples (%" PRIu32 " ms target)",
                 this->frame_count_, this->window_sample_count_, samples_in_window, this->measurement_duration_ms_);
        this->emit_window_();
      }
    } else {
      memset(this->accum_, 0, this->num_bins_ * sizeof(float));
      this->frame_count_ = 0;
      this->window_sample_count_ = 0;
    }
  }
}

bool SoundFrequencyComponent::process_goertzel_frame_(const int16_t *samples) {
  const uint32_t n = this->window_size_;
  const uint32_t nb = this->num_bins_;

  // Reset Goertzel state for this frame
  for (uint32_t b = 0; b < nb; ++b) {
    this->goertzel_v1_[b] = 0.0f;
    this->goertzel_v2_[b] = 0.0f;
  }

  // Run the IIR recursion: v[n] = c2 * v[n-1] - v[n-2] + x[n]
  // Process sample-by-sample across all bins for cache-friendly access to the
  // windowed input, while bin state lives in a small contiguous array.
  for (uint32_t i = 0; i < n; ++i) {
    const float x = static_cast<float>(samples[i]) / 32768.0f * this->window_[i];
    for (uint32_t b = 0; b < nb; ++b) {
      const float v = this->goertzel_c2_[b] * this->goertzel_v1_[b] - this->goertzel_v2_[b] + x;
      this->goertzel_v2_[b] = this->goertzel_v1_[b];
      this->goertzel_v1_[b] = v;
    }
  }

  // Optimized magnitude-squared (no complex arithmetic):
  //   |X[k]|^2 = v1^2 + v2^2 - c2 * v1 * v2
  // Accumulate power (periodogram averaging over the measurement window)
  for (uint32_t b = 0; b < nb; ++b) {
    const float v1 = this->goertzel_v1_[b];
    const float v2 = this->goertzel_v2_[b];
    const float mag2 = v1 * v1 + v2 * v2 - this->goertzel_c2_[b] * v1 * v2;
    this->accum_[b] += mag2;
  }

  return true;
}

void SoundFrequencyComponent::emit_window_() {
  const uint32_t n = this->window_size_;
  const uint32_t nb = this->num_bins_;

  // Average the accumulated power over the frames collected for this window
  if (this->frame_count_ > 0) {
    const float inv_frames = 1.0f / static_cast<float>(this->frame_count_);
    for (uint32_t b = 0; b < nb; ++b) {
      this->accum_[b] *= inv_frames;
    }
  }

  // Peak-pick the strongest in-band bin
  uint32_t b_star = 0;
  float peak_p = 0.0f;
  if (this->band_valid_ && nb > 0) {
    b_star = 0;
    peak_p = this->accum_[0];
    for (uint32_t b = 1; b < nb; ++b) {
      if (this->accum_[b] > peak_p) {
        peak_p = this->accum_[b];
        b_star = b;
      }
    }
  }

  // Approximate dBFS for a bin-centered full-scale sine (Hann window halves in-band energy)
  const float ref = static_cast<float>(n);
  const float peak_db = (peak_p > 0.0f) ? 10.0f * log10f(peak_p / ref) : -300.0f;

  if (this->peak_magnitude_sensor_ != nullptr) {
    this->peak_magnitude_sensor_->publish_state(peak_db);
  }

  bool publish_frequency = false;
  float frequency_hz = NAN;

  if (this->band_valid_ && peak_db >= this->peak_threshold_db_ && nb > 0) {
    // Sub-bin refinement: parabolic fit on log-magnitude around the peak.
    // Requires the two adjacent bins, so skip if the peak is at the edge of
    // the evaluated range.
    float alpha = 0.0f;
    if (b_star > 0 && b_star + 1 < nb) {
      const float p_m1 = this->accum_[b_star - 1];
      const float p_p1 = this->accum_[b_star + 1];
      if (p_m1 > 0.0f && peak_p > 0.0f && p_p1 > 0.0f) {
        const float db_m1 = 0.5f * log10f(p_m1);
        const float db_0 = 0.5f * log10f(peak_p);
        const float db_p1 = 0.5f * log10f(p_p1);
        const float denom = db_m1 - 2.0f * db_0 + db_p1;
        if (denom != 0.0f) {
          alpha = 0.5f * (db_m1 - db_p1) / denom;
          if (alpha > 0.5f) {
            alpha = 0.5f;
          } else if (alpha < -0.5f) {
            alpha = -0.5f;
          }
        }
      }
    }

    const uint32_t k_star = this->k_min_ + b_star;
    frequency_hz = (static_cast<float>(k_star) + alpha) * this->sample_rate_hz_ / static_cast<float>(n);
    publish_frequency = true;
  }

  if (this->frequency_sensor_ != nullptr) {
    this->frequency_sensor_->publish_state(publish_frequency ? frequency_hz : NAN);
  }

  const uint32_t k_star_abs = this->k_min_ + b_star;
  ESP_LOGW(TAG, "Window result: bin %lu (%.1f Hz), peak %.2f dB vs threshold %.1f dB -> %s",
           static_cast<unsigned long>(k_star_abs),
           (static_cast<float>(k_star_abs) * this->sample_rate_hz_) / static_cast<float>(n), peak_db,
           this->peak_threshold_db_, publish_frequency ? "publish" : "suppress");

  // Reset accumulators for the next measurement window
  memset(this->accum_, 0, nb * sizeof(float));
  this->frame_count_ = 0;
  this->window_sample_count_ = 0;
}

void SoundFrequencyComponent::start() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't start the microphone in passive mode");
    return;
  }
  this->microphone_source_->start();
}

void SoundFrequencyComponent::stop() {
  if (this->microphone_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't stop the microphone in passive mode");
    return;
  }
  this->microphone_source_->stop();
}

bool SoundFrequencyComponent::start_() {
  if (this->audio_source_ != nullptr) {
    return true;
  }

  const auto &stream_info = this->microphone_source_->get_audio_stream_info();
  const size_t bytes_per_frame = stream_info.frames_to_bytes(1);

  this->ring_buffer_.reset();
  const size_t ring_buffer_size =
      (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bytes_per_frame) * bytes_per_frame;
  std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer = ring_buffer::RingBuffer::create(ring_buffer_size);
  if (temp_ring_buffer == nullptr) {
    this->status_momentary_error("ring_buffer", 15000);
    return false;
  }

  this->audio_source_ = audio::RingBufferAudioSource::create(
      temp_ring_buffer, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS), static_cast<uint8_t>(bytes_per_frame));
  if (this->audio_source_ == nullptr) {
    this->status_momentary_error("audio_source", 15000);
    return false;
  }

  this->ring_buffer_ = temp_ring_buffer;

  this->frame_buf_ = static_cast<int16_t *>(malloc((this->window_size_ + 1) * sizeof(int16_t)));
  if (this->frame_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate Goertzel frame buffer");
    this->audio_source_.reset();
    this->ring_buffer_.reset();
    this->status_momentary_error("frame_buf", 15000);
    return false;
  }

  this->status_clear_error();
  return true;
}

void SoundFrequencyComponent::stop_() {
  this->audio_source_.reset();
  if (this->frame_buf_ != nullptr) {
    free(this->frame_buf_);
    this->frame_buf_ = nullptr;
  }
  this->frame_buf_offset_ = 0;
}

}  // namespace esphome::sound_frequency

#endif
