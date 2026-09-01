#include "frequency_engine.h"
#include "../dsp/goertzel.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome::frequency::core {

FrequencyEngine::FrequencyEngine(const FrequencyConfig &cfg) : cfg_(cfg) { ensure_buffers_(); }
FrequencyEngine::~FrequencyEngine() = default;

void FrequencyEngine::set_config(const FrequencyConfig &cfg) {
  cfg_ = cfg;
  ensure_buffers_();
  if (sample_rate_hz_ > 0) set_sample_rate(sample_rate_hz_);
}

void FrequencyEngine::set_window_size(uint16_t n) {
  cfg_.window_size = n;
  ensure_buffers_();
  if (sample_rate_hz_ > 0) set_sample_rate(sample_rate_hz_);
  reset_streaming_state();
}

void FrequencyEngine::set_band(float min_hz, float max_hz) {
  cfg_.min_frequency_hz = min_hz;
  cfg_.max_frequency_hz = max_hz;
  if (sample_rate_hz_ > 0) update_band_();
}

void FrequencyEngine::ensure_buffers_() {
  uint32_t n = cfg_.window_size;
  window_.assign(n, 0.0f);
  esphome::dsp::make_hann_window(window_.data(), n);
  uint32_t max_bins = n / 2;
  accum_.assign(max_bins, 0.0f);
  v1_.assign(max_bins, 0.0f);
  v2_.assign(max_bins, 0.0f);
  c2_.assign(max_bins, 0.0f);
  frame_buf_.assign(n + 1, 0);
  frame_buf_offset_ = 0;
  frame_count_ = 0;
  window_sample_count_ = 0;
  band_valid_ = false;
  num_bins_ = 0;
}

void FrequencyEngine::set_sample_rate(float fs) {
  sample_rate_hz_ = fs;
  if (cfg_.window_size == 0) return;
  update_band_();
}

void FrequencyEngine::update_band_() {
  uint32_t n = cfg_.window_size;
  float fs = sample_rate_hz_;
  if (fs <= 0 || n == 0) {
    band_valid_ = false;
    return;
  }
  uint32_t k_min = static_cast<uint32_t>(std::ceil((cfg_.min_frequency_hz * static_cast<double>(n)) / fs));
  uint32_t k_max = static_cast<uint32_t>((cfg_.max_frequency_hz * static_cast<double>(n) / fs));
  if (k_min < 1) k_min = 1;
  if (k_max > n / 2 - 1) k_max = n / 2 - 1;
  k_min_ = k_min;
  k_max_ = k_max;
  band_valid_ = (k_min < k_max);
  if (!band_valid_) {
    num_bins_ = 0;
    return;
  }
  num_bins_ = k_max - k_min + 1;
  for (uint32_t b = 0; b < num_bins_; ++b) {
    uint32_t k = k_min + b;
    float angle = 2.0f * (float)M_PI * (float)k / (float)n;
    c2_[b] = 2.0f * cosf(angle);
  }
  std::fill(accum_.begin(), accum_.end(), 0.0f);
  frame_count_ = 0;
  window_sample_count_ = 0;
}

void FrequencyEngine::reset_streaming_state() {
  frame_buf_offset_ = 0;
  frame_count_ = 0;
  window_sample_count_ = 0;
  std::fill(accum_.begin(), accum_.end(), 0.0f);
}

bool FrequencyEngine::process_goertzel_frame_(const int16_t *samples) {
  if (!band_valid_ || num_bins_ == 0) return false;
  esphome::dsp::goertzel_frame(samples, window_.data(), c2_.data(), v1_.data(), v2_.data(), accum_.data(),
                               cfg_.window_size, num_bins_);
  return true;
}

void FrequencyEngine::feed_pcm(const int16_t *samples, size_t count) {
  uint32_t n = cfg_.window_size;
  size_t pos = 0;
  while (pos < count) {
    if (frame_buf_offset_ >= n) {
      process_goertzel_frame_(frame_buf_.data());
      frame_buf_offset_ = 0;
      frame_count_++;
      window_sample_count_ += n;
    }
    uint32_t need = n - frame_buf_offset_;
    uint32_t take = std::min<uint32_t>(need, static_cast<uint32_t>(count - pos));
    std::memcpy(frame_buf_.data() + frame_buf_offset_, samples + pos, take * sizeof(int16_t));
    frame_buf_offset_ += take;
    pos += take;
  }
}

FrequencyResult FrequencyEngine::emit_window_() {
  FrequencyResult res{};
  uint32_t n = cfg_.window_size;
  uint32_t nb = num_bins_;
  if (frame_count_ > 0) {
    float inv = 1.0f / static_cast<float>(frame_count_);
    for (uint32_t b = 0; b < nb; ++b) accum_[b] *= inv;
  }
  uint32_t b_star = 0;
  float peak_p = 0.0f;
  if (band_valid_ && nb > 0) {
    b_star = 0;
    peak_p = accum_[0];
    for (uint32_t b = 1; b < nb; ++b) {
      if (accum_[b] > peak_p) {
        peak_p = accum_[b];
        b_star = b;
      }
    }
  }
  float ref = static_cast<float>(n);
  float peak_db = esphome::dsp::power_to_db(peak_p, ref);
  res.peak_db = peak_db;
  uint32_t k_star_abs = (band_valid_ ? k_min_ + b_star : 0);
  res.bin = k_star_abs;
  res.raw_bin_hz = (sample_rate_hz_ > 0 && n > 0) ? (static_cast<float>(k_star_abs) * sample_rate_hz_ / static_cast<float>(n)) : 0.0f;

  bool publish = false;
  float freq_hz = NAN;
  if (band_valid_ && peak_db >= cfg_.peak_threshold_db && nb > 0) {
    float alpha = 0.0f;
    if (b_star > 0 && b_star + 1 < nb) {
      float p_m1 = accum_[b_star - 1];
      float p_p1 = accum_[b_star + 1];
      alpha = esphome::dsp::parabolic_alpha(p_m1, peak_p, p_p1);
    }
    uint32_t k_star = k_min_ + b_star;
    freq_hz = (static_cast<float>(k_star) + alpha) * sample_rate_hz_ / static_cast<float>(n);
    publish = true;
  }
  res.valid = publish;
  res.frequency_hz = freq_hz;

  std::fill(accum_.begin(), accum_.end(), 0.0f);
  frame_count_ = 0;
  window_sample_count_ = 0;
  return res;
}

bool FrequencyEngine::try_emit_window(FrequencyResult &out_result) {
  if (frame_count_ == 0) return false;
  if (!band_valid_) {
    std::fill(accum_.begin(), accum_.end(), 0.0f);
    frame_count_ = 0;
    window_sample_count_ = 0;
    return false;
  }
  // Mirror frequency loop's emit condition:
  // emit if window_sample_count >= samples_in_window OR frame_count >= MAX_FFT_FRAMES
  // samples_in_window derived from measurement_duration_ms and sample_rate_hz_
  uint32_t samples_in_window = 0;
  if (sample_rate_hz_ > 0) {
    samples_in_window = static_cast<uint32_t>(sample_rate_hz_ * cfg_.measurement_duration_ms / 1000.0f);
  }
  bool should_emit = false;
  if (samples_in_window > 0 && window_sample_count_ >= samples_in_window) should_emit = true;
  if (frame_count_ >= MAX_FFT_FRAMES) should_emit = true;
  if (!should_emit) return false;
  out_result = emit_window_();
  return true;
}

std::vector<FrequencyResult> FrequencyEngine::run_offline(const float *mono, size_t num_samples, float fs) {
  set_sample_rate(fs);
  reset_streaming_state();
  std::vector<int16_t> pcm(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    int v = (int)std::lround(mono[i] * 32768.0f);
    if (v < -32768) v = -32768;
    if (v > 32767) v = 32767;
    pcm[i] = (int16_t)v;
  }
  std::vector<FrequencyResult> out;
  feed_pcm(pcm.data(), pcm.size());
  // Flush: if leftover partial frame, ignore; try to emit if enough frames
  FrequencyResult r;
  if (try_emit_window(r)) out.push_back(r);
  // For offline, emit as many windows as possible by feeding in chunks
  // The initial feed may have left pending frames not yet emitted due to window duration.
  // To mimic streaming, re-feed loop: already fed all, so emit remaining if band valid
  // but with current logic try_emit only emits when window full. Offline caller expects
  // multiple windows; handle by iterative emit without new samples is not needed.
  // If still has frame_count_ >0 and want to force emit, emit anyway:
  if (frame_count_ > 0 && band_valid_) {
    r = emit_window_();
    out.push_back(r);
  }
  return out;
}

}  // namespace esphome::frequency::core
