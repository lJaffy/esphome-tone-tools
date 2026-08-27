#include "chime_engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome::chime::core {

ChimeEngine::ChimeEngine(const DetectorConfig &cfg) : det_cfg_(cfg) {}
ChimeEngine::~ChimeEngine() = default;

void ChimeEngine::set_detector_config(const DetectorConfig &cfg) {
  det_cfg_ = cfg;
  if (sample_rate_hz_ > 0) set_sample_rate(sample_rate_hz_);
}

void ChimeEngine::add_chime(const ChimeConfig &cfg) {
  ChimePattern p(cfg);
  chimes_.push_back(std::move(p));
}

void ChimeEngine::clear_chimes() {
  chimes_.clear();
  global_freqs_.clear();
  effective_freqs_.clear();
  total_filters_ = 0;
}

void ChimeEngine::build_frequency_map() {
  std::vector<float> all_freqs;
  auto add_unique = [&all_freqs](float f) {
    for (auto &existing : all_freqs)
      if (std::fabs(existing - f) < 0.5f) return;
    all_freqs.push_back(f);
  };
  for (auto &c : chimes_) {
    for (const auto &chord : c.config().pattern) {
      for (auto f : chord) add_unique(f);
    }
  }
  global_freqs_ = all_freqs;
  effective_freqs_ = all_freqs;
  total_filters_ = static_cast<uint32_t>(all_freqs.size());

  auto find_global_idx = [&all_freqs](float f) -> uint32_t {
    for (uint32_t i = 0; i < all_freqs.size(); ++i)
      if (std::fabs(all_freqs[i] - f) < 0.5f) return i;
    return 0;
  };
  for (auto &c : chimes_) {
    c.chord_filter_indices.clear();
    for (const auto &chord : c.config().pattern) {
      std::vector<uint32_t> indices;
      for (auto f : chord) indices.push_back(find_global_idx(f));
      c.chord_filter_indices.push_back(indices);
    }
  }

  // Allocate/resize DSP vectors to new nf, and rebuild window / sample-rate dep state if already known
  window_.assign(det_cfg_.window_size, 0.0f);
  for (uint32_t i = 0; i < det_cfg_.window_size; ++i) {
    window_[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(det_cfg_.window_size - 1)));
  }
  ensure_buffers_();
  if (sample_rate_hz_ > 0) set_sample_rate(sample_rate_hz_);
  reset_streaming_state();
}

void ChimeEngine::ensure_buffers_() {
  uint32_t nf = total_filters_;
  accum_.assign(nf, 0.0f);
  g_v1_.assign(nf, 0.0f);
  g_v2_.assign(nf, 0.0f);
  g_c2_.assign(nf, 0.0f);
  spectrum_db_.assign(nf, -300.0f);
  noise_floor_.assign(nf, -300.0f);
  noise_floor_init_.assign(nf, -300.0f);
  local_bg_db_.assign(nf, -300.0f);
  onset_history_.assign(nf * ONSET_LOOKBACK, 0.0f);
  onset_count_.assign(nf, 0);
  onset_contrast_.assign(nf, 0.0f);
  onset_ring_pos_ = 0;
  noise_floor_ready_ = false;
}

void ChimeEngine::set_sample_rate(float fs) {
  sample_rate_hz_ = fs;
  if (global_freqs_.empty() || total_filters_ == 0) return;
  uint32_t n = det_cfg_.window_size;
  float nyquist = fs / 2.0f;
  effective_freqs_.resize(total_filters_);
  for (uint32_t t = 0; t < total_filters_; ++t) {
    float freq = global_freqs_[t];
    if (freq < 20.0f) freq = 20.0f;
    if (freq > nyquist - 20.0f) freq = nyquist - 20.0f;
    effective_freqs_[t] = freq;
    float k = (freq * static_cast<float>(n)) / fs;
    g_c2_[t] = 2.0f * cosf(2.0f * (float)M_PI * k / static_cast<float>(n));
  }
  // Rebuild window in case window_size changed
  window_.assign(n, 0.0f);
  for (uint32_t i = 0; i < n; ++i) {
    window_[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1)));
  }
  dsp_ready_ = true;
}

void ChimeEngine::reset_streaming_state() {
  uint32_t n = det_cfg_.window_size;
  frame_buf_.assign(n + 1, 0);
  frame_buf_offset_ = 0;
  frame_count_ = 0;
  tick_sample_count_ = 0;
  // Onset history cleared on fresh mic start (mirrors ChimeComponent::start_())
  std::fill(onset_count_.begin(), onset_count_.end(), 0);
  std::fill(onset_history_.begin(), onset_history_.end(), 0.0f);
  std::fill(onset_contrast_.begin(), onset_contrast_.end(), 0.0f);
  onset_ring_pos_ = 0;
  std::fill(accum_.begin(), accum_.end(), 0.0f);
}

void ChimeEngine::reset_onset_history() {
  std::fill(onset_count_.begin(), onset_count_.end(), 0);
  onset_ring_pos_ = 0;
}

void ChimeEngine::feed_pcm(const int16_t *samples, size_t count) {
  // Streaming feeder used by ESP glue in a loop; we stage into frame_buf_ and
  // process full windows. The outer loop in ChimeComponent::loop still governs
  // tick_sample_count / framesPerTick gating – this just stages.
  size_t pos = 0;
  uint32_t n = det_cfg_.window_size;
  while (pos < count) {
    // If frame_buf is full, caller should have called try_emit_tick / processed
    if (frame_buf_offset_ >= n) {
      // Caller must drain; we process here so feed never overflows
      process_frame_(frame_buf_.data());
      frame_buf_offset_ = 0;
      frame_count_++;
      tick_sample_count_ += n;
    }
    uint32_t need = n - frame_buf_offset_;
    uint32_t take = std::min<uint32_t>(need, static_cast<uint32_t>(count - pos));
    std::memcpy(frame_buf_.data() + frame_buf_offset_, samples + pos, take * sizeof(int16_t));
    frame_buf_offset_ += take;
    pos += take;
  }
}

void ChimeEngine::process_frame_(const int16_t *samples) {
  uint32_t n = det_cfg_.window_size;
  uint32_t nf = total_filters_;
  for (uint32_t t = 0; t < nf; ++t) { g_v1_[t] = 0.0f; g_v2_[t] = 0.0f; }
  for (uint32_t i = 0; i < n; ++i) {
    const float x = (static_cast<float>(samples[i]) / 32768.0f) * window_[i];
    for (uint32_t t = 0; t < nf; ++t) {
      const float v = g_c2_[t] * g_v1_[t] - g_v2_[t] + x;
      g_v2_[t] = g_v1_[t];
      g_v1_[t] = v;
    }
  }
  for (uint32_t t = 0; t < nf; ++t) {
    const float v1 = g_v1_[t], v2 = g_v2_[t];
    accum_[t] += v1 * v1 + v2 * v2 - g_c2_[t] * v1 * v2;
  }
}

void ChimeEngine::compute_local_background_() {
  uint32_t nf = total_filters_;
  for (uint32_t i = 0; i < nf; ++i) {
    float min_db = 300.0f;
    for (uint32_t j = 0; j < nf; ++j) {
      if (i == j) continue;
      if (std::fabs(effective_freqs_[i] - effective_freqs_[j]) < det_cfg_.guard_separation_hz) continue;
      if (spectrum_db_[j] < min_db) min_db = spectrum_db_[j];
    }
    if (min_db > 299.0f) min_db = *std::min_element(spectrum_db_.begin(), spectrum_db_.end());
    local_bg_db_[i] = min_db;
  }
}

bool ChimeEngine::bin_is_busy_(uint32_t filter_idx) const {
  for (const auto &c : chimes_) {
    if (!c.pattern_active) continue;
    for (const auto &chord_indices : c.chord_filter_indices)
      for (uint32_t idx : chord_indices)
        if (idx == filter_idx) return true;
  }
  return false;
}

uint32_t ChimeEngine::frames_per_tick_() const {
  if (sample_rate_hz_ <= 0) return 1;
  float samples_in_tick = sample_rate_hz_ * det_cfg_.tick_interval_ms / 1000.0f;
  uint32_t n = det_cfg_.window_size;
  uint32_t fpt = static_cast<uint32_t>(std::ceil(samples_in_tick / n - 1e-9f));
  if (fpt < 1) fpt = 1;
  if (fpt > MAX_FRAMES_PER_TICK) fpt = MAX_FRAMES_PER_TICK;
  return fpt;
}

float ChimeEngine::tick_audio_ms_() const {
  if (sample_rate_hz_ <= 0) return (float)det_cfg_.tick_interval_ms;
  return (frames_per_tick_() * det_cfg_.window_size) / sample_rate_hz_ * 1000.0f;
}

bool ChimeEngine::try_emit_tick(uint32_t now_ms, std::vector<Event> &out_events, TickSnapshot *out_tick) {
  if (frame_count_ == 0) return false;
  // Check if tick window full or frame cap hit
  uint32_t samples_in_tick = 0;
  if (sample_rate_hz_ > 0) samples_in_tick = static_cast<uint32_t>(sample_rate_hz_ * det_cfg_.tick_interval_ms / 1000.0f);
  bool full = (samples_in_tick > 0 && tick_sample_count_ >= samples_in_tick) ||
              (frame_count_ >= MAX_FRAMES_PER_TICK);
  if (!full) return false;

  // --- emit tick (mirrors ChimeComponent::emit_tick_) ---
  uint32_t n = det_cfg_.window_size;
  float ref = static_cast<float>(n);
  float inv = 1.0f / static_cast<float>(frame_count_);
  for (uint32_t t = 0; t < total_filters_; ++t) {
    accum_[t] *= inv;
    float p = accum_[t];
    spectrum_db_[t] = (p > 0.0f) ? 10.0f * log10f(p / ref) : -300.0f;
  }

  compute_local_background_();

  for (uint32_t t = 0; t < total_filters_; ++t) {
    onset_history_[t * ONSET_LOOKBACK + onset_ring_pos_] = spectrum_db_[t];
    if (onset_count_[t] < ONSET_LOOKBACK) onset_count_[t]++;
  }
  onset_ring_pos_ = (onset_ring_pos_ + 1) % ONSET_LOOKBACK;
  for (uint32_t t = 0; t < total_filters_; ++t) {
    uint32_t nvalid = std::min<uint32_t>(onset_count_[t], ONSET_LOOKBACK);
    uint32_t pos = onset_ring_pos_;
    float min_db = 300.0f;
    for (uint32_t i = 0; i < nvalid; ++i) {
      uint32_t idx = (pos + ONSET_LOOKBACK - 1 - i) % ONSET_LOOKBACK;
      if (onset_history_[t * ONSET_LOOKBACK + idx] < min_db) min_db = onset_history_[t * ONSET_LOOKBACK + idx];
    }
    if (min_db > 299.0f) min_db = spectrum_db_[t];
    onset_contrast_[t] = spectrum_db_[t] - min_db;
  }

  bool floor_was_ready = noise_floor_ready_;
  if (!noise_floor_ready_) {
    noise_floor_ = spectrum_db_;
    noise_floor_init_ = spectrum_db_;
    noise_floor_ready_ = true;
  } else {
    for (uint32_t t = 0; t < total_filters_; ++t) {
      if (bin_is_busy_(t)) continue;
      float cur = spectrum_db_[t];
      float fl = noise_floor_[t];
      noise_floor_[t] = cur < fl ? fl + det_cfg_.noise_floor_alpha_down * (cur - fl)
                                 : fl + det_cfg_.noise_floor_alpha_up * (cur - fl);
    }
  }

  const float *eval_floor = floor_was_ready ? noise_floor_.data() : nullptr;
  uint32_t tick_idx = debug_tick_count_;  // for event tick_index
  for (size_t ci = 0; ci < chimes_.size(); ++ci) {
    auto &c = chimes_[ci];
    const float *eval_onset = (c.config().onset_contrast_db > 0.0f) ? onset_contrast_.data() : nullptr;
    c.evaluate_pattern(now_ms, spectrum_db_.data(), eval_floor, local_bg_db_.data(), eval_onset,
                       static_cast<uint32_t>(ci), tick_idx,
                       [&](const Event &ev) { out_events.push_back(ev); });
  }

  // Release & max-duration deadlines (mirrors ChimeComponent::loop after emit)
  for (size_t ci = 0; ci < chimes_.size(); ++ci) {
    auto &c = chimes_[ci];
    if (c.detected_latched && now_ms >= c.release_until_ms) {
      c.detected_latched = false;
      Event ev;
      ev.type = EventType::Released;
      ev.chime_index = static_cast<uint32_t>(ci);
      ev.tick_index = tick_idx;
      ev.now_ms = now_ms;
      ev.hold_ms = c.config().release_time_ms;
      out_events.push_back(ev);
    }
    if (c.pattern_active) {
      uint32_t elapsed = now_ms - c.pattern_start_ms;
      if (elapsed > c.config().max_duration_ms) {
        Event ev;
        ev.type = EventType::MaxDurationTimeout;
        ev.chime_index = static_cast<uint32_t>(ci);
        ev.tick_index = tick_idx;
        ev.now_ms = now_ms;
        ev.elapsed_ms = elapsed;
        ev.max_ms = c.config().max_duration_ms;
        ev.matched = c.match_index;
        ev.num_steps = static_cast<uint32_t>(c.config().pattern.size());
        out_events.push_back(ev);
        c.reset_pattern();
      }
    }
  }

  if (out_tick != nullptr) {
    out_tick->tick = tick_idx;
    out_tick->ms = now_ms;
    out_tick->duration_ms = tick_audio_ms_();
    out_tick->bins.resize(total_filters_);
    for (uint32_t t = 0; t < total_filters_; ++t) {
      out_tick->bins[t].freq = effective_freqs_[t];
      out_tick->bins[t].db = spectrum_db_[t];
      out_tick->bins[t].floor = noise_floor_[t];
      out_tick->bins[t].floor_valid = noise_floor_ready_;
      out_tick->bins[t].local_bg = local_bg_db_[t];
      out_tick->bins[t].onset = onset_contrast_[t];
    }
    out_tick->diag.resize(chimes_.size());
    for (size_t ci = 0; ci < chimes_.size(); ++ci) {
      const auto &c = chimes_[ci];
      auto &d = out_tick->diag[ci];
      d.name = c.config().name;
      d.active = c.pattern_active;
      uint32_t num_steps = static_cast<uint32_t>(c.config().pattern.size());
      d.num_steps = num_steps;
      if (num_steps == 0) { d.step = 0; d.timed = false; d.time_ms = NO_TIME; continue; }
      uint32_t step = c.pattern_active ? std::min<uint32_t>(c.match_index, num_steps - 1) : 0;
      d.step = step;
      uint32_t t_ms = (step < c.config().pattern_times_ms.size()) ? c.config().pattern_times_ms[step] : NO_TIME;
      d.timed = t_ms != NO_TIME;
      d.time_ms = t_ms;
      const auto &indices = c.chord_filter_indices[step];
      d.bins.reserve(indices.size());
      for (size_t i = 0; i < indices.size(); ++i) {
        uint32_t idx = indices[i];
        auto g = c.bin_gates(spectrum_db_.data(), noise_floor_.data(), noise_floor_ready_, local_bg_db_.data(),
                             onset_contrast_.data(), idx);
        TickDiagBin b;
        b.bin = idx;
        b.freq = effective_freqs_[idx];
        b.db = g.db;
        b.eff_threshold = g.eff;
        b.threshold_ok = g.thr_ok;
        b.local_bg = g.local_bg;
        b.prominence = g.prom;
        b.prominence_ok = g.prom_ok;
        b.onset = g.onset;
        b.onset_ok = g.onset_ok;
        d.bins.push_back(b);
      }
    }
  }

  std::fill(accum_.begin(), accum_.end(), 0.0f);
  frame_count_ = 0;
  tick_sample_count_ = 0;
  debug_tick_count_++;
  return true;
}

RunResult ChimeEngine::run_offline(const float *mono, size_t num_samples, float fs) {
  RunResult res;
  // Save/restore streaming state by working in a fresh engine copy logic inlined here
  // Build freq map if not yet built (allow caller to add_chime first)
  if (total_filters_ == 0 && !chimes_.empty()) build_frequency_map();
  set_sample_rate(fs);

  uint32_t n = det_cfg_.window_size;
  uint32_t fpt = frames_per_tick_();
  float tick_ms = tick_audio_ms_();
  uint32_t total_ticks = (fpt * n > 0) ? static_cast<uint32_t>(num_samples / (fpt * n)) : 0;
  float duration_ms = total_ticks * tick_ms;

  // Quantize to int16 like js/chime.js
  std::vector<int16_t> pcm(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    int v = (int)std::lround(mono[i] * 32768.0f);
    if (v < -32768) v = -32768;
    if (v > 32767) v = 32767;
    pcm[i] = (int16_t)v;
  }

  // Reset DSP accum/history for offline run
  ensure_buffers_();
  set_sample_rate(fs);
  reset_streaming_state();
  // Note: ensure_buffers already cleared noise floor etc.

  res.meta.sample_rate_hz = fs;
  res.meta.window_size = n;
  res.meta.tick_interval_ms = det_cfg_.tick_interval_ms;
  res.meta.frames_per_tick = fpt;
  res.meta.tick_audio_ms = tick_ms;
  res.meta.guard_separation_hz = det_cfg_.guard_separation_hz;
  res.meta.total_ticks = total_ticks;
  res.meta.duration_ms = duration_ms;
  res.meta.bins = total_filters_;
  res.meta.freqs = effective_freqs_;

  res.events.reserve(64);
  res.ticks.reserve(total_ticks);

  for (uint32_t tick_idx = 0; tick_idx < total_ticks; ++tick_idx) {
    uint32_t frame_base = tick_idx * fpt * n;
    // Goertzel over fpt frames
    std::fill(accum_.begin(), accum_.end(), 0.0f);
    for (uint32_t f = 0; f < fpt; ++f) {
      std::fill(g_v1_.begin(), g_v1_.end(), 0.0f);
      std::fill(g_v2_.begin(), g_v2_.end(), 0.0f);
      uint32_t base = frame_base + f * n;
      for (uint32_t i = 0; i < n; ++i) {
        float x = (static_cast<float>(pcm[base + i]) / 32768.0f) * window_[i];
        for (uint32_t t = 0; t < total_filters_; ++t) {
          float v = g_c2_[t] * g_v1_[t] - g_v2_[t] + x;
          g_v2_[t] = g_v1_[t];
          g_v1_[t] = v;
        }
      }
      for (uint32_t t = 0; t < total_filters_; ++t) {
        float a = g_v1_[t], b = g_v2_[t];
        accum_[t] += a * a + b * b - g_c2_[t] * a * b;
      }
    }
    uint32_t now_ms = static_cast<uint32_t>((tick_idx + 1) * tick_ms);
    // Reuse try_emit_tick path by faking frame_count/tick_sample_count
    frame_count_ = fpt;
    tick_sample_count_ = fpt * n;  // ensures try_emit_tick thinks tick is full
    // But we already computed accum, so bypass feed and emit inline via try_emit's spectrum logic:
    // To avoid double accum, we set accum already, so call a helper emit directly:
    // Instead call try_emit_tick which will do spectrum conversion and state machine.
    // It expects accum already averaged with inv, so we leave accum as accumulated power
    // and let try_emit average. We already filled accum, so just set frame_count and call.
    std::vector<Event> tick_events;
    TickSnapshot snap;
    // Adjust: try_emit will average accum*inv; we already have accum as sum, so fine.
    bool emitted = try_emit_tick(now_ms, tick_events, &snap);
    if (emitted) {
      for (auto &ev : tick_events) {
        ev.tick_index = tick_idx;
        res.events.push_back(ev);
      }
      res.ticks.push_back(std::move(snap));
    }
  }

  // Build sensor intervals like js (not needed for C++ caller but useful for wasm)
  // Caller can derive from events.

  return res;
}

}  // namespace esphome::chime::core
