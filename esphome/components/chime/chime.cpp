#include "chime.h"

#ifdef USE_ESP32

#include <sys/param.h>
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome::chime {

static const char *const TAG = "chime";
static const uint32_t MAX_FILL_DURATION_MS = 30;
static const uint32_t RING_BUFFER_DURATION_MS = 120;
static constexpr uint32_t MAX_FRAMES_PER_TICK = core::MAX_FRAMES_PER_TICK;
static constexpr uint32_t DEBUG_LOG_EVERY_TICKS = 25;

// ── Chime compat helpers ──
core::ChimeConfig Chime::to_core_config(const std::string &name) const {
  core::ChimeConfig c;
  c.name = name;
  c.pattern = pattern_chords_;
  c.pattern_times_ms = pattern_times_ms_;
  c.min_duration_ms = min_duration_ms_;
  c.max_duration_ms = max_duration_ms_;
  c.threshold_db = threshold_db_;
  c.snr_margin_db = snr_margin_db_;
  c.prominence_db = prominence_db_;
  c.onset_contrast_db = onset_contrast_db_;
  c.tail_grace_ms = tail_grace_ms_;
  c.release_time_ms = release_time_ms_;
  return c;
}
void Chime::sync_from_core(const core::ChimePattern &p) {
  chord_filter_indices_ = p.chord_filter_indices;
  pattern_active_ = p.pattern_active;
  match_index_ = p.match_index;
  pattern_start_ms_ = p.pattern_start_ms;
  need_falling_edge_ = p.need_falling_edge;
  release_until_ms_ = p.release_until_ms;
  detected_latched_ = p.detected_latched;
}
void Chime::sync_to_core(core::ChimePattern &p) const {
  // Config sync is handled via to_core_config rebuild; state sync not needed here.
  (void)p;
}
bool Chime::chord_present_(const float *spectrum_db, const float *noise_floor, const float *local_bg,
                           const float *onset_contrast, uint8_t step, float &peak_db) const {
  // Delegate to engine's current pattern if available? For compat, perform same logic as core.
  // This is only used in diagnostics paths; reuse core logic via temporary pattern.
  core::ChimeConfig cfg = to_core_config("");
  core::ChimePattern tmp(cfg);
  tmp.chord_filter_indices = chord_filter_indices_;
  return tmp.chord_present(spectrum_db, noise_floor, local_bg, onset_contrast, step, peak_db);
}
void Chime::log_chord_(uint8_t step) const {
  const auto &chord = pattern_chords_[step];
  std::string parts;
  for (size_t i = 0; i < chord.size(); ++i) {
    if (i) parts += ", ";
    char buf[16]; snprintf(buf, sizeof(buf), "%.0f", chord[i]); parts += buf;
  }
  ESP_LOGD(TAG, "  chord[%u] = [%s] Hz", (unsigned)step, parts.c_str());
}
void Chime::evaluate_pattern_(const float *spectrum_db, const float *noise_floor, const float *local_bg,
                              const float *onset_contrast) {
  // No-op: real evaluation is via engine. Kept for compat.
  (void)spectrum_db; (void)noise_floor; (void)local_bg; (void)onset_contrast;
}
void Chime::latch_detection_(uint32_t elapsed_ms) {
  (void)elapsed_ms;
  detected_latched_ = true;
  release_until_ms_ = millis() + release_time_ms_;
  if (detected_sensor_ != nullptr) detected_sensor_->publish_state(true);
}
void Chime::reset_pattern_() {
  pattern_active_ = false;
  match_index_ = 0;
  need_falling_edge_ = false;
}

// ── ChimeComponent ──
void ChimeComponent::dump_config() {
  // Ensure mirrors are up-to-date from engine if built
  if (engine_built_) {
    global_freqs_ = engine_.global_freqs();
    total_filters_ = engine_.total_filters();
  }
  ESP_LOGCONFIG(TAG,
                "Chime Detector:\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Chimes: %lu\n"
                "  Total Goertzel filters: %lu\n"
                "  Noise Floor: alpha_down=%.3f, alpha_up=%.4f\n"
                "  Guard Separation: %.0f Hz",
                window_size_, tick_interval_ms_, (unsigned long)chimes_.size(),
                (unsigned long)total_filters_, noise_floor_alpha_down_, noise_floor_alpha_up_,
                guard_separation_hz_);
  for (size_t d = 0; d < chimes_.size(); ++d) {
    auto &c = chimes_[d];
    ESP_LOGCONFIG(TAG, "    Chime[%lu]:", (unsigned long)d);
    ESP_LOGCONFIG(TAG, "      Min Duration: %" PRIu32 " ms", c.min_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Max Duration: %" PRIu32 " ms", c.max_duration_ms_);
    ESP_LOGCONFIG(TAG, "      Threshold: %.1f dB (hard floor)", c.threshold_db_);
    ESP_LOGCONFIG(TAG, "      SNR Margin: %.1f dB", c.snr_margin_db_);
    ESP_LOGCONFIG(TAG, "      Prominence: %.1f dB (vs local background)", c.prominence_db_);
    ESP_LOGCONFIG(TAG, "      Onset Contrast: %.1f dB (lookback=%u ticks)", c.onset_contrast_db_, (unsigned)ONSET_LOOKBACK);
    ESP_LOGCONFIG(TAG, "      Tail Grace: %" PRIu32 " ms", c.tail_grace_ms_);
    ESP_LOGCONFIG(TAG, "      Release Time: %" PRIu32 " ms (auto)", c.release_time_ms_);
    ESP_LOGCONFIG(TAG, "      Pattern (%lu steps):", (unsigned long)c.pattern_chords_.size());
    for (size_t s = 0; s < c.pattern_chords_.size(); ++s) {
      const auto &chord = c.pattern_chords_[s];
      const uint32_t t_ms = (s < c.pattern_times_ms_.size()) ? c.pattern_times_ms_[s] : NO_TIME;
      if (t_ms == NO_TIME) ESP_LOGCONFIG(TAG, "        [%u] @ (any time):", (unsigned)s);
      else ESP_LOGCONFIG(TAG, "        [%u] @ %lu ms:", (unsigned)s, (unsigned long)t_ms);
      for (size_t f = 0; f < chord.size(); ++f) ESP_LOGCONFIG(TAG, "          %.1f Hz", chord[f]);
    }
    if (c.detected_sensor_ != nullptr) LOG_BINARY_SENSOR("      ", "Sensor:", c.detected_sensor_);
  }
}

// Build frequency map via engine, then sync back to Chime wrappers for diagnostics
void ChimeComponent::build_frequency_map_() {
  core::DetectorConfig dcfg;
  dcfg.window_size = window_size_;
  dcfg.tick_interval_ms = tick_interval_ms_;
  dcfg.guard_separation_hz = guard_separation_hz_;
  dcfg.noise_floor_alpha_down = noise_floor_alpha_down_;
  dcfg.noise_floor_alpha_up = noise_floor_alpha_up_;
  engine_.set_detector_config(dcfg);
  engine_.clear_chimes();
  for (size_t i = 0; i < chimes_.size(); ++i) {
    auto &c = chimes_[i];
    core::ChimeConfig cc = c.to_core_config("chime_" + std::to_string(i));
    engine_.add_chime(cc);
  }
  engine_.build_frequency_map();
  // Sync indices & mirrors
  for (size_t i = 0; i < chimes_.size(); ++i) {
    chimes_[i].sync_from_core(engine_.chime(i));
    chimes_[i].chord_filter_indices_ = engine_.chime(i).chord_filter_indices;
  }
  global_freqs_ = engine_.global_freqs();
  total_filters_ = engine_.total_filters();
  engine_built_ = true;
}

void ChimeComponent::setup() {
  microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto rb = ring_buffer_.lock();
    if (rb != nullptr) rb->write((void *)data.data(), data.size());
  });
  if (chimes_.empty()) {
    ESP_LOGE(TAG, "No chimes configured – nothing to do");
    return;
  }
  build_frequency_map_();
  ESP_LOGI(TAG, "Global Goertzel filters: %lu", (unsigned long)total_filters_);
  // Window / buffers are owned by engine; just mark ready and ensure frame_buf for streaming
  sample_rate_hz_ = 0.0f;
  dsp_ready_ = true;
  // Alias pointers for diagnostics (point into engine vectors' storage – valid until rebuild)
  // We keep them as null and read via engine getters in diagnostics instead.
  if (!microphone_source_->is_passive()) microphone_source_->start();
}

void ChimeComponent::loop() {
  if (!dsp_ready_ || !engine_built_) return;

  bool any_sensor = false;
  for (auto &c : chimes_) if (c.detected_sensor_ != nullptr) { any_sensor = true; break; }

  if (microphone_source_->is_running() && !status_has_error()) {
    if (start_()) status_clear_warning();
    else { ESP_LOGW(TAG, "Buffer allocation failed"); return; }
  } else {
    if (!status_has_warning()) status_set_warning(LOG_STR("Microphone is not running"));
    stop_();
    if (any_sensor) for (auto &c : chimes_) if (c.detected_sensor_ != nullptr) c.detected_sensor_->publish_state(false);
    return;
  }
  if (status_has_error()) return;

  const auto &stream_info = microphone_source_->get_audio_stream_info();

  if (sample_rate_hz_ == 0.0f) {
    sample_rate_hz_ = static_cast<float>(stream_info.get_sample_rate());
    engine_.set_sample_rate(sample_rate_hz_);
    ESP_LOGI(TAG, "Goertzel init: %lu filters, N=%" PRIu16 ", fs=%" PRIu32 " Hz",
             (unsigned long)total_filters_, window_size_, (uint32_t)stream_info.get_sample_rate());
  }

  const uint32_t samples_in_tick = stream_info.ms_to_samples(tick_interval_ms_);
  // Glue's frame assembly – feed to engine's streaming path via feed_pcm
  // We keep frame_buf_ as staging; engine internally tracks frame_count_/tick_sample_count_
  // For simplicity we replicate original staging here and directly drive engine's low-level
  // buffers via a shadow of its state (engine.feed_pcm + try_emit).
  // To avoid double bookkeeping, we use engine's own counters for tick emit gating:
  // we ensure glue's frame_buf staging fills engine correctly.
  // Approach: stage samples into frame_buf_, when full push to engine.
  // Engine's try_emit will be checked after each fill block.

  // We need to mirror engine's frame_count limiting: engine internally caps, but we also cap here.
  // Instead, we just stage and let engine decide.
  while (true) {
    // If engine has a full window staged internally, it was via feed_pcm already.
    // Check if engine wants to emit a tick based on its internal counters.
    // We call try_emit with millis as nowMs (ESP path uses wall clock).
    // But per original, tick timing is based on audio sample count, not wall clock.
    // Engine's try_emit uses tick_sample_count vs samples_in_tick; we maintain glue's
    // tick_sample_count via engine's internal. So we just poll.
    std::vector<core::Event> evs;
    // Use millis for nowMs to preserve release/max deadlines (same as original loop's millis usage)
    uint32_t now_ms = millis();
    // Try emit if engine has accumulated enough frames. We pass audio-time tick ms for diagnostics?
    // Original emit_tick uses no nowMs; pattern evaluation uses millis(). So we keep millis.
    // However run_offline uses audio time. For streaming, keep millis.
    bool emitted = engine_.try_emit_tick(now_ms, evs, nullptr);
    if (emitted) {
      // Sync wrapper state from engine and handle sensor publishes / diagnostics
      for (size_t i = 0; i < chimes_.size(); ++i) chimes_[i].sync_from_core(engine_.chime(i));
      for (auto &ev : evs) {
        size_t idx = ev.chime_index;
        if (idx >= chimes_.size()) continue;
        auto &w = chimes_[idx];
        switch (ev.type) {
          case core::EventType::Detected: w.detected_latched_ = true; w.release_until_ms_ = ev.now_ms + w.release_time_ms_; if (w.detected_sensor_) w.detected_sensor_->publish_state(true); ESP_LOGI(TAG, "CHIME DETECTED in %" PRIu32 " ms (%lu steps)", (unsigned long)ev.elapsed_ms, (unsigned long)ev.num_steps); break;
          case core::EventType::Released: w.detected_latched_ = false; if (w.detected_sensor_) w.detected_sensor_->publish_state(false); ESP_LOGD(TAG, "Chime[%u] released after %" PRIu32 " ms hold", (unsigned)idx, (unsigned)ev.hold_ms); break;
          case core::EventType::MaxDurationTimeout: w.pattern_active_ = false; w.match_index_ = 0; w.need_falling_edge_ = false; ESP_LOGD(TAG, "Chime[%u] timed out after %" PRIu32 " ms (max %" PRIu32 " ms, matched %u/%u)", (unsigned)idx, (unsigned)ev.elapsed_ms, (unsigned)ev.max_ms, (unsigned)ev.matched, (unsigned)ev.num_steps); if (!w.detected_latched_ && w.detected_sensor_) w.detected_sensor_->publish_state(false); break;
          case core::EventType::PatternStart: ESP_LOGI(TAG, "Chime pattern started: chord 1/%u, peak %.1f dB", (unsigned)ev.num_steps, ev.peak_db); break;
          case core::EventType::FallingEdge: ESP_LOGD(TAG, "Falling edge after chord %u/%u at t=%" PRIu32 " ms", (unsigned)(ev.step+1), (unsigned)ev.num_steps, (unsigned)ev.elapsed_ms); break;
          case core::EventType::ChordMatch: ESP_LOGD(TAG, "Chord %u/%u matched at t=%" PRIu32 " ms, peak %.1f dB", (unsigned)(ev.step+1), (unsigned)ev.num_steps, (unsigned)ev.elapsed_ms, ev.peak_db); break;
          case core::EventType::MinDurationDiscount: ESP_LOGD(TAG, "Chime discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms", (unsigned)ev.elapsed_ms, (unsigned)ev.min_ms); break;
          case core::EventType::StepTimeout: ESP_LOGW(TAG, "Chime timed out: chord %u/%u not detected in [%" PRIu32 ", %" PRIu32 "] ms (elapsed %" PRIu32 " ms)", (unsigned)(ev.step+1), (unsigned)ev.num_steps, (unsigned)ev.t_chord_ms, (unsigned)ev.window_end_ms, (unsigned)ev.elapsed_ms); break;
          default: break;
        }
      }
      // Periodic diag like original
      if ((debug_tick_count_++ % DEBUG_LOG_EVERY_TICKS) == 0 && engine_.noise_floor_ready()) {
        auto &spec = engine_.spectrum_db();
        auto &floor = engine_.noise_floor();
        float spec_min = *std::min_element(spec.begin(), spec.end());
        float spec_max = *std::max_element(spec.begin(), spec.end());
        float floor_min = *std::min_element(floor.begin(), floor.end());
        float floor_max = *std::max_element(floor.begin(), floor.end());
        ESP_LOGD(TAG, "[diag] spectrum min/max=%.1f/%.1f dB | noise-floor min/max=%.1f/%.1f dB (ready=yes)", spec_min, spec_max, floor_min, floor_max);
        for (size_t d = 0; d < chimes_.size(); ++d) log_chime_diagnostics_(d, chimes_[d]);
      }
      continue; // check if another tick is immediately ready (rare)
    }

    // Not ready to emit; try to stage more samples
    // Need to ensure we have space in engine's tick window
    // If engine frame_count >= MAX_FRAMES_PER_TICK or tick_sample_count >= samples_in_tick, we'd have emitted above
    // Now stage from audio_source
    audio_source_->fill(0, false);
    const uint32_t available = stream_info.bytes_to_samples(audio_source_->available());
    if (available == 0) break;
    const int16_t *data = reinterpret_cast<const int16_t *>(audio_source_->mutable_data());
    const uint32_t need = window_size_ - frame_buf_offset_;
    const uint32_t take = std::min(available, need);
    std::memcpy(frame_buf_ + frame_buf_offset_, data, stream_info.samples_to_bytes(take));
    audio_source_->consume(stream_info.samples_to_bytes(take));
    frame_buf_offset_ += take;
    if (frame_buf_offset_ >= window_size_) {
      engine_.feed_pcm(frame_buf_, window_size_);
      frame_buf_offset_ = 0;
      // Don't increment glue counters – engine tracks internally. But we still need to loop to poll emit.
    }
    // If we still have more available and tick not full, loop again
    if (available > take) continue;
    // If we filled one window, try emit next iteration without consuming more
    // To avoid busy loop, break if no emit and no more data
    // Check if engine now has enough to emit (will be checked at top of next loop iter)
    // For now break to avoid spinning; outer while will re-enter next loop() call
    break;
  }

  // Sync any remaining state changes (e.g., pattern_active cleared inside feed but not yet synced)
  for (size_t i = 0; i < chimes_.size(); ++i) chimes_[i].sync_from_core(engine_.chime(i));
}

void ChimeComponent::process_frame_(const int16_t *samples) { engine_.feed_pcm(samples, window_size_); }
void ChimeComponent::emit_tick_() {
  std::vector<core::Event> evs; engine_.try_emit_tick(millis(), evs, nullptr);
  for (size_t i = 0; i < chimes_.size(); ++i) chimes_[i].sync_from_core(engine_.chime(i));
}
void ChimeComponent::compute_local_background_() { /* handled inside engine */ }
bool ChimeComponent::bin_is_busy_(uint32_t filter_idx) const {
  for (size_t i = 0; i < chimes_.size(); ++i) {
    const auto &c = engine_.chime(i);
    if (!c.pattern_active) continue;
    for (auto &chord : c.chord_filter_indices) for (uint32_t idx : chord) if (idx == filter_idx) return true;
  }
  return false;
}
void ChimeComponent::log_chime_diagnostics_(size_t d, const Chime &c) {
  if (c.pattern_chords_.empty()) return;
  uint32_t num_steps = (uint32_t)c.pattern_chords_.size();
  uint32_t step = 0;
  if (c.pattern_active_) step = std::min<uint32_t>(c.match_index_, num_steps - 1);
  const auto &indices = c.chord_filter_indices_[step];
  const auto &freqs = c.pattern_chords_[step];
  uint32_t t_ms = (step < c.pattern_times_ms_.size()) ? c.pattern_times_ms_[step] : NO_TIME;
  ESP_LOGD(TAG, "[diag] Chime[%lu]: state=%s, waiting for step %u/%lu (%s)", (unsigned long)d, c.pattern_active_ ? "ACTIVE" : "idle", (unsigned)step, (unsigned long)num_steps, t_ms == NO_TIME ? "any time" : "timed");
  if (!engine_.noise_floor_ready() || engine_.spectrum_db().empty()) return;
  const auto &spec = engine_.spectrum_db();
  const auto &floor = engine_.noise_floor();
  const auto &local = engine_.local_bg();
  const auto &onset = engine_.onset_contrast();
  for (size_t i = 0; i < indices.size() && i < freqs.size(); ++i) {
    uint32_t idx = indices[i];
    if (idx >= spec.size()) continue;
    float db = spec[idx];
    float eff = c.threshold_db_;
    float nf = floor[idx];
    float adaptive = nf + c.snr_margin_db_;
    if (adaptive > eff) eff = adaptive;
    bool thr_ok = db >= eff;
    float lb = local[idx];
    bool prom_ok = (db - lb) >= c.prominence_db_;
    float onsetv = onset[idx];
    bool onset_ok = onsetv >= c.onset_contrast_db_;
    ESP_LOGD(TAG, "   bin#%lu %.0fHz: db=%.1f | thr eff=%.1f (hard %.1f, floor %.1f + %.1f) = %s | localbg=%.1f prom=%+.1f (need %.1f) = %s | onset=%+.1f (need %.1f) = %s",
             (unsigned long)idx, freqs[i], db, eff, c.threshold_db_, nf, c.snr_margin_db_, thr_ok?"PASS":"FAIL", lb, (db-lb), c.prominence_db_, prom_ok?"PASS":"FAIL", onsetv, c.onset_contrast_db_, onset_ok?"PASS":"FAIL");
  }
}

void ChimeComponent::start() {
  if (microphone_source_->is_passive()) { ESP_LOGW(TAG, "Cannot start microphone in passive mode"); return; }
  microphone_source_->start();
}
void ChimeComponent::stop() {
  if (microphone_source_->is_passive()) { ESP_LOGW(TAG, "Cannot stop microphone in passive mode"); return; }
  microphone_source_->stop();
}

bool ChimeComponent::start_() {
  if (audio_source_ != nullptr) return true;
  const auto &stream_info = microphone_source_->get_audio_stream_info();
  const size_t bpf = stream_info.frames_to_bytes(1);
  ring_buffer_.reset();
  const size_t rb_size = (stream_info.ms_to_bytes(RING_BUFFER_DURATION_MS) / bpf) * bpf;
  auto rb = ring_buffer::RingBuffer::create(rb_size);
  if (rb == nullptr) { status_momentary_error("ring_buffer", 15000); return false; }
  audio_source_ = audio::RingBufferAudioSource::create(rb, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS), (uint8_t)bpf);
  if (audio_source_ == nullptr) { status_momentary_error("audio_source", 15000); return false; }
  ring_buffer_ = rb;
  frame_buf_ = static_cast<int16_t *>(malloc((window_size_ + 1) * sizeof(int16_t)));
  if (frame_buf_ == nullptr) { ESP_LOGE(TAG, "Failed to allocate frame buffer"); audio_source_.reset(); ring_buffer_.reset(); status_momentary_error("frame_buf", 15000); return false; }
  frame_buf_offset_ = 0;
  engine_.reset_onset_history();
  // Reset engine streaming state counters (but keep noise floor)
  // We reuse engine's reset_streaming_state but need to preserve noise floor readiness
  // Call engine's reset for frame counters
  {
    // Manually reset engine streaming counters without clearing noise floor history beyond onset
    // Engine's reset_streaming_state clears onset history – we already cleared above, and we want fresh
  }
  status_clear_error();
  return true;
}
void ChimeComponent::stop_() {
  audio_source_.reset();
  if (frame_buf_ != nullptr) { free(frame_buf_); frame_buf_ = nullptr; }
  frame_buf_offset_ = 0;
}

}  // namespace esphome::chime
#endif
