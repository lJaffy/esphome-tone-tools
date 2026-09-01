#include "chime.h"

#ifdef USE_ESP32

#include <sys/param.h>
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <cstring>

#if defined(USE_ESP32)
#if __has_include(<esp_adc/adc_continuous.h>)
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#else
#include <driver/adc.h>
#include <esp_adc_cal.h>
#endif
#include <driver/gpio.h>
#include <esp_err.h>
#ifndef SOC_ADC_DIGI_MAX_BITWIDTH
#define SOC_ADC_DIGI_MAX_BITWIDTH 12
#endif
#ifndef SOC_ADC_DIGI_RESULT_BYTES
#define SOC_ADC_DIGI_RESULT_BYTES 2
#endif
#ifndef SOC_ADC_CALIB_SUPPORTED
#define SOC_ADC_CALIB_SUPPORTED 0
#endif
#endif

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
  if (has_adc_ && has_mic_) {
    ESP_LOGE(TAG, "Invalid config: both microphone and adc configured (exclusive)");
  }
  const char *src = has_adc_ ? "ADC" : (has_mic_ ? "Microphone" : "None");
  ESP_LOGCONFIG(TAG,
                "Chime Detector:\n"
                "  Source: %s\n"
                "  Window Size: %" PRIu16 " samples\n"
                "  Tick Interval: %" PRIu32 " ms\n"
                "  Chimes: %lu\n"
                "  Total Goertzel filters: %lu\n"
                "  Noise Floor: alpha_down=%.3f, alpha_up=%.4f\n"
                "  Guard Separation: %.0f Hz",
                src, window_size_, tick_interval_ms_, (unsigned long)chimes_.size(),
                (unsigned long)total_filters_, noise_floor_alpha_down_, noise_floor_alpha_up_,
                guard_separation_hz_);
  if (has_adc_) {
    ESP_LOGCONFIG(TAG, "  ADC pin: GPIO%u", (unsigned)adc_pin_);
    ESP_LOGCONFIG(TAG, "  ADC sample rate: %" PRIu32 " Hz", adc_sample_rate_);
    ESP_LOGCONFIG(TAG, "  ADC attenuation: %d", (int)adc_atten_);
    ESP_LOGCONFIG(TAG, "  ADC gain: %" PRIu32, adc_gain_);
    ESP_LOGCONFIG(TAG, "  ADC passive: %s", passive_ ? "true" : "false");
  } else if (has_mic_) {
    ESP_LOGCONFIG(TAG, "  Microphone passive: %s", passive_ ? "true" : "false");
  }
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

void ChimeComponent::update_adc_hpf_() {
  // 20 Hz high-pass to strip DC drift of piezo bias on pipe
  const float fs = has_adc_ ? static_cast<float>(adc_sample_rate_) : sample_rate_hz_;
  if (fs <= 0) return;
  const float rc = fs / (2.0f * (float)M_PI * 20.0f);
  adc_hpf_alpha_ = rc / (rc + 1.0f);
  adc_hpf_x_prev_ = 0.0f;
  adc_hpf_y_prev_ = 0.0f;
}

void ChimeComponent::setup() {
  if (has_mic_ && has_adc_) {
    ESP_LOGE(TAG, "Chime: exclusive source violated – both mic and adc set. Choose one.");
    mark_failed();
    return;
  }
  if (!has_mic_ && !has_adc_) {
    ESP_LOGE(TAG, "Chime: no source configured – set either 'microphone:' or 'adc:'");
    mark_failed();
    return;
  }
  if (has_mic_) {
    if (microphone_source_ == nullptr) {
      ESP_LOGE(TAG, "Microphone source is null");
      mark_failed();
      return;
    }
    microphone_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
      auto rb = ring_buffer_.lock();
      if (rb != nullptr) rb->write((void *)data.data(), data.size());
    });
  }
  if (chimes_.empty()) {
    ESP_LOGE(TAG, "No chimes configured – nothing to do");
    return;
  }
  build_frequency_map_();
  ESP_LOGI(TAG, "Global Goertzel filters: %lu", (unsigned long)total_filters_);
  sample_rate_hz_ = 0.0f;
  dsp_ready_ = true;
  if (has_adc_) {
    update_adc_hpf_();
    // passive_ mirrors microphone passive semantics for ADC: if not passive, auto-start
    if (!passive_) {
      if (!start_adc_()) {
        ESP_LOGE(TAG, "ADC start failed");
      }
    }
  } else {
    if (!microphone_source_->is_passive()) microphone_source_->start();
  }
}

void ChimeComponent::handle_tick_events_(const std::vector<core::Event> &evs) {
  for (auto &ev : evs) {
    size_t idx = ev.chime_index;
    if (idx >= chimes_.size()) continue;
    auto &w = chimes_[idx];
    switch (ev.type) {
      case core::EventType::Detected:
        w.detected_latched_ = true;
        w.release_until_ms_ = ev.now_ms + w.release_time_ms_;
        if (w.detected_sensor_) w.detected_sensor_->publish_state(true);
        ESP_LOGI(TAG, "CHIME DETECTED in %" PRIu32 " ms (%lu steps)", (unsigned long)ev.elapsed_ms, (unsigned long)ev.num_steps);
        break;
      case core::EventType::Released:
        w.detected_latched_ = false;
        if (w.detected_sensor_) w.detected_sensor_->publish_state(false);
        ESP_LOGD(TAG, "Chime[%u] released after %" PRIu32 " ms hold", (unsigned)idx, (unsigned long)ev.hold_ms);
        break;
      case core::EventType::MaxDurationTimeout:
        w.pattern_active_ = false;
        w.match_index_ = 0;
        w.need_falling_edge_ = false;
        ESP_LOGD(TAG, "Chime[%u] timed out after %" PRIu32 " ms (max %" PRIu32 " ms, matched %u/%u)", (unsigned)idx,
                 (unsigned long)ev.elapsed_ms, (unsigned long)ev.max_ms, (unsigned)ev.matched, (unsigned)ev.num_steps);
        if (!w.detected_latched_ && w.detected_sensor_) w.detected_sensor_->publish_state(false);
        break;
      case core::EventType::PatternStart:
        ESP_LOGI(TAG, "Chime pattern started: chord 1/%u, peak %.1f dB", (unsigned)ev.num_steps, ev.peak_db);
        break;
      case core::EventType::FallingEdge:
        ESP_LOGD(TAG, "Falling edge after chord %u/%u at t=%" PRIu32 " ms", (unsigned)(ev.step + 1), (unsigned)ev.num_steps,
                 (unsigned long)ev.elapsed_ms);
        break;
      case core::EventType::ChordMatch:
        ESP_LOGD(TAG, "Chord %u/%u matched at t=%" PRIu32 " ms, peak %.1f dB", (unsigned)(ev.step + 1), (unsigned)ev.num_steps,
                 (unsigned long)ev.elapsed_ms, ev.peak_db);
        break;
      case core::EventType::MinDurationDiscount:
        ESP_LOGD(TAG, "Chime discounted: completed in %" PRIu32 " ms, below min %" PRIu32 " ms", (unsigned long)ev.elapsed_ms,
                 (unsigned long)ev.min_ms);
        break;
      case core::EventType::StepTimeout:
        ESP_LOGW(TAG, "Chime timed out: chord %u/%u not detected in [%" PRIu32 ", %" PRIu32 "] ms (elapsed %" PRIu32 " ms)",
                 (unsigned)(ev.step + 1), (unsigned)ev.num_steps, (unsigned long)ev.t_chord_ms, (unsigned long)ev.window_end_ms,
                 (unsigned long)ev.elapsed_ms);
        break;
      default: break;
    }
  }
  // Sync wrappers
  for (size_t i = 0; i < chimes_.size(); ++i) chimes_[i].sync_from_core(engine_.chime(i));
  // Periodic diag
  if ((debug_tick_count_++ % DEBUG_LOG_EVERY_TICKS) == 0 && engine_.noise_floor_ready()) {
    auto &spec = engine_.spectrum_db();
    auto &floor = engine_.noise_floor();
    float spec_min = *std::min_element(spec.begin(), spec.end());
    float spec_max = *std::max_element(spec.begin(), spec.end());
    float floor_min = *std::min_element(floor.begin(), floor.end());
    float floor_max = *std::max_element(floor.begin(), floor.end());
    ESP_LOGD(TAG, "[diag] spectrum min/max=%.1f/%.1f dB | noise-floor min/max=%.1f/%.1f dB (ready=yes)", spec_min, spec_max,
             floor_min, floor_max);
    for (size_t d = 0; d < chimes_.size(); ++d) log_chime_diagnostics_(d, chimes_[d]);
  }
}

void ChimeComponent::loop() {
  if (!dsp_ready_ || !engine_built_) return;

  bool any_sensor = false;
  for (auto &c : chimes_) if (c.detected_sensor_ != nullptr) { any_sensor = true; break; }

  // ── Exclusive source: ADC vs Microphone ──
  if (has_adc_) {
    if (adc_running_ && !status_has_error()) {
      if (!status_has_warning()) status_clear_warning();
    } else {
      if (!status_has_warning()) {
        if (!adc_running_ && !passive_) status_set_warning(LOG_STR("ADC is not running"));
        else if (status_has_error()) status_set_warning(LOG_STR("ADC error"));
      }
      // keep adc_running_ false until next start attempt; still need to drain tick
      if (any_sensor) {
        // Check if we should auto-restart when not passive
        if (!passive_ && !adc_running_) {
          start_adc_();
        }
      }
      if (!adc_running_) {
        if (any_sensor) for (auto &c : chimes_) if (c.detected_sensor_ != nullptr) c.detected_sensor_->publish_state(false);
        // allow try_emit to handle release timeouts even without new samples
        std::vector<core::Event> evs;
        if (engine_.try_emit_tick(millis(), evs, nullptr)) handle_tick_events_(evs);
        return;
      }
    }
    if (status_has_error()) return;

    if (sample_rate_hz_ == 0.0f) {
      sample_rate_hz_ = static_cast<float>(adc_sample_rate_);
      engine_.set_sample_rate(sample_rate_hz_);
      update_adc_hpf_();
      ESP_LOGI(TAG, "Goertzel init: %lu filters, N=%" PRIu16 ", fs=%" PRIu32 " Hz (ADC GPIO%u)",
               (unsigned long)total_filters_, window_size_, adc_sample_rate_, (unsigned)adc_pin_);
    }

    // Feed ADC samples into engine
    read_adc_into_engine_();

    // Poll tick emission (may emit 0 or 1 tick per loop)
    std::vector<core::Event> evs;
    uint32_t now_ms = millis();
    if (engine_.try_emit_tick(now_ms, evs, nullptr)) {
      handle_tick_events_(evs);
      // try to drain second tick if immediately ready (rare)
      std::vector<core::Event> evs2;
      if (engine_.try_emit_tick(now_ms, evs2, nullptr)) handle_tick_events_(evs2);
    }
    // Sync remaining state
    for (size_t i = 0; i < chimes_.size(); ++i) chimes_[i].sync_from_core(engine_.chime(i));
    return;
  }

  // ── Microphone path (original) ──
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
    std::vector<core::Event> evs;
    uint32_t now_ms = millis();
    bool emitted = engine_.try_emit_tick(now_ms, evs, nullptr);
    if (emitted) {
      handle_tick_events_(evs);
      continue;
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
  if (has_adc_) {
    if (passive_) { ESP_LOGW(TAG, "Cannot start ADC in passive mode"); return; }
    if (!adc_running_) start_adc_();
    return;
  }
  if (microphone_source_ == nullptr) return;
  if (microphone_source_->is_passive()) { ESP_LOGW(TAG, "Cannot start microphone in passive mode"); return; }
  microphone_source_->start();
}
void ChimeComponent::stop() {
  if (has_adc_) {
    if (passive_) { ESP_LOGW(TAG, "Cannot stop ADC in passive mode"); return; }
    stop_adc_();
    return;
  }
  if (microphone_source_ == nullptr) return;
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
  std::shared_ptr<ring_buffer::RingBuffer> rb_shared(std::move(rb));
  audio_source_ = audio::RingBufferAudioSource::create(rb_shared, stream_info.ms_to_bytes(MAX_FILL_DURATION_MS), (uint8_t)bpf);
  if (audio_source_ == nullptr) { status_momentary_error("audio_source", 15000); return false; }
  ring_buffer_ = rb_shared;
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

// ── ADC continuous driver ──
bool ChimeComponent::start_adc_() {
  if (adc_running_) return true;
  if (adc_pin_ == 255) {
    ESP_LOGE(TAG, "ADC pin not configured");
    return false;
  }
  // Allocate frame staging buffer (same as mic path)
  if (frame_buf_ == nullptr) {
    frame_buf_ = static_cast<int16_t *>(malloc((window_size_ + 1) * sizeof(int16_t)));
    if (frame_buf_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate ADC frame buffer");
      status_momentary_error("frame_buf", 15000);
      return false;
    }
    frame_buf_offset_ = 0;
  }
  if (adc_raw_buf_ == nullptr) {
    adc_raw_buf_ = static_cast<uint8_t *>(malloc(ADC_RAW_BUF_BYTES));
    if (adc_raw_buf_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate ADC DMA buffer");
      status_momentary_error("adc_buf", 15000);
      return false;
    }
    adc_raw_buf_len_ = ADC_RAW_BUF_BYTES;
  }

#if defined(USE_ESP32)
  // Resolve pin -> channel/unit. Use adc_oneshot_io_to_channel if available.
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_0;
  bool pin_mapped = false;
#if __has_include("esp_adc/adc_oneshot.h")
  // Preferred: adc_oneshot_io_to_channel (IDF 5.x)
  extern esp_err_t adc_oneshot_io_to_channel(int io_num, adc_unit_t *unit_id, adc_channel_t *chan);
  if (adc_oneshot_io_to_channel(adc_pin_, &unit, &channel) == ESP_OK) pin_mapped = true;
#endif
  if (!pin_mapped) {
    // Fallback: brute force search via ADC1 channel map (ESP32 classic pins)
    // GPIO 32-39 -> ADC1 CH 4-9, etc. For other variants, warn and assume ADC1 CH0.
    ESP_LOGW(TAG, "ADC pin GPIO%u mapping fallback – assuming ADC_UNIT_1 CH0; check wiring", (unsigned)adc_pin_);
    channel = ADC_CHANNEL_0;
    unit = ADC_UNIT_1;
  }
  if (unit != ADC_UNIT_1) {
    ESP_LOGW(TAG, "ADC continuous only supports ADC_UNIT_1 on this chip – using UNIT_1");
    unit = ADC_UNIT_1;
  }

  adc_continuous_handle_cfg_t handle_cfg{};
  handle_cfg.max_store_buf_size = ADC_RAW_BUF_BYTES * 2;
  handle_cfg.conv_frame_size = 256;
  esp_err_t err = adc_continuous_new_handle(&handle_cfg, &adc_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(err));
    status_momentary_error("adc_handle", 15000);
    return false;
  }

  adc_digi_pattern_config_t pattern{};
  pattern.atten = adc_atten_;
  pattern.channel = channel & 0x7;
  pattern.unit = unit;
  pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t dig_cfg{};
  dig_cfg.pattern_num = 1;
  dig_cfg.adc_pattern = &pattern;
  dig_cfg.sample_freq_hz = adc_sample_rate_;
  dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

  err = adc_continuous_config(adc_handle_, &dig_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(err));
    adc_continuous_deinit(adc_handle_);
    adc_handle_ = nullptr;
    status_momentary_error("adc_config", 15000);
    return false;
  }

  // Optional calibration (line fitting) – best-effort, ignore failure
#if SOC_ADC_CALIB_SUPPORTED
  adc_cali_line_fitting_config_t cali_cfg{};
  cali_cfg.unit_id = unit;
  cali_cfg.atten = adc_atten_;
  cali_cfg.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH;
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle_) != ESP_OK) {
    adc_cali_handle_ = nullptr;
  }
#endif

  err = adc_continuous_start(adc_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_start failed: %s", esp_err_to_name(err));
    adc_continuous_deinit(adc_handle_);
    adc_handle_ = nullptr;
    status_momentary_error("adc_start", 15000);
    return false;
  }

  adc_running_ = true;
  engine_.reset_onset_history();
  update_adc_hpf_();
  adc_midpoint_ = 2048;
  status_clear_error();
  ESP_LOGI(TAG, "ADC started: GPIO%u unit %d ch %d @ %" PRIu32 " Hz atten %d", (unsigned)adc_pin_, (int)unit, (int)channel,
           adc_sample_rate_, (int)adc_atten_);
  return true;
#else
  ESP_LOGE(TAG, "ADC not supported on this target");
  return false;
#endif
}

void ChimeComponent::stop_adc_() {
  if (!adc_running_) return;
#if defined(USE_ESP32)
  if (adc_handle_ != nullptr) {
    adc_continuous_stop(adc_handle_);
    adc_continuous_deinit(adc_handle_);
    adc_handle_ = nullptr;
  }
#if SOC_ADC_CALIB_SUPPORTED
  if (adc_cali_handle_ != nullptr) {
#if __has_include("esp_adc/adc_cali_scheme.h")
    adc_cali_delete_scheme_line_fitting(adc_cali_handle_);
#endif
    adc_cali_handle_ = nullptr;
  }
#endif
#endif
  adc_running_ = false;
  if (frame_buf_ != nullptr) {
    free(frame_buf_);
    frame_buf_ = nullptr;
  }
  frame_buf_offset_ = 0;
  if (adc_raw_buf_ != nullptr) {
    free(adc_raw_buf_);
    adc_raw_buf_ = nullptr;
    adc_raw_buf_len_ = 0;
  }
}

bool ChimeComponent::read_adc_into_engine_() {
#if defined(USE_ESP32)
  if (adc_handle_ == nullptr || !adc_running_) return false;
  uint32_t out_len = 0;
  esp_err_t err = adc_continuous_read(adc_handle_, adc_raw_buf_, adc_raw_buf_len_, &out_len, 0);
  if (err != ESP_OK || out_len == 0) return false;

  const size_t stride = SOC_ADC_DIGI_RESULT_BYTES;
  size_t num_samples = out_len / stride;
  for (size_t i = 0; i < num_samples; ++i) {
    adc_digi_output_data_t *p = reinterpret_cast<adc_digi_output_data_t *>(adc_raw_buf_ + i * stride);
    uint32_t raw = 0;
    // TYPE1: p->type1.data, TYPE2 similar – unify via macro
#if SOC_ADC_DIGI_RESULT_BYTES == 4
    raw = p->type2.data;
#else
    raw = p->type1.data;
#endif
    // Slow midpoint tracking for pipe bias drift (temperature / supply)
    // 12-bit ADC: 0..4095, midpoint ~2048
    adc_midpoint_ = (adc_midpoint_ * 2047 + (int32_t)raw) / 2048;

    int32_t centered = (int32_t)raw - adc_midpoint_;
    // Scale 12-bit centered (-2048..2047) -> 16-bit (-32768..32767) and apply gain
    int32_t scaled = centered * 16 * (int32_t)adc_gain_;
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;

    // First-order HPF to remove residual DC / slow pipe drift
    float x = static_cast<float>(scaled);
    float y = adc_hpf_alpha_ * (adc_hpf_y_prev_ + x - adc_hpf_x_prev_);
    adc_hpf_x_prev_ = x;
    adc_hpf_y_prev_ = y;
    int16_t sample = static_cast<int16_t>(std::clamp<int>(static_cast<int>(std::lround(y)), -32768, 32767));

    // Stage into engine's frame buffer
    frame_buf_[frame_buf_offset_++] = sample;
    if (frame_buf_offset_ >= window_size_) {
      engine_.feed_pcm(frame_buf_, window_size_);
      frame_buf_offset_ = 0;
    }
  }
  return num_samples > 0;
#else
  return false;
#endif
}

}  // namespace esphome::chime
#endif
