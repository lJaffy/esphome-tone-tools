#include "chime_pattern.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::chime::core {

ChimePattern::ChimePattern(const ChimeConfig &cfg) { set_config(cfg); }

void ChimePattern::set_config(const ChimeConfig &cfg) {
  cfg_ = cfg;
  // Keep derived release_time_ms in sync like Chime::set_max_duration_ms.
  cfg_.release_time_ms = cfg_.max_duration_ms + 2000;
}

void ChimePattern::reset_pattern() {
  pattern_active = false;
  match_index = 0;
  need_falling_edge = false;
}

bool ChimePattern::chord_present(const float *spectrum_db,
                                 const float *noise_floor,
                                 const float *local_bg,
                                 const float *onset_contrast, uint8_t step,
                                 float &peak_db) const {
  const auto &indices = chord_filter_indices[step];
  peak_db = -300.0f;
  for (size_t i = 0; i < indices.size(); ++i) {
    const uint32_t idx = indices[i];
    const float db = spectrum_db[idx];
    float effective = cfg_.threshold_db;
    if (noise_floor != nullptr) {
      const float adaptive = noise_floor[idx] + cfg_.snr_margin_db;
      if (adaptive > effective)
        effective = adaptive;
    }
    if (db < effective)
      return false;
    if (local_bg != nullptr && (db - local_bg[idx]) < cfg_.prominence_db)
      return false;
    if (onset_contrast != nullptr && cfg_.onset_contrast_db > 0.0f) {
      if (onset_contrast[idx] < cfg_.onset_contrast_db)
        return false;
    }
    if (db > peak_db)
      peak_db = db;
  }
  return true;
}

ChimePattern::BinGate ChimePattern::bin_gates(const float *spectrum_db,
                                              const float *noise_floor,
                                              bool noise_floor_ready,
                                              const float *local_bg,
                                              const float *onset_contrast,
                                              uint32_t filter_idx) const {
  BinGate g;
  g.db = spectrum_db[filter_idx];
  float eff = cfg_.threshold_db;
  float fl = noise_floor_ready ? noise_floor[filter_idx] : 0.0f;
  g.floor = fl;
  float adaptive = fl + cfg_.snr_margin_db;
  if (adaptive > eff)
    eff = adaptive;
  g.eff = eff;
  g.thr_ok = g.db >= eff;
  g.local_bg = local_bg[filter_idx];
  g.prom = g.db - g.local_bg;
  g.prom_ok = g.prom >= cfg_.prominence_db;
  g.onset = onset_contrast[filter_idx];
  g.onset_ok = g.onset >= cfg_.onset_contrast_db;
  return g;
}

void ChimePattern::latch_detection_(
    uint32_t now_ms, uint32_t elapsed_ms, uint32_t chime_index,
    uint32_t tick_index, const std::function<void(const Event &)> &emit) {
  Event ev;
  ev.type = EventType::Detected;
  ev.chime_index = chime_index;
  ev.tick_index = tick_index;
  ev.now_ms = now_ms;
  ev.elapsed_ms = elapsed_ms;
  ev.num_steps = static_cast<uint32_t>(cfg_.pattern.size());
  if (emit)
    emit(ev);
  detected_latched = true;
  release_until_ms = now_ms + cfg_.release_time_ms;
  pattern_active = false;
  match_index = 0;
  need_falling_edge = false;
}

void ChimePattern::evaluate_pattern(
    uint32_t now_ms, const float *spectrum_db, const float *noise_floor,
    const float *local_bg, const float *onset_contrast, uint32_t chime_index,
    uint32_t tick_index, const std::function<void(const Event &)> &emit) {
  const uint32_t num_steps = static_cast<uint32_t>(cfg_.pattern.size());
  const uint32_t elapsed = now_ms - pattern_start_ms;

  if (!pattern_active) {
    float peak = -300.0f;
    if (chord_present(spectrum_db, noise_floor, local_bg, onset_contrast, 0,
                      peak)) {
      pattern_active = true;
      match_index = 1;
      pattern_start_ms = now_ms;
      need_falling_edge = true;
      if (emit) {
        Event ev;
        ev.type = EventType::PatternStart;
        ev.chime_index = chime_index;
        ev.tick_index = tick_index;
        ev.now_ms = now_ms;
        ev.step = 0;
        ev.num_steps = num_steps;
        ev.peak_db = peak;
        emit(ev);
      }
    }
    return;
  }

  if (need_falling_edge) {
    const uint8_t prev_idx = static_cast<uint8_t>(match_index - 1);
    const uint8_t next_idx = static_cast<uint8_t>(match_index);
    const auto &prev_chord = cfg_.pattern[prev_idx];
    const std::vector<float> *next_chord =
        (next_idx < num_steps) ? &cfg_.pattern[next_idx] : nullptr;

    bool prev_subset_of_next = false;
    if (next_chord != nullptr) {
      prev_subset_of_next = true;
      for (float f_prev : prev_chord) {
        bool in_next = false;
        for (float f_next : *next_chord) {
          if (std::fabs(f_prev - f_next) < 0.5f) {
            in_next = true;
            break;
          }
        }
        if (!in_next) {
          prev_subset_of_next = false;
          break;
        }
      }
    }

    if (prev_subset_of_next) {
      need_falling_edge = false;
    } else {
      float prev_peak = -300.0f;
      if (chord_present(spectrum_db, noise_floor, local_bg, onset_contrast,
                        prev_idx, prev_peak)) {
        return;
      }
      need_falling_edge = false;
      if (emit) {
        Event ev;
        ev.type = EventType::FallingEdge;
        ev.chime_index = chime_index;
        ev.tick_index = tick_index;
        ev.now_ms = now_ms;
        ev.step = prev_idx;
        ev.num_steps = num_steps;
        ev.elapsed_ms = elapsed;
        emit(ev);
      }
    }
  }

  if (match_index >= num_steps) {
    const uint32_t elapsed_now = now_ms - pattern_start_ms;
    if (elapsed_now >= cfg_.min_duration_ms) {
      latch_detection_(now_ms, elapsed_now, chime_index, tick_index, emit);
    } else {
      if (emit) {
        Event ev;
        ev.type = EventType::MinDurationDiscount;
        ev.chime_index = chime_index;
        ev.tick_index = tick_index;
        ev.now_ms = now_ms;
        ev.elapsed_ms = elapsed_now;
        ev.min_ms = cfg_.min_duration_ms;
        ev.num_steps = num_steps;
        emit(ev);
      }
      reset_pattern();
    }
    return;
  }

  const uint32_t chord_idx = match_index;
  const uint32_t t_chord = (chord_idx < cfg_.pattern_times_ms.size())
                               ? cfg_.pattern_times_ms[chord_idx]
                               : NO_TIME;

  if (t_chord != NO_TIME && elapsed < t_chord)
    return;

  if (t_chord != NO_TIME) {
    const uint32_t next_idx = chord_idx + 1;
    const uint32_t t_next =
        (next_idx < num_steps && next_idx < cfg_.pattern_times_ms.size())
            ? cfg_.pattern_times_ms[next_idx]
            : NO_TIME;
    const uint32_t window_end =
        (t_next != NO_TIME) ? t_next : (t_chord + cfg_.tail_grace_ms);
    // Narrow latch window to first quarter of inter-step interval or 2 ticks,
    // whichever larger, capped to window.
    const uint32_t window_len =
        (window_end > t_chord) ? window_end - t_chord : 0;
    const uint32_t quarter = window_len / 4;
    const uint32_t two_ticks = 2 * tick_interval_ms_;
    uint32_t allowed_len = std::max(quarter, two_ticks);
    if (allowed_len > window_len)
      allowed_len = window_len;
    const uint32_t effective_end = t_chord + allowed_len;
    if (elapsed >= effective_end) {
      if (emit) {
        Event ev;
        ev.type = EventType::StepTimeout;
        ev.chime_index = chime_index;
        ev.tick_index = tick_index;
        ev.now_ms = now_ms;
        ev.step = chord_idx;
        ev.num_steps = num_steps;
        ev.t_chord_ms = t_chord;
        ev.window_end_ms = effective_end;
        ev.elapsed_ms = elapsed;
        emit(ev);
      }
      reset_pattern();
      return;
    }
  }

  float peak = -300.0f;
  if (chord_present(spectrum_db, noise_floor, local_bg, onset_contrast,
                    chord_idx, peak)) {
    if (emit) {
      Event ev;
      ev.type = EventType::ChordMatch;
      ev.chime_index = chime_index;
      ev.tick_index = tick_index;
      ev.now_ms = now_ms;
      ev.step = chord_idx;
      ev.num_steps = num_steps;
      ev.elapsed_ms = elapsed;
      ev.t_chord_ms = t_chord;
      ev.peak_db = peak;
      emit(ev);
    }
    match_index++;
    need_falling_edge = true;
    if (match_index >= num_steps) {
      const uint32_t elapsed_now = now_ms - pattern_start_ms;
      if (elapsed_now >= cfg_.min_duration_ms) {
        latch_detection_(now_ms, elapsed_now, chime_index, tick_index, emit);
      } else {
        if (emit) {
          Event ev;
          ev.type = EventType::MinDurationDiscount;
          ev.chime_index = chime_index;
          ev.tick_index = tick_index;
          ev.now_ms = now_ms;
          ev.elapsed_ms = elapsed_now;
          ev.min_ms = cfg_.min_duration_ms;
          ev.num_steps = num_steps;
          emit(ev);
        }
        reset_pattern();
      }
    }
  }
}

} // namespace esphome::chime::core
