#pragma once
// chime/core/chime_types.h — platform-agnostic constants and plain data types.
// No ESPHome / Arduino / Emscripten dependencies. Compiles for ESP-IDF and WASM.

#include <cstdint>
#include <string>
#include <vector>

namespace esphome::chime::core {

static constexpr uint32_t NO_TIME = 0xFFFFFFFF;
static constexpr uint32_t ONSET_LOOKBACK = 5;
static constexpr uint32_t MAX_FRAMES_PER_TICK = 16;

// Shared detector configuration (one per ChimeEngine instance).
struct DetectorConfig {
  uint16_t window_size = 1024;
  uint32_t tick_interval_ms = 100;
  float guard_separation_hz = 150.0f;
  float noise_floor_alpha_down = 0.05f;
  float noise_floor_alpha_up = 0.005f;
};

// Per-chime pattern configuration.
struct ChimeConfig {
  std::string name;
  // One entry per step; each inner vector is the chord frequencies (Hz) for that step.
  std::vector<std::vector<float>> pattern;
  // One entry per step; NO_TIME == unconstrained timing for that step.
  std::vector<uint32_t> pattern_times_ms;
  uint32_t min_duration_ms = 2000;
  uint32_t max_duration_ms = 5000;
  float threshold_db = -50.0f;
  float snr_margin_db = 8.0f;
  float prominence_db = 6.0f;
  float onset_contrast_db = 8.0f;
  uint32_t tail_grace_ms = 2000;
  // Derived, kept in sync with max_duration_ms (max + 2000) like Chime::set_max_duration_ms.
  uint32_t release_time_ms = 7000;
};

enum class EventType : uint8_t {
  PatternStart,
  FallingEdge,
  ChordMatch,
  Detected,
  MinDurationDiscount,
  StepTimeout,
  MaxDurationTimeout,
  Released,
};

struct Event {
  EventType type = EventType::PatternStart;
  uint32_t chime_index = 0;
  uint32_t tick_index = 0;
  uint32_t now_ms = 0;
  // Optional payload (interpret per type)
  uint32_t step = 0;
  uint32_t num_steps = 0;
  float peak_db = -300.0f;
  uint32_t elapsed_ms = 0;
  uint32_t window_end_ms = 0;
  uint32_t t_chord_ms = 0;
  uint32_t hold_ms = 0;
  uint32_t min_ms = 0;
  uint32_t max_ms = 0;
  uint32_t matched = 0;
};

// Per-tick diagnostic snapshot for the JS chart (wasm facade).
struct TickBin {
  float freq = 0;
  float db = -300;
  float floor = 0;
  bool floor_valid = false;
  float local_bg = -300;
  float onset = 0;
};

struct TickDiagBin {
  uint32_t bin = 0;
  float freq = 0;
  float db = -300;
  float eff_threshold = -300;
  bool threshold_ok = false;
  float local_bg = -300;
  float prominence = 0;
  bool prominence_ok = false;
  float onset = 0;
  bool onset_ok = false;
};

struct TickDiagChime {
  std::string name;
  bool active = false;
  uint32_t step = 0;
  uint32_t num_steps = 0;
  bool timed = false;
  uint32_t time_ms = NO_TIME;
  std::vector<TickDiagBin> bins;
};

struct TickSnapshot {
  uint32_t tick = 0;
  uint32_t ms = 0;
  float duration_ms = 0;
  std::vector<TickBin> bins;
  std::vector<TickDiagChime> diag;
};

struct RunResult {
  struct Meta {
    float sample_rate_hz = 0;
    uint16_t window_size = 0;
    uint32_t tick_interval_ms = 0;
    uint32_t frames_per_tick = 0;
    float tick_audio_ms = 0;
    float guard_separation_hz = 0;
    uint32_t total_ticks = 0;
    float duration_ms = 0;
    uint32_t bins = 0;
    std::vector<float> freqs;
  } meta;
  std::vector<Event> events;
  std::vector<TickSnapshot> ticks;
};

}  // namespace esphome::chime::core
