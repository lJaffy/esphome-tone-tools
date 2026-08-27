// esphome/components/chime/wasm/bindings.cpp — Emscripten embind for chime core.
// Built with emsdk 3.1.6: emcc ../core/chime_pattern.cpp ../core/chime_engine.cpp bindings.cpp --bind -o chime.js
// Also compiles as plain C++ (no-op) for syntax checking without emcc.

#include "../core/chime_engine.h"
#include "../core/chime_types.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/bind.h>
#include <emscripten/val.h>
using namespace emscripten;
using namespace esphome::chime::core;

// Helpers to convert JS arrays to C++ vectors
static ChimeConfig jsToChimeConfig(const val &js) {
  ChimeConfig c;
  if (js.hasOwnProperty("name")) c.name = js["name"].as<std::string>();
  if (js.hasOwnProperty("pattern")) {
    val pat = js["pattern"];
    size_t n = pat["length"].as<size_t>();
    c.pattern.resize(n);
    for (size_t i = 0; i < n; ++i) {
      val chord = pat[i];
      size_t m = chord["length"].as<size_t>();
      c.pattern[i].resize(m);
      for (size_t j = 0; j < m; ++j) c.pattern[i][j] = chord[j].as<float>();
    }
  }
  if (js.hasOwnProperty("times")) {
    val times = js["times"];
    size_t n = times["length"].as<size_t>();
    c.pattern_times_ms.resize(n);
    for (size_t i = 0; i < n; ++i) {
      val v = times[i];
      if (v.isNull() || v.isUndefined()) c.pattern_times_ms[i] = NO_TIME;
      else c.pattern_times_ms[i] = (uint32_t)(v.as<double>() * 1000.0);
    }
  } else if (js.hasOwnProperty("patternTimesMs")) {
    val times = js["patternTimesMs"];
    size_t n = times["length"].as<size_t>();
    c.pattern_times_ms.resize(n);
    for (size_t i = 0; i < n; ++i) c.pattern_times_ms[i] = times[i].as<uint32_t>();
  }
  if (js.hasOwnProperty("minDurationMs")) c.min_duration_ms = js["minDurationMs"].as<uint32_t>();
  if (js.hasOwnProperty("maxDurationMs")) c.max_duration_ms = js["maxDurationMs"].as<uint32_t>();
  if (js.hasOwnProperty("thresholdDb")) c.threshold_db = js["thresholdDb"].as<float>();
  if (js.hasOwnProperty("snrMarginDb")) c.snr_margin_db = js["snrMarginDb"].as<float>();
  if (js.hasOwnProperty("prominenceDb")) c.prominence_db = js["prominenceDb"].as<float>();
  if (js.hasOwnProperty("onsetContrastDb")) c.onset_contrast_db = js["onsetContrastDb"].as<float>();
  if (js.hasOwnProperty("tailGraceMs")) c.tail_grace_ms = js["tailGraceMs"].as<uint32_t>();
  c.release_time_ms = c.max_duration_ms + 2000;
  return c;
}

static DetectorConfig jsToDetectorConfig(const val &js) {
  DetectorConfig d;
  if (!js.isNull() && !js.isUndefined()) {
    if (js.hasOwnProperty("windowSize")) d.window_size = js["windowSize"].as<uint16_t>();
    if (js.hasOwnProperty("tickIntervalMs")) d.tick_interval_ms = js["tickIntervalMs"].as<uint32_t>();
    if (js.hasOwnProperty("guardSeparationHz")) d.guard_separation_hz = js["guardSeparationHz"].as<float>();
    if (js.hasOwnProperty("noiseFloorAlphaDown")) d.noise_floor_alpha_down = js["noiseFloorAlphaDown"].as<float>();
    if (js.hasOwnProperty("noiseFloorAlphaUp")) d.noise_floor_alpha_up = js["noiseFloorAlphaUp"].as<float>();
    // legacy aliases
    if (js.hasOwnProperty("window_size")) d.window_size = js["window_size"].as<uint16_t>();
    if (js.hasOwnProperty("tick_interval_ms")) d.tick_interval_ms = js["tick_interval_ms"].as<uint32_t>();
  }
  return d;
}

static val eventToJs(const Event &e) {
  val o = val::object();
  const char *typeStr = "unknown";
  switch (e.type) {
    case EventType::PatternStart: typeStr = "pattern_start"; break;
    case EventType::FallingEdge: typeStr = "falling_edge"; break;
    case EventType::ChordMatch: typeStr = "chord_match"; break;
    case EventType::Detected: typeStr = "detected"; break;
    case EventType::MinDurationDiscount: typeStr = "min_duration_discount"; break;
    case EventType::StepTimeout: typeStr = "step_timeout"; break;
    case EventType::MaxDurationTimeout: typeStr = "max_duration_timeout"; break;
    case EventType::Released: typeStr = "released"; break;
  }
  o.set("type", val(typeStr));
  o.set("chime", val(e.chime_index));
  o.set("tick", val(e.tick_index));
  o.set("ms", val(e.now_ms));
  o.set("step", val(e.step));
  o.set("numSteps", val(e.num_steps));
  o.set("peakDb", val(e.peak_db));
  o.set("elapsed", val(e.elapsed_ms));
  o.set("windowEnd", val(e.window_end_ms));
  o.set("tChord", val(e.t_chord_ms));
  o.set("holdMs", val(e.hold_ms));
  return o;
}

// Thin wrapper class exposed to JS – mirrors js/chime.js ChimeDetector API
class WasmChimeDetector {
 public:
  WasmChimeDetector(const val &monoVal, double fs, const val &opts) {
    // monoVal is Float32Array
    size_t len = monoVal["length"].as<size_t>();
    mono_.resize(len);
    val sliced = monoVal.call<val>("slice");
    for (size_t i = 0; i < len; ++i) mono_[i] = sliced[i].as<float>();
    // Alternative fast path if slice not available: read via HEAPF32
    // Fallback handled above.
    fs_ = (float)fs;
    DetectorConfig dcfg = jsToDetectorConfig(opts);
    if (opts.hasOwnProperty("chimes")) {
      val chimes = opts["chimes"];
      size_t n = chimes["length"].as<size_t>();
      for (size_t i = 0; i < n; ++i) {
        ChimeConfig cc = jsToChimeConfig(chimes[i]);
        dcfg_chimes_.push_back(cc);
      }
    }
    engine_ = std::make_unique<ChimeEngine>(dcfg);
    for (auto &cc : dcfg_chimes_) engine_->add_chime(cc);
  }

  val run() {
    RunResult r = engine_->run_offline(mono_.data(), mono_.size(), fs_);
    val out = val::object();
    val meta = val::object();
    meta.set("sampleRateHz", val(r.meta.sample_rate_hz));
    meta.set("windowSize", val(r.meta.window_size));
    meta.set("tickIntervalMs", val(r.meta.tick_interval_ms));
    meta.set("framesPerTick", val(r.meta.frames_per_tick));
    meta.set("tickAudioMs", val(r.meta.tick_audio_ms));
    meta.set("guardSeparationHz", val(r.meta.guard_separation_hz));
    meta.set("totalTicks", val(r.meta.total_ticks));
    meta.set("durationMs", val(r.meta.duration_ms));
    meta.set("bins", val(r.meta.bins));
    val freqs = val::array();
    for (size_t i = 0; i < r.meta.freqs.size(); ++i) freqs.set(i, val(r.meta.freqs[i]));
    meta.set("freqs", freqs);
    out.set("meta", meta);

    val events = val::array();
    for (size_t i = 0; i < r.events.size(); ++i) {
      // enrich with chimeName like js/chime.js
      val ev = eventToJs(r.events[i]);
      if (r.events[i].chime_index < dcfg_chimes_.size())
        ev.set("chimeName", val(dcfg_chimes_[r.events[i].chime_index].name));
      events.set(i, ev);
    }
    out.set("events", events);

    val ticks = val::array();
    for (size_t i = 0; i < r.ticks.size(); ++i) {
      const auto &t = r.ticks[i];
      val tj = val::object();
      tj.set("tick", val(t.tick));
      tj.set("ms", val(t.ms));
      tj.set("durationMs", val(t.duration_ms));
      val bins = val::array();
      for (size_t b = 0; b < t.bins.size(); ++b) {
        val bj = val::object();
        bj.set("freq", val(t.bins[b].freq));
        bj.set("db", val(t.bins[b].db));
        if (t.bins[b].floor_valid) {
          bj.set("floor", t.bins[b].floor);
        } else {
          bj.set("floor", val::null());
        }
        bj.set("localBg", val(t.bins[b].local_bg));
        bj.set("onset", val(t.bins[b].onset));
        bins.set(b, bj);
      }
      tj.set("bins", bins);
      val diag = val::array();
      for (size_t d = 0; d < t.diag.size(); ++d) {
        val dj = val::object();
        dj.set("name", val(t.diag[d].name));
        dj.set("active", val(t.diag[d].active));
        dj.set("step", val(t.diag[d].step));
        dj.set("numSteps", val(t.diag[d].num_steps));
        dj.set("timed", val(t.diag[d].timed));
        dj.set("timeMs", val(t.diag[d].time_ms));
        val dbins = val::array();
        for (size_t b = 0; b < t.diag[d].bins.size(); ++b) {
          const auto &bb = t.diag[d].bins[b];
          val bj = val::object();
          bj.set("bin", val(bb.bin));
          bj.set("freq", val(bb.freq));
          bj.set("db", val(bb.db));
          bj.set("effThreshold", val(bb.eff_threshold));
          bj.set("thresholdOk", val(bb.threshold_ok));
          bj.set("localBg", val(bb.local_bg));
          bj.set("prominence", val(bb.prominence));
          bj.set("prominenceOk", val(bb.prominence_ok));
          bj.set("onset", val(bb.onset));
          bj.set("onsetOk", val(bb.onset_ok));
          dbins.set(b, bj);
        }
        dj.set("bins", dbins);
        diag.set(d, dj);
      }
      tj.set("diag", diag);
      // compat: also expose chimes snapshot like js version's tick.chimes
      val chSnap = val::array();
      for (size_t ci = 0; ci < dcfg_chimes_.size(); ++ci) {
        // Derive from engine state? For offline we don't have per-chime snapshot separate; approximate from diag
        val cj = val::object();
        cj.set("name", val(t.diag[ci].name));
        cj.set("active", val(t.diag[ci].active));
        chSnap.set(ci, cj);
      }
      tj.set("chimes", chSnap);
      ticks.set(i, tj);
    }
    out.set("ticks", ticks);
    return out;
  }

 private:
  std::vector<float> mono_;
  float fs_;
  std::vector<ChimeConfig> dcfg_chimes_;
  std::unique_ptr<ChimeEngine> engine_;
};

// Factory function: create detector, call run() in one shot for convenience
val runChimeDetector(const val &mono, double fs, const val &opts) {
  WasmChimeDetector d(mono, fs, opts);
  return d.run();
}

EMSCRIPTEN_BINDINGS(chime_wasm) {
  class_<WasmChimeDetector>("WasmChimeDetector")
      .constructor<val, double, val>()
      .function("run", &WasmChimeDetector::run);

  function("runChimeDetector", &runChimeDetector);

  // Expose constants for JS compat
  constant("NO_TIME", (uint32_t)NO_TIME);
  constant("ONSET_LOOKBACK", (uint32_t)ONSET_LOOKBACK);
  constant("MAX_FRAMES_PER_TICK", (uint32_t)MAX_FRAMES_PER_TICK);
}

#else
// Non-emscripten stub so file still parses with plain g++ without -DEMSCRIPTEN
int dummy_chime_wasm_bindings = 0;
#endif
