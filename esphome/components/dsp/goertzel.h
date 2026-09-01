#pragma once
// dsp/goertzel.h — shared Goertzel + Hann helpers for chime & frequency.
// Platform-agnostic, no ESPHome dependencies. Used by ChimeEngine and
// FrequencyEngine (and WASM via the engines).

#include <cmath>
#include <cstdint>
#include <vector>

namespace esphome::dsp {

static constexpr float kDbFloor = -300.0f;

// Hann window: w[i] = 0.5*(1 - cos(2*pi*i/(N-1)))
inline void make_hann_window(float *out, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    out[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1)));
  }
}
inline std::vector<float> make_hann_window_vec(uint32_t n) {
  std::vector<float> w(n);
  make_hann_window(w.data(), n);
  return w;
}

// Goertzel coefficient: 2*cos(2*pi*k/N) where k = f*N/fs (continuous, not integer).
inline float goertzel_coeff(float freq_hz, float sample_rate_hz, uint32_t window_size) {
  float k = (freq_hz * static_cast<float>(window_size)) / sample_rate_hz;
  return 2.0f * cosf(2.0f * (float)M_PI * k / static_cast<float>(window_size));
}

// Clamp frequency to analyzable range [20, nyquist-20] (matches ChimeEngine).
inline float clamp_freq(float f, float sample_rate_hz) {
  if (f < 20.0f) f = 20.0f;
  float nyquist = sample_rate_hz / 2.0f;
  if (f > nyquist - 20.0f) f = nyquist - 20.0f;
  return f;
}

// Single-frame Goertzel: run IIR over N samples for nf bins, accumulate power.
// v1/v2 are state (must be zeroed before frame), c2 are coefficients, window is Hann.
inline void goertzel_frame(const int16_t *samples, const float *window, const float *c2, float *v1,
                           float *v2, float *accum, uint32_t n, uint32_t nf) {
  for (uint32_t t = 0; t < nf; ++t) {
    v1[t] = 0.0f;
    v2[t] = 0.0f;
  }
  for (uint32_t i = 0; i < n; ++i) {
    const float x = (static_cast<float>(samples[i]) / 32768.0f) * window[i];
    for (uint32_t t = 0; t < nf; ++t) {
      const float v = c2[t] * v1[t] - v2[t] + x;
      v2[t] = v1[t];
      v1[t] = v;
    }
  }
  for (uint32_t t = 0; t < nf; ++t) {
    const float a = v1[t], b = v2[t];
    accum[t] += a * a + b * b - c2[t] * a * b;
  }
}

// dB conversion for a bin-centered full-scale sine (Hann halves in-band energy).
inline float power_to_db(float power, float ref_n) {
  return (power > 0.0f) ? 10.0f * log10f(power / ref_n) : kDbFloor;
}

// Parabolic sub-bin refinement on log-magnitude around peak (for frequency).
// p_m1, peak, p_p1 are linear powers (>0). Returns alpha in [-0.5, 0.5].
inline float parabolic_alpha(float p_m1, float p0, float p_p1) {
  if (p_m1 <= 0.0f || p0 <= 0.0f || p_p1 <= 0.0f) return 0.0f;
  float db_m1 = 0.5f * log10f(p_m1);
  float db_0 = 0.5f * log10f(p0);
  float db_p1 = 0.5f * log10f(p_p1);
  float denom = db_m1 - 2.0f * db_0 + db_p1;
  if (denom == 0.0f) return 0.0f;
  float alpha = 0.5f * (db_m1 - db_p1) / denom;
  if (alpha > 0.5f) alpha = 0.5f;
  if (alpha < -0.5f) alpha = -0.5f;
  return alpha;
}

}  // namespace esphome::dsp
