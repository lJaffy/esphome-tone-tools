#pragma once
// dsp/sample_source.h — unified audio/ADC sample-source abstraction for chime + frequency.
// Platform-agnostic interface + ESP32 helpers. The engines themselves never
// include this; only the Component glue does.

#ifdef USE_ESP32
#include <cstdint>
#include <memory>
#include <vector>

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"

#if __has_include(<esp_adc/adc_continuous.h>)
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <hal/adc_types.h>
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

namespace esphome::dsp {

// ── Microphone source wrapper ──────────────────────────────────────────────
class MicrophoneSampleSource {
 public:
  MicrophoneSampleSource() = default;
  ~MicrophoneSampleSource() { stop(); }

  void set_microphone(microphone::MicrophoneSource *mic) { mic_ = mic; }

  bool start();
  void stop();
  bool is_running() const { return audio_source_ != nullptr; }
  float sample_rate_hz() const { return sample_rate_hz_; }
  void set_sample_rate(float sr) { sample_rate_hz_ = sr; }

  // Stage samples: fill ring buffer, consume up to max_samples into out, return count.
  // Returns 0 if no data or not running.
  size_t read(int16_t *out, size_t max_samples);

  // Call from Component::setup() to hook mic callback
  void setup_callback();

 private:
  microphone::MicrophoneSource *mic_{nullptr};
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
  float sample_rate_hz_{0.0f};
  static constexpr uint32_t kMaxFillMs = 30;
  static constexpr uint32_t kRingBufferMs = 120;
};

// ── ADC continuous source wrapper ─────────────────────────────────────────
class AdcSampleSource {
 public:
  AdcSampleSource() = default;
  ~AdcSampleSource() { stop(); }

  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_attenuation(adc_atten_t atten) { atten_ = atten; }
  void set_sample_rate(uint32_t sr) { sample_rate_hz_ = sr; }
  void set_gain(uint32_t gain) { gain_ = gain; }  // linear 1..64

  uint8_t pin() const { return pin_; }
  uint32_t sample_rate() const { return sample_rate_hz_; }

  bool start();
  void stop();
  bool is_running() const { return running_; }
  float sample_rate_hz_float() const { return static_cast<float>(sample_rate_hz_); }

  // Poll ADC DMA and stage into out buffer (up to max_samples). Returns count staged.
  // Applies midpoint tracking + 20 Hz HPF + 12→16-bit scaling + gain.
  size_t read(int16_t *out, size_t max_samples);

  void update_hpf_alpha();

 private:
  uint8_t pin_{255};
  adc_atten_t atten_{ADC_ATTEN_DB_11};
  uint32_t sample_rate_hz_{16000};
  uint32_t gain_{1};
  bool running_{false};

  adc_continuous_handle_t handle_{nullptr};
  adc_cali_handle_t cali_handle_{nullptr};
  uint8_t *raw_buf_{nullptr};
  size_t raw_buf_len_{0};
  static constexpr size_t kRawBufBytes = 2048;

  int32_t midpoint_{2048};
  float hpf_x_prev_{0.0f};
  float hpf_y_prev_{0.0f};
  float hpf_alpha_{0.995f};
};

// Inline implementations to keep header-only for simple inclusion.
// Non-trivial definitions are in sample_source.cpp if needed.

}  // namespace esphome::dsp

#endif  // USE_ESP32
