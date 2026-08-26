#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome::sound_frequency {

class SoundFrequencyComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void set_measurement_duration(uint32_t measurement_duration_ms) {
    this->measurement_duration_ms_ = measurement_duration_ms;
  }
  void set_microphone_source(microphone::MicrophoneSource *microphone_source) {
    this->microphone_source_ = microphone_source;
  }
  void set_window_size(uint16_t window_size) { this->window_size_ = window_size; }
  void set_min_frequency_hz(float min_frequency_hz) { this->min_frequency_hz_ = min_frequency_hz; }
  void set_max_frequency_hz(float max_frequency_hz) { this->max_frequency_hz_ = max_frequency_hz; }
  void set_peak_threshold_db(float peak_threshold_db) { this->peak_threshold_db_ = peak_threshold_db; }
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { this->frequency_sensor_ = frequency_sensor; }
  void set_peak_magnitude_sensor(sensor::Sensor *peak_magnitude_sensor) {
    this->peak_magnitude_sensor_ = peak_magnitude_sensor;
  }

  /// @brief Starts the MicrophoneSource to start measuring the dominant frequency
  void start();

  /// @brief Stops the MicrophoneSource
  void stop();

 protected:
  /// @brief Internal start command that, if necessary, allocates a ring buffer and a zero-copy
  /// ``RingBufferAudioSource`` that reads directly from it. ``ring_buffer_`` weakly references the
  /// ring buffer owned by ``audio_source_``. Returns true if allocations were successful.
  bool start_();

  /// @brief Internal stop command that deallocates ``audio_source_`` (which releases its ring buffer)
  void stop_();

  /// @brief Runs the Goertzel IIR recursion on one full N-sample window for all in-band bins
  /// and accumulates the resulting power into ``accum_``
  bool process_goertzel_frame_(const int16_t *samples);

  /// @brief Averages the accumulated power spectrum, picks the in-band peak, refines it sub-bin, and publishes
  void emit_window_();

  microphone::MicrophoneSource *microphone_source_;

  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *peak_magnitude_sensor_{nullptr};

  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;

  uint16_t window_size_{1024};
  float min_frequency_hz_{100.0f};
  float max_frequency_hz_{12000.0f};
  float peak_threshold_db_{-50.0f};

  uint32_t measurement_duration_ms_{1000};

  // DSP working set, allocated once in setup() – no heap is touched from loop() after that
  float *window_{nullptr};       ///< Hann window coefficients, N floats
  float *accum_{nullptr};        ///< averaged power spectrum accumulator, num_bins_ floats
  float *goertzel_v1_{nullptr};  ///< IIR state v1 (previous sample), num_bins_ floats
  float *goertzel_v2_{nullptr};  ///< IIR state v2 (sample before previous), num_bins_ floats
  float *goertzel_c2_{nullptr};  ///< precomputed 2·cos(2πk/N) per bin, num_bins_ floats

  uint32_t k_min_{0};     ///< first in-band DFT bin index (excludes DC)
  uint32_t k_max_{0};     ///< last in-band DFT bin index (excludes Nyquist)
  uint32_t num_bins_{0};  ///< number of Goertzel filters = k_max - k_min + 1 (valid once band is known)

  float sample_rate_hz_{0.0f};  ///< runtime sample rate taken from the microphone device

  bool dsp_initialized_{false};
  bool band_valid_{false};

  uint32_t diagnostic_log_ms_{0};          ///< wall-clock time of the last periodic diagnostic log (ms)
  bool diagnostic_window_emitted_{false};  ///< true once the first measurement window has been emitted

  uint32_t frame_count_{0};          ///< Goertzel frames accumulated into ``accum_`` since the last window emit
  uint32_t window_sample_count_{0};  ///< samples consumed since the last window emit

  // Ring buffer for partial frames. The audio source only exposes up to MAX_FILL_DURATION_MS of audio per
  // fill() call, which is usually far less than a full N-sample window, so the window must be assembled from
  // several fill/consume cycles. Samples are staged here and run through Goertzel in whole-window units.
  // Allocated in start_() (re-created whenever the audio source is created), freed in stop_().
  int16_t *frame_buf_{nullptr};
  uint32_t frame_buf_offset_{0};  ///< samples currently staged in ``frame_buf_``
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<SoundFrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<SoundFrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::sound_frequency

#endif
