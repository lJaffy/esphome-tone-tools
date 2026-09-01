#pragma once

#ifdef USE_ESP32

#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "frequency_engine.h"
#include "../dsp/sample_source.h"

#if __has_include(<esp_adc/adc_continuous.h>)
#include <esp_adc/adc_continuous.h>
#else
#include <driver/adc.h>
#endif

namespace esphome::frequency {

class FrequencyComponent : public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void set_measurement_duration(uint32_t ms) { measurement_duration_ms_ = ms; }
  void set_microphone_source(microphone::MicrophoneSource *mic) {
    mic_source_ = mic;
    has_mic_ = (mic != nullptr);
  }
  void set_adc_pin(uint8_t pin) {
    adc_pin_ = pin;
    has_adc_ = true;
  }
  void set_adc_attenuation(adc_atten_t atten) { adc_atten_ = atten; }
  void set_adc_sample_rate(uint32_t sr) { adc_sample_rate_ = sr; }
  void set_adc_gain(uint32_t g) { adc_gain_ = g; }
  void set_passive(bool p) { passive_ = p; }

  void set_window_size(uint16_t n) { window_size_ = n; }
  void set_min_frequency_hz(float f) { min_frequency_hz_ = f; }
  void set_max_frequency_hz(float f) { max_frequency_hz_ = f; }
  void set_peak_threshold_db(float db) { peak_threshold_db_ = db; }
  void set_frequency_sensor(sensor::Sensor *s) { frequency_sensor_ = s; }
  void set_peak_magnitude_sensor(sensor::Sensor *s) { peak_magnitude_sensor_ = s; }

  void start();
  void stop();

  bool has_microphone() const { return has_mic_; }
  bool has_adc() const { return has_adc_; }

 protected:
  bool ensure_engine_();
  void handle_result_(const core::FrequencyResult &res);

  // Config
  microphone::MicrophoneSource *mic_source_{nullptr};
  bool has_mic_{false};
  bool has_adc_{false};
  uint8_t adc_pin_{255};
  adc_atten_t adc_atten_{ADC_ATTEN_DB_11};
  uint32_t adc_sample_rate_{16000};
  uint32_t adc_gain_{1};
  bool passive_{false};

  uint16_t window_size_{1024};
  float min_frequency_hz_{100.0f};
  float max_frequency_hz_{12000.0f};
  float peak_threshold_db_{-50.0f};
  uint32_t measurement_duration_ms_{1000};

  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *peak_magnitude_sensor_{nullptr};

  // Engine (pure DSP)
  core::FrequencyEngine engine_;
  bool engine_ready_{false};
  float sample_rate_hz_{0.0f};

  // Sample sources (wrappers around dsp helpers)
  dsp::MicrophoneSampleSource mic_adapter_;
  dsp::AdcSampleSource adc_adapter_;

  // Temp read buffer (stack-friendly, heap allocated once)
  int16_t *read_buf_{nullptr};
  static constexpr size_t kReadBufSamples = 1024;
};

template<typename... Ts> class StartAction : public Action<Ts...>, public Parented<FrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start(); }
};

template<typename... Ts> class StopAction : public Action<Ts...>, public Parented<FrequencyComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop(); }
};

}  // namespace esphome::frequency

#endif
