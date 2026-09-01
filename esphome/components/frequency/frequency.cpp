#include "frequency.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cmath>
#include <cstring>

namespace esphome::frequency {

static const char *const TAG = "frequency";

void FrequencyComponent::dump_config() {
  if (has_adc_ && has_mic_) {
    ESP_LOGE(TAG, "Invalid config: both microphone and adc configured (exclusive)");
  }
  const char *src = has_adc_ ? "ADC" : (has_mic_ ? "Microphone" : "None");
  ESP_LOGCONFIG(TAG,
                "Frequency Component:\n"
                "  Source: %s\n"
                "  Measurement Duration: %u ms\n"
                "  Window Size: %u samples\n"
                "  Min Frequency: %.1f Hz\n"
                "  Max Frequency: %.1f Hz\n"
                "  Peak Threshold: %.1f dB",
                src, measurement_duration_ms_, window_size_, min_frequency_hz_, max_frequency_hz_, peak_threshold_db_);
  if (has_adc_) {
    ESP_LOGCONFIG(TAG, "  ADC pin: GPIO%u", (unsigned)adc_pin_);
    ESP_LOGCONFIG(TAG, "  ADC sample rate: %u Hz", (unsigned)adc_sample_rate_);
    ESP_LOGCONFIG(TAG, "  ADC attenuation: %d", (int)adc_atten_);
    ESP_LOGCONFIG(TAG, "  ADC gain: %u", (unsigned)adc_gain_);
    ESP_LOGCONFIG(TAG, "  ADC passive: %s", passive_ ? "true" : "false");
  } else if (has_mic_) {
    ESP_LOGCONFIG(TAG, "  Microphone passive: %s", passive_ ? "true" : "false");
  }
  LOG_SENSOR("  ", "Frequency:", frequency_sensor_);
  LOG_SENSOR("  ", "Peak Magnitude:", peak_magnitude_sensor_);
}

bool FrequencyComponent::ensure_engine_() {
  if (engine_ready_) return true;
  core::FrequencyConfig cfg;
  cfg.window_size = window_size_;
  cfg.min_frequency_hz = min_frequency_hz_;
  cfg.max_frequency_hz = max_frequency_hz_;
  cfg.peak_threshold_db = peak_threshold_db_;
  cfg.measurement_duration_ms = measurement_duration_ms_;
  engine_.set_config(cfg);
  if (sample_rate_hz_ > 0) engine_.set_sample_rate(sample_rate_hz_);
  engine_ready_ = true;
  return true;
}

void FrequencyComponent::setup() {
  if (has_mic_ && has_adc_) {
    ESP_LOGE(TAG, "Frequency: exclusive source violated – both mic and adc set. Choose one.");
    mark_failed();
    return;
  }
  if (!has_mic_ && !has_adc_) {
    ESP_LOGE(TAG, "Frequency: no source configured – set either 'microphone:' or 'adc:'");
    mark_failed();
    return;
  }

  ensure_engine_();

  if (has_mic_) {
    if (mic_source_ == nullptr) {
      ESP_LOGE(TAG, "Microphone source is null");
      mark_failed();
      return;
    }
    mic_adapter_.set_microphone(mic_source_);
    mic_adapter_.setup_callback();
    if (!passive_ && !mic_source_->is_passive()) mic_source_->start();
  } else {
    adc_adapter_.set_pin(adc_pin_);
    adc_adapter_.set_attenuation(adc_atten_);
    adc_adapter_.set_sample_rate(adc_sample_rate_);
    adc_adapter_.set_gain(adc_gain_);
    if (!passive_) {
      if (!adc_adapter_.start()) {
        ESP_LOGE(TAG, "ADC start failed");
      }
    }
  }

  read_buf_ = static_cast<int16_t *>(malloc(kReadBufSamples * sizeof(int16_t)));
  if (read_buf_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate read buffer");
    mark_failed();
    return;
  }
}

void FrequencyComponent::loop() {
  if ((frequency_sensor_ == nullptr) && (peak_magnitude_sensor_ == nullptr)) return;
  if (!engine_ready_ && !ensure_engine_()) return;

  // ── Source-agnostic loop ──
  if (has_adc_) {
    if (!adc_adapter_.is_running()) {
      if (!passive_) {
        // Try to restart if not passive
        adc_adapter_.start();
      }
      if (!adc_adapter_.is_running()) {
        if (!status_has_warning()) status_set_warning(LOG_STR("ADC is not running"));
        if (frequency_sensor_ != nullptr) frequency_sensor_->publish_state(NAN);
        if (peak_magnitude_sensor_ != nullptr) peak_magnitude_sensor_->publish_state(NAN);
        return;
      }
    }
    if (status_has_warning()) status_clear_warning();
    if (sample_rate_hz_ == 0.0f) {
      sample_rate_hz_ = static_cast<float>(adc_sample_rate_);
      engine_.set_sample_rate(sample_rate_hz_);
      ESP_LOGI(TAG, "Goertzel band %.0f-%.0f Hz -> %u bins, fs=%u Hz, N=%u", min_frequency_hz_,
               max_frequency_hz_, (unsigned)engine_.num_bins(), (unsigned)adc_sample_rate_, (unsigned)window_size_);
    }
    size_t nread = adc_adapter_.read(read_buf_, kReadBufSamples);
    if (nread > 0) engine_.feed_pcm(read_buf_, nread);
    core::FrequencyResult res;
    if (engine_.try_emit_window(res)) handle_result_(res);
    return;
  }

  // Microphone path
  if (mic_source_ == nullptr) return;
  // Ensure adapter running when mic is running
  if (mic_source_->is_running() && !status_has_error()) {
    if (!mic_adapter_.is_running()) {
      if (!mic_adapter_.start()) {
        ESP_LOGW(TAG, "Mic adapter start failed");
        return;
      }
    }
    if (status_has_warning()) status_clear_warning();
  } else {
    if (!status_has_warning()) {
      status_set_warning(LOG_STR("Microphone is not running, can't compute dominant frequency"));
      if (frequency_sensor_ != nullptr) frequency_sensor_->publish_state(NAN);
      if (peak_magnitude_sensor_ != nullptr) peak_magnitude_sensor_->publish_state(NAN);
      engine_.reset_streaming_state();
    }
    return;
  }
  if (status_has_error()) return;

  const auto &info = mic_source_->get_audio_stream_info();
  if (sample_rate_hz_ == 0.0f) {
    sample_rate_hz_ = static_cast<float>(info.get_sample_rate());
    engine_.set_sample_rate(sample_rate_hz_);
    ESP_LOGI(TAG, "Goertzel band %.0f-%.0f Hz -> %u bins, fs=%u Hz, N=%u", min_frequency_hz_, max_frequency_hz_,
             (unsigned)engine_.num_bins(), (unsigned)info.get_sample_rate(), (unsigned)window_size_);
  }

  // Pump samples from adapter into engine; emit windows as they become ready
  while (true) {
    core::FrequencyResult res;
    if (engine_.try_emit_window(res)) {
      handle_result_(res);
      continue;
    }
    size_t nread = mic_adapter_.read(read_buf_, kReadBufSamples);
    if (nread == 0) break;
    engine_.feed_pcm(read_buf_, nread);
  }
}

void FrequencyComponent::handle_result_(const core::FrequencyResult &res) {
  if (peak_magnitude_sensor_ != nullptr) peak_magnitude_sensor_->publish_state(res.peak_db);
  if (frequency_sensor_ != nullptr) {
    float out = res.valid ? res.frequency_hz : NAN;
    frequency_sensor_->publish_state(out);
  }
  ESP_LOGW(TAG, "Window result: bin %u (%.1f Hz), peak %.2f dB vs threshold %.1f dB -> %s", (unsigned)res.bin,
           res.raw_bin_hz, res.peak_db, peak_threshold_db_, res.valid ? "publish" : "suppress");
}

void FrequencyComponent::start() {
  if (has_adc_) {
    if (passive_) {
      ESP_LOGW(TAG, "Can't start ADC in passive mode");
      return;
    }
    adc_adapter_.start();
    return;
  }
  if (mic_source_ == nullptr) return;
  if (mic_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't start the microphone in passive mode");
    return;
  }
  mic_source_->start();
}

void FrequencyComponent::stop() {
  if (has_adc_) {
    if (passive_) {
      ESP_LOGW(TAG, "Can't stop ADC in passive mode");
      return;
    }
    adc_adapter_.stop();
    return;
  }
  if (mic_source_ == nullptr) return;
  if (mic_source_->is_passive()) {
    ESP_LOGW(TAG, "Can't stop the microphone in passive mode");
    return;
  }
  mic_source_->stop();
}

}  // namespace esphome::frequency

#endif
