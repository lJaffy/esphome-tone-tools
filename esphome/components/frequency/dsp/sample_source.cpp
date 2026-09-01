#include "sample_source.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome::dsp {

static const char *const TAG = "dsp.sample_source";

// ── MicrophoneSampleSource ─────────────────────────────────────────────────

void MicrophoneSampleSource::setup_callback() {
  if (mic_ == nullptr) return;
  mic_->add_data_callback([this](const std::vector<uint8_t> &data) {
    auto rb = ring_buffer_.lock();
    if (rb != nullptr) rb->write((void *)data.data(), data.size());
  });
}

bool MicrophoneSampleSource::start() {
  if (audio_source_ != nullptr) return true;
  if (mic_ == nullptr) return false;
  const auto &info = mic_->get_audio_stream_info();
  const size_t bpf = info.frames_to_bytes(1);
  ring_buffer_.reset();
  const size_t rb_size = (info.ms_to_bytes(kRingBufferMs) / bpf) * bpf;
  auto rb = ring_buffer::RingBuffer::create(rb_size);
  if (rb == nullptr) return false;
  std::shared_ptr<ring_buffer::RingBuffer> rb_shared(std::move(rb));
  audio_source_ = audio::RingBufferAudioSource::create(rb_shared, info.ms_to_bytes(kMaxFillMs), (uint8_t)bpf);
  if (audio_source_ == nullptr) return false;
  ring_buffer_ = rb_shared;
  // Sample rate known only at runtime; caller should query info after start.
  return true;
}

void MicrophoneSampleSource::stop() {
  audio_source_.reset();
  ring_buffer_.reset();
}

size_t MicrophoneSampleSource::read(int16_t *out, size_t max_samples) {
  if (audio_source_ == nullptr || out == nullptr || max_samples == 0) return 0;
  if (mic_ == nullptr) return 0;
  const auto &info = mic_->get_audio_stream_info();
  audio_source_->fill(0, false);
  const uint32_t avail_samples = info.bytes_to_samples(audio_source_->available());
  if (avail_samples == 0) return 0;
  const uint32_t take = std::min<uint32_t>(avail_samples, (uint32_t)max_samples);
  const int16_t *data = reinterpret_cast<const int16_t *>(audio_source_->mutable_data());
  std::memcpy(out, data, info.samples_to_bytes(take));
  audio_source_->consume(info.samples_to_bytes(take));
  return take;
}

// ── AdcSampleSource ────────────────────────────────────────────────────────

void AdcSampleSource::update_hpf_alpha() {
  float fs = static_cast<float>(sample_rate_hz_);
  if (fs <= 0) return;
  const float rc = fs / (2.0f * (float)M_PI * 20.0f);
  hpf_alpha_ = rc / (rc + 1.0f);
  hpf_x_prev_ = 0.0f;
  hpf_y_prev_ = 0.0f;
}

bool AdcSampleSource::start() {
  if (running_) return true;
  if (pin_ == 255) {
    ESP_LOGE(TAG, "ADC pin not configured");
    return false;
  }
  if (raw_buf_ == nullptr) {
    raw_buf_ = static_cast<uint8_t *>(malloc(kRawBufBytes));
    if (raw_buf_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate ADC DMA buffer");
      return false;
    }
    raw_buf_len_ = kRawBufBytes;
  }

  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_0;
  bool pin_mapped = false;
#if __has_include("esp_adc/adc_oneshot.h")
  extern esp_err_t adc_oneshot_io_to_channel(int io_num, adc_unit_t *unit_id, adc_channel_t *chan);
  if (adc_oneshot_io_to_channel(pin_, &unit, &channel) == ESP_OK) pin_mapped = true;
#endif
  if (!pin_mapped) {
    ESP_LOGW(TAG, "ADC pin GPIO%u mapping fallback – assuming ADC_UNIT_1 CH0; check wiring", (unsigned)pin_);
    channel = ADC_CHANNEL_0;
    unit = ADC_UNIT_1;
  }
  if (unit != ADC_UNIT_1) {
    ESP_LOGW(TAG, "ADC continuous only supports ADC_UNIT_1 on this chip – using UNIT_1");
    unit = ADC_UNIT_1;
  }

  adc_continuous_handle_cfg_t handle_cfg{};
  handle_cfg.max_store_buf_size = kRawBufBytes * 2;
  handle_cfg.conv_frame_size = 256;
  esp_err_t err = adc_continuous_new_handle(&handle_cfg, &handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(err));
    return false;
  }

  adc_digi_pattern_config_t pattern{};
  pattern.atten = atten_;
  pattern.channel = channel & 0x7;
  pattern.unit = unit;
  pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t dig_cfg{};
  dig_cfg.pattern_num = 1;
  dig_cfg.adc_pattern = &pattern;
  dig_cfg.sample_freq_hz = sample_rate_hz_;
  dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

  err = adc_continuous_config(handle_, &dig_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(err));
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    return false;
  }

#if SOC_ADC_CALIB_SUPPORTED
  adc_cali_line_fitting_config_t cali_cfg{};
  cali_cfg.unit_id = unit;
  cali_cfg.atten = atten_;
  cali_cfg.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH;
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle_) != ESP_OK) {
    cali_handle_ = nullptr;
  }
#endif

  err = adc_continuous_start(handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_start failed: %s", esp_err_to_name(err));
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    return false;
  }

  running_ = true;
  update_hpf_alpha();
  midpoint_ = 2048;
  ESP_LOGI(TAG, "ADC started: GPIO%u unit %d ch %d @ %u Hz atten %d", (unsigned)pin_, (int)unit, (int)channel,
           (unsigned)sample_rate_hz_, (int)atten_);
  return true;
}

void AdcSampleSource::stop() {
  if (!running_) return;
  if (handle_ != nullptr) {
    adc_continuous_stop(handle_);
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
  }
#if SOC_ADC_CALIB_SUPPORTED
  if (cali_handle_ != nullptr) {
#if __has_include("esp_adc/adc_cali_scheme.h")
    adc_cali_delete_scheme_line_fitting(cali_handle_);
#endif
    cali_handle_ = nullptr;
  }
#endif
  running_ = false;
  if (raw_buf_ != nullptr) {
    free(raw_buf_);
    raw_buf_ = nullptr;
    raw_buf_len_ = 0;
  }
}

size_t AdcSampleSource::read(int16_t *out, size_t max_samples) {
  if (handle_ == nullptr || !running_ || out == nullptr || max_samples == 0) return 0;
  uint32_t out_len = 0;
  esp_err_t err = adc_continuous_read(handle_, raw_buf_, raw_buf_len_, &out_len, 0);
  if (err != ESP_OK || out_len == 0) return 0;
  const size_t stride = SOC_ADC_DIGI_RESULT_BYTES;
  size_t num_samples = out_len / stride;
  if (num_samples > max_samples) num_samples = max_samples;
  for (size_t i = 0; i < num_samples; ++i) {
    adc_digi_output_data_t *p = reinterpret_cast<adc_digi_output_data_t *>(raw_buf_ + i * stride);
    uint32_t raw = 0;
#if SOC_ADC_DIGI_RESULT_BYTES == 4
    raw = p->type2.data;
#else
    raw = p->type1.data;
#endif
    midpoint_ = (midpoint_ * 2047 + (int32_t)raw) / 2048;
    int32_t centered = (int32_t)raw - midpoint_;
    int32_t scaled = centered * 16 * (int32_t)gain_;
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;
    float x = static_cast<float>(scaled);
    float y = hpf_alpha_ * (hpf_y_prev_ + x - hpf_x_prev_);
    hpf_x_prev_ = x;
    hpf_y_prev_ = y;
    out[i] = static_cast<int16_t>(std::clamp<int>(static_cast<int>(std::lround(y)), -32768, 32767));
  }
  return num_samples;
}

}  // namespace esphome::dsp

#endif  // USE_ESP32
