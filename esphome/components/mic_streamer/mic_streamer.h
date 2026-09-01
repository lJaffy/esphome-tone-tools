#pragma once

#include "esphome/core/defines.h"

#ifdef USE_MIC_STREAMER

#include "esphome/components/microphone/microphone_source.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32
#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#endif

namespace esphome::mic_streamer {

class MicStreamer : public Component, public AsyncWebHandler {
 public:
  explicit MicStreamer(web_server_base::WebServerBase *base) : base_(base) {}

  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override;

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

  void set_microphone_source(microphone::MicrophoneSource *mic_source) { this->mic_source_ = mic_source; }
  void set_allow_without_auth(bool v) { this->allow_without_auth_ = v; }
  void set_max_duration(uint32_t ms) { this->max_duration_ms_ = ms; }

 protected:
  bool allocate_buffers_();
  void clear_buffers_();
  void deallocate_buffers_();

  void start_streaming_();
  void stop_streaming_();

#ifdef USE_ESP32
  size_t fill_chunk_idf_(uint8_t *out, size_t max_len);
#endif

  web_server_base::WebServerBase *base_{nullptr};
  microphone::MicrophoneSource *mic_source_{nullptr};

  bool allow_without_auth_{false};
  uint32_t max_duration_ms_{300000};

  // Streaming state. On ESP32 IDF the handler blocks on the httpd task,
  // so these are accessed from both that task and the main loop task.
  // Simple volatile bool is sufficient for single-client exclusive mode.
  volatile bool streaming_active_{false};
  uint32_t stream_start_ms_{0};

#ifdef USE_ESP32
  // Zero-copy ring buffer pipeline (ESP32 only, PSRAM if available).
  // Mirrors voice_assistant allocation pattern.
  std::unique_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;
#else
  // Fallback for non-ESP32 Arduino targets: lightweight queue of chunks.
  // Protected by main loop single-thread, handler fills via callback.
  std::vector<std::vector<uint8_t> > pending_chunks_;
  size_t pending_bytes_{0};
  static constexpr size_t MAX_PENDING_BYTES = 32768;
#endif
};

}  // namespace esphome::mic_streamer

#endif  // USE_MIC_STREAMER
