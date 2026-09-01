#include "mic_streamer.h"

#include "esphome/core/defines.h"

#ifdef USE_MIC_STREAMER

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#if defined(USE_ESP32) && !defined(USE_ARDUINO)
#include "esphome/components/web_server_idf/web_server_idf.h"
#include <esp_http_server.h>
#endif
#include <cinttypes>

namespace esphome::mic_streamer {

static const char *const TAG = "mic_streamer";

// 512 ms ring buffer, 32 ms send chunk — mirrors voice_assistant
static const size_t SAMPLE_RATE_HZ = 16000;
static const size_t RING_BUFFER_SAMPLES = 512 * SAMPLE_RATE_HZ / 1000;
static const size_t RING_BUFFER_SIZE = RING_BUFFER_SAMPLES * sizeof(int16_t);
static const size_t SEND_BUFFER_SAMPLES = 32 * SAMPLE_RATE_HZ / 1000;
static const size_t SEND_BUFFER_SIZE = SEND_BUFFER_SAMPLES * sizeof(int16_t);

float MicStreamer::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void MicStreamer::setup() {
  // Data callback — runs on I2S mic task thread
  this->mic_source_->add_data_callback([this](const std::vector<uint8_t> &data) {
#ifdef USE_ESP32
    std::shared_ptr<ring_buffer::RingBuffer> rb = this->ring_buffer_.lock();
    if (rb != nullptr) {
      rb->write((void *) data.data(), data.size());
    }
#else
    // Non-ESP32 fallback: queue chunks (single-consumer, main loop /
    // filler) Keep bounded to avoid OOM.
    if (this->pending_bytes_ + data.size() > MAX_PENDING_BYTES) {
      // Drop oldest chunk to make room
      if (!this->pending_chunks_.empty()) {
        this->pending_bytes_ -= this->pending_chunks_.front().size();
        this->pending_chunks_.erase(this->pending_chunks_.begin());
      }
    }
    if (this->pending_bytes_ + data.size() <= MAX_PENDING_BYTES) {
      this->pending_chunks_.push_back(data);
      this->pending_bytes_ += data.size();
    }
#endif
  });

  // Register HTTP handler
  if (this->allow_without_auth_) {
    this->base_->add_handler_without_auth(this);
  } else {
    this->base_->add_handler(this);
  }
  // Ensure HTTP server is started even if no web_server component is present.
  // WebServerBase::init() is ref-counted, safe to call multiple times.
  this->base_->init();

  ESP_LOGCONFIG(TAG,
                "Mic streamer setup complete (allow_without_auth=%s, "
                "max_duration=%" PRIu32 " ms)",
                YESNO(this->allow_without_auth_), this->max_duration_ms_);
}

void MicStreamer::dump_config() {
  ESP_LOGCONFIG(TAG, "Mic Streamer:");
  ESP_LOGCONFIG(TAG, "  Max duration: %" PRIu32 " ms", this->max_duration_ms_);
  ESP_LOGCONFIG(TAG, "  Allow without auth: %s", YESNO(this->allow_without_auth_));
}

void MicStreamer::loop() {
#ifdef USE_ESP32
  // For IDF the streaming is done blocking inside handleRequest on the httpd
  // task, so the main loop has nothing to do while streaming.
  // Keep loop free for non-IDF timeout handling.
  if (this->streaming_active_ && (millis() - this->stream_start_ms_ > this->max_duration_ms_)) {
    // Timeout reached — the IDF task will notice on next chunk and exit,
    // but set flag as fallback.
    ESP_LOGW(TAG, "Max duration reached, stopping stream (loop watchdog)");
    this->streaming_active_ = false;
  }
#else
  if (this->streaming_active_ && (millis() - this->stream_start_ms_ > this->max_duration_ms_)) {
    ESP_LOGW(TAG, "Max duration reached, stopping stream");
    this->stop_streaming_();
  }
#endif
}

bool MicStreamer::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
#if defined(USE_ESP32) && !defined(USE_ARDUINO)
  char buf[AsyncWebServerRequest::URL_BUF_SIZE];
  return request->url_to(buf) == "/mic_stream";
#else
  // Arduino String comparison
  return request->url() == "/mic_stream";
#endif
}

#ifdef USE_ESP32
bool MicStreamer::allocate_buffers_() {
  if (this->audio_source_ != nullptr)
    return true;
  std::shared_ptr<ring_buffer::RingBuffer> rb = ring_buffer::RingBuffer::create(RING_BUFFER_SIZE);
  if (rb == nullptr) {
    ESP_LOGE(TAG, "Could not allocate ring buffer (%zu bytes)", RING_BUFFER_SIZE);
    return false;
  }
  auto src = audio::RingBufferAudioSource::create(rb, SEND_BUFFER_SIZE, sizeof(int16_t));
  if (src == nullptr) {
    ESP_LOGE(TAG, "Could not allocate audio source");
    return false;
  }
  this->audio_source_ = std::move(src);
  this->ring_buffer_ = rb;
  return true;
}

void MicStreamer::clear_buffers_() {
  if (this->audio_source_ != nullptr)
    this->audio_source_->clear_buffered_data();
}

void MicStreamer::deallocate_buffers_() {
  this->audio_source_.reset();
  // ring_buffer_ weak_ptr expires automatically
}
#else
bool MicStreamer::allocate_buffers_() { return true; }
void MicStreamer::clear_buffers_() {
  this->pending_chunks_.clear();
  this->pending_bytes_ = 0;
}
void MicStreamer::deallocate_buffers_() { this->clear_buffers_(); }
#endif

void MicStreamer::start_streaming_() {
  this->mic_source_->start();
  this->stream_start_ms_ = millis();
  this->streaming_active_ = true;
  ESP_LOGD(TAG, "Microphone started for streaming");
}

void MicStreamer::stop_streaming_() {
  this->streaming_active_ = false;
  this->mic_source_->stop();
  this->clear_buffers_();
  ESP_LOGD(TAG, "Microphone stopped after streaming");
}

void MicStreamer::handleRequest(AsyncWebServerRequest *request) {
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Rejecting /mic_stream: already streaming");
    request->send(429, "text/plain", "Already streaming");
    return;
  }

  if (!this->allocate_buffers_()) {
    request->send(500, "text/plain", "Failed to allocate buffers");
    return;
  }
  this->clear_buffers_();
  this->start_streaming_();

  ESP_LOGI(TAG, "Client connected to /mic_stream");

#if defined(USE_ESP32) && !defined(USE_ARDUINO)
  // ----- IDF (esp_http_server) path: blocking chunked transfer -----
  httpd_req_t *req = *request;

  httpd_resp_set_status(req, HTTPD_200);
  httpd_resp_set_type(req, "audio/L16; rate=16000; channels=1");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Connection", "keep-alive");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
#ifdef USE_WEBSERVER_PRIVATE_NETWORK_ACCESS
  httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
#endif

  // Stream until client disconnects or max duration
  while (this->streaming_active_) {
    if (millis() - this->stream_start_ms_ > this->max_duration_ms_) {
      ESP_LOGI(TAG, "Max duration reached, ending stream");
      break;
    }

    this->audio_source_->fill(0, false);
    size_t avail = this->audio_source_->available();
    if (avail == 0) {
      // No data yet — yield briefly. 5 ms keeps latency low without busy loop.
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    esp_err_t err = httpd_resp_send_chunk(req, (const char *) this->audio_source_->data(), avail);
    if (err != ESP_OK) {
      ESP_LOGI(TAG, "Client disconnected (send err %d)", err);
      break;
    }
    this->audio_source_->consume(avail);
    // Small yield to let other tasks run; IDF httpd task is high priority.
    taskYIELD();
  }

  // Terminating chunk
  httpd_resp_send_chunk(req, nullptr, 0);
  this->stop_streaming_();
  ESP_LOGI(TAG, "Client disconnected from /mic_stream");

#else
  // ----- Arduino (ESP8266 / RP2040 / Arduino-ESP32) path: Async chunked -----
  // start flag already set; max duration check is done in filler and loop().
  request->onDisconnect([this]() { this->stop_streaming_(); });

  auto filler = [this](uint8_t *buffer, size_t max_len, size_t index) -> size_t {
    (void) index;
    if (!this->streaming_active_) {
      return 0;
    }
    if (millis() - this->stream_start_ms_ > this->max_duration_ms_) {
      ESP_LOGI(TAG, "Max duration reached (async), ending stream");
      this->stop_streaming_();
      return 0;
    }
#ifdef USE_ESP32
    // ESP32 Arduino still has ring buffer
    this->audio_source_->fill(0, false);
    size_t avail = this->audio_source_->available();
    if (avail == 0) {
      return RESPONSE_TRY_AGAIN;
    }
    size_t to_copy = std::min(avail, max_len);
    // Keep frame alignment
    if (to_copy % sizeof(int16_t) != 0) {
      to_copy -= to_copy % sizeof(int16_t);
      if (to_copy == 0)
        return RESPONSE_TRY_AGAIN;
    }
    memcpy(buffer, this->audio_source_->data(), to_copy);
    this->audio_source_->consume(to_copy);
    return to_copy;
#else
    if (this->pending_chunks_.empty()) {
      return RESPONSE_TRY_AGAIN;
    }
    size_t copied = 0;
    while (copied < max_len && !this->pending_chunks_.empty()) {
      auto &front = this->pending_chunks_.front();
      size_t to_copy = std::min(front.size(), max_len - copied);
      memcpy(buffer + copied, front.data(), to_copy);
      copied += to_copy;
      if (to_copy == front.size()) {
        this->pending_bytes_ -= front.size();
        this->pending_chunks_.erase(this->pending_chunks_.begin());
      } else {
        // Partial consume
        front.erase(front.begin(), front.begin() + to_copy);
        this->pending_bytes_ -= to_copy;
        break;
      }
    }
    if (copied == 0)
      return RESPONSE_TRY_AGAIN;
    return copied;
#endif
  };

  // sendChunked will keep calling filler until it returns 0.
  // Content-Type raw PCM; browsers use fetch() + AudioContext.
  AsyncWebServerResponse *response = request->beginChunkedResponse("audio/L16; rate=16000; channels=1", filler);
  // Add CORS if not already via DefaultHeaders — AsyncWebServer handles
  // separately, but we ensure no-cache.
  response->addHeader("Cache-Control", "no-cache");
  response->addHeader("Connection", "keep-alive");
  request->send(response);
  // Note: streaming continues via filler callbacks; stop is via max_duration or
  // onDisconnect.
#endif
}

}  // namespace esphome::mic_streamer

#endif  // USE_MIC_STREAMER
