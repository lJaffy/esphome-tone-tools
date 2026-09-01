ARG BUILD_FROM=ghcr.io/home-assistant/base:latest
FROM ${BUILD_FROM}

ARG BUILD_VERSION
ARG BUILD_ARCH
ARG TEMPIO_VERSION=2021.09.0

RUN apk add --no-cache nginx python3 py3-flask py3-gunicorn bash jq curl \
    && rm -rf /var/cache/apk/* \
    && if [ -z "${BUILD_ARCH}" ]; then BUILD_ARCH="$(uname -m)"; fi \
    && case "${BUILD_ARCH}" in \
        amd64) tempio_arch="amd64" ;; \
        aarch64|arm64) tempio_arch="aarch64" ;; \
        *) tempio_arch="amd64" ;; \
    esac \
    && curl -sSLf -o /usr/bin/tempio \
        "https://github.com/home-assistant/tempio/releases/download/${TEMPIO_VERSION}/tempio_${tempio_arch}" \
    && chmod +x /usr/bin/tempio

COPY rootfs /
COPY www /www

RUN chmod +x /etc/services.d/*/run 2>/dev/null || true \
    && chmod +x /etc/services.d/*/finish 2>/dev/null || true \
    && chmod +x /usr/bin/tone-tools-api 2>/dev/null || true

EXPOSE 8099

LABEL \
    io.hass.type="app" \
    io.hass.name="Tone Tools Simulator" \
    io.hass.description="Interactive tuner for the ESPHome chime detector" \
    io.hass.version="${BUILD_VERSION}" \
    org.opencontainers.image.title="Tone Tools Simulator" \
    org.opencontainers.image.description="Interactive tuner for the ESPHome chime detector — same C++ engine as ESP32 firmware (WASM)" \
    org.opencontainers.image.source="https://github.com/lJaffy/esphome-tone-tools" \
    org.opencontainers.image.licenses="MIT"
