#!/usr/bin/with-contenv bashio
bashio::log.info "Tone Tools Simulator starting..."
bashio::log.info "Ingress port: 8099 (HA ingress)"
if bashio::config.exists 'log_level'; then
  bashio::log.info "Log level: $(bashio::config 'log_level')"
fi
mkdir -p /data /share/tone-tools
