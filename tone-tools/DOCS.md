# Home Assistant App: Tone Tools Simulator

Interactive tuner for the ESPHome `chime` detector. Same C++ engine (`core/ChimeEngine`) compiled to WASM — results match the ESP32 device.

## Installation

1. Add repository `https://github.com/lJaffy/esphome-tone-tools` to HA → Settings → Add-ons → App Store → ⋮ → Repositories.
2. Install **Tone Tools Simulator** (`tone-tools`), start it, open via **Open Web UI** (ingress) or sidebar panel *Tone Tools*.
3. Follow the 5-step workflow in the UI: upload/tune → spectrogram → pattern → simulate → export YAML.

## Usage

- **Upload** WAV/MP3/FLAC/OGG/M4A/PCM or capture live via `mic_streamer` (`GET /mic_stream` 16-bit 16 kHz mono). Gain slider emulates mic sensitivity.
- **Tune gates** until only desired tones are highlighted; **import selected segments** into pattern.
- **Simulate** with `run detection`; adjust `threshold`, `snr_margin_db`, `prominence_db`, `onset_contrast_db`, `tail_grace`, `guard_separation_hz`, `window_size`, `tick_interval`.
- **Export** `chime:` YAML, paste into ESPHome, replace `microphone: mic1`.

Patterns auto-saved to `/data/patterns.json` and mirrored to `/share/tone-tools/patterns.json`. YAML exports mirrored to `/share/tone-tools/chime.yaml` and `/config/esphome/tone-tools.yaml` if writable. API at `/api/patterns` and `/api/yaml`.

## Mic stream proxy (avoids CORS)

When running inside HA, prefer the in-app proxy: `api/mic_stream?host=<esphome-host>` — browser never contacts the device directly, so `allow_without_auth: true` is not required and mixed-content is avoided when HA is on HTTPS.

## Configuration

```yaml
log_level: info # trace|debug|info|notice|warning|error|fatal
```

## Data persistence

- `/data` — internal (survives updates)
- `/share/tone-tools/` — visible in HA file editor
- `/config` — optional auto-export

## Support

Issues: https://github.com/lJaffy/esphome-tone-tools/issues
