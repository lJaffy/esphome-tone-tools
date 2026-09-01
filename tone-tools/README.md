# Home Assistant App: Tone Tools Simulator

_Interactive tuner for the ESPHome `chime` custom component — runs the same `core/ChimeEngine` (WASM) as the ESP32 firmware._

![Supports aarch64 Architecture][aarch64-shield]
![Supports amd64 Architecture][amd64-shield]

App serves the existing `index.html` simulator via HA ingress (`8099` internal) with a lightweight `/api` for pattern persistence (`/data` + `/share/tone-tools`) and `mic_stream` CORS proxy.

[aarch64-shield]: https://img.shields.io/badge/aarch64-yes-green.svg
[amd64-shield]: https://img.shields.io/badge/amd64-yes-green.svg
