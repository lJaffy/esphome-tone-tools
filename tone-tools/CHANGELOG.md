# Changelog

## 1.0.0 - 2026-09-01

- Initial Home Assistant app release — converts GitHub Pages simulator (`index.html` + `js/app.js` + `wasm/chime.{js,wasm}`) to ingress app `tone-tools`.
- `ingress: true` on `8099` with `panel_icon: mdi:bell-ring`, nginx + tempio, static `/www` + `/api` (Flask) for pattern persistence and mic_stream proxy.
- Persistence to `/data/patterns.json` mirrored to `/share/tone-tools/`.
