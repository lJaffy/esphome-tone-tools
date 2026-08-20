# ESPHome Tone Tools

A collection of ESPHome components for audio analysis, designed to be used with microphone components (ESP32, 16-bit audio). These tools use the **Goertzel algorithm** to efficiently detect specific frequencies in a digital signal without the heavy computational overhead of a full Fast Fourier Transform (FFT).

## Components

### `sound_frequency`
Detects the dominant frequency being picked up by a microphone within a specified range. Requires at least one of `frequency` or `peak_magnitude` to be configured.

**Configuration Options:**
- `measurement_duration`: (Optional, default `1000ms`) The duration of each measurement window (50ms–60s).
- `microphone`: (Optional) The ID of the microphone to measure from (16-bit audio).
- `passive`: (Required, bool) If `true`, the component never starts or stops the microphone; the microphone is expected to be controlled externally.
- `window_size`: (Optional, default `1024`) Number of samples per analysis window (64–4096).
- `min_frequency`: (Optional, default `100Hz`) The lower bound of the frequency search band.
- `max_frequency`: (Optional, default `12000Hz`) The upper bound of the frequency search band.
- `threshold_db`: (Optional, default `-50`) Only publish frequency if the peak is above this dB level (-80 to 0).
- `frequency`: (Optional) Configuration for a `sensor` component that publishes the detected frequency in Hz.
- `peak_magnitude`: (Optional) Configuration for a `sensor` component that publishes the loudness of the peak in dB.

#### Automations
- `sound_frequency.start` — starts the microphone (ignored with a warning in passive mode).
- `sound_frequency.stop` — stops the microphone (ignored with a warning in passive mode).

### `tone_sequence`
Detects specific patterns of audible tones (e.g., an alarm or a machine alert) and triggers a binary sensor. A single component can host up to eight independent **detectors**, each with its own pattern, duration window, threshold, and binary sensor. All detectors share one microphone and a single deduplicated Goertzel filter bank covering the union of every frequency used across all detector patterns.

Each pattern is a sequence of **chord steps** (2–32 steps). A chord step is a list of 1–8 frequencies and matches only when **every** frequency in the chord is simultaneously present above the detector's `threshold`. A single-frequency chord (e.g. `[2000Hz]`) behaves like a plain tone.

**Matching rules:**

- When a chord step matches, the detector waits for a **falling edge** — the previous chord must stop sounding (drop below `threshold`) before the next step can be accepted. This enforces the silence interval between tones; the minimum silence is one `tick_interval`.
- The whole pattern must complete within the `duration` window: patterns that complete in less than `duration.min` are discounted as too fast, and an attempt still incomplete when `duration.max` has elapsed is discarded.
- On a successful match, the detector's binary sensor is latched to `true` for **`duration.max` + 2s** (release time is derived automatically and cannot be configured), then switches back to `false` on its own. The pattern can restart immediately after a match, so a repeating alert keeps re-triggering while the sensor is latched and extends the release hold.

**Component Configuration Options:**
- `microphone`: (Required) The ID of the microphone to listen on (16-bit audio).
- `passive`: (Required, bool) If `true`, the component does not manage the microphone's lifecycle; the microphone must already be running.
- `window_size`: (Optional, default `1024`) Number of samples per Goertzel window (64–4096).
- `tick_interval`: (Optional, default `100ms`) How often the audio is evaluated (64ms–5s).
- `detectors`: (Required, 1–8 entries) List of pattern detectors, each configured as below.

**Per-`detector` Configuration Options:**
- `pattern`: (Required, 2–32 steps) List of chord steps. Each step is a list of 1–8 frequencies (e.g. `- [2000Hz]` or `- [440Hz, 880Hz]`).
- `duration`: (Required) The expected time for the whole pattern to play, with:
  - `min`: (Required) Minimum time the pattern must take to be considered a valid match.
  - `max`: (Required) Maximum time allowed to complete the pattern. The binary sensor's release hold is automatically `max` + 2s.
- `threshold`: (Optional, default `-50`) dBFS level a chord frequency must exceed to count as present (-80 to 0).
- `detected`: (Required) Configuration for the `binary_sensor` triggered on a successful match.

**Removed options (migration notes):**
- `tolerance` — no longer exists; each pattern frequency is matched by an exact Goertzel bin shared across all detectors.
- `dominance_db` — removed; detection is purely based on `threshold`.
- `pattern_duration` — replaced by `duration` (`min`/`max`).
- `min_match_span` — replaced by `duration.min`.
- `release_time` — no longer configurable; derived as `duration.max` + 2s.
- `guard_offset` — replaced by the built-in falling-edge requirement between chord steps.
- Flat `pattern`/`detected` keys — moved inside a `detectors` list.

#### Automations
- `tone_sequence.start` — starts the microphone (ignored with a warning in passive mode).
- `tone_sequence.stop` — stops the microphone (ignored with a warning in passive mode).

## Example Configuration

```yaml
external_components:
  - source: github://ljaffy/esphome-tone-tools@main
    components: [sound_frequency, tone_sequence]
    refresh: 0h

microphone:
  - platform: i2s
    id: i2s_mic
    # ... microphone configuration ...

tone_sequence:
  microphone: i2s_mic
  passive: true
  window_size: 512
  tick_interval: 100ms
  detectors:
    # Three short beeps: 2000 Hz, each separated by silence
    - pattern:
        - [2000Hz]
        - [2000Hz]
        - [2000Hz]
      duration:
        min: 3s
        max: 5s
      threshold: -50
      detected:
        name: "Microwave Finished"
    # Two two-tone chords (440 + 880 Hz, then 660 + 1320 Hz)
    - pattern:
        - [440Hz, 880Hz]
        - [660Hz, 1320Hz]
      duration:
        min: 1s
        max: 3s
      threshold: -55
      detected:
        name: "Alarm Bell"

sound_frequency:
  microphone: i2s_mic
  passive: true
  window_size: 1024
  min_frequency: 100Hz
  max_frequency: 8000Hz
  threshold_db: -50.0
  measurement_duration: 1s
  frequency:
    name: "Dominant Frequency"
    unit_of_measurement: "Hz"
    accuracy_decimals: 0
  peak_magnitude:
    name: "Peak Level"
    unit_of_measurement: "dB"
```
