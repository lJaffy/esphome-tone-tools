# ESPHome Tone Tools

A collection of ESPHome components for audio analysis, designed to be used with microphone components. These tools use the **Goertzel algorithm** to efficiently detect specific frequencies in a digital signal without the heavy computational overhead of a full Fast Fourier Transform (FFT).

## Components

### `sound_frequency`
Detects the dominant frequency being picked up by a microphone within a specified range.

**Configuration Options:**
- `id`: (Optional) Component ID.
- `passive`: (bool) If `true`, the component does not require a microphone configuration.
- `window_size`: Number of samples per measurement window.
- `min_frequency`: The lower bound of the frequency search band (e.g., `100Hz`).
- `max_frequency`: The upper bound of the frequency search band (e.g., `8000Hz`).
- `threshold_db`: Only publish frequency if the peak is above this dB level.
- `measurement_duration`: The duration of the measurement window.
- `frequency`: Configuration for the `sensor` component that publishes the detected frequency.
- `peak_magnitude`: (Optional) Configuration for a `sensor` component that publishes the loudness of the peak in dB.

### `tone_sequence`
Listens for a specific pattern of audible tones (e.g., an alarm or a machine alert) and triggers a binary sensor.

**Configuration Options:**
- `microphone`: The ID of the microphone to listen to.
- `passive`: (bool) If `true`, the component does not require a microphone configuration.
- `window_size`: Number of samples per measurement window.
- `tick_interval`: The interval between processing windows.
- `pattern`: A list of frequencies defining the sequence (e.g., `- 2000Hz`).
- `pattern_duration`: Total time allowed to complete the sequence.
- `tolerance`: Frequency deviation allowed (e.g., `50Hz`).
- `threshold`: dB threshold for detection.
- `release_time`: Time to wait after detection before resetting.
- `dominance_db`: The required difference between the target frequency and background noise.
- `guard_offset`: Silence interval between tones.
- `min_match_span`: Minimum time the pattern must be matched to trigger.
- `detected`: Configuration for the `binary_sensor` component triggered upon a successful match.

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
  window_size: 512
  tick_interval: 100ms
  pattern:
    - 2000Hz
    - 2000Hz
    - 2000Hz
  pattern_duration: 5s
  tolerance: 50Hz
  threshold: -50
  release_time: 5s
  dominance_db: 6     
  guard_offset: 150
  min_match_span: 3s 
  detected:
    name: "Microwave Finished"

sound_frequency:
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
