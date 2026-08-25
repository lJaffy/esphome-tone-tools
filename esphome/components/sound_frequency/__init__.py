"""ESPHome component: tone_sequence – detects timed tone sequences using Goertzel."""

from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, microphone
import esphome.config_validation as cv
from esphome.const import (
    CONF_DURATION,
    CONF_ID,
    CONF_MICROPHONE,
    CONF_PATTERN,
    CONF_THRESHOLD,
    CONF_WINDOW_SIZE,
    PLATFORM_ESP32,
)

AUTO_LOAD = ["audio", "binary_sensor"]
CODEOWNERS = ["@lJaffy"]
DEPENDENCIES = ["microphone"]


CONF_PASSIVE = "passive"
CONF_TICK_INTERVAL = "tick_interval"
CONF_MIN = "min"
CONF_MAX = "max"
CONF_DETECTED = "detected"
CONF_DETECTORS = "detectors"
CONF_CHORD = "chord"
CONF_TIME = "time"
CONF_TAIL_GRACE = "tail_grace"

tone_sequence_ns = cg.esphome_ns.namespace("tone_sequence")
ToneSequenceComponent = tone_sequence_ns.class_("ToneSequenceComponent", cg.Component)
StartAction = tone_sequence_ns.class_("StartAction", automation.Action)
StopAction = tone_sequence_ns.class_("StopAction", automation.Action)


# ── Duration (min / max expected span of the whole pattern) ──


def _validate_duration(value):
    min_ms = value[CONF_MIN].total_milliseconds
    max_ms = value[CONF_MAX].total_milliseconds
    if min_ms > max_ms:
        raise cv.Invalid(
            f"Duration 'min' ({min_ms}ms) must not exceed 'max' ({max_ms}ms)"
        )
    return value


DURATION_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_MIN): cv.positive_time_period_milliseconds,
            cv.Required(CONF_MAX): cv.positive_time_period_milliseconds,
        }
    ),
    _validate_duration,
)


# ── Pattern step schema (chord + time) ──

PATTERN_STEP_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CHORD): cv.All(
            cv.ensure_list(cv.frequency),
            cv.Length(min=1, max=8),
        ),
        cv.Required(CONF_TIME): cv.All(
            cv.ensure_float,
            cv.Range(min=0.0, max=3600.0),
        ),
    }
)


def _validate_pattern(steps):
    """Validate that timestamps are strictly increasing and the first is 0.0."""
    if steps[0][CONF_TIME] != 0.0:
        raise cv.Invalid("First pattern timestamp must be 0.0 (pattern start)")
    for i in range(1, len(steps)):
        if steps[i][CONF_TIME] <= steps[i - 1][CONF_TIME]:
            raise cv.Invalid(
                f"Pattern timestamps must be strictly increasing: "
                f"step {i} ({steps[i][CONF_TIME]}s) <= step {i - 1} ({steps[i - 1][CONF_TIME]}s)"
            )
    return steps


# ── Per-detector schema ──

DETECTOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PATTERN): cv.All(
            cv.ensure_list(PATTERN_STEP_SCHEMA),
            cv.Length(min=2, max=32),
            _validate_pattern,
        ),
        cv.Required(CONF_DURATION): DURATION_SCHEMA,
        cv.Optional(CONF_THRESHOLD, default=-50.0): cv.All(
            cv.float_, cv.Range(min=-80.0, max=0.0)
        ),
        cv.Optional(CONF_TAIL_GRACE, default="2s"): cv.positive_time_period_milliseconds,
        cv.Required(CONF_DETECTED): binary_sensor.binary_sensor_schema(),
    }
)


# ── Component schema ──

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ToneSequenceComponent),
            cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(
                min_bits_per_sample=16,
                max_bits_per_sample=16,
            ),
            cv.Required(CONF_PASSIVE): cv.boolean,
            cv.Optional(CONF_WINDOW_SIZE, default=1024): cv.int_range(min=64, max=4096),
            cv.Optional(CONF_TICK_INTERVAL, default="100ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=64),
                    max=cv.TimePeriod(seconds=5),
                ),
            ),
            cv.Required(CONF_DETECTORS): cv.All(
                cv.ensure_list(DETECTOR_SCHEMA),
                cv.Length(min=1, max=8),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32]),
)


# ── Code generation ──


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(
        config[CONF_MICROPHONE], passive=config[CONF_PASSIVE]
    )
    cg.add(var.set_microphone_source(mic_source))
    cg.add(var.set_window_size(config[CONF_WINDOW_SIZE]))
    cg.add(var.set_tick_interval(config[CONF_TICK_INTERVAL]))

    # Build each detector
    for _det_cfg in config[CONF_DETECTORS]:
        cg.add(var.add_detector())  # returns an int index

    for i, det_cfg in enumerate(config[CONF_DETECTORS]):
        pattern = det_cfg[CONF_PATTERN]  # list of {chord: [...], time: float}

        # Build C++ nested-vector initializer for chords: {{{440.0f, 6000.0f}}, ...}
        chord_parts = []
        for step in pattern:
            freq_strs = ", ".join(f"{float(f)}f" for f in step[CONF_CHORD])
            chord_parts.append("{" + freq_strs + "}")
        chords_cstr = "std::vector<std::vector<float>>{" + ", ".join(chord_parts) + "}"

        # Build C++ time vector (seconds → ms)
        time_parts = [f"{int(step[CONF_TIME] * 1000)}" for step in pattern]
        times_cstr = "std::vector<uint32_t>{" + ", ".join(time_parts) + "}"

        cg.add(var.detector(i).set_pattern(cg.RawExpression(chords_cstr)))
        cg.add(var.detector(i).set_pattern_times(cg.RawExpression(times_cstr)))
        cg.add(var.detector(i).set_min_duration_ms(det_cfg[CONF_DURATION][CONF_MIN]))
        cg.add(var.detector(i).set_max_duration_ms(det_cfg[CONF_DURATION][CONF_MAX]))
        cg.add(var.detector(i).set_threshold_db(det_cfg[CONF_THRESHOLD]))
        cg.add(var.detector(i).set_tail_grace_ms(det_cfg[CONF_TAIL_GRACE]))

        detected = await binary_sensor.new_binary_sensor(det_cfg[CONF_DETECTED])
        cg.add(var.detector(i).set_detected_sensor(detected))


# ── Actions ──

TONE_SEQUENCE_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(ToneSequenceComponent),
    }
)


@automation.register_action(
    "tone_sequence.start",
    StartAction,
    TONE_SEQUENCE_ACTION_SCHEMA,
    synchronous=True,
)
@automation.register_action(
    "tone_sequence.stop",
    StopAction,
    TONE_SEQUENCE_ACTION_SCHEMA,
    synchronous=True,
)
async def tone_sequence_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var