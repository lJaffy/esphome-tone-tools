"""ESPHome component: tone_sequence – detects a sequence of tones using Goertzel."""

from esphome import automation
import esphome.codegen as cg
from esphome.components import binary_sensor, microphone
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MICROPHONE,
    CONF_PATTERN,
    CONF_THRESHOLD,
    CONF_TOLERANCE,
    CONF_WINDOW_SIZE,
    PLATFORM_ESP32,
)

AUTO_LOAD = ["audio", "binary_sensor"]
CODEOWNERS = ["@lJaffy"]
DEPENDENCIES = ["microphone"]


CONF_PASSIVE = "passive"
CONF_TICK_INTERVAL = "tick_interval"
CONF_PATTERN_DURATION = "pattern_duration"
CONF_DETECTED = "detected"
CONF_RELEASE_TIME = "release_time"
CONF_DOMINANCE_DB = "dominance_db"
CONF_GUARD_OFFSET = "guard_offset"
CONF_MIN_MATCH_SPAN = "min_match_span"
CONF_DETECTORS = "detectors"

tone_sequence_ns = cg.esphome_ns.namespace("tone_sequence")
ToneSequenceComponent = tone_sequence_ns.class_("ToneSequenceComponent", cg.Component)
StartAction = tone_sequence_ns.class_("StartAction", automation.Action)
StopAction = tone_sequence_ns.class_("StopAction", automation.Action)


# ── Per-detector schema ──

DETECTOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PATTERN): cv.All(
            cv.ensure_list(cv.frequency),
            cv.Length(min=2, max=32),
        ),
        cv.Optional(CONF_PATTERN_DURATION, default="2s"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(
                min=cv.TimePeriod(milliseconds=200),
                max=cv.TimePeriod(seconds=60),
            ),
        ),
        cv.Optional(CONF_TOLERANCE, default="50Hz"): cv.All(
            cv.frequency,
            cv.Range(min=5, max=500),
        ),
        cv.Optional(CONF_THRESHOLD, default=-50.0): cv.All(
            cv.float_, cv.Range(min=-80.0, max=0.0)
        ),
        cv.Optional(CONF_DOMINANCE_DB, default=6.0): cv.All(
            cv.positive_float, cv.Range(min=0.0, max=30.0)
        ),
        cv.Optional(CONF_GUARD_OFFSET, default=150): cv.All(
            cv.positive_int, cv.Range(min=20, max=500)
        ),
        cv.Optional(CONF_MIN_MATCH_SPAN): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(
                min=cv.TimePeriod(milliseconds=100),
                max=cv.TimePeriod(seconds=60),
            ),
        ),
        cv.Required(CONF_DETECTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_RELEASE_TIME, default="3s"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(
                min=cv.TimePeriod(milliseconds=500),
                max=cv.TimePeriod(minutes=5),
            ),
        ),
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
        tones = det_cfg[CONF_PATTERN]
        tones_init = ", ".join(f"{float(t)}f" for t in tones)

        # Pattern tones
        cg.add(
            var.detector(i).set_pattern(
                cg.RawExpression(f"std::vector<float>{{{tones_init}}}")
            )
        )
        cg.add(var.detector(i).set_pattern_duration(det_cfg[CONF_PATTERN_DURATION]))
        cg.add(var.detector(i).set_tolerance_hz(det_cfg[CONF_TOLERANCE]))
        cg.add(var.detector(i).set_threshold_db(det_cfg[CONF_THRESHOLD]))
        cg.add(var.detector(i).set_dominance_db(det_cfg[CONF_DOMINANCE_DB]))
        cg.add(var.detector(i).set_guard_offset_hz(det_cfg[CONF_GUARD_OFFSET]))

        # min_match_span defaults to 75 % of pattern_duration if not set
        if det_cfg.get(CONF_MIN_MATCH_SPAN) is not None:
            cg.add(var.detector(i).set_min_match_span_ms(det_cfg[CONF_MIN_MATCH_SPAN]))
        else:
            pgm = f"static_cast<uint32_t>({det_cfg[CONF_PATTERN_DURATION].total_milliseconds} * 0.75f)"
            cg.add(var.detector(i).set_min_match_span_ms(cg.RawExpression(pgm)))

        detected = await binary_sensor.new_binary_sensor(det_cfg[CONF_DETECTED])
        cg.add(var.detector(i).set_detected_sensor(detected))
        cg.add(var.detector(i).set_release_time(det_cfg[CONF_RELEASE_TIME]))


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
