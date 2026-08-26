"""ESPHome component: chime – detects timed tone sequences using Goertzel."""

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
    CONF_TIME,
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
CONF_CHIMES = "chimes"
CONF_CHORD = "chord"
CONF_TAIL_GRACE = "tail_grace"
CONF_SNR_MARGIN_DB = "snr_margin_db"
CONF_PROMINENCE_DB = "prominence_db"
CONF_COHERENCE = "coherence"
CONF_GUARD_SEPARATION_HZ = "guard_separation_hz"
CONF_NOISE_FLOOR_ALPHA_DOWN = "noise_floor_alpha_down"
CONF_NOISE_FLOOR_ALPHA_UP = "noise_floor_alpha_up"

chime_ns = cg.esphome_ns.namespace("chime")
ChimeComponent = chime_ns.class_("ChimeComponent", cg.Component)
StartAction = chime_ns.class_("StartAction", automation.Action)
StopAction = chime_ns.class_("StopAction", automation.Action)


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


# ── Pattern step schema (chord + optional time) ──

PATTERN_STEP_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CHORD): cv.All(
            cv.ensure_list(cv.frequency),
            cv.Length(min=1, max=8),
        ),
        cv.Optional(CONF_TIME): cv.All(
            cv.positive_float,
            cv.Range(min=0.0, max=3600.0),
        ),
    }
)


def _validate_pattern(steps):
    """If any times are provided, they must be strictly increasing among themselves."""
    timed = [
        (i, s[CONF_TIME]) for i, s in enumerate(steps) if s.get(CONF_TIME) is not None
    ]
    if len(timed) < 2:
        return steps
    for j in range(1, len(timed)):
        if timed[j][1] <= timed[j - 1][1]:
            raise cv.Invalid(
                f"Pattern timestamps must be strictly increasing: "
                f"step {timed[j][0]} ({timed[j][1]}s) <= step {timed[j - 1][0]} ({timed[j - 1][1]}s)"
            )
    return steps


# ── Per-chime schema: IS a binary sensor, extended with pattern fields ──

CHIME_SCHEMA = binary_sensor.binary_sensor_schema().extend(
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
        cv.Optional(CONF_SNR_MARGIN_DB, default=8.0): cv.All(
            cv.positive_float, cv.Range(min=0.0, max=60.0)
        ),
        cv.Optional(CONF_PROMINENCE_DB, default=6.0): cv.All(
            cv.float_, cv.Range(min=0.0, max=60.0)
        ),
        cv.Optional(CONF_COHERENCE, default=True): cv.boolean,
        cv.Optional(
            CONF_TAIL_GRACE, default="2s"
        ): cv.positive_time_period_milliseconds,
    }
)


# ── Component schema ──

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ChimeComponent),
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
            cv.Optional(CONF_GUARD_SEPARATION_HZ, default=150.0): cv.All(
                cv.float_, cv.Range(min=10.0, max=1000.0)
            ),
            cv.Optional(CONF_NOISE_FLOOR_ALPHA_DOWN, default=0.05): cv.All(
                cv.positive_float, cv.Range(min=0.0001, max=1.0)
            ),
            cv.Optional(CONF_NOISE_FLOOR_ALPHA_UP, default=0.005): cv.All(
                cv.positive_float, cv.Range(min=0.0001, max=1.0)
            ),
            cv.Required(CONF_CHIMES): cv.All(
                cv.ensure_list(CHIME_SCHEMA),
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
    cg.add(var.set_guard_separation_hz(config[CONF_GUARD_SEPARATION_HZ]))
    cg.add(var.set_noise_floor_alpha_down(config[CONF_NOISE_FLOOR_ALPHA_DOWN]))
    cg.add(var.set_noise_floor_alpha_up(config[CONF_NOISE_FLOOR_ALPHA_UP]))

    for _chime_cfg in config[CONF_CHIMES]:
        cg.add(var.add_chime())

    for i, chime_cfg in enumerate(config[CONF_CHIMES]):
        pattern = chime_cfg[CONF_PATTERN]

        chord_parts = []
        for step in pattern:
            freq_strs = ", ".join(f"{float(f)}f" for f in step[CONF_CHORD])
            chord_parts.append("{" + freq_strs + "}")
        chords_cstr = "std::vector<std::vector<float>>{" + ", ".join(chord_parts) + "}"

        time_parts = []
        for step in pattern:
            t = step.get(CONF_TIME)
            if t is None:
                time_parts.append("0xFFFFFFFF")
            else:
                time_parts.append(f"{int(t * 1000)}")
        times_cstr = "std::vector<uint32_t>{" + ", ".join(time_parts) + "}"

        cg.add(var.chime(i).set_pattern(cg.RawExpression(chords_cstr)))
        cg.add(var.chime(i).set_pattern_times(cg.RawExpression(times_cstr)))
        cg.add(var.chime(i).set_min_duration_ms(chime_cfg[CONF_DURATION][CONF_MIN]))
        cg.add(var.chime(i).set_max_duration_ms(chime_cfg[CONF_DURATION][CONF_MAX]))
        cg.add(var.chime(i).set_threshold_db(chime_cfg[CONF_THRESHOLD]))
        cg.add(var.chime(i).set_snr_margin_db(chime_cfg[CONF_SNR_MARGIN_DB]))
        cg.add(var.chime(i).set_prominence_db(chime_cfg[CONF_PROMINENCE_DB]))
        cg.add(var.chime(i).set_coherence(chime_cfg[CONF_COHERENCE]))
        cg.add(var.chime(i).set_tail_grace_ms(chime_cfg[CONF_TAIL_GRACE]))

        detected = await binary_sensor.new_binary_sensor(chime_cfg)
        cg.add(var.chime(i).set_detected_sensor(detected))


# ── Actions ──

CHIME_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(ChimeComponent),
    }
)


@automation.register_action(
    "chime.start",
    StartAction,
    CHIME_ACTION_SCHEMA,
    synchronous=True,
)
@automation.register_action(
    "chime.stop",
    StopAction,
    CHIME_ACTION_SCHEMA,
    synchronous=True,
)
async def chime_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
