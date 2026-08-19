from esphome import automation
import esphome.codegen as cg
from esphome.components import microphone, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_MEASUREMENT_DURATION,
    CONF_MICROPHONE,
    CONF_WINDOW_SIZE,
    DEVICE_CLASS_FREQUENCY,
    PLATFORM_ESP32,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL,
    UNIT_HERTZ,
)

AUTO_LOAD = ["audio"]
CODEOWNERS = ["@lJaffy"]
DEPENDENCIES = ["microphone"]


CONF_PASSIVE = "passive"
CONF_MIN_FREQUENCY = "min_frequency"
CONF_MAX_FREQUENCY = "max_frequency"
CONF_THRESHOLD_DB = "threshold_db"
CONF_PEAK_MAGNITUDE = "peak_magnitude"

sound_frequency_ns = cg.esphome_ns.namespace("sound_frequency")
SoundFrequencyComponent = sound_frequency_ns.class_(
    "SoundFrequencyComponent", cg.Component
)

StartAction = sound_frequency_ns.class_("StartAction", automation.Action)
StopAction = sound_frequency_ns.class_("StopAction", automation.Action)


def _check_min_below_max(config):
    if config[CONF_MIN_FREQUENCY] >= config[CONF_MAX_FREQUENCY]:
        raise cv.Invalid(
            f"min_frequency ({config[CONF_MIN_FREQUENCY]}Hz) must be lower than "
            f"max_frequency ({config[CONF_MAX_FREQUENCY]}Hz)"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SoundFrequencyComponent),
            cv.Optional(CONF_MEASUREMENT_DURATION, default="1000ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=50),
                    max=cv.TimePeriod(seconds=60),
                ),
            ),
            cv.Optional(
                CONF_MICROPHONE, default={}
            ): microphone.microphone_source_schema(
                min_bits_per_sample=16,
                max_bits_per_sample=16,
            ),
            cv.Required(CONF_PASSIVE): cv.boolean,
            cv.Optional(CONF_WINDOW_SIZE, default=1024): cv.int_range(min=64, max=4096),
            cv.Optional(CONF_MIN_FREQUENCY, default="100Hz"): cv.frequency,
            cv.Optional(CONF_MAX_FREQUENCY, default="12000Hz"): cv.frequency,
            cv.Optional(CONF_THRESHOLD_DB, default=-50.0): cv.All(
                cv.float_, cv.Range(min=-80.0, max=0.0), cv.decibel
            ),
            cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PEAK_MAGNITUDE): sensor.sensor_schema(
                unit_of_measurement=UNIT_DECIBEL,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _check_min_below_max,
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(
        config[CONF_MICROPHONE], passive=config[CONF_PASSIVE]
    )
    cg.add(var.set_microphone_source(mic_source))

    cg.add(var.set_measurement_duration(config[CONF_MEASUREMENT_DURATION]))
    cg.add(var.set_window_size(config[CONF_WINDOW_SIZE]))
    cg.add(var.set_min_frequency_hz(config[CONF_MIN_FREQUENCY]))
    cg.add(var.set_max_frequency_hz(config[CONF_MAX_FREQUENCY]))
    cg.add(var.set_peak_threshold_db(config[CONF_THRESHOLD_DB]))

    if freq_config := config.get(CONF_FREQUENCY):
        sens = await sensor.new_sensor(freq_config)
        cg.add(var.set_frequency_sensor(sens))

    if peak_config := config.get(CONF_PEAK_MAGNITUDE):
        sens = await sensor.new_sensor(peak_config)
        cg.add(var.set_peak_magnitude_sensor(sens))

    if not config.get(CONF_FREQUENCY) and not config.get(CONF_PEAK_MAGNITUDE):
        raise cv.Invalid(
            "Component must expose at least one sensor (frequency or peak_magnitude)"
        )


SOUND_FREQUENCY_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(SoundFrequencyComponent),
    }
)


@automation.register_action(
    "sound_frequency.start",
    StartAction,
    SOUND_FREQUENCY_ACTION_SCHEMA,
    synchronous=True,
)
@automation.register_action(
    "sound_frequency.stop", StopAction, SOUND_FREQUENCY_ACTION_SCHEMA, synchronous=True
)
async def sound_frequency_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
