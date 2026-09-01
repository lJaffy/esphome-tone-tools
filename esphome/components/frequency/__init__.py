"""ESPHome component: frequency – dominant frequency in a band (mic or ADC)."""

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import microphone, sensor

try:
    from esphome.components.adc import validate_adc_pin
except ImportError:
    validate_adc_pin = None  # type: ignore[assignment]
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

AUTO_LOAD = ["audio", "sensor"]
CODEOWNERS = ["@lJaffy"]
DEPENDENCIES = []

CONF_PASSIVE = "passive"
CONF_MIN_FREQUENCY = "min_frequency"
CONF_MAX_FREQUENCY = "max_frequency"
CONF_THRESHOLD_DB = "threshold_db"
CONF_PEAK_MAGNITUDE = "peak_magnitude"

# ADC source (mirrors chime)
CONF_ADC = "adc"
CONF_ADC_PIN = "pin"
CONF_ADC_ATTENUATION = "attenuation"
CONF_SAMPLE_RATE = "sample_rate"
CONF_ADC_GAIN = "adc_gain"

frequency_ns = cg.esphome_ns.namespace("frequency")
FrequencyComponent = frequency_ns.class_("FrequencyComponent", cg.Component)
StartAction = frequency_ns.class_("StartAction", automation.Action)
StopAction = frequency_ns.class_("StopAction", automation.Action)

if validate_adc_pin is not None:
    _ADC_PIN_VALIDATOR = validate_adc_pin  # type: ignore[assignment]
else:  # pragma: no cover – fallback
    try:
        _ADC_PIN_VALIDATOR = pins.internal_gpio_input_pin_number  # type: ignore[attr-defined]
    except AttributeError:
        import esphome.config_validation as _cv_fallback  # type: ignore[no-redef]

        _ADC_PIN_VALIDATOR = _cv_fallback.int_range(min=0, max=48)  # type: ignore[assignment]

ADC_ATTENUATIONS = {
    "0db": "ADC_ATTEN_DB_0",
    "2.5db": "ADC_ATTEN_DB_2_5",
    "6db": "ADC_ATTEN_DB_6",
    "11db": "ADC_ATTEN_DB_11",
    "12db": "ADC_ATTEN_DB_12",
}

ADC_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ADC_PIN): _ADC_PIN_VALIDATOR,
        cv.Optional(CONF_ADC_ATTENUATION, default="11db"): cv.one_of(
            *ADC_ATTENUATIONS, lower=True
        ),
        cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.All(
            cv.int_, cv.Range(min=4000, max=48000)
        ),
        cv.Optional(CONF_ADC_GAIN, default=1): cv.int_range(min=1, max=64),
    }
)


def _check_min_below_max(config):
    if config[CONF_MIN_FREQUENCY] >= config[CONF_MAX_FREQUENCY]:
        raise cv.Invalid(
            f"min_frequency ({config[CONF_MIN_FREQUENCY]}Hz) must be lower than "
            f"max_frequency ({config[CONF_MAX_FREQUENCY]}Hz)"
        )
    return config


def _validate_source(config):
    has_mic = CONF_MICROPHONE in config
    has_adc = CONF_ADC in config
    if has_mic == has_adc:
        raise cv.Invalid(
            "Specify exactly one of 'microphone' or 'adc' (exclusive source). "
            "Use 'microphone:' for acoustic I2S/PDM mics or 'adc:' with a pin for "
            "mechanically-coupled pipe/piezo sensors."
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FrequencyComponent),
            cv.Optional(CONF_MICROPHONE): microphone.microphone_source_schema(
                min_bits_per_sample=16,
                max_bits_per_sample=16,
            ),
            cv.Optional(CONF_ADC): ADC_SCHEMA,
            cv.Required(CONF_PASSIVE): cv.boolean,
            cv.Optional(CONF_WINDOW_SIZE, default=1024): cv.int_range(min=64, max=4096),
            cv.Optional(CONF_MIN_FREQUENCY, default="100Hz"): cv.frequency,
            cv.Optional(CONF_MAX_FREQUENCY, default="12000Hz"): cv.frequency,
            cv.Optional(CONF_MEASUREMENT_DURATION, default="1000ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=50),
                    max=cv.TimePeriod(seconds=60),
                ),
            ),
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
    _validate_source,
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_MICROPHONE in config:
        mic_source = await microphone.microphone_source_to_code(
            config[CONF_MICROPHONE], passive=config[CONF_PASSIVE]
        )
        cg.add(var.set_microphone_source(mic_source))
    else:
        adc_cfg = config[CONF_ADC]
        pin = adc_cfg[CONF_ADC_PIN]
        try:
            pin_num = pin.number  # type: ignore[union-attr]
        except AttributeError:
            pin_num = int(pin)
        atten_cpp = ADC_ATTENUATIONS[adc_cfg[CONF_ADC_ATTENUATION].lower()]
        cg.add(var.set_adc_pin(pin_num))
        cg.add(var.set_adc_attenuation(cg.RawExpression(atten_cpp)))
        cg.add(var.set_adc_sample_rate(adc_cfg[CONF_SAMPLE_RATE]))
        cg.add(var.set_adc_gain(adc_cfg[CONF_ADC_GAIN]))
    cg.add(var.set_passive(config[CONF_PASSIVE]))

    cg.add(var.set_window_size(config[CONF_WINDOW_SIZE]))
    cg.add(var.set_min_frequency_hz(config[CONF_MIN_FREQUENCY]))
    cg.add(var.set_max_frequency_hz(config[CONF_MAX_FREQUENCY]))
    cg.add(var.set_measurement_duration(config[CONF_MEASUREMENT_DURATION]))
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


FREQUENCY_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(FrequencyComponent),
    }
)


@automation.register_action(
    "frequency.start",
    StartAction,
    FREQUENCY_ACTION_SCHEMA,
    synchronous=True,
)
@automation.register_action(
    "frequency.stop", StopAction, FREQUENCY_ACTION_SCHEMA, synchronous=True
)
async def frequency_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
