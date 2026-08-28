import esphome.codegen as cg
from esphome.components import microphone, web_server_base
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["audio", "ring_buffer", "web_server_base"]
DEPENDENCIES = ["microphone", "network"]

CODEOWNERS = ["lJaffy"]

mic_streamer_ns = cg.esphome_ns.namespace("mic_streamer")
MicStreamer = mic_streamer_ns.class_("MicStreamer", cg.Component)

CONF_MICROPHONE = "microphone"
CONF_ALLOW_WITHOUT_AUTH = "allow_without_auth"
CONF_MAX_DURATION = "max_duration"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MicStreamer),
            cv.GenerateID(CONF_WEB_SERVER_BASE_ID): cv.use_id(
                web_server_base.WebServerBase
            ),
            cv.Required(CONF_MICROPHONE): microphone.microphone_source_schema(
                min_bits_per_sample=16,
                max_bits_per_sample=16,
                min_channels=1,
                max_channels=1,
            ),
            cv.Optional(CONF_ALLOW_WITHOUT_AUTH, default=False): cv.boolean,
            cv.Optional(CONF_MAX_DURATION, default="300s"): cv.All(
                cv.positive_time_period,
                cv.Range(min=cv.TimePeriod(seconds=10), max=cv.TimePeriod(seconds=600)),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


FINAL_VALIDATE_SCHEMA = cv.Schema(
    {
        cv.Required(
            CONF_MICROPHONE
        ): microphone.final_validate_microphone_source_schema(
            "mic_streamer", sample_rate=16000
        ),
    },
    extra=cv.ALLOW_EXTRA,
)


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID], await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
    )
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(
        config[CONF_MICROPHONE], passive=False
    )
    cg.add(var.set_microphone_source(mic_source))

    cg.add(var.set_allow_without_auth(config[CONF_ALLOW_WITHOUT_AUTH]))
    cg.add(var.set_max_duration(int(config[CONF_MAX_DURATION].total_milliseconds)))

    cg.add_define("USE_MIC_STREAMER")
