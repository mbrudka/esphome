import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32, media_source
from esphome.const import CONF_ID
from esphome.types import ConfigType

AUTO_LOAD = ["audio"]
DEPENDENCIES = ["esp32"]

a2dp_wav_ns = cg.esphome_ns.namespace("a2dp_wav")
A2DPWavMediaSource = a2dp_wav_ns.class_(
    "A2DPWavMediaSource", cg.Component, media_source.MediaSource
)

CONFIG_SCHEMA = cv.All(
    media_source.media_source_schema(A2DPWavMediaSource).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await media_source.register_media_source(var, config)
    esp32.include_builtin_idf_component("esp_http_client")
