"""Experimental full-duplex I2S hardware probe for ESP32."""

from esphome import pins
import esphome.codegen as cg
from esphome.components import i2s_audio
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@mbrudka"]
DEPENDENCIES = ["i2s_audio"]
DOMAIN = "i2s_duplex_probe"

CONF_I2S_AUDIO_ID = "i2s_audio_id"
CONF_I2S_DIN_PIN = "i2s_din_pin"
CONF_I2S_DOUT_PIN = "i2s_dout_pin"

probe_ns = cg.esphome_ns.namespace("i2s_duplex_probe")
I2SDuplexProbe = probe_ns.class_(
    "I2SDuplexProbe",
    cg.Component,
    cg.Parented.template(i2s_audio.I2SAudioComponent),
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(I2SDuplexProbe),
        cv.GenerateID(CONF_I2S_AUDIO_ID): cv.use_id(i2s_audio.I2SAudioComponent),
        cv.Required(CONF_I2S_DIN_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_I2S_AUDIO_ID])
    cg.add(var.set_din_pin(config[CONF_I2S_DIN_PIN]))
    cg.add(var.set_dout_pin(config[CONF_I2S_DOUT_PIN]))
