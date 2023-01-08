import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c

CODEOWNERS = ["@mrgnr"]
DEPENDENCIES = ["i2c"]

as7341_ns = cg.esphome_ns.namespace("as7341")

AS7341Component = as7341_ns.class_("AS7341Cls", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS7341Component),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x39))
)
