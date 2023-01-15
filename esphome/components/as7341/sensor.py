import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    ICON_BRIGHTNESS_5,
    STATE_CLASS_MEASUREMENT,
)


CODEOWNERS = ["@mrgnr"]
DEPENDENCIES = ["i2c"]

as7341_ns = cg.esphome_ns.namespace("as7341")

AS7341Component = as7341_ns.class_(
    "AS7341Component", cg.PollingComponent, i2c.I2CDevice
)

CONF_F1 = "f1"
CONF_F2 = "f2"
CONF_F3 = "f3"
CONF_F4 = "f4"
CONF_F5 = "f5"
CONF_F6 = "f6"
CONF_F7 = "f7"
CONF_F8 = "f8"
CONF_CLEAR = "clear"
CONF_NIR = "nir"

UNIT_COUNTS = "#"


def sensor_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_COUNTS,
        icon=ICON_BRIGHTNESS_5,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_ILLUMINANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    )


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AS7341Component),
            cv.Optional(CONF_F1): sensor_schema(),
            cv.Optional(CONF_F2): sensor_schema(),
            cv.Optional(CONF_F3): sensor_schema(),
            cv.Optional(CONF_F4): sensor_schema(),
            cv.Optional(CONF_F5): sensor_schema(),
            cv.Optional(CONF_F6): sensor_schema(),
            cv.Optional(CONF_F7): sensor_schema(),
            cv.Optional(CONF_F8): sensor_schema(),
            cv.Optional(CONF_CLEAR): sensor_schema(),
            cv.Optional(CONF_NIR): sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x39))
)

SENSORS = {
    CONF_F1: "set_f1_sensor",
    CONF_F2: "set_f2_sensor",
    CONF_F3: "set_f3_sensor",
    CONF_F4: "set_f4_sensor",
    CONF_F5: "set_f5_sensor",
    CONF_F6: "set_f6_sensor",
    CONF_F7: "set_f7_sensor",
    CONF_F8: "set_f8_sensor",
    CONF_CLEAR: "set_clear_sensor",
    CONF_NIR: "set_nir_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for conf_id, set_sensor_func in SENSORS.items():
        sens = await sensor.new_sensor(config[conf_id])
        cg.add(getattr(var, set_sensor_func)(sens))
