import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_MOISTURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_MOISTURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_THERMOMETER,
    ICON_WATER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_EMPTY,
)

CODEOWNERS = ["@mrgnr"]

DEPENDENCIES = ["i2c"]

adafruit_soil_ns = cg.esphome_ns.namespace("adafruit_soil")
AdafruitSoilComponent = adafruit_soil_ns.class_(
    "AdafruitSoilComponent", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AdafruitSoilComponent),
            cv.Required(CONF_MOISTURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                icon=ICON_WATER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_MOISTURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Required(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x36))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_MOISTURE in config:
        sens = await sensor.new_sensor(config[CONF_MOISTURE])
        cg.add(var.set_moisture_sensor(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
