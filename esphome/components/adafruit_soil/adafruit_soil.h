#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace adafruit_soil {

using esphome::i2c::ErrorCode;
using esphome::i2c::ERROR_OK;

static const uint8_t ADAFRUIT_SOIL_MOISTURE_BASE = 0x0F;
static const uint8_t ADAFRUIT_SOIL_MOISTURE_OFFSET = 0x10;
static const uint8_t ADAFRUIT_SOIL_STATUS_BASE = 0x00;
static const uint8_t ADAFRUIT_SOIL_TEMP_OFFSET = 0x04;

class AdafruitSoilComponent : public PollingComponent, public i2c::I2CDevice {
  public:
    void setup() override;
    void dump_config() override;
    float get_setup_priority() const override;
    void update() override;

    void set_moisture_sensor(sensor::Sensor *moisture_sensor) { moisture_sensor_ = moisture_sensor; }
    void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

    esphome::i2c::ErrorCode read_register16(uint16_t a_register, uint8_t *data, size_t len, bool stop = true);


  protected:
    sensor::Sensor *moisture_sensor_{nullptr};
    sensor::Sensor *temperature_sensor_{nullptr};
};

}  // namespace adafruit_soil
}  // namespace esphome
