#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace ds3502 {

static const uint8_t DS3502_WIPER = 0x00;  // Wiper value register
static const uint8_t DS3502_MODE = 0x02;  // Mode selection register

class DS3502Component : public Component, public output::FloatOutput, public i2c::I2CDevice {
public:
    void setup() override;
    void dump_config() override;
    void write_state(float state) override;
    void set_initial_value(float initial_value) { initial_value_ = initial_value; }

protected:
    float initial_value_;
};

}  // namespace ds3502
}  // namespace esphome
