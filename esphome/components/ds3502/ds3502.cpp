#include "ds3502.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ds3502 {

static const char *const TAG = "ds3502";

void DS3502Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DS3502...");
  this->write_byte(DS3502_MODE, 0x00);
  this->write_state(this->initial_value_);

  delay(100); // delay to allow EEPROM write to IVR to finish
  this->write_byte(DS3502_MODE, 0x80);
}

void DS3502Component::dump_config() {
  ESP_LOGCONFIG(TAG, "DS3502:");
  LOG_I2C_DEVICE(this);
}

void DS3502Component::write_state(float state) {
  const uint8_t value = remap(state, 0.0f, 1.0f, 0, 127);
  ESP_LOGCONFIG(TAG, "  write value: %u", value);
  this->write_byte(DS3502_WIPER, value);

  uint8_t data;
  this->read_byte(DS3502_WIPER, &data);
  ESP_LOGCONFIG(TAG, "  read value: %u", value);

}

}  // namespace ds3502
}  // namespace esphome
