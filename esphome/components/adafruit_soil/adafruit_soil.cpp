#include "adafruit_soil.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace adafruit_soil {

static const char *const TAG = "adafruit_soil";

void AdafruitSoilComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Adafruit Soil Sensor...");
  LOG_I2C_DEVICE(this);
}

void AdafruitSoilComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Adafruit Soil Sensor:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with Adafruit Soil Sensor failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Moisture", this->moisture_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

float AdafruitSoilComponent::get_setup_priority() const { return setup_priority::DATA; }

void AdafruitSoilComponent::update() {
    uint8_t moisture[2];
    uint16_t maddr = (((uint16_t)ADAFRUIT_SOIL_MOISTURE_BASE << 8) | ADAFRUIT_SOIL_MOISTURE_OFFSET);
    ESP_LOGE("moisture addr: ", "%x", maddr);
    this->read_register16(maddr, moisture, 2);

    ESP_LOGE("moisture[0]: ", "%u", moisture[0]);
    ESP_LOGE("moisture[1]: ", "%u", moisture[1]);

    float moisture_ = ((uint16_t)moisture[0] << 8) | moisture[1];


    uint8_t temperature[4];
    uint16_t taddr = ((uint16_t)ADAFRUIT_SOIL_STATUS_BASE << 8) | ADAFRUIT_SOIL_TEMP_OFFSET;
    ESP_LOGE("temperature addr: ", "%x", taddr);
    this->read_register16(taddr, temperature, 4);

    uint32_t temperature_ = ((uint32_t)temperature[0] << 24) | ((uint32_t)temperature[1] << 16) | ((uint32_t)temperature[2] << 8) | (uint32_t)temperature[3];
    float temperature__ = (1.0 / (1UL << 16)) * temperature_;

    if (this->moisture_sensor_ != nullptr) {
      this->moisture_sensor_->publish_state(moisture_);
    }
    if (this->temperature_sensor_ != nullptr) {
      this->temperature_sensor_->publish_state(temperature__);
    }
}

ErrorCode AdafruitSoilComponent::read_register16(uint16_t a_register, uint8_t *data, size_t len, bool stop) {
  a_register = convert_big_endian(a_register);
  ErrorCode const err = this->write(reinterpret_cast<const uint8_t *>(&a_register), 2, stop);
  delay(10);
  if (err != ERROR_OK)
    return err;
  return bus_->read(address_, data, len);
}

}  // namespace adafruit_soil
}  // namespace esphome
