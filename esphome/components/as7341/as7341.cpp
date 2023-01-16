#include "as7341.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace as7341 {

// static const char *const TAG = "as7341";

void AS7341Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AS7341...");

  // this->address_ = AS7341_I2C_ADDR;
  // set_i2c_address(AS7341_I2C_ADDR);
  // ESP_LOGCONFIG(TAG, "Set address AS7341...");
  LOG_I2C_DEVICE(this);

  // Verify device ID
  uint8_t id;
  this->read_byte(AS7341_ID, &id);
  ESP_LOGCONFIG(TAG, "  Read ID: 0x%X", id);
  if ((id & 0xFC) != (AS7341_CHIP_ID << 2)) {
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "  Power on...");
  // Power on (enter IDLE state)
  if (!this->enable_power(true)) {
    ESP_LOGCONFIG(TAG, "  Power on failed!!!");
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "  Power on success");

  // Set configuration
  this->write_byte(AS7341_CONFIG, 0x00);
  uint8_t config;
  this->read_byte(AS7341_CONFIG, &config);
  ESP_LOGCONFIG(TAG, "  Config: 0x%X", config);

  ESP_LOGCONFIG(TAG, "  Setup gain atime astep");
  bool atime_success = setup_atime(_atime);
  bool astep_success = setup_astep(_astep);
  bool gain_success = setup_gain(_gain);

  ESP_LOGCONFIG(TAG, "    atime_success: %u", atime_success);
  ESP_LOGCONFIG(TAG, "    astep_success: %u", astep_success);
  ESP_LOGCONFIG(TAG, "    gain_success: %u", gain_success);
}

void AS7341Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS7341:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Gain: %u", get_gain());
  ESP_LOGCONFIG(TAG, "  ATIME: %u", get_atime());
  ESP_LOGCONFIG(TAG, "  ASTEP: %u", get_astep());
}

float AS7341Component::get_setup_priority() const { return setup_priority::DATA; }

void AS7341Component::update() {
  ESP_LOGCONFIG(TAG, "Update AS7341...");
  LOG_I2C_DEVICE(this);

  uint8_t config;
  this->read_byte(AS7341_CONFIG, &config);
  ESP_LOGCONFIG(TAG, "  Config: 0x%X", config);

  ESP_LOGCONFIG(TAG, "  Gain: %u", get_gain());
  ESP_LOGCONFIG(TAG, "  ATIME: %u", get_atime());
  ESP_LOGCONFIG(TAG, "  ASTEP: %u", get_astep());

  bool success = read_channels(_channel_readings);
  // ESP_LOGCONFIG(TAG, "  read_channels: %u", success);

  // uint16_t ch0 = read_channel(AS7341_ADC_CHANNEL_0);
  // uint16_t f2 = read_channel(AS7341_ADC_CHANNEL_1);
  // uint16_t f3 = read_channel(AS7341_ADC_CHANNEL_2);
  // uint16_t f4 = read_channel(AS7341_ADC_CHANNEL_3);
  // uint16_t ch4 = read_channel(AS7341_ADC_CHANNEL_4);
  // uint16_t ch5 = read_channel(AS7341_ADC_CHANNEL_5);

  f1->publish_state(_channel_readings[0]);
  f2->publish_state(_channel_readings[1]);
  f3->publish_state(_channel_readings[2]);
  f4->publish_state(_channel_readings[3]);
  // clear_->publish_state(ch4);
  // nir_->publish_state(ch5);
  f5->publish_state(_channel_readings[6]);
  f6->publish_state(_channel_readings[7]);
  f7->publish_state(_channel_readings[8]);
  f8->publish_state(_channel_readings[9]);
  clear->publish_state(_channel_readings[10]);
  nir->publish_state(_channel_readings[11]);
}

as7341_gain_t AS7341Component::get_gain() {
  uint8_t data;
  this->read_byte(AS7341_CFG1, &data);
  return (as7341_gain_t) data;
}

uint8_t AS7341Component::get_atime() {
  uint8_t data;
  this->read_byte(AS7341_ATIME, &data);
  return data;
}

uint16_t AS7341Component::get_astep() {
  uint16_t data;
  this->read_byte_16(AS7341_ASTEP, &data);
  return (data >> 8) | (data << 8);
}

bool AS7341Component::setup_gain(as7341_gain_t gain) {
  ESP_LOGCONFIG(TAG, "!!!! Gain: 0x%X", gain);
  return this->write_byte(AS7341_CFG1, gain);
}

bool AS7341Component::setup_atime(uint8_t atime) {
  ESP_LOGCONFIG(TAG, "!!!! ATIME: 0x%X", atime);
  return this->write_byte(AS7341_ATIME, atime);
}

bool AS7341Component::setup_astep(uint16_t astep) {
  ESP_LOGCONFIG(TAG, "!!!! ASTEP: 0x%X", astep);
  uint16_t astep_swapped = (astep >> 8) | (astep << 8);
  return this->write_byte_16(AS7341_ASTEP, astep_swapped);
}

uint16_t AS7341Component::read_channel(as7341_adc_channel_t channel) {
  enable_spectral_measurement(true);

  uint16_t data;
  this->read_byte_16(AS7341_CH0_DATA_L + 2 * channel, &data);

  // enable_spectral_measurement(false);

  return swap_bytes(data);
}

bool AS7341Component::read_channels(uint16_t *data) {
  set_smux_low_channels(true);
  enable_spectral_measurement(true);
  wait_for_data();
  bool low_success = this->read_bytes_16(AS7341_CH0_DATA_L, data, 6);

  set_smux_low_channels(false);
  enable_spectral_measurement(true);
  wait_for_data();
  bool high_sucess = this->read_bytes_16(AS7341_CH0_DATA_L, &data[6], 6);

  return low_success && high_sucess;
  // return low_success;
}

void AS7341Component::set_smux_low_channels(bool enable) {
  // ESP_LOGCONFIG(TAG, "Set SMUX low channels: %u", enable);
  enable_spectral_measurement(false);
  set_smux_command(AS7341_SMUX_CMD_WRITE);

  if (enable) {
    configure_smux_low_channels();

  } else {
    configure_smux_high_channels();
  }
  enable_smux();
}

bool AS7341Component::set_smux_command(as7341_smux_cmd_t command) {
  uint8_t data = command << 3;  // Write to bits 4:3 of the register
  ESP_LOGCONFIG(TAG, "Set MUX Command: 0x%X", data);
  return this->write_byte(AS7341_CFG6, data);
}

void AS7341Component::configure_smux_low_channels() {
  ESP_LOGCONFIG(TAG, "Configure SMUX low channels");
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  this->write_byte(0x00, 0x30);  // F3 left set to ADC2
  this->write_byte(0x01, 0x01);  // F1 left set to ADC0
  this->write_byte(0x02, 0x00);  // Reserved or disabled
  this->write_byte(0x03, 0x00);  // F8 left disabled
  this->write_byte(0x04, 0x00);  // F6 left disabled
  this->write_byte(0x05, 0x42);  // F4 left connected to ADC3/f2 left connected to ADC1
  this->write_byte(0x06, 0x00);  // F5 left disbled
  this->write_byte(0x07, 0x00);  // F7 left disbled
  this->write_byte(0x08, 0x50);  // CLEAR connected to ADC4
  this->write_byte(0x09, 0x00);  // F5 right disabled
  this->write_byte(0x0A, 0x00);  // F7 right disabled
  this->write_byte(0x0B, 0x00);  // Reserved or disabled
  this->write_byte(0x0C, 0x20);  // F2 right connected to ADC1
  this->write_byte(0x0D, 0x04);  // F4 right connected to ADC3
  this->write_byte(0x0E, 0x00);  // F6/F8 right disabled
  this->write_byte(0x0F, 0x30);  // F3 right connected to AD2
  this->write_byte(0x10, 0x01);  // F1 right connected to AD0
  this->write_byte(0x11, 0x50);  // CLEAR right connected to AD4
  this->write_byte(0x12, 0x00);  // Reserved or disabled
  this->write_byte(0x13, 0x06);  // NIR connected to ADC5
}

void AS7341Component::configure_smux_high_channels() {
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  this->write_byte(0x00, 0x00);  // F3 left disable
  this->write_byte(0x01, 0x00);  // F1 left disable
  this->write_byte(0x02, 0x00);  // reserved/disable
  this->write_byte(0x03, 0x40);  // F8 left connected to ADC3
  this->write_byte(0x04, 0x02);  // F6 left connected to ADC1
  this->write_byte(0x05, 0x00);  // F4/ F2 disabled
  this->write_byte(0x06, 0x10);  // F5 left connected to ADC0
  this->write_byte(0x07, 0x03);  // F7 left connected to ADC2
  this->write_byte(0x08, 0x50);  // CLEAR Connected to ADC4
  this->write_byte(0x09, 0x10);  // F5 right connected to ADC0
  this->write_byte(0x0A, 0x03);  // F7 right connected to ADC2
  this->write_byte(0x0B, 0x00);  // Reserved or disabled
  this->write_byte(0x0C, 0x00);  // F2 right disabled
  this->write_byte(0x0D, 0x00);  // F4 right disabled
  this->write_byte(0x0E, 0x24);  // F8 right connected to ADC2/ F6 right connected to ADC1
  this->write_byte(0x0F, 0x00);  // F3 right disabled
  this->write_byte(0x10, 0x00);  // F1 right disabled
  this->write_byte(0x11, 0x50);  // CLEAR right connected to AD4
  this->write_byte(0x12, 0x00);  // Reserved or disabled
  this->write_byte(0x13, 0x06);  // NIR connected to ADC5
}

bool AS7341Component::enable_smux() {
  ESP_LOGCONFIG(TAG, "Enable SMUX...");
  set_register_bit(AS7341_ENABLE, 4);

  uint16_t timeout = 1000;
  bool success = false;
  for (uint16_t time = 0; time < timeout; time++) {
    // The SMUXEN bit is cleared once the SMUX operation is finished
    bool smuxen = read_register_bit(AS7341_ENABLE, 4);
    if (!smuxen) {
      ESP_LOGCONFIG(TAG, "SMUX enabled!!!");
      success = true;
      break;
    }

    ESP_LOGCONFIG(TAG, "SMUX delay: %u", time);

    delay(1);
  }

  ESP_LOGCONFIG(TAG, "SMUX enabled success: %u", success);
  return success;
}

bool AS7341Component::wait_for_data() {
  // TODO
  ESP_LOGCONFIG(TAG, "Wait for data...");

  uint16_t timeout = 1000;
  bool success = false;
  for (uint16_t time = 0; time < timeout; time++) {
    success = is_data_ready();

    if (success) {
      ESP_LOGCONFIG(TAG, "Data is ready!!!");
      break;
    }

    // TODO
    ESP_LOGCONFIG(TAG, "Data delay: %u", time);

    delay(1);
  }

  // TODO
  ESP_LOGCONFIG(TAG, "Data ready: %u", success);
  return success;
}

bool AS7341Component::is_data_ready() {
  // uint8_t status;
  // this->read_byte(AS7341_STATUS, &status);
  // bool success = read_register_bit(AS7341_STATUS2, 6);
  bool success = read_register_bit(AS7341_STATUS2, 6);
  // ESP_LOGCONFIG(TAG, "Is data ready?: %u", success);
  return success;
}

bool AS7341Component::enable_power(bool enable) { return write_register_bit(AS7341_ENABLE, enable, 0); }

bool AS7341Component::enable_spectral_measurement(bool enable) { return write_register_bit(AS7341_ENABLE, enable, 1); }

bool AS7341Component::read_register_bit(uint8_t address, uint8_t bit_position) {
  uint8_t data;
  this->read_byte(address, &data);
  // ESP_LOGCONFIG(TAG, "  read_byte: 0x%X", data);
  bool bit = (data & (1 << bit_position)) > 0;
  // ESP_LOGCONFIG(TAG, "  read bit[%u]: 0x%u", bit_position, bit);
  return bit;
}

bool AS7341Component::write_register_bit(uint8_t address, bool value, uint8_t bit_position) {
  if (value) {
    return set_register_bit(address, bit_position);
  }

  return clear_register_bit(address, bit_position);
}

bool AS7341Component::set_register_bit(uint8_t address, uint8_t bit_position) {
  uint8_t data;
  this->read_byte(address, &data);
  data |= (1 << bit_position);
  return this->write_byte(address, data);
}

bool AS7341Component::clear_register_bit(uint8_t address, uint8_t bit_position) {
  uint8_t data;
  this->read_byte(address, &data);
  data &= ~(1 << bit_position);
  return this->write_byte(address, data);
}

uint16_t AS7341Component::swap_bytes(uint16_t data) { return (data >> 8) | (data << 8); }

}  // namespace as7341
}  // namespace esphome
