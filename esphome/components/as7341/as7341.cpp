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
  if (!this->enablePower(true)) {
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
  bool atime_success = setupATIME(_atime);
  bool astep_success = setupASTEP(_astep);
  bool gain_success = setupGain(_gain);

  ESP_LOGCONFIG(TAG, "    atime_success: %u", atime_success);
  ESP_LOGCONFIG(TAG, "    astep_success: %u", astep_success);
  ESP_LOGCONFIG(TAG, "    gain_success: %u", gain_success);
}

void AS7341Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS7341:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Gain: %u", getGain());
  ESP_LOGCONFIG(TAG, "  ATIME: %u", getATIME());
  ESP_LOGCONFIG(TAG, "  ASTEP: %u", getASTEP());
}

float AS7341Component::get_setup_priority() const { return setup_priority::DATA; }

void AS7341Component::update() {
  ESP_LOGCONFIG(TAG, "Update AS7341...");
  LOG_I2C_DEVICE(this);

  uint8_t config;
  this->read_byte(AS7341_CONFIG, &config);
  ESP_LOGCONFIG(TAG, "  Config: 0x%X", config);

  ESP_LOGCONFIG(TAG, "  Gain: %u", getGain());
  ESP_LOGCONFIG(TAG, "  ATIME: %u", getATIME());
  ESP_LOGCONFIG(TAG, "  ASTEP: %u", getASTEP());

  bool success = readChannels(_channel_readings);
  // ESP_LOGCONFIG(TAG, "  readChannels: %u", success);

  // uint16_t ch0 = readChannel(AS7341_ADC_CHANNEL_0);
  // uint16_t f2 = readChannel(AS7341_ADC_CHANNEL_1);
  // uint16_t f3 = readChannel(AS7341_ADC_CHANNEL_2);
  // uint16_t f4 = readChannel(AS7341_ADC_CHANNEL_3);
  // uint16_t ch4 = readChannel(AS7341_ADC_CHANNEL_4);
  // uint16_t ch5 = readChannel(AS7341_ADC_CHANNEL_5);

  uint16_t f1_ = _channel_readings[0];
  uint16_t f2_ = _channel_readings[1];
  uint16_t f3_ = _channel_readings[2];
  uint16_t f4_ = _channel_readings[3];
  // uint16_t ch4 = _channel_readings[4];
  // uint16_t ch5 = _channel_readings[5];
  uint16_t f5_ = _channel_readings[6];
  uint16_t f6_ = _channel_readings[7];
  uint16_t f7_ = _channel_readings[8];
  uint16_t f8_ = _channel_readings[9];
  uint16_t clear_ = _channel_readings[10];
  uint16_t nir_ = _channel_readings[11];

  f1->publish_state(f1_);
  f2->publish_state(f2_);
  f3->publish_state(f3_);
  f4->publish_state(f4_);
  // clear_->publish_state(ch4);
  // nir_->publish_state(ch5);
  f5->publish_state(f5_);
  f6->publish_state(f6_);
  f7->publish_state(f7_);
  f8->publish_state(f8_);
  clear->publish_state(clear_);
  nir->publish_state(nir_);
}

as7341_gain_t AS7341Component::getGain() {
  uint8_t data;
  this->read_byte(AS7341_CFG1, &data);
  return (as7341_gain_t) data;
}

uint8_t AS7341Component::getATIME() {
  uint8_t data;
  this->read_byte(AS7341_ATIME, &data);
  return data;
}

uint16_t AS7341Component::getASTEP() {
  uint16_t data;
  this->read_byte_16(AS7341_ASTEP, &data);
  return (data >> 8) | (data << 8);
}

bool AS7341Component::setupGain(as7341_gain_t gain) {
  ESP_LOGCONFIG(TAG, "!!!! Gain: 0x%X", gain);
  return this->write_byte(AS7341_CFG1, gain);
}

bool AS7341Component::setupATIME(uint8_t atime) {
  ESP_LOGCONFIG(TAG, "!!!! ATIME: 0x%X", atime);
  return this->write_byte(AS7341_ATIME, atime);
}

bool AS7341Component::setupASTEP(uint16_t astep) {
  ESP_LOGCONFIG(TAG, "!!!! ASTEP: 0x%X", astep);
  uint16_t astep_swapped = (astep >> 8) | (astep << 8);
  return this->write_byte_16(AS7341_ASTEP, astep_swapped);
}

uint16_t AS7341Component::readChannel(as7341_adc_channel_t channel) {
  enableSpectralMeasurement(true);

  uint16_t data;
  this->read_byte_16(AS7341_CH0_DATA_L + 2 * channel, &data);

  // enableSpectralMeasurement(false);

  return swapBytes(data);
}

bool AS7341Component::readChannels(uint16_t *data) {
  setSMUXLowChannels(true);
  enableSpectralMeasurement(true);
  waitForData();
  bool low_success = this->read_bytes_16(AS7341_CH0_DATA_L, data, 6);

  setSMUXLowChannels(false);
  enableSpectralMeasurement(true);
  waitForData();
  bool high_sucess = this->read_bytes_16(AS7341_CH0_DATA_L, &data[6], 6);

  return low_success && high_sucess;
  // return low_success;
}

void AS7341Component::setSMUXLowChannels(bool enable) {
  // ESP_LOGCONFIG(TAG, "Set SMUX low channels: %u", enable);
  enableSpectralMeasurement(false);
  setSMUXCommand(AS7341_SMUX_CMD_WRITE);

  if (enable) {
    configureSMUXLowChannels();

  } else {
    configureSMUXHighChannels();
  }
  enableSmux();
}

bool AS7341Component::setSMUXCommand(as7341_smux_cmd_t command) {
  uint8_t data = command << 3;  // Write to bits 4:3 of the register
  ESP_LOGCONFIG(TAG, "Set MUX Command: 0x%X", data);
  return this->write_byte(AS7341_CFG6, data);
}

void AS7341Component::configureSMUXLowChannels() {
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

void AS7341Component::configureSMUXHighChannels() {
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

bool AS7341Component::enableSmux() {
  ESP_LOGCONFIG(TAG, "Enable SMUX...");
  setRegisterBit(AS7341_ENABLE, 4);

  uint16_t timeout = 1000;
  bool success = false;
  for (uint16_t time = 0; time < timeout; time++) {
    // The SMUXEN bit is cleared once the SMUX operation is finished
    bool smuxen = readRegisterBit(AS7341_ENABLE, 4);
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

bool AS7341Component::waitForData() {
  // TODO
  ESP_LOGCONFIG(TAG, "Wait for data...");

  uint16_t timeout = 1000;
  bool success = false;
  for (uint16_t time = 0; time < timeout; time++) {
    success = isDataReady();

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

bool AS7341Component::isDataReady() {
  // uint8_t status;
  // this->read_byte(AS7341_STATUS, &status);
  // bool success = readRegisterBit(AS7341_STATUS2, 6);
  bool success = readRegisterBit(AS7341_STATUS2, 6);
  // ESP_LOGCONFIG(TAG, "Is data ready?: %u", success);
  return success;
}

bool AS7341Component::enablePower(bool enable) { return writeRegisterBit(AS7341_ENABLE, enable, 0); }

bool AS7341Component::enableSpectralMeasurement(bool enable) { return writeRegisterBit(AS7341_ENABLE, enable, 1); }

bool AS7341Component::readRegisterBit(uint8_t address, uint8_t bitPosition) {
  uint8_t data;
  this->read_byte(address, &data);
  // ESP_LOGCONFIG(TAG, "  read_byte: 0x%X", data);
  bool bit = (data & (1 << bitPosition)) > 0;
  // ESP_LOGCONFIG(TAG, "  read bit[%u]: 0x%u", bitPosition, bit);
  return bit;
}

bool AS7341Component::writeRegisterBit(uint8_t address, bool value, uint8_t bitPosition) {
  if (value) {
    return setRegisterBit(address, bitPosition);
  }

  return clearRegisterBit(address, bitPosition);
}

bool AS7341Component::setRegisterBit(uint8_t address, uint8_t bitPosition) {
  uint8_t data;
  this->read_byte(address, &data);
  data |= (1 << bitPosition);
  return this->write_byte(address, data);
}

bool AS7341Component::clearRegisterBit(uint8_t address, uint8_t bitPosition) {
  uint8_t data;
  this->read_byte(address, &data);
  data &= ~(1 << bitPosition);
  return this->write_byte(address, data);
}

uint16_t AS7341Component::swapBytes(uint16_t data) { return (data >> 8) | (data << 8); }
