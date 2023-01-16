#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace as7341 {

static const uint8_t AS7341_I2C_ADDR = 0x39;
static const uint8_t AS7341_CHIP_ID = 0X09;

static const uint8_t AS7341_ASTATUS = 0x60;
// static const uint8_t AS7341_CH0_DATA_L = 0x61;
// static const uint8_t AS7341_CH0_DATA_H = 0x62;
static const uint8_t AS7341_ITIME_L_1 = 0x63;
static const uint8_t AS7341_ITIME_L_2 = 0x64;
static const uint8_t AS7341_ITIME_L_3 = 0x65;
// static const uint8_t AS7341_CH1_DATA_L = 0x66;
// static const uint8_t AS7341_CH1_DATA_H = 0x67;
// static const uint8_t AS7341_CH2_DATA_L = 0x68;
// static const uint8_t AS7341_CH2_DATA_H = 0x69;
// static const uint8_t AS7341_CH3_DATA_L = 0x6A;
// static const uint8_t AS7341_CH3_DATA_H = 0x6B;
// static const uint8_t AS7341_CH4_DATA_L = 0x6C;
// static const uint8_t AS7341_CH4_DATA_H = 0x6D;
// static const uint8_t AS7341_CH5_DATA_L = 0x6E;
// static const uint8_t AS7341_CH5_DATA_H = 0x6F;
static const uint8_t AS7341_CONFIG = 0x70;
static const uint8_t AS7341_STAT = 0x71;
static const uint8_t AS7341_EDGE = 0x72;
static const uint8_t AS7341_GPIO = 0x73;
static const uint8_t AS7341_LED = 0x74;

static const uint8_t AS7341_ENABLE = 0x80;
static const uint8_t AS7341_ATIME = 0x81;

static const uint8_t AS7341_WTIME = 0x83;

static const uint8_t AS7341_SP_TH_L_LSB = 0x84;
static const uint8_t AS7341_SP_TH_L_MSB = 0x85;
static const uint8_t AS7341_SP_TH_H_LSB = 0x86;
static const uint8_t AS7341_SP_TH_H_MSB = 0x87;

static const uint8_t AS7341_AUXID = 0x90;
static const uint8_t AS7341_REVID = 0x91;
static const uint8_t AS7341_ID = 0x92;
static const uint8_t AS7341_STATUS = 0x93;

static const uint8_t AS7341_CH0_DATA_L = 0x95;
static const uint8_t AS7341_CH0_DATA_H = 0x96;
static const uint8_t AS7341_CH1_DATA_L = 0x97;
static const uint8_t AS7341_CH1_DATA_H = 0x98;
static const uint8_t AS7341_CH2_DATA_L = 0x99;
static const uint8_t AS7341_CH2_DATA_H = 0x9A;
static const uint8_t AS7341_CH3_DATA_L = 0x9B;
static const uint8_t AS7341_CH3_DATA_H = 0x9C;
static const uint8_t AS7341_CH4_DATA_L = 0x9D;
static const uint8_t AS7341_CH4_DATA_H = 0x9E;
static const uint8_t AS7341_CH5_DATA_L = 0x9F;
static const uint8_t AS7341_CH5_DATA_H = 0xA0;

static const uint8_t AS7341_STATUS2 = 0xA3;

static const uint8_t AS7341_CFG1 = 0xAA;  ///< Controls ADC Gain

static const uint8_t AS7341_CFG6 = 0xAF;  // Stores SMUX command
static const uint8_t AS7341_CFG9 = 0xB2;  // Config for system interrupts (SMUX, Flicker detection)

static const uint8_t AS7341_ASTEP = 0xCA;      // LSB
static const uint8_t AS7341_ASTEP_MSB = 0xCB;  // MSB

static const char *const TAG = "as7341";

typedef enum {
  AS7341_ADC_CHANNEL_0,
  AS7341_ADC_CHANNEL_1,
  AS7341_ADC_CHANNEL_2,
  AS7341_ADC_CHANNEL_3,
  AS7341_ADC_CHANNEL_4,
  AS7341_ADC_CHANNEL_5,
} as7341_adc_channel_t;

typedef enum {
  AS7341_SMUX_CMD_ROM_RESET,  ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,       ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE,      ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;

typedef enum {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X,
} as7341_gain_t;

class AS7341Component : public PollingComponent, public i2c::I2CDevice {
 public:
  AS7341Component() {
    // this->address_ = AS7341_I2C_ADDR;
    // set_i2c_address(AS7341_I2C_ADDR);
    ESP_LOGCONFIG(TAG, "Constructor AS7341...");
    LOG_I2C_DEVICE(this);
  }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_f1_sensor(sensor::Sensor *f1_sensor) { this->f1 = f1_sensor; }
  void set_f2_sensor(sensor::Sensor *f2_sensor) { this->f2 = f2_sensor; }
  void set_f3_sensor(sensor::Sensor *f3_sensor) { this->f3 = f3_sensor; }
  void set_f4_sensor(sensor::Sensor *f4_sensor) { this->f4 = f4_sensor; }
  void set_f5_sensor(sensor::Sensor *f5_sensor) { this->f5 = f5_sensor; }
  void set_f6_sensor(sensor::Sensor *f6_sensor) { this->f6 = f6_sensor; }
  void set_f7_sensor(sensor::Sensor *f7_sensor) { this->f7 = f7_sensor; }
  void set_f8_sensor(sensor::Sensor *f8_sensor) { this->f8 = f8_sensor; }
  void set_clear_sensor(sensor::Sensor *clear_sensor) { this->clear = clear_sensor; }
  void set_nir_sensor(sensor::Sensor *nir_sensor) { this->nir = nir_sensor; }

  void set_gain(as7341_gain_t gain) { _gain = gain; }
  void set_atime(uint8_t atime) { _atime = atime; }
  void set_astep(uint16_t astep) { _astep = astep; }

  as7341_gain_t get_gain();
  uint8_t get_atime();
  uint16_t get_astep();
  bool setup_gain(as7341_gain_t gain);
  bool setup_atime(uint8_t atime);
  bool setup_astep(uint16_t astep);

  uint16_t read_channel(as7341_adc_channel_t channel);
  bool read_channels(uint16_t *data);
  void set_smux_low_channels(bool enable);
  bool set_smux_command(as7341_smux_cmd_t command);
  void configure_smux_low_channels();
  void configure_smux_high_channels();
  bool enable_smux();

  bool wait_for_data();
  bool is_data_ready();
  bool enable_power(bool enable);
  bool enable_spectral_measurement(bool enable);

  bool read_register_bit(uint8_t address, uint8_t bit_position);
  bool write_register_bit(uint8_t address, bool value, uint8_t bit_position);
  bool set_register_bit(uint8_t address, uint8_t bit_position);
  bool clear_register_bit(uint8_t address, uint8_t bit_position);
  uint16_t swap_bytes(uint16_t data);

 protected:
  sensor::Sensor *f1{nullptr};
  sensor::Sensor *f2{nullptr};
  sensor::Sensor *f3{nullptr};
  sensor::Sensor *f4{nullptr};
  // sensor::Sensor *clear_{nullptr};
  // sensor::Sensor *nir_{nullptr};
  sensor::Sensor *f5{nullptr};
  sensor::Sensor *f6{nullptr};
  sensor::Sensor *f7{nullptr};
  sensor::Sensor *f8{nullptr};
  sensor::Sensor *clear{nullptr};
  sensor::Sensor *nir{nullptr};

  // sensor::Sensor *f1 = new sensor::Sensor();
  // sensor::Sensor *f2 = new sensor::Sensor();
  // sensor::Sensor *f3 = new sensor::Sensor();
  // sensor::Sensor *f4 = new sensor::Sensor();
  // // sensor::Sensor *clear_ = new sensor::Sensor();
  // // sensor::Sensor *nir_ = new sensor::Sensor();
  // sensor::Sensor *f5 = new sensor::Sensor();
  // sensor::Sensor *f6 = new sensor::Sensor();
  // sensor::Sensor *f7 = new sensor::Sensor();
  // sensor::Sensor *f8 = new sensor::Sensor();
  // sensor::Sensor *clear = new sensor::Sensor();
  // sensor::Sensor *nir = new sensor::Sensor();
  uint16_t _astep;
  as7341_gain_t _gain;
  uint8_t _atime;
  uint16_t _channel_readings[12];
};

}  // namespace as7341
}  // namespace esphome
