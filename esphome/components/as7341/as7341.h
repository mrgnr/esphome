#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace as7341 {

static const uint8_t AS7341_I2C_ADDR = 0x39;

static const uint8_t AS7341_ASTATUS = 0x60;
static const uint8_t AS7341_CH0_DATA_L = 0x61;
static const uint8_t AS7341_CH0_DATA_H = 0x62;
static const uint8_t AS7341_ITIME_L_1 = 0x63;
static const uint8_t AS7341_ITIME_L_2 = 0x64;
static const uint8_t AS7341_ITIME_L_3 = 0x65;
static const uint8_t AS7341_CH1_DATA_L = 0x66;
static const uint8_t AS7341_CH1_DATA_H = 0x67;
static const uint8_t AS7341_CH2_DATA_L = 0x68;
static const uint8_t AS7341_CH2_DATA_H = 0x69;
static const uint8_t AS7341_CH3_DATA_L = 0x6A;
static const uint8_t AS7341_CH3_DATA_H = 0x6B;
static const uint8_t AS7341_CH4_DATA_L = 0x6C;
static const uint8_t AS7341_CH4_DATA_H = 0x6D;
static const uint8_t AS7341_CH5_DATA_L = 0x6E;
static const uint8_t AS7341_CH5_DATA_H = 0x6F;
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

class AS7341Component : public PollingComponent, public i2c::I2CDevice {

};


} // namespace as7341
} // namespace esphome
