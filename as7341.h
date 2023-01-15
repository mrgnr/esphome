#pragma once

#include "esphome.h"
// #include "esphome/core/component.h"
// #include "esphome/components/sensor/sensor.h"
// #include "esphome/components/i2c/i2c.h"

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

static const uint8_t AS7341_CFG1 = 0xAA; ///< Controls ADC Gain


static const uint8_t AS7341_CFG6 = 0xAF;  // Stores SMUX command
static const uint8_t AS7341_CFG9 = 0xB2;  // Config for system interrupts (SMUX, Flicker detection)



static const uint8_t AS7341_ASTEP = 0xCA;  // LSB
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
  AS7341_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE, ///< Write SMUX configuration from RAM to SMUX chain
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
        AS7341Component() : PollingComponent(30000) {
            // this->address_ = AS7341_I2C_ADDR;
            // set_i2c_address(AS7341_I2C_ADDR);
            ESP_LOGCONFIG(TAG, "Constructor AS7341...");
            LOG_I2C_DEVICE(this);
        }

        Sensor *f1 = new Sensor();
        Sensor *f2 = new Sensor();
        Sensor *f3 = new Sensor();
        Sensor *f4 = new Sensor();
        // Sensor *clear_ = new Sensor();
        // Sensor *nir_ = new Sensor();
        Sensor *f5 = new Sensor();
        Sensor *f6 = new Sensor();
        Sensor *f7 = new Sensor();
        Sensor *f8 = new Sensor();
        Sensor *clear = new Sensor();
        Sensor *nir = new Sensor();

        void setup() override {
            ESP_LOGCONFIG(TAG, "Setting up AS7341...");
            
            // this->address_ = AS7341_I2C_ADDR;
            // set_i2c_address(AS7341_I2C_ADDR);
            // ESP_LOGCONFIG(TAG, "Set address AS7341...");
            LOG_I2C_DEVICE(this);


            // Verify device ID
            uint8_t id;
            this->read_byte(AS7341_ID, &id);
            ESP_LOGCONFIG(TAG, "  Read ID: 0x%X", id);
            if ((id & 0xFC ) != (AS7341_CHIP_ID<<2)) {
                this->mark_failed();
                return;
            }

            // Power on (enter IDLE state)
            if (!this->enablePower(true)) {
                this->mark_failed();
                return;
            }

            // Set configuration
            this->write_byte(AS7341_CONFIG, 0x00);
            uint8_t config;
            this->read_byte(AS7341_CONFIG, &config);
            ESP_LOGCONFIG(TAG, "  Config: 0x%X", config);


            // Set measurement parameters
            // setGain(AS7341_GAIN_0_5X);
            setGain(AS7341_GAIN_8X);
            // setGain(AS7341_GAIN_64X);
            // setGain(AS7341_GAIN_512X);
            setATIME(29);
            setASTEP(599);
        }
        
        float get_setup_priority() const {
            return setup_priority::DATA;
        }

        void dump_config() override {
            ESP_LOGCONFIG(TAG, "Dump config AS7341:");
            LOG_I2C_DEVICE(this);
            // ESP_LOGCONFIG(TAG, "  Product ID: %u", this->prod_rev_id_ & 0x0F);
            // ESP_LOGCONFIG(TAG, "  Revision ID: %u", (this->prod_rev_id_ >> 4) & 0x0F);


        }

        void update() override {
            ESP_LOGCONFIG(TAG, "Update AS7341...");
            LOG_I2C_DEVICE(this);

            uint8_t config;
            this->read_byte(AS7341_CONFIG, &config);
            ESP_LOGCONFIG(TAG, "  Config: 0x%X", config);


            bool success = readChannels(_channel_readings);
            ESP_LOGCONFIG(TAG, "  readChannels: %u", success);

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

        bool setGain(as7341_gain_t gain) {
            return this->write_byte(AS7341_CFG1, gain);
        }

        bool setATIME(uint8_t atime) {
            return this->write_byte(AS7341_ATIME, atime);
        }

        bool setASTEP(uint16_t astep) {
            uint16_t astep_swapped = (astep>>8) | (astep<<8);
            return this->write_byte_16(AS7341_ASTEP, astep_swapped);
        }

        uint16_t readChannel(as7341_adc_channel_t channel) {
            enableSpectralMeasurement(true);

            uint16_t data;
            this->read_byte_16(AS7341_CH0_DATA_L + 2*channel, &data);

            // enableSpectralMeasurement(false);

            return swapBytes(data);
        }

        bool readChannels(uint16_t *data) {
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

        void setSMUXLowChannels(bool enable) {
            ESP_LOGCONFIG(TAG, "Set SMUX low channels: %u", enable);
            enableSpectralMeasurement(false);
            setSMUXCommand(AS7341_SMUX_CMD_WRITE);

            if (enable) {
                configureSMUXLowChannels();

            } else {
                configureSMUXHighChannels();

            }
            enableSmux();
        }

        bool setSMUXCommand(as7341_smux_cmd_t command) {
            uint8_t data = command << 3;  // Write to bits 4:3 of the register
            ESP_LOGCONFIG(TAG, "Set MUX Command: 0x%X", data);
            return this->write_byte(AS7341_CFG6, data);
        }

        void configureSMUXLowChannels() {
            ESP_LOGCONFIG(TAG, "Configure SMUX low channels");
            // SMUX Config for F1,F2,F3,F4,NIR,Clear
            this->write_byte(0x00, 0x30); // F3 left set to ADC2
            this->write_byte(0x01, 0x01); // F1 left set to ADC0
            this->write_byte(0x02, 0x00); // Reserved or disabled
            this->write_byte(0x03, 0x00); // F8 left disabled
            this->write_byte(0x04, 0x00); // F6 left disabled
            this->write_byte(0x05, 0x42); // F4 left connected to ADC3/f2 left connected to ADC1
            this->write_byte(0x06, 0x00); // F5 left disbled
            this->write_byte(0x07, 0x00); // F7 left disbled
            this->write_byte(0x08, 0x50); // CLEAR connected to ADC4
            this->write_byte(0x09, 0x00); // F5 right disabled
            this->write_byte(0x0A, 0x00); // F7 right disabled
            this->write_byte(0x0B, 0x00); // Reserved or disabled
            this->write_byte(0x0C, 0x20); // F2 right connected to ADC1
            this->write_byte(0x0D, 0x04); // F4 right connected to ADC3
            this->write_byte(0x0E, 0x00); // F6/F8 right disabled
            this->write_byte(0x0F, 0x30); // F3 right connected to AD2
            this->write_byte(0x10, 0x01); // F1 right connected to AD0
            this->write_byte(0x11, 0x50); // CLEAR right connected to AD4
            this->write_byte(0x12, 0x00); // Reserved or disabled
            this->write_byte(0x13, 0x06); // NIR connected to ADC5
        }

        void configureSMUXHighChannels() {
            // SMUX Config for F5,F6,F7,F8,NIR,Clear
            this->write_byte(0x00, 0x00); // F3 left disable
            this->write_byte(0x01, 0x00); // F1 left disable
            this->write_byte(0x02, 0x00); // reserved/disable
            this->write_byte(0x03, 0x40); // F8 left connected to ADC3
            this->write_byte(0x04, 0x02); // F6 left connected to ADC1
            this->write_byte(0x05, 0x00); // F4/ F2 disabled
            this->write_byte(0x06, 0x10); // F5 left connected to ADC0
            this->write_byte(0x07, 0x03); // F7 left connected to ADC2
            this->write_byte(0x08, 0x50); // CLEAR Connected to ADC4
            this->write_byte(0x09, 0x10); // F5 right connected to ADC0
            this->write_byte(0x0A, 0x03); // F7 right connected to ADC2
            this->write_byte(0x0B, 0x00); // Reserved or disabled
            this->write_byte(0x0C, 0x00); // F2 right disabled
            this->write_byte(0x0D, 0x00); // F4 right disabled
            this->write_byte(0x0E, 0x24); // F8 right connected to ADC2/ F6 right connected to ADC1
            this->write_byte(0x0F, 0x00); // F3 right disabled
            this->write_byte(0x10, 0x00); // F1 right disabled
            this->write_byte(0x11, 0x50); // CLEAR right connected to AD4
            this->write_byte(0x12, 0x00); // Reserved or disabled
            this->write_byte(0x13, 0x06); // NIR connected to ADC5
        }

        bool enableSmux() {
            ESP_LOGCONFIG(TAG, "Enable SMUX...");
            setRegisterBit(AS7341_ENABLE, 4);

            uint16_t timeout = 100;
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

                delay(10);
            }

            ESP_LOGCONFIG(TAG, "SMUX enabled success: %u", success);
            return success;
        }

        bool waitForData() {
            // TODO
            ESP_LOGCONFIG(TAG, "Wait for data...");

            uint16_t timeout = 10;
            bool success = false;
            for (uint16_t time = 0; time < timeout; time++) {
                success = isDataReady();

                if (success) {
                    ESP_LOGCONFIG(TAG, "Data is ready!!!");
                    break;
                }

                // TODO
                // ESP_LOGCONFIG(TAG, "Data delay: %u", time);

                delay(100);
            }

            // TODO
            // ESP_LOGCONFIG(TAG, "Data ready: %u", success);
            return success;
        }

        bool isDataReady() {
            // uint8_t status;
            // this->read_byte(AS7341_STATUS, &status);
            // bool success = readRegisterBit(AS7341_STATUS2, 6);
            bool success = readRegisterBit(AS7341_STATUS2, 6);
            ESP_LOGCONFIG(TAG, "Is data ready?: %u", success);
            return success;
        }

        bool enablePower(bool enable) {
            return writeRegisterBit(AS7341_ENABLE, enable, 0);
        }

        bool enableSpectralMeasurement(bool enable) {
            return writeRegisterBit(AS7341_ENABLE, enable, 1);
        }

        bool readRegisterBit(uint8_t address, uint8_t bitPosition) {
            uint8_t data;
            this->read_byte(address, &data);
            // ESP_LOGCONFIG(TAG, "  read_byte: 0x%X", data);
            bool bit = ( data & ( 1 << bitPosition ) ) > 0;
            // ESP_LOGCONFIG(TAG, "  read bit[%u]: 0x%u", bitPosition, bit);
            return bit;
        }

        bool writeRegisterBit(uint8_t address, bool value, uint8_t bitPosition) {
            if (value) { 
                return setRegisterBit(address, bitPosition);
            }

            return clearRegisterBit(address, bitPosition);
        }

        bool setRegisterBit(uint8_t address, uint8_t bitPosition) {
            uint8_t data;
            this->read_byte(address, &data);
            data |= (1 << bitPosition);
            return this->write_byte(address, data);
        }

        bool clearRegisterBit(uint8_t address, uint8_t bitPosition) {
            uint8_t data;
            this->read_byte(address, &data);
            data &= ~(1 << bitPosition);
            return this->write_byte(address, data);
        }

        uint16_t swapBytes(uint16_t data) {
            return (data>>8) | (data<<8);
        }
    private:
        uint16_t _channel_readings[12];

};


} // namespace as7341
} // namespace esphome
