/*
  BMP280_Registers.h - Register map for the Bosch BMP280 pressure + temperature
  sensor (the barometer fitted next to the MPU-6500 on GY-91 boards).

  Only the registers the driver uses are listed. The factory calibration
  coefficients (0x88..0x9F) are read once at begin() and fed into the Bosch
  compensation formulas; they are the chip's own trim, not a user calibration.
*/
#ifndef ARDUINONRF_IMU_BMP280_REGISTERS_H
#define ARDUINONRF_IMU_BMP280_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace bmp280 {

// --- I2C addresses (SDO pin selects) --------------------------------------
static constexpr uint8_t kAddrPrimary = 0x76;    // SDO low  (GY-91 default)
static constexpr uint8_t kAddrSecondary = 0x77;  // SDO high

// Chip id (register 0xD0). BME280 (with humidity) reads 0x60 instead.
static constexpr uint8_t kChipId = 0x58;

// --- registers ------------------------------------------------------------
static constexpr uint8_t REG_CALIB00 = 0x88;   // 24 bytes of dig_T*/dig_P*
static constexpr uint8_t REG_ID = 0xD0;
static constexpr uint8_t REG_RESET = 0xE0;     // write 0xB6 to soft-reset
static constexpr uint8_t REG_STATUS = 0xF3;    // bit3 measuring, bit0 im_update
static constexpr uint8_t REG_CTRL_MEAS = 0xF4; // osrs_t[7:5] osrs_p[4:2] mode[1:0]
static constexpr uint8_t REG_CONFIG = 0xF5;    // t_sb[7:5] filter[4:2] spi3w[0]
static constexpr uint8_t REG_PRESS_MSB = 0xF7; // F7..F9 pressure, FA..FC temp

static constexpr uint8_t RESET_WORD = 0xB6;

// ctrl_meas mode field
static constexpr uint8_t MODE_SLEEP = 0x00;
static constexpr uint8_t MODE_FORCED = 0x01;
static constexpr uint8_t MODE_NORMAL = 0x03;

// Oversampling codes (used for both osrs_t and osrs_p)
static constexpr uint8_t OSRS_SKIP = 0x00;
static constexpr uint8_t OSRS_X1 = 0x01;
static constexpr uint8_t OSRS_X2 = 0x02;
static constexpr uint8_t OSRS_X4 = 0x03;
static constexpr uint8_t OSRS_X8 = 0x04;
static constexpr uint8_t OSRS_X16 = 0x05;

// IIR filter coefficients (config[4:2])
static constexpr uint8_t FILTER_OFF = 0x00;
static constexpr uint8_t FILTER_2 = 0x01;
static constexpr uint8_t FILTER_4 = 0x02;
static constexpr uint8_t FILTER_8 = 0x03;
static constexpr uint8_t FILTER_16 = 0x04;

// Standby time in normal mode (config[7:5])
static constexpr uint8_t STANDBY_0_5MS = 0x00;
static constexpr uint8_t STANDBY_62_5MS = 0x01;
static constexpr uint8_t STANDBY_125MS = 0x02;
static constexpr uint8_t STANDBY_250MS = 0x03;

}  // namespace bmp280
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMP280_REGISTERS_H
