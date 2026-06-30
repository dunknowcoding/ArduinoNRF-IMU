/*
  ADXL345_Registers.h - Register constants for Analog Devices ADXL345.
*/
#ifndef ARDUINONRF_IMU_ADXL345_REGISTERS_H
#define ARDUINONRF_IMU_ADXL345_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace adxl345 {

static constexpr uint8_t kAddrSDOLow = 0x53;
static constexpr uint8_t kAddrSDOHigh = 0x1D;
static constexpr uint8_t kDeviceId = 0xE5;

static constexpr uint8_t DEVID = 0x00;
static constexpr uint8_t BW_RATE = 0x2C;
static constexpr uint8_t POWER_CTL = 0x2D;
static constexpr uint8_t INT_ENABLE = 0x2E;
static constexpr uint8_t INT_MAP = 0x2F;
static constexpr uint8_t INT_SOURCE = 0x30;
static constexpr uint8_t DATA_FORMAT = 0x31;
static constexpr uint8_t DATAX0 = 0x32;

static constexpr uint8_t POWER_MEASURE = 0x08;
static constexpr uint8_t INT_DATA_READY = 0x80;
static constexpr uint8_t DATA_FULL_RES = 0x08;
static constexpr uint8_t DATA_INT_INVERT = 0x20;

}  // namespace adxl345
}  // namespace nimu

#endif  // ARDUINONRF_IMU_ADXL345_REGISTERS_H
