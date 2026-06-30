/*
  BMP180_Registers.h - Register map for the Bosch BMP180 pressure sensor.
*/
#ifndef ARDUINONRF_IMU_BMP180_REGISTERS_H
#define ARDUINONRF_IMU_BMP180_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace bmp180 {

static constexpr uint8_t kAddr = 0x77;
static constexpr uint8_t kChipId = 0x55;

static constexpr uint8_t REG_CALIB = 0xAA;
static constexpr uint8_t REG_ID = 0xD0;
static constexpr uint8_t REG_CONTROL = 0xF4;
static constexpr uint8_t REG_DATA = 0xF6;
static constexpr uint8_t CMD_TEMP = 0x2E;
static constexpr uint8_t CMD_PRESSURE = 0x34;

}  // namespace bmp180
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMP180_REGISTERS_H
