/*
  BMI270_Registers.h - Register constants for Bosch BMI270.
*/
#ifndef ARDUINONRF_IMU_BMI270_REGISTERS_H
#define ARDUINONRF_IMU_BMI270_REGISTERS_H

#include <Arduino.h>

namespace nimu {
namespace bmi270 {

constexpr uint8_t kAddrSDOLow = 0x68;
constexpr uint8_t kAddrSDOHigh = 0x69;
constexpr uint8_t kChipId = 0x24;

constexpr uint8_t CHIP_ID = 0x00;
constexpr uint8_t STATUS = 0x03;
constexpr uint8_t DATA_ACCEL_X_L = 0x0C;
constexpr uint8_t DATA_GYRO_X_L = 0x12;
constexpr uint8_t INTERNAL_STATUS = 0x21;
constexpr uint8_t DATA_TEMP_L = 0x22;
constexpr uint8_t ACC_CONF = 0x40;
constexpr uint8_t ACC_RANGE = 0x41;
constexpr uint8_t GYR_CONF = 0x42;
constexpr uint8_t GYR_RANGE = 0x43;
constexpr uint8_t INT1_IO_CTRL = 0x53;
constexpr uint8_t INT2_IO_CTRL = 0x54;
constexpr uint8_t INT_LATCH = 0x55;
constexpr uint8_t INT_MAP_DATA = 0x58;
constexpr uint8_t INIT_CTRL = 0x59;
constexpr uint8_t INIT_ADDR_0 = 0x5B;
constexpr uint8_t INIT_ADDR_1 = 0x5C;
constexpr uint8_t INIT_DATA = 0x5E;
constexpr uint8_t PWR_CONF = 0x7C;
constexpr uint8_t PWR_CTRL = 0x7D;
constexpr uint8_t CMD = 0x7E;

constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t STATUS_DRDY_GYR = 0x40;
constexpr uint8_t STATUS_DRDY_ACC = 0x80;
constexpr uint8_t PWR_GYR_EN = 0x02;
constexpr uint8_t PWR_ACC_EN = 0x04;
constexpr uint8_t INTERNAL_STATUS_INIT_OK = 0x01;

constexpr uint8_t PERF_MODE = 0x80;
constexpr uint8_t INT_OUTPUT_ENABLE = 0x08;
constexpr uint8_t INT_OPEN_DRAIN = 0x04;
constexpr uint8_t INT_ACTIVE_HIGH = 0x02;

}  // namespace bmi270
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMI270_REGISTERS_H
