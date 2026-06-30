/*
  BMI160_Registers.h - Register map for Bosch BMI160 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_BMI160_REGISTERS_H
#define ARDUINONRF_IMU_BMI160_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace bmi160 {

static constexpr uint8_t kAddrSDOLow = 0x68;
static constexpr uint8_t kAddrSDOHigh = 0x69;
static constexpr uint8_t kChipId = 0xD1;

static constexpr uint8_t CHIP_ID = 0x00;
static constexpr uint8_t DATA_GYRO_X_L = 0x0C;
static constexpr uint8_t DATA_ACCEL_X_L = 0x12;
static constexpr uint8_t STATUS = 0x1B;
static constexpr uint8_t DATA_TEMP_L = 0x20;
static constexpr uint8_t ACC_CONF = 0x40;
static constexpr uint8_t ACC_RANGE = 0x41;
static constexpr uint8_t GYR_CONF = 0x42;
static constexpr uint8_t GYR_RANGE = 0x43;
static constexpr uint8_t INT_EN_1 = 0x51;
static constexpr uint8_t INT_OUT_CTRL = 0x53;
static constexpr uint8_t INT_LATCH = 0x54;
static constexpr uint8_t INT_MAP_1 = 0x56;
static constexpr uint8_t CMD = 0x7E;

static constexpr uint8_t STATUS_DRDY_ACC = 0x80;
static constexpr uint8_t STATUS_DRDY_GYR = 0x40;

static constexpr uint8_t CMD_ACC_NORMAL = 0x11;
static constexpr uint8_t CMD_GYR_NORMAL = 0x15;
static constexpr uint8_t CMD_SOFT_RESET = 0xB6;
static constexpr uint8_t INT_DATA_READY_ENABLE = 0x10;

}  // namespace bmi160
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMI160_REGISTERS_H
