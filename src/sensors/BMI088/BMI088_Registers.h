/* Register constants for the Bosch BMI088 accel + gyro pair. */
#ifndef ARDUINONRF_IMU_BMI088_REGISTERS_H
#define ARDUINONRF_IMU_BMI088_REGISTERS_H

#include <Arduino.h>

namespace nimu {
namespace bmi088 {

static constexpr uint8_t kAccelAddrLow = 0x18;
static constexpr uint8_t kAccelAddrHigh = 0x19;
static constexpr uint8_t kGyroAddrLow = 0x68;
static constexpr uint8_t kGyroAddrHigh = 0x69;
static constexpr uint8_t kAccelId = 0x1E;
static constexpr uint8_t kGyroId = 0x0F;

static constexpr uint8_t ACC_CHIP_ID = 0x00;
static constexpr uint8_t ACC_STATUS = 0x03;
static constexpr uint8_t ACC_DATA = 0x12;
static constexpr uint8_t ACC_TEMP = 0x22;
static constexpr uint8_t ACC_CONF = 0x40;
static constexpr uint8_t ACC_RANGE = 0x41;
static constexpr uint8_t ACC_INT1_IO_CTRL = 0x53;
static constexpr uint8_t ACC_INT2_IO_CTRL = 0x54;
static constexpr uint8_t ACC_INT_MAP_DATA = 0x58;
static constexpr uint8_t ACC_PWR_CONF = 0x7C;
static constexpr uint8_t ACC_PWR_CTRL = 0x7D;
static constexpr uint8_t ACC_SOFT_RESET = 0x7E;

static constexpr uint8_t GYR_CHIP_ID = 0x00;
static constexpr uint8_t GYR_DATA = 0x02;
static constexpr uint8_t GYR_INT_STATUS = 0x0A;
static constexpr uint8_t GYR_RANGE = 0x0F;
static constexpr uint8_t GYR_BANDWIDTH = 0x10;
static constexpr uint8_t GYR_POWER = 0x11;
static constexpr uint8_t GYR_SOFT_RESET = 0x14;
static constexpr uint8_t GYR_INT_CTRL = 0x15;
static constexpr uint8_t GYR_INT3_INT4_IO_CONF = 0x16;
static constexpr uint8_t GYR_INT3_INT4_IO_MAP = 0x18;

static constexpr uint8_t SOFT_RESET = 0xB6;

}  // namespace bmi088
}  // namespace nimu

#endif
