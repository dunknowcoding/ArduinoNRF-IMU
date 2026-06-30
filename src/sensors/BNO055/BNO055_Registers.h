/*
  BNO055_Registers.h - Register map for Bosch BNO055 absolute orientation IMU.
*/
#ifndef ARDUINONRF_IMU_BNO055_REGISTERS_H
#define ARDUINONRF_IMU_BNO055_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace bno055 {

static constexpr uint8_t kAddrLow = 0x28;
static constexpr uint8_t kAddrHigh = 0x29;
static constexpr uint8_t kChipId = 0xA0;

static constexpr uint8_t CHIP_ID = 0x00;
static constexpr uint8_t PAGE_ID = 0x07;
static constexpr uint8_t ACCEL_DATA_X_LSB = 0x08;
static constexpr uint8_t MAG_DATA_X_LSB = 0x0E;
static constexpr uint8_t GYRO_DATA_X_LSB = 0x14;
static constexpr uint8_t EULER_H_LSB = 0x1A;
static constexpr uint8_t QUATERNION_W_LSB = 0x20;
static constexpr uint8_t LINEAR_ACCEL_DATA_X_LSB = 0x28;
static constexpr uint8_t GRAVITY_DATA_X_LSB = 0x2E;
static constexpr uint8_t TEMP = 0x34;
static constexpr uint8_t CALIB_STAT = 0x35;
static constexpr uint8_t INT_STA = 0x37;
static constexpr uint8_t UNIT_SEL = 0x3B;
static constexpr uint8_t OPR_MODE = 0x3D;
static constexpr uint8_t PWR_MODE = 0x3E;
static constexpr uint8_t SYS_TRIGGER = 0x3F;
static constexpr uint8_t ACC_OFFSET_X_LSB = 0x55;
static constexpr uint8_t CALIBRATION_PROFILE_LENGTH = 22;

static constexpr uint8_t MODE_CONFIG = 0x00;
static constexpr uint8_t MODE_ACCONLY = 0x01;
static constexpr uint8_t MODE_MAGONLY = 0x02;
static constexpr uint8_t MODE_GYRONLY = 0x03;
static constexpr uint8_t MODE_AMG = 0x07;
static constexpr uint8_t MODE_IMUPLUS = 0x08;
static constexpr uint8_t MODE_NDOF = 0x0C;

static constexpr uint8_t POWER_NORMAL = 0x00;
static constexpr uint8_t SYS_TRIGGER_RST = 0x20;
static constexpr uint8_t SYS_TRIGGER_RST_INT = 0x40;
static constexpr uint8_t SYS_TRIGGER_EXTCLK = 0x80;

}  // namespace bno055
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BNO055_REGISTERS_H
