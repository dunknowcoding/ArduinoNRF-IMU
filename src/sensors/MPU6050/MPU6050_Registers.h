/*
  MPU6050_Registers.h - Register map for the InvenSense MPU-6050 6-axis IMU
  (accelerometer + gyroscope + die temperature).

  The MPU-6050 is the chip on the common GY-521 board and on many GY-87
  10-DOF boards. It is close to the MPU-6500 family, but lacks ACCEL_CONFIG2
  and uses a different temperature conversion, so it has its own driver.
*/
#ifndef ARDUINONRF_IMU_MPU6050_REGISTERS_H
#define ARDUINONRF_IMU_MPU6050_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace mpu6050 {

// --- I2C addresses --------------------------------------------------------
static constexpr uint8_t kAddrAD0Low = 0x68;   // AD0 pin low (GY-521 default)
static constexpr uint8_t kAddrAD0High = 0x69;  // AD0 pin high
static constexpr uint8_t kWhoAmI = 0x68;

// --- register addresses ---------------------------------------------------
static constexpr uint8_t SMPLRT_DIV = 0x19;
static constexpr uint8_t CONFIG = 0x1A;
static constexpr uint8_t GYRO_CONFIG = 0x1B;
static constexpr uint8_t ACCEL_CONFIG = 0x1C;
static constexpr uint8_t INT_PIN_CFG = 0x37;
static constexpr uint8_t INT_ENABLE = 0x38;
static constexpr uint8_t INT_STATUS = 0x3A;
static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t TEMP_OUT_H = 0x41;
static constexpr uint8_t GYRO_XOUT_H = 0x43;
static constexpr uint8_t USER_CTRL = 0x6A;
static constexpr uint8_t PWR_MGMT_1 = 0x6B;
static constexpr uint8_t PWR_MGMT_2 = 0x6C;
static constexpr uint8_t WHO_AM_I = 0x75;

// PWR_MGMT_1 bits
static constexpr uint8_t PWR1_H_RESET = 0x80;
static constexpr uint8_t PWR1_SLEEP = 0x40;
static constexpr uint8_t PWR1_CLKSEL_PLL_XGYRO = 0x01;

// INT_PIN_CFG bits
static constexpr uint8_t INTCFG_BYPASS_EN = 0x02;  // expose aux I2C on main bus
static constexpr uint8_t INTCFG_LATCH_INT = 0x20;
static constexpr uint8_t INTCFG_ACTIVE_LOW = 0x80;
static constexpr uint8_t INTCFG_OPEN_DRAIN = 0x40;
static constexpr uint8_t INTCFG_CLEAR_ANY_READ = 0x10;
static constexpr uint8_t EXT_SYNC_MASK = 0x38;

// USER_CTRL bits
static constexpr uint8_t USERCTRL_I2C_MST_EN = 0x20;

// INT_ENABLE / INT_STATUS bits
static constexpr uint8_t INT_RAW_RDY = 0x01;

// GYRO_CONFIG: full-scale select occupies bits[4:3]
static constexpr uint8_t GYRO_FS_250 = 0x00 << 3;
static constexpr uint8_t GYRO_FS_500 = 0x01 << 3;
static constexpr uint8_t GYRO_FS_1000 = 0x02 << 3;
static constexpr uint8_t GYRO_FS_2000 = 0x03 << 3;
static constexpr uint8_t GYRO_FS_MASK = 0x18;

// ACCEL_CONFIG: full-scale select occupies bits[4:3]
static constexpr uint8_t ACCEL_FS_2 = 0x00 << 3;
static constexpr uint8_t ACCEL_FS_4 = 0x01 << 3;
static constexpr uint8_t ACCEL_FS_8 = 0x02 << 3;
static constexpr uint8_t ACCEL_FS_16 = 0x03 << 3;
static constexpr uint8_t ACCEL_FS_MASK = 0x18;

// Temperature conversion: degC = raw / 340.0 + 36.53
static constexpr float kTempSensitivity = 340.0f;
static constexpr float kTempOffsetC = 36.53f;

}  // namespace mpu6050
}  // namespace nimu

#endif  // ARDUINONRF_IMU_MPU6050_REGISTERS_H
