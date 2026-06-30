/*
  ICM20948_Registers.h - Register constants for TDK InvenSense ICM-20948.
*/
#ifndef ARDUINONRF_IMU_ICM20948_REGISTERS_H
#define ARDUINONRF_IMU_ICM20948_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace icm20948 {

static constexpr uint8_t kAddrAD0Low = 0x68;
static constexpr uint8_t kAddrAD0High = 0x69;
static constexpr uint8_t kWhoAmI = 0xEA;

static constexpr uint8_t REG_BANK_SEL = 0x7F;

// Bank 0
static constexpr uint8_t WHO_AM_I = 0x00;
static constexpr uint8_t USER_CTRL = 0x03;
static constexpr uint8_t PWR_MGMT_1 = 0x06;
static constexpr uint8_t PWR_MGMT_2 = 0x07;
static constexpr uint8_t INT_PIN_CFG = 0x0F;
static constexpr uint8_t INT_ENABLE = 0x10;
static constexpr uint8_t INT_ENABLE_1 = 0x11;
static constexpr uint8_t I2C_MST_STATUS = 0x17;
static constexpr uint8_t INT_STATUS_1 = 0x1A;
static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
static constexpr uint8_t GYRO_XOUT_H = 0x33;
static constexpr uint8_t TEMP_OUT_H = 0x39;

// Bank 2
static constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
static constexpr uint8_t GYRO_CONFIG_1 = 0x01;
static constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
static constexpr uint8_t ACCEL_CONFIG = 0x14;

// Bank 3: internal I2C master, connected to the ACL/ADA auxiliary pins.
static constexpr uint8_t I2C_MST_CTRL = 0x01;
static constexpr uint8_t I2C_SLV4_ADDR = 0x13;
static constexpr uint8_t I2C_SLV4_REG = 0x14;
static constexpr uint8_t I2C_SLV4_CTRL = 0x15;
static constexpr uint8_t I2C_SLV4_DO = 0x16;
static constexpr uint8_t I2C_SLV4_DI = 0x17;

static constexpr uint8_t PWR_DEVICE_RESET = 0x80;
static constexpr uint8_t PWR_CLK_AUTO = 0x01;
static constexpr uint8_t INT_RAW_DATA_0_RDY = 0x01;
static constexpr uint8_t INT_ACTIVE_LOW = 0x80;
static constexpr uint8_t INT_OPEN_DRAIN = 0x40;
static constexpr uint8_t INT_LATCH = 0x20;
static constexpr uint8_t INT_CLEAR_ANY_READ = 0x10;
static constexpr uint8_t FSYNC_ACTIVE_LOW = 0x08;
static constexpr uint8_t FSYNC_INTERRUPT = 0x04;
static constexpr uint8_t BYPASS_ENABLE = 0x02;
static constexpr uint8_t RAW_DATA_0_RDY_ENABLE = 0x01;
static constexpr uint8_t REG_WOF_ENABLE = 0x80;
static constexpr uint8_t I2C_MST_ENABLE = 0x20;
static constexpr uint8_t I2C_SLV4_ENABLE = 0x80;
static constexpr uint8_t I2C_SLV4_DONE = 0x40;
static constexpr uint8_t I2C_SLV4_NACK = 0x10;
static constexpr uint8_t FS_MASK = 0x06;
static constexpr uint8_t DLPF_MASK = 0x39;

}  // namespace icm20948
}  // namespace nimu

#endif  // ARDUINONRF_IMU_ICM20948_REGISTERS_H
