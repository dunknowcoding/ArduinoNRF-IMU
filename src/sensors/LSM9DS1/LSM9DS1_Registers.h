/*
  LSM9DS1_Registers.h - Register map for ST LSM9DS1 9-axis IMU.
*/
#ifndef ARDUINONRF_IMU_LSM9DS1_REGISTERS_H
#define ARDUINONRF_IMU_LSM9DS1_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace lsm9ds1 {

static constexpr uint8_t kAddrAGHigh = 0x6B;
static constexpr uint8_t kAddrAGLow = 0x6A;
static constexpr uint8_t kAddrMagHigh = 0x1E;
static constexpr uint8_t kAddrMagLow = 0x1C;
static constexpr uint8_t kWhoAmIAG = 0x68;
static constexpr uint8_t kWhoAmIMag = 0x3D;

static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t INT1_CTRL = 0x0C;
static constexpr uint8_t INT2_CTRL = 0x0D;

// Accel/gyro register map.
static constexpr uint8_t OUT_TEMP_L = 0x15;
static constexpr uint8_t OUT_X_L_G = 0x18;
static constexpr uint8_t CTRL_REG1_G = 0x10;
static constexpr uint8_t CTRL_REG6_XL = 0x20;
static constexpr uint8_t CTRL_REG8 = 0x22;
static constexpr uint8_t STATUS_REG = 0x17;
static constexpr uint8_t OUT_X_L_XL = 0x28;
static constexpr uint8_t CTRL8_BDU = 0x40;
static constexpr uint8_t CTRL8_IF_ADD_INC = 0x04;
static constexpr uint8_t CTRL8_SW_RESET = 0x01;

// Magnetometer register map.
static constexpr uint8_t CTRL_REG1_M = 0x20;
static constexpr uint8_t CTRL_REG2_M = 0x21;
static constexpr uint8_t CTRL_REG3_M = 0x22;
static constexpr uint8_t CTRL_REG4_M = 0x23;
static constexpr uint8_t OUT_X_L_M = 0x28;
static constexpr uint8_t STATUS_REG_M = 0x27;
static constexpr uint8_t INT_CFG_M = 0x30;
static constexpr uint8_t INT_SRC_M = 0x31;
static constexpr uint8_t INT_THS_L_M = 0x32;

static constexpr uint8_t INT_DRDY_XL = 0x01;
static constexpr uint8_t INT_DRDY_G = 0x02;
static constexpr uint8_t STATUS_XLDA = 0x01;
static constexpr uint8_t STATUS_GDA = 0x02;
static constexpr uint8_t STATUS_M_ZYXDA = 0x08;

}  // namespace lsm9ds1
}  // namespace nimu

#endif  // ARDUINONRF_IMU_LSM9DS1_REGISTERS_H
