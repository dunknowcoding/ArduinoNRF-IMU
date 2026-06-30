/*
  L3G4200D_Registers.h - Register constants for ST L3G4200D gyroscope.
*/
#ifndef ARDUINONRF_IMU_L3G4200D_REGISTERS_H
#define ARDUINONRF_IMU_L3G4200D_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace l3g4200d {

static constexpr uint8_t kAddrSDOLow = 0x68;
static constexpr uint8_t kAddrSDOHigh = 0x69;
static constexpr uint8_t kWhoAmI = 0xD3;

static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t CTRL_REG1 = 0x20;
static constexpr uint8_t CTRL_REG2 = 0x21;
static constexpr uint8_t CTRL_REG3 = 0x22;
static constexpr uint8_t CTRL_REG4 = 0x23;
static constexpr uint8_t CTRL_REG5 = 0x24;
static constexpr uint8_t OUT_TEMP = 0x26;
static constexpr uint8_t STATUS_REG = 0x27;
static constexpr uint8_t OUT_X_L = 0x28;

static constexpr uint8_t AUTO_INCREMENT = 0x80;
static constexpr uint8_t CTRL1_POWER_XYZ = 0x0F;
static constexpr uint8_t CTRL4_BDU = 0x80;
static constexpr uint8_t STATUS_ZYXDA = 0x08;
static constexpr uint8_t CTRL3_I1_INT1 = 0x80;
static constexpr uint8_t CTRL3_I1_BOOT = 0x40;
static constexpr uint8_t CTRL3_H_LACTIVE = 0x20;
static constexpr uint8_t CTRL3_PP_OD = 0x10;
static constexpr uint8_t CTRL3_I2_DRDY = 0x08;
static constexpr uint8_t CTRL3_I2_WTM = 0x04;
static constexpr uint8_t CTRL3_I2_ORUN = 0x02;
static constexpr uint8_t CTRL3_I2_EMPTY = 0x01;

static constexpr uint8_t FS_250 = 0x00;
static constexpr uint8_t FS_500 = 0x10;
static constexpr uint8_t FS_2000 = 0x20;
static constexpr uint8_t FS_MASK = 0x30;

}  // namespace l3g4200d
}  // namespace nimu

#endif  // ARDUINONRF_IMU_L3G4200D_REGISTERS_H
