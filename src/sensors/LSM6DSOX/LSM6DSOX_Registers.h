/*
  LSM6DSOX_Registers.h - Register map for ST LSM6DSOX 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_LSM6DSOX_REGISTERS_H
#define ARDUINONRF_IMU_LSM6DSOX_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace lsm6dsox {

static constexpr uint8_t kAddrSA0Low = 0x6A;
static constexpr uint8_t kAddrSA0High = 0x6B;
static constexpr uint8_t kWhoAmI = 0x6C;

static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t INT1_CTRL = 0x0D;
static constexpr uint8_t INT2_CTRL = 0x0E;
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G = 0x11;
static constexpr uint8_t CTRL3_C = 0x12;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t OUT_TEMP_L = 0x20;
static constexpr uint8_t OUTX_L_G = 0x22;
static constexpr uint8_t OUTX_L_XL = 0x28;

static constexpr uint8_t CTRL3_BDU = 0x40;
static constexpr uint8_t CTRL3_IF_INC = 0x04;
static constexpr uint8_t CTRL3_SW_RESET = 0x01;
static constexpr uint8_t CTRL3_H_LACTIVE = 0x20;
static constexpr uint8_t CTRL3_PP_OD = 0x10;
static constexpr uint8_t INT_DRDY_XL = 0x01;
static constexpr uint8_t INT_DRDY_G = 0x02;
static constexpr uint8_t MLC0_SRC = 0x70;

static constexpr uint8_t STATUS_XLDA = 0x01;
static constexpr uint8_t STATUS_GDA = 0x02;

static constexpr uint8_t ODR_12_5 = 0x10;
static constexpr uint8_t ODR_26 = 0x20;
static constexpr uint8_t ODR_52 = 0x30;
static constexpr uint8_t ODR_104 = 0x40;
static constexpr uint8_t ODR_208 = 0x50;
static constexpr uint8_t ODR_416 = 0x60;
static constexpr uint8_t ODR_833 = 0x70;
static constexpr uint8_t ODR_1660 = 0x80;
static constexpr uint8_t ODR_3330 = 0x90;
static constexpr uint8_t ODR_6660 = 0xA0;

static constexpr uint8_t XL_FS_2 = 0x00;
static constexpr uint8_t XL_FS_16 = 0x04;
static constexpr uint8_t XL_FS_4 = 0x08;
static constexpr uint8_t XL_FS_8 = 0x0C;
static constexpr uint8_t XL_FS_MASK = 0x0C;

static constexpr uint8_t G_FS_250 = 0x00;
static constexpr uint8_t G_FS_500 = 0x04;
static constexpr uint8_t G_FS_1000 = 0x08;
static constexpr uint8_t G_FS_2000 = 0x0C;
static constexpr uint8_t G_FS_125 = 0x02;
static constexpr uint8_t G_FS_MASK = 0x0E;

}  // namespace lsm6dsox
}  // namespace nimu

#endif  // ARDUINONRF_IMU_LSM6DSOX_REGISTERS_H
