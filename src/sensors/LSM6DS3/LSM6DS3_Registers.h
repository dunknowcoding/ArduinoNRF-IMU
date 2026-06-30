/*
  LSM6DS3_Registers.h - Register map for ST LSM6DS3 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_LSM6DS3_REGISTERS_H
#define ARDUINONRF_IMU_LSM6DS3_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace lsm6ds3 {

static constexpr uint8_t kAddrSA0Low = 0x6A;
static constexpr uint8_t kAddrSA0High = 0x6B;
static constexpr uint8_t kWhoAmI = 0x69;

static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t FUNC_CFG_ACCESS = 0x01;
static constexpr uint8_t INT1_CTRL = 0x0D;
static constexpr uint8_t INT2_CTRL = 0x0E;
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G = 0x11;
static constexpr uint8_t CTRL3_C = 0x12;
static constexpr uint8_t CTRL4_C = 0x13;
static constexpr uint8_t CTRL10_C = 0x19;
static constexpr uint8_t MASTER_CONFIG = 0x1A;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t OUT_TEMP_L = 0x20;
static constexpr uint8_t OUTX_L_G = 0x22;
static constexpr uint8_t OUTX_L_XL = 0x28;
static constexpr uint8_t SENSORHUB1 = 0x2E;
static constexpr uint8_t STEP_COUNTER_L = 0x4B;
static constexpr uint8_t FUNC_SRC = 0x53;
static constexpr uint8_t TAP_CFG = 0x58;
static constexpr uint8_t MD1_CFG = 0x5E;
static constexpr uint8_t MD2_CFG = 0x5F;

// Embedded-function register bank.
static constexpr uint8_t SLV0_ADD = 0x02;
static constexpr uint8_t SLV0_SUBADD = 0x03;
static constexpr uint8_t SLAVE0_CONFIG = 0x04;
static constexpr uint8_t DATAWRITE_SLV0 = 0x0E;
static constexpr uint8_t PEDO_THS_MIN = 0x0F;

static constexpr uint8_t CTRL3_BDU = 0x40;
static constexpr uint8_t CTRL3_IF_INC = 0x04;
static constexpr uint8_t CTRL3_SW_RESET = 0x01;
static constexpr uint8_t CTRL3_H_LACTIVE = 0x20;
static constexpr uint8_t CTRL3_PP_OD = 0x10;
static constexpr uint8_t FUNC_CFG_EN = 0x80;
static constexpr uint8_t CTRL10_SIGN_MOTION_EN = 0x01;
static constexpr uint8_t CTRL10_PEDO_RST_STEP = 0x02;
static constexpr uint8_t CTRL10_FUNC_EN = 0x04;
static constexpr uint8_t CTRL10_TILT_EN = 0x20;
static constexpr uint8_t CTRL10_PEDO_EN = 0x40;
static constexpr uint8_t MASTER_ON = 0x01;
static constexpr uint8_t PASS_THRU_MODE = 0x04;
static constexpr uint8_t SENSOR_HUB_PULL_UP = 0x08;
static constexpr uint8_t FUNC_SENS_HUB_END = 0x01;
static constexpr uint8_t FUNC_STEP_DETECTED = 0x10;
static constexpr uint8_t FUNC_TILT_DETECTED = 0x20;
static constexpr uint8_t FUNC_SIGN_MOTION_DETECTED = 0x40;
static constexpr uint8_t TAP_TILT_ENABLE = 0x20;
static constexpr uint8_t MD_TILT = 0x02;

static constexpr uint8_t INT_DRDY_XL = 0x01;
static constexpr uint8_t INT_DRDY_G = 0x02;
static constexpr uint8_t INT_BOOT = 0x04;
static constexpr uint8_t INT_FTH = 0x08;
static constexpr uint8_t INT_OVR = 0x10;
static constexpr uint8_t INT_FULL = 0x20;
static constexpr uint8_t INT_SIGN_MOT = 0x40;
static constexpr uint8_t INT_STEP = 0x80;

static constexpr uint8_t STATUS_XLDA = 0x01;
static constexpr uint8_t STATUS_GDA = 0x02;

static constexpr uint8_t ODR_POWER_DOWN = 0x00;
static constexpr uint8_t ODR_12_5 = 0x10;
static constexpr uint8_t ODR_26 = 0x20;
static constexpr uint8_t ODR_52 = 0x30;
static constexpr uint8_t ODR_104 = 0x40;
static constexpr uint8_t ODR_208 = 0x50;
static constexpr uint8_t ODR_416 = 0x60;
static constexpr uint8_t ODR_833 = 0x70;
static constexpr uint8_t ODR_1660 = 0x80;

static constexpr uint8_t XL_FS_2 = 0x00;
static constexpr uint8_t XL_FS_16 = 0x04;
static constexpr uint8_t XL_FS_4 = 0x08;
static constexpr uint8_t XL_FS_8 = 0x0C;
static constexpr uint8_t XL_FS_MASK = 0x0C;

static constexpr uint8_t G_FS_245 = 0x00;
static constexpr uint8_t G_FS_500 = 0x04;
static constexpr uint8_t G_FS_1000 = 0x08;
static constexpr uint8_t G_FS_2000 = 0x0C;
static constexpr uint8_t G_FS_MASK = 0x0C;

}  // namespace lsm6ds3
}  // namespace nimu

#endif  // ARDUINONRF_IMU_LSM6DS3_REGISTERS_H
