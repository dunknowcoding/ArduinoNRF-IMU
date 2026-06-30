#ifndef ARDUINONRF_IMU_LSM6DSV_REGISTERS_H
#define ARDUINONRF_IMU_LSM6DSV_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace lsm6dsv {

static constexpr uint8_t kAddrSA0Low = 0x6A;
static constexpr uint8_t kAddrSA0High = 0x6B;
static constexpr uint8_t kWhoAmI = 0x70;

static constexpr uint8_t FUNC_CFG_ACCESS = 0x01;
static constexpr uint8_t UI_STATUS_REG_OIS = 0x44;
static constexpr uint8_t UI_CTRL1_OIS = 0x70;
static constexpr uint8_t IF_CFG = 0x03;
static constexpr uint8_t INT1_CTRL = 0x0D;
static constexpr uint8_t INT2_CTRL = 0x0E;
static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t CTRL1 = 0x10;
static constexpr uint8_t CTRL2 = 0x11;
static constexpr uint8_t CTRL3 = 0x12;
static constexpr uint8_t CTRL6 = 0x15;
static constexpr uint8_t CTRL8 = 0x17;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t FIFO_CTRL4 = 0x0A;
static constexpr uint8_t FIFO_STATUS1 = 0x1B;
static constexpr uint8_t FIFO_STATUS2 = 0x1C;
static constexpr uint8_t OUT_TEMP_L = 0x20;
static constexpr uint8_t OUTX_L_G = 0x22;
static constexpr uint8_t OUTX_L_A = 0x28;
static constexpr uint8_t STATUS_MASTER = 0x48;
static constexpr uint8_t FIFO_DATA_OUT_TAG = 0x78;
static constexpr uint8_t FIFO_DATA_OUT_X_L = 0x79;

static constexpr uint8_t EMB_FUNC_EN_A = 0x04;
static constexpr uint8_t EMB_FUNC_FIFO_EN_A = 0x44;
static constexpr uint8_t SFLP_ODR = 0x5E;

static constexpr uint8_t SHUB_ACCESS = 0x40;
static constexpr uint8_t EMBED_ACCESS = 0x80;
static constexpr uint8_t OIS_CTRL_FROM_UI = 0x01;
static constexpr uint8_t OIS_G_ENABLE = 0x01;
static constexpr uint8_t OIS_XL_ENABLE = 0x02;
static constexpr uint8_t OIS_XL_DATA_READY = 0x01;
static constexpr uint8_t OIS_G_DATA_READY = 0x02;
static constexpr uint8_t SENSOR_HUB_1 = 0x02;
static constexpr uint8_t MASTER_CONFIG = 0x14;
static constexpr uint8_t SLV0_ADD = 0x15;
static constexpr uint8_t SLV0_SUBADD = 0x16;
static constexpr uint8_t SLV0_CONFIG = 0x17;
static constexpr uint8_t DATAWRITE_SLV0 = 0x21;

static constexpr uint8_t CTRL3_SW_RESET = 0x01;
static constexpr uint8_t CTRL3_IF_INC = 0x04;
static constexpr uint8_t CTRL3_BDU = 0x40;
static constexpr uint8_t IF_PP_OD = 0x08;
static constexpr uint8_t IF_H_LACTIVE = 0x10;
static constexpr uint8_t IF_SHUB_PU_EN = 0x40;
static constexpr uint8_t STATUS_XLDA = 0x01;
static constexpr uint8_t STATUS_GDA = 0x02;
static constexpr uint8_t STATUS_SHUB_DONE = 0x01;
static constexpr uint8_t INT_DRDY_XL = 0x01;
static constexpr uint8_t INT_DRDY_G = 0x02;
static constexpr uint8_t MASTER_ON = 0x04;
static constexpr uint8_t SFLP_GAME_ENABLE = 0x02;
static constexpr uint8_t FIFO_STREAM_MODE = 0x06;
static constexpr uint8_t SFLP_GAME_TAG = 0x13;

}  // namespace lsm6dsv
}  // namespace nimu

#endif
