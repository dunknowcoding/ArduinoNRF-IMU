#ifndef ARDUINONRF_IMU_LSM6DSV320X_REGISTERS_H
#define ARDUINONRF_IMU_LSM6DSV320X_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace lsm6dsv320x {

static constexpr uint8_t kWhoAmI = 0x73;
static constexpr uint8_t CTRL5 = 0x14;
static constexpr uint8_t CTRL6 = 0x15;
static constexpr uint8_t CTRL7 = 0x16;
static constexpr uint8_t CTRL1_XL_HG = 0x4E;
static constexpr uint8_t HG_WAKE_UP_SRC = 0x4C;
static constexpr uint8_t HG_FUNCTIONS_ENABLE = 0x52;
static constexpr uint8_t HG_WAKE_UP_THS = 0x53;
static constexpr uint8_t OUTX_L_A_HG = 0x34;

static constexpr uint8_t EMB_FUNC_EN_B = 0x05;
static constexpr uint8_t FSM_ENABLE = 0x46;
static constexpr uint8_t FSM_OUTS1 = 0x4C;
static constexpr uint8_t MLC1_SRC = 0x70;

static constexpr uint8_t I3C_INTERRUPT_ENABLE = 0x01;
static constexpr uint8_t ASC_FSM_WRITE_CTRL = 0x08;
static constexpr uint8_t FSM_CORE_ENABLE = 0x01;
static constexpr uint8_t MLC_CORE_ENABLE = 0x10;
static constexpr uint8_t HG_INT1_DRDY = 0x80;
static constexpr uint8_t HG_INT2_DRDY = 0x40;
static constexpr uint8_t HG_INTERRUPTS_ENABLE = 0x80;
static constexpr uint8_t HG_INT1_WAKE = 0x10;
static constexpr uint8_t HG_INT2_WAKE = 0x20;
static constexpr uint8_t HG_WAKE_EVENT = 0x08;
static constexpr uint8_t HG_SHOCK_STATE = 0x20;

}  // namespace lsm6dsv320x
}  // namespace nimu

#endif
