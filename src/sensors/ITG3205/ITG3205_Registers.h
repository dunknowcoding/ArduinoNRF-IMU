/*
  ITG3205_Registers.h - Register constants for InvenSense ITG-3200/ITG-3205.
*/
#ifndef ARDUINONRF_IMU_ITG3205_REGISTERS_H
#define ARDUINONRF_IMU_ITG3205_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace itg3205 {

static constexpr uint8_t kAddrAD0Low = 0x68;
static constexpr uint8_t kAddrAD0High = 0x69;
static constexpr uint8_t kWhoAmIMasked = 0x68;

static constexpr uint8_t WHO_AM_I = 0x00;
static constexpr uint8_t SMPLRT_DIV = 0x15;
static constexpr uint8_t DLPF_FS = 0x16;
static constexpr uint8_t INT_CFG = 0x17;
static constexpr uint8_t INT_STATUS = 0x1A;
static constexpr uint8_t TEMP_OUT_H = 0x1B;
static constexpr uint8_t GYRO_XOUT_H = 0x1D;
static constexpr uint8_t PWR_MGM = 0x3E;

static constexpr uint8_t FS_SEL_2000 = 0x18;
static constexpr uint8_t RAW_RDY_EN = 0x01;
static constexpr uint8_t PLL_RDY_EN = 0x04;
static constexpr uint8_t INT_ACTIVE_LOW = 0x80;
static constexpr uint8_t INT_OPEN_DRAIN = 0x40;
static constexpr uint8_t INT_LATCH = 0x20;
static constexpr uint8_t INT_CLEAR_ANY_READ = 0x10;
static constexpr uint8_t RAW_DATA_RDY = 0x01;
static constexpr uint8_t H_RESET = 0x80;
static constexpr uint8_t CLK_SEL_X_GYRO = 0x01;

}  // namespace itg3205
}  // namespace nimu

#endif  // ARDUINONRF_IMU_ITG3205_REGISTERS_H
