/*
  MMA8452Q_Registers.h - Register constants for NXP MMA8452Q.
*/
#ifndef ARDUINONRF_IMU_MMA8452Q_REGISTERS_H
#define ARDUINONRF_IMU_MMA8452Q_REGISTERS_H

#include <Arduino.h>

namespace nimu::mma8452q {

static constexpr uint8_t kAddrSA0High = 0x1D;
static constexpr uint8_t kAddrSA0Low = 0x1C;
static constexpr uint8_t kWhoAmI = 0x2A;

static constexpr uint8_t STATUS = 0x00;
static constexpr uint8_t OUT_X_MSB = 0x01;
static constexpr uint8_t WHO_AM_I = 0x0D;
static constexpr uint8_t XYZ_DATA_CFG = 0x0E;
static constexpr uint8_t CTRL_REG1 = 0x2A;
static constexpr uint8_t CTRL_REG3 = 0x2C;
static constexpr uint8_t CTRL_REG4 = 0x2D;
static constexpr uint8_t CTRL_REG5 = 0x2E;

static constexpr uint8_t STATUS_ZYXDR = 0x08;
static constexpr uint8_t CTRL_ACTIVE = 0x01;
static constexpr uint8_t CTRL_ODR_MASK = 0x38;
static constexpr uint8_t FS_MASK = 0x03;
static constexpr uint8_t INT_ACTIVE_HIGH = 0x02;
static constexpr uint8_t INT_OPEN_DRAIN = 0x01;
static constexpr uint8_t INT_DRDY = 0x01;

}  // namespace nimu::mma8452q

#endif  // ARDUINONRF_IMU_MMA8452Q_REGISTERS_H
