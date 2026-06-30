/*
  LSM303DLHC_Registers.h - Register constants for ST LSM303DLHC.
*/
#ifndef ARDUINONRF_IMU_LSM303DLHC_REGISTERS_H
#define ARDUINONRF_IMU_LSM303DLHC_REGISTERS_H

#include <Arduino.h>

namespace nimu::lsm303dlhc {

static constexpr uint8_t kAccelAddrHigh = 0x19;
static constexpr uint8_t kAccelAddrLow = 0x18;
static constexpr uint8_t kMagAddr = 0x1E;
static constexpr uint8_t kAccelWhoAmI = 0x33;

static constexpr uint8_t WHO_AM_I_A = 0x0F;
static constexpr uint8_t CTRL_REG1_A = 0x20;
static constexpr uint8_t CTRL_REG3_A = 0x22;
static constexpr uint8_t CTRL_REG4_A = 0x23;
static constexpr uint8_t CTRL_REG6_A = 0x25;
static constexpr uint8_t STATUS_REG_A = 0x27;
static constexpr uint8_t OUT_X_L_A = 0x28;

static constexpr uint8_t CRA_REG_M = 0x00;
static constexpr uint8_t CRB_REG_M = 0x01;
static constexpr uint8_t MR_REG_M = 0x02;
static constexpr uint8_t OUT_X_H_M = 0x03;
static constexpr uint8_t SR_REG_M = 0x09;
static constexpr uint8_t IRA_REG_M = 0x0A;
static constexpr uint8_t IRB_REG_M = 0x0B;
static constexpr uint8_t IRC_REG_M = 0x0C;
static constexpr uint8_t TEMP_OUT_H_M = 0x31;

static constexpr uint8_t CTRL1_AXES_ENABLE = 0x07;
static constexpr uint8_t CTRL4_BDU = 0x80;
static constexpr uint8_t CTRL4_HR = 0x08;
static constexpr uint8_t CTRL4_FS_MASK = 0x30;
static constexpr uint8_t STATUS_ZYXDA = 0x08;
static constexpr uint8_t MR_CONTINUOUS = 0x00;
static constexpr uint8_t SR_DRDY = 0x01;
static constexpr uint8_t I1_DATA_READY = 0x10;
static constexpr uint8_t INT_ACTIVE_LOW = 0x02;

}  // namespace nimu::lsm303dlhc

#endif  // ARDUINONRF_IMU_LSM303DLHC_REGISTERS_H
