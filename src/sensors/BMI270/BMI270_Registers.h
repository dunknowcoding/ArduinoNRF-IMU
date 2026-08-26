/*
  BMI270_Registers.h - Register constants for Bosch BMI270.
*/
#ifndef ARDUINONRF_IMU_BMI270_REGISTERS_H
#define ARDUINONRF_IMU_BMI270_REGISTERS_H

#include <Arduino.h>

namespace nimu {
namespace bmi270 {

constexpr uint8_t kAddrSDOLow = 0x68;
constexpr uint8_t kAddrSDOHigh = 0x69;
constexpr uint8_t kChipId = 0x24;

constexpr uint8_t CHIP_ID = 0x00;
constexpr uint8_t STATUS = 0x03;
constexpr uint8_t DATA_ACCEL_X_L = 0x0C;
constexpr uint8_t DATA_GYRO_X_L = 0x12;
constexpr uint8_t INTERNAL_STATUS = 0x21;
constexpr uint8_t DATA_TEMP_L = 0x22;
constexpr uint8_t ACC_CONF = 0x40;
constexpr uint8_t ACC_RANGE = 0x41;
constexpr uint8_t GYR_CONF = 0x42;
constexpr uint8_t GYR_RANGE = 0x43;
constexpr uint8_t INT1_IO_CTRL = 0x53;
constexpr uint8_t INT2_IO_CTRL = 0x54;
constexpr uint8_t INT_LATCH = 0x55;
constexpr uint8_t INT_MAP_DATA = 0x58;
constexpr uint8_t INIT_CTRL = 0x59;
constexpr uint8_t INIT_ADDR_0 = 0x5B;
constexpr uint8_t INIT_ADDR_1 = 0x5C;
constexpr uint8_t INIT_DATA = 0x5E;
// NV_CONF is deliberately not defined here, and this driver never writes it.
//
// Bit 0 is spi_en. Setting it disables the I2C interface until the next power
// cycle - a soft reset will not bring it back - so a single stray write to
// 0x70 bricks the part for the rest of the session. Bosch issue #26 on
// BMI270_SensorAPI is exactly this: "BMI270 Initialization Causes I2C NACK
// After Writing to Register 0x70". The other bits there are the I2C watchdog
// and the accelerometer offset enable, none of which this driver needs.
//
// If you ever add NV_CONF support, read the register first and preserve
// spi_en, and never burst-write across 0x70.

constexpr uint8_t PWR_CONF = 0x7C;
constexpr uint8_t PWR_CTRL = 0x7D;
constexpr uint8_t CMD = 0x7E;

constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t STATUS_DRDY_GYR = 0x40;
constexpr uint8_t STATUS_DRDY_ACC = 0x80;
constexpr uint8_t PWR_GYR_EN = 0x02;

// ERR_REG bit 0 is fatal_err: the part is not operable and only a power-on
// reset clears it. EVENT bit 0 is por_detected, set by a power-on reset and
// cleared when the register is read - which makes it the one way to find out,
// after the event, that the supply dipped rather than something logical
// having gone wrong.
constexpr uint8_t ERR_REG = 0x02;
constexpr uint8_t ERR_FATAL = 0x01;
constexpr uint8_t EVENT = 0x1B;
constexpr uint8_t EVENT_POR_DETECTED = 0x01;
constexpr uint8_t PWR_ACC_EN = 0x04;
// PWR_CTRL bit 3. Without it DATA_TEMP reads 0x8000, the "no reading" value,
// which this driver was then reporting as -41 C.
constexpr uint8_t PWR_TEMP_EN = 0x08;
constexpr uint8_t INTERNAL_STATUS_INIT_OK = 0x01;

constexpr uint8_t PERF_MODE = 0x80;
constexpr uint8_t INT_OUTPUT_ENABLE = 0x08;
constexpr uint8_t INT_OPEN_DRAIN = 0x04;
constexpr uint8_t INT_ACTIVE_HIGH = 0x02;

}  // namespace bmi270
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMI270_REGISTERS_H
