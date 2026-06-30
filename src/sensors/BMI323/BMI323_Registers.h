/* Register constants for the Bosch BMI323 word-addressed interface. */
#ifndef ARDUINONRF_IMU_BMI323_REGISTERS_H
#define ARDUINONRF_IMU_BMI323_REGISTERS_H

#include <Arduino.h>

namespace nimu {
namespace bmi323 {

static constexpr uint8_t kAddrLow = 0x68;
static constexpr uint8_t kAddrHigh = 0x69;
static constexpr uint8_t kChipId = 0x43;
static constexpr uint8_t CHIP_ID = 0x00;
static constexpr uint8_t STATUS = 0x02;
static constexpr uint8_t ACC_DATA_X = 0x03;
static constexpr uint8_t TEMP_DATA = 0x09;
static constexpr uint8_t ACC_CONF = 0x20;
static constexpr uint8_t GYR_CONF = 0x21;
static constexpr uint8_t IO_INT_CTRL = 0x38;
static constexpr uint8_t INT_CONF = 0x39;
static constexpr uint8_t INT_MAP2 = 0x3B;
static constexpr uint8_t FEATURE_CTRL = 0x40;
static constexpr uint8_t FEATURE_DATA_ADDR = 0x41;
static constexpr uint8_t FEATURE_DATA_TX = 0x42;
static constexpr uint8_t FEATURE_DATA_STATUS = 0x43;
static constexpr uint8_t CMD = 0x7E;
static constexpr uint16_t SOFT_RESET = 0xDEAF;

}  // namespace bmi323
}  // namespace nimu

#endif
