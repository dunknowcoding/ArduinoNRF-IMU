/*
  ICM42688P_Registers.h - Register constants for TDK InvenSense ICM-42688-P.
*/
#ifndef ARDUINONRF_IMU_ICM42688P_REGISTERS_H
#define ARDUINONRF_IMU_ICM42688P_REGISTERS_H

#include <Arduino.h>

namespace nimu {
namespace icm42688p {

constexpr uint8_t kAddrAD0Low = 0x68;
constexpr uint8_t kAddrAD0High = 0x69;
constexpr uint8_t kWhoAmI = 0x47;

constexpr uint8_t DEVICE_CONFIG = 0x11;
constexpr uint8_t INT_CONFIG = 0x14;
constexpr uint8_t TEMP_DATA1 = 0x1D;
constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t GYRO_DATA_X1 = 0x25;
constexpr uint8_t INT_STATUS = 0x2D;
constexpr uint8_t INTF_CONFIG0 = 0x4C;
constexpr uint8_t PWR_MGMT0 = 0x4E;
constexpr uint8_t GYRO_CONFIG0 = 0x4F;
constexpr uint8_t ACCEL_CONFIG0 = 0x50;
constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
constexpr uint8_t INT_CONFIG1 = 0x64;
constexpr uint8_t INT_SOURCE0 = 0x65;
constexpr uint8_t INT_SOURCE3 = 0x68;
constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t REG_BANK_SEL = 0x76;
constexpr uint8_t INTF_CONFIG5_B1 = 0x7B;

constexpr uint8_t DEVICE_SOFT_RESET = 0x01;
constexpr uint8_t INT_STATUS_DATA_READY = 0x08;
constexpr uint8_t PWR_GYRO_LN = 0x0C;
constexpr uint8_t PWR_ACCEL_LN = 0x03;
constexpr uint8_t INT_UI_FSYNC = 0x40;
constexpr uint8_t INT_UI_DRDY = 0x08;

}  // namespace icm42688p
}  // namespace nimu

#endif  // ARDUINONRF_IMU_ICM42688P_REGISTERS_H
