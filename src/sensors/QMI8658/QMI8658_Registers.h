/*
  QMI8658_Registers.h - Register map for QST QMI8658/QMI8658C 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_QMI8658_REGISTERS_H
#define ARDUINONRF_IMU_QMI8658_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace qmi8658 {

static constexpr uint8_t kAddrLow = 0x6A;
static constexpr uint8_t kAddrHigh = 0x6B;
static constexpr uint8_t kWhoAmI = 0x05;

static constexpr uint8_t WHO_AM_I = 0x00;
static constexpr uint8_t CTRL1 = 0x02;
static constexpr uint8_t CTRL2 = 0x03;  // accel range + ODR
static constexpr uint8_t CTRL3 = 0x04;  // gyro range + ODR
static constexpr uint8_t CTRL5 = 0x06;
static constexpr uint8_t CTRL7 = 0x08;  // sensor enables
static constexpr uint8_t STATUS_INT = 0x2D;
static constexpr uint8_t STATUS0 = 0x2E;
static constexpr uint8_t TEMP_L = 0x33;
static constexpr uint8_t AX_L = 0x35;
static constexpr uint8_t GX_L = 0x3B;

static constexpr uint8_t CTRL1_SPI_AI = 0x40;  // auto-increment addresses
static constexpr uint8_t ENABLE_ACCEL = 0x01;
static constexpr uint8_t ENABLE_GYRO = 0x02;
static constexpr uint8_t STATUS_DATA_READY = 0x03;
static constexpr uint8_t CTRL7_SYNC_SAMPLE = 0x80;

}  // namespace qmi8658
}  // namespace nimu

#endif  // ARDUINONRF_IMU_QMI8658_REGISTERS_H
