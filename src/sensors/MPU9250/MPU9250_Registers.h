/*
  MPU9250_Registers.h - AK8963 magnetometer register map for the MPU-9250.

  The MPU-9250 is two dies in one package:
    * an MPU-6500 accelerometer + gyroscope (all its registers live in
      MPU6500_Registers.h, included below), and
    * an AK8963 magnetometer (its own I2C address 0x0C), wired to the MPU's
      AUXILIARY I2C bus and reached through the MPU's internal I2C master.

  This driver talks to the AK8963 ONLY through that internal master (the SLV0/
  SLV4 registers in MPU6500_Registers.h), never through I2C bypass. Bypass
  shorts the AK8963's aux bus onto the main SDA/SCL; on clone GY-9250 boards
  with a dead/absent AK8963 that wedges the whole bus and takes accel+gyro down
  with it. The internal master keeps the AK8963 isolated, so a bad magnetometer
  can never jam the main bus.
*/
#ifndef NIUSIMU_MPU9250_REGISTERS_H
#define NIUSIMU_MPU9250_REGISTERS_H

#include <stdint.h>

#include "../MPU6500/MPU6500_Registers.h"  // shared MPU-6500 inertial registers

namespace nimu {
namespace mpu9250 {

// --- AK8963 magnetometer --------------------------------------------------
static constexpr uint8_t kMagAddr = 0x0C;
static constexpr uint8_t kMagWhoAmI = 0x48;  // AK8963 WIA value

static constexpr uint8_t MAG_WIA = 0x00;
static constexpr uint8_t MAG_ST1 = 0x02;    // bit0 DRDY
static constexpr uint8_t MAG_HXL = 0x03;    // 6 data bytes follow (XL..ZH)
static constexpr uint8_t MAG_ST2 = 0x09;    // bit3 HOFL; MUST be read to latch
static constexpr uint8_t MAG_CNTL1 = 0x0A;
static constexpr uint8_t MAG_CNTL2 = 0x0B;  // bit0 SRST soft reset
static constexpr uint8_t MAG_ASAX = 0x10;   // 3 fuse-ROM sensitivity bytes

static constexpr uint8_t MAG_ST1_DRDY = 0x01;
static constexpr uint8_t MAG_ST2_HOFL = 0x08;

// CNTL1 modes (low nibble) and resolution bit (bit4)
static constexpr uint8_t MAG_MODE_POWERDOWN = 0x00;
static constexpr uint8_t MAG_MODE_CONT_100HZ = 0x06;  // continuous mode 2
static constexpr uint8_t MAG_MODE_FUSE_ROM = 0x0F;
static constexpr uint8_t MAG_BIT_16 = 0x10;  // 16-bit output (vs 14-bit)

// AK8963 full scale is +/-4912 uT across a 16-bit signed output.
static constexpr float kMagScaleUT = 4912.0f / 32760.0f;  // ~0.15 uT/LSB

}  // namespace mpu9250
}  // namespace nimu

#endif  // NIUSIMU_MPU9250_REGISTERS_H
