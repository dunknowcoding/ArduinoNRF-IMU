/*
  MPU9250_Registers.h - Register map for the InvenSense MPU-9250.

  The MPU-9250 is two dies in one package:
    * an MPU-6500 accelerometer + gyroscope (I2C address 0x68 / 0x69), and
    * an AK8963 magnetometer (its own I2C address 0x0C), reachable directly
      when the MPU's I2C bypass is enabled.

  Only the registers this driver actually uses are listed, each with a short
  note. Bit fields used in more than one place get a named constant.
*/
#ifndef NIUSIMU_MPU9250_REGISTERS_H
#define NIUSIMU_MPU9250_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace mpu9250 {

// --- I2C addresses --------------------------------------------------------
static constexpr uint8_t kAddrAD0Low = 0x68;   // AD0 pin low  (default)
static constexpr uint8_t kAddrAD0High = 0x69;  // AD0 pin high

// WHO_AM_I values seen on genuine and clone GY-9250 boards.
static constexpr uint8_t kWhoAmIMPU9250 = 0x71;
static constexpr uint8_t kWhoAmIMPU9255 = 0x73;
static constexpr uint8_t kWhoAmIMPU6500 = 0x70;  // clone with no magnetometer

// --- MPU-6500 register addresses -----------------------------------------
static constexpr uint8_t SMPLRT_DIV = 0x19;     // sample-rate divider
static constexpr uint8_t CONFIG = 0x1A;         // gyro/temp DLPF (bits[2:0])
static constexpr uint8_t GYRO_CONFIG = 0x1B;    // gyro full scale + Fchoice
static constexpr uint8_t ACCEL_CONFIG = 0x1C;   // accel full scale
static constexpr uint8_t ACCEL_CONFIG2 = 0x1D;  // accel DLPF (bits[2:0])
static constexpr uint8_t INT_PIN_CFG = 0x37;    // INT pin + I2C bypass
static constexpr uint8_t INT_ENABLE = 0x38;     // interrupt enables
static constexpr uint8_t INT_STATUS = 0x3A;     // interrupt status (read clears)
static constexpr uint8_t ACCEL_XOUT_H = 0x3B;   // start of 14-byte data burst
static constexpr uint8_t TEMP_OUT_H = 0x41;
static constexpr uint8_t GYRO_XOUT_H = 0x43;
static constexpr uint8_t USER_CTRL = 0x6A;      // FIFO/I2C-master/reset
static constexpr uint8_t PWR_MGMT_1 = 0x6B;     // reset / sleep / clock select
static constexpr uint8_t PWR_MGMT_2 = 0x6C;     // per-axis stand-by
static constexpr uint8_t WHO_AM_I = 0x75;

// PWR_MGMT_1 bits
static constexpr uint8_t PWR1_H_RESET = 0x80;   // full device reset
static constexpr uint8_t PWR1_SLEEP = 0x40;     // sleep mode
static constexpr uint8_t PWR1_CLKSEL_AUTO = 0x01;  // best available clock (PLL)

// INT_PIN_CFG bits
static constexpr uint8_t INTCFG_BYPASS_EN = 0x02;  // expose AK8963 on main I2C
static constexpr uint8_t INTCFG_LATCH_INT = 0x20;  // latch INT until cleared

// INT_ENABLE / INT_STATUS bits
static constexpr uint8_t INT_RAW_RDY = 0x01;    // raw data ready

// GYRO_CONFIG: full-scale select occupies bits[4:3]
static constexpr uint8_t GYRO_FS_250 = 0x00 << 3;
static constexpr uint8_t GYRO_FS_500 = 0x01 << 3;
static constexpr uint8_t GYRO_FS_1000 = 0x02 << 3;
static constexpr uint8_t GYRO_FS_2000 = 0x03 << 3;
static constexpr uint8_t GYRO_FS_MASK = 0x18;

// ACCEL_CONFIG: full-scale select occupies bits[4:3]
static constexpr uint8_t ACCEL_FS_2 = 0x00 << 3;
static constexpr uint8_t ACCEL_FS_4 = 0x01 << 3;
static constexpr uint8_t ACCEL_FS_8 = 0x02 << 3;
static constexpr uint8_t ACCEL_FS_16 = 0x03 << 3;
static constexpr uint8_t ACCEL_FS_MASK = 0x18;

// Temperature conversion (datasheet): degC = raw / 333.87 + 21.0
static constexpr float kTempSensitivity = 333.87f;
static constexpr float kTempOffsetC = 21.0f;

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
