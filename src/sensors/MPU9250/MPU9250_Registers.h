/*
  MPU9250_Registers.h - Register map for the InvenSense MPU-9250.

  The MPU-9250 is two dies in one package:
    * an MPU-6500 accelerometer + gyroscope (I2C address 0x68 / 0x69), and
    * an AK8963 magnetometer (its own I2C address 0x0C), wired to the MPU's
      AUXILIARY I2C bus and reached through the MPU's internal I2C master.

  This driver talks to the AK8963 ONLY through that internal master (the SLV0/
  SLV4 registers below), never through I2C bypass. Bypass shorts the AK8963's
  aux bus onto the main SDA/SCL; on clone GY-9250 boards with a dead/absent
  AK8963 that wedges the whole bus and takes accel+gyro down with it. The
  internal master keeps the AK8963 isolated, so a bad magnetometer can never
  jam the main bus.

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
// Internal I2C master: drives the auxiliary bus (where the AK8963 lives).
static constexpr uint8_t I2C_MST_CTRL = 0x24;   // aux-bus clock + options
static constexpr uint8_t I2C_SLV0_ADDR = 0x25;  // SLV0: continuous mag reads
static constexpr uint8_t I2C_SLV0_REG = 0x26;
static constexpr uint8_t I2C_SLV0_CTRL = 0x27;
static constexpr uint8_t I2C_SLV4_ADDR = 0x31;  // SLV4: single config reads/writes
static constexpr uint8_t I2C_SLV4_REG = 0x32;
static constexpr uint8_t I2C_SLV4_DO = 0x33;
static constexpr uint8_t I2C_SLV4_CTRL = 0x34;
static constexpr uint8_t I2C_SLV4_DI = 0x35;
static constexpr uint8_t I2C_MST_STATUS = 0x36;
static constexpr uint8_t EXT_SENS_DATA_00 = 0x49;  // SLV0 read results land here
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

// USER_CTRL bits
static constexpr uint8_t USERCTRL_I2C_MST_EN = 0x20;   // run the internal master
static constexpr uint8_t USERCTRL_I2C_MST_RST = 0x02;  // reset the master logic

// I2C_MST_CTRL: 0x0D selects the 400 kHz aux-bus clock; WAIT_FOR_ES delays
// data-ready until the external (mag) read finishes so frames stay coherent.
static constexpr uint8_t I2C_MST_CLK_400 = 0x0D;
static constexpr uint8_t I2C_MST_WAIT_FOR_ES = 0x40;

// I2C_SLVx_CTRL bits (low nibble of SLV0 is the read length)
static constexpr uint8_t I2C_SLV_EN = 0x80;     // enable this slave slot
static constexpr uint8_t I2C_READ_FLAG = 0x80;  // OR into SLVx_ADDR to read

// I2C_MST_STATUS bits
static constexpr uint8_t I2C_MST_SLV4_DONE = 0x40;  // SLV4 single transfer done
static constexpr uint8_t I2C_MST_SLV4_NACK = 0x10;  // SLV4 transfer was NACKed

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
