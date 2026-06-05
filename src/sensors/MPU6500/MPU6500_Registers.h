/*
  MPU6500_Registers.h - Register map for the InvenSense MPU-6500 6-axis
  accelerometer + gyroscope.

  The MPU-6500 is also the "inertial half" of the MPU-9250 (which adds an AK8963
  magnetometer in the same package) and the part fitted to many GY-91 boards. So
  this header holds every accel/gyro/temperature/clock register the family
  shares; the MPU-9250 driver includes it and adds only the AK8963 constants.

  The internal I2C-master registers live here too: the master is MPU-6500
  silicon (it drives the chip's auxiliary bus). The MPU-6500 itself never uses
  it, but the MPU-9250 reaches its magnetometer through it.

  Only the registers the drivers actually use are listed, each with a short
  note. Bit fields used in more than one place get a named constant.
*/
#ifndef NIUSIMU_MPU6500_REGISTERS_H
#define NIUSIMU_MPU6500_REGISTERS_H

#include <stdint.h>

namespace nimu {
namespace mpu6500 {

// --- I2C addresses --------------------------------------------------------
static constexpr uint8_t kAddrAD0Low = 0x68;   // AD0 pin low  (default)
static constexpr uint8_t kAddrAD0High = 0x69;  // AD0 pin high

// WHO_AM_I values across the family. The 6-axis driver accepts all of them
// (an MPU-9250 works perfectly as a 6-axis part); the 9-axis driver simply
// finds no magnetometer on a plain 0x70 MPU-6500.
static constexpr uint8_t kWhoAmIMPU9250 = 0x71;
static constexpr uint8_t kWhoAmIMPU9255 = 0x73;
static constexpr uint8_t kWhoAmIMPU6500 = 0x70;
static constexpr uint8_t kWhoAmIMPU6050 = 0x68;

// --- register addresses ---------------------------------------------------
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
// Internal I2C master: drives the auxiliary bus (used by the MPU-9250's mag).
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
static constexpr uint8_t INTCFG_BYPASS_EN = 0x02;  // expose aux bus on main I2C
static constexpr uint8_t INTCFG_LATCH_INT = 0x20;  // latch INT until cleared

// USER_CTRL bits
static constexpr uint8_t USERCTRL_I2C_MST_EN = 0x20;   // run the internal master
static constexpr uint8_t USERCTRL_I2C_MST_RST = 0x02;  // reset the master logic

// I2C_MST_CTRL: 0x0D selects the 400 kHz aux-bus clock; WAIT_FOR_ES delays
// data-ready until the external read finishes so frames stay coherent.
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

}  // namespace mpu6500
}  // namespace nimu

#endif  // NIUSIMU_MPU6500_REGISTERS_H
