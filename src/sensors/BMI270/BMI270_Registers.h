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
constexpr uint8_t SENSOR_TIME_0 = 0x18;
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
constexpr uint8_t PWR_CONF = 0x7C;
constexpr uint8_t PWR_CTRL = 0x7D;
constexpr uint8_t CMD = 0x7E;

constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t STATUS_DRDY_GYR = 0x40;
constexpr uint8_t STATUS_DRDY_ACC = 0x80;
constexpr uint8_t PWR_GYR_EN = 0x02;

// ERR_REG bit 0 is fatal_err: the part is not operable and only a power-on
// reset clears it.
constexpr uint8_t ERR_REG = 0x02;
constexpr uint8_t ERR_FATAL = 0x01;
constexpr uint8_t PWR_ACC_EN = 0x04;
// PWR_CTRL bit 3. Without it DATA_TEMP reads 0x8000, the "no reading" value,
// which this driver was then reporting as -41 C.
constexpr uint8_t PWR_TEMP_EN = 0x08;
constexpr uint8_t INTERNAL_STATUS_INIT_OK = 0x01;

constexpr uint8_t PERF_MODE = 0x80;
constexpr uint8_t INT_OUTPUT_ENABLE = 0x08;
constexpr uint8_t INT_OPEN_DRAIN = 0x04;
constexpr uint8_t INT_ACTIVE_HIGH = 0x02;


// ---------------------------------------------------------------------------
// Advanced features
//
// The BMI270's step counter, motion detectors and wrist gestures all run on
// its internal core, so none of them exist until a configuration image has
// been loaded - see BMI270_Config.h.
//
// They are not ordinary registers. FEATURES is a sixteen-byte window at 0x30
// onto one of eight pages, selected by FEAT_PAGE. Datasheet section 4.8.1:
// "Writes to a FEATURES register must be 16-bit word oriented, i.e. writes
// should start at an even address (2m) and the last byte written should be at
// an odd address (2n+1)". This driver reads the whole page, modifies it, and
// writes the whole page back, which satisfies that by construction.
//
// The page and offset of each feature are not in the datasheet. They come from
// the bmi270_feat_in[] and bmi270_feat_out[] tables in Bosch's own bmi270.c,
// and the enable masks from bmi2_defs.h.
constexpr uint8_t FEAT_PAGE = 0x2F;
constexpr uint8_t FEATURES = 0x30;
constexpr uint8_t FEATURES_SIZE = 16;

// Inputs: which page a feature's configuration lives on, and where in it.
constexpr uint8_t PAGE_ANY_MOTION = 1;
constexpr uint8_t OFF_ANY_MOTION = 0x0C;
constexpr uint8_t PAGE_AXIS_MAP = 1;
constexpr uint8_t OFF_AXIS_MAP = 0x04;
constexpr uint8_t PAGE_NO_MOTION = 2;
constexpr uint8_t OFF_NO_MOTION = 0x00;
constexpr uint8_t PAGE_SIG_MOTION = 2;
constexpr uint8_t OFF_SIG_MOTION = 0x04;
constexpr uint8_t PAGE_STEP_PARAMS = 3;
constexpr uint8_t OFF_STEP_PARAMS = 0x00;
constexpr uint8_t PAGE_STEP_CONF = 6;
constexpr uint8_t OFF_STEP_CONF = 0x02;
constexpr uint8_t PAGE_WRIST_GESTURE = 6;
constexpr uint8_t OFF_WRIST_GESTURE = 0x06;
constexpr uint8_t PAGE_WRIST_WEAR = 7;
constexpr uint8_t OFF_WRIST_WEAR = 0x00;

// Outputs, all on page 0.
constexpr uint8_t PAGE_FEATURE_OUT = 0;
constexpr uint8_t OFF_STEP_COUNT_OUT = 0x00;   // 32-bit, little endian
constexpr uint8_t OFF_STEP_ACT_OUT = 0x04;
constexpr uint8_t OFF_WRIST_GEST_OUT = 0x06;

// Enable bits, applied at (start address + this offset) within the page.
constexpr uint8_t EN_OFF_ANY_NO_MOTION = 0x03;
constexpr uint8_t EN_OFF_SIG_MOTION = 0x0A;
constexpr uint8_t EN_OFF_STEP = 0x01;
constexpr uint8_t EN_MASK_ANY_NO_MOTION = 0x80;
constexpr uint8_t EN_MASK_SIG_MOTION = 0x01;
constexpr uint8_t EN_MASK_STEP_DETECTOR = 0x08;
constexpr uint8_t EN_MASK_STEP_COUNTER = 0x10;
constexpr uint8_t EN_MASK_STEP_ACTIVITY = 0x20;
constexpr uint8_t EN_MASK_WRIST_GESTURE = 0x20;
constexpr uint8_t EN_MASK_WRIST_WEAR = 0x10;

// Any-motion and no-motion share a layout: a duration word then a threshold
// word. Duration is 13 bits at 20 ms per count; threshold is 11 bits in 5.11
// format, so one count is 1/2048 g.
constexpr uint16_t ANY_NO_MOT_DUR_MASK = 0x1FFF;
constexpr uint16_t ANY_NO_MOT_X_SEL = 0x2000;
constexpr uint16_t ANY_NO_MOT_Y_SEL = 0x4000;
constexpr uint16_t ANY_NO_MOT_Z_SEL = 0x8000;
constexpr uint16_t ANY_NO_MOT_THRES_MASK = 0x07FF;
constexpr uint16_t STEP_COUNT_RST_MASK = 0x0400;

// INT_STATUS_0 - one bit per feature, cleared when the register is read.
constexpr uint8_t INT_STATUS_0 = 0x1C;
constexpr uint8_t INT_STATUS_1 = 0x1D;
constexpr uint8_t INT1_MAP_FEAT = 0x56;
constexpr uint8_t INT2_MAP_FEAT = 0x57;
constexpr uint8_t FEAT_INT_SIG_MOTION = 0x01;
constexpr uint8_t FEAT_INT_STEP = 0x02;
constexpr uint8_t FEAT_INT_STEP_ACTIVITY = 0x04;
constexpr uint8_t FEAT_INT_WRIST_WEAR = 0x08;
constexpr uint8_t FEAT_INT_WRIST_GESTURE = 0x10;
constexpr uint8_t FEAT_INT_NO_MOTION = 0x20;
constexpr uint8_t FEAT_INT_ANY_MOTION = 0x40;

// FIFO
constexpr uint8_t FIFO_LENGTH_0 = 0x24;
constexpr uint8_t FIFO_DATA = 0x26;
constexpr uint8_t FIFO_DOWNS = 0x45;
constexpr uint8_t FIFO_WTM_0 = 0x46;
constexpr uint8_t FIFO_CONFIG_0 = 0x48;
constexpr uint8_t FIFO_CONFIG_1 = 0x49;
constexpr uint8_t FIFO_CONF1_ACC_EN = 0x40;
constexpr uint8_t FIFO_CONF1_GYR_EN = 0x80;
constexpr uint8_t FIFO_CONF1_HEADER_EN = 0x10;
constexpr uint8_t CMD_FIFO_FLUSH = 0xB0;

// Offsets and self-test
constexpr uint8_t ACC_SELF_TEST = 0x6D;
constexpr uint8_t NV_CONF = 0x70;
constexpr uint8_t NV_ACC_OFF_EN = 0x08;
constexpr uint8_t OFFSET_ACC_X = 0x71;
constexpr uint8_t OFFSET_GYR_X = 0x74;
constexpr uint8_t OFFSET_6 = 0x77;
constexpr uint8_t OFFSET6_GYR_OFF_EN = 0x40;
}  // namespace bmi270
}  // namespace nimu

#endif  // ARDUINONRF_IMU_BMI270_REGISTERS_H
