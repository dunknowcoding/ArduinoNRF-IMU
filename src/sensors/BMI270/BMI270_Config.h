#ifndef ARDUINONRF_IMU_BMI270_CONFIG_H
#define ARDUINONRF_IMU_BMI270_CONFIG_H

// The BMI270 configuration image is NOT bundled with this library.
//
// It used to be. The array that lived here was 328 bytes, which is exactly
// Bosch's bmi270_maximum_fifo_config_file - a real image for one of the
// BMI270 builds, carried without their copyright notice or licence. It was
// removed for that reason rather than because it was a stub.
//
// A BMI270 has an on-chip core that must be loaded with an 8192-byte image
// before the part does anything at all. Measured on real silicon with no image
// loaded: CHIP_ID reads 0x24, INTERNAL_STATUS reads not_init, the acc_rdy and
// gyr_rdy bits never set, and every data register stays at exactly zero. There
// is no subset of the sensor that works without it - not even raw
// acceleration.
//
// That image is Bosch Sensortec's, distributed with their BMI270 API under
// BSD-3-Clause. This library does not redistribute other vendors' firmware, so
// you supply it:
//
//   1. Take a config array from https://github.com/boschsensortec/BMI270_SensorAPI
//      There are several, and they are not interchangeable - each matches a
//      particular BMI270 build:
//        bmi270.c               bmi270_config_file[]              8192 bytes
//        bmi270_legacy.c        bmi270_legacy_config_file[]       8192 bytes
//        bmi270_context.c       bmi270_context_config_file[]      8192 bytes
//        bmi270_maximum_fifo.c  bmi270_maximum_fifo_config_file[]  328 bytes
//      Start with bmi270.c; if the chip reports init_err, try another.
//   2. Keep Bosch's copyright notice and licence with it, as BSD-3-Clause
//      requires.
//   3. Hand it to the driver before begin():
//
//        #include "bmi270_config_file.h"   // your copy
//        BMI270 imu;
//        imu.setConfigImage(bmi270_config_file, sizeof(bmi270_config_file));
//        imu.begin();
//
// Without that call begin() returns false and lastStageText() says so rather
// than leaving you to wonder whether the sensor is dead.

#endif  // ARDUINONRF_IMU_BMI270_CONFIG_H
