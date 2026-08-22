#ifndef ARDUINONRF_IMU_BMI270_CONFIG_H
#define ARDUINONRF_IMU_BMI270_CONFIG_H

// The BMI270 configuration image is NOT bundled with this library.
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
//   1. Take bmi270_config_file[] from bmi270.c in
//      https://github.com/boschsensortec/BMI270_SensorAPI
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
