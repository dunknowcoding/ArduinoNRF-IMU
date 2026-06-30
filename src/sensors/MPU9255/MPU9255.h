/*
  MPU9255.h - MPU-9255 alias on top of the MPU-9250 family driver.

  MPU-9255 is register-compatible with the MPU-9250 for this driver's accel,
  gyro and AK8963 magnetometer path; the main software difference is WHO_AM_I
  returning 0x73 instead of the MPU-9250's 0x71.
*/
#ifndef ARDUINONRF_IMU_MPU9255_H
#define ARDUINONRF_IMU_MPU9255_H

#include "../MPU9250/MPU9250.h"

namespace nimu {

class MPU9255 : public MPU9250 {
 public:
  MPU9255() { name_ = "MPU9255"; }
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MPU9255;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_MPU9255_H
