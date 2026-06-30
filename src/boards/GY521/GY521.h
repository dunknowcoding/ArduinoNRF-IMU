/*
  GY521.h - Board alias for the common GY-521 MPU-6050 breakout.

  Typical GY-521 pins:
    VCC, GND, SCL, SDA, XDA, XCL, AD0, INT

  This board class is the MPU6050 driver under the marketplace module name.
*/
#ifndef ARDUINONRF_IMU_GY521_H
#define ARDUINONRF_IMU_GY521_H

#include "../../sensors/MPU6050/MPU6050.h"

namespace nimu {

class GY521 : public MPU6050 {
 public:
  GY521() { name_ = "GY-521"; }
};

}  // namespace nimu

using nimu::GY521;

#endif  // ARDUINONRF_IMU_GY521_H
