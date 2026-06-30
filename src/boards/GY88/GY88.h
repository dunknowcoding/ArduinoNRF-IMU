/*
  GY88.h - Board alias for common GY-88 10-DOF marketplace modules.

  GY-88 listings usually describe an MPU-6050 + HMC5883L + BMP085 board. Many
  current modules use BMP180-compatible pressure parts and/or QMC5883L compass
  replacements, so this class reuses the GY87 composition path.
*/
#ifndef ARDUINONRF_IMU_GY88_H
#define ARDUINONRF_IMU_GY88_H

#include "../GY87/GY87.h"

namespace nimu {

class GY88 : public GY87 {
 public:
  GY88() = default;
};

}  // namespace nimu

using nimu::GY88;

#endif  // ARDUINONRF_IMU_GY88_H
