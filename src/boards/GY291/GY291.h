/*
  GY291.h - Board alias for GY-291 ADXL345 accelerometer modules.
*/
#ifndef ARDUINONRF_IMU_GY291_H
#define ARDUINONRF_IMU_GY291_H

#include "../../sensors/ADXL345/ADXL345.h"

namespace nimu {

class GY291 : public ADXL345 {
 public:
  GY291() { name_ = "GY291"; }
};

}  // namespace nimu

using nimu::GY291;

#endif  // ARDUINONRF_IMU_GY291_H
