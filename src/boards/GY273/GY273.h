/*
  GY273.h - Board alias for GY-273 HMC5883L/QMC5883L compass modules.
*/
#ifndef ARDUINONRF_IMU_GY273_H
#define ARDUINONRF_IMU_GY273_H

#include "../GY271/GY271.h"

namespace nimu {

class GY273 : public GY271 {
 public:
  GY273() = default;
};

}  // namespace nimu

using nimu::GY273;

#endif  // ARDUINONRF_IMU_GY273_H
