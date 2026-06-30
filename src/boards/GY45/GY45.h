/*
  GY45.h - Board alias for GY-45 MMA8452Q accelerometer modules.
*/
#ifndef ARDUINONRF_IMU_GY45_H
#define ARDUINONRF_IMU_GY45_H

#include "../../sensors/MMA8452Q/MMA8452Q.h"

namespace nimu {

class GY45 : public MMA8452Q {
 public:
  GY45() { name_ = "GY45"; }
};

}  // namespace nimu

using nimu::GY45;

#endif  // ARDUINONRF_IMU_GY45_H
