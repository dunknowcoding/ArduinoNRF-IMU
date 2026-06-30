/*
  GY68.h - Board alias for GY-68 BMP180/BMP085 barometer modules.
*/
#ifndef ARDUINONRF_IMU_GY68_H
#define ARDUINONRF_IMU_GY68_H

#include "../../sensors/BMP180/BMP180.h"

namespace nimu {

class GY68 : public BMP180 {
 public:
  GY68() = default;
};

}  // namespace nimu

using nimu::GY68;

#endif  // ARDUINONRF_IMU_GY68_H
