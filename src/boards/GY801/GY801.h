/*
  GY801.h - Board alias for GY-801 modules, a common GY-80 variant.
*/
#ifndef ARDUINONRF_IMU_GY801_H
#define ARDUINONRF_IMU_GY801_H

#include "../GY80/GY80.h"

namespace nimu {

class GY801 : public GY80 {
 public:
  GY801() = default;
};

}  // namespace nimu

using nimu::GY801;

#endif  // ARDUINONRF_IMU_GY801_H
