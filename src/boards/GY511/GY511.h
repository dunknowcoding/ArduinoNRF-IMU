/*
  GY511.h - Board alias for the GY-511 LSM303DLHC eCompass module.

  Exact visible silk:
  VIN, 3.3V, GND, SCL, SDA, INT2, INT1, DRDY.
*/
#ifndef ARDUINONRF_IMU_GY511_H
#define ARDUINONRF_IMU_GY511_H

#include "../../sensors/LSM303DLHC/LSM303DLHC.h"

namespace nimu {

class GY511 : public LSM303DLHC {
 public:
  GY511() { name_ = "GY511"; }
};

}  // namespace nimu

using nimu::GY511;

#endif  // ARDUINONRF_IMU_GY511_H
