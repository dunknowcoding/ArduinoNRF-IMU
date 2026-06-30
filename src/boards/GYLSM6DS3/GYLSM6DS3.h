/*
  GYLSM6DS3.h - Board alias for GY-LSM6DS3 marketplace breakouts.
*/
#ifndef ARDUINONRF_IMU_GYLSM6DS3_H
#define ARDUINONRF_IMU_GYLSM6DS3_H

#include "../../sensors/LSM6DS3/LSM6DS3.h"

namespace nimu {

/*
  Full breakout silk: INT1, INT2, OCS, SCX, SDX, +3.3V on one side;
  CS, SDO, SCL, SDA, +3.3V, GND on the other.
*/
class GYLSM6DS3 : public LSM6DS3 {
 public:
  GYLSM6DS3() { name_ = "GY-LSM6DS3"; }
};

}  // namespace nimu

using nimu::GYLSM6DS3;

#endif  // ARDUINONRF_IMU_GYLSM6DS3_H
