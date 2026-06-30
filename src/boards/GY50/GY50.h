/*
  GY50.h - Board alias for GY-50 L3G4200D gyroscope modules.
*/
#ifndef ARDUINONRF_IMU_GY50_H
#define ARDUINONRF_IMU_GY50_H

#include "../../sensors/L3G4200D/L3G4200D.h"

namespace nimu {

class GY50 : public L3G4200D {
 public:
  GY50() { name_ = "GY50"; }
};

}  // namespace nimu

using nimu::GY50;

#endif  // ARDUINONRF_IMU_GY50_H
