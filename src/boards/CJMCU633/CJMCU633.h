/* Alias for the purple CJMCU-633 / CJMCU-603F LSM6DS3 breakout. */
#ifndef ARDUINONRF_IMU_CJMCU633_H
#define ARDUINONRF_IMU_CJMCU633_H

#include "../GYLSM6DS3/GYLSM6DS3.h"

namespace nimu {

class CJMCU633 : public GYLSM6DS3 {
 public:
  CJMCU633() { name_ = "CJMCU-633 LSM6DS3"; }
};

}  // namespace nimu

using nimu::CJMCU633;

#endif
