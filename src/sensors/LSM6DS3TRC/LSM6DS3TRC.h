/* LSM6DS3TRC.h - LSM6DS3TR-C compatibility driver. */
#ifndef ARDUINONRF_IMU_LSM6DS3TRC_H
#define ARDUINONRF_IMU_LSM6DS3TRC_H

#include "../LSM6DS3/LSM6DS3.h"

namespace nimu {

class LSM6DS3TRC : public LSM6DS3 {
 public:
  LSM6DS3TRC() { name_ = "LSM6DS3TR-C"; }
};

}  // namespace nimu

using nimu::LSM6DS3TRC;

#endif
