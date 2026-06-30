/*
  GYBNO055.h - Board alias for the eight-pad GY-BNO055 breakout.

  Exact silk: VIN, GND, SCL/RX, SDA/TX, ADO, INT, BOOT, REST.
*/
#ifndef ARDUINONRF_IMU_GYBNO055_H
#define ARDUINONRF_IMU_GYBNO055_H

#include "../../sensors/BNO055/BNO055.h"

namespace nimu {

class GYBNO055 : public BNO055 {
 public:
  GYBNO055() { name_ = "GY-BNO055"; }
};

}  // namespace nimu

using nimu::GYBNO055;

#endif  // ARDUINONRF_IMU_GYBNO055_H
