/*
  GY63.h - Board alias for GY-63 MS5611 barometer modules.
*/
#ifndef ARDUINONRF_IMU_GY63_H
#define ARDUINONRF_IMU_GY63_H

#include "../../sensors/MS5611/MS5611.h"

namespace nimu {

class GY63 : public MS5611 {
 public:
  GY63() = default;

  static void selectI2C(uint8_t psPin) {
    pinMode(psPin, OUTPUT);
    digitalWrite(psPin, HIGH);
  }

  static void selectSPI(uint8_t psPin) {
    pinMode(psPin, OUTPUT);
    digitalWrite(psPin, LOW);
  }
};

}  // namespace nimu

using nimu::GY63;

#endif  // ARDUINONRF_IMU_GY63_H
