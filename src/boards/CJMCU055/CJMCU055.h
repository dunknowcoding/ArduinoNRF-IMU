/* Board alias for the purple CJMCU-055 9DOF BMX055 breakout. */
#ifndef ARDUINONRF_IMU_CJMCU055_H
#define ARDUINONRF_IMU_CJMCU055_H

#include "../../sensors/BMX055/BMX055.h"

namespace nimu {

/*
  Exact silk: 3.3V, GND, SCL, SDA, PS, SDO1, SDO2, CSB1, CSB2, CSB3.
  CSB1/2/3 select the accelerometer, gyroscope, and magnetometer.
*/
class CJMCU055 : public BMX055 {
 public:
  CJMCU055() { name_ = "CJMCU-055"; }

  static void selectSPI(uint8_t psPin, uint8_t csb1Pin,
                        uint8_t csb2Pin, uint8_t csb3Pin) {
    pinMode(csb1Pin, OUTPUT);
    pinMode(csb2Pin, OUTPUT);
    pinMode(csb3Pin, OUTPUT);
    digitalWrite(csb1Pin, HIGH);
    digitalWrite(csb2Pin, HIGH);
    digitalWrite(csb3Pin, HIGH);
    pinMode(psPin, OUTPUT);
    digitalWrite(psPin, LOW);
  }

  static void selectI2C(uint8_t psPin) {
    pinMode(psPin, OUTPUT);
    digitalWrite(psPin, HIGH);
  }
};

}  // namespace nimu

using nimu::CJMCU055;

#endif
