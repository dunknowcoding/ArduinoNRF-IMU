/*
  QMC5883L.h - QST QMC5883L 3-axis magnetometer driver.
*/
#ifndef ARDUINONRF_IMU_QMC5883L_H
#define ARDUINONRF_IMU_QMC5883L_H

#include <Arduino.h>
#include <Wire.h>

#include "../../imu/IMUBus.h"
#include "../../imu/IMUTypes.h"

namespace nimu {

class QMC5883L {
 public:
  static constexpr uint8_t kAddr = 0x0D;

  bool begin(uint8_t address = kAddr, TwoWire& wire = Wire);
  bool isConnected();
  bool update();
  bool dataReady();
  bool dataOverflow();

  const Vec3& magUT() const { return mag_; }
  const char* name() const { return "QMC5883L"; }

 private:
  IMUBus bus_;
  Vec3 mag_{};
};

}  // namespace nimu

using nimu::QMC5883L;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_QMC5883L_H
