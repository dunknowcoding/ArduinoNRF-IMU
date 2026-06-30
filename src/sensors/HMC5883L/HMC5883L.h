/*
  HMC5883L.h - Honeywell HMC5883L 3-axis magnetometer driver.
*/
#ifndef ARDUINONRF_IMU_HMC5883L_H
#define ARDUINONRF_IMU_HMC5883L_H

#include <Arduino.h>
#include <Wire.h>

#include "../../imu/IMUBus.h"
#include "../../imu/IMUTypes.h"

namespace nimu {

class HMC5883L {
 public:
  static constexpr uint8_t kAddr = 0x1E;

  bool begin(uint8_t address = kAddr, TwoWire& wire = Wire);
  bool isConnected();
  bool update();
  bool dataReady();
  bool dataLocked();

  const Vec3& magUT() const { return mag_; }
  float temperatureC() const { return 0.0f; }
  const char* name() const { return "HMC5883L"; }

  bool setContinuousMode();
  bool setGainGauss(float gauss);

 private:
  IMUBus bus_;
  Vec3 mag_{};
  float lsbPerGauss_ = 1090.0f;
};

}  // namespace nimu

using nimu::HMC5883L;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_HMC5883L_H
