/* QMC6309.h - QST QMC6309 3-axis magnetometer driver. */
#ifndef ARDUINONRF_IMU_QMC6309_H
#define ARDUINONRF_IMU_QMC6309_H

#include <Arduino.h>
#include <Wire.h>

#include "../../imu/IMUBus.h"
#include "../../imu/IMUTypes.h"

namespace nimu {

class QMC6309 {
 public:
  static constexpr uint8_t kAddr = 0x3E;

  bool begin(uint8_t address = kAddr, TwoWire& wire = Wire);
  uint8_t whoAmI();
  bool isConnected();
  bool update();
  bool dataReady();
  bool reset();

  const Vec3& magUT() const { return mag_; }
  const char* name() const { return "QMC6309"; }

 private:
  IMUBus bus_;
  Vec3 mag_{};
};

}  // namespace nimu

using nimu::QMC6309;
using nimu::Vec3;

#endif
