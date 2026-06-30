/* LSM303_L3GD20.h - Adafruit-style 9DOF board and marketplace clones. */
#ifndef ARDUINONRF_IMU_LSM303_L3GD20_BOARD_H
#define ARDUINONRF_IMU_LSM303_L3GD20_BOARD_H

#include "../../sensors/L3GD20/L3GD20.h"
#include "../../sensors/LSM303DLHC/LSM303DLHC.h"

namespace nimu {

class LSM303_L3GD20 {
 public:
  bool begin(TwoWire& wire = Wire) {
    motionOk_ = motion_.beginI2C(wire, lsm303dlhc::kAccelAddrHigh,
                                  lsm303dlhc::kMagAddr);
    gyroOk_ = gyro_.beginI2C(wire, L3GD20::kAddrHigh);
    if (!gyroOk_) gyroOk_ = gyro_.beginI2C(wire, L3GD20::kAddrLow);
    return motionOk_ && gyroOk_;
  }

  bool update() {
    return motionOk_ && gyroOk_ && motion_.update() && gyro_.update();
  }
  bool motionOk() const { return motionOk_; }
  bool gyroOk() const { return gyroOk_; }
  LSM303DLHC& motion() { return motion_; }
  L3GD20& gyro() { return gyro_; }
  Vec3 accelG() const { return motion_.accelG(); }
  Vec3 gyroDps() const { return gyro_.gyroDps(); }
  Vec3 magUT() const { return motion_.magUT(); }

 private:
  LSM303DLHC motion_;
  L3GD20 gyro_;
  bool motionOk_ = false;
  bool gyroOk_ = false;
};

using Adafruit9DOFClone = LSM303_L3GD20;

}  // namespace nimu

using nimu::Adafruit9DOFClone;
using nimu::LSM303_L3GD20;

#endif
