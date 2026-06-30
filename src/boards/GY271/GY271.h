/*
  GY271.h - Board class for GY-271 compass modules.

  GY-271 boards are sold with both genuine HMC5883L parts and QMC5883L clone or
  replacement parts. This wrapper probes both common addresses and exposes one
  small compass-facing API.
*/
#ifndef ARDUINONRF_IMU_GY271_H
#define ARDUINONRF_IMU_GY271_H

#include "../../sensors/HMC5883L/HMC5883L.h"
#include "../../sensors/QMC5883L/QMC5883L.h"

namespace nimu {

class GY271 {
 public:
  enum class CompassKind : uint8_t { None, HMC5883L, QMC5883L };

  bool begin(TwoWire& wire = Wire) {
    compassKind_ = CompassKind::None;
    if (hmc_.begin(HMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::HMC5883L;
      return true;
    }
    if (qmc_.begin(QMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::QMC5883L;
      return true;
    }
    return false;
  }

  bool isConnected() {
    if (compassKind_ == CompassKind::HMC5883L) return hmc_.isConnected();
    if (compassKind_ == CompassKind::QMC5883L) return qmc_.isConnected();
    return false;
  }

  bool update() {
    if (compassKind_ == CompassKind::HMC5883L) return hmc_.update();
    if (compassKind_ == CompassKind::QMC5883L) return qmc_.update();
    return false;
  }

  bool hasMagnetometer() const { return compassKind_ != CompassKind::None; }
  CompassKind compassKind() const { return compassKind_; }
  const char* compassName() const {
    switch (compassKind_) {
      case CompassKind::HMC5883L: return "HMC5883L";
      case CompassKind::QMC5883L: return "QMC5883L";
      default: return "none";
    }
  }

  Vec3 magUT() const {
    if (compassKind_ == CompassKind::HMC5883L) return hmc_.magUT();
    if (compassKind_ == CompassKind::QMC5883L) return qmc_.magUT();
    return Vec3{};
  }

  HMC5883L& hmc() { return hmc_; }
  QMC5883L& qmc() { return qmc_; }

 private:
  HMC5883L hmc_;
  QMC5883L qmc_;
  CompassKind compassKind_ = CompassKind::None;
};

}  // namespace nimu

using nimu::GY271;

#endif  // ARDUINONRF_IMU_GY271_H
