/*
  GY85.h - Board class for common GY-85 9-DOF marketplace modules.

  Typical advertised chips:
    * ADXL345 accelerometer at 0x53/0x1D
    * ITG-3200/ITG-3205 gyroscope at 0x68/0x69
    * HMC5883L compass at 0x1E, sometimes replaced by QMC5883L at 0x0D
*/
#ifndef ARDUINONRF_IMU_GY85_H
#define ARDUINONRF_IMU_GY85_H

#include "../../sensors/ADXL345/ADXL345.h"
#include "../../sensors/HMC5883L/HMC5883L.h"
#include "../../sensors/ITG3205/ITG3205.h"
#include "../../sensors/QMC5883L/QMC5883L.h"

namespace nimu {

class GY85 {
 public:
  enum class CompassKind : uint8_t { None, HMC5883L, QMC5883L };

  bool begin(TwoWire& wire = Wire) {
    bool ok = beginI2C(wire, adxl345::kAddrSDOLow, itg3205::kAddrAD0Low);
    if (ok || (accelOk_ && gyroOk_)) {
      return ok;
    }
    return beginI2C(wire, adxl345::kAddrSDOLow, itg3205::kAddrAD0High);
  }

  bool beginI2C(TwoWire& wire, uint8_t accelAddress, uint8_t gyroAddress) {
    accelOk_ = accel_.beginI2C(wire, accelAddress);
    if (!accelOk_ && accelAddress != adxl345::kAddrSDOHigh) {
      accelOk_ = accel_.beginI2C(wire, adxl345::kAddrSDOHigh);
    }

    gyroOk_ = gyro_.beginI2C(wire, gyroAddress);

    compassKind_ = CompassKind::None;
    magOk_ = false;
    if (hmc_.begin(HMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::HMC5883L;
      magOk_ = true;
    } else if (qmc_.begin(QMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::QMC5883L;
      magOk_ = true;
    }

    return accelOk_ && gyroOk_;
  }

  bool accelOk() const { return accelOk_; }
  bool gyroOk() const { return gyroOk_; }
  bool magOk() const { return magOk_; }
  bool hasMagnetometer() const { return magOk_; }
  CompassKind compassKind() const { return compassKind_; }
  const char* compassName() const {
    switch (compassKind_) {
      case CompassKind::HMC5883L: return "HMC5883L";
      case CompassKind::QMC5883L: return "QMC5883L";
      default: return "none";
    }
  }

  ADXL345& accel() { return accel_; }
  ITG3205& gyro() { return gyro_; }
  HMC5883L& hmc() { return hmc_; }
  QMC5883L& qmc() { return qmc_; }

  bool update() {
    bool ok = true;
    if (accelOk_) ok &= accel_.update();
    if (gyroOk_) ok &= gyro_.update();
    if (magOk_) {
      ok &= (compassKind_ == CompassKind::HMC5883L) ? hmc_.update()
                                                    : qmc_.update();
    }
    return ok;
  }

  Vec3 accelG() const { return accel_.accelG(); }
  Vec3 gyroDps() const { return gyro_.gyroDps(); }
  Vec3 magUT() const {
    if (compassKind_ == CompassKind::HMC5883L) return hmc_.magUT();
    if (compassKind_ == CompassKind::QMC5883L) return qmc_.magUT();
    return Vec3{};
  }
  float gyroTemperatureC() const { return gyro_.temperatureC(); }

  bool calibrateGyro(uint16_t samples = 200) {
    return gyro_.calibrateGyro(samples);
  }
  bool calibrateAccel(uint16_t samples = 200) {
    return accel_.calibrateAccel(samples);
  }

  IMUCalibration getAccelCalibration() const { return accel_.getCalibration(); }
  IMUCalibration getGyroCalibration() const { return gyro_.getCalibration(); }
  void setAccelCalibration(const IMUCalibration& c) { accel_.setCalibration(c); }
  void setGyroCalibration(const IMUCalibration& c) { gyro_.setCalibration(c); }

 private:
  ADXL345 accel_;
  ITG3205 gyro_;
  HMC5883L hmc_;
  QMC5883L qmc_;
  bool accelOk_ = false;
  bool gyroOk_ = false;
  bool magOk_ = false;
  CompassKind compassKind_ = CompassKind::None;
};

}  // namespace nimu

using nimu::GY85;

#endif  // ARDUINONRF_IMU_GY85_H
