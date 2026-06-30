/*
  GY86.h - Board class for common GY-86 10-DOF marketplace modules.

  Typical advertised chips:
    * MPU-6050 accel + gyro at 0x68/0x69
    * MS5611 pressure + temperature at 0x77/0x76
    * HMC5883L compass at 0x1E, often behind MPU-6050 aux I2C
*/
#ifndef ARDUINONRF_IMU_GY86_H
#define ARDUINONRF_IMU_GY86_H

#include "../../sensors/HMC5883L/HMC5883L.h"
#include "../../sensors/MPU6050/MPU6050.h"
#include "../../sensors/MS5611/MS5611.h"
#include "../../sensors/QMC5883L/QMC5883L.h"

namespace nimu {

class GY86 {
 public:
  enum class CompassKind : uint8_t { None, HMC5883L, QMC5883L };

  bool begin(TwoWire& wire = Wire) {
    bool ok68 = beginI2C(wire, MS5611::kAddrHigh, mpu6050::kAddrAD0Low);
    if (ok68 || imuOk_) {
      return ok68;
    }
    return beginI2C(wire, MS5611::kAddrHigh, mpu6050::kAddrAD0High);
  }

  bool beginI2C(TwoWire& wire, uint8_t baroAddress, uint8_t mpuAddress) {
    imuOk_ = imu_.beginI2C(wire, mpuAddress);
    if (imuOk_) {
      imu_.setAuxI2CBypass(true);
    }

    baroOk_ = baro_.begin(baroAddress, wire);

    compassKind_ = CompassKind::None;
    magOk_ = false;
    if (imuOk_ && hmc_.begin(HMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::HMC5883L;
      magOk_ = true;
    } else if (imuOk_ && qmc_.begin(QMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::QMC5883L;
      magOk_ = true;
    }

    return imuOk_ && baroOk_;
  }

  bool imuOk() const { return imuOk_; }
  bool baroOk() const { return baroOk_; }
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

  MPU6050& imu() { return imu_; }
  MS5611& baro() { return baro_; }
  HMC5883L& hmc() { return hmc_; }
  QMC5883L& qmc() { return qmc_; }

  bool update() {
    bool ok = true;
    if (imuOk_) ok &= imu_.update();
    if (baroOk_) ok &= baro_.update();
    if (magOk_) {
      ok &= (compassKind_ == CompassKind::HMC5883L) ? hmc_.update()
                                                    : qmc_.update();
    }
    return ok;
  }

  Vec3 accelG() const { return imu_.accelG(); }
  Vec3 gyroDps() const { return imu_.gyroDps(); }
  Vec3 magUT() const {
    if (compassKind_ == CompassKind::HMC5883L) return hmc_.magUT();
    if (compassKind_ == CompassKind::QMC5883L) return qmc_.magUT();
    return Vec3{};
  }
  float imuTemperatureC() const { return imu_.temperatureC(); }

  float pressurePa() const { return baro_.pressurePa(); }
  float pressureHpa() const { return baro_.pressureHpa(); }
  float baroTemperatureC() const { return baro_.temperatureC(); }
  float altitudeM() const { return baro_.altitudeM(); }
  void setSeaLevelPressureHpa(float hpa) { baro_.setSeaLevelPressureHpa(hpa); }

  bool calibrateGyro(uint16_t samples = 200) { return imu_.calibrateGyro(samples); }
  bool calibrateAccel(uint16_t samples = 200) { return imu_.calibrateAccel(samples); }
  bool calibrateAltitude(float knownAltitudeM = 0.0f) {
    return baro_.calibrateAltitude(knownAltitudeM);
  }

  IMUCalibration getImuCalibration() const { return imu_.getCalibration(); }
  void setImuCalibration(const IMUCalibration& c) { imu_.setCalibration(c); }

 private:
  MPU6050 imu_;
  MS5611 baro_;
  HMC5883L hmc_;
  QMC5883L qmc_;
  bool imuOk_ = false;
  bool baroOk_ = false;
  bool magOk_ = false;
  CompassKind compassKind_ = CompassKind::None;
};

}  // namespace nimu

using nimu::GY86;

#endif  // ARDUINONRF_IMU_GY86_H
