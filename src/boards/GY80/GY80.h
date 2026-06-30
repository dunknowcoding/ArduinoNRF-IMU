/*
  GY80.h - Board class for common GY-80/GY-801 marketplace modules.

  Typical advertised chips:
    * ADXL345 accelerometer at 0x53/0x1D
    * L3G4200D gyroscope at 0x69/0x68
    * HMC5883L compass at 0x1E, sometimes replaced by QMC5883L at 0x0D
    * BMP085/BMP180 pressure + temperature at 0x77
*/
#ifndef ARDUINONRF_IMU_GY80_H
#define ARDUINONRF_IMU_GY80_H

#include "../../sensors/ADXL345/ADXL345.h"
#include "../../sensors/BMP180/BMP180.h"
#include "../../sensors/HMC5883L/HMC5883L.h"
#include "../../sensors/L3G4200D/L3G4200D.h"
#include "../../sensors/QMC5883L/QMC5883L.h"

namespace nimu {

class GY80 {
 public:
  enum class CompassKind : uint8_t { None, HMC5883L, QMC5883L };

  bool begin(TwoWire& wire = Wire) {
    return beginI2C(wire, adxl345::kAddrSDOLow, l3g4200d::kAddrSDOHigh,
                    bmp180::kAddr);
  }

  bool beginI2C(TwoWire& wire, uint8_t accelAddress, uint8_t gyroAddress,
                uint8_t baroAddress) {
    accelOk_ = accel_.beginI2C(wire, accelAddress);
    if (!accelOk_ && accelAddress != adxl345::kAddrSDOHigh) {
      accelOk_ = accel_.beginI2C(wire, adxl345::kAddrSDOHigh);
    }

    gyroOk_ = gyro_.beginI2C(wire, gyroAddress);
    if (!gyroOk_ && gyroAddress != l3g4200d::kAddrSDOLow) {
      gyroOk_ = gyro_.beginI2C(wire, l3g4200d::kAddrSDOLow);
    }

    baroOk_ = baro_.begin(baroAddress, wire);

    compassKind_ = CompassKind::None;
    magOk_ = false;
    if (hmc_.begin(HMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::HMC5883L;
      magOk_ = true;
    } else if (qmc_.begin(QMC5883L::kAddr, wire)) {
      compassKind_ = CompassKind::QMC5883L;
      magOk_ = true;
    }

    return accelOk_ && gyroOk_ && baroOk_;
  }

  void configurePressurePins(uint8_t xclrPin, uint8_t eocPin) {
    pressureXclrPin_ = xclrPin;
    pressureEocPin_ = eocPin;
    pinMode(pressureXclrPin_, OUTPUT);
    digitalWrite(pressureXclrPin_, HIGH);
    pinMode(pressureEocPin_, INPUT);
  }

  bool resetPressure() {
    if (pressureXclrPin_ == 0xFF) return false;
    digitalWrite(pressureXclrPin_, LOW);
    delayMicroseconds(10);
    digitalWrite(pressureXclrPin_, HIGH);
    delay(3);
    return true;
  }

  bool pressureConversionComplete() const {
    return pressureEocPin_ != 0xFF && digitalRead(pressureEocPin_) == HIGH;
  }

  bool accelOk() const { return accelOk_; }
  bool gyroOk() const { return gyroOk_; }
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

  ADXL345& accel() { return accel_; }
  L3G4200D& gyro() { return gyro_; }
  BMP180& baro() { return baro_; }
  HMC5883L& hmc() { return hmc_; }
  QMC5883L& qmc() { return qmc_; }

  bool update() {
    bool ok = true;
    if (accelOk_) ok &= accel_.update();
    if (gyroOk_) ok &= gyro_.update();
    if (baroOk_) ok &= baro_.update();
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
  float pressurePa() const { return baro_.pressurePa(); }
  float pressureHpa() const { return baro_.pressureHpa(); }
  float baroTemperatureC() const { return baro_.temperatureC(); }
  float altitudeM() const { return baro_.altitudeM(); }
  void setSeaLevelPressureHpa(float hpa) { baro_.setSeaLevelPressureHpa(hpa); }

  bool calibrateGyro(uint16_t samples = 200) {
    return gyro_.calibrateGyro(samples);
  }
  bool calibrateAccel(uint16_t samples = 200) {
    return accel_.calibrateAccel(samples);
  }
  bool calibrateAltitude(float knownAltitudeM = 0.0f) {
    return baro_.calibrateAltitude(knownAltitudeM);
  }

  IMUCalibration getAccelCalibration() const { return accel_.getCalibration(); }
  IMUCalibration getGyroCalibration() const { return gyro_.getCalibration(); }
  void setAccelCalibration(const IMUCalibration& c) { accel_.setCalibration(c); }
  void setGyroCalibration(const IMUCalibration& c) { gyro_.setCalibration(c); }

 private:
  ADXL345 accel_;
  L3G4200D gyro_;
  BMP180 baro_;
  HMC5883L hmc_;
  QMC5883L qmc_;
  bool accelOk_ = false;
  bool gyroOk_ = false;
  bool baroOk_ = false;
  bool magOk_ = false;
  uint8_t pressureXclrPin_ = 0xFF;
  uint8_t pressureEocPin_ = 0xFF;
  CompassKind compassKind_ = CompassKind::None;
};

}  // namespace nimu

using nimu::GY80;

#endif  // ARDUINONRF_IMU_GY80_H
