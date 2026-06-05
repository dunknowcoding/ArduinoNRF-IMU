/*
  GY91.h - NiusIMU driver for the GY-91 10-DOF breakout.

  The GY-91 carries two chips on one I2C bus:
    * an InvenSense MPU-9250 / MPU-6500 (accel + gyro, and a magnetometer on the
      genuine 9250) at 0x68, and
    * a Bosch BMP280 pressure + temperature sensor at 0x76.

  This board class brings both up together and gives you one object for the
  whole module, while still exposing imu() and baro() for full per-chip control.
  It uses the MPU9250 driver, which auto-detects whether a magnetometer is
  actually present - so it works on both genuine (9-axis) and clone (6-axis,
  WHO_AM_I 0x70) GY-91 boards. Check hasMagnetometer() to know which you have.

  Wiring (ArduinoNRF ProMicro nRF52840 default I2C "Wire"):
    SDA -> SDA (silk D6)   SCL -> SCL (silk D7)   VCC -> 3V3   GND -> GND
  (The GY-91 also breaks out SPI/INT/etc.; only SDA/SCL/VCC/GND are needed for
  this I2C driver.)

  Quick start:

      #include <GY91.h>
      GY91 board;

      void setup() {
        Serial.begin(115200);
        board.begin();
      }

      void loop() {
        board.update();
        Vec3  a   = board.accelG();        // g
        Vec3  g   = board.gyroDps();       // deg/s
        float hpa = board.pressureHpa();   // hPa
        float alt = board.altitudeM();     // m (set local QNH for accuracy)
        delay(100);
      }
*/
#ifndef NIUSIMU_GY91_H
#define NIUSIMU_GY91_H

#include "../../sensors/BMP280/BMP280.h"
#include "../../sensors/MPU9250/MPU9250.h"

namespace nimu {

class GY91 {
 public:
  GY91() = default;

  // ------------------------------------------------------------ lifecycle --

  /** Bring up both chips on the given bus at their default GY-91 addresses. */
  bool begin(TwoWire& wire = Wire) {
    return beginI2C(wire, bmp280::kAddrPrimary, mpu6500::kAddrAD0Low);
  }

  /**
   * Bring up both chips explicitly. Each is attempted independently; check
   * imuOk()/baroOk() to see which answered. Returns true only if BOTH did.
   * @param baroAddress 0x76 (default) or 0x77.
   * @param mpuAddress  0x68 (default) or 0x69.
   */
  bool beginI2C(TwoWire& wire, uint8_t baroAddress, uint8_t mpuAddress) {
    imuOk_ = imu_.beginI2C(wire, mpuAddress);
    baroOk_ = baro_.begin(baroAddress, wire);
    return imuOk_ && baroOk_;
  }

  bool imuOk() const { return imuOk_; }
  bool baroOk() const { return baroOk_; }

  // ------------------------------------------------ full per-chip access ---
  MPU9250& imu() { return imu_; }    ///< the inertial sensor, full API
  BMP280& baro() { return baro_; }   ///< the barometer, full API

  // -------------------------------------------------------------- reading --

  /** Refresh every present sensor. True if all present sensors updated. */
  bool update() {
    bool ok = true;
    if (imuOk_) ok &= imu_.update();
    if (baroOk_) ok &= baro_.update();
    return ok;
  }

  // Inertial (from the MPU). Zero if the IMU did not come up.
  Vec3 accelG() const { return imu_.accelG(); }      ///< g
  Vec3 gyroDps() const { return imu_.gyroDps(); }    ///< deg/s
  Vec3 magUT() const { return imu_.magUT(); }        ///< uT (0 if no mag)
  bool hasMagnetometer() const { return imu_.hasMagnetometer(); }
  float imuTemperatureC() const { return imu_.temperatureC(); }

  // Barometric (from the BMP280). Zero if the barometer did not come up.
  float pressurePa() const { return baro_.pressurePa(); }
  float pressureHpa() const { return baro_.pressureHpa(); }
  float baroTemperatureC() const { return baro_.temperatureC(); }
  float altitudeM() const { return baro_.altitudeM(); }
  void setSeaLevelPressureHpa(float hpa) { baro_.setSeaLevelPressureHpa(hpa); }

  // --------------------------------------------------------- calibration ---
  // Forwards to the relevant chip so user code calibrates the whole board in
  // one place. See the GY91_Advanced example for the recommended sequence.

  /** Gyro zero-rate bias: hold the board perfectly still. */
  bool calibrateGyro(uint16_t samples = 200) { return imu_.calibrateGyro(samples); }
  /** Accel bias: hold the board still in any fixed orientation. */
  bool calibrateAccel(uint16_t samples = 200) { return imu_.calibrateAccel(samples); }
  /** Magnetometer hard/soft iron (genuine 9-axis boards only): figure-8 motion. */
  bool calibrateMag(uint16_t durationMs = 15000) { return imu_.calibrateMag(durationMs); }
  /** Reference the altimeter to a known height (default 0 m) at your location. */
  bool calibrateAltitude(float knownAltitudeM = 0.0f) {
    return baro_.calibrateAltitude(knownAltitudeM);
  }

  /** The IMU's full calibration block (copy to EEPROM / restore at boot). */
  IMUCalibration getImuCalibration() const { return imu_.getCalibration(); }
  void setImuCalibration(const IMUCalibration& c) { imu_.setCalibration(c); }

 private:
  MPU9250 imu_;
  BMP280 baro_;
  bool imuOk_ = false;
  bool baroOk_ = false;
};

}  // namespace nimu

using nimu::GY91;

#endif  // NIUSIMU_GY91_H
