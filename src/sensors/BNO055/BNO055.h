/*
  BNO055.h - ArduinoNRF-IMU driver for Bosch BNO055 absolute orientation IMU.
*/
#ifndef ARDUINONRF_IMU_BNO055_H
#define ARDUINONRF_IMU_BNO055_H

#include "../../imu/IMUSensor.h"
#include "BNO055_Registers.h"

namespace nimu {

class BNO055 : public IMUSensor {
 public:
  static constexpr uint8_t MODE_CONFIG = bno055::MODE_CONFIG;
  static constexpr uint8_t MODE_AMG = bno055::MODE_AMG;
  static constexpr uint8_t MODE_IMUPLUS = bno055::MODE_IMUPLUS;
  static constexpr uint8_t MODE_NDOF = bno055::MODE_NDOF;

  struct Quaternion {
    float w, x, y, z;
  };

  struct CalibrationProfile {
    uint8_t data[bno055::CALIBRATION_PROFILE_LENGTH] = {0};
  };

  BNO055() {
    name_ = "BNO055";
    hasMag_ = true;
  }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;

  bool setMode(uint8_t mode);
  uint8_t mode() const { return mode_; }
  bool setExternalCrystal(bool enable);
  bool reset();
  void configurePins(int8_t resetPin, int8_t interruptPin = -1);
  bool hardwareReset(uint16_t bootDelayMs = 700);
  bool interruptAsserted() const;
  uint8_t interruptStatus();
  bool clearInterrupt();

  bool fullyCalibrated();
  bool readCalibrationProfile(CalibrationProfile& profile);
  bool writeCalibrationProfile(const CalibrationProfile& profile);
  bool readPageRegister(uint8_t page, uint8_t reg, uint8_t& value);
  bool writePageRegister(uint8_t page, uint8_t reg, uint8_t value,
                         bool configurationMode = true);

  uint8_t calibrationStatus();
  uint8_t systemCalibration();
  uint8_t gyroCalibration();
  uint8_t accelCalibration();
  uint8_t magCalibration();

  Vec3 eulerDeg();
  Quaternion quaternion();
  Vec3 linearAccelMs2();
  Vec3 gravityMs2();

 private:
  bool setPage(uint8_t page);
  bool readVector(uint8_t reg, int16_t& x, int16_t& y, int16_t& z);

  uint8_t mode_ = bno055::MODE_NDOF;
  bool externalCrystal_ = false;
  int8_t resetPin_ = -1;
  int8_t interruptPin_ = -1;
};

}  // namespace nimu

using nimu::BNO055;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_BNO055_H
