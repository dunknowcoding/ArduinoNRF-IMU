/*
  BMP180.h - ArduinoNRF-IMU driver for the Bosch BMP180 barometer.

  The BMP180 appears on common GY-87 10-DOF marketplace boards. It provides
  pressure, temperature, and altitude derived from a sea-level pressure setting.
*/
#ifndef ARDUINONRF_IMU_BMP180_H
#define ARDUINONRF_IMU_BMP180_H

#include <Arduino.h>
#include <Wire.h>

#include "../../imu/IMUBus.h"
#include "BMP180_Registers.h"

namespace nimu {

class BMP180 {
 public:
  enum Oversampling : uint8_t {
    OSS_ULTRA_LOW_POWER = 0,
    OSS_STANDARD = 1,
    OSS_HIGH_RES = 2,
    OSS_ULTRA_HIGH_RES = 3,
  };

  bool begin(uint8_t address = bmp180::kAddr, TwoWire& wire = Wire);
  uint8_t chipId();
  bool isConnected();
  bool update();

  float temperatureC() const { return tempC_; }
  float pressurePa() const { return pressPa_; }
  float pressureHpa() const { return pressPa_ * 0.01f; }
  float altitudeM() const;

  void setSeaLevelPressureHpa(float hpa) { seaLevelHpa_ = hpa; }
  float seaLevelPressureHpa() const { return seaLevelHpa_; }
  bool calibrateAltitude(float knownAltitudeM = 0.0f);

  void setOversampling(Oversampling oss) { oss_ = oss; }
  Oversampling oversampling() const { return oss_; }

 private:
  bool readCalibration();
  bool readRawTemperature(int32_t& out);
  bool readRawPressure(int32_t& out);

  IMUBus bus_;

  int16_t ac1_ = 0, ac2_ = 0, ac3_ = 0;
  uint16_t ac4_ = 0, ac5_ = 0, ac6_ = 0;
  int16_t b1_ = 0, b2_ = 0, mb_ = 0, mc_ = 0, md_ = 0;
  int32_t b5_ = 0;

  Oversampling oss_ = OSS_STANDARD;
  float tempC_ = 0.0f;
  float pressPa_ = 0.0f;
  float seaLevelHpa_ = 1013.25f;
};

}  // namespace nimu

using nimu::BMP180;

#endif  // ARDUINONRF_IMU_BMP180_H
