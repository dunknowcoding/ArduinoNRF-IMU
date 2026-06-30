/*
  LSM303DLHC.h - ArduinoNRF-IMU driver for ST LSM303DLHC eCompass modules.
*/
#ifndef ARDUINONRF_IMU_LSM303DLHC_H
#define ARDUINONRF_IMU_LSM303DLHC_H

#include "../../imu/IMUSensor.h"
#include "LSM303DLHC_Registers.h"

namespace nimu {

class LSM303DLHC : public IMUSensor {
 public:
  LSM303DLHC() {
    name_ = "LSM303DLHC";
    hasMag_ = true;
  }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool beginI2C(TwoWire& wire, uint8_t accelAddress, uint8_t magAddress);
  uint8_t magWhoAmI();

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool setMagRangeGauss(float gauss);

  bool dataReady();
  bool magDataReady();
  bool setAccelDataReadyInterrupt(bool enable = true);
  bool configureAccelInterruptPolarity(bool activeLow);
  IMUStatus readAccelRegister(uint8_t reg, uint8_t& value) {
    return bus_.readRegister(reg, value);
  }
  IMUStatus writeAccelRegister(uint8_t reg, uint8_t value) {
    return bus_.writeRegister(reg, value);
  }
  IMUStatus readMagRegister(uint8_t reg, uint8_t& value) {
    return magBus_.readRegister(reg, value);
  }
  IMUStatus writeMagRegister(uint8_t reg, uint8_t value) {
    return magBus_.writeRegister(reg, value);
  }
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  uint8_t odrBitsForHz(uint16_t hz, uint16_t& actualHz) const;

  IMUBus magBus_;
  uint32_t clockHz_ = 400000;
  float accelSensitivityG_ = 0.001f;
  float magLsbPerGaussXY_ = 1100.0f;
  float magLsbPerGaussZ_ = 980.0f;
  uint16_t accelRangeG_ = 2;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::LSM303DLHC;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_LSM303DLHC_H
