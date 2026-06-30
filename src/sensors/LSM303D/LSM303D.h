/* LSM303D.h - ST single-chip accelerometer + magnetometer driver. */
#ifndef ARDUINONRF_IMU_LSM303D_H
#define ARDUINONRF_IMU_LSM303D_H

#include "../../imu/IMUSensor.h"

namespace nimu {

class LSM303D : public IMUSensor {
 public:
  static constexpr uint8_t kAddrLow = 0x1D;
  static constexpr uint8_t kAddrHigh = 0x1E;
  static constexpr uint8_t kWhoAmI = 0x49;

  LSM303D() { name_ = "LSM303D"; hasMag_ = true; }
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
  bool setMagRangeGauss(uint16_t gauss);
  bool dataReady();
  bool magDataReady();
  IMUStatus readRegister(uint8_t reg, uint8_t& value) {
    return bus_.readRegister(reg, value);
  }
  IMUStatus writeRegister(uint8_t reg, uint8_t value) {
    return bus_.writeRegister(reg, value);
  }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  float accelGPerLsb_ = 0.000122f;
  float magUtPerLsb_ = 0.016f;
  uint8_t accelRangeCode_ = 1;
  uint8_t accelOdrCode_ = 6;
};

}  // namespace nimu

using nimu::LSM303D;
using nimu::Vec3;

#endif
