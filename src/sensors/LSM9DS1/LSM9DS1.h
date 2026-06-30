/*
  LSM9DS1.h - ArduinoNRF-IMU driver for ST's LSM9DS1 9-axis IMU.
*/
#ifndef ARDUINONRF_IMU_LSM9DS1_H
#define ARDUINONRF_IMU_LSM9DS1_H

#include "../../imu/IMUSensor.h"
#include "LSM9DS1_Registers.h"

namespace nimu {

class LSM9DS1 : public IMUSensor {
 public:
  LSM9DS1() {
    name_ = "LSM9DS1";
    hasMag_ = true;
  }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginI2C(TwoWire& wire, uint8_t agAddress, uint8_t magAddress);
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  uint8_t magWhoAmI();
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool setMagRangeGauss(uint16_t gauss);

  bool reset();
  bool dataReady();
  bool magDataReady();
  bool routeDataReadyInterrupt(uint8_t pin, bool accel = true,
                               bool gyro = true);
  bool configureMagInterrupt(uint8_t axesMask, uint16_t threshold,
                             bool activeHigh = true, bool latched = false,
                             bool enable = true);
  uint8_t magInterruptSource();
  IMUStatus readAccelGyroRegister(uint8_t reg, uint8_t& value) {
    return bus_.readRegister(reg, value);
  }
  IMUStatus writeAccelGyroRegister(uint8_t reg, uint8_t value) {
    return bus_.writeRegister(reg, value);
  }
  IMUStatus readMagRegister(uint8_t reg, uint8_t& value) {
    return magBus_.readRegister(reg, value);
  }
  IMUStatus writeMagRegister(uint8_t reg, uint8_t value) {
    return magBus_.writeRegister(reg, value);
  }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  uint8_t odrBitsForHz(uint16_t hz, uint16_t& actualHz) const;

  IMUBus magBus_;
  uint32_t clockHz_ = 400000;

  uint8_t odrBits_ = 0x60;  // 119 Hz default for accel/gyro.
  float accelLsbPerG_ = 8196.7213f;
  float gyroLsbPerDps_ = 57.1429f;
  float magLsbPerUT_ = 1.0f / 0.014f;  // +/-4 gauss default: 0.014 uT/LSB
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 119;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::LSM9DS1;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_LSM9DS1_H
