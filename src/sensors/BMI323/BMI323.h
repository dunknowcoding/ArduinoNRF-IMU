/* ArduinoNRF-IMU driver for the Bosch BMI323. */
#ifndef ARDUINONRF_IMU_BMI323_H
#define ARDUINONRF_IMU_BMI323_H

#include "../../imu/IMUSensor.h"
#include "BMI323_Registers.h"

namespace nimu {

class BMI323 : public IMUSensor {
 public:
  BMI323() { name_ = "BMI323"; }

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
  bool dataReady();
  bool configureInterruptPin(uint8_t pin, bool activeHigh, bool openDrain,
                             bool latched = false);
  bool routeDataReadyInterrupt(uint8_t pin, bool accel = true,
                               bool gyro = true);
  bool routeDataReadyInterrupts(uint8_t accelPin, uint8_t gyroPin);
  bool enableFeatureEngine(bool enable = true);
  bool readRegisterWord(uint8_t reg, uint16_t& value) {
    return readWords(reg, &value, 1);
  }
  bool writeRegisterWord(uint8_t reg, uint16_t value) {
    return writeWord(reg, value);
  }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool readWords(uint8_t reg, uint16_t* words, size_t count);
  bool writeWord(uint8_t reg, uint16_t value);
  bool reset();
  bool configureDefaults();
  uint8_t odrForHz(uint16_t hz, uint16_t& actualHz) const;
  uint16_t configWord(uint8_t odr, uint8_t range) const;

  TwoWire* wire_ = nullptr;
  bool spiMode_ = false;
  uint8_t address_ = bmi323::kAddrLow;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 65.536f;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
  uint8_t accelRangeCode_ = 1;
  uint8_t gyroRangeCode_ = 2;
  uint8_t odrCode_ = 0x08;
};

}  // namespace nimu

using nimu::BMI323;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif
