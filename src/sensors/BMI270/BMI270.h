/*
  BMI270.h - ArduinoNRF-IMU driver for Bosch BMI270 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_BMI270_H
#define ARDUINONRF_IMU_BMI270_H

#include "../../imu/IMUSensor.h"
#include "BMI270_Registers.h"

namespace nimu {

class BMI270 : public IMUSensor {
 public:
  BMI270() { name_ = "BMI270"; }

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

  bool reset();
  bool dataReady();
  bool configurationLoaded();
  bool configureInterruptPin(uint8_t pin, bool activeHigh, bool openDrain,
                             bool latched = false);
  bool routeDataReadyInterrupt(uint8_t pin, bool enable = true);
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  bool uploadConfiguration();
  bool writeConfigChunk(uint16_t index, const uint8_t* data, uint16_t len);
  uint8_t odrCodeForHz(uint16_t hz, uint16_t& actualHz, bool gyro) const;

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 65.536f;
  uint8_t accelOdr_ = 0x08;
  uint8_t gyroOdr_ = 0x08;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::BMI270;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_BMI270_H
