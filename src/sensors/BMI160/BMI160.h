/*
  BMI160.h - ArduinoNRF-IMU driver for Bosch BMI160 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_BMI160_H
#define ARDUINONRF_IMU_BMI160_H

#include "../../imu/IMUSensor.h"
#include "BMI160_Registers.h"

namespace nimu {

class BMI160 : public IMUSensor {
 public:
  BMI160() { name_ = "BMI160"; }

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
  uint8_t odrCodeForHz(uint16_t hz, uint16_t& actualHz) const;

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 65.6f;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::BMI160;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_BMI160_H
