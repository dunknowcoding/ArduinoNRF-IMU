/*
  ITG3205.h - ArduinoNRF-IMU driver for InvenSense ITG-3200/ITG-3205 gyro.
*/
#ifndef ARDUINONRF_IMU_ITG3205_H
#define ARDUINONRF_IMU_ITG3205_H

#include "../../imu/IMUSensor.h"
#include "ITG3205_Registers.h"

namespace nimu {

class ITG3205 : public IMUSensor {
 public:
  ITG3205() { name_ = "ITG3205"; }

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
  bool configureInterrupt(bool activeLow, bool openDrain,
                          bool latched = false,
                          bool clearOnAnyRead = false);
  bool setDataReadyInterrupt(bool enable = true);
  bool setPllReadyInterrupt(bool enable = true);
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  uint8_t dlpfCodeForHz(uint16_t hz) const;

  uint32_t clockHz_ = 400000;
  uint8_t dlpfCode_ = 0x03;
  float gyroLsbPerDps_ = 14.375f;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::ITG3205;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_ITG3205_H
