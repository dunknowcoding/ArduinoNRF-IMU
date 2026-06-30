/*
  L3G4200D.h - ArduinoNRF-IMU driver for ST L3G4200D gyroscope.
*/
#ifndef ARDUINONRF_IMU_L3G4200D_H
#define ARDUINONRF_IMU_L3G4200D_H

#include "../../imu/IMUSensor.h"
#include "L3G4200D_Registers.h"

namespace nimu {

class L3G4200D : public IMUSensor {
 public:
  L3G4200D() { name_ = "L3G4200D"; }

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
  bool configureInterruptPins(bool activeLow, bool openDrain);
  bool setDataReadyInterrupt(bool enable = true);
  bool setThresholdInterrupt(bool enable = true);
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t gx, gy, gz;
    int8_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  uint8_t odrBitsForHz(uint16_t hz, uint16_t& actualHz) const;

  uint32_t clockHz_ = 400000;
  uint8_t odrBits_ = 0x00;
  float gyroLsbPerDps_ = 114.2857f;
  uint16_t gyroRangeDps_ = 250;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::L3G4200D;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_L3G4200D_H
