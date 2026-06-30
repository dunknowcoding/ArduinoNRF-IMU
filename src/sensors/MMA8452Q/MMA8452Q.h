/*
  MMA8452Q.h - ArduinoNRF-IMU driver for NXP MMA8452Q accelerometers.
*/
#ifndef ARDUINONRF_IMU_MMA8452Q_H
#define ARDUINONRF_IMU_MMA8452Q_H

#include "../../imu/IMUSensor.h"
#include "MMA8452Q_Registers.h"

namespace nimu {

class MMA8452Q : public IMUSensor {
 public:
  static constexpr uint8_t INTERRUPT_AUTO_SLEEP = 0x80;
  static constexpr uint8_t INTERRUPT_TRANSIENT = 0x20;
  static constexpr uint8_t INTERRUPT_ORIENTATION = 0x10;
  static constexpr uint8_t INTERRUPT_PULSE = 0x08;
  static constexpr uint8_t INTERRUPT_FREEFALL_MOTION = 0x04;
  static constexpr uint8_t INTERRUPT_DATA_READY = 0x01;
  MMA8452Q() { name_ = "MMA8452Q"; }

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
  bool configureInterruptPins(bool activeHigh, bool openDrain);
  bool routeInterrupt(uint8_t sources, uint8_t pin, bool enable = true);
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  bool setStandby(bool standby);
  uint8_t odrBitsForHz(uint16_t hz, uint16_t& actualHz) const;

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 512.0f;
  uint16_t accelRangeG_ = 4;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MMA8452Q;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_MMA8452Q_H
