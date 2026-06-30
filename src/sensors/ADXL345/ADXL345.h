/*
  ADXL345.h - ArduinoNRF-IMU driver for Analog Devices ADXL345 accelerometer.
*/
#ifndef ARDUINONRF_IMU_ADXL345_H
#define ARDUINONRF_IMU_ADXL345_H

#include "../../imu/IMUSensor.h"
#include "ADXL345_Registers.h"

namespace nimu {

class ADXL345 : public IMUSensor {
 public:
  static constexpr uint8_t INTERRUPT_DATA_READY = 0x80;
  static constexpr uint8_t INTERRUPT_SINGLE_TAP = 0x40;
  static constexpr uint8_t INTERRUPT_DOUBLE_TAP = 0x20;
  static constexpr uint8_t INTERRUPT_ACTIVITY = 0x10;
  static constexpr uint8_t INTERRUPT_INACTIVITY = 0x08;
  static constexpr uint8_t INTERRUPT_FREE_FALL = 0x04;
  static constexpr uint8_t INTERRUPT_WATERMARK = 0x02;
  static constexpr uint8_t INTERRUPT_OVERRUN = 0x01;
  ADXL345() { name_ = "ADXL345"; }

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
  bool configureInterruptPolarity(bool activeLow);
  bool routeInterrupt(uint8_t sources, uint8_t pin, bool enable = true);
  uint8_t interruptSource();
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  uint8_t odrCodeForHz(uint16_t hz, uint16_t& actualHz) const;

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 256.0f;
  uint16_t accelRangeG_ = 16;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::ADXL345;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_ADXL345_H
