/*
  QMI8658.h - ArduinoNRF-IMU driver for QST QMI8658/QMI8658C 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_QMI8658_H
#define ARDUINONRF_IMU_QMI8658_H

#include "../../imu/IMUSensor.h"
#include "QMI8658_Registers.h"

namespace nimu {

class QMI8658 : public IMUSensor {
 public:
  QMI8658() { name_ = "QMI8658"; }

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
  bool setSynchronizedSampleMode(bool enable = true);
  uint8_t interruptStatus();
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
  uint8_t accelOdr_ = 0x03;  // 1000 Hz
  uint8_t gyroOdr_ = 0x03;
  float accelLsbPerG_ = 4096.0f;  // +/-8 g default
  float gyroLsbPerDps_ = 64.0f;   // +/-512 dps default
  uint16_t accelRangeG_ = 8;
  uint16_t gyroRangeDps_ = 512;
  uint16_t sampleRateHz_ = 1000;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::QMI8658;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_QMI8658_H
