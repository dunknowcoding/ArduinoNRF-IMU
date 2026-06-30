/*
  LSM6DSOX.h - ArduinoNRF-IMU driver for ST's LSM6DSOX 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_LSM6DSOX_H
#define ARDUINONRF_IMU_LSM6DSOX_H

#include "../../imu/IMUSensor.h"
#include "LSM6DSOX_Registers.h"

namespace nimu {

class LSM6DSOX : public IMUSensor {
 public:
  LSM6DSOX() { name_ = "LSM6DSOX"; }

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
  bool configureInterruptPins(bool activeLow, bool openDrain);
  bool routeInterrupt(uint8_t pin, uint8_t sources);
  bool setDataReadyInterrupt(uint8_t pin, bool accel = true,
                             bool gyro = true);
  template <typename UcfLine>
  bool loadUcf(const UcfLine* configuration, size_t count) {
    if (configuration == nullptr || count == 0) return false;
    for (size_t i = 0; i < count; ++i) {
      if (bus_.writeRegister(configuration[i].address,
                             configuration[i].data) != IMUStatus::Ok) {
        return false;
      }
    }
    return true;
  }
  bool readMlcOutput(uint8_t index, uint8_t& value);
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
  uint8_t odrBitsForHz(uint16_t hz, uint16_t& actualHz) const;

  uint32_t clockHz_ = 400000;
  uint8_t odrBits_ = lsm6dsox::ODR_104;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 32.768f;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 1000;
  uint16_t sampleRateHz_ = 104;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::LSM6DSOX;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_LSM6DSOX_H
