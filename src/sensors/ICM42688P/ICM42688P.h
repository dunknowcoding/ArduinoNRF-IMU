/*
  ICM42688P.h - ArduinoNRF-IMU driver for TDK InvenSense ICM-42688-P.
*/
#ifndef ARDUINONRF_IMU_ICM42688P_H
#define ARDUINONRF_IMU_ICM42688P_H

#include "../../imu/IMUSensor.h"
#include "ICM42688P_Registers.h"

namespace nimu {

class ICM42688P : public IMUSensor {
 public:
  enum class Pin9Function : uint8_t { Interrupt2 = 0, Fsync = 1, ClockIn = 2 };
  ICM42688P() { name_ = "ICM42688P"; }

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
  bool routeFsyncInterrupt(uint8_t pin, bool enable = true);
  bool setPin9Function(Pin9Function function);
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
  uint8_t accelFsCode_ = 0x01;  // +/-8 g
  uint8_t gyroFsCode_ = 0x02;   // +/-500 dps
  uint8_t odrCode_ = 0x08;      // 100 Hz
  float accelLsbPerG_ = 4096.0f;
  float gyroLsbPerDps_ = 65.5f;
  uint16_t accelRangeG_ = 8;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::ICM42688P;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_ICM42688P_H
