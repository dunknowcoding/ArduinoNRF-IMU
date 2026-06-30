/*
  ICM20948.h - ArduinoNRF-IMU driver for TDK InvenSense ICM-20948.
*/
#ifndef ARDUINONRF_IMU_ICM20948_H
#define ARDUINONRF_IMU_ICM20948_H

#include "../../imu/IMUSensor.h"
#include "ICM20948_Registers.h"

namespace nimu {

class ICM20948 : public IMUSensor {
 public:
  ICM20948() { name_ = "ICM20948"; }

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
  bool configureInterruptPin(bool activeLow, bool openDrain,
                             bool latched = false,
                             bool clearOnAnyRead = false);
  bool setDataReadyInterrupt(bool enable = true);
  bool setFsyncInterrupt(bool enable, bool activeLow = false);
  bool setAuxI2CBypass(bool enable);
  bool enableAuxI2CMaster(bool enable = true);
  bool auxReadRegister(uint8_t address, uint8_t reg, uint8_t& value,
                       uint16_t timeoutMs = 20);
  bool auxWriteRegister(uint8_t address, uint8_t reg, uint8_t value,
                        uint16_t timeoutMs = 20);
  bool auxPing(uint8_t address);
  bool enableMagnetometer(bool enable = true);
  uint8_t magWhoAmI();
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool selectBank(uint8_t bank);
  bool configureDefaults();
  uint8_t dlpfCodeForHz(uint16_t hz) const;
  bool waitAuxTransaction(uint16_t timeoutMs);

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 65.5f;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
  uint8_t accelFs_ = 0x01;
  uint8_t gyroFs_ = 0x01;
  uint8_t dlpfCode_ = 0x03;
  uint8_t currentBank_ = 0xFF;
  IMUBus magBus_;
  TwoWire* wire_ = nullptr;
  bool magEnabled_ = false;
};

}  // namespace nimu

using nimu::ICM20948;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_ICM20948_H
