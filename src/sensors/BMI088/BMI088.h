/* ArduinoNRF-IMU driver for the Bosch BMI088. */
#ifndef ARDUINONRF_IMU_BMI088_H
#define ARDUINONRF_IMU_BMI088_H

#include "../../imu/IMUSensor.h"
#include "BMI088_Registers.h"

namespace nimu {

class BMI088 : public IMUSensor {
 public:
  BMI088() { name_ = "BMI088"; }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginI2C(TwoWire& wire, uint8_t accelAddress, uint8_t gyroAddress);
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  bool beginSPI(SPIClass& spi, uint8_t accelCsPin, uint8_t gyroCsPin);
  uint8_t whoAmI() override;
  uint8_t gyroWhoAmI();
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool dataReady();
  bool configureAccelInterruptPin(uint8_t pin, bool activeHigh,
                                  bool openDrain);
  bool routeAccelDataReadyInterrupt(uint8_t pin, bool enable = true);
  bool configureGyroInterruptPin(uint8_t pin, bool activeHigh,
                                 bool openDrain);
  bool routeGyroDataReadyInterrupt(uint8_t pin, bool enable = true);

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool reset();
  bool configureDefaults();
  uint8_t accelOdrForHz(uint16_t hz, uint16_t& actualHz) const;
  uint8_t gyroBandwidthForHz(uint16_t hz, uint16_t& actualHz) const;

  IMUBus gyroBus_;
  float accelLsbPerG_ = 5460.0f;
  float gyroLsbPerDps_ = 65.536f;
  uint16_t accelRangeG_ = 6;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
  uint8_t accelOdr_ = 0x08;
  uint8_t gyroBandwidth_ = 0x07;
};

}  // namespace nimu

using nimu::BMI088;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif
