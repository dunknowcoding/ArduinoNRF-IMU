/* LSM6DSV.h - ArduinoNRF-IMU driver for ST LSM6DSV. */
#ifndef ARDUINONRF_IMU_LSM6DSV_H
#define ARDUINONRF_IMU_LSM6DSV_H

#include "../../imu/IMUSensor.h"
#include "LSM6DSV_Registers.h"

namespace nimu {

class LSM6DSV : public IMUSensor {
 public:
  LSM6DSV() { name_ = "LSM6DSV"; }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;
  bool reset();

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool dataReady();
  bool configureInterruptPins(bool activeLow, bool openDrain);
  bool routeInterrupt(uint8_t pin, uint8_t sources);
  bool setDataReadyInterrupt(uint8_t pin, bool accel = true,
                             bool gyro = true);
  bool configureOisInterface(bool accel = true, bool gyro = true);
  bool oisDataReady();

  bool enableSensorHubPullups(bool enable = true);
  bool sensorHubRead(uint8_t address, uint8_t reg, uint8_t* data,
                     uint8_t length, uint16_t timeoutMs = 20);
  bool sensorHubWrite(uint8_t address, uint8_t reg, uint8_t value,
                      uint16_t timeoutMs = 20);
  bool configureQMC6309();
  bool readQMC6309(Vec3& magUT);

  struct SflpQuaternion {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
  };
  bool enableSflp(uint16_t rateHz = 120);
  bool disableSflp();
  uint16_t fifoSamples();
  bool readSflpQuaternion(SflpQuaternion& quaternion);

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 protected:
  bool sensorHubBank(bool enable);
  bool embeddedBank(bool enable);
  bool waitSensorHub(uint16_t timeoutMs);
  static float halfToFloat(uint16_t value);

  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 32.768f;
  uint8_t odrCode_ = 6;
  bool qmcEnabled_ = false;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::LSM6DSV;
using nimu::Vec3;

#endif
