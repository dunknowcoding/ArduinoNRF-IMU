/* BMX055.h - Bosch BMX055 (BMA280 + BMG160 + BMM150) driver. */
#ifndef ARDUINONRF_IMU_BMX055_H
#define ARDUINONRF_IMU_BMX055_H

#include "../../imu/IMUSensor.h"

namespace nimu {

class BMX055 : public IMUSensor {
 public:
  static constexpr uint8_t kAccelAddr = 0x18;
  static constexpr uint8_t kGyroAddr = 0x68;
  static constexpr uint8_t kMagAddr = 0x10;

  BMX055() { name_ = "BMX055"; hasMag_ = true; }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t accelAddress) override;
  bool beginI2C(TwoWire& wire, uint8_t accelAddress, uint8_t gyroAddress,
                uint8_t magAddress);
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  bool beginSPI(SPIClass& spi, uint8_t accelCsPin, uint8_t gyroCsPin,
                uint8_t magCsPin);
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool accelConnected();
  bool gyroConnected();
  bool magConnected();
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool dataReady();

  IMUStatus readAccelRegister(uint8_t reg, uint8_t& value) {
    return accelBus_.readRegister(reg, value);
  }
  IMUStatus writeAccelRegister(uint8_t reg, uint8_t value) {
    return accelBus_.writeRegister(reg, value);
  }
  IMUStatus readGyroRegister(uint8_t reg, uint8_t& value) {
    return gyroBus_.readRegister(reg, value);
  }
  IMUStatus writeGyroRegister(uint8_t reg, uint8_t value) {
    return gyroBus_.writeRegister(reg, value);
  }
  IMUStatus readMagRegister(uint8_t reg, uint8_t& value) {
    return magBus_.readRegister(reg, value);
  }
  IMUStatus writeMagRegister(uint8_t reg, uint8_t value) {
    return magBus_.writeRegister(reg, value);
  }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    uint16_t rhall;
    int8_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  struct MagTrim {
    int8_t x1 = 0, y1 = 0, x2 = 0, y2 = 0, xy2 = 0;
    uint8_t xy1 = 0;
    uint16_t z1 = 0, xyz1 = 0;
    int16_t z2 = 0, z3 = 0, z4 = 0;
  } trim_;

  bool configureDefaults();
  bool readMagTrim();
  float compensateMagX(int16_t raw, uint16_t rhall) const;
  float compensateMagY(int16_t raw, uint16_t rhall) const;
  float compensateMagZ(int16_t raw, uint16_t rhall) const;

  IMUBus accelBus_;
  IMUBus gyroBus_;
  IMUBus magBus_;
  float accelLsbPerG_ = 2048.0f;
  float gyroLsbPerDps_ = 65.6f;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
};

}  // namespace nimu

using nimu::BMX055;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif
