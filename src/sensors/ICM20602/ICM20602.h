/*
  ICM20602.h - ArduinoNRF-IMU driver for TDK InvenSense ICM-20602.
*/
#ifndef ARDUINONRF_IMU_ICM20602_H
#define ARDUINONRF_IMU_ICM20602_H

#include "../MPU6500/MPU6500.h"

namespace nimu {

class ICM20602 : public MPU6500 {
 public:
  static constexpr uint8_t kWhoAmI = 0x12;

  ICM20602() { name_ = "ICM20602"; }

  bool begin() override {
    if (beginI2C(Wire, mpu6500::kAddrAD0Low)) {
      return true;
    }
    return beginI2C(Wire, mpu6500::kAddrAD0High);
  }

  bool beginI2C(TwoWire& wire, uint8_t address) override {
    i2cWire_ = &wire;
    bus_.beginI2C(wire, address, clockHz_);
    bus_.recoverBus();
    if (!isConnected()) {
      // Some boards only report WHO_AM_I after the sleep bit is cleared.
      bus_.writeRegister(mpu6500::PWR_MGMT_1, mpu6500::PWR1_CLKSEL_AUTO);
      delay(10);
    }
    if (!isConnected()) {
      return false;
    }
    return reset() && configureDefaults() && initExtras();
  }

  bool beginSPI(SPIClass& spi, uint8_t csPin) override {
    bus_.beginSPI(spi, csPin, 1000000, 0x80);
    i2cWire_ = nullptr;
    if (!isConnected()) {
      bus_.writeRegister(mpu6500::PWR_MGMT_1, mpu6500::PWR1_CLKSEL_AUTO);
      delay(10);
    }
    if (!isConnected() || !reset()) {
      return false;
    }
    bus_.updateRegister(mpu6500::USER_CTRL, mpu6500::USERCTRL_I2C_IF_DIS,
                        mpu6500::USERCTRL_I2C_IF_DIS);
    return configureDefaults() && initExtras();
  }

  bool isConnected() override { return whoAmI() == kWhoAmI; }
};

}  // namespace nimu

using nimu::ICM20602;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_ICM20602_H
