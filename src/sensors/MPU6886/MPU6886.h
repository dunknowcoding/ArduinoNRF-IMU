/*
  MPU6886.h - ArduinoNRF-IMU driver for TDK InvenSense MPU-6886.
*/
#ifndef ARDUINONRF_IMU_MPU6886_H
#define ARDUINONRF_IMU_MPU6886_H

#include "../MPU6500/MPU6500.h"

namespace nimu {

class MPU6886 : public MPU6500 {
 public:
  static constexpr uint8_t kWhoAmI = 0x19;

  MPU6886() { name_ = "MPU6886"; }

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

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MPU6886;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_MPU6886_H
