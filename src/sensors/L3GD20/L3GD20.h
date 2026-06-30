/* L3GD20.h - ST L3GD20/L3GD20H gyroscope driver. */
#ifndef ARDUINONRF_IMU_L3GD20_H
#define ARDUINONRF_IMU_L3GD20_H

#include "../L3G4200D/L3G4200D.h"

namespace nimu {

class L3GD20 : public L3G4200D {
 public:
  static constexpr uint8_t kAddrLow = 0x6A;
  static constexpr uint8_t kAddrHigh = 0x6B;
  static constexpr uint8_t kWhoAmI = 0xD4;
  static constexpr uint8_t kWhoAmIH = 0xD7;

  L3GD20() { name_ = "L3GD20"; }

  bool begin() override {
    if (beginI2C(Wire, kAddrHigh)) return true;
    return beginI2C(Wire, kAddrLow);
  }
  bool beginI2C(TwoWire& wire, uint8_t address) override {
    return L3G4200D::beginI2C(wire, address);
  }
  bool isConnected() override {
    uint8_t id = whoAmI();
    return id == kWhoAmI || id == kWhoAmIH;
  }
};

}  // namespace nimu

using nimu::L3GD20;

#endif
