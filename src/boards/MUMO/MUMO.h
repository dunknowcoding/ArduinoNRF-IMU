/* Board alias for the open MUMO ICM-45686 + QMC6309 module. */
#ifndef ARDUINONRF_IMU_MUMO_H
#define ARDUINONRF_IMU_MUMO_H

#include "../../sensors/ICM45686/ICM45686.h"

namespace nimu {

/*
  Exact MUMO PCB silk: SCL, INT, CS, GND, +3V3, OSDO, OCS, CLK, SDA, CLK_CTL.
  OSDO/OCS are the auxiliary/OIS interface. CLK is ICM INT2 on this PCB.
*/
class MUMO : public ICM45686 {
 public:
  MUMO() { name_ = "MUMO"; }

  bool begin() override {
    if (!ICM45686::beginI2C(Wire, 0x68)) return false;
    return configureQMC6309();
  }

  bool beginI2C(TwoWire& wire, uint8_t address) override {
    if (!ICM45686::beginI2C(wire, address)) return false;
    return configureQMC6309();
  }
};

}  // namespace nimu

using nimu::MUMO;

#endif
