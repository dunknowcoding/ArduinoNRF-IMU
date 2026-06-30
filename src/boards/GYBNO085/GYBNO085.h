/* Board alias for the purple GY-BNO085 marketplace module. */
#ifndef ARDUINONRF_IMU_GYBNO085_H
#define ARDUINONRF_IMU_GYBNO085_H

#include "../../sensors/BNO08x/BNO08x.h"

namespace nimu {

/*
  Exact ten-pad silk observed on this board family:
    VCC_3V3, GND, SCL/SCK/RX, SDA/MISO/TX, ADDR/MOSI,
    CS, INT, RST, PS1, PS0

  I2C mode is PS1=LOW, PS0=LOW. ADDR low selects 0x4A; otherwise 0x4B.
  This alias does not claim compatibility with differently labelled BNO08x PCBs.
*/
class GYBNO085 : public BNO08x {
 public:
  GYBNO085() { name_ = "GY-BNO085"; }
};

}  // namespace nimu

using nimu::GYBNO085;

#endif
