/*
  GY9250.h - ArduinoNRF-IMU board class for the GY-9250 9-DOF breakout.

  The GY-9250 is a single InvenSense MPU-9250 (accel + gyro + AK8963
  magnetometer) on a breakout board, so this board class is simply the MPU9250
  driver under the board's own name. Use whichever name reads better in your
  sketch - GY9250 (the board you bought) or MPU9250 (the chip on it); they are
  the same object with the same API.

  Quick start:

      #include <GY9250.h>
      GY9250 board;

      void setup() {
        Serial.begin(115200);
        board.begin();              // Wire @ 0x68, mag on (if the AK8963 is real)
        board.calibrateGyro();      // hold still
      }

      void loop() {
        board.update();
        Vec3 a = board.accelG();
        Vec3 g = board.gyroDps();
        Vec3 m = board.magUT();
        delay(50);
      }
*/
#ifndef ARDUINONRF_IMU_GY9250_H
#define ARDUINONRF_IMU_GY9250_H

#include "../../sensors/MPU9250/MPU9250.h"

namespace nimu {

/** The GY-9250 board: an MPU-9250 under the breakout's name. */
class GY9250 : public MPU9250 {
 public:
  GY9250() { name_ = "GY-9250"; }
};

}  // namespace nimu

using nimu::GY9250;

#endif  // ARDUINONRF_IMU_GY9250_H
