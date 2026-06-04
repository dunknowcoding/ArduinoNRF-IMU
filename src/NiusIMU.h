/*
  NiusIMU.h - Umbrella header for the NiusIMU library.

  A unified, friendly IMU library for the ArduinoNRF board package. Every
  supported sensor lives in its own folder under src/sensors/ and implements one
  common interface (nimu::IMUSensor), so your code reads the same regardless of
  which chip is on the board.

  You normally include the specific sensor you have, e.g.:

      #include <MPU9250.h>

  Include THIS header only if you want the framework pieces (types, the bus
  helper, the base class) without a particular sensor - for instance when
  writing your own driver. See docs/ADDING_A_SENSOR.md.
*/
#ifndef NIUSIMU_H
#define NIUSIMU_H

#define NIUSIMU_VERSION_MAJOR 0
#define NIUSIMU_VERSION_MINOR 1
#define NIUSIMU_VERSION_PATCH 0
#define NIUSIMU_VERSION "0.1.0"

#include "imu/IMUBus.h"
#include "imu/IMUSensor.h"
#include "imu/IMUTypes.h"

#endif  // NIUSIMU_H
