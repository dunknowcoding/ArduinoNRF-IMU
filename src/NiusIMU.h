/*
  NiusIMU.h - Umbrella header for the NiusIMU library.

  A unified, friendly IMU library for the ArduinoNRF board package. Each chip
  lives under src/sensors/ and each multi-chip breakout under src/boards/; the
  inertial sensors share one interface (nimu::IMUSensor) so your code reads the
  same regardless of which one is fitted.

  You normally include the specific board or sensor you have, e.g.:

      #include <GY91.h>     // GY-91 board: MPU-9250/6500 + BMP280
      #include <GY9250.h>   // GY-9250 board: MPU-9250 (9-axis)
      #include <MPU9250.h>  // bare MPU-9250 chip (9-axis)
      #include <MPU6500.h>  // bare MPU-6500 chip (6-axis, no magnetometer)
      #include <BMP280.h>   // bare BMP280 barometer (pressure + temperature)

  Include THIS header only if you want the framework pieces (types, the bus
  helper, the base class) without a particular sensor - for instance when
  writing your own driver. See docs/ADDING_A_SENSOR.md.
*/
#ifndef NIUSIMU_H
#define NIUSIMU_H

#define NIUSIMU_VERSION_MAJOR 0
#define NIUSIMU_VERSION_MINOR 2
#define NIUSIMU_VERSION_PATCH 0
#define NIUSIMU_VERSION "0.2.0"

#include "imu/IMUBus.h"
#include "imu/IMUSensor.h"
#include "imu/IMUTypes.h"

#endif  // NIUSIMU_H
