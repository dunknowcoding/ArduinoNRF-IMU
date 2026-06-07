/*
  MPU9250.h - Public include for the MPU-9250 / GY-9250 driver.

  This one-line header is the library's public entry point for this sensor. It
  simply forwards to the driver that lives in its own folder
  (src/sensors/MPU9250/). That folder-per-sensor layout is how new IMUs are
  "mounted": drop the driver in src/sensors/<NAME>/ and add a matching forwarder
  here at the src/ root so users can write `#include <NAME.h>`.
  See docs/ADDING_A_SENSOR.md.
*/
#ifndef ARDUINONRF_IMU_PUBLIC_MPU9250_H
#define ARDUINONRF_IMU_PUBLIC_MPU9250_H

#include "sensors/MPU9250/MPU9250.h"

#endif  // ARDUINONRF_IMU_PUBLIC_MPU9250_H
