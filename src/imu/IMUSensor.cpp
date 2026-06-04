#include "IMUSensor.h"

namespace nimu {

// Generic calibration that works for ANY driver: we momentarily zero the
// relevant calibration term, average raw readings, then store the result as the
// new bias. Because update() applies (raw - bias) * scale, reading with
// bias = 0 and scale = 1 hands us the raw value back.

bool IMUSensor::calibrateGyro(uint16_t samples) {
  if (samples == 0) {
    return false;
  }
  const Vec3 savedBias = cal_.gyroBias;
  cal_.gyroBias = Vec3{0, 0, 0};

  Vec3 sum;
  uint16_t taken = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    if (!update()) {
      continue;
    }
    Vec3 g = gyroDps();
    sum.x += g.x;
    sum.y += g.y;
    sum.z += g.z;
    ++taken;
    delay(3);
  }
  if (taken == 0) {
    cal_.gyroBias = savedBias;  // nothing read - leave calibration untouched
    return false;
  }

  cal_.gyroBias = Vec3{sum.x / taken, sum.y / taken, sum.z / taken};
  return true;
}

bool IMUSensor::calibrateAccel(uint16_t samples) {
  if (samples == 0) {
    return false;
  }
  const Vec3 savedBias = cal_.accelBias;
  const Vec3 savedScale = cal_.accelScale;
  cal_.accelBias = Vec3{0, 0, 0};
  cal_.accelScale = Vec3{1, 1, 1};

  Vec3 sum;
  uint16_t taken = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    if (!update()) {
      continue;
    }
    Vec3 a = accelG();
    sum.x += a.x;
    sum.y += a.y;
    sum.z += a.z;
    ++taken;
    delay(3);
  }
  if (taken == 0) {
    cal_.accelBias = savedBias;
    cal_.accelScale = savedScale;
    return false;
  }

  // Board assumed level and still: X,Y should read 0 g and Z should read +1 g.
  cal_.accelBias =
      Vec3{sum.x / taken, sum.y / taken, (sum.z / taken) - 1.0f};
  cal_.accelScale = Vec3{1, 1, 1};
  return true;
}

}  // namespace nimu
