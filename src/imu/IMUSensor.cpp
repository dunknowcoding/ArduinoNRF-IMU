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

  // The board is still in SOME orientation: exactly one axis carries gravity
  // (+/-1 g) and the other two read ~0. Detect the dominant (gravity) axis
  // instead of assuming Z-up, so calibration is correct however the board is
  // mounted - e.g. a board whose Z points down reads -1 g, not +1 g.
  const Vec3 avg{sum.x / taken, sum.y / taken, sum.z / taken};
  const float ax = (avg.x < 0.0f) ? -avg.x : avg.x;
  const float ay = (avg.y < 0.0f) ? -avg.y : avg.y;
  const float az = (avg.z < 0.0f) ? -avg.z : avg.z;
  Vec3 expected{0.0f, 0.0f, 0.0f};
  if (ax >= ay && ax >= az) {
    expected.x = (avg.x >= 0.0f) ? 1.0f : -1.0f;
  } else if (ay >= az) {
    expected.y = (avg.y >= 0.0f) ? 1.0f : -1.0f;
  } else {
    expected.z = (avg.z >= 0.0f) ? 1.0f : -1.0f;
  }
  cal_.accelBias =
      Vec3{avg.x - expected.x, avg.y - expected.y, avg.z - expected.z};
  cal_.accelScale = Vec3{1, 1, 1};
  return true;
}

}  // namespace nimu
