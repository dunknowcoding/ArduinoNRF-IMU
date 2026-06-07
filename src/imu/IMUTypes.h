/*
  IMUTypes.h - Common data types shared by every IMU in the ArduinoNRF-IMU library.

  These types are deliberately small, plain structs so they are cheap to copy
  and easy to print. Every concrete sensor driver (MPU9250, ICM20948, ...)
  speaks in exactly these units, so user code written against one IMU reads the
  same way against another.

  Units (the single source of truth for the whole library):
    - Acceleration .......... g            (1 g = 9.80665 m/s^2)
    - Angular rate .......... degrees / s  (deg/s, "dps")
    - Magnetic field ........ microtesla   (uT)
    - Temperature ........... degrees C
*/
#ifndef ARDUINONRF_IMU_IMUTYPES_H
#define ARDUINONRF_IMU_IMUTYPES_H

#include <Arduino.h>

// We need just one libm function. Including <math.h> here would pull in the C++
// <limits>/<algorithm> headers, which clash with Arduino's min()/max() macros in
// sketches, so we forward-declare the single symbol we use instead.
extern "C" float sqrtf(float);

namespace nimu {

/** Standard gravity, used to convert between g and m/s^2. */
static constexpr float kGravityMs2 = 9.80665f;
/** Degrees -> radians factor. */
static constexpr float kDegToRad = 0.017453292519943295f;

/**
 * A 3-axis vector. The meaning of the axes follows the sensor's own body
 * frame as silk-screened on the breakout (X, Y, Z). The library does not
 * re-orient axes for you - what the chip reports is what you get.
 */
struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  /** Length of the vector (e.g. total acceleration magnitude). */
  float magnitude() const {
    return sqrtf(x * x + y * y + z * z);
  }
};

/**
 * One synchronized snapshot of every quantity an IMU can measure.
 * Magnetometer fields stay at zero on chips without a magnetometer
 * (check IMUSensor::hasMagnetometer()).
 */
struct IMUData {
  Vec3 accel;             ///< acceleration, g
  Vec3 gyro;              ///< angular rate, deg/s
  Vec3 mag;               ///< magnetic field, uT
  float temperature = 0;  ///< die temperature, degrees C
  uint32_t timestamp = 0; ///< micros() captured when the sample was read
};

/**
 * Persistent calibration for one sensor. Capture it once with the calibrate*
 * helpers, print it, and paste the numbers back via IMUSensor::setCalibration()
 * (or store it in EEPROM) so you never have to re-calibrate at every boot.
 *
 * Correction model applied on every sample:
 *     value_corrected = (raw - bias) * scale
 */
struct IMUCalibration {
  Vec3 accelBias;                    ///< g, subtracted from accel
  Vec3 accelScale{1.0f, 1.0f, 1.0f}; ///< unitless gain per axis
  Vec3 gyroBias;                     ///< deg/s, subtracted from gyro
  Vec3 magBias;                      ///< uT, hard-iron offset
  Vec3 magScale{1.0f, 1.0f, 1.0f};   ///< unitless soft-iron gain per axis

  /** Magic marker so EEPROM-restored calibration can be sanity-checked. */
  uint16_t magic = 0x4D49;           // 'MI'
  bool valid() const { return magic == 0x4D49; }
};

/** Result codes returned by the bus / driver layer. */
enum class IMUStatus : uint8_t {
  Ok = 0,
  BusError,        ///< I2C/SPI transfer failed (NACK, timeout, wiring)
  NotConnected,    ///< WHO_AM_I did not match the expected device id
  Unsupported,     ///< the chip does not implement this feature
  BadParameter,    ///< an argument was out of range
};

/** Human-readable text for an IMUStatus (handy in Serial diagnostics). */
inline const char* toString(IMUStatus s) {
  switch (s) {
    case IMUStatus::Ok:           return "Ok";
    case IMUStatus::BusError:     return "BusError";
    case IMUStatus::NotConnected: return "NotConnected";
    case IMUStatus::Unsupported:  return "Unsupported";
    case IMUStatus::BadParameter: return "BadParameter";
  }
  return "Unknown";
}

}  // namespace nimu

#endif  // ARDUINONRF_IMU_IMUTYPES_H
