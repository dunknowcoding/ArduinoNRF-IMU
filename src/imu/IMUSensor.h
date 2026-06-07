/*
  IMUSensor.h - The unified IMU interface.

  Every sensor in this library (MPU9250 today, more later) derives from
  IMUSensor and therefore exposes the SAME high-level API. Your sketch reads
  acceleration the same way no matter which chip is on the board:

      sensor.update();
      Vec3 a = sensor.accelG();

  Design rules that keep all IMUs "equal citizens":
    * Common behaviour lives here as ordinary (non-virtual) methods so it is
      identical everywhere - units, calibration maths, the cached snapshot.
    * Things every IMU CAN do but does differently are virtual hooks the driver
      fills in (configure ranges, read a sample, run a self-test).
    * Anything a single chip uniquely offers (e.g. the MPU's on-board DMP) is
      NOT forced into this interface - the driver adds its own extra methods.

  A driver's job is therefore small: implement the pure-virtual hooks, and on
  each read store engineering-unit values into `data_`. This base class applies
  calibration and serves every accessor and unit conversion.
*/
#ifndef ARDUINONRF_IMU_IMUSENSOR_H
#define ARDUINONRF_IMU_IMUSENSOR_H

#include <Arduino.h>

#include "IMUBus.h"
#include "IMUTypes.h"

namespace nimu {

class IMUSensor {
 public:
  virtual ~IMUSensor() = default;

  // ------------------------------------------------------------ lifecycle --

  /**
   * Bring the sensor up on its default I2C bus and address, applying sane
   * defaults (a usable range, output data rate and filter). Returns true once
   * the chip is identified and configured.
   *
   * Most users call this no-argument form. The overloads below let you pick a
   * different bus, address, or wire it up for SPI.
   */
  virtual bool begin() = 0;

  /** Begin on a specific I2C bus / address (e.g. Wire1, or the AD0=1 address). */
  virtual bool beginI2C(TwoWire& wire, uint8_t address) = 0;

  /** Begin in SPI mode using the given chip-select pin. */
  virtual bool beginSPI(SPIClass& spi, uint8_t csPin) = 0;

  /** Read the chip id register. */
  virtual uint8_t whoAmI() = 0;

  /** True if WHO_AM_I matches and the chip is responding on the bus. */
  virtual bool isConnected() = 0;

  // -------------------------------------------------------------- reading --

  /**
   * Fetch one fresh synchronized sample (accel + gyro + temp, plus mag if the
   * chip has one) into the cached snapshot. Returns true on success.
   * Every accessor below simply returns values from this snapshot.
   */
  virtual bool update() = 0;

  /** The whole last snapshot at once. */
  const IMUData& data() const { return data_; }

  Vec3 accelG() const { return data_.accel; }                 ///< g
  Vec3 accelMs2() const { return scale(data_.accel, kGravityMs2); }
  Vec3 gyroDps() const { return data_.gyro; }                 ///< deg/s
  Vec3 gyroRps() const { return scale(data_.gyro, kDegToRad); }
  Vec3 magUT() const { return data_.mag; }                    ///< uT (0 if none)
  float temperatureC() const { return data_.temperature; }
  uint32_t timestamp() const { return data_.timestamp; }

  // --------------------------------------------------------- capabilities --

  /** Short device name, e.g. "MPU9250". */
  const char* name() const { return name_; }

  /** True if this chip can measure a magnetic field. */
  bool hasMagnetometer() const { return hasMag_; }

  // ------------------------------------------------------- configuration ---
  //
  // These take plain engineering values (g, deg/s, Hz). A driver rounds UP to
  // the nearest hardware setting it supports and returns false only if the
  // request cannot be met at all. Override per chip.

  /** Full-scale accel range, e.g. 2/4/8/16 g. */
  virtual bool setAccelRangeG(uint16_t maxG) = 0;

  /** Full-scale gyro range, e.g. 250/500/1000/2000 deg/s. */
  virtual bool setGyroRangeDps(uint16_t maxDps) = 0;

  /** Internal low-pass (anti-alias) bandwidth in Hz. */
  virtual bool setLowPassFilterHz(uint16_t hz) = 0;

  /** Output/sample data rate in Hz. */
  virtual bool setSampleRateHz(uint16_t hz) = 0;

  // ---------------------------------------------------------- calibration --
  //
  // The simple, common path: hold the board still and call calibrateGyro();
  // optionally calibrateAccel() with the board level. Magnetometer calibration
  // needs a figure-8 motion and is offered where the chip has a magnetometer.
  // Capture getCalibration(), print it, and restore with setCalibration() so
  // you only ever calibrate once.

  /** Measure and store gyro zero-rate bias. Keep the board perfectly still. */
  virtual bool calibrateGyro(uint16_t samples = 200);

  /**
   * Measure accelerometer bias with the board resting still in any orientation.
   * Auto-detects which axis carries gravity (so it works whether the board is
   * Z-up, Z-down, on its side, ...) and stores per-axis bias (scale stays 1.0).
   */
  virtual bool calibrateAccel(uint16_t samples = 200);

  /**
   * Magnetometer hard/soft-iron calibration. Rotate the board slowly through
   * all orientations (figure-8) for the whole duration. Returns false on chips
   * with no magnetometer. Override where supported.
   */
  virtual bool calibrateMag(uint16_t durationMs = 15000) {
    (void)durationMs;
    return false;
  }

  /** Current calibration (copy it to EEPROM / print it). */
  IMUCalibration getCalibration() const { return cal_; }

  /** Restore a previously captured calibration. */
  void setCalibration(const IMUCalibration& cal) { cal_ = cal; }

  /** Reset calibration to identity (no bias, unit scale). */
  void clearCalibration() { cal_ = IMUCalibration{}; }

 protected:
  // -- Helpers a driver uses while implementing the hooks above. --

  /** Multiply a vector by a scalar (used for unit conversions). */
  static Vec3 scale(const Vec3& v, float k) {
    return Vec3{v.x * k, v.y * k, v.z * k};
  }

  /** Apply (raw - bias) * scale, the library's standard correction model. */
  static Vec3 correct(const Vec3& raw, const Vec3& bias, const Vec3& gain) {
    return Vec3{(raw.x - bias.x) * gain.x, (raw.y - bias.y) * gain.y,
                (raw.z - bias.z) * gain.z};
  }

  IMUBus bus_;             ///< the only path to the chip's registers
  IMUData data_;           ///< latest snapshot, filled by update()
  IMUCalibration cal_;     ///< active calibration, applied by the driver
  const char* name_ = "IMU";
  bool hasMag_ = false;
};

}  // namespace nimu

#endif  // ARDUINONRF_IMU_IMUSENSOR_H
