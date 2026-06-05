/*
  MPU9250.h - NiusIMU driver for the InvenSense MPU-9250 (and the common
  AliExpress "GY-9250" breakout).

  The MPU-9250 is an MPU-6500 (accel + gyro) plus an AK8963 magnetometer, so
  this driver simply EXTENDS MPU6500 with the magnetometer. Everything inertial
  (ranges, filters, sample rate, gyro/accel calibration, sleep, interrupts)
  comes from MPU6500 unchanged; the extras declared here are magnetometer-only.

  Quick start (I2C, the GY-9250's default wiring):

      #include <MPU9250.h>
      MPU9250 imu;

      void setup() {
        Serial.begin(115200);
        imu.begin();                 // Wire @ 0x68, sensible defaults, mag on
        imu.calibrateGyro();         // keep the board still for a second
      }

      void loop() {
        imu.update();
        Vec3 a = imu.accelG();       // g
        Vec3 g = imu.gyroDps();      // deg/s
        Vec3 m = imu.magUT();        // uT (zero if the board has no AK8963)
        delay(50);
      }

  On a board that turns out to be a plain MPU-6500 (WHO_AM_I 0x70, no AK8963),
  begin() still succeeds; hasMagnetometer() is then false and magUT() reads 0.
*/
#ifndef NIUSIMU_MPU9250_H
#define NIUSIMU_MPU9250_H

#include "../MPU6500/MPU6500.h"
#include "MPU9250_Registers.h"

namespace nimu {

class MPU9250 : public MPU6500 {
 public:
  MPU9250() { name_ = "MPU9250"; }

  // --------------------------------------------------------- magnetometer ---

  /** AK8963 die id (0x48). 0 if the magnetometer is absent/unreachable. */
  uint8_t magWhoAmI();

  /**
   * Turn the magnetometer on or off. begin() enables it automatically on boards
   * that have one; this lets you save power or skip it on MPU-6500 clones.
   * @return false if no magnetometer is present.
   */
  bool enableMagnetometer(bool enable = true);

  /**
   * Magnetometer hard/soft-iron calibration. Rotate the board slowly through
   * every orientation (a figure-8 in the air) for the whole duration. Returns
   * false if there is no magnetometer or too little motion was seen.
   */
  bool calibrateMag(uint16_t durationMs = 15000) override;

  // ------------------------------------------ auxiliary I2C (EDA / ECL) -----
  //
  // The MPU-9250 has a second I2C bus on its EDA/ECL pins, driven by the chip's
  // own I2C master. The AK8963 magnetometer lives there, but on breakouts that
  // expose EDA/ECL you can wire your OWN I2C sensors to it and reach them
  // through the MPU - useful to hang extra sensors off one MCU bus, or to keep
  // them on an isolated bus. These work after begin() over I2C (begin enables
  // the master); they are unavailable in SPI mode.

  /** Read one register from a device on the aux bus. False on bus NACK. */
  bool auxReadRegister(uint8_t address, uint8_t reg, uint8_t& value);
  /** Write one register to a device on the aux bus. False on bus NACK. */
  bool auxWriteRegister(uint8_t address, uint8_t reg, uint8_t value);
  /** True if a device acknowledges its address on the aux bus. */
  bool auxPing(uint8_t address);

  // The unscaled magnetometer bytes also appear in MPU6500::RawSample via the
  // readRaw() override below, so MPU6500's readRaw/diagnostics keep working.
  bool readRaw(RawSample& out) override;

 protected:
  bool initExtras() override;             ///< bring up the AK8963 (master mode)
  void processExtra(const RawSample& raw) override;  ///< scale + map the mag

 private:
  bool initMagnetometer();
  bool readMagRaw(int16_t& mx, int16_t& my, int16_t& mz, bool& valid);

  // --- AK8963 access via the MPU's internal I2C master (never bypass) ---
  /** Wait for the SLV4 single transfer to finish; false on timeout or NACK. */
  bool magWaitSlv4();
  /** Single-byte write to an AK8963 register over the aux bus (via SLV4). */
  bool magWriteReg(uint8_t reg, uint8_t value);
  /** Single-byte read from an AK8963 register over the aux bus (via SLV4). */
  bool magReadReg(uint8_t reg, uint8_t& value);
  /** Point SLV0 at the AK8963 data block for automatic per-sample reads. */
  bool magConfigureSlv0(bool enable);

  Vec3 magAsa_{1.0f, 1.0f, 1.0f};  // AK8963 fuse-ROM sensitivity adjustment
  bool magPresent_ = false;
  bool magEnabled_ = false;
};

}  // namespace nimu

// Bring the common types into the global namespace so beginner sketches can
// write `MPU9250`, `Vec3`, `IMUData` without a namespace prefix.
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MPU9250;
using nimu::Vec3;

#endif  // NIUSIMU_MPU9250_H
