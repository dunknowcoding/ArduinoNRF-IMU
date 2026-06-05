/*
  MPU6500.h - NiusIMU driver for the InvenSense MPU-6500 6-axis IMU
  (accelerometer + gyroscope + die temperature).

  The MPU-6500 is the chip on many GY-91 boards and the inertial core of the
  MPU-9250. This driver does everything except the magnetometer; the MPU9250
  class derives from it and adds the AK8963. Use MPU6500 directly when your
  board has no magnetometer (a plain MPU-6500, WHO_AM_I 0x70).

  Quick start (I2C, GY-91 default wiring):

      #include <MPU6500.h>
      MPU6500 imu;

      void setup() {
        Serial.begin(115200);
        imu.begin();                 // Wire @ 0x68, sensible defaults
        imu.calibrateGyro();         // keep the board still for a second
      }

      void loop() {
        imu.update();
        Vec3 a = imu.accelG();       // g
        Vec3 g = imu.gyroDps();      // deg/s
        delay(50);
      }
*/
#ifndef NIUSIMU_MPU6500_H
#define NIUSIMU_MPU6500_H

#include "../../imu/IMUSensor.h"
#include "MPU6500_Registers.h"

namespace nimu {

class MPU6500 : public IMUSensor {
 public:
  MPU6500() { name_ = "MPU6500"; }

  // FSYNC routing targets for setExternalSync(), exposed on the class so a
  // sketch can write MPU6500::FSYNC_TEMP without the internal namespace.
  static constexpr uint8_t FSYNC_OFF = mpu6500::EXT_SYNC_DISABLED;
  static constexpr uint8_t FSYNC_TEMP = mpu6500::EXT_SYNC_TEMP_OUT;
  static constexpr uint8_t FSYNC_GYRO_X = mpu6500::EXT_SYNC_GYRO_XOUT;
  static constexpr uint8_t FSYNC_GYRO_Y = mpu6500::EXT_SYNC_GYRO_YOUT;
  static constexpr uint8_t FSYNC_GYRO_Z = mpu6500::EXT_SYNC_GYRO_ZOUT;
  static constexpr uint8_t FSYNC_ACCEL_X = mpu6500::EXT_SYNC_ACCEL_XOUT;
  static constexpr uint8_t FSYNC_ACCEL_Y = mpu6500::EXT_SYNC_ACCEL_YOUT;
  static constexpr uint8_t FSYNC_ACCEL_Z = mpu6500::EXT_SYNC_ACCEL_ZOUT;

  // ----------------------------------------------------- unified interface --
  bool begin() override;  ///< Wire @ 0x68, +/-4 g, +/-500 dps, 100 Hz
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;

  // ---------------------------------------------------------- chip extras ---

  /**
   * Write the raw 3-bit gyro/temperature DLPF_CFG field (CONFIG register).
   * Prefer setLowPassFilterHz(); use this only when you want an exact code.
   * @param cfg 0..7 (see datasheet table 5.2).
   */
  bool setGyroDlpfConfig(uint8_t cfg);

  /** Write the raw 3-bit accel A_DLPF_CFG field (ACCEL_CONFIG2 register). */
  bool setAccelDlpfConfig(uint8_t cfg);

  /**
   * Route a data-ready pulse to the INT pin. Combine with an interrupt on the
   * MCU side to read only when a fresh sample exists.
   * @param enable  true to drive INT on each new sample.
   * @param latch   true keeps INT asserted until INT_STATUS is read.
   */
  bool setDataReadyInterrupt(bool enable, bool latch = false);

  /** True if INT_STATUS reports a new sample is ready. */
  bool dataReady();

  /** Put the chip to sleep (low power, no measurements) or wake it. */
  bool sleep(bool enable = true);

  /** Full device reset, then re-apply the active configuration. */
  bool reset();

  // ----------------------------------------------------------- bus speed ---

  /**
   * Change the bus clock at runtime. On SPI this lets you configure the chip at
   * a safe 1 MHz and then burst sensor data far faster (the MPU tolerates up to
   * ~20 MHz for data reads, but only ~1 MHz for register writes - so set this
   * high only for reading, or restore it before changing configuration).
   */
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  // -------------------------------------------------------------- FSYNC -----

  /**
   * Route the FSYNC pin into the LSB of one sensor output (an MPU6500::FSYNC_*
   * target, or FSYNC_OFF). After each update() the captured FSYNC level is then
   * available from fsyncLevel(); use it to time-stamp an external event against
   * the IMU samples. Note: the chosen axis loses its least-significant bit.
   */
  bool setExternalSync(uint8_t fsyncTarget);

  /** FSYNC level captured by the last update(): 0 or 1, or -1 if FSYNC is off. */
  int fsyncLevel() const { return fsyncBit_; }

  // -- Raw 16-bit register values (no scaling/calibration), for diagnostics. --
  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
    int16_t mx, my, mz;  // zero on a 6-axis part; filled by the MPU9250 driver
    bool magValid;
  };
  /** Read the unscaled register values directly - useful for bus debugging. */
  virtual bool readRaw(RawSample& out);

 protected:
  bool configureDefaults();

  // Extension points the MPU9250 (9-axis) driver overrides to add the AK8963.
  // The base MPU-6500 leaves them empty, so it never touches the aux bus.
  /** Called at the end of a successful begin*(); init extra sensors here. */
  virtual bool initExtras() { return true; }
  /** Called by update() after accel/gyro/temp; fill data_.mag here. */
  virtual void processExtra(const RawSample& raw) { (void)raw; }

  TwoWire* i2cWire_ = nullptr;  ///< non-null in I2C mode (the 9-axis mag needs it)
  uint32_t clockHz_ = 400000;

  // Cached scale factors so update() avoids re-reading config registers.
  float accelLsbPerG_ = 8192.0f;  // default +/-4 g
  float gyroLsbPerDps_ = 65.5f;   // default +/-500 dps
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;

  uint8_t extSyncTarget_ = mpu6500::EXT_SYNC_DISABLED;  // FSYNC routing, if any
  int8_t fsyncBit_ = -1;  // FSYNC level from the last update() (-1 = off)
};

}  // namespace nimu

// Bring the common types into the global namespace so beginner sketches can
// write `MPU6500`, `Vec3`, `IMUData` without a namespace prefix.
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MPU6500;
using nimu::Vec3;

#endif  // NIUSIMU_MPU6500_H
