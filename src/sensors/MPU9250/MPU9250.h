/*
  MPU9250.h - NiusIMU driver for the InvenSense MPU-9250 (and the common
  AliExpress "GY-9250" breakout).

  Quick start (I2C, the GY-9250's default wiring):

      #include <MPU9250.h>
      MPU9250 imu;

      void setup() {
        Serial.begin(115200);
        imu.begin();                 // Wire @ 0x68, sensible defaults
        imu.calibrateGyro();         // keep the board still for a second
      }

      void loop() {
        imu.update();
        Vec3 a = imu.accelG();       // g
        Vec3 g = imu.gyroDps();      // deg/s
        Vec3 m = imu.magUT();        // uT
        delay(50);
      }

  Everything in IMUSensor (units, calibration, snapshot) works here unchanged.
  Methods declared in THIS header on top of the base class are MPU-specific
  extras (magnetometer control, raw DLPF config, interrupt pin, sleep).
*/
#ifndef NIUSIMU_MPU9250_H
#define NIUSIMU_MPU9250_H

#include "../../imu/IMUSensor.h"
#include "MPU9250_Registers.h"

namespace nimu {

class MPU9250 : public IMUSensor {
 public:
  MPU9250() { name_ = "MPU9250"; }

  // ----------------------------------------------------- unified interface --
  bool begin() override;  ///< Wire @ 0x68, +/-4 g, +/-500 dps, 100 Hz, mag on
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;
  bool calibrateMag(uint16_t durationMs = 15000) override;

  // ------------------------------------------------------- MPU-only extras --

  /** AK8963 die id (0x48). 0 if the magnetometer is absent/unreachable. */
  uint8_t magWhoAmI();

  /**
   * Turn the magnetometer on or off. begin() enables it automatically on chips
   * that have one; this lets you save power or skip it on MPU-6500 clones.
   * @return false if no magnetometer is present.
   */
  bool enableMagnetometer(bool enable = true);

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

  // -- Raw 16-bit register values (no scaling/calibration), for diagnostics. --
  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
    int16_t mx, my, mz;
    bool magValid;
  };
  /** Read the unscaled register values directly - useful for bus debugging. */
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();
  bool initMagnetometer();
  bool readMagRaw(int16_t& mx, int16_t& my, int16_t& mz, bool& valid);

  // The AK8963 magnetometer sits at its own I2C address (0x0C), reached over
  // the same wires once the MPU's I2C bypass is on, so it gets its own bus.
  IMUBus magBus_;
  TwoWire* i2cWire_ = nullptr;
  uint32_t clockHz_ = 400000;

  // Cached scale factors so update() avoids re-reading config registers.
  float accelLsbPerG_ = 8192.0f;     // default +/-4 g
  float gyroLsbPerDps_ = 65.5f;      // default +/-500 dps
  Vec3 magAsa_{1.0f, 1.0f, 1.0f};    // AK8963 fuse-ROM sensitivity adjustment
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
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
