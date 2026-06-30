/*
  MPU6050.h - ArduinoNRF-IMU driver for the InvenSense MPU-6050 6-axis IMU.

  The MPU-6050 is the sensor on the very common GY-521 module. It provides a
  3-axis accelerometer, 3-axis gyroscope and die temperature over I2C. This
  driver intentionally reads raw accel/gyro/temp only; the chip's DMP is left
  for a later chip-specific extension.
*/
#ifndef ARDUINONRF_IMU_MPU6050_H
#define ARDUINONRF_IMU_MPU6050_H

#include "../../imu/IMUSensor.h"
#include "MPU6050_Registers.h"

namespace nimu {

class MPU6050 : public IMUSensor {
 public:
  static constexpr uint8_t FSYNC_OFF = 0x00;
  static constexpr uint8_t FSYNC_TEMP = 0x08;
  static constexpr uint8_t FSYNC_GYRO_X = 0x10;
  static constexpr uint8_t FSYNC_GYRO_Y = 0x18;
  static constexpr uint8_t FSYNC_GYRO_Z = 0x20;
  static constexpr uint8_t FSYNC_ACCEL_X = 0x28;
  static constexpr uint8_t FSYNC_ACCEL_Y = 0x30;
  static constexpr uint8_t FSYNC_ACCEL_Z = 0x38;
  MPU6050() { name_ = "MPU6050"; }

  bool begin() override;  ///< Wire @ 0x68, fallback 0x69
  bool beginI2C(TwoWire& wire, uint8_t address) override;
  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;

  bool setGyroDlpfConfig(uint8_t cfg);
  bool setDataReadyInterrupt(bool enable, bool latch = false);
  bool configureInterruptPin(bool activeLow, bool openDrain,
                             bool latched = false,
                             bool clearOnAnyRead = false);
  bool setExternalSync(uint8_t target);
  bool dataReady();
  bool sleep(bool enable = true);
  bool reset();

  /**
   * Expose the MPU-6050 auxiliary I2C pins on the host bus. GY-87-style boards
   * often put their compass behind these pins, so this is the bridge needed
   * before scanning for HMC5883L/QMC5883L.
   */
  bool setAuxI2CBypass(bool enable = true);

  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  bool configureDefaults();

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 8192.0f;  // default +/-4 g
  float gyroLsbPerDps_ = 65.5f;   // default +/-500 dps
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::MPU6050;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_MPU6050_H
