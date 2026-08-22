/*
  BMI270.h - ArduinoNRF-IMU driver for Bosch BMI270 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_BMI270_H
#define ARDUINONRF_IMU_BMI270_H

#include "../../imu/IMUSensor.h"
#include "BMI270_Registers.h"

namespace nimu {

class BMI270 : public IMUSensor {
 public:
  BMI270() { name_ = "BMI270"; }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;

  // Supply Bosch's 8192-byte configuration image. Required before begin().
  //
  // This library does not redistribute other vendors' firmware, so the image
  // is yours to provide - see BMI270_Config.h for where to get it and what
  // BSD-3-Clause asks of you. The pointer must stay valid for the life of the
  // object; pointing it at a PROGMEM array will not work, the bytes have to be
  // readable directly.
  void setConfigImage(const uint8_t* image, size_t length) {
    configImage_ = image;
    configImageLength_ = length;
  }

  // Which stage of bring-up failed. begin() returning false says nothing about
  // whether the part is absent, refused its identity check, or choked part way
  // through the 8 KB configuration upload - and those need different fixes.
  enum class Stage : uint8_t {
    None,
    NotConnected,     // WHO_AM_I did not read back 0x24
    ResetFailed,      // soft reset issued, chip did not come back
    ConfigFileStub,   // the bundled configuration image is a placeholder
    ConfigUpload,     // the configuration blob did not transfer
    ConfigNotLoaded,  // blob sent, but INTERNAL_STATUS never reported ready
    Defaults          // configured, but the default settings would not apply
  };
  Stage lastStage() const { return lastStage_; }
  const char* lastStageText() const;

  // The raw INTERNAL_STATUS byte from the last bring-up attempt. Bits [3:0]
  // are a message field: 0 not_init, 1 init_ok, 2 init_err, 3 drv_err,
  // 4 sns_stop, 5 nvm_error.
  //
  // Worth reading when begin() fails. not_init means the part never started
  // initialising - the configuration image never reached it. init_err means it
  // received an image and rejected it, which is a different problem entirely.
  uint8_t internalStatus() const { return lastInternalStatus_; }

  // The BMI270's step counter, gesture and motion detectors all run on its
  // internal core, so none of them exist until a configuration image has been
  // loaded - see setConfigImage(). There is no API for them here because there
  // is nothing to talk to without that image.

  bool beginSPI(SPIClass& spi, uint8_t csPin) override;
  uint8_t whoAmI() override;
  bool isConnected() override;
  bool update() override;

  bool setAccelRangeG(uint16_t maxG) override;
  bool setGyroRangeDps(uint16_t maxDps) override;
  bool setLowPassFilterHz(uint16_t hz) override;
  bool setSampleRateHz(uint16_t hz) override;

  bool reset();
  bool dataReady();
  bool configurationLoaded();
  bool configureInterruptPin(uint8_t pin, bool activeHigh, bool openDrain,
                             bool latched = false);
  bool routeDataReadyInterrupt(uint8_t pin, bool enable = true);
  void setBusClockHz(uint32_t hz);
  uint32_t busClockHz() const { return clockHz_; }

  struct RawSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
  };
  bool readRaw(RawSample& out);

 private:
  Stage lastStage_ = Stage::None;
  uint8_t lastInternalStatus_ = 0xFF;
  const uint8_t* configImage_ = nullptr;
  size_t configImageLength_ = 0;
  bool configureDefaults();
  bool uploadConfiguration();
  bool writeConfigChunk(uint16_t index, const uint8_t* data, uint16_t len);
  uint8_t odrCodeForHz(uint16_t hz, uint16_t& actualHz, bool gyro) const;

  uint32_t clockHz_ = 400000;
  float accelLsbPerG_ = 8192.0f;
  float gyroLsbPerDps_ = 65.536f;
  uint8_t accelOdr_ = 0x08;
  uint8_t gyroOdr_ = 0x08;
  uint16_t accelRangeG_ = 4;
  uint16_t gyroRangeDps_ = 500;
  uint16_t sampleRateHz_ = 100;
};

}  // namespace nimu

using nimu::BMI270;
using nimu::IMUCalibration;
using nimu::IMUData;
using nimu::Vec3;

#endif  // ARDUINONRF_IMU_BMI270_H
