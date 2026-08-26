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
  // Optional bring-up and recovery trace. A BMI270 failure is almost never
  // visible from outside - the part answers, accepts writes, and then quietly
  // returns nothing - so point this at Serial and the sequence explains
  // itself.
  //
  //   imu.setDebugStream(&Serial);
  void setDebugStream(Print* out) { debug_ = out; }

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

  // True when the part power-on-reset itself while its core was being
  // started. Sampled during begin() when the configuration fails to take.
  //
  // This is the difference between a part that rejected the image and a part
  // that cannot run at all: the image is irrelevant if the die restarts the
  // moment INIT_CTRL is written. It does not, on its own, say why - see
  // losesStateWhenIdle(), which is the far more common reason.
  bool poweredDownDuringInit() const { return porDuringInit_; }

  // True when the part cannot hold a register value across an idle bus.
  //
  // Some BMI270 modules lose their whole register file - configuration RAM
  // included, so INTERNAL_STATUS falls back to not_init - within about 25 ms
  // of the I2C bus going quiet. They are not broken and they are not
  // necessarily mis-wired: driven without idle gaps they deliver correct data,
  // measured here at 1.009 g with normal gyro noise.
  //
  // update() recovers from this by itself, so this flag is diagnostic rather
  // than fatal. It is worth knowing about because it makes such a part look
  // dead under a debugger: any delay(), and even a single Serial.print between
  // configuring the sensor and reading it, is long enough to lose everything -
  // so the trace reports success and the sample after it reads zero.
  //
  // An earlier version of this called the same condition "not being supplied"
  // and told the caller to check VDD. That was a guess dressed as a diagnosis
  // and it sent the investigation the wrong way for two sessions. A supply
  // fault can certainly cause this, but so can a part that simply behaves this
  // way, and this test cannot tell them apart.
  bool losesStateWhenIdle() const { return losesStateWhenIdle_; }

  // How long update() may spend waking a sleeping part and waiting for it to
  // produce a sample. 150 ms by default; measured, a part that has dozed off
  // needs about 46 ms before drdy_acc sets.
  //
  // Only ever spent when a sample comes back dead, so a sensor that stays
  // awake never pays for it. Set it to zero to have update() fail immediately
  // instead of recovering.
  void setWakeTimeoutMs(uint16_t ms) { wakeTimeoutMs_ = ms; }
  uint16_t wakeTimeoutMs() const { return wakeTimeoutMs_; }

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
  // Accepted for interface compatibility, but the cut-off in Hz is ignored.
  //
  // The BMI270 has no configurable filter cut-off. ACC_CONF.acc_bwp and
  // GYR_CONF.gyr_bwp select an oversampling and averaging mode, not a
  // frequency, and the resulting bandwidth is a fixed fraction of whatever
  // output data rate is in force. Calling this enables the aliasing-free
  // performance filter on both sensors, which is the closest thing the part
  // offers; use setSampleRateHz() to change the bandwidth.
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
  bool porDuringInit_ = false;
  bool losesStateWhenIdle_ = false;
  uint16_t wakeTimeoutMs_ = 300;
  Print* debug_ = nullptr;
  uint8_t accConf_ = 0xA8;    // 100 Hz, normal filter - the reset default
  uint8_t accRange_ = 0x02;   // +/- 8 g
  uint8_t gyrConf_ = 0xA9;    // 200 Hz
  uint8_t gyrRange_ = 0x00;   // +/- 2000 dps
  uint8_t intMapData_ = 0x00; // what routeDataReadyInterrupt() has mapped
  uint8_t lastInternalStatus_ = 0xFF;

  // Writes a register, lets the bus go quiet, and reads it back. False when
  // the value did not survive, which means the die is not being supplied.
  bool holdsStateAcrossIdleBus();

  // Clears advanced power save and enables the given sensors in a single
  // transaction, so nothing can fall asleep between the two writes.
  bool wakeAndEnable(uint8_t sensors);

  // Re-applies the whole configuration to a part that has lost it, then waits
  // for real data. False if none arrives.
  bool resumeAfterSleep();

  // One line of optional trace output.
  void trace(const char* what, uint8_t value);

  // Wakes the part and polls until it has a sample, keeping the bus busy
  // throughout. False if nothing arrived within the timeout.
  bool wakeAndAwaitData(uint16_t timeoutMs);

  // Reads the accelerometer, gyroscope and temperature registers in one go.
  bool readSampleRegisters(uint8_t* a, uint8_t* g, uint8_t* t);

  // True when a sample is the all-zero, invalid-temperature signature of a
  // part that is not running.
  static bool sampleIsDead(const uint8_t* a, const uint8_t* g,
                           const uint8_t* t);
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
