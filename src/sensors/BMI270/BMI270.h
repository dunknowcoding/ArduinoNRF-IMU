/*
  BMI270.h - ArduinoNRF-IMU driver for Bosch BMI270 6-axis IMU.
*/
#ifndef ARDUINONRF_IMU_BMI270_H
#define ARDUINONRF_IMU_BMI270_H

#include "../../imu/IMUSensor.h"
#include "BMI270_Config.h"
#include "BMI270_Registers.h"

namespace nimu {

class BMI270 : public IMUSensor {
 public:
  BMI270() { name_ = "BMI270"; useDefaultConfigImage(); }

  bool begin() override;
  bool beginI2C(TwoWire& wire, uint8_t address) override;

  // Supply a custom configuration image instead of the bundled standard Bosch
  // image. The pointer must stay valid for the life of the object.
  void setConfigImage(const uint8_t* image, size_t length) {
    configImage_ = image;
    configImageLength_ = length;
    configImageProgmem_ = false;
  }

  // AVR callers whose custom image is in PROGMEM use this form.
  void setConfigImageProgmem(const uint8_t* image, size_t length) {
    configImage_ = image;
    configImageLength_ = length;
    configImageProgmem_ = true;
  }

  void useDefaultConfigImage() {
    configImage_ = bmi270::kDefaultConfigImage;
    configImageLength_ = bmi270::kDefaultConfigImageLength;
    configImageProgmem_ = true;
  }

  // Which stage of bring-up failed: identity, reset, configuration transfer,
  // configuration status, or default setup.
  enum class Stage : uint8_t {
    None,
    NotConnected,     // WHO_AM_I did not read back 0x24
    ResetFailed,      // soft reset issued, chip did not come back
    ConfigFileStub,   // no usable configuration image is selected
    ConfigUpload,     // the configuration blob did not transfer
    ConfigNotLoaded,  // blob sent, but INTERNAL_STATUS never reported ready
    Defaults          // configured, but the default settings would not apply
  };
  // Optional bring-up and recovery trace.
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

  // How long update() may spend restoring the configured state and waiting
  // for a fresh sample. Set it to zero to fail immediately instead.
  void setWakeTimeoutMs(uint16_t ms) { wakeTimeoutMs_ = ms; }
  uint16_t wakeTimeoutMs() const { return wakeTimeoutMs_; }

  // ---- Advanced features -------------------------------------------------
  //
  // All of these run on the BMI270's internal core, so none of them exist
  // until a configuration image has been loaded - see setConfigImage() - and
  // every call below fails cleanly if begin() did not reach init_ok.
  //
  // Their register layout is not in the datasheet. FEATURES is a sixteen-byte
  // window onto one of eight pages, and which page each feature lives on comes
  // from the tables in Bosch's own bmi270.c; see BMI270_Registers.h.

  enum class Feature : uint8_t {
    AnyMotion,
    NoMotion,
    SignificantMotion,
    StepDetector,
    StepCounter,
    StepActivity,
    WristGesture,
    WristWearWakeup
  };

  // Turn a feature on or off. The accelerometer must be running for any of
  // them to do anything.
  bool enableFeature(Feature feature, bool enable = true);

  // Steps since the counter was last reset. Meaningless unless StepCounter is
  // enabled; the counter needs a few strides before it reports anything at
  // all, which is the algorithm being careful rather than a fault.
  uint32_t stepCount();
  bool resetStepCount();

  enum class Activity : uint8_t { Still = 0, Walking = 1, Running = 2, Unknown = 3 };
  Activity stepActivity();

  // Any-motion fires while acceleration on any selected axis stays above the
  // threshold for the duration; no-motion fires while it stays below.
  //
  // Threshold is in milli-g and duration in milliseconds. Both are quantised
  // by the hardware: the threshold to 1/2048 g steps, the duration to 20 ms
  // steps, with 11 and 13 bits respectively.
  bool configureAnyMotion(uint16_t thresholdMg, uint16_t durationMs,
                          bool x = true, bool y = true, bool z = true);
  bool configureNoMotion(uint16_t thresholdMg, uint16_t durationMs,
                         bool x = true, bool y = true, bool z = true);

  // Which features have fired since this was last called. Cleared by reading,
  // so call it once per pass and keep the result. Test it against the
  // FEAT_INT_* masks in BMI270_Registers.h.
  uint8_t featureInterrupts();

  // Route feature interrupts to a physical pin. pin is 1 or 2, mask is any
  // combination of the FEAT_INT_* values. Call configureInterruptPin() first
  // to set that pin's electrical behaviour.
  bool mapFeatureInterrupt(uint8_t pin, uint8_t mask);

  // ---- FIFO ---------------------------------------------------------------

  // Header mode tags each frame with what it contains and is what you want
  // unless you have a reason otherwise; headerless mode packs frames tighter
  // but you must know the enabled sensors to parse it.
  bool configureFifo(bool accel, bool gyro, bool headerMode = true);
  uint16_t fifoLength();
  uint16_t readFifo(uint8_t* out, uint16_t maxBytes);
  bool flushFifo();

  // ---- Offsets and self-test ----------------------------------------------

  // Accelerometer offsets are 8-bit two's complement, 3.9 mg per count.
  bool setAccelOffset(int8_t x, int8_t y, int8_t z, bool enable = true);

  // Gyroscope offsets are 10-bit two's complement, 0.061 dps per count.
  bool setGyroOffset(int16_t x, int16_t y, int16_t z, bool enable = true);

  // Drives the accelerometer against its own internal test signal and checks
  // the deflection is within the datasheet's limits. Disturbs the sensor
  // configuration, so it re-applies the defaults afterwards; keep the board
  // still while it runs.
  bool selfTestAccel();

  // Reorder and invert the axes in hardware, so everything downstream -
  // including the step counter and motion detectors - sees the board the way
  // it is actually mounted. Each axis takes 0, 1 or 2 for x, y or z.
  bool remapAxes(uint8_t xTo, bool xInvert, uint8_t yTo, bool yInvert,
                 uint8_t zTo, bool zInvert);

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
    uint32_t sensorTime;
  };
  bool readRaw(RawSample& out);
  uint32_t sensorTimeTicks() const { return lastSensorTime_; }

 private:
  Stage lastStage_ = Stage::None;
  uint16_t wakeTimeoutMs_ = 300;
  Print* debug_ = nullptr;
  uint8_t accConf_ = 0xA8;    // 100 Hz, normal filter - the reset default
  uint8_t accRange_ = 0x02;   // +/- 8 g
  uint8_t gyrConf_ = 0xA9;    // 200 Hz
  uint8_t gyrRange_ = 0x00;   // +/- 2000 dps
  uint8_t intMapData_ = 0x00; // what routeDataReadyInterrupt() has mapped
  uint8_t lastInternalStatus_ = 0xFF;
  uint32_t lastSensorTime_ = 0;

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

  // Read a whole sixteen-byte FEATURES page, and write one back. Feature
  // configuration is always read-modify-write of the entire page, which keeps
  // every write 16-bit word aligned as section 4.8.1 requires.
  bool readFeaturePage(uint8_t page, uint8_t* out);
  bool writeFeaturePage(uint8_t page, const uint8_t* in);
  bool setFeatureBit(uint8_t page, uint8_t index, uint8_t mask, bool enable);
  bool configureMotion(uint8_t page, uint8_t offset, uint16_t thresholdMg,
                       uint16_t durationMs, bool x, bool y, bool z);

  // Reads acceleration, angular rate, sensor time and temperature.
  bool readSampleRegisters(uint8_t* a, uint8_t* g, uint8_t* t,
                           uint32_t& sensorTime);

  // True when a sample is the all-zero, invalid-temperature signature of a
  // part that is not running.
  static bool sampleIsDead(const uint8_t* a, const uint8_t* g,
                           const uint8_t* t);
  const uint8_t* configImage_ = nullptr;
  size_t configImageLength_ = 0;
  bool configImageProgmem_ = false;
  uint8_t configByte(size_t offset) const;
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
