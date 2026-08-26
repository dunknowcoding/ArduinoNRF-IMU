#include "BMI270.h"

namespace nimu {
using namespace bmi270;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool BMI270::begin() {
  // Find out which address answers before committing to a full bring-up.
  //
  // Trying the primary and falling through to the alternate meant every board
  // strapped to the alternate paid for a failed init first - a soft reset and
  // an 8 KB upload aimed at nothing - and, worse, the NACK from that attempt
  // leaves the controller briefly unwilling, so the identity read on the
  // correct address then failed too. The part was reported absent while
  // answering perfectly.
  //
  // A ping is one addressing phase and leaves no state behind.
  bus_.beginI2C(Wire, kAddrSDOLow, clockHz_);
  bus_.setPostTransactionDelayUs(450);
  const bool low = (bus_.ping() == IMUStatus::Ok);
  if (!low) {
    bus_.beginI2C(Wire, kAddrSDOHigh, clockHz_);
    if (bus_.ping() != IMUStatus::Ok) {
      lastStage_ = Stage::NotConnected;
      return false;   // neither address answers; nothing is there
    }
    return beginI2C(Wire, kAddrSDOHigh);
  }
  return beginI2C(Wire, kAddrSDOLow);
}

bool BMI270::beginI2C(TwoWire& wire, uint8_t address) {
  lastStage_ = Stage::None;
  bus_.beginI2C(wire, address, clockHz_);
  // The BMI270 starts with advanced power save enabled. Bosch requires a
  // 450-us quiet interval after every interface transaction in that state.
  // Keep the conservative interval for the session so recovery and optional
  // power-mode changes cannot violate the device timing contract.
  bus_.setPostTransactionDelayUs(450);
  bus_.recoverBus();
  // An address ACK is enough authority to place the explicitly selected
  // BMI270 into a defined state. Requiring CHIP_ID before soft reset can
  // deadlock recovery when the register interface is between power modes:
  // writes are accepted, but the first read is not yet available.
  bool acknowledged = false;
  for (uint8_t attempt = 0; attempt < 4 && !acknowledged; ++attempt) {
    acknowledged = bus_.ping() == IMUStatus::Ok;
    if (!acknowledged) delay(2);
  }
  if (!acknowledged) {
    lastStage_ = Stage::NotConnected;
    return false;
  }
  if (!reset()) {
    lastStage_ = Stage::ResetFailed;
    return false;
  }
  if (!uploadConfiguration()) {
    // uploadConfiguration() distinguishes these itself; if it got as far as
    // asking the chip whether the blob took, the transfer worked.
    if (lastStage_ != Stage::ConfigNotLoaded &&
        lastStage_ != Stage::ConfigFileStub) {
      lastStage_ = Stage::ConfigUpload;
    }
    return false;
  }
  if (!configureDefaults()) {
    lastStage_ = Stage::Defaults;
    return false;
  }
  return true;
}

const char* BMI270::lastStageText() const {
  switch (lastStage_) {
    case Stage::None:            return "no error";
    case Stage::NotConnected:    return "WHO_AM_I did not read 0x24 - wrong address, "
                                        "or the part is not a BMI270";
    case Stage::ResetFailed:     return "soft reset issued but the chip did not come back";
    case Stage::ConfigFileStub:  return "no usable configuration image supplied - a "
                                        "BMI270 does nothing until one of Bosch's "
                                        "images is loaded; call setConfigImage() first";
    case Stage::ConfigUpload:    return "the configuration blob would not transfer";
    case Stage::ConfigNotLoaded:
      switch (lastInternalStatus_ & 0x0Fu) {
        case 0x00: return "configuration state machine did not reach init_ok "
                          "before the bounded timeout";
        case 0x02: return "the chip reports init_err - it received an image but "
                          "rejected it";
        case 0x03: return "the chip reports drv_err";
        default:   return "blob sent, but the chip never reported it loaded";
      }
    case Stage::Defaults:        return "configured, but the default settings would not apply";
  }
  return "unknown";
}

bool BMI270::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0x80);
  bus_.setPostTransactionDelayUs(450);
  whoAmI();  // BMI2 SPI needs one dummy transaction after interface select.
  if (!isConnected()) {
    return false;
  }
  return reset() && uploadConfiguration() && configureDefaults();
}

uint8_t BMI270::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(CHIP_ID, id);
  return id;
}

bool BMI270::isConnected() {
  // Tolerate a transient controller or bus response during bring-up before
  // declaring that the expected identity is unavailable.
  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    if (whoAmI() == kChipId) return true;
    delay(2);
  }
  return false;
}

bool BMI270::reset() {
  // The reset command can be accepted even when the controller reports a
  // missing final ACK. Decide success from CHIP_ID after the mandatory reset
  // interval. A controller left in an error state is reset by IMUBus before
  // this function continues.
  for (uint8_t commandAttempt = 0; commandAttempt < 3; ++commandAttempt) {
    (void)bus_.writeRegisterOnce(CMD, CMD_SOFT_RESET);
    delay(5);

    // Start configuration as soon as the reset window opens. Some interfaces
    // provide only a short post-reset access window until PWR_CONF is written.
    const uint32_t deadline = millis() + 15;
    while (millis() < deadline) {
      if (whoAmI() == kChipId) return true;
    }
  }
  return false;
}

bool BMI270::writeConfigChunk(uint16_t index, const uint8_t* data,
                              uint16_t len) {
  uint8_t addr[2] = {
      static_cast<uint8_t>((index / 2) & 0x0F),
      static_cast<uint8_t>((index / 2) >> 4),
  };
  if (bus_.writeRegisters(INIT_ADDR_0, addr, sizeof(addr)) != IMUStatus::Ok) {
    return false;
  }
  return bus_.writeRegisters(INIT_DATA, data, len) == IMUStatus::Ok;
}

uint8_t BMI270::configByte(size_t offset) const {
#if defined(__AVR__)
  if (configImageProgmem_) return pgm_read_byte(configImage_ + offset);
#else
  (void)configImageProgmem_;
#endif
  return configImage_[offset];
}

bool BMI270::uploadConfiguration() {
  // Do not police the length beyond "present and even". Bosch ships several
  // images for this chip and they are not all the same size: the standard,
  // legacy and context builds are 8192 bytes, while maximum_fifo is 328. A
  // guard demanding 8192 would reject a perfectly good image. The chip does
  // its own validation and reports init_err if it dislikes what it got.
  if (configImage_ == nullptr || configImageLength_ == 0 ||
      (configImageLength_ % 2u) != 0u) {
    lastStage_ = Stage::ConfigFileStub;
    return false;
  }

  if (bus_.writeRegister(PWR_CONF, 0x00) != IMUStatus::Ok) {
    return false;
  }
  delay(1);
  if (bus_.writeRegister(INIT_CTRL, 0x00) != IMUStatus::Ok) {
    return false;
  }

  constexpr uint16_t kChunk = 16;
  for (uint16_t i = 0; i < configImageLength_; i += kChunk) {
    uint16_t len = static_cast<uint16_t>(configImageLength_ - i);
    if (len > kChunk) {
      len = kChunk;
    }
    uint8_t chunk[kChunk];
    for (uint16_t j = 0; j < len; ++j) chunk[j] = configByte(i + j);
    if (!writeConfigChunk(i, chunk, len)) {
      return false;
    }
  }

  if (bus_.writeRegister(INIT_CTRL, 0x01) != IMUStatus::Ok) {
    return false;
  }

  // Keep the explicitly selected power policy stable while the internal
  // configuration state machine starts. This is bounded to the documented
  // startup window and stops before normal sampling begins.
  for (uint32_t until = millis() + 150; millis() < until;) {
    if (bus_.writeRegister(PWR_CONF, 0x00) != IMUStatus::Ok) return false;
  }
  if (!configurationLoaded()) {
    lastStage_ = Stage::ConfigNotLoaded;
    return false;
  }
  return true;
}

bool BMI270::configurationLoaded() {
  // INTERNAL_STATUS bits [3:0] are a message field, not flags: 0 = not_init,
  // 1 = init_ok, 2 = init_err, 3 = drv_err, and so on. Keep the raw byte so a
  // failure can say which of those it was instead of just "no".
  //
  // Poll rather than read once: the chip finishes its self-configuration in
  // its own time after INIT_CTRL is set.
  // One second, not the datasheet's "at most 20 msec".
  //
  // Bosch's own API carried a 20 ms wait here and had to change it: issue #7
  // on BMI270_SensorAPI, "20ms not enough until asic initialized", replaced it
  // with a retry loop and a one-second timeout, and issue #18 reports the same
  // failure from the wait being dropped altogether. Parts exist that need far
  // longer than the datasheet figure, and the cost of waiting is only paid by
  // a part that was going to fail anyway.
  const uint32_t deadline = millis() + 1000;
  for (;;) {
    uint8_t status = 0;
    if (bus_.readRegister(INTERNAL_STATUS, status) == IMUStatus::Ok) {
      lastInternalStatus_ = status;
      if ((status & 0x0Fu) == 0x01u) return true;   // init_ok
    }

    if (millis() >= deadline) {
      return false;
    }
    delay(5);
  }
}

bool BMI270::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  ok &= wakeAndEnable(PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN);

  // Poll for the first sample so readiness is observed as soon as it occurs.
  ok &= wakeAndAwaitData(wakeTimeoutMs_);
  return ok;
}

bool BMI270::wakeAndEnable(uint8_t sensors) {
  // Bosch requires 450 us after access while advanced power save is active.
  // PWR_CONF therefore cannot be burst together with PWR_CTRL: the second
  // byte can arrive before the mode transition has completed and be ignored.
  if (bus_.writeRegister(PWR_CONF, 0x00) != IMUStatus::Ok) return false;
  delayMicroseconds(450);
  if (bus_.writeRegister(PWR_CTRL, sensors) != IMUStatus::Ok) return false;
  delayMicroseconds(2);
  return true;
}

void BMI270::trace(const char* what, uint8_t value) {
  if (debug_ == nullptr) return;
  debug_->print(F("  [bmi270] "));
  debug_->print(what);
  debug_->print(F(" 0x"));
  debug_->println(value, HEX);
}

bool BMI270::resumeAfterSleep() {
  uint8_t internal = 0;
  bool coreAlive = whoAmI() == kChipId;
  if (coreAlive) {
    coreAlive = bus_.writeRegister(PWR_CONF, 0x00) == IMUStatus::Ok &&
                bus_.readRegister(INTERNAL_STATUS, internal) == IMUStatus::Ok &&
                (internal & 0x0Fu) == INTERNAL_STATUS_INIT_OK;
  }
  trace("resume: INTERNAL_STATUS", internal);

  if (!coreAlive) {
    const bool didReset = reset();
    trace("resume: soft reset ok", didReset ? 1 : 0);
    if (!didReset) return false;
    const bool uploaded = uploadConfiguration();
    trace("resume: upload ok", uploaded ? 1 : 0);
    if (!uploaded) return false;
  }

  const uint8_t conf[4] = {accConf_, accRange_, gyrConf_, gyrRange_};
  trace("resume: writing ACC_CONF", accConf_);
  if (bus_.writeRegisters(ACC_CONF, conf, sizeof(conf)) != IMUStatus::Ok) {
    return false;
  }
  return wakeAndAwaitData(wakeTimeoutMs_);
}

bool BMI270::wakeAndAwaitData(uint16_t timeoutMs) {
  // Enable once. Rewriting PWR_CTRL while polling restarts sensor startup and
  // can prevent data-ready from ever asserting.
  if (!wakeAndEnable(PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN)) return false;

  for (uint32_t end = millis() + timeoutMs; millis() < end; ) {
    uint8_t status = 0;
    // Wait for both sensors, not just the accelerometer. The gyroscope takes
    // noticeably longer to start, so returning on drdy_acc alone handed back
    // samples with the gyro reading exactly 0.00 on every axis - which looks
    // like a perfectly still board rather than one that has not woken up yet.
    const uint8_t both = STATUS_DRDY_ACC | STATUS_DRDY_GYR;
    if (bus_.readRegister(STATUS, status) == IMUStatus::Ok &&
        (status & both) == both) {
      return true;
    }
    delayMicroseconds(100);
  }
  return false;
}

bool BMI270::sampleIsDead(const uint8_t* a, const uint8_t* g,
                          const uint8_t* t) {
  if (!(t[0] == 0x00 && t[1] == 0x80)) return false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (a[i] != 0 || g[i] != 0) return false;
  }
  return true;
}

bool BMI270::readSampleRegisters(uint8_t* a, uint8_t* g, uint8_t* t,
                                 uint32_t& sensorTime) {
  uint8_t time[3];
  if (bus_.readRegisters(DATA_ACCEL_X_L, a, 6) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_GYRO_X_L, g, 6) != IMUStatus::Ok ||
      bus_.readRegisters(SENSOR_TIME_0, time, sizeof(time)) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_TEMP_L, t, 2) != IMUStatus::Ok) {
    return false;
  }
  sensorTime = static_cast<uint32_t>(time[0]) |
               (static_cast<uint32_t>(time[1]) << 8) |
               (static_cast<uint32_t>(time[2]) << 16);
  return true;
}

bool BMI270::readRaw(RawSample& out) {
  uint8_t a[6];
  uint8_t g[6];
  uint8_t t[2];
  uint32_t sensorTime = 0;

  bool got = readSampleRegisters(a, g, t, sensorTime);
  if (!got || sampleIsDead(a, g, t)) {
    trace(got ? "sample invalid" : "sample transfer failed", got ? 1 : 0);
    if (!resumeAfterSleep() ||
        !readSampleRegisters(a, g, t, sensorTime)) return false;
    if (sampleIsDead(a, g, t)) return false;
  }
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.temp = le16(t);
  out.sensorTime = sensorTime;
  lastSensorTime_ = sensorTime;
  return true;
}

bool BMI270::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }
  Vec3 a{raw.ax / accelLsbPerG_, raw.ay / accelLsbPerG_,
         raw.az / accelLsbPerG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);

  Vec3 g{raw.gx / gyroLsbPerDps_, raw.gy / gyroLsbPerDps_,
         raw.gz / gyroLsbPerDps_};
  data_.gyro = correct(g, cal_.gyroBias, Vec3{1, 1, 1});

  data_.mag = Vec3{0, 0, 0};
  data_.temperature = 23.0f + (raw.temp / 512.0f);
  data_.timestamp = micros();
  return true;
}

bool BMI270::setAccelRangeG(uint16_t maxG) {
  uint8_t range;
  if (maxG <= 2) {
    range = 0x00; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    range = 0x01; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    range = 0x02; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    range = 0x03; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  accRange_ = range;
  return bus_.writeRegister(ACC_RANGE, range) == IMUStatus::Ok;
}

bool BMI270::setGyroRangeDps(uint16_t maxDps) {
  uint8_t range;
  if (maxDps <= 125) {
    range = 0x04; gyroLsbPerDps_ = 262.144f; gyroRangeDps_ = 125;
  } else if (maxDps <= 250) {
    range = 0x03; gyroLsbPerDps_ = 131.072f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    range = 0x02; gyroLsbPerDps_ = 65.536f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    range = 0x01; gyroLsbPerDps_ = 32.768f; gyroRangeDps_ = 1000;
  } else {
    range = 0x00; gyroLsbPerDps_ = 16.384f; gyroRangeDps_ = 2000;
  }
  gyrRange_ = range;
  return bus_.writeRegister(GYR_RANGE, range) == IMUStatus::Ok;
}

uint8_t BMI270::odrCodeForHz(uint16_t hz, uint16_t& actualHz, bool gyro) const {
  if (!gyro && hz <= 1) { actualHz = 1; return 0x01; }
  if (!gyro && hz <= 3) { actualHz = 3; return 0x03; }
  if (!gyro && hz <= 7) { actualHz = 6; return 0x04; }
  if (hz <= 13) { actualHz = 12; return 0x05; }
  if (hz <= 25) { actualHz = 25; return 0x06; }
  if (hz <= 50) { actualHz = 50; return 0x07; }
  if (hz <= 100) { actualHz = 100; return 0x08; }
  if (hz <= 200) { actualHz = 200; return 0x09; }
  if (hz <= 400) { actualHz = 400; return 0x0A; }
  if (hz <= 800) { actualHz = 800; return 0x0B; }
  if (hz <= 1600) { actualHz = 1600; return 0x0C; }
  actualHz = 3200;
  return gyro ? 0x0D : 0x0C;
}

bool BMI270::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actualAccel = 0;
  uint16_t actualGyro = 0;
  accelOdr_ = odrCodeForHz(hz, actualAccel, false);
  gyroOdr_ = odrCodeForHz(hz, actualGyro, true);
  sampleRateHz_ = actualAccel;
  // Write these outright instead of read-modify-write. A part that has dozed
  // off answers the read with its reset default, so updateRegister() quietly
  // rebuilds the register from the wrong base. The top nibble is the filter
  // and performance configuration this driver always uses, so there is nothing
  // in there worth preserving from a read.
  accConf_ = static_cast<uint8_t>(0xA0u | (accelOdr_ & 0x0Fu));
  gyrConf_ = static_cast<uint8_t>(0xA0u | (gyroOdr_ & 0x0Fu));
  bool ok = true;
  ok &= bus_.writeRegister(ACC_CONF, accConf_) == IMUStatus::Ok;
  ok &= bus_.writeRegister(GYR_CONF, gyrConf_) == IMUStatus::Ok;
  return ok;
}

bool BMI270::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  // Write outright rather than read-modify-write.
  //
  // Bosch issue #16 is this same hazard on a different pair of registers:
  // a read-modify-write on INT1_IO_CTRL/INT2_IO_CTRL "returns zeros", so the
  // value written is built from a base that was never really read. Here the
  // part can be asleep, which has exactly the same effect - the read yields
  // the reset default and the performance bit gets written on top of that,
  // silently discarding the output data rate configured a moment earlier.
  //
  // The cached values are the truth about what this driver has configured, so
  // set the bit in them and write the whole register.
  accConf_ |= PERF_MODE;
  gyrConf_ |= PERF_MODE;
  bool ok = true;
  ok &= bus_.writeRegister(ACC_CONF, accConf_) == IMUStatus::Ok;
  ok &= bus_.writeRegister(GYR_CONF, gyrConf_) == IMUStatus::Ok;
  return ok;
}

bool BMI270::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & (STATUS_DRDY_ACC | STATUS_DRDY_GYR)) ==
         (STATUS_DRDY_ACC | STATUS_DRDY_GYR);
}

bool BMI270::configureInterruptPin(uint8_t pin, bool activeHigh,
                                   bool openDrain, bool latched) {
  uint8_t reg = pin == 1 ? INT1_IO_CTRL : pin == 2 ? INT2_IO_CTRL : 0;
  if (reg == 0) return false;
  uint8_t value = INT_OUTPUT_ENABLE |
                  (openDrain ? INT_OPEN_DRAIN : 0) |
                  (activeHigh ? INT_ACTIVE_HIGH : 0);
  bool ok = bus_.writeRegister(reg, value) == IMUStatus::Ok;
  ok &= bus_.writeRegister(INT_LATCH, latched ? 0x0F : 0x00) == IMUStatus::Ok;
  return ok;
}

bool BMI270::routeDataReadyInterrupt(uint8_t pin, bool enable) {
  if (pin != 1 && pin != 2) return false;
  const uint8_t mask = pin == 1 ? 0x04 : 0x40;
  // Track the mapping here rather than reading it back, for the reason given
  // in setLowPassFilterHz: a read-modify-write on a part that has dozed off
  // rebuilds the register from its reset default and quietly unmaps whatever
  // was routed to the other pin.
  intMapData_ = static_cast<uint8_t>((intMapData_ & ~mask) | (enable ? mask : 0));
  return bus_.writeRegister(INT_MAP_DATA, intMapData_) == IMUStatus::Ok;
}

void BMI270::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}


// ---------------------------------------------------------------------------
// Advanced features
// ---------------------------------------------------------------------------

bool BMI270::readFeaturePage(uint8_t page, uint8_t* out) {
  if (bus_.writeRegister(FEAT_PAGE, page) != IMUStatus::Ok) return false;
  return bus_.readRegisters(FEATURES, out, FEATURES_SIZE) == IMUStatus::Ok;
}

bool BMI270::writeFeaturePage(uint8_t page, const uint8_t* in) {
  if (bus_.writeRegister(FEAT_PAGE, page) != IMUStatus::Ok) return false;
  // The whole page goes back in one transfer. Section 4.8.1 requires feature
  // writes to be 16-bit word aligned; writing all sixteen bytes from an even
  // start satisfies that without the caller having to think about it.
  return bus_.writeRegisters(FEATURES, in, FEATURES_SIZE) == IMUStatus::Ok;
}

bool BMI270::setFeatureBit(uint8_t page, uint8_t index, uint8_t mask,
                           bool enable) {
  if (index >= FEATURES_SIZE) return false;
  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(page, buffer)) return false;
  if (enable) {
    buffer[index] |= mask;
  } else {
    buffer[index] &= static_cast<uint8_t>(~mask);
  }
  return writeFeaturePage(page, buffer);
}

bool BMI270::enableFeature(Feature feature, bool enable) {
  // Nothing here exists without the core running, and silently doing nothing
  // would be worse than saying so.
  if ((lastInternalStatus_ & 0x0Fu) != INTERNAL_STATUS_INIT_OK) return false;

  switch (feature) {
    case Feature::AnyMotion:
      return setFeatureBit(PAGE_ANY_MOTION, OFF_ANY_MOTION + EN_OFF_ANY_NO_MOTION,
                           EN_MASK_ANY_NO_MOTION, enable);
    case Feature::NoMotion:
      return setFeatureBit(PAGE_NO_MOTION, OFF_NO_MOTION + EN_OFF_ANY_NO_MOTION,
                           EN_MASK_ANY_NO_MOTION, enable);
    case Feature::SignificantMotion:
      return setFeatureBit(PAGE_SIG_MOTION, OFF_SIG_MOTION + EN_OFF_SIG_MOTION,
                           EN_MASK_SIG_MOTION, enable);
    case Feature::StepDetector:
      return setFeatureBit(PAGE_STEP_CONF, OFF_STEP_CONF + EN_OFF_STEP,
                           EN_MASK_STEP_DETECTOR, enable);
    case Feature::StepCounter:
      return setFeatureBit(PAGE_STEP_CONF, OFF_STEP_CONF + EN_OFF_STEP,
                           EN_MASK_STEP_COUNTER, enable);
    case Feature::StepActivity:
      return setFeatureBit(PAGE_STEP_CONF, OFF_STEP_CONF + EN_OFF_STEP,
                           EN_MASK_STEP_ACTIVITY, enable);
    case Feature::WristGesture:
      return setFeatureBit(PAGE_WRIST_GESTURE, OFF_WRIST_GESTURE + EN_OFF_STEP,
                           EN_MASK_WRIST_GESTURE, enable);
    case Feature::WristWearWakeup:
      return setFeatureBit(PAGE_WRIST_WEAR, OFF_WRIST_WEAR + EN_OFF_STEP,
                           EN_MASK_WRIST_WEAR, enable);
  }
  return false;
}

uint32_t BMI270::stepCount() {
  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(PAGE_FEATURE_OUT, buffer)) return 0;
  const uint8_t* p = &buffer[OFF_STEP_COUNT_OUT];
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

bool BMI270::resetStepCount() {
  // The reset is a bit in the step counter's own configuration word, and it is
  // self-clearing: set it, write the page back, and the core zeroes the count.
  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(PAGE_STEP_CONF, buffer)) return false;
  const uint8_t high = OFF_STEP_CONF + 1;
  if (high >= FEATURES_SIZE) return false;
  buffer[high] |= static_cast<uint8_t>(STEP_COUNT_RST_MASK >> 8);
  return writeFeaturePage(PAGE_STEP_CONF, buffer);
}

BMI270::Activity BMI270::stepActivity() {
  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(PAGE_FEATURE_OUT, buffer)) return Activity::Unknown;
  const uint8_t value = buffer[OFF_STEP_ACT_OUT] & 0x03u;
  return static_cast<Activity>(value);
}

bool BMI270::configureMotion(uint8_t page, uint8_t offset, uint16_t thresholdMg,
                             uint16_t durationMs, bool x, bool y, bool z) {
  if ((lastInternalStatus_ & 0x0Fu) != INTERNAL_STATUS_INIT_OK) return false;
  if (static_cast<uint16_t>(offset) + 4u > FEATURES_SIZE) return false;

  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(page, buffer)) return false;

  // Duration counts 20 ms steps, 13 bits of them.
  uint32_t duration = durationMs / 20u;
  if (duration > ANY_NO_MOT_DUR_MASK) duration = ANY_NO_MOT_DUR_MASK;

  // Threshold is 5.11 format - one count is 1/2048 g, so 1 mg is 2.048 counts.
  uint32_t threshold = (static_cast<uint32_t>(thresholdMg) * 2048u) / 1000u;
  if (threshold > ANY_NO_MOT_THRES_MASK) threshold = ANY_NO_MOT_THRES_MASK;

  uint16_t word0 = static_cast<uint16_t>(duration);
  if (x) word0 |= ANY_NO_MOT_X_SEL;
  if (y) word0 |= ANY_NO_MOT_Y_SEL;
  if (z) word0 |= ANY_NO_MOT_Z_SEL;

  // Preserve the enable bit, which lives in the top of the threshold word, so
  // that configuring a detector does not silently switch it off again.
  const uint16_t existing =
      static_cast<uint16_t>(buffer[offset + 2]) |
      (static_cast<uint16_t>(buffer[offset + 3]) << 8);
  const uint16_t word1 = static_cast<uint16_t>(threshold) |
                         static_cast<uint16_t>(existing & 0xF800u);

  buffer[offset + 0] = static_cast<uint8_t>(word0 & 0xFFu);
  buffer[offset + 1] = static_cast<uint8_t>(word0 >> 8);
  buffer[offset + 2] = static_cast<uint8_t>(word1 & 0xFFu);
  buffer[offset + 3] = static_cast<uint8_t>(word1 >> 8);
  return writeFeaturePage(page, buffer);
}

bool BMI270::configureAnyMotion(uint16_t thresholdMg, uint16_t durationMs,
                                bool x, bool y, bool z) {
  return configureMotion(PAGE_ANY_MOTION, OFF_ANY_MOTION, thresholdMg,
                         durationMs, x, y, z);
}

bool BMI270::configureNoMotion(uint16_t thresholdMg, uint16_t durationMs,
                               bool x, bool y, bool z) {
  return configureMotion(PAGE_NO_MOTION, OFF_NO_MOTION, thresholdMg, durationMs,
                         x, y, z);
}

uint8_t BMI270::featureInterrupts() {
  uint8_t status = 0;
  if (bus_.readRegister(INT_STATUS_0, status) != IMUStatus::Ok) return 0;
  return status;
}

bool BMI270::mapFeatureInterrupt(uint8_t pin, uint8_t mask) {
  if (pin != 1 && pin != 2) return false;
  const uint8_t reg = (pin == 1) ? INT1_MAP_FEAT : INT2_MAP_FEAT;
  return bus_.writeRegister(reg, mask) == IMUStatus::Ok;
}

bool BMI270::configureFifo(bool accel, bool gyro, bool headerMode) {
  uint8_t config = 0;
  if (accel) config |= FIFO_CONF1_ACC_EN;
  if (gyro) config |= FIFO_CONF1_GYR_EN;
  if (headerMode) config |= FIFO_CONF1_HEADER_EN;
  return bus_.writeRegister(FIFO_CONFIG_1, config) == IMUStatus::Ok;
}

uint16_t BMI270::fifoLength() {
  uint8_t raw[2] = {0, 0};
  if (bus_.readRegisters(FIFO_LENGTH_0, raw, sizeof(raw)) != IMUStatus::Ok) {
    return 0;
  }
  // Fourteen bits; the top two of the high byte are reserved.
  return static_cast<uint16_t>(raw[0]) |
         (static_cast<uint16_t>(raw[1] & 0x3Fu) << 8);
}

uint16_t BMI270::readFifo(uint8_t* out, uint16_t maxBytes) {
  if (out == nullptr || maxBytes == 0) return 0;
  uint16_t available = fifoLength();
  if (available == 0) return 0;
  if (available > maxBytes) available = maxBytes;
  if (bus_.readRegisters(FIFO_DATA, out, available) != IMUStatus::Ok) return 0;
  return available;
}

bool BMI270::flushFifo() {
  return bus_.writeRegister(CMD, CMD_FIFO_FLUSH) == IMUStatus::Ok;
}

bool BMI270::setAccelOffset(int8_t x, int8_t y, int8_t z, bool enable) {
  const uint8_t offsets[3] = {static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                              static_cast<uint8_t>(z)};
  if (bus_.writeRegisters(OFFSET_ACC_X, offsets, sizeof(offsets)) !=
      IMUStatus::Ok) {
    return false;
  }
  // acc_off_en lives in NV_CONF alongside the interface settings, so this has
  // to be a read-modify-write - clobbering spi_en here would take the part off
  // the bus entirely.
  uint8_t nv = 0;
  if (bus_.readRegister(NV_CONF, nv) != IMUStatus::Ok) return false;
  if (enable) {
    nv |= NV_ACC_OFF_EN;
  } else {
    nv &= static_cast<uint8_t>(~NV_ACC_OFF_EN);
  }
  return bus_.writeRegister(NV_CONF, nv) == IMUStatus::Ok;
}

bool BMI270::setGyroOffset(int16_t x, int16_t y, int16_t z, bool enable) {
  // Ten bits per axis: the low eight in their own register, the top two packed
  // into OFFSET_6 together with the enable bit.
  const uint8_t low[3] = {static_cast<uint8_t>(x & 0xFF),
                          static_cast<uint8_t>(y & 0xFF),
                          static_cast<uint8_t>(z & 0xFF)};
  if (bus_.writeRegisters(OFFSET_GYR_X, low, sizeof(low)) != IMUStatus::Ok) {
    return false;
  }

  uint8_t high = 0;
  if (bus_.readRegister(OFFSET_6, high) != IMUStatus::Ok) return false;
  high &= 0xC0u;   // keep gyr_gain_en and gyr_off_en, clear the packed bits
  high |= static_cast<uint8_t>((x >> 8) & 0x03u);
  high |= static_cast<uint8_t>(((y >> 8) & 0x03u) << 2);
  high |= static_cast<uint8_t>(((z >> 8) & 0x03u) << 4);
  if (enable) {
    high |= OFFSET6_GYR_OFF_EN;
  } else {
    high &= static_cast<uint8_t>(~OFFSET6_GYR_OFF_EN);
  }
  return bus_.writeRegister(OFFSET_6, high) == IMUStatus::Ok;
}

bool BMI270::selfTestAccel() {
  // Run at +/-16 g in performance mode, drive the internal test signal
  // positive then negative, and check the deflection on each axis clears the
  // datasheet's minimum. The board has to be still while this runs.
  const uint8_t savedRange = accRange_;
  bool ok = true;
  ok &= bus_.writeRegister(ACC_CONF, 0xA7) == IMUStatus::Ok;   // 1600 Hz, perf
  ok &= bus_.writeRegister(ACC_RANGE, 0x03) == IMUStatus::Ok;  // +/- 16 g
  ok &= wakeAndEnable(PWR_ACC_EN);
  if (!ok) return false;
  delay(2);

  int16_t positive[3] = {0, 0, 0};
  int16_t negative[3] = {0, 0, 0};
  for (uint8_t phase = 0; phase < 2; ++phase) {
    // acc_self_test_en, with the sign chosen by the phase.
    const uint8_t sign = (phase == 0) ? 0x04 : 0x00;
    if (bus_.writeRegister(ACC_SELF_TEST, static_cast<uint8_t>(0x01 | sign)) !=
        IMUStatus::Ok) {
      ok = false;
      break;
    }
    // The datasheet asks for at least 50 ms to settle - but waiting with
    // delay() puts a part that does not hold state across an idle bus straight
    // back to sleep, and the self-test then measures a sensor that is not
    // running. That is why this reported a failure on a part whose readings
    // were otherwise perfect. Poll instead, which settles and keeps it awake.
    for (uint32_t end = millis() + 60; millis() < end; ) {
      uint8_t ignored = 0;
      bus_.readRegister(STATUS, ignored);
    }
    uint8_t raw[6] = {0};
    if (bus_.readRegisters(DATA_ACCEL_X_L, raw, sizeof(raw)) != IMUStatus::Ok) {
      ok = false;
      break;
    }
    int16_t* target = (phase == 0) ? positive : negative;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      target[axis] = le16(&raw[axis * 2]);
    }
  }

  bus_.writeRegister(ACC_SELF_TEST, 0x00);
  bus_.writeRegister(ACC_RANGE, savedRange);
  configureDefaults();
  if (!ok) return false;

  // At +/-16 g one count is about 0.49 mg. The datasheet's minimum deflections
  // are 1000 mg on x and y, 500 mg on z.
  const float mgPerCount = 16000.0f / 32768.0f;
  const float minimum[3] = {1000.0f, 1000.0f, 500.0f};
  for (uint8_t axis = 0; axis < 3; ++axis) {
    float difference =
        (static_cast<float>(positive[axis]) - static_cast<float>(negative[axis])) *
        mgPerCount;
    if (difference < 0.0f) difference = -difference;
    if (difference < minimum[axis]) return false;
  }
  return true;
}

bool BMI270::remapAxes(uint8_t xTo, bool xInvert, uint8_t yTo, bool yInvert,
                       uint8_t zTo, bool zInvert) {
  if ((lastInternalStatus_ & 0x0Fu) != INTERNAL_STATUS_INIT_OK) return false;
  if (xTo > 2 || yTo > 2 || zTo > 2) return false;

  uint8_t buffer[FEATURES_SIZE];
  if (!readFeaturePage(PAGE_AXIS_MAP, buffer)) return false;

  // Two bits of destination plus one of sign per axis, packed three to a byte
  // with the last one spilling into the next.
  const uint8_t xField = static_cast<uint8_t>((xTo & 0x03u) | (xInvert ? 0x04u : 0x00u));
  const uint8_t yField = static_cast<uint8_t>((yTo & 0x03u) | (yInvert ? 0x04u : 0x00u));
  const uint8_t zField = static_cast<uint8_t>((zTo & 0x03u) | (zInvert ? 0x04u : 0x00u));

  buffer[OFF_AXIS_MAP] = static_cast<uint8_t>(xField | (yField << 3) |
                                              ((zField & 0x03u) << 6));
  buffer[OFF_AXIS_MAP + 1] = static_cast<uint8_t>(
      (buffer[OFF_AXIS_MAP + 1] & 0xFEu) | ((zField >> 2) & 0x01u));
  return writeFeaturePage(PAGE_AXIS_MAP, buffer);
}

}  // namespace nimu
