#include "BMI270.h"

#include "BMI270_Config.h"

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
  bus_.recoverBus();
  if (!isConnected()) {
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
        case 0x00:
          if (losesStateWhenIdle_) {
            return "this part does not keep its registers across an idle bus - "
                   "it answered a moment earlier and had forgotten a value by "
                   "the time it was read back. The driver recovers from that "
                   "by itself during update(), so if bring-up still failed the "
                   "cause is elsewhere; check the configuration image first";
          }
          if (porDuringInit_) {
            return "por_detected was set while the core was starting and the "
                   "configuration registers reverted. On a part that does not "
                   "hold state across an idle bus this is normal and the "
                   "driver handles it; if it persists, look at the supply at "
                   "the module's VDD and GND pins";
          }
          return "image sent, chip still reports not_init - its internal core "
                 "is not running. A working BMI270 answers a bad image with "
                 "init_err, so a part that never leaves not_init has not "
                 "started at all";
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
  // Ask more than once before declaring the part absent.
  //
  // A single identity read is not a reliable test. Sampled 30 times on a
  // quiet, freshly reset BMI270, CHIP_ID came back correct 29 times and the
  // read failed outright once - and the first transaction after an idle bus
  // is the likeliest one to go, which is exactly where this call sits. One
  // unlucky read was enough to report a healthy sensor as "not a BMI270",
  // which sends you looking at the wiring for a fault that is not there.
  for (uint8_t attempt = 0; attempt < 4; ++attempt) {
    if (whoAmI() == kChipId) return true;
    delay(2);
  }
  return false;
}

bool BMI270::reset() {
  if (bus_.writeRegister(CMD, CMD_SOFT_RESET) != IMUStatus::Ok) {
    return false;
  }

  // Poll for the chip to come back rather than assuming a fixed delay is
  // enough.
  //
  // A soft reset drops the BMI270 into suspend and it NACKs everything until
  // it is ready again; the datasheet's 2 ms is a minimum, not a guarantee, and
  // on a shared bus it is routinely longer. Checking once after 20 ms meant a
  // perfectly good part - one reading 0x24 to a bus scanner all day - was
  // reported as "did not come back" and bring-up stopped there.
  //
  // Polling costs nothing when the chip is quick and rescues the case where it
  // is not.
  // Poll without sleeping.
  //
  // This used to delay(5) between attempts, which is a small idle gap each
  // time round - and on a part that does not hold its state across an idle
  // bus, those gaps are the very thing being recovered from. Reading flat out
  // both polls and keeps the bus alive, and it costs nothing on a part that
  // comes back immediately.
  const uint32_t deadline = millis() + 300;
  while (millis() < deadline) {
    if (whoAmI() == kChipId) return true;
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

bool BMI270::uploadConfiguration() {
  // The image belongs to the caller - see BMI270_Config.h for why this library
  // ships none of it. Without one there is nothing to upload and nothing the
  // part can do, so say that plainly instead of pretending to try.
  //
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
    if (!writeConfigChunk(i, &configImage_[i], len)) {
      return false;
    }
  }

  if (bus_.writeRegister(INIT_CTRL, 0x01) != IMUStatus::Ok) {
    return false;
  }

  // Hold advanced power save off while the core starts, instead of simply
  // waiting out the initialisation window.
  //
  // The datasheet has you clear PWR_CONF once before the upload and assumes it
  // stays clear. On some parts it does not: advanced power save comes back
  // during the 150 ms the core needs, the part drops into suspend before it
  // has finished, and INTERNAL_STATUS sits at not_init for ever afterwards.
  //
  // Measured three times each on a part that behaved this way: clearing it
  // once and waiting reached init_ok 0 times out of 3, and so did rewriting it
  // before every chunk of the upload - the upload was never the problem.
  // Holding it down across this window reached init_ok 3 times out of 3.
  //
  // On a part that does not need it this rewrites a register with the value it
  // already holds, which costs a few hundred microseconds and changes nothing.
  for (uint32_t until = millis() + 150; millis() < until; ) {
    bus_.writeRegister(PWR_CONF, 0x00);
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
  porDuringInit_ = false;
  losesStateWhenIdle_ = false;

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

    // Ask the part why it is not answering, rather than only that it is not.
    //
    // A BMI270 whose core browns out as it starts looks identical from
    // INTERNAL_STATUS to one that simply never got its image: both sit at
    // not_init for ever. EVENT tells them apart, and it is the difference
    // between "try a different image" and "fix the power". por_detected
    // clears on read, so catch it here rather than after the loop.
    uint8_t event = 0;
    if (bus_.readRegister(EVENT, event) == IMUStatus::Ok &&
        (event & EVENT_POR_DETECTED) != 0) {
      porDuringInit_ = true;
    }

    if (millis() >= deadline) {
      // Before blaming the image or the silicon, check the part is actually
      // powered. This costs one write, one idle wait and two reads, and only
      // ever runs on a bring-up that has already failed.
      losesStateWhenIdle_ = !holdsStateAcrossIdleBus();
      return false;
    }
    delay(5);
  }
}

bool BMI270::holdsStateAcrossIdleBus() {
  // Establish that it is talking right now, so that going quiet later means
  // something. Without this the test cannot tell an unpowered part from one
  // that was never on the bus at all, and it must not blame the supply for a
  // sensor that simply is not fitted.
  uint8_t identity = 0;
  if (bus_.readRegister(CHIP_ID, identity) != IMUStatus::Ok ||
      identity != kChipId) {
    return true;   // not answering even now; a different fault entirely
  }

  // ACC_CONF is a full eight-bit register with no reserved bits, so the
  // readback is an exact comparison rather than a masked one, and it is safe
  // to disturb on a bring-up that has already failed.
  const uint8_t probe = 0x57;          // not the 0xA8 reset default
  if (bus_.writeRegister(ACC_CONF, probe) != IMUStatus::Ok) {
    // It answered a moment ago and will not take a write now. That is the
    // fault, not an excuse to ignore it - the original version of this check
    // treated a refused transaction as "inconclusive" and so never fired on
    // the very board it was written for.
    return false;
  }

  uint8_t immediate = 0;
  if (bus_.readRegister(ACC_CONF, immediate) != IMUStatus::Ok) return false;
  if (immediate != probe) {
    // Accepted the write but did not keep it even for a moment. Still a part
    // that cannot hold state.
    return false;
  }

  // The bus must be genuinely quiet here. Any transaction, even one addressed
  // to another device, tops an unpowered part up through its clamp diodes and
  // hides exactly what this is looking for.
  //
  // Measured on such a module the value survived 20 ms and was gone by 40, so
  // waiting only 40 landed on the boundary and reported "held" about as often
  // as not. This is deliberately far clear of it; a powered part holds a
  // register indefinitely, so a long wait costs nothing but certainty.
  delay(120);

  uint8_t later = 0;
  if (bus_.readRegister(ACC_CONF, later) != IMUStatus::Ok) {
    return false;    // went silent across an idle gap
  }
  const bool held = (later == probe);

  bus_.writeRegister(ACC_CONF, 0xA8);  // put the default back
  return held;
}

bool BMI270::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  ok &= wakeAndEnable(PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN);

  // Wait for the first sample by polling, not by sleeping. delay(80) here was
  // an eighty millisecond idle gap immediately after enabling the sensors,
  // which is exactly long enough for a part that will not hold its state
  // across an idle bus to lose everything that was just configured.
  wakeAndAwaitData(wakeTimeoutMs_);
  return ok;
}

bool BMI270::wakeAndEnable(uint8_t sensors) {
  // PWR_CONF is 0x7C and PWR_CTRL is 0x7D, so both go out in one burst.
  //
  // Written separately, a part that re-asserts advanced power save can fall
  // asleep in the gap between the two transactions, and the enable then
  // arrives at a sleeping chip and is discarded - PWR_CTRL reads back 0x00
  // having been given 0x0E, and every sample is zero. Adjacent registers in
  // one transaction leave no gap to fall asleep in.
  const uint8_t pair[2] = {0x00, sensors};
  return bus_.writeRegisters(PWR_CONF, pair, sizeof(pair)) == IMUStatus::Ok;
}

void BMI270::trace(const char* what, uint8_t value) {
  if (debug_ == nullptr) return;
  debug_->print(F("  [bmi270] "));
  debug_->print(what);
  debug_->print(F(" 0x"));
  debug_->println(value, HEX);
}

bool BMI270::resumeAfterSleep() {
  // Get it answering again before trying to do anything to it.
  //
  // A part that has gone to sleep does not merely return stale registers, it
  // stops acknowledging altogether: the read that got us here came back as a
  // bus error, and so did the soft-reset command that followed it. Every
  // recovery step was failing on its first transaction.
  //
  // Reading flat out revives it - the traffic itself is what brings it back -
  // so spend a moment doing that before anything that matters.
  bool answering = false;
  for (uint32_t end = millis() + 100; millis() < end; ) {
    if (whoAmI() == kChipId) { answering = true; break; }
  }
  trace("resume: answering again", answering ? 1 : 0);
  if (!answering) return false;

  // How much has to be put back depends on how far the part fell.
  //
  // INTERNAL_STATUS is the test. If it still reads init_ok the core is running
  // and only the sensor registers need re-asserting, which is quick. If it has
  // dropped to not_init the configuration RAM itself is gone - the 8 KB image
  // is not something PWR_CTRL can restore - and nothing short of the whole
  // bring-up will bring it back.
  uint8_t internal = 0;
  const bool coreAlive =
      bus_.readRegister(INTERNAL_STATUS, internal) == IMUStatus::Ok &&
      (internal & 0x0Fu) == INTERNAL_STATUS_INIT_OK;
  trace("resume: INTERNAL_STATUS", internal);

  if (!coreAlive) {
    // Soft reset first. Section 4.4 is explicit that writing
    // INIT_CTRL.init_ctrl = 0x01 "must not be performed more than once after
    // POR or soft reset", so a second upload without a reset in front of it
    // is simply ignored - which is exactly what happened when this was tried
    // the other way round.
    //
    // A full re-bring-up costs around 600 ms, which is far too slow to be
    // doing per sample; but stale or absent data is worse, and a part that
    // holds its configuration never comes down this path at all.
    const bool didReset = reset();
    trace("resume: soft reset ok", didReset ? 1 : 0);
    if (!didReset) return false;
    const bool uploaded = uploadConfiguration();
    trace("resume: upload ok", uploaded ? 1 : 0);
    if (!uploaded) return false;
    // Deliberately not configureDefaults(). That routes through the public
    // setters, which read-modify-write ACC_CONF and GYR_CONF - and a part that
    // has only just been re-initialised can lose its registers between the
    // read and the write, so the whole call reports failure and update()
    // returns nothing even though the core came up perfectly. Write the cached
    // configuration straight out instead, which is what recovering by hand
    // does and what was measured working five times out of five.
  }

  // Both bursts are adjacent-register writes - ACC_CONF through GYR_RANGE at
  // 0x40..0x43, then PWR_CONF and PWR_CTRL at 0x7C..0x7D - so neither can be
  // split by the part falling asleep half way through.
  //
  // The cached values are used rather than reading the registers first,
  // because reading a part that has lost its configuration just returns the
  // reset defaults and would configure it to those.
  const uint8_t conf[4] = {accConf_, accRange_, gyrConf_, gyrRange_};
  trace("resume: writing ACC_CONF", accConf_);
  bus_.writeRegisters(ACC_CONF, conf, sizeof(conf));
  wakeAndEnable(PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN);
  // Deliberately no tracing between here and the caller's read.
  //
  // There was, and it cost an hour: five debug reads with a Serial.println
  // after each is tens of milliseconds of idle bus, so the part went back to
  // sleep between a successful recovery and the sample it was recovered for.
  // The trace reported everything healthy - PWR_CTRL 0x0E, drdy set - and the
  // sample that followed read zero.
  return wakeAndAwaitData(wakeTimeoutMs_);
}

bool BMI270::wakeAndAwaitData(uint16_t timeoutMs) {
  // Enable once, then keep the part awake without touching the enables again.
  //
  // Rewriting PWR_CTRL on every poll restarts the accelerometer's start-up
  // each time round, so drdy never gets the ~46 ms it needs to appear and the
  // loop times out with the sensor perpetually one step from ready. Only
  // PWR_CONF gets refreshed below, which holds off advanced power save without
  // disturbing the sensors.
  //
  // The refresh is necessary at all because an idle bus is what puts this part
  // to sleep in the first place - waiting with delay() would undo the wake-up
  // it is waiting on.
  const uint8_t pair[2] = {0x00, PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN};
  if (bus_.writeRegisters(PWR_CONF, pair, sizeof(pair)) != IMUStatus::Ok) {
    return false;
  }

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
    bus_.writeRegister(PWR_CONF, 0x00);   // keep-alive, enables untouched
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

bool BMI270::readSampleRegisters(uint8_t* a, uint8_t* g, uint8_t* t) {
  return bus_.readRegisters(DATA_ACCEL_X_L, a, 6) == IMUStatus::Ok &&
         bus_.readRegisters(DATA_GYRO_X_L, g, 6) == IMUStatus::Ok &&
         bus_.readRegisters(DATA_TEMP_L, t, 2) == IMUStatus::Ok;
}

bool BMI270::readRaw(RawSample& out) {
  uint8_t a[6];
  uint8_t g[6];
  uint8_t t[2];

  // A part that dozes off between samples fails in two different ways, and
  // both have to be caught here or update() just returns false for ever.
  //
  // It either stops answering altogether - the read comes back as a bus error
  // - or it answers with every data register at exactly zero, which reads as a
  // working sensor lying perfectly still. The temperature separates the second
  // case from a genuinely stationary board: 0x8000 is the BMI270's documented
  // "no reading" value and a running part never reports it.
  //
  // Either way the cure is the same. Wake it, enable the sensors again in the
  // same transaction, and ask once more. On a sensor that stays awake neither
  // branch is ever taken, so this costs nothing.
  // Before trusting a sample, check the part still holds the configuration it
  // was given.
  //
  // The all-zero test below is not enough on its own. A part that has dozed
  // off does not necessarily return zeros - it can return the last values it
  // latched, which look entirely plausible: measured, twenty consecutive
  // reads came back byte-identical at 1.124 g with the gyro at exactly 0.00
  // and the temperature stuck at 10.8 C, on a board that had not moved. That
  // passes every sanity check you would think to write, and it is not data.
  //
  // PWR_CTRL is the honest witness. A running part holds the enables it was
  // given; one that has lost its configuration reads back 0x00. Checking it
  // costs a single register read per sample.
  const uint8_t wanted = PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN;
  uint8_t enables = 0;
  if (bus_.readRegister(PWR_CTRL, enables) != IMUStatus::Ok || enables != wanted) {
    trace("readRaw: PWR_CTRL was", enables);
    if (!resumeAfterSleep()) {
      trace("readRaw: resume failed", 0);
      return false;
    }
  }

  bool got = readSampleRegisters(a, g, t);

  if (!got) {
    wakeAndEnable(PWR_ACC_EN | PWR_GYR_EN | PWR_TEMP_EN);
    got = readSampleRegisters(a, g, t);
    if (!got) return false;
  }

  if (sampleIsDead(a, g, t)) {
    // Wake it and wait for real data rather than reading straight back. A part
    // that has dozed off needs its sensors re-enabled and then time to produce
    // a sample - 46 ms, measured - and the wait has to keep the bus busy or it
    // simply goes back to sleep during it.
    wakeAndAwaitData(wakeTimeoutMs_);
    if (!readSampleRegisters(a, g, t)) return false;

    // Still nothing. Report the failure rather than handing back a sample of
    // all zeros and a temperature of -41 C, which reads like a working sensor
    // lying perfectly still in a freezer.
    if (sampleIsDead(a, g, t)) return false;
  }
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.temp = le16(t);
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

}  // namespace nimu
