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
  if (beginI2C(Wire, kAddrSDOLow)) {
    return true;
  }
  return beginI2C(Wire, kAddrSDOHigh);
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
    case Stage::ConfigFileStub:  return "the bundled configuration image is a "
                                        "placeholder, not Bosch's 8192-byte file - "
                                        "the part cannot start without it";
    case Stage::ConfigUpload:    return "the configuration blob would not transfer";
    case Stage::ConfigNotLoaded: return "blob sent, but the chip never reported it loaded";
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
  return whoAmI() == kChipId;
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
  const uint32_t deadline = millis() + 300;
  while (millis() < deadline) {
    delay(5);
    if (isConnected()) return true;
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
  // The BMI270 has no usable function until an 8192-byte configuration image
  // from Bosch has been loaded into it. kConfigFile is currently a few hundred
  // bytes - a placeholder, not that image.
  //
  // Uploading it succeeds at the bus level, every write is acknowledged, and
  // then INTERNAL_STATUS never reports init_ok because the chip was handed
  // something that is not firmware. That failure is indistinguishable from a
  // wiring fault unless you happen to count the bytes, so say it plainly here
  // rather than letting the caller conclude their sensor is broken.
  //
  // Fixing this means vendoring bmi270_config_file[] from the Bosch Sensortec
  // BMI270 API (BSD-3-Clause) into BMI270_Config.h.
  if (sizeof(kConfigFile) < 8192u) {
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
  for (uint16_t i = 0; i < sizeof(kConfigFile); i += kChunk) {
    uint16_t len = sizeof(kConfigFile) - i;
    if (len > kChunk) {
      len = kChunk;
    }
    if (!writeConfigChunk(i, &kConfigFile[i], len)) {
      return false;
    }
  }

  if (bus_.writeRegister(INIT_CTRL, 0x01) != IMUStatus::Ok) {
    return false;
  }
  delay(150);
  if (!configurationLoaded()) {
    lastStage_ = Stage::ConfigNotLoaded;
    return false;
  }
  return true;
}

bool BMI270::configurationLoaded() {
  uint8_t status = 0;
  if (bus_.readRegister(INTERNAL_STATUS, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & INTERNAL_STATUS_INIT_OK) != 0;
}

bool BMI270::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  ok &= bus_.writeRegister(PWR_CTRL, PWR_ACC_EN | PWR_GYR_EN) == IMUStatus::Ok;
  delay(80);
  return ok;
}

bool BMI270::readRaw(RawSample& out) {
  uint8_t a[6];
  uint8_t g[6];
  uint8_t t[2];
  if (bus_.readRegisters(DATA_ACCEL_X_L, a, sizeof(a)) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_GYRO_X_L, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(DATA_TEMP_L, t, sizeof(t)) != IMUStatus::Ok) {
    return false;
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
  bool ok = true;
  ok &= bus_.updateRegister(ACC_CONF, 0x0F, accelOdr_) == IMUStatus::Ok;
  ok &= bus_.updateRegister(GYR_CONF, 0x0F, gyroOdr_) == IMUStatus::Ok;
  return ok;
}

bool BMI270::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  bool ok = true;
  ok &= bus_.updateRegister(ACC_CONF, PERF_MODE, PERF_MODE) == IMUStatus::Ok;
  ok &= bus_.updateRegister(GYR_CONF, PERF_MODE, PERF_MODE) == IMUStatus::Ok;
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
  uint8_t mask = pin == 1 ? 0x04 : 0x40;
  return bus_.updateRegister(INT_MAP_DATA, mask, enable ? mask : 0) ==
         IMUStatus::Ok;
}

void BMI270::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
