#include "LSM6DSOX.h"

namespace nimu {
using namespace lsm6dsox;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool LSM6DSOX::begin() {
  if (beginI2C(Wire, kAddrSA0Low)) {
    return true;
  }
  return beginI2C(Wire, kAddrSA0High);
}

bool LSM6DSOX::beginI2C(TwoWire& wire, uint8_t address) {
  bus_.beginI2C(wire, address, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool LSM6DSOX::beginSPI(SPIClass& spi, uint8_t csPin) {
  bus_.beginSPI(spi, csPin, 1000000, 0xC0);
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

uint8_t LSM6DSOX::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

bool LSM6DSOX::isConnected() {
  return whoAmI() == kWhoAmI;
}

bool LSM6DSOX::reset() {
  if (bus_.writeRegister(CTRL3_C, CTRL3_SW_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(20);
  return bus_.writeRegister(CTRL3_C, CTRL3_BDU | CTRL3_IF_INC) == IMUStatus::Ok;
}

bool LSM6DSOX::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setLowPassFilterHz(50);
  delay(35);
  return ok;
}

bool LSM6DSOX::readRaw(RawSample& out) {
  uint8_t t[2];
  uint8_t g[6];
  uint8_t a[6];
  if (bus_.readRegisters(OUT_TEMP_L, t, sizeof(t)) != IMUStatus::Ok ||
      bus_.readRegisters(OUTX_L_G, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(OUTX_L_XL, a, sizeof(a)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = le16(t);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  return true;
}

bool LSM6DSOX::update() {
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
  data_.temperature = 25.0f + (raw.temp / 256.0f);
  data_.timestamp = micros();
  return true;
}

bool LSM6DSOX::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = XL_FS_2; accelLsbPerG_ = 16384.0f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = XL_FS_4; accelLsbPerG_ = 8192.0f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = XL_FS_8; accelLsbPerG_ = 4096.0f; accelRangeG_ = 8;
  } else {
    fs = XL_FS_16; accelLsbPerG_ = 2048.0f; accelRangeG_ = 16;
  }
  return bus_.updateRegister(CTRL1_XL, XL_FS_MASK, fs) == IMUStatus::Ok;
}

bool LSM6DSOX::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 125) {
    fs = G_FS_125; gyroLsbPerDps_ = 262.144f; gyroRangeDps_ = 125;
  } else if (maxDps <= 250) {
    fs = G_FS_250; gyroLsbPerDps_ = 131.072f; gyroRangeDps_ = 250;
  } else if (maxDps <= 500) {
    fs = G_FS_500; gyroLsbPerDps_ = 65.536f; gyroRangeDps_ = 500;
  } else if (maxDps <= 1000) {
    fs = G_FS_1000; gyroLsbPerDps_ = 32.768f; gyroRangeDps_ = 1000;
  } else {
    fs = G_FS_2000; gyroLsbPerDps_ = 16.384f; gyroRangeDps_ = 2000;
  }
  return bus_.updateRegister(CTRL2_G, G_FS_MASK, fs) == IMUStatus::Ok;
}

uint8_t LSM6DSOX::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 13) { actualHz = 12; return ODR_12_5; }
  if (hz <= 26) { actualHz = 26; return ODR_26; }
  if (hz <= 52) { actualHz = 52; return ODR_52; }
  if (hz <= 104) { actualHz = 104; return ODR_104; }
  if (hz <= 208) { actualHz = 208; return ODR_208; }
  if (hz <= 416) { actualHz = 416; return ODR_416; }
  if (hz <= 833) { actualHz = 833; return ODR_833; }
  if (hz <= 1660) { actualHz = 1660; return ODR_1660; }
  if (hz <= 3330) { actualHz = 3330; return ODR_3330; }
  actualHz = 6660;
  return ODR_6660;
}

bool LSM6DSOX::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  odrBits_ = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;
  bool ok = true;
  ok &= bus_.updateRegister(CTRL1_XL, 0xF0, odrBits_) == IMUStatus::Ok;
  ok &= bus_.updateRegister(CTRL2_G, 0xF0, odrBits_) == IMUStatus::Ok;
  return ok;
}

bool LSM6DSOX::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

bool LSM6DSOX::dataReady() {
  uint8_t status = 0;
  if (bus_.readRegister(STATUS_REG, status) != IMUStatus::Ok) {
    return false;
  }
  return (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool LSM6DSOX::configureInterruptPins(bool activeLow, bool openDrain) {
  uint8_t value = (activeLow ? CTRL3_H_LACTIVE : 0) |
                  (openDrain ? CTRL3_PP_OD : 0);
  return bus_.updateRegister(CTRL3_C, CTRL3_H_LACTIVE | CTRL3_PP_OD,
                             value) == IMUStatus::Ok;
}

bool LSM6DSOX::routeInterrupt(uint8_t pin, uint8_t sources) {
  uint8_t reg = pin == 1 ? INT1_CTRL : pin == 2 ? INT2_CTRL : 0;
  return reg != 0 && bus_.writeRegister(reg, sources) == IMUStatus::Ok;
}

bool LSM6DSOX::setDataReadyInterrupt(uint8_t pin, bool accel, bool gyro) {
  uint8_t sources = (accel ? INT_DRDY_XL : 0) | (gyro ? INT_DRDY_G : 0);
  return routeInterrupt(pin, sources);
}

bool LSM6DSOX::readMlcOutput(uint8_t index, uint8_t& value) {
  if (index >= 8) return false;
  return bus_.readRegister(static_cast<uint8_t>(MLC0_SRC + index), value) ==
         IMUStatus::Ok;
}

void LSM6DSOX::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
}

}  // namespace nimu
