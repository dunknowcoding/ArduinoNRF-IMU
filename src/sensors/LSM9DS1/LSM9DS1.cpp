#include "LSM9DS1.h"

namespace nimu {
using namespace lsm9ds1;

namespace {
inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

bool LSM9DS1::begin() {
  if (beginI2C(Wire, kAddrAGHigh, kAddrMagHigh)) {
    return true;
  }
  if (beginI2C(Wire, kAddrAGLow, kAddrMagLow)) {
    return true;
  }
  if (beginI2C(Wire, kAddrAGHigh, kAddrMagLow)) {
    return true;
  }
  return beginI2C(Wire, kAddrAGLow, kAddrMagHigh);
}

bool LSM9DS1::beginI2C(TwoWire& wire, uint8_t address) {
  return beginI2C(wire, address, (address == kAddrAGLow) ? kAddrMagLow
                                                         : kAddrMagHigh);
}

bool LSM9DS1::beginI2C(TwoWire& wire, uint8_t agAddress, uint8_t magAddress) {
  bus_.beginI2C(wire, agAddress, clockHz_);
  magBus_.beginI2C(wire, magAddress, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  return reset() && configureDefaults();
}

bool LSM9DS1::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;  // LSM9DS1 SPI needs separate accel/gyro and mag chip selects.
}

uint8_t LSM9DS1::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I, id);
  return id;
}

uint8_t LSM9DS1::magWhoAmI() {
  uint8_t id = 0;
  magBus_.readRegister(WHO_AM_I, id);
  return id;
}

bool LSM9DS1::isConnected() {
  return whoAmI() == kWhoAmIAG && magWhoAmI() == kWhoAmIMag;
}

bool LSM9DS1::reset() {
  if (bus_.writeRegister(CTRL_REG8, CTRL8_SW_RESET) != IMUStatus::Ok) {
    return false;
  }
  delay(20);
  return bus_.writeRegister(CTRL_REG8, CTRL8_BDU | CTRL8_IF_ADD_INC) ==
         IMUStatus::Ok;
}

bool LSM9DS1::dataReady() {
  uint8_t status = 0;
  return bus_.readRegister(STATUS_REG, status) == IMUStatus::Ok &&
         (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool LSM9DS1::magDataReady() {
  uint8_t status = 0;
  return magBus_.readRegister(STATUS_REG_M, status) == IMUStatus::Ok &&
         (status & STATUS_M_ZYXDA) != 0;
}

bool LSM9DS1::routeDataReadyInterrupt(uint8_t pin, bool accel, bool gyro) {
  uint8_t reg = pin == 1 ? INT1_CTRL : pin == 2 ? INT2_CTRL : 0;
  if (reg == 0) return false;
  uint8_t sources = (accel ? INT_DRDY_XL : 0) | (gyro ? INT_DRDY_G : 0);
  return bus_.updateRegister(reg, INT_DRDY_XL | INT_DRDY_G, sources) ==
         IMUStatus::Ok;
}

bool LSM9DS1::configureMagInterrupt(uint8_t axesMask, uint16_t threshold,
                                    bool activeHigh, bool latched,
                                    bool enable) {
  uint8_t thresholdBytes[2] = {static_cast<uint8_t>(threshold),
                               static_cast<uint8_t>(threshold >> 8)};
  if (magBus_.writeRegisters(INT_THS_L_M, thresholdBytes, 2) != IMUStatus::Ok)
    return false;
  uint8_t value = static_cast<uint8_t>((axesMask & 0x07) << 5);
  if (activeHigh) value |= 0x04;
  if (latched) value |= 0x02;
  if (enable) value |= 0x01;
  return magBus_.writeRegister(INT_CFG_M, value) == IMUStatus::Ok;
}

uint8_t LSM9DS1::magInterruptSource() {
  uint8_t source = 0;
  magBus_.readRegister(INT_SRC_M, source);
  return source;
}

bool LSM9DS1::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setGyroRangeDps(gyroRangeDps_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setMagRangeGauss(4);
  // Mag: temp compensation, high-performance XY, 80 Hz, continuous conversion.
  ok &= magBus_.writeRegister(CTRL_REG1_M, 0xFC) == IMUStatus::Ok;
  ok &= magBus_.writeRegister(CTRL_REG3_M, 0x00) == IMUStatus::Ok;
  ok &= magBus_.writeRegister(CTRL_REG4_M, 0x0C) == IMUStatus::Ok;
  return ok;
}

bool LSM9DS1::readRaw(RawSample& out) {
  uint8_t t[2];
  uint8_t g[6];
  uint8_t a[6];
  uint8_t m[6];
  if (bus_.readRegisters(OUT_TEMP_L, t, sizeof(t)) != IMUStatus::Ok ||
      bus_.readRegisters(OUT_X_L_G, g, sizeof(g)) != IMUStatus::Ok ||
      bus_.readRegisters(OUT_X_L_XL, a, sizeof(a)) != IMUStatus::Ok ||
      magBus_.readRegisters(OUT_X_L_M, m, sizeof(m)) != IMUStatus::Ok) {
    return false;
  }
  out.temp = le16(t);
  out.gx = le16(&g[0]);
  out.gy = le16(&g[2]);
  out.gz = le16(&g[4]);
  out.ax = le16(&a[0]);
  out.ay = le16(&a[2]);
  out.az = le16(&a[4]);
  out.mx = le16(&m[0]);
  out.my = le16(&m[2]);
  out.mz = le16(&m[4]);
  return true;
}

bool LSM9DS1::update() {
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

  Vec3 m{raw.mx / magLsbPerUT_, raw.my / magLsbPerUT_, raw.mz / magLsbPerUT_};
  data_.mag = correct(m, cal_.magBias, cal_.magScale);

  data_.temperature = 25.0f + (raw.temp / 16.0f);
  data_.timestamp = micros();
  return true;
}

bool LSM9DS1::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = 0x00; accelLsbPerG_ = 16393.4426f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = 0x10; accelLsbPerG_ = 8196.7213f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = 0x18; accelLsbPerG_ = 4098.3607f; accelRangeG_ = 8;
  } else {
    fs = 0x08; accelLsbPerG_ = 1366.1202f; accelRangeG_ = 16;
  }
  return bus_.updateRegister(CTRL_REG6_XL, 0x18, fs) == IMUStatus::Ok;
}

bool LSM9DS1::setGyroRangeDps(uint16_t maxDps) {
  uint8_t fs;
  if (maxDps <= 245) {
    fs = 0x00; gyroLsbPerDps_ = 114.2857f; gyroRangeDps_ = 245;
  } else if (maxDps <= 500) {
    fs = 0x08; gyroLsbPerDps_ = 57.1429f; gyroRangeDps_ = 500;
  } else {
    fs = 0x18; gyroLsbPerDps_ = 14.2857f; gyroRangeDps_ = 2000;
  }
  return bus_.updateRegister(CTRL_REG1_G, 0x18, fs) == IMUStatus::Ok;
}

uint8_t LSM9DS1::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 15) { actualHz = 14; return 0x20; }
  if (hz <= 59) { actualHz = 59; return 0x40; }
  if (hz <= 119) { actualHz = 119; return 0x60; }
  if (hz <= 238) { actualHz = 238; return 0x80; }
  if (hz <= 476) { actualHz = 476; return 0xA0; }
  actualHz = 952;
  return 0xC0;
}

bool LSM9DS1::setSampleRateHz(uint16_t hz) {
  if (hz == 0) return false;
  uint16_t actual = 0;
  odrBits_ = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;
  bool ok = true;
  ok &= bus_.updateRegister(CTRL_REG1_G, 0xE0, odrBits_) == IMUStatus::Ok;
  ok &= bus_.updateRegister(CTRL_REG6_XL, 0xE0, odrBits_) == IMUStatus::Ok;
  return ok;
}

bool LSM9DS1::setMagRangeGauss(uint16_t gauss) {
  uint8_t fs;
  if (gauss <= 4) {
    fs = 0x00; magLsbPerUT_ = 1.0f / 0.014f;
  } else if (gauss <= 8) {
    fs = 0x20; magLsbPerUT_ = 1.0f / 0.029f;
  } else if (gauss <= 12) {
    fs = 0x40; magLsbPerUT_ = 1.0f / 0.043f;
  } else {
    fs = 0x60; magLsbPerUT_ = 1.0f / 0.058f;
  }
  return magBus_.updateRegister(CTRL_REG2_M, 0x60, fs) == IMUStatus::Ok;
}

bool LSM9DS1::setLowPassFilterHz(uint16_t hz) {
  (void)hz;
  return true;
}

}  // namespace nimu
