#include "LSM303DLHC.h"

namespace nimu {
using namespace lsm303dlhc;

namespace {
inline int16_t le12Left(const uint8_t* p) {
  int16_t v = static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                   (static_cast<uint16_t>(p[1]) << 8));
  return static_cast<int16_t>(v >> 4);
}

inline int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

bool LSM303DLHC::begin() {
  if (beginI2C(Wire, kAccelAddrHigh, kMagAddr)) {
    return true;
  }
  return beginI2C(Wire, kAccelAddrLow, kMagAddr);
}

bool LSM303DLHC::beginI2C(TwoWire& wire, uint8_t address) {
  return beginI2C(wire, address, kMagAddr);
}

bool LSM303DLHC::beginI2C(TwoWire& wire, uint8_t accelAddress,
                          uint8_t magAddress) {
  bus_.beginI2C(wire, accelAddress, clockHz_);
  magBus_.beginI2C(wire, magAddress, clockHz_);
  bus_.recoverBus();
  if (!isConnected()) {
    return false;
  }
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (magBus_.readRegister(IRA_REG_M, a) != IMUStatus::Ok ||
      magBus_.readRegister(IRB_REG_M, b) != IMUStatus::Ok ||
      magBus_.readRegister(IRC_REG_M, c) != IMUStatus::Ok) {
    return false;
  }
  if (a != 'H' || b != '4' || c != '3') {
    return false;
  }
  return configureDefaults();
}

bool LSM303DLHC::beginSPI(SPIClass& spi, uint8_t csPin) {
  (void)spi;
  (void)csPin;
  return false;
}

uint8_t LSM303DLHC::whoAmI() {
  uint8_t id = 0;
  bus_.readRegister(WHO_AM_I_A, id);
  return id;
}

uint8_t LSM303DLHC::magWhoAmI() {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (magBus_.readRegister(IRA_REG_M, a) != IMUStatus::Ok ||
      magBus_.readRegister(IRB_REG_M, b) != IMUStatus::Ok ||
      magBus_.readRegister(IRC_REG_M, c) != IMUStatus::Ok) {
    return 0;
  }
  return (a == 'H' && b == '4' && c == '3') ? a : 0;
}

bool LSM303DLHC::isConnected() {
  return whoAmI() == kAccelWhoAmI;
}

bool LSM303DLHC::configureDefaults() {
  bool ok = true;
  ok &= setAccelRangeG(accelRangeG_);
  ok &= setSampleRateHz(sampleRateHz_);
  ok &= setMagRangeGauss(1.3f);
  ok &= magBus_.writeRegister(CRA_REG_M, 0x14) == IMUStatus::Ok;  // 15 Hz mag
  ok &= magBus_.writeRegister(MR_REG_M, MR_CONTINUOUS) == IMUStatus::Ok;
  return ok;
}

bool LSM303DLHC::readRaw(RawSample& out) {
  uint8_t a[6];
  uint8_t m[6];
  uint8_t t[2];
  if (bus_.readRegisters(OUT_X_L_A | 0x80, a, sizeof(a)) != IMUStatus::Ok ||
      magBus_.readRegisters(OUT_X_H_M, m, sizeof(m)) != IMUStatus::Ok) {
    return false;
  }
  out.ax = le12Left(&a[0]);
  out.ay = le12Left(&a[2]);
  out.az = le12Left(&a[4]);
  out.mx = be16(&m[0]);
  out.mz = be16(&m[2]);
  out.my = be16(&m[4]);
  if (magBus_.readRegisters(TEMP_OUT_H_M, t, sizeof(t)) == IMUStatus::Ok) {
    out.temp = be16(t);
  } else {
    out.temp = 0;
  }
  return true;
}

bool LSM303DLHC::update() {
  RawSample raw;
  if (!readRaw(raw)) {
    return false;
  }

  Vec3 a{raw.ax * accelSensitivityG_, raw.ay * accelSensitivityG_,
         raw.az * accelSensitivityG_};
  data_.accel = correct(a, cal_.accelBias, cal_.accelScale);
  data_.gyro = Vec3{};

  Vec3 m{raw.mx / magLsbPerGaussXY_ * 100.0f,
         raw.my / magLsbPerGaussXY_ * 100.0f,
         raw.mz / magLsbPerGaussZ_ * 100.0f};
  data_.mag = correct(m, cal_.magBias, cal_.magScale);
  data_.temperature = 25.0f + (raw.temp / 8.0f);
  data_.timestamp = micros();
  return true;
}

bool LSM303DLHC::setAccelRangeG(uint16_t maxG) {
  uint8_t fs;
  if (maxG <= 2) {
    fs = 0x00; accelSensitivityG_ = 0.001f; accelRangeG_ = 2;
  } else if (maxG <= 4) {
    fs = 0x10; accelSensitivityG_ = 0.002f; accelRangeG_ = 4;
  } else if (maxG <= 8) {
    fs = 0x20; accelSensitivityG_ = 0.004f; accelRangeG_ = 8;
  } else {
    fs = 0x30; accelSensitivityG_ = 0.012f; accelRangeG_ = 16;
  }
  return bus_.updateRegister(CTRL_REG4_A, CTRL4_FS_MASK | CTRL4_BDU | CTRL4_HR,
                             fs | CTRL4_BDU | CTRL4_HR) == IMUStatus::Ok;
}

bool LSM303DLHC::setGyroRangeDps(uint16_t maxDps) {
  (void)maxDps;
  return true;
}

uint8_t LSM303DLHC::odrBitsForHz(uint16_t hz, uint16_t& actualHz) const {
  if (hz <= 1) { actualHz = 1; return 0x10; }
  if (hz <= 10) { actualHz = 10; return 0x20; }
  if (hz <= 25) { actualHz = 25; return 0x30; }
  if (hz <= 50) { actualHz = 50; return 0x40; }
  if (hz <= 100) { actualHz = 100; return 0x50; }
  if (hz <= 200) { actualHz = 200; return 0x60; }
  actualHz = 400;
  return 0x70;
}

bool LSM303DLHC::setSampleRateHz(uint16_t hz) {
  if (hz == 0) {
    return false;
  }
  uint16_t actual = 0;
  uint8_t odr = odrBitsForHz(hz, actual);
  sampleRateHz_ = actual;
  return bus_.writeRegister(CTRL_REG1_A, odr | CTRL1_AXES_ENABLE) ==
         IMUStatus::Ok;
}

bool LSM303DLHC::setLowPassFilterHz(uint16_t hz) {
  return setSampleRateHz(hz * 2);
}

bool LSM303DLHC::setMagRangeGauss(float gauss) {
  uint8_t gain = 0x20;
  if (gauss <= 1.3f) {
    gain = 0x20; magLsbPerGaussXY_ = 1100.0f; magLsbPerGaussZ_ = 980.0f;
  } else if (gauss <= 1.9f) {
    gain = 0x40; magLsbPerGaussXY_ = 855.0f; magLsbPerGaussZ_ = 760.0f;
  } else if (gauss <= 2.5f) {
    gain = 0x60; magLsbPerGaussXY_ = 670.0f; magLsbPerGaussZ_ = 600.0f;
  } else if (gauss <= 4.0f) {
    gain = 0x80; magLsbPerGaussXY_ = 450.0f; magLsbPerGaussZ_ = 400.0f;
  } else if (gauss <= 4.7f) {
    gain = 0xA0; magLsbPerGaussXY_ = 400.0f; magLsbPerGaussZ_ = 355.0f;
  } else if (gauss <= 5.6f) {
    gain = 0xC0; magLsbPerGaussXY_ = 330.0f; magLsbPerGaussZ_ = 295.0f;
  } else {
    gain = 0xE0; magLsbPerGaussXY_ = 230.0f; magLsbPerGaussZ_ = 205.0f;
  }
  return magBus_.writeRegister(CRB_REG_M, gain) == IMUStatus::Ok;
}

bool LSM303DLHC::dataReady() {
  uint8_t accelStatus = 0;
  uint8_t magStatus = 0;
  if (bus_.readRegister(STATUS_REG_A, accelStatus) != IMUStatus::Ok ||
      magBus_.readRegister(SR_REG_M, magStatus) != IMUStatus::Ok) {
    return false;
  }
  return (accelStatus & STATUS_ZYXDA) && (magStatus & SR_DRDY);
}

bool LSM303DLHC::magDataReady() {
  uint8_t status = 0;
  return magBus_.readRegister(SR_REG_M, status) == IMUStatus::Ok &&
         (status & SR_DRDY) != 0;
}

bool LSM303DLHC::setAccelDataReadyInterrupt(bool enable) {
  return bus_.updateRegister(CTRL_REG3_A, I1_DATA_READY,
                             enable ? I1_DATA_READY : 0) == IMUStatus::Ok;
}

bool LSM303DLHC::configureAccelInterruptPolarity(bool activeLow) {
  return bus_.updateRegister(CTRL_REG6_A, INT_ACTIVE_LOW,
                             activeLow ? INT_ACTIVE_LOW : 0) == IMUStatus::Ok;
}

void LSM303DLHC::setBusClockHz(uint32_t hz) {
  clockHz_ = hz;
  bus_.setClockHz(hz);
  magBus_.setClockHz(hz);
}

}  // namespace nimu
